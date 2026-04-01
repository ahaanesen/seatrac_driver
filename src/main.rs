use std::sync::{Arc, Mutex};
use std::{thread::{self, sleep}, time::Duration};

use serde_yaml::{self};

mod modem_driver;
mod seatrac;
mod comms;
mod parameters;

use crate::modem_driver::ModemAbstraction;
use crate::comms::{ack_manager::{AcknowledgmentManager, SentMessage}, 
                    message_types::{self, NewMsg}, 
                    protocols, 
                    ros2_node, 
                    tdma_utils};

static DEFAULT_BAUD_RATE: u32 = 115200;



#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Load configurations
    let modem_config_file = std::fs::File::open("config/modem.yaml").expect("Could not open file.");
    let modem_params_cfg: parameters::ModemParameters =
        serde_yaml::from_reader(modem_config_file).expect("Could not parse file.");

    let comms_config_file = std::fs::File::open("config/comms.yaml").expect("Could not open file.");
    let comms_params_cfg: parameters::CommsParameters =
        serde_yaml::from_reader(comms_config_file).expect("Could not parse file.");

    let ros2_config_file = std::fs::File::open("config/ros2_parameters.yaml").expect("Could not open file.");
    let ros2_params_cfg: parameters::ROS2Parameters =
        serde_yaml::from_reader(ros2_config_file).expect("Could not parse file.");

    // Modem init
    let mut modem;
    match modem_params_cfg.modem_type.as_str() {
        "seatrac" => {
            modem = seatrac::seatrac_driver::SeatracModem::new(
                &modem_params_cfg.port_name,
                modem_params_cfg.baud_rate,
                modem_params_cfg.usbl,
                modem_params_cfg.beacon_id,
            )
            .unwrap_or_else(|e| {
                panic!("Failed to open modem: {}", e);
            });
        }
        // Can add other modem types here, e.g. "luma100"
        // NB. might want to switch from modem_type being string to enum, but not sure how well it works in the config file
        _ => panic!("Unsupported modem type: {}", modem_params_cfg.modem_type),
    }
    modem.configure(modem_params_cfg.usbl, DEFAULT_BAUD_RATE, modem_params_cfg.beacon_id, modem_params_cfg.salinity).unwrap();


    // ROS2 init
    let node_name = "seatrac_".to_string() + &comms_params_cfg.agent_id.to_string(); //most important the comms and ros2 node have the same id
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, &node_name, "")?;

    let acoustic_send_subscriber = node
        .subscribe::<r2r::blueboat_interfaces::msg::AcousticCommSend>(
            &ros2_params_cfg.acoustic_comm_send_topic_name, r2r::QosProfile::default()
    )?;

    let acoustic_receive_publisher = node
        .create_publisher::<r2r::blueboat_interfaces::msg::AcousticCommReceive>(
            &ros2_params_cfg.acoustic_comm_receive_topic_name, r2r::QosProfile::default()
    )?;

    if modem.is_usbl() {
        let usbl_publisher = node
            .create_publisher::<r2r::blueboat_interfaces::msg::USBLMeasurement>(
                &ros2_params_cfg.usbl_topic_name, r2r::QosProfile::default()
        )?;
    }

    let arc_node = Arc::new(Mutex::new(node));
    let bridge_node = ros2_node::RosBridge::new(&arc_node, &node_name)?;
    let queues = bridge_node.queues.clone();


    // TDMA scheduler init
    let tdma_scheduler = comms::tdma_scheduler::TdmaScheduler::new(
        comms_params_cfg.network_participant_count,
        comms_params_cfg.tdma_slot_duration_s,
        comms_params_cfg.agent_id,
    );


    // Clock init
    // let initial_time = fs::read_to_string("/home/khadas/ros2_ws/start_time.txt")
    //     .expect("Unable to read file")
    //     .trim()
    //     .parse::<f64>()
    //     .expect("Unable to parse start_time");
    //let initial_time: f64 = 0.0; // Hardcoded initial time for testing
    let initial_time = tdma_utils::get_total_seconds() as f64;
    // TODO: clock synchronization


    // Start TDMA scheduler thread
    thread::spawn(move || {
        let mut ack_handler = AcknowledgmentManager::new();
        println!("Starting TDMA scheduler for node ID {}", comms_params_cfg.agent_id);
        loop {

            if tdma_scheduler.is_my_slot() {
                println!("\n \n Node {}: It's my slot to send! ", comms_params_cfg.agent_id);
                let acks_to_send = ack_handler.prepare_round();
                protocols::broadcast_status_msg(&comms_params_cfg, &mut modem, initial_time as u64, &mut ack_handler, &acks_to_send);

                while tdma_scheduler.is_my_slot() {
                    // Send queued messages from ROS
                    let mut send_queue = queues.to_modem.lock().unwrap();

                    for msg in send_queue.drain(..) { // NB: the ros2 messages need the format: "destination_id:{} x:{} y:{} z:{}"
                        sleep(Duration::from_millis(ack_handler.wait_time));

                        println!("Message from ROS2 to send: {}", msg);
                        if tdma_utils::get_slot_after_propag(&comms_params_cfg, ack_handler.wait_time) != tdma_scheduler.assigned_slot {
                            println!("Not enough time to send new message, skipping.");
                            break;
                        }
                        let (destination_id, position) = message_types::parse_new_msg(&msg).expect("Failed to parse new message from ROS2");
                        println!("Parsed destination ID: {}, position: {:?}", destination_id, position);

                        let new_msg = NewMsg::new(position, tdma_utils::get_total_seconds());
                        ack_handler.message_index += 1;
                        ack_handler.add_message(ack_handler.message_index, SentMessage::new(new_msg.position, new_msg.t));
                        println!("Prepared new message with index {}", ack_handler.message_index);

                        ack_handler.wait_time =protocols::send_message(&comms_params_cfg, &mut modem, ack_handler.message_index, destination_id, &new_msg, &acks_to_send);
                        println!("Set wait time to {} milliseconds", ack_handler.wait_time);
                    }

                    // Resend logic
                    let next_slot = tdma_utils::get_slot_after_propag(&comms_params_cfg, ack_handler.wait_time);
                    if  !ack_handler.has_unacked_messages() && next_slot == tdma_scheduler.assigned_slot {
                        protocols::resend_messages(&comms_params_cfg, &mut modem, &mut ack_handler, &acks_to_send);
                    }

                }

            } else { // Receiving slot
                println!("Node {}: Listening for messages...", comms_params_cfg.agent_id);

                while !tdma_scheduler.is_my_slot() {
                    match protocols::receive_message(&mut modem) { 
                        Ok((received_msg, usbl_data)) => {
                            
                            if let Some(received_msg) = received_msg {
                                queues.from_modem.lock().unwrap().push(received_msg.to_string()); // To ROS2 network

                                ack_handler.listened_msg.push(received_msg.clone()); // Store for ack handling later
                            }

                            // If usbl data is present, process it
                            if let Some(usbl_data) = usbl_data {
                                println!("Received USBL data: {:?}", usbl_data);
                                // TODO: fix this, want another ros2 topic to_ekf
                                queues.from_modem.lock().unwrap().push(format!("{:?}", usbl_data)); // To ROS2 network
                            }

                        }
                        Err(_e) => {
                            //eprintln!("Error receiving message: {}", e);
                            continue;
                        }
                    }
                }
                for msg in &ack_handler.listened_msg.clone() {
                    ack_handler.handle_acknowledgments(&msg, comms_params_cfg.network_participant_count);
                }

            }

            // publish modem -> ROS
            if let Err(e) = bridge_node.publish_from_queue() {
                eprintln!("Publish error: {:?}", e);
            }

            std::thread::sleep(Duration::from_millis(10));
        }
    });
    

    // Spin the node in a blocking task (same pattern as kf_node.rs)
    let handle = tokio::task::spawn_blocking(move || loop {
        {
            arc_node
                .lock()
                .unwrap()
                .spin_once(std::time::Duration::from_micros(5));
        }
        std::thread::sleep(std::time::Duration::from_micros(1));
    });

    handle.await?;
    Ok(())

}