use core::task;
use std::sync::{Arc, Mutex};
use std::{env, thread::{self, sleep}, time::Duration};

mod modem_driver;
mod seatrac;
mod comms;
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
    let mut driver_config = modem_driver::ModemConfig::load_from_file("config_modem.json").unwrap_or_else(|e| {
        panic!("Failed to load driver configuration: {}", e);
    });

    let mut comms_config = comms::tdma_utils::CommunicationConfig::load_from_file("config_comms.json").unwrap_or_else(|e| {
        panic!("Failed to load communication configuration: {}", e);
    });


    // Parse command-line arguments to set node id and optional usbl flag
/*     let args: Vec<String> = env::args().collect();
    let agent_id = if args.len() > 1 {
        args[1].parse::<u8>().unwrap_or_else(|_| {
            panic!("Invalid node ID argument. Please provide a valid u8 value.");
        })
    } else {
        panic!("Node ID argument is required.");
    };
    let usbl = args.iter().any(|arg| arg == "usbl"); // Check for optional "usbl" argument

    // Overwrite loaded configs with command-line args
    let port_name = format!("/dev/ttyUSB{}", agent_id);
    driver_config.port_name = port_name;
    driver_config.beacon_id = agent_id;
    driver_config.usbl = usbl || false;

    comms_config.agent_id = agent_id; */


    // Modem init
    let mut modem;
    match driver_config.modem_type.as_str() {
        "seatrac" => {
            modem = seatrac::seatrac_driver::SeatracModem::new(
                &driver_config.port_name,
                driver_config.baud_rate,
                driver_config.usbl,
                driver_config.beacon_id,
            )
            .unwrap_or_else(|e| {
                panic!("Failed to open modem: {}", e);
            });
        }
        // Can add other modem types here, e.g. "luma100"
        // NB. might want to switch from modem_type being string to enum, but not sure how well it works in the config file
        _ => panic!("Unsupported modem type: {}", driver_config.modem_type),
    }
    modem.configure(driver_config.usbl, DEFAULT_BAUD_RATE, driver_config.beacon_id, driver_config.salinity).unwrap();


    // ROS2 init
    let node_name = "seatrac_".to_string() + &comms_config.agent_id.to_string(); //most important the comms and ros2 node have the same id
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, &node_name, "")?;
    let arc_node = Arc::new(Mutex::new(node));

    let bridge_node = ros2_node::RosBridge::new(&arc_node, &node_name)?;
    let queues = bridge_node.queues.clone();


    // TDMA scheduler init
    let tdma_scheduler = comms::tdma_scheduler::TdmaScheduler::new(
        comms_config.network_participant_count,
        comms_config.tdma_slot_duration_s,
        comms_config.agent_id,
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
        println!("Starting TDMA scheduler for node ID {}", comms_config.agent_id);
        loop {

            if tdma_scheduler.is_my_slot() {
                println!("\n \n Node {}: It's my slot to send! ", comms_config.agent_id);
                let acks_to_send = ack_handler.prepare_round();
                protocols::broadcast_status_msg(&comms_config, &mut modem, initial_time as u64, &mut ack_handler, &acks_to_send);

                while tdma_scheduler.is_my_slot() {
                    // Send queued messages from ROS
                    let mut send_queue = queues.to_modem.lock().unwrap();

                    for msg in send_queue.drain(..) { // NB: the ros2 messages need the format: "destination_id:{} x:{} y:{} z:{}"
                        sleep(Duration::from_millis(ack_handler.wait_time));

                        println!("Message from ROS2 to send: {}", msg);
                        if tdma_utils::get_slot_after_propag(&comms_config, ack_handler.wait_time) != tdma_scheduler.assigned_slot {
                            println!("Not enough time to send new message, skipping.");
                            break;
                        }
                        let (destination_id, position) = message_types::parse_new_msg(&msg).expect("Failed to parse new message from ROS2");
                        println!("Parsed destination ID: {}, position: {:?}", destination_id, position);

                        let new_msg = NewMsg::new(position, tdma_utils::get_total_seconds());
                        ack_handler.message_index += 1;
                        ack_handler.add_message(ack_handler.message_index, SentMessage::new(new_msg.position, new_msg.t));
                        println!("Prepared new message with index {}", ack_handler.message_index);

                        ack_handler.wait_time =protocols::send_message(&comms_config, &mut modem, ack_handler.message_index, destination_id, &new_msg, &acks_to_send);
                        println!("Set wait time to {} milliseconds", ack_handler.wait_time);
                    }

                    // Resend logic
                    let next_slot = tdma_utils::get_slot_after_propag(&comms_config, ack_handler.wait_time);
                    if  !ack_handler.has_unacked_messages() && next_slot == tdma_scheduler.assigned_slot {
                        protocols::resend_messages(&comms_config, &mut modem, &mut ack_handler, &acks_to_send);
                    }

                }

            } else { // Receiving slot
                println!("Node {}: Listening for messages...", comms_config.agent_id);

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
                        Err(_e) => { // TODO: return timeout instead of error
                            //eprintln!("Error receiving message: {}", e);
                            continue;
                        }
                    }
                }
                for msg in &ack_handler.listened_msg.clone() {
                    ack_handler.handle_acknowledgments(&msg, comms_config.network_participant_count);
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