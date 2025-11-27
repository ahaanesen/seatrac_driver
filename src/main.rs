use rclrs::*;
use std::{env, thread::{self, sleep}, time::Duration};
// use std_msgs::msg::String as StringMsg;

mod modem_driver;
mod seatrac;
mod comms;
use crate::{comms::{
                ack_manager::{SentMsgManager}, 
                ros2_node,
                tdma_utils,
                comm_protocols,
            },
            modem_driver::ModemDriver};

static DEFAULT_BAUD_RATE: u32 = 115200;


/// Run with "cargo run <node_id> [usbl]"
fn main() -> Result<(), RclrsError> {
    // Load configurations
    let mut driver_config = modem_driver::DriverConfig::load_from_file("config_driver.json").unwrap_or_else(|e| {
        panic!("Failed to load driver configuration: {}", e);
    });

    let mut comms_config = comms::tdma_utils::CommsConfig::load_from_file("config_comms.json").unwrap_or_else(|e| {
        panic!("Failed to load communication configuration: {}", e);
    });


    // Parse command-line arguments to set node id and optional usbl flag
    let args: Vec<String> = env::args().collect();
    let node_id = if args.len() > 1 {
        args[1].parse::<u8>().unwrap_or_else(|_| {
            panic!("Invalid node ID argument. Please provide a valid u8 value.");
        })
    } else {
        panic!("Node ID argument is required.");
    };
    let usbl = args.iter().any(|arg| arg == "usbl"); // Check for optional "usbl" argument

    // Overwrite loaded configs with command-line args
    let port_name = format!("/dev/ttyUSB{}", node_id);
    driver_config.port_name = port_name;
    driver_config.beacon_id = node_id;
    driver_config.usbl = usbl || false;

    comms_config.node_id = node_id;


    // Modem init
    let mut modem;
    match driver_config.modem_type.as_str() {
        "seatrac" => {
            modem = seatrac::serial_driver::SerialModem::new(
                &driver_config.port_name,
                driver_config.baud_rate,
                driver_config.usbl,
                driver_config.beacon_id,
                driver_config.propagation_time,
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
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node_name = "seatrac_".to_string() + &comms_config.node_id.to_string(); //most important the comms and ros2 node have the same id
    let bridge_node = ros2_node::RosBridge::new(&executor, &node_name)?;
    let queues = bridge_node.queues.clone();


    // TDMA scheduler init
    let tdma_scheduler = comms::tdma_scheduler::TdmaScheduler::new(
        comms_config.beacons_in_network,
        comms_config.tdma_slot_duration_s,
        comms_config.node_id,
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
        let mut ack_handler = SentMsgManager::new();
        println!("Starting TDMA scheduler for node ID {}", comms_config.node_id);
        loop {

            if tdma_scheduler.is_my_slot() {
                println!("\n \n Node {}: It's my slot to send! ", comms_config.node_id);
                let acks_to_send = ack_handler.prepare_round();
                comm_protocols::broadcast_status_msg(&comms_config, &mut modem, initial_time as u64, &mut ack_handler, &acks_to_send);

                while tdma_scheduler.is_my_slot() {
                    // Send queued messages from ROS
                    // TODO: still missing implementation
                    let mut send_queue = queues.to_modem.lock().unwrap();

                    for msg in send_queue.drain(..) { // NB: the ros2 messages need the format: "node_id:{} msg_idx:{} x:{} y:{} z:{} t:{} ack:{}"
                        println!("Message from ROS2 to send: {}", msg);
                        if tdma_utils::get_slot_after_propag(&comms_config, ack_handler.wait_time) != tdma_scheduler.assigned_slot {
                            println!("Not enough time to send new message, skipping.");
                            break;
                        }
                        // println!("Waiting propagation previous message ({} seconds) before sending new message.", ack_handler.wait_time);
                        sleep(Duration::from_secs(ack_handler.wait_time));
                        // TODO: process message here maybe?
                        // TODO: can implement lisas basic position dat msg as test first?
                        println!("Not implemented what to do with message: {}", msg);
                    }

                    // Resend logic
                    let next_slot = tdma_utils::get_slot_after_propag(&comms_config, ack_handler.wait_time);
                    if  !ack_handler.is_empty() && next_slot == tdma_scheduler.assigned_slot {
                        comm_protocols::resend_messages(&comms_config, &mut modem, &mut ack_handler, &acks_to_send);
                    }

                }

            } else { // Receiving slot
                println!("Node {}: Listening for messages...", comms_config.node_id);

                while !tdma_scheduler.is_my_slot() {
                    match comm_protocols::receive_message(&mut modem) { 
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
                    ack_handler.handle_acknowledgments(&msg, comms_config.beacons_in_network);
                }

            }

            std::thread::sleep(Duration::from_millis(50));
        }
    });
    

    // Main thread: run the bridge loop => keep Worker/Node/Publisher calls on the same thread that created them.
    loop {
        if let Err(e) = bridge_node.publish_from_queue() {
            eprintln!("Publish error: {:?}", e);
        }
        if let Err(e) = bridge_node.queue_from_subscription() {
            eprintln!("Subscription queue error: {:?}", e);
        }

        // main thread can also do other housekeeping (or integrate with your main loop)
        std::thread::sleep(Duration::from_millis(100));
    }

    // Note: we never reach here because of the infinite loop; if you want graceful shutdown,
    // add a signal handler and join the exec_handle, etc.
    // exec_handle.join().unwrap();
    // Ok(())
}