use rclrs::*;
use std::{thread, time::Duration};
use std_msgs::msg::String as StringMsg;

mod modem_driver;
mod seatrac;
mod comms;
use crate::{comms::{ack_manager::SentMsgManager, message_types, ros2_node, tdma_utils}, modem_driver::ModemDriver, seatrac::ascii_message::parse_response};

static DEFAULT_BAUD_RATE: u32 = 115200;


fn main() -> Result<(), RclrsError> {
    let driver_config = modem_driver::DriverConfig::load_from_file("config_driver.json").unwrap_or_else(|e| {
        panic!("Failed to load driver configuration: {}", e);
    });

    let comms_config = comms::tdma_utils::CommsConfig::load_from_file("config_comms.json").unwrap_or_else(|e| {
        panic!("Failed to load communication configuration: {}", e);
    });
    // TODO: nb - should have beacon_id=node_id in driver and comms config
    let scheduler = comms::tdma_scheduler::TdmaScheduler::new(
        comms_config.beacons_in_network,
        comms_config.tdma_slot_duration_s,
        comms_config.node_id,
    );
    // let initial_time = fs::read_to_string("/home/khadas/ros2_ws/start_time.txt")
    //     .expect("Unable to read file")
    //     .trim()
    //     .parse::<f64>()
    //     .expect("Unable to parse start_time");
    let initial_time: f64 = 0.0; // Hardcoded initial time for testing

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

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node_name = "seatrac_node_".to_string() + &comms_config.node_id.to_string(); //most important the comms and ros2 node have the same id
    let bridge_node = ros2_node::RosBridge::new(&executor, &node_name)?;
    let queues = bridge_node.queues.clone();

    // Spawn a thread for serial + TDMA logic
    // NB: all sent and recieved CID_DAT messages need to be handeled for acks
    thread::spawn(move || {
        let mut ack_handler = SentMsgManager::new();
        loop {
            let slot = scheduler.current_slot();

            if scheduler.is_my_slot() {
                // SEND SLOT
                scheduler.broadcast_status_msg(&mut modem, initial_time as u64, &mut ack_handler);
                // TODO: resend if needed
                


                // Send queued messages from ROS
                let mut send_queue = queues.to_modem.lock().unwrap();
                for msg in send_queue.drain(..) {
                    // TODO: process message here maybe?
                    // TODO: can implement lisas basic position dat msg as test first?
                    modem.send(msg.as_bytes()).unwrap();
                }

            } else {
                // RECEIVE SLOT
                if let Ok(data) = modem.receive() {
                    if let Ok((cid, payload, _)) = parse_response(&data) {
                        let formatted = format!(
                            "CID: {:?}, Payload: {}",
                            cid,
                            String::from_utf8_lossy(&payload)
                        );
                        queues.from_modem.lock().unwrap().push(formatted.clone()); // To ROS2 network
                        // TODO: process recieved message here 
                        // if usbl modem, extract position data and publish to specific topic
                        if formatted.contains("CID_DAT"){
                            // extract position data and publish to specific topic
                        }
                    }
                }
            }

            std::thread::sleep(Duration::from_millis(50));
        }
    });
    
    // Spin the ROS executor (handles /seatrac/input subscription)
    thread::spawn({
        let bridge_clone = bridge_node.clone();
        move || loop {
            if let Err(e) = bridge_clone.publish_from_queue() {
                eprintln!("Publish error: {:?}", e);
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    });

    executor.spin(SpinOptions::default()).first_error()
}