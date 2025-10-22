use crate::modem_driver::ModemDriver;
use crate::comms::tdma_utils::{CommsConfig};
use crate::comms::tdma_utils::{get_tdma_slot_after_propag, get_current_tdma_slot, get_total_seconds};
use crate::comms::message_manager::{create_ack, calculate_propagation_time, MsgReceivedAck, SentMessage, SentMsgManager};
use std::sync::mpsc::{Sender, Receiver};
use std::time::{Duration, Instant};
use std::collections::VecDeque;
use std::thread::sleep;


/// Handles TDMA-based communication between nodes using a serial port/TCP protocol.
///
/// Accepts a trait object reference so callers can pass any concrete modem
/// implementing `ModemDriver` (e.g. `SerialModem`) or a boxed trait object
/// (`Box<dyn ModemDriver>`). This avoids forcing the function to be generic
/// and makes it easier to call from places where the modem type is chosen at
/// runtime.
/// # Args:
/// - `modem`: A mutable reference to a modem implementing the ModemDriver trait.
/// - `config`: The communication configuration settings.
/// - `to_tdma_rx`: Receiver for incoming messages to be sent as vec<u8> (independent of modem specifics functions/structs)
/// - `from_tdma_tx`: Sender for outgoing received messages as vec<u8> (independent of modem specific functions/structs)
pub fn tdma_communication_tcp(
    modem: &mut dyn ModemDriver,
    config: CommsConfig,
    to_tdma_rx: Receiver<Vec<u8>>,
    from_tdma_tx: Sender<Vec<u8>>,
) {
    // Implementation for TDMA communication over TCP
    println!("Starting TDMA communication over TCP with node ID: {}", config.node_id);

    // Initialize variables for TDMA communication
    let slot_duration = Duration::from_secs(config.tdma_slot_duration_s as u64);
    let cycle_duration = slot_duration * (config.num_beacons_in_network as u32);
    let assigned_slot = config.node_id;
    // let initial_time = fs::read_to_string("/home/khadas/ros2_ws/start_time.txt")
    //     .expect("Unable to read file")
    //     .trim()
    //     .parse::<f64>()
    //     .expect("Unable to parse start_time");
    let initial_time: f64 = 0.0; // Hardcoded initial time for testing

    let mut sent_manager = SentMsgManager::new(); // Message manager to track sent messages
    let mut message_index = 0; // Message counter
    let mut received_acks: Vec<MsgReceivedAck> = vec![]; // Vector to store received acknowledgments
    let mut queue_new_msg: VecDeque<Vec<u8>> = VecDeque::new();
    let mut listened_msg: Vec<Vec<i32>> = vec![];
    
    loop{
        let current_slot = get_current_tdma_slot(cycle_duration.as_secs(), slot_duration.as_secs());

        if current_slot == assigned_slot as u64 {
            let mut wait_time = 0;
            let ack_msg = create_ack(&mut received_acks);
            received_acks.clear();

            // Collect new messages to send
            while let Ok(outgoing_msg) = to_tdma_rx.try_recv() { // TODO: check if it should only recieve once or if this logic is good
                queue_new_msg.push_back(outgoing_msg);
            }
            
            if !queue_new_msg.is_empty(){
                message_index +=1;
                let message = queue_new_msg.pop_front().expect("Empty queue");
                sent_manager.add_message(
                    message_index, 
                    SentMessage::new(message.clone(), get_total_seconds()-initial_time as u64)
                );
                wait_time = calculate_propagation_time(&message, config.propagation_time);
                modem.send(&message).unwrap();
            }

            // Resend messages if needed
            let slot_after_propag = get_tdma_slot_after_propag(cycle_duration.as_secs(), slot_duration.as_secs(), wait_time);
            if !sent_manager.is_empty() && slot_after_propag == assigned_slot as u64 {
                // Resend all messages in the sent_manager
                for (index, message) in sent_manager.list_messages() {
                    if index != message_index { // Don't resend the just sent message
                        // Wait before resending the message
                        println!("Waiting propagation previous message ({} seconds) before resending.", wait_time);
                        sleep(Duration::from_secs(wait_time));

                        wait_time = calculate_propagation_time(&message.byte_msg, config.propagation_time);
                        let slot_after_propag = get_tdma_slot_after_propag(cycle_duration.as_secs(), slot_duration.as_secs(), wait_time);
                        if slot_after_propag == assigned_slot as u64 {
                            modem.send(&message.byte_msg).unwrap();
                        } else {
                            break; // Exit the loop if it's no longer our slot
                        }
                    }

                }
            }

        } else {
            // TODO: implement listen_for_messages_TCP functionality
            // It's not our turn, listen for messages
            match modem.receive() {
                Ok(data) => {
                    // Process received data
                    listened_msg.push(data.clone());

                    from_tdma_tx.send(data.clone()).unwrap(); // Send received data to main thread
                    
                    // Handle acknowledgments and other logic here
                    // ...
                }
                Err(e) => {
                    // Handle receive error (e.g., timeout)
                    continue; // Ignore timeout errors
                }
            
            }
        }
    }
}