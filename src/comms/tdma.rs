use crate::modem_driver::ModemDriver;
use crate::seatrac::ascii_message::{make_command, parse_response};
use crate::comms::tdma_utils::{CommsConfig, AcousticMessage};
use crate::comms::tdma_utils::{get_tdma_slot_after_propag, get_current_tdma_slot, get_total_seconds};
use crate::comms::message_manager::{create_ack, calculate_propagation_time, MsgReceivedAck, SentMessage, SentMsgManager};
use std::sync::mpsc::{Sender, Receiver};
use std::time::{Duration, Instant};
use std::collections::VecDeque;


/// Handles TDMA-based communication between nodes using a serial port/TCP protocol.
///
/// Accepts a trait object reference so callers can pass any concrete modem
/// implementing `ModemDriver` (e.g. `SerialModem`) or a boxed trait object
/// (`Box<dyn ModemDriver>`). This avoids forcing the function to be generic
/// and makes it easier to call from places where the modem type is chosen at
/// runtime.
pub fn tdma_communication_tcp(
    modem: &mut dyn ModemDriver,
    config: CommsConfig,
    to_tdma_rx: Receiver<AcousticMessage>,
    from_tdma_tx: Sender<AcousticMessage>,
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
    let mut queue_new_msg: VecDeque<AcousticMessage> = VecDeque::new(); // TODO: modify NewMsg so can hold other message types
    
    loop{
        let current_slot = get_current_tdma_slot(cycle_duration.as_secs(), slot_duration.as_secs());

        if current_slot == assigned_slot as u64 {
            let mut wait_time = 0; // for resend?
            let ack_msg = create_ack(&mut received_acks);
            received_acks.clear();

            while let Ok(msg) = to_tdma_rx.try_recv() {
                queue_new_msg.push_back(msg);
            }
            if !queue_new_msg.is_empty(){
                message_index +=1;
                let message = queue_new_msg.pop_front().expect("Empty queue");
                sent_manager.add_message(
                    message_index, 
                    SentMessage::new(message.clone(), get_total_seconds()-initial_time as u64)
                );
                wait_time = calculate_propagation_time(&message, config.propagation_time);
                let bytes_to_send = make_command(message.command_id, &message.payload);
                modem.send(&bytes_to_send).unwrap();
            }
            // TODO: Resend messages if needed

        } else {
            // TODO: implement listen_for_messages_TCP functionality
            // It's not our turn, listen for messages
            match modem.receive() {
                Ok(data) => {
                    // Process received data
                    let (cid, payload, _) = parse_response(data.as_slice()).unwrap();
                    println!("Received message with CID: {:?}", cid);
                    
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