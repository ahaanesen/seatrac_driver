use std::{mem, thread::sleep, time::{Duration}};
use crate::{comms::{ack_manager::{SentMessage, SentMsgManager}, dccl, message_types::{self, NewMsg, PositionalCoordinates}, tdma_utils::{self, CommsConfig}}, 
            modem_driver::ModemDriver, 
            seatrac::{self, structs}};


/// Broadcasts a status message with the nodes position
pub fn broadcast_status_msg(comms_config: &CommsConfig, modem: &mut dyn ModemDriver, initial_time: u64, ack_handler: &mut SentMsgManager, slot_acks: &Vec<i32>) {
    let t_send = tdma_utils::get_total_seconds() - initial_time;

    let pos = modem.get_position(t_send).unwrap(); // Get the node's position
    let pos = PositionalCoordinates::new(pos[0], pos[1], pos[2]);

    let new_msg = NewMsg::new(pos, t_send);
    ack_handler.queue_new_msg.push_back(new_msg);

    if !ack_handler.queue_new_msg.is_empty() {
        ack_handler.message_index += 1;
        let new_msg = ack_handler.queue_new_msg.pop_front().expect("Empty queue");
        ack_handler.add_message(ack_handler.message_index, SentMessage::new(new_msg.position, new_msg.t));
        let dccl_message = new_msg.to_bytes(comms_config.node_id, ack_handler.message_index, slot_acks.clone());
        println!("Broadcasting status message: {:?}", dccl::decode_output(&dccl_message));
        let modem_command = modem.message_out(0, &dccl_message);
        // println!("Modem command to send: {:?}", String::from_utf8(modem_command.clone()).unwrap_or("Non-UTF8 data".to_string()));
        ack_handler.wait_time = mem::size_of_val(&modem_command) as u64 * comms_config.propagation_time; // TODO: make func?
        modem.send(&modem_command).unwrap();
    }
    
}


/// Resends all messages in the SentMsgManager that have not been acknowledged
pub fn resend_messages(comms_config: &CommsConfig, modem: &mut dyn ModemDriver, ack_handler: &mut SentMsgManager, slot_acks: &Vec<i32>) {
    for (index, message) in ack_handler.list_messages() {
        if index != ack_handler.message_index { // Don't resend the just sent message
            // Wait before resending the message to not clog the channel
            sleep(Duration::from_millis(ack_handler.wait_time));

            println!("Resending nmsg {}: {:?}", index, message);
            let dccl_message = message.to_bytes(comms_config.node_id, index, slot_acks.clone());
            let modem_command = modem.message_out(0, &dccl_message);
            ack_handler.wait_time = mem::size_of_val(&modem_command) as u64 * comms_config.propagation_time; // TODO: make func?
            // TODO: if still in slot after waittime, send message, else not enough time to resend
            //ack_handler.wait_time = calculate_propagation_time(&message.byte_msg, propagation_time_ms);
            let slot_after_propag = tdma_utils::get_slot_after_propag(comms_config, ack_handler.wait_time);
            if slot_after_propag == comms_config.node_id {
                modem.send(&modem_command).unwrap();
            } else {
                println!("Not enough time to resend message {}, skipping.", index);
                break; // Exit the loop if it's no longer our slot
            }
        }

    }
}


/// Checks for incoming messages during non-assigned slots
/// Returns a tuple of optional received dat message and USBL data
pub fn receive_message(modem: &mut dyn ModemDriver) -> Result<(Option<message_types::ReceivedMsg>, Option<message_types::UsblData>), Box<dyn std::error::Error>> {
    let mut received_msg = None;
    let mut usbl_data: Option<message_types::UsblData> = None; 

    let received_serial = match modem.receive() {
        Ok(data) => data,
        Err(e) => return Err(e),
    };
    let t_received = tdma_utils::get_total_seconds();

    let (message_type, recieved_bytes) = modem.message_in(&received_serial)?;
    // println!("Received message type: {}", message_type.as_str());

    // Can match on other message types here also, if many, might want to switch to a hashmap instead
    match message_type.as_str() { 
        "CID_DAT_RECEIVE" => { // TODO: move functionality under to driver module
            let dat_receive = structs::DAT_RECEIVE::from_bytes(recieved_bytes)?;

            if dat_receive.local_flag { // Means that message was sent to this node
                let encoded_packet = dat_receive.packet_data;
                let packet_string = dccl::decode_output(&encoded_packet)?;
                // println!("Decoded packet data: {:?}", packet_string);
                // println!("made recieved message struct: {:?}", rec);
                received_msg = Some(message_types::ReceivedMsg::from_string(&packet_string, t_received)?);

                // TODO: can calculate range here?

                // println!("Received local DAT_RECEIVE packet: {}", packet_string);

                // If a broadcased message is read by a usbl modem
                if modem.is_usbl() {
                    // let aco_fix = structs::ACOFIX_T::from_bytes(&dat_receive.aco_fix)?;
                    let aco_fix = dat_receive.aco_fix;
                    usbl_data = Some(message_types::UsblData::new(
                        aco_fix.usbl_channels.unwrap_or(0),
                        aco_fix.usbl_rssi.unwrap_or(vec![0]),
                        aco_fix.usbl_azimuth.unwrap_or(0),
                        aco_fix.usbl_elevation.unwrap_or(0),
                        aco_fix.usbl_fit_error.unwrap_or(0),
                    ));
                }

            } else if !dat_receive.local_flag && modem.is_usbl() { // Assumes all messages are not broadcasted!
                // Handle USBL specific logic here
                // let aco_fix = structs::ACOFIX_T::from_bytes(&dat_receive.aco_fix)?;
                let aco_fix = dat_receive.aco_fix;
                usbl_data = Some(message_types::UsblData::new(
                    aco_fix.usbl_channels.unwrap_or(0),
                    aco_fix.usbl_rssi.unwrap_or(vec![0]),
                    aco_fix.usbl_azimuth.unwrap_or(0),
                    aco_fix.usbl_elevation.unwrap_or(0),
                    aco_fix.usbl_fit_error.unwrap_or(0),
                ));

            }
        },
        "CID_DAT_SEND" => {
            // Can ignore, just if sending dat was sucessfull
            let status = seatrac::enums::CST_E::from_u8(recieved_bytes[0])
                .ok_or_else(|| "Invalid CST_E value")?;
            println!("Received CID_DAT_SEND message: {:?}", status);
        }
        _ => {
            // println!("Received unsupported message type: {}", message_type);
        }
    }
    Ok((received_msg, usbl_data))
}