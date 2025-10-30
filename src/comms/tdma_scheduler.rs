use std::{mem, thread::sleep, time::{Duration, SystemTime, UNIX_EPOCH}};
use crate::{comms::{ack_manager::{SentMessage, SentMsgManager}, 
                dccl, 
                message_types::{self, NewMsg, PositionalCoordinates}, 
                tdma_utils}, 
            modem_driver::ModemDriver, 
            seatrac::{structs::DAT_RECEIVE}}; // Adjust the path based on the actual location of ModemDriver

pub struct TdmaScheduler {
    pub total_slots: u8,
    pub slot_duration_s: u8,
    pub assigned_slot: u8,
}

impl TdmaScheduler {
    pub fn new(total_slots: u8, slot_duration_s: u8, assigned_slot: u8) -> Self {
        Self { total_slots, slot_duration_s, assigned_slot }
    }

    pub fn current_slot(&self) -> u8 {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs() as u64;
        let cycle_time = self.total_slots as u64 * self.slot_duration_s as u64;
        ((now % cycle_time) / self.slot_duration_s as u64) as u8
    }

    pub fn is_my_slot(&self) -> bool {
        self.current_slot() == self.assigned_slot
    }

    pub fn get_slot_after_propag(&self, wait_time_ms: u64) -> u8 {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs() as u64 + wait_time_ms / 1000;
        let cycle_time = self.total_slots as u64 * self.slot_duration_s as u64;
        ((now % cycle_time) / self.slot_duration_s as u64) as u8
    }

    // TODO: move into other module??
    pub fn broadcast_status_msg(&self, modem: &mut dyn ModemDriver, initial_time: u64, ack_handler: &mut SentMsgManager, slot_acks: &Vec<i32>, propagation_time_ms: u64) {
        let t = tdma_utils::get_total_seconds() - initial_time;
        // message_types::prepare_new_msg(&mut *modem, t, &mut ack_handler.queue_new_msg);
        let pos = modem.get_position(t).unwrap(); // Get the node's position
        let pos = PositionalCoordinates::new(pos[0], pos[1], pos[2]);
        let new_msg = NewMsg::new(pos, t);
        ack_handler.queue_new_msg.push_back(new_msg);

        if !ack_handler.queue_new_msg.is_empty() {
            ack_handler.message_index += 1;
            let new_msg = ack_handler.queue_new_msg.pop_front().expect("Empty queue");
            ack_handler.add_message(ack_handler.message_index, SentMessage::new(new_msg.position, new_msg.t));
            let dccl_message = new_msg.to_bytes(self.assigned_slot, ack_handler.message_index, slot_acks.clone());
            let modem_command = modem.message_out(0, &dccl_message);
            ack_handler.wait_time = mem::size_of_val(&modem_command) as u64 * propagation_time_ms; // TODO: make func?
            modem.send(&modem_command).unwrap();
        }
        
    }

    // TODO: move into own module
    pub fn resend_messages(&self, modem: &mut dyn ModemDriver, ack_handler: &mut SentMsgManager, slot_acks: &Vec<i32>, propagation_time_ms: u64) {
        // Resend all messages in the sent_manager
        for (index, message) in ack_handler.list_messages() {
            if index != ack_handler.message_index { // Don't resend the just sent message
                // Wait before resending the message
                println!("Waiting propagation previous message ({} milliseconds) before resending.", ack_handler.wait_time);
                sleep(Duration::from_millis(ack_handler.wait_time));

                let dccl_message = message.to_bytes(self.assigned_slot, index, slot_acks.clone());
                let modem_command = modem.message_out(0, &dccl_message);
                ack_handler.wait_time = mem::size_of_val(&modem_command) as u64 * propagation_time_ms; // TODO: make func
                // TODO: if still in slot after waittime, send message, else not enough time to resend
                //ack_handler.wait_time = calculate_propagation_time(&message.byte_msg, propagation_time_ms);
                let slot_after_propag = self.get_slot_after_propag(ack_handler.wait_time);
                if slot_after_propag == self.assigned_slot {
                    modem.send(&modem_command).unwrap();
                } else {
                    println!("Not enough time to resend message {}, skipping.", index);
                    break; // Exit the loop if it's no longer our slot
                }
            }

        }
    }

    // move into other module 
    pub fn receive_message(&self, modem: &mut dyn ModemDriver) -> Result<(message_types::ReceivedMsg, Option<message_types::UsblData>), Box<dyn std::error::Error>> {
        let mut received_msg = message_types::ReceivedMsg::new_empty();
        let mut usbl_data: Option<message_types::UsblData> = Some(message_types::UsblData::new(0, vec![0], 0, 0, 0));

        // It's not our turn, listen for messages
        let received_serial = match modem.receive() {
            Ok(data) => data,
            Err(e) => return Err(e),
        };
        let t_received = tdma_utils::get_total_seconds();

        // parse response with modem.message_in()
        let (message_type, recieved_bytes) = modem.message_in(&received_serial)?;
        match message_type.as_str() { // Can match on other message types here also, if many, might want to switch to a hashmap instead
            "CID_DAT_RECEIVE" => { // TODO: move functionality under to driver module
                // now make dat_recieve message!
                let dat_receive = DAT_RECEIVE::from_bytes(recieved_bytes)?;
                // check local flag
                if dat_receive.local_flag { // Means that message was sent to this node
                    let encoded_packet = dat_receive.packet_data;
                    let packet_string = dccl::decode_output(&encoded_packet)?;
                    received_msg = message_types::ReceivedMsg::from_string(&packet_string, t_received)?;

                    // TODO: can calculate range here?

                    println!("Received local DAT_RECEIVE packet: {}", packet_string);

                    // If a broadcased message is read by a usbl modem
                    if modem.is_usbl() {
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
            _ => {
                println!("Received unsupported message type: {}", message_type);
            }
        }
        // now make dat_recieve message!
        
        // check: local flag = true
        // dccl decode dat_recieve.payload
        // publish this string to topic
        
        // check: local flag = false and usbl=true
        // extract azimuth and elevation from dat_recieve.aco_fix
        // publish to ekf_input topic


        Ok((received_msg, usbl_data))
    }
}
