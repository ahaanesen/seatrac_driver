use std::time::{SystemTime, UNIX_EPOCH};
use crate::{comms::{ack_manager::{SentMessage, SentMsgManager}, dccl::encode_input, message_types::{self, NewMsg, PositionalCoordinates}, tdma_utils}, modem_driver::ModemDriver}; // Adjust the path based on the actual location of ModemDriver

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

    // TODO: move into other module??
    pub fn broadcast_status_msg(&self, modem: &mut dyn ModemDriver, initial_time: u64, ack_handler: &mut SentMsgManager) {
        let slot_acks = ack_handler.initialize_ack_slot();
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

            let position_string = new_msg.position.to_string();
            let status_string = format!("node_id:{}, msg_idx:{}, pos:{}, t:{}, ack:{:?}", 
                self.assigned_slot,
                ack_handler.message_index,
                position_string,
                new_msg.t,
                slot_acks
            );
            let packet_data = encode_input(&status_string).expect("Encoding failed");
            ack_handler.wait_time = modem.broadcast_msg(&packet_data).unwrap();

            println!("Broadcasted status message: {}", status_string);
        }
        
    }

    // TODO: move into own module
    pub fn resend_messages(&self, modem: &mut dyn ModemDriver, ack_handler: &mut SentMsgManager) {
        // TODO:
    }
}
