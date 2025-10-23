use std::time::{SystemTime, UNIX_EPOCH};

pub struct TdmaScheduler {
    pub total_slots: u8,
    pub slot_duration_s: u8,
}

impl TdmaScheduler {
    pub fn new(total_slots: u8, slot_duration_s: u8) -> Self {
        Self { total_slots, slot_duration_s }
    }

    pub fn current_slot(&self) -> u8 {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs() as u64;
        let cycle_time = self.total_slots as u64 * self.slot_duration_s as u64;
        ((now % cycle_time) / self.slot_duration_s as u64) as u8
    }
}
