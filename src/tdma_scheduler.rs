use std::time::{SystemTime, UNIX_EPOCH};

pub struct TdmaScheduler {
    pub total_slots: u8,
    pub slot_duration_ms: u64,
}

impl TdmaScheduler {
    pub fn new(total_slots: u8, slot_duration_ms: u64) -> Self {
        Self { total_slots, slot_duration_ms }
    }

    pub fn current_slot(&self) -> u8 {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        let cycle_time = self.total_slots as u64 * self.slot_duration_ms;
        ((now % cycle_time) / self.slot_duration_ms) as u8
    }
}
