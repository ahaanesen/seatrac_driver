// Import necessary modules
use std::time::{SystemTime, UNIX_EPOCH}; // Time management utilities
use crate::parameters;


// Function to get the total seconds since UNIX epoch
pub fn get_total_seconds() -> u64 {
    let now = SystemTime::now();
    let duration_since_epoch = now.duration_since(UNIX_EPOCH)
        .expect("Time went backwards!");

    duration_since_epoch.as_secs()
}

/// Calculates the TDMA slot after a given message propagation time.
/// # Args:
/// - `comms_config`: Reference to the communication configuration settings.
/// - `wait_time_ms`: The propagation time in milliseconds of the previously sent message.
pub fn get_slot_after_propag(comms_config: &parameters::CommsParameters, wait_time_ms: u64) -> u8 {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs() as u64 + wait_time_ms / 1000;
    let cycle_time = comms_config.tdma_slot_duration_s as u64 * comms_config.network_participant_count as u64;
    ((now % cycle_time) / comms_config.tdma_slot_duration_s as u64) as u8
}