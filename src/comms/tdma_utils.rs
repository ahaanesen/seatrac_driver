// Import necessary modules
use std::time::{SystemTime, UNIX_EPOCH, Duration}; // Time management utilities

// Function to get the total seconds since UNIX epoch
pub fn get_total_seconds() -> u64 {
    let now = SystemTime::now();
    let duration_since_epoch = now.duration_since(UNIX_EPOCH)
        .expect("Time went backwards!");

    duration_since_epoch.as_secs()
}

// Function to calculate the current slot in the TDMA cycle based on cycle and slot durations
pub fn get_current_TDMA_slot(cycle_duration: u64, slot_duration: u64) -> u64 {
    let total_seconds = get_total_seconds();
    let elapsed = total_seconds % cycle_duration; // Calculate time passed within the current cycle
    elapsed / slot_duration // Return current slot number
}
pub fn get_TDMA_slot_after_propag(cycle_duration: u64, slot_duration: u64, propagation_time: u64) -> u64 {
    let total_seconds = get_total_seconds() + propagation_time;
    let elapsed = total_seconds % cycle_duration; // Calculate time passed within the current cycle
    elapsed / slot_duration // Return current slot number
}

