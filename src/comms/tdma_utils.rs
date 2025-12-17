// Import necessary modules
use std::time::{SystemTime, UNIX_EPOCH}; // Time management utilities
use std::fs;
use std::error::Error;
use serde_json;


#[derive(serde::Deserialize)]
pub struct CommunicationConfig {
    pub tdma_slot_duration_s: u8,
    pub network_participant_count: u8,
    pub agent_id: u8,
    pub msg_propgagation_speed: u64, // bytes per millisecond
}

impl CommunicationConfig {
    pub fn load_from_file(path: &str) -> Result<Self, Box<dyn Error>> {
        let contents = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read configuration file '{}': {}", path, e))?;
        let config: CommunicationConfig = serde_json::from_str(&contents)
            .map_err(|e| format!("Failed to parse JSON configuration: {}", e))?;
        Ok(config)
    }
}


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
pub fn get_slot_after_propag(comms_config: &CommunicationConfig, wait_time_ms: u64) -> u8 {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs() as u64 + wait_time_ms / 1000;
    let cycle_time = comms_config.tdma_slot_duration_s as u64 * comms_config.network_participant_count as u64;
    ((now % cycle_time) / comms_config.tdma_slot_duration_s as u64) as u8
}