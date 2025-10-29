// Import necessary modules
use std::time::{SystemTime, UNIX_EPOCH}; // Time management utilities
use std::fs;
use std::error::Error;
use serde_json;

use crate::seatrac::enums::CID_E;

#[derive(serde::Deserialize)]
pub struct CommsConfig {
    pub tdma_slot_duration_s: u8,
    pub beacons_in_network: u8,
    pub node_id: u8,
    pub num_floats_pos: u8,
    pub propagation_time: u64, // in ms
}

impl CommsConfig {
    pub fn load_from_file(path: &str) -> Result<Self, Box<dyn Error>> {
        let contents = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read configuration file '{}': {}", path, e))?;
        let config: CommsConfig = serde_json::from_str(&contents)
            .map_err(|e| format!("Failed to parse JSON configuration: {}", e))?;
        Ok(config)
    }
}

#[derive(Clone, Debug)]
pub struct AcousticMessage {
    pub command_id: CID_E,
    pub payload: Vec<u8>,
}

// Function to get the total seconds since UNIX epoch
pub fn get_total_seconds() -> u64 {
    let now = SystemTime::now();
    let duration_since_epoch = now.duration_since(UNIX_EPOCH)
        .expect("Time went backwards!");

    duration_since_epoch.as_secs()
}
