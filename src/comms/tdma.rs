use crate::modem_driver::ModemDriver;
use crate::comms::tdma_utils::{get_TDMA_slot_after_propag, get_current_TDMA_slot, get_total_seconds};
use crate::comms::message_manager::{MsgReceivedAck, SentMsgManager};
use std::fs;
use std::error::Error;
use serde_json;

#[derive(serde::Deserialize)]
pub struct CommsConfig {
    pub tdma_slot_duration_s: u8,
    pub num_beacons_in_network: u8,
    pub node_id: u8,
    pub num_floats_pos: u8,
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

/// Handles TDMA-based communication between nodes using a serial port/TCP protocol.
///
/// Accepts a trait object reference so callers can pass any concrete modem
/// implementing `ModemDriver` (e.g. `SerialModem`) or a boxed trait object
/// (`Box<dyn ModemDriver>`). This avoids forcing the function to be generic
/// and makes it easier to call from places where the modem type is chosen at
/// runtime.
pub fn tdma_communication_tcp(
    modem: &mut dyn ModemDriver,
    config: &CommsConfig,
) {
    // Implementation for TDMA communication over TCP
    println!("Starting TDMA communication over TCP with node ID: {}", config.node_id);

    // Initialize variables for TDMA communication
    let mut nmsg = 0; // Message counter
    let mut received: Vec<MsgReceivedAck> = vec![]; // Vector to store received acknowledgments
    let mut sentManage = SentMsgManager::new(); // Message manager to track sent messages
    let cycle_duration_s = config.tdma_slot_duration_s * config.num_beacons_in_network; // Calculate total cycle duration

}