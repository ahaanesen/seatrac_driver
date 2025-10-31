use std::fs;
use std::error::Error;
use serde_json;
pub trait ModemDriver {
    /// Sends raw bytes or a message struct, returns Result<(), Error>
    fn send(&mut self, data: &[u8]) -> Result<(), Box<dyn std::error::Error>>;

    /// Receives raw bytes or a message struct, returns Result<Vec<u8>, Error>
    fn receive(&mut self) -> Result<Vec<u8>, Box<dyn std::error::Error>>;
    /// Optional: configure modem, set baudrate, etc.

    /// Configure the modem with the given parameters.
    /// - `baud_rate`: The desired baud rate for communication.
    /// - `beacon_id`: The ID of the beacon to communicate with.
    /// - `salinity`: The salinity setting for the modem. Values are encoded as deci-parts-per-thousand (i.e. a value of
    ///   345 represents 34.5 ppt)
    fn configure(&mut self, usbl: bool, baud_rate: u32, beacon_id: u8, salinity: u16) -> Result<(), Box<dyn std::error::Error>>;
    
    fn get_position(&mut self, t: u64) -> Result<Vec<u8>, Box<dyn Error>>;

    fn broadcast_msg(&mut self, data: &[u8]) -> Result<(u64), Box<dyn Error>>;

    /// Making a raw byte message of given (dccl encoded) data that the modem can send
    fn message_out(&self, destination_id: u8, data: &[u8]) -> Vec<u8>;

    /// Parses a raw byte message received from the modem into message type and byte payload
    fn message_in(&self, data: &[u8]) -> Result<(String, Vec<u8>), Box<dyn Error>>;

    fn is_usbl(&self) -> bool;
}

#[derive(serde::Deserialize)]
pub struct DriverConfig {
    pub port_name: String,
    pub baud_rate: u32,

    pub modem_type: String, // "seatrac"
    pub usbl: bool, // true if using USBL modem

    pub beacon_id: u8,

    pub propagation_time: u32, // in ms
    pub salinity: u16, // in deci-parts-per-thousand

}
impl DriverConfig {
    pub fn load_from_file(path: &str) -> Result<Self, Box<dyn Error>> {
        let contents = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read configuration file '{}': {}", path, e))?;
        let config: DriverConfig = serde_json::from_str(&contents)
            .map_err(|e| format!("Failed to parse JSON configuration: {}", e))?;
        Ok(config)
    }
}