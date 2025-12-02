use std::fs;
use std::error::Error;
use serde_json;
pub trait ModemDriver {
    // These methods made local local (writ_port and read_port) to avoid exposing serialport dependency outside driver
    // fn write_port(&mut self, data: &[u8]) -> Result<(), Box<dyn std::error::Error>>;
    // fn read_port(&mut self) -> Result<Vec<u8>, Box<dyn std::error::Error>>;


    /// Configure the modem with the given parameters.
    /// - `baud_rate`: The desired baud rate for communication.
    /// - `beacon_id`: The ID of the beacon to communicate with.
    /// - `salinity`: The salinity setting for the modem. Values are encoded as deci-parts-per-thousand (i.e. a value of
    ///   345 represents 34.5 ppt)
    fn configure(&mut self, usbl: bool, baud_rate: u32, beacon_id: u8, salinity: u16) -> Result<(), Box<dyn std::error::Error>>;
    
    fn get_position(&mut self, t: u64) -> Result<Vec<u8>, Box<dyn Error>>;


    /// Given a destination ID and data payload, sends the data via the modem.
    /// Returns the serialized message as a vector of bytes.
    fn send(&mut self, destination_id: u8, data: &[u8]) -> Result<(Vec<u8>), Box<dyn Error>>;

    /// Receives data from the modem
    /// Returns a tuple of message type string and received bytes
    fn receive(&mut self) -> Result<(String, Vec<u8>), Box<dyn Error>>;

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