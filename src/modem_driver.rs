pub trait ModemDriver {
    /// Sends raw bytes or a message struct, returns Result<(), Error>
    fn send(&mut self, data: &[u8]) -> Result<(), Box<dyn std::error::Error>>;
    /// Receives raw bytes or a message struct, returns Result<Vec<u8>, Error>
    fn receive(&mut self) -> Result<Vec<u8>, Box<dyn std::error::Error>>;
    /// Optional: configure modem, set baudrate, etc.
    fn configure(&mut self, baud_rate: u32, beacon_id: u8) -> Result<(), Box<dyn std::error::Error>>;
}