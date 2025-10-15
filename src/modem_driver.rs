pub trait ModemDriver {
    /// Sends raw bytes or a message struct, returns Result<(), Error>
    fn send(&mut self, data: &[u8]) -> Result<(), Box<dyn std::error::Error>>;
    /// Receives raw bytes or a message struct, returns Result<Vec<u8>, Error>
    fn receive(&mut self) -> Result<Vec<u8>, Box<dyn std::error::Error>>;
    /// Optional: configure modem, set baudrate, etc.

    // TODO: take in salinity as well
    /// Configure the modem with the given parameters.
    /// - `baud_rate`: The desired baud rate for communication.
    /// - `beacon_id`: The ID of the beacon to communicate with.
    /// - `salinity`: The salinity setting for the modem.
    ///
    fn configure(&mut self, baud_rate: u32, beacon_id: u8, salinity: f32) -> Result<(), Box<dyn std::error::Error>>;
}