use serialport::{SerialPort, FlowControl, Parity, DataBits};
use std::error::Error;
use std::io::{Read, Write};
use crate::modem_driver::ModemDriver;
use crate::seatrac::enums::{self, CID_E};
use crate::seatrac::ascii_message::{make_command_message, parse_response_message};
use crate::seatrac::structs::{SETTINGS_T};

pub struct SerialModem {
    port: Box<dyn SerialPort>,
}

impl SerialModem {
    pub fn new(port_name: &str, baud_rate: u32) -> Result<Self, Box<dyn Error>> {
        let builder = serialport::new(port_name, baud_rate)
            .timeout(std::time::Duration::from_millis(50))
            .stop_bits(serialport::StopBits::Two)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .data_bits(DataBits::Eight);

        let port = builder.open()
            .expect("Failed to open port");

        Ok(Self { port })
    }

    /// Host-side only: change serial baud rate
    pub fn set_local_baud_rate(&mut self, baud_rate: u32) -> Result<(), Box<dyn Error>> {
        self.port.set_baud_rate(baud_rate)?;
        Ok(())
    }
}

impl ModemDriver for SerialModem {
    fn send(&mut self, data: &[u8]) -> Result<(), Box<dyn Error>> {
        self.port.write_all(data)?;
        Ok(())
    }
    fn receive(&mut self) -> Result<Vec<u8>, Box<dyn Error>> {
        // Read one byte at a time, ignore CR (\r), stop on LF (\n).
        let mut response: Vec<u8> = Vec::new();
        let mut buf = [0u8; 1];

        loop {
            match self.port.read(&mut buf) {
                Ok(0) => {
                    // Port closed / EOF
                    return Err(Box::new(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "port closed while reading",
                    )));
                }
                Ok(_) => {
                    let b = buf[0];
                    if b == b'\r' {
                        // ignore carriage return
                        continue;
                    } else if b == b'\n' {
                        // end of message
                        break;
                    } else {
                        response.push(b);
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    // timeout — try again to read until newline
                    continue;
                }
                Err(e) => return Err(Box::new(e)),
            }
        }

        Ok(response)
    }

    /// Configure both host and beacon via protocol (change baud rate, beacon ID, etc.)
    fn configure(&mut self, baud_rate: u32, beacon_id: u8) -> Result<(), Box<dyn Error>> {
        // 1. Set local serial port baud rate
        self.set_local_baud_rate(baud_rate)?;
        // 2. Get current beacon settings
        let get_cmd = make_command_message(CID_E::CID_SETTINGS_GET, &[]);
        self.send(&get_cmd)?;
        let resp = self.receive()?;
        let (_cid, payload, _csum) = parse_response_message(&resp)?;
        let mut settings = SETTINGS_T::from_bytes(payload)?; // take ownership of payload Vec<u8>

        // 3. Modify settings as needed: map incoming values to existing struct fields
        // Convert the requested baud_rate to the nearest BAUDRATE_E variant if possible
        settings.uart_main_baud = match baud_rate {
            4800 => enums::BAUDRATE_E::BAUD_4800,
            9600 => enums::BAUDRATE_E::BAUD_9600,
            14400 => enums::BAUDRATE_E::BAUD_14400,
            19200 => enums::BAUDRATE_E::BAUD_19200,
            38400 => enums::BAUDRATE_E::BAUD_38400,
            57600 => enums::BAUDRATE_E::BAUD_57600,
            115200 => enums::BAUDRATE_E::BAUD_115200,
            _ => settings.uart_main_baud, // leave unchanged if not recognized
        };
        settings.xcvr_beacon_id = match enums::BID_E::from_u8(beacon_id) {
            Some(b) => b,
            None => settings.xcvr_beacon_id,
        };

        // 4. Send new settings to beacon
        let settings_bytes = settings.to_bytes()?;
        let set_cmd = make_command_message(CID_E::CID_SETTINGS_SET, &settings_bytes);
        self.send(&set_cmd)?;
        let set_resp = self.receive()?;
        let (_cid, _payload, _csum) = parse_response_message(&set_resp)?;
        // Optionally check status code from _payload

        // 5. Save settings to EEPROM if needed
        let save_cmd = make_command_message(CID_E::CID_SETTINGS_SAVE, &[]);
        self.send(&save_cmd)?;
        let save_resp = self.receive()?;
        let (_cid, _payload, _csum) = parse_response_message(&save_resp)?;
        // Optionally check status code from _payload

        Ok(())
    }
}