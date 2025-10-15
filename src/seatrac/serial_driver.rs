use serialport::{SerialPort, DataBits, Parity, StopBits, FlowControl}; // Serial port settings and controls
use std::time::Duration;
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
        let port = serialport::new(port_name, baud_rate)
                .timeout(Duration::from_millis(50))
                .stop_bits(StopBits::Two)
                .flow_control(FlowControl::None)
                .parity(Parity::None)
                .data_bits(DataBits::Eight)
                .open()
                .expect("Failed to open port");
        Ok(Self { port })
    }

    /// Host-side only: change serial baud rate
    pub fn set_local_baud_rate(&mut self, baud_rate: u32) -> Result<(), Box<dyn Error>> {
        self.port.set_baud_rate(baud_rate)?;
        Ok(())
    }

    fn wait_for_response(&mut self, expected_cid: CID_E) -> Result<Vec<u8>, Box<dyn Error>> {
        let mut payload = Vec::new();

        // TODO: implement timeout/retry limit
        loop {
            if let Ok(data) = self.receive() {
                println!("Received data: {:?}", String::from_utf8_lossy(&data));
                let Ok((cid, resp_payload, _, _)) = parse_response_message(data.as_slice()) else {
                    continue; // Ignore parse errors
                };
                if cid == expected_cid {
                    // We got the expected response
                    payload = resp_payload;
                    break;
                }
            } else {
                continue; // Ignore timeout errors
            }
        }

        Ok(payload)
    }

}

impl ModemDriver for SerialModem {
    fn send(&mut self, data: &[u8]) -> Result<(), Box<dyn Error>> {
        self.port.write_all(data)?;
        Ok(())
    }
    fn receive(&mut self) -> Result<Vec<u8>, Box<dyn Error>> {
/*         // Read one byte at a time, ignore CR (\r), stop on LF (\n).
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

        Ok(response) */
        let mut response = Vec::new();
        let mut buffer = [0u8; 1];

        loop {
            match self.port.read(&mut buffer) {
                Ok(0) => {
                    return Err(Box::new(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "Port closed while reading",
                    )));
                }
                Ok(_) => match buffer[0] {
                    b'\r' => continue,         // Ignore carriage return
                    b'\n' => break,            // End of message
                    byte => response.push(byte),
                },
                Err(e) => return Err(Box::new(e)),
            }
        }

    Ok(response) // Return the full response
/*         let mut buf = vec![0u8; 1024];
        let n = self.port.read(&mut buf)?;
        buf.truncate(n);
        Ok(buf) */
    }

    /// Configure both host and beacon via protocol (change baud rate, beacon ID, etc.)
    fn configure(&mut self, baud_rate: u32, beacon_id: u8, salinity: f32) -> Result<(), Box<dyn Error>> {

        // 1. Get current beacon settings
        let get_cmd = make_command_message(CID_E::CID_SETTINGS_GET, &[]);
        // println!("Requesting current settings from beacon with command: {:?}", String::from_utf8_lossy(&get_cmd));
        self.send(&get_cmd)?; // TODO: spørsmålstegn er stress

        let mut payload = Vec::new();
        // TODO: implement timeout/retry limit
        // TODO: implement as function, ie pub fn wait_for_response(&mut self, expected_cid: CID_E) -> Result<Vec<u8>, Box<dyn Error>>
        loop {
            if let Ok(data) = self.receive() {
                println!("Received data: {:?}", String::from_utf8_lossy(&data));
                let Ok((cid, resp_payload, _, _)) = parse_response_message(data.as_slice()) else {
                    continue; // Ignore parse errors
                };
                if cid == CID_E::CID_SETTINGS_GET {
                    // We got the expected response
                    payload = resp_payload;
                    break;
                }
            } else {
                continue; // Ignore timeout errors
            }
        }
        
        print!("Payload: {:x?}\n", payload);
        let mut settings = SETTINGS_T::from_hex(payload)?;

        // 3. Modify settings as needed: map incoming values to existing struct fields
        let current_baud = settings.uart_main_baud.clone();
        println!("Current beacon baud rate setting: {:?}", current_baud);
        println!("Changing beacon baud rate to {}", baud_rate);
        settings.uart_main_baud = enums::BAUDRATE_E::from_u32(baud_rate).unwrap_or(current_baud); // Is match better, can explicitly say could not change

        println!("New beacon baud rate setting: {:?}", settings.uart_main_baud);

        println!("Changing beacon ID from {:?} to {}", settings.xcvr_beacon_id, beacon_id);
        settings.xcvr_beacon_id = match enums::BID_E::from_u8(beacon_id) {
            Some(b) => b,
            None => settings.xcvr_beacon_id,
        };

        // 4. Send new settings to beacon
        let settings_bytes = settings.to_bytes()?;
        let set_cmd = make_command_message(CID_E::CID_SETTINGS_SET, &settings_bytes);
        self.send(&set_cmd)?;

        let mut response_payload = Vec::new();
        loop {
            match self.receive() {
                Ok(data) => {
                    println!("Received data: {:?}", String::from_utf8_lossy(&data));
                    let (cid, payload, _, _) = parse_response_message(data.as_slice())?;
                    if cid == CID_E::CID_SETTINGS_SET {
                        response_payload = payload;
                        break;
                    }
                }
                Err(e) => {
                    // eprintln!("Error receiving data: {}", e);
                    continue; // Ignore timeout errors
                }
            }
        }
        // Optionally check status code from response_payload
        match enums::CST_E::from_u8(response_payload[0]) {
            Some(status) => println!("Settings set, status: {:?}", status),
            None => println!("Failed to parse status from settings response"),
        } // Impl note: second way to match enums, using num_traits::FromPrimitive, no need for manual implementation as in first but req. dependency


        // 5. Save settings to EEPROM if needed
        let save_cmd = make_command_message(CID_E::CID_SETTINGS_SAVE, &[]);
        self.send(&save_cmd)?;
        let save_resp = self.receive()?;
        let (_cid, _payload, _csum, _raw) = parse_response_message(&save_resp)?;
        // Optionally check status code from _payload
        match enums::CST_E::from_u8(_payload[0]) {
            Some(status) => println!("Settings saved, status: {:?}", status),
            None => println!("Failed to parse status from settings response"),
        }

        // // 1. Set local serial port baud rate
        // self.set_local_baud_rate(baud_rate)?;
        // println!("Local serial port baud rate set to {}", baud_rate);

        Ok(())
    }
}