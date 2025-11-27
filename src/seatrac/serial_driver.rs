use serialport::{SerialPort, DataBits, Parity, StopBits, FlowControl};
use std::time::{Duration, Instant};
use std::error::Error;
use std::io::{Read, Write};
use crate::modem_driver::ModemDriver;
use crate::seatrac::enums::{self, CID_E, CST_E};
use crate::seatrac::ascii_message::{make_command, make_command_u16, parse_message, parse_response, prepare_message};
// use crate::seatrac::helpers::calculate_propagation_time;
use crate::seatrac::structs::{DAT_SEND, SETTINGS_T, STATUS_BITS_T, STATUS_RESPONSE};


static _DEFAULT_BAUD_RATE: u32 = 115200;

pub struct SerialModem {
    port: Box<dyn SerialPort>,
    usbl: bool,
    node_id: u8,
    propagation_time_ms: u32
}

impl SerialModem {
    pub fn new(port_name: &str, baud_rate: u32, usbl: bool, node_id: u8, propagation_time_ms: u32) -> Result<Self, Box<dyn Error>> {
        let port = serialport::new(port_name, baud_rate)
                .timeout(Duration::from_millis(50))
                .stop_bits(StopBits::Two)
                .flow_control(FlowControl::None)
                .parity(Parity::None)
                .data_bits(DataBits::Eight)
                .open()
                .map_err(|e| format!("Failed to open port '{}': {e}", port_name))?;
        Ok(Self { port, usbl, node_id, propagation_time_ms })
    }

    /// Host-side only: change serial baud rate
    fn set_local_baud_rate(&mut self, baud_rate: u32) -> Result<(), Box<dyn Error>> {
        self.port.set_baud_rate(baud_rate)?;
        Ok(())
    }

    /// Waits for a response with a specific CID, returns the payload or timeout error.
    /// Useful for configuration
    fn wait_for_response(&mut self, expected_cid: CID_E, timeout: Duration) -> Result<Vec<u8>, Box<dyn Error>> {
        // println!("Waiting for response: {:?}", expected_cid);
        let start = Instant::now();
        while start.elapsed() < timeout {
            match self.receive() {
                Ok(data) => {
                    // println!("Received data while waiting for response: {:?}", String::from_utf8_lossy(&data));
                    if let Ok((cid, resp_payload, _)) = parse_response(&data) {
                        if cid == expected_cid {
                            // println!("Received expected response: {:?}", resp_payload);
                            return Ok(resp_payload);
                        }
                    }
                }
                Err(e) => {
                    if let Some(io_err) = e.downcast_ref::<std::io::Error>() {
                        if io_err.kind() == std::io::ErrorKind::TimedOut {
                            continue;
                        }
                    }
                    return Err(e);
                }
            }
        }
        Err(format!("Timeout waiting for response with CID: {:?}", expected_cid).into())
    }

    // Helper to clamp floats into u8 range safely
    fn clamp_to_u8(v: f64) -> u8 {
        if v.is_nan() {
            return 0;
        }
        if v <= 0.0 {
            0
        } else if v >= 255.0 {
            255
        } else {
            v as u8
        }
    }

}

impl ModemDriver for SerialModem {
    fn send(&mut self, data: &[u8]) -> Result<(), Box<dyn Error>> {
        self.port.write_all(data)?;
        Ok(())
    }
    fn receive(&mut self) -> Result<Vec<u8>, Box<dyn Error>> {
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
    fn configure(&mut self, usbl: bool, baud_rate: u32, beacon_id: u8, salinity: u16) -> Result<(), Box<dyn Error>> {
        let mut reboot = false;
        println!("Configuring modem: usbl={}, baud_rate={}, beacon_id={}, salinity={}", usbl, baud_rate, beacon_id, salinity);
        // 1. Get current beacon settings
        let get_cmd = make_command(CID_E::CID_SETTINGS_GET, &[]);
        println!("Sending settings get command: {:?}", String::from_utf8_lossy(&get_cmd));
        self.send(&get_cmd)?;

        let get_resp = self.wait_for_response(CID_E::CID_SETTINGS_GET, Duration::from_secs(10))?;
        let mut settings = SETTINGS_T::from_bytes(&get_resp)?;

        // 2. Modify settings as needed: map incoming values to existing struct fields
        // if usbl {
        //     settings.xcvr_flags |= XCVR_FLAGS::FIX_MSGS | XCVR_FLAGS::BASELINES_MSGS; 
        // } // Not necessary, as DAT_RECIEVE will generate USBL info if on a usbl modem
        settings.status_flags = enums::STATUSMODE_E::STATUS_MODE_MANUAL.to_u8(); // Set to manual mode to control status message contents
        settings.status_mode = enums::STATUSMODE_E::STATUS_MODE_MANUAL; // Set to manual mode to control status message contents
        // TODO: doesnt work yet

        let current_baud = settings.uart_main_baud.clone();
        let new_baud = enums::BAUDRATE_E::from_u32(baud_rate).unwrap_or(current_baud.clone());
        if new_baud == current_baud.clone() {
            println!("Beacon already at desired baud rate: {:?}", current_baud);
        } else {
            log::warn!("Changing baudrate requires reboot, implementation not finished. Will not apply new baudrate");
            println!("Baud rate change from {:?} to {:?}", current_baud, new_baud.clone());
            settings.uart_main_baud = new_baud;
            reboot = true;

        }
        settings.xcvr_beacon_id = enums::BID_E::from_u8(beacon_id).unwrap_or(settings.xcvr_beacon_id);
        settings.env_salinity = salinity;
        // settings.status_output |= STATUS_BITS_T::ENVIRONMENT; // Enable env data in status messages

        // 4. Send new settings to beacon
        let settings_bytes = settings.to_bytes()?;
        let set_cmd = make_command(CID_E::CID_SETTINGS_SET, &settings_bytes);
        self.send(&set_cmd)?;

        let set_resp = self.wait_for_response(CID_E::CID_SETTINGS_SET, Duration::from_secs(10))?;
        match set_resp.get(0).map(|&b| CST_E::from_u8(b)) {
            Some(Some(status)) => log::info!("Settings set, status: {:?}", status),
            _ => log::warn!("Failed to parse status from settings response"),
        }

        // 5. Save settings to EEPROM if needed
        let save_cmd = make_command(CID_E::CID_SETTINGS_SAVE, &[]);
        self.send(&save_cmd)?;
        let save_resp = self.wait_for_response(CID_E::CID_SETTINGS_SAVE, Duration::from_secs(2))?;
        match save_resp.get(0).map(|&b| CST_E::from_u8(b)) {
            Some(Some(status)) => log::info!("Settings saved, status: {:?}", status),
            _ => log::warn!("Failed to parse status from settings response"),
        }

        // 6. Reboot beacon if needed to apply new settings
        // TODO: check if this works. reboot=true currently commented out
        if reboot {
            let reboot_cmd = make_command_u16(CID_E::CID_SYS_REBOOT, 0x6A95);
            self.send(&reboot_cmd)?;
            log::info!("Sent reboot command to beacon.");

            self.set_local_baud_rate(baud_rate)?;
            println!("Local serial port baud rate set to {}", baud_rate);

            let reboot_resp = self.wait_for_response(CID_E::CID_SYS_REBOOT, Duration::from_millis(700))?; // Reboot has 500ms deadtime
            match reboot_resp.get(0).map(|&b| CST_E::from_u8(b)) {
                Some(Some(status)) => log::info!("Reboot command sent, status: {:?}", status),
                _ => log::warn!("Failed to parse status from reboot response"),
            }
        }


        Ok(())
    }

    fn get_position(&mut self, t: u64) -> Result<Vec<u8>, Box<dyn Error>> {
        // println!("Getting position for node ID {}", self.node_id);
        // Create a command with the STATUS_BITS_T::ENVIRONMENT flag set
        let environment_flag = STATUS_BITS_T::ENVIRONMENT.bits();
        let get_cmd = make_command(CID_E::CID_STATUS, &[environment_flag]);
        // println!("Sending status get command: {:?}", String::from_utf8_lossy(&get_cmd));
        self.send(&get_cmd)?;
        let response_payload = self.wait_for_response(CID_E::CID_STATUS, Duration::from_secs(3))?;
        //let (_cid, payload, _rec_checksum) = parse_response(&response)?;
        let status = STATUS_RESPONSE::from_bytes(&response_payload)?;

        let z: f64;
        if let Some(env_depth) = status.env_depth {
            z = env_depth as f64;
        } else {
            z = 4.0 + 0.5 * (0.1 * t as f64).sin(); // Default value if env_depth is not available
        }

        // Calculate position based on node_id
        match self.node_id {
            0 => {
                let x = 2.0 * t as f64;
                let y = 5.0 * (0.1 * t as f64).sin();
                let x_u8 = Self::clamp_to_u8(x);
                let y_u8 = Self::clamp_to_u8(y);
                let z_u8 = Self::clamp_to_u8(z);
                Ok(vec![x_u8, y_u8, z_u8])
            }
            1 => {
                let x = 2.0 * t as f64; 
                let y = 20.0 - 4.0 * (0.07 * t as f64).sin();
                let x_u8 = Self::clamp_to_u8(x);
                let y_u8 = Self::clamp_to_u8(y);
                let z_u8 = Self::clamp_to_u8(z);
                Ok(vec![x_u8, y_u8, z_u8])
            }
            2 => {
                let x = 15.0 * (0.05 * t as f64).cos();
                let y = 15.0 * (0.05 * t as f64).sin();
                let x_u8 = Self::clamp_to_u8(x);
                let y_u8 = Self::clamp_to_u8(y);
                let z_u8 = Self::clamp_to_u8(z);
                Ok(vec![x_u8, y_u8, z_u8])
            }
            _ => Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "Error: get_position received an invalid node_id"))), // Handle invalid node IDs
        }
    }

    fn to_serial(&self, destination_id: u8, data: &[u8]) -> Vec<u8> {
        let dat_msg = DAT_SEND::new(destination_id, data.to_vec());
        // println!("Preparing DAT_SEND message to node ID {}: {:?}", destination_id, dat_msg);
        // println!("{:?}", dat_msg);
        // make_command(CID_E::CID_DAT_SEND, &dat_msg.to_bytes())
        prepare_message(CID_E::CID_DAT_SEND, dat_msg)
    }

    // Parses a raw byte message received from the modem into command id and byte payload
    fn from_serial(&self, data: &[u8]) -> Result<(String, Vec<u8>), Box<dyn Error>> {
        let (cid, _payload, _checksum, byte_representation) = parse_message(data.to_vec())?;
        // println!("Received {} message", cid.to_string());
        // TODO: change to only allow dat_receive?
        Ok((cid.to_string().into(), byte_representation))
    }

    fn is_usbl(&self) -> bool {
        self.usbl
    }
}