use serialport::{SerialPort, DataBits, Parity, StopBits, FlowControl}; // Serial port settings and controls
use std::time::{Duration, Instant};
use std::error::Error;
use std::io::{Read, Write};
use crate::modem_driver::ModemDriver;
use log::{info, warn};
use crate::seatrac::enums::{self, CID_E, CST_E, BAUDRATE_E, BID_E};
use crate::seatrac::ascii_message::{make_command, parse_response, make_command_u16};
use crate::seatrac::structs::{SETTINGS_T, XCVR_FLAGS};


static DEFAULT_BAUD_RATE: u32 = 115200;

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
                .map_err(|e| format!("Failed to open port '{}': {e}", port_name))?;
        Ok(Self { port })
    }

    /// Host-side only: change serial baud rate
    pub fn set_local_baud_rate(&mut self, baud_rate: u32) -> Result<(), Box<dyn Error>> {
        self.port.set_baud_rate(baud_rate)?;
        Ok(())
    }

    /// Waits for a response with a specific CID, returns the payload or timeout error.
    /// Useful for configuration
    fn wait_for_response(&mut self, expected_cid: CID_E, timeout: Duration) -> Result<Vec<u8>, Box<dyn Error>> {
        println!("Waiting for response: {:?}", expected_cid);
        let start = Instant::now();
        while start.elapsed() < timeout {
            match self.receive() {
                Ok(data) => {
                    if let Ok((cid, resp_payload, _)) = parse_response(&data) {
                        if cid == expected_cid {
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
        Err(Box::new(std::io::Error::new(std::io::ErrorKind::TimedOut, "Timeout waiting for response")))
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

        // 1. Get current beacon settings
        let get_cmd = make_command(CID_E::CID_SETTINGS_GET, &[]);
        println!("Sending settings get command: {:?}", String::from_utf8_lossy(&get_cmd));
        self.send(&get_cmd)?;

        let get_resp = self.wait_for_response(CID_E::CID_SETTINGS_GET, Duration::from_secs(10))?;
        let mut settings = SETTINGS_T::from_bytes(&get_resp)?;

        // 2. Modify settings as needed: map incoming values to existing struct fields
        if usbl {
            settings.xcvr_flags |= XCVR_FLAGS::FIX_MSGS | XCVR_FLAGS::BASELINES_MSGS; 
        }
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

            let reboot_resp = self.wait_for_response(CID_E::CID_SYS_REBOOT, Duration::from_secs(7))?;
            match reboot_resp.get(0).map(|&b| CST_E::from_u8(b)) {
                Some(Some(status)) => log::info!("Reboot command sent, status: {:?}", status),
                _ => log::warn!("Failed to parse status from reboot response"),
            }
        }


        Ok(())
    }
}