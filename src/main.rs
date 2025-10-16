mod modem_driver;
mod seatrac;
mod protocols;
use crate::{modem_driver::ModemDriver, seatrac::ascii_message::parse_response};

static DEFAULT_BAUD_RATE: u32 = 115200;


fn main() {
    let driver_config = modem_driver::DriverConfig::load_from_file("config_driver.json").unwrap_or_else(|e| {
        panic!("Failed to load driver configuration: {}", e);
    });

    let mut modem;
    match driver_config.modem_type.as_str() {
        "seatrac" => {
            modem = seatrac::serial_driver::SerialModem::new(
                &driver_config.port_name,
                driver_config.baud_rate,
            )
            .unwrap_or_else(|e| {
                panic!("Failed to open modem: {}", e);
            });
        }
        _ => panic!("Unsupported modem type: {}", driver_config.modem_type),
        
    }

    modem.configure(driver_config.usbl, DEFAULT_BAUD_RATE, driver_config.beacon_id, driver_config.salinity).unwrap();
    // protocols::send_tdma_message(&mut modem, &[0x01, 0x02, 0x03]).unwrap();
    loop {
        match modem.receive() {
            Ok(data) => {
                // println!("Received data: {:?}", data);
                let (cid, payload, _) = parse_response(data.as_slice()).unwrap();
                println!("CID: {:?}, payload: {:?}", cid, payload);
            }
            Err(e) => {
                // eprintln!("Error receiving data: {}", e);
                continue; // Ignore timeout errors
            }
        }
    }
}