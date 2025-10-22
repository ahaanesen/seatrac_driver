mod modem_driver;
mod seatrac;
mod comms;
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
        // Can add other modem types here, e.g. "luma100"
        // NB. might want to switch from modem_type being string to enum, but not sure how well it works in the config file
        _ => panic!("Unsupported modem type: {}", driver_config.modem_type),
    }
    modem.configure(driver_config.usbl, DEFAULT_BAUD_RATE, driver_config.beacon_id, driver_config.salinity).unwrap();


    let comms_config = comms::tdma_utils::CommsConfig::load_from_file("config_comms.json").unwrap_or_else(|e| {
        panic!("Failed to load communication configuration: {}", e);
    });

    
    loop {
        // Use TDMA slots to send and receive messages
        // want a ros2 node that publishes received messages and subscribes to outgoing messages (with some extra logic in between)
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