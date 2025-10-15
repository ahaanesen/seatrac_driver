mod modem_driver;
mod seatrac;
mod protocols;
use crate::{modem_driver::ModemDriver, seatrac::ascii_message::parse_response_message};

static BAUD1: u32 = 115200;
static BAUD2: u32 = 57600;


fn main() {
    println!("Starting Seatrac modem driver...");
    let mut modem = seatrac::serial_driver::SerialModem::new("/dev/ttyUSB0", BAUD1).unwrap_or_else(|e| {
        panic!("Failed to open modem: {}", e);
    });
    println!("serialport opened");
    println!("Configuring modem...");
    modem.configure(BAUD2, 1, 0.0).unwrap();
    // protocols::send_tdma_message(&mut modem, &[0x01, 0x02, 0x03]).unwrap();
    loop {
        match modem.receive() {
            Ok(data) => {
                println!("Received data: {:?}", data);
                let (cid, payload, _, _) = parse_response_message(data.as_slice()).unwrap();
                println!("CID: {:?}, payload: {:?}", cid, payload);
            }
            Err(e) => {
                // eprintln!("Error receiving data: {}", e);
                continue; // Ignore timeout errors
            }
        }
    }
}