mod modem_driver;
mod seatrac;
mod protocols;
use crate::modem_driver::ModemDriver;

fn main() {
    let mut modem = seatrac::serial_driver::SerialModem::new("/dev/ttyUSB0", 115200).unwrap();
    modem.configure(115200, 1).unwrap();
    protocols::send_tdma_message(&mut modem, &[0x01, 0x02, 0x03]).unwrap();
}