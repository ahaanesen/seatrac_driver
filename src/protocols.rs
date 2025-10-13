use crate::modem_driver::ModemDriver;

pub fn send_tdma_message<D: ModemDriver>(
    modem: &mut D, 
    packet_data: &[u8]
) -> Result<(), Box<dyn std::error::Error>> {
    modem.send(packet_data)?;
    Ok(())
}

pub fn listen_for_messages<D: ModemDriver>(
    modem: &mut D,
    // ... other args
) -> Result<(), Box<dyn std::error::Error>> {
    let data = modem.receive()?;
    // ... process data as before
    Ok(())
}