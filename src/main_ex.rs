use crate::{
    modem_driver::ModemDriver,
    seatrac::ascii_message::parse_response,
    tdma_scheduler::TdmaScheduler,
    ros_bridge::RosBridge,
};
use rclrs;
use std::time::{Duration, Instant};

static DEFAULT_BAUD_RATE: u32 = 115200;

fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::new(std::env::args())?;
    let ros_bridge = RosBridge::new(&context, "seatrac_modem_node")?;

    let driver_config = modem_driver::DriverConfig::load_from_file("config_driver.json")?;
    let comms_config = comms::tdma_utils::CommsConfig::load_from_file("config_comms.json")?;

    let mut modem = seatrac::serial_driver::SerialModem::new(
        &driver_config.port_name,
        driver_config.baud_rate,
    )?;
    modem.configure(driver_config.usbl, DEFAULT_BAUD_RATE, driver_config.beacon_id, driver_config.salinity)?;

    let scheduler = TdmaScheduler::new(
        comms_config.total_slots,
        comms_config.slot_duration_ms,
    );
    let my_slot = comms_config.slot_id;

    let mut last_beacon = Instant::now();
    let beacon_interval = Duration::from_secs(10); // send a beacon every 10 seconds

    loop {
        let slot = scheduler.current_slot();

        if slot == my_slot {
            // ----------------------------
            // TRANSMIT SLOT
            // ----------------------------
            if last_beacon.elapsed() >= beacon_interval {
                modem.send(b"BEACON")?;
                last_beacon = Instant::now();
            }

            if let Some(msg) = ros_bridge.next_outgoing() {
                modem.send(msg.as_bytes())?;
            }

            // TODO: handle retransmissions / ACKs here
        } else {
            // ----------------------------
            // LISTEN SLOT
            // ----------------------------
            match modem.receive() {
                Ok(data) => {
                    if let Ok((cid, payload, _)) = parse_response(data.as_slice()) {
                        let formatted = format!("CID: {}, Payload: {}", cid, String::from_utf8_lossy(&payload));
                        ros_bridge.publish_incoming(formatted);
                    }
                }
                Err(_e) => {
                    // Likely timeout; just continue listening
                }
            }
        }

        std::thread::sleep(Duration::from_millis(50)); // prevent 100% CPU usage
    }
}
