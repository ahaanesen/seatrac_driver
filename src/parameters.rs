use serde::Deserialize;

#[derive(Debug, Deserialize, Default)]
pub struct ROS2Parameters {
    pub acoustic_comm_send_topic_name: String,
    pub acoustic_comm_receive_topic_name: String,
    pub usbl_topic_name: String,
    // pub range_topic_name: String,
    // pub depth_topic_name: String,
}


#[derive(Debug, Deserialize, Default, Clone)]
pub struct ModemParameters {
    pub port_name: String,
    pub baud_rate: u32,
    pub modem_type: String,
    pub usbl: bool,
    pub beacon_id: u8,
    pub salinity: u16,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct CommsParameters {
    pub tdma_slot_duration_s: u8,
    pub network_participant_count: u8,
    pub agent_id: u8,
    pub msg_propagation_speed: u64,
}
