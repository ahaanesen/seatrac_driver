// connected to acknowledgment manager
use crate::{comms::dccl::encode_input};

pub struct NewMsg {
    pub position: PositionalCoordinates,
    pub t: u64
}

impl NewMsg{
    pub fn new(fields: PositionalCoordinates, t: u64) -> Self{
        NewMsg{
            position: fields,
            t
        }
    }

    /// Convert the NewMsg struct to bytes for transmission with dccl encoding
    pub fn to_bytes(&self, node_id: u8, message_index: i32) -> Vec<u8> {
        // Convert the NewMsg struct to bytes for transmission
        // Implement serialization logic here
            let position_string = self.position.to_string();
            let status_string = format!("node_id:{} msg_idx:{} {} t:{}", 
                node_id,
                message_index,
                position_string,
                self.t
            );
            // println!("Encoding message: {}", status_string);
            let packet_data = encode_input(&status_string).expect("Encoding failed");
            // println!("Encoded packet data: {:?}", packet_data);
            packet_data
    }
}
/// FieldMsg struct to hold coordinates
/// Represents the positional coordinates of the node
/// fields format: vec![x as u8, y as u8, z as u8]
#[derive(Debug, Clone, PartialEq, Copy)]
pub struct PositionalCoordinates {
    pub x: u8,
    pub y: u8,
    pub z: u8,
}

impl PositionalCoordinates {
    pub fn new(x: u8, y: u8, z: u8) -> Self {
        PositionalCoordinates { x, y, z }
    }

    pub fn to_string(&self) -> String {
        format!("x:{} y:{} z:{}", self.x, self.y, self.z)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReceivedMsg {
    pub node_id: u8,
    pub message_index: i32,
    pub position: PositionalCoordinates,
    pub t_sent: u64,
    pub t_received: u64,
    pub acks: Vec<i32>,
}

impl ReceivedMsg {
    pub fn new_empty () -> Self {
        ReceivedMsg {
            node_id: 0,
            message_index: 0,
            position: PositionalCoordinates::new(0, 0, 0),
            t_sent: 0,
            t_received: 0,
            acks: vec![],
        }
    }
    pub fn from_string(data: &str, t_received: u64) -> Result<Self, Box<dyn std::error::Error>> {
        let parts: Vec<&str> = data.split_whitespace().collect();
        let mut node_id = 0;
        let mut message_index = 0;
        let mut x = 0;
        let mut y = 0;
        let mut z = 0;
        let mut t_sent = 0;
        let mut acks: Vec<i32> = vec![];

        for part in parts {
            let kv: Vec<&str> = part.split(':').collect();
            if kv.len() != 2 {
                continue;
            }
            match kv[0] {
                "node_id" => node_id = kv[1].parse()?,
                "msg_idx" => message_index = kv[1].parse()?,
                "x" => x = kv[1].parse()?,
                "y" => y = kv[1].parse()?,
                "z" => z = kv[1].parse()?,
                "t" => t_sent = kv[1].parse()?,
                "ack" => {
                    let ack_str = kv[1].trim_matches(&['[', ']'][..]);
                    for ack in ack_str.split(',') {
                        if !ack.trim().is_empty() {
                            acks.push(ack.trim().parse()?);
                        }
                    }
                }
                _ => (),
            }
        }

        Ok(ReceivedMsg {
            node_id,
            message_index,
            position: PositionalCoordinates::new(x, y, z),
            t_sent,
            t_received,
            acks,
        })
    }
    pub fn to_string(&self) -> String {
        let position_string = self.position.to_string();
        format!(
            "node_id:{}, msg_idx:{}, pos:{}, t_sent:{}, t_received:{}, ack:{:?}",
            self.node_id,
            self.message_index,
            position_string,
            self.t_sent,
            self.t_received,
            self.acks
        )
    }
}


#[derive(Debug, Clone, PartialEq)]
pub struct UsblData{
    pub channels: u8,
    pub rssi: Vec<i16>,
    pub azimuth: i16,
    pub elevation: i16,
    pub fit_error: i16,
}

impl UsblData {
    pub fn new(channels: u8, rssi: Vec<i16>, azimuth: i16, elevation: i16, fit_error: i16) -> Self {
        UsblData {
            channels,
            rssi,
            azimuth,
            elevation,
            fit_error,
        }
    }
}