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
    pub fn to_bytes(&self, node_id: u8, message_index: i32, acks: Vec<i32>) -> Vec<u8> {
        // Convert the NewMsg struct to bytes for transmission
        // Implement serialization logic here
            let position_string = self.position.to_string();
            let status_string = format!("node_id:{} msg_idx:{} {} t:{} ack:{:?}", 
                node_id,
                message_index,
                position_string,
                self.t,
                acks
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
    pub fn _new_empty () -> Self { // TODO:Remove?
        ReceivedMsg {
            node_id: 0,
            message_index: 0,
            position: PositionalCoordinates::new(0, 0, 0),
            t_sent: 0,
            t_received: 0,
            acks: vec![],
        }
    }

    /// Parse messages such as:
    /// - "node_id:1 msg_idx:42 x:10 y:20 z:30 t:123456 ack:[1,2,3]"
    /// - "node_id: 0 msg_idx: 1 x: 255 y: 4 z: 0 t: 0\n"
    /// - mixed forms like "ack: [1,2,3]" or "ack:[1,2,3]"
    pub fn from_string(data: &str, t_received: u64) -> Result<Self, Box<dyn std::error::Error>> {
        let tokens: Vec<&str> = data.split_whitespace().collect();
        let mut i = 0usize;

        let mut node_id: u8 = 0;
        let mut message_index: i32 = 0;
        let mut x: u8 = 0;
        let mut y: u8 = 0;
        let mut z: u8 = 0;
        let mut t_sent: u64 = 0;
        let mut acks: Vec<i32> = vec![];

        while i < tokens.len() {
            let token = tokens[i];
            // Prefer splitn so values with ":" in them (unlikely here) are handled conservatively
            if let Some(colon_pos) = token.find(':') {
                let key = token[..colon_pos].trim();
                let val_inline = token[colon_pos + 1..].trim();

                if !val_inline.is_empty() {
                    // e.g., "x:255" or "ack:[1,2]"
                    match key {
                        "node_id" => node_id = val_inline.parse()?,
                        "msg_idx" => message_index = val_inline.parse()?,
                        "x" => x = val_inline.parse()?,
                        "y" => y = val_inline.parse()?,
                        "z" => z = val_inline.parse()?,
                        "t" => t_sent = val_inline.parse()?,
                        "ack" => {
                            let ack_str = val_inline.trim_matches(&['[', ']'][..]);
                            for ack in ack_str.split(',') {
                                let a = ack.trim();
                                if !a.is_empty() {
                                    acks.push(a.parse()?);
                                }
                            }
                        }
                        _ => {}
                    }
                } else {
                    // token is like "node_id:" or "node_id:" followed by the value in the next token
                    if i + 1 < tokens.len() {
                        let next = tokens[i + 1].trim();
                        match key {
                            "node_id" => node_id = next.parse()?,
                            "msg_idx" => message_index = next.parse()?,
                            "x" => x = next.parse()?,
                            "y" => y = next.parse()?,
                            "z" => z = next.parse()?,
                            "t" => t_sent = next.parse()?,
                            "ack" => {
                                let ack_str = next.trim_matches(&['[', ']'][..]);
                                for ack in ack_str.split(',') {
                                    let a = ack.trim();
                                    if !a.is_empty() {
                                        acks.push(a.parse()?);
                                    }
                                }
                            }
                            _ => {}
                        }
                        i += 1; // consumed the next token as value
                    }
                }
            } else if token.ends_with(':') {
                // token like "node_id:" (split_whitespace might keep trailing colon)
                let key = token.trim_end_matches(':').trim();
                if i + 1 < tokens.len() {
                    let next = tokens[i + 1].trim();
                    match key {
                        "node_id" => node_id = next.parse()?,
                        "msg_idx" => message_index = next.parse()?,
                        "x" => x = next.parse()?,
                        "y" => y = next.parse()?,
                        "z" => z = next.parse()?,
                        "t" => t_sent = next.parse()?,
                        "ack" => {
                            let ack_str = next.trim_matches(&['[', ']'][..]);
                            for ack in ack_str.split(',') {
                                let a = ack.trim();
                                if !a.is_empty() {
                                    acks.push(a.parse()?);
                                }
                            }
                        }
                        _ => {}
                    }
                    i += 1; // consumed the next token as value
                }
            }
            // otherwise skip unknown token
            i += 1;
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


// Some tests for ReceivedMsg::from_string
#[cfg(test)]
#[test]
fn from_string_parses_with_acks() {
    let data = "node_id:1 msg_idx:42 x:10 y:20 z:30 t:123456 ack:[1,2,3]";
    let t_received = 1_600_000u64;
    let msg = ReceivedMsg::from_string(data, t_received).unwrap();

    let expected = ReceivedMsg {
        node_id: 1u8,
        message_index: 42i32,
        position: PositionalCoordinates::new(10, 20, 30),
        t_sent: 123456u64,
        t_received,
        acks: vec![1, 2, 3],
    };

    assert_eq!(msg, expected);
}

#[test]
fn from_string_parses_empty_ack_and_ignores_bad_parts() {
    // new_msg.rs defines PositionalCoordinates fields as u8, so x must be in 0..=255.
    // Replace negative value with a valid u8 (e.g., 255) in the test input and expected struct.
    let data = "node_id:2 msg_idx:7 x:255 y:0 z:5 t:42 ack:[] garbage_key:foo";
    let t_received = 2_000u64;
    let msg = ReceivedMsg::from_string(data, t_received).unwrap();

    let expected = ReceivedMsg {
        node_id: 2u8,
        message_index: 7i32,
        position: PositionalCoordinates::new(255, 0, 5),
        t_sent: 42u64,
        t_received,
        acks: vec![],
    };

    assert_eq!(msg, expected);
}
#[test]
fn from_string_parses_spaced_input() {
    // new test for the spaced form you mentioned:
    let data = "node_id: 0 msg_idx: 1 x: 255 y: 4 z: 0 t: 0\n";
    let t_received = 3_000u64;
    let msg = ReceivedMsg::from_string(data, t_received).unwrap();

    let expected = ReceivedMsg {
        node_id: 0u8,
        message_index: 1i32,
        position: PositionalCoordinates::new(255, 4, 0),
        t_sent: 0u64,
        t_received,
        acks: vec![],
    };

    assert_eq!(msg, expected);
}