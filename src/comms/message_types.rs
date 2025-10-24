// connected to acknowledgment manager
use std::collections::VecDeque;
use crate::modem_driver::ModemDriver;

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
    
}

/// FieldMsg struct to hold coordinates
/// Represents the positional coordinates of the node
/// fields format: vec![x as u8, y as u8, z as u8]
#[derive(Debug, Clone, Copy)]
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
// pub struct PositionalCoordinates {
//     pub fields: Vec<u8>, //TODO: maybe change to x, y and z. this is how lisa did it
// }

// impl PositionalCoordinates {
    
//     pub fn new(fields:Vec<u8>) -> Self {
//         PositionalCoordinates { fields }
//     }

//     pub fn to_string(&self) -> String {
//         self.fields.iter()
//             .enumerate()  // Get the index and value
//             .map(|(i, value)| format!("f_{}:{}", i, value))  // Create the string f_i:value
//             .collect::<Vec<String>>()  // Collect all strings into a Vec
//             .join(" ")  // Join the values with a space
//     }
// }


// pub fn prepare_new_msg(
//     modem: &mut dyn ModemDriver,
//     t:u64,
//     queue_new_msg: &mut VecDeque<NewMsg>,
// ){
//     let pos = modem.get_position(t).unwrap(); // Get the node's position
//     let pos = PositionalCoordinates::new(pos);
//     let new_msg = NewMsg::new(pos, t);
//     queue_new_msg.push_back(new_msg);
// }