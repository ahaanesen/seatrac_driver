use std::collections::{HashMap, VecDeque};
use log::info;
use std::mem;
use crate::comms::{message_types::{NewMsg, PositionalCoordinates}, tdma_utils::AcousticMessage};

/// Struct to hold acknowledgment information for received messages
pub struct MsgReceivedAck {
    pub node_id: i32, // ID of the node that sent the message
    pub nmsg: i32,     // Message ID
}

impl MsgReceivedAck {
    pub fn new(node_id: i32, nmsg: i32) -> Self {
        MsgReceivedAck { node_id, nmsg }
    }
}

/// Creates an acknowledgment message from a vector of received acknowledgments
pub fn create_ack(msgs: &mut Vec<MsgReceivedAck>) -> Vec<i32> {
    let mut result = Vec::new();
    for msg in msgs {
        result.push(msg.node_id); // Add node ID
        result.push(msg.nmsg);     // Add message ID
    }
    result
}

/// Converts a vector of acknowledgment bytes into MsgReceivedAck structs
pub fn received_ack_from_list(ack: Vec<i32>) -> Vec<MsgReceivedAck> {
    let mut result = Vec::new();
    
    // Iterate through the acknowledgment byte pairs (node_id, nmsg)
    let mut i = 0;
    while i < ack.len() {
        if i + 1 < ack.len() {
            let node_id = ack[i];
            let nmsg = ack[i + 1];
            result.push(MsgReceivedAck::new(node_id, nmsg)); // Create MsgReceivedAck for each pair
        }
        i += 2; // Move to the next pair
    }
    
    result
}

// Handle acknowledgments from received messages
// fn handle_acknowledgments(
//     result: &mut Vec<i32>, 
//     received: &mut Vec<MsgReceivedAck>, 
//     sent_manage: &mut SentMsgManager, 
//     num_beacons: i32, 
//     initial_time: u64,
//     num_field: usize
// ) {
//     // Additional logic to handle acknowledgments would go here
//     if result.len() >= (num_field+3).try_into().unwrap() {
//         let ack = received_ack_from_list(result[num_field+3..].to_vec()); // Parse acknowledgments
//         for a in ack {
//             let log_entry = format!("Received ACK from node {} of {} message\n", a.node_id, a.nmsg);
//             println!("{}", log_entry);
//             info!("{}", log_entry);
//             sent_manage.add_nodeid_to_message(a.nmsg, a.node_id, num_beacons); // Update sent message manager with acknowledgments
//         }
//         received.push(MsgReceivedAck::new(result[0], result[1])); // Store received acknowledgment
//     }else{
//         print!("No ack\n");
//     }
// }


/// Struct to hold information about sent messages and their acknowledgments
#[derive(Clone, Debug)]
pub struct SentMessage {
    pub position: PositionalCoordinates,
    pub t: u64,
    pub nodes_acked: Vec<i32>, // IDs of nodes that acknowledged this message
}

impl SentMessage {
    pub fn new(position: PositionalCoordinates, t: u64) -> Self {
        SentMessage {
            position,
            t,
            nodes_acked: Vec::new(),
        }
    }

    /// Adds a node ID to nodoid_received; returns true if the count matches a configured constant
    pub fn add_node_ack(&mut self, node_id: i32, num_beacons: i32) -> bool {
        if !self.nodes_acked.contains(&node_id) {
            self.nodes_acked.push(node_id); // Add new node ID if not already present
        }
        // Return true if the number of acknowledged nodes matches a configured constant
        self.nodes_acked.len() as i32 == (num_beacons - 1) // Replace with config constant if needed
    }
}

/// Struct to manage all sent messages awaiting acknowledgment
pub struct SentMsgManager {
    messages: HashMap<i32, SentMessage>, // Maps message ID to SentMessage
    pub message_index: i32,
    received_acks: Vec<MsgReceivedAck>,
    pub queue_new_msg: VecDeque<NewMsg>,
    listened_msg: Vec<Vec<i32>>,
    pub wait_time: u64,
}

impl SentMsgManager {
    pub fn new() -> Self {
        SentMsgManager {
            messages: HashMap::new(),
            message_index: 0,
            received_acks: vec![],
            queue_new_msg: VecDeque::new(),
            listened_msg: vec![],
            wait_time: 0,
        }
    }
    pub fn initialize_ack_slot(&mut self) -> Vec<i32> {
        self.wait_time = 0;
        let ack_msg = create_ack(&mut self.received_acks);
        self.received_acks.clear();
        // TODO: more here?
        ack_msg
    }

    //LISA
    /// Checks if there are any sent messages awaiting acknowledgment
    pub fn is_empty(&self) -> bool {
        self.messages.is_empty()
    }

    /// Adds a new message to the manager
    pub fn add_message(&mut self, key: i32, message: SentMessage) {
        self.messages.insert(key, message);
    }

    /// Removes a message from the manager
    fn remove_message(&mut self, key: i32) {
        self.messages.remove(&key);
    }

    /// Adds a node ID to the acknowledgment list of a specific message
    pub fn add_nodeid_to_message(&mut self, key: i32, node_id: i32, num_beacons: i32) -> bool {
        if let Some(message) = self.messages.get_mut(&key) {
            let to_remove = message.add_node_ack(node_id, num_beacons); // Update acknowledgment
            if to_remove {
                self.remove_message(key); // Remove message if all acknowledgments received
                println!("Message read by ALL");
            }
            true
        } else {
            false // Return false if the message key does not exist
        }
    }

    /// Lists currently stored messages with their details
    pub fn list_messages(&self) -> Vec<(i32, SentMessage)> {
        // Convert the HashMap into a vector of (key, message) pairs
        let mut messages_vec: Vec<(&i32, &SentMessage)> = self.messages.iter().collect();

        // Sort the vector in descending order of keys
        messages_vec.sort_by(|a, b| b.0.cmp(a.0));

        // Transform the sorted vector into the desired format by cloning the messages
        messages_vec
            .into_iter()
            .map(|(key, msg)| (*key, msg.clone()))
            .collect::<Vec<(i32, SentMessage)>>()
    }

    // Handle acknowledgments from received messages
    fn handle_acknowledgments(
        &mut self,
        result: &mut Vec<i32>, 
        received: &mut Vec<MsgReceivedAck>, 
        sent_manage: &mut SentMsgManager, 
        num_beacons: i32, 
        initial_time: u64,
        num_field: usize
    ) {
        for result in self.listened_msg.iter() {
            println!("Listened msg: {:?}", result);
            // Additional logic to handle listened messages would go here
            if result.len() >= (num_field+3).try_into().unwrap() {
                let ack = received_ack_from_list(result[num_field+3..].to_vec()); // Parse acknowledgments
                for a in ack {
                    let log_entry = format!("Received ACK from node {} of {} message\n", a.node_id, a.nmsg);
                    println!("{}", log_entry);
                    info!("{}", log_entry);
                    sent_manage.add_nodeid_to_message(a.nmsg, a.node_id, num_beacons); // Update sent message manager with acknowledgments
                }
                self.received_acks.push(MsgReceivedAck::new(result[0], result[1])); // Store received acknowledgment
            }else{
                print!("No ack\n");
            }
        }

    }
    
}

