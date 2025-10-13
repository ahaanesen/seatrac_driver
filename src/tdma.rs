// Import necessary modules
use crate::record_message::{FieldMsg};
use std::time::{SystemTime, UNIX_EPOCH, Duration}; // Time management utilities
use serialport::{SerialPort, DataBits, Parity, StopBits, FlowControl}; // Serial port settings and controls
use std::fs::{File, OpenOptions};
use std::io::{self, Write, Read};

// Function to get the total seconds since UNIX epoch
pub fn get_total_seconds() -> u64 {
    let now = SystemTime::now();
    let duration_since_epoch = now.duration_since(UNIX_EPOCH)
        .expect("Time went backwards!");

    duration_since_epoch.as_secs()
}

// Function to calculate the current slot in the TDMA cycle based on cycle and slot durations
pub fn get_current_TDMA_slot(cycle_duration: u64, slot_duration: u64) -> u64 {
    let total_seconds = get_total_seconds();
    let elapsed = total_seconds % cycle_duration; // Calculate time passed within the current cycle
    elapsed / slot_duration // Return current slot number
}
pub fn get_TDMA_slot_after_propag(cycle_duration: u64, slot_duration: u64, propagation_time: u64) -> u64 {
    let total_seconds = get_total_seconds() + propagation_time;
    let elapsed = total_seconds % cycle_duration; // Calculate time passed within the current cycle
    elapsed / slot_duration // Return current slot number
}


struct MsgToSend {
    cid: enums::CID_E,
    payload: Vec<u8>, // DCCL/protobuf encoded
    // Add metadata if needed
}
struct MsgReceived {
    cid: enums::CID_E,
    payload: Vec<u8>,
    // Add metadata if needed
}