// References to the "SeaTrac Developer Guide (for Beacon firmware version 3.7)"
// are indicated in comments throughout this file.


// Chapter 6. Message field type
// 6.3. Structures

// NOTE: This file uses the same identifier naming as the SeaTrac Developer Guide.
// Many of the types in the firmware documentation use an `_E` suffix to indicate
// an enumeration (for example `BID_E`, `CID_E`). The Rust structs reference the
// enums declared in `seatrac_enums.rs` which intentionally keep those spec names
// so it's straightforward to cross-reference the firmware documentation.


/* Structures (sometimes called Records or Structs) are declarations defining complex data types of physically grouped
variables placed under one name in a block of memory.
For the scope of this document, the members (or fields) of Structures should be assumed to be defined sequentially
in memory, with no additional packing bytes added. */

use crate::seatrac_enums::{AMSGTYPE_E, APAYLOAD_E, CID_E}; // Import enums from seatrac_enums.rs


#[derive(Debug, Clone)]
pub struct ACOMSG_T { // Acoustic Message
    pub msg_dest_id: u8,
    pub msg_src_id: u8,
    pub msg_type: AMSGTYPE_E,
    pub msg_depth: u16,
    pub msg_payload_id: APAYLOAD_E,
    pub msg_payload_len: u8,
    pub msg_payload: [u8; 31],
}

impl ACOMSG_T {
    pub fn new(
        msg_dest_id: u8,
        msg_src_id: u8,
        msg_type: AMSGTYPE_E,
        msg_depth: u16,
        msg_payload_id: APAYLOAD_E,
        msg_payload: &[u8],
    ) -> Self {
        let mut msg_payload_array = [0; 31];
        for (i, &item) in msg_payload.iter().enumerate() {
            msg_payload_array[i] = item;
        }

        Self {
            msg_dest_id,
            msg_src_id,
            msg_type,
            msg_depth,
            msg_payload_id,
            msg_payload_len: msg_payload.len() as u8,
            msg_payload: msg_payload_array,
        }
    }
}

// ACOFIX_T is incomplete, fix later
#[derive(Debug, Clone)]
pub struct ACOFIX_T { // Acoustic Position and Range Fix Summary
    pub dest_id: u8, // BID_E
    pub src_id: u8, // BID_E
    pub flags: u8,
    pub msg_type: AMSGTYPE_E,
    pub attitude_yaw: i16,
    pub attitude_pitch: i16,
    pub attitude_roll: i16,
    pub depth_local: u16,
    pub vos: u16,
    pub rssi: i16,

    // Range
    pub range_count: u32,
    pub range_time: i32,
    pub range_dist: u16,

    // USBL
    pub usbl_channels: u8,
    pub usbl_rssi: Vec<i16>,
    pub usbl_azimuth: i16,
    pub usbl_elevation: i16,
    pub usbl_fit_error: i16,
    
    // Position
    pub position_easting: i16,
    pub position_northing: i16,
    pub position_depth: i16,
}

/* #[derive(Debug, Clone)]
pub struct RangeFields {
    pub range_count: u32,
    pub range_time: i32,
    pub range_dist: u16,
}

#[derive(Debug, Clone)]
pub struct UsblFields {
    pub usbl_channels: u8,
    pub usbl_rssi: Vec<i16>,
    pub usbl_azimuth: i16,
    pub usbl_elevation: i16,
    pub usbl_fit_error: i16,
}

#[derive(Debug, Clone)]
pub struct PositionFields {
    pub position_easting: i16,
    pub position_northing: i16,
    pub position_depth: i16,
} */

/// ACOFIX incomplete end
/// 

// AHRSCAL_T goes here
#[derive(Debug, Clone)]
pub struct AHRSCAL_T { // AHRS Calibration Coefficients
    // Accelerometer calibration values
    pub acc_min_x: i16,
    pub acc_min_y: i16,
    pub acc_min_z: i16,
    pub acc_max_x: i16,
    pub acc_max_y: i16,
    pub acc_max_z: i16,

    // Magnetometer calibration values
    pub mag_valid: bool,
    pub mag_hard_x: f32,
    pub mag_hard_y: f32,
    pub mag_hard_z: f32,
    pub mag_soft_x: f32,
    pub mag_soft_y: f32,
    pub mag_soft_z: f32,
    pub mag_field: f32,
    pub mag_error: f32,

    // Gyroscope offsets
    pub gyro_offset_x: i16,
    pub gyro_offset_y: i16,
    pub gyro_offset_z: i16,
}
// AHRSCAL_T incomplete end


// Chapter 7. Beacon Management Message Definitions
// TODO: add settings_t

// Chapter 8. Acoustic Protocol Stack Message Definitions

#[derive(Debug, Clone)]
pub struct DAT_SEND {
    pub msg_id: CID_E,
    pub dest_id: u8,
    pub msg_type: AMSGTYPE_E,
    pub packet_len: u8,
    pub packet_data: Vec<u8>,
}
pub struct DAT_RECEIVE {
    pub msg_id: CID_E,
    pub aco_fix: Vec<u8>,
    pub ack_flag: bool,
    pub packet_len: u8,
    pub packet_data: Vec<u8>,
    pub local_flag: bool
}

impl DAT_SEND{
    pub fn new(dest_id: u8, packet_data: Vec<u8>) -> DAT_SEND {
        let dat_send_str = DAT_SEND {
            msg_id: CID_E::CID_DAT_SEND,
            dest_id: dest_id,
            msg_type: AMSGTYPE_E::MSG_OWAY,
            packet_len: packet_data.len() as u8,
            packet_data: packet_data,
        };
        dat_send_str
    }
}

impl DAT_RECEIVE{
    pub fn new(received: Vec<u8>) -> DAT_RECEIVE {
        let pac_len = received[18];
        let n_us = usize::try_from(pac_len).unwrap();
        let dat_send_str = DAT_RECEIVE {
            msg_id: CID_E::CID_DAT_RECEIVE,
            aco_fix: received[1..17].to_vec(),
            ack_flag: u8_to_bool(received[17]),
            packet_len: received[18],
            packet_data: received[19..(19+n_us)].to_vec(),
            local_flag: u8_to_bool(received[19+n_us])
        };
        dat_send_str
    }
}

pub fn u8_to_bool(v: u8)->bool{
    match v{
       0 => false,
       255 => true,
       _ => panic!("Invalid bool in u8 {}", v),
    }
}