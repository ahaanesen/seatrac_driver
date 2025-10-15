// References to the "SeaTrac Developer Guide (for Beacon firmware version 3.7)"
// are indicated in comments throughout this file.


// Chapter 6. Message field type
// 6.3. Structures

// NOTE: This file uses the same identifier naming as the SeaTrac Developer Guide.
// Many of the types in the firmware documentation use an `_E` suffix to indicate
// an enumeration (for example `BID_E`, `CID_E`). The Rust structs reference the
// enums declared in `seatrac_enums.rs` which intentionally keep those spec names
// so it's straightforward to cross-reference the firmware documentation.

#![allow(non_camel_case_types)]


/* Structures (sometimes called Records or Structs) are declarations defining complex data types of physically grouped
variables placed under one name in a block of memory.
For the scope of this document, the members (or fields) of Structures should be assumed to be defined sequentially
in memory, with no additional packing bytes added. */

use crate::seatrac::enums::{self, AMSGTYPE_E, APAYLOAD_E, BAUDRATE_E, CID_E, CST_E, STATUSMODE_E}; // Import enums from seatrac_enums.rs


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

impl ACOFIX_T {
    pub fn new(
        dest_id: u8,
        src_id: u8,
        flags: u8,
        msg_type: AMSGTYPE_E,
        attitude_yaw: i16,
        attitude_pitch: i16,
        attitude_roll: i16,
        depth_local: u16,
        vos: u16,
        rssi: i16,
        range_count: u32,
        range_time: i32,
        range_dist: u16,
        usbl_channels: u8,
        usbl_rssi: Vec<i16>,
        usbl_azimuth: i16,
        usbl_elevation: i16,
        usbl_fit_error: i16,
        position_easting: i16,
        position_northing: i16,
        position_depth: i16,
    ) -> Self {
        Self {
            dest_id,
            src_id,
            flags,
            msg_type,
            attitude_yaw,
            attitude_pitch,
            attitude_roll,
            depth_local,
            vos,
            rssi,
            range_count,
            range_time,
            range_dist,
            usbl_channels,
            usbl_rssi,
            usbl_azimuth,
            usbl_elevation,
            usbl_fit_error,
            position_easting,
            position_northing,
            position_depth,
        }
    }
}
/// ACOFIX incomplete end
/// 

// AHRSCAL_T goes here
#[derive(Debug, Clone, PartialEq)]
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

impl SETTINGS_T {
    ///ulti-byte integers and IEEE-754 little-endian
    /// for f32 values. Returns an e Parse SETTINGS_T from a byte vector.
    ///
    /// The function reads fields in the order declared in the struct and expects
    /// little-endian encoding for mrror if the buffer is too short or if any
    /// enum conversion fails.
    pub fn from_hex(data: Vec<u8>) -> Result<Self, Box<dyn std::error::Error>> {
        use std::convert::TryInto;

        let buf = data.as_slice();
        let mut i: usize = 0;

        fn ensure(buf: &[u8], idx: usize, need: usize) -> Result<(), Box<dyn std::error::Error>> {
            if idx + need > buf.len() {
                Err(format!("buffer too short at offset {} need {} remaining {}", idx, need, buf.len() - idx).into())
            } else {
                Ok(())
            }
        }

        fn read_u8(buf: &[u8], idx: &mut usize) -> Result<u8, Box<dyn std::error::Error>> {
            ensure(buf, *idx, 1)?;
            let v = buf[*idx]; *idx += 1; Ok(v)
        }

        fn read_i16(buf: &[u8], idx: &mut usize) -> Result<i16, Box<dyn std::error::Error>> {
            ensure(buf, *idx, 2)?;
            let a: [u8;2] = buf[*idx..*idx+2].try_into().map_err(|_| "slice to array failed")?; *idx += 2; Ok(i16::from_le_bytes(a))
        }

        fn read_u16(buf: &[u8], idx: &mut usize) -> Result<u16, Box<dyn std::error::Error>> {
            ensure(buf, *idx, 2)?;
            let a: [u8;2] = buf[*idx..*idx+2].try_into().map_err(|_| "slice to array failed")?; *idx += 2; Ok(u16::from_le_bytes(a))
        }

        fn read_i32(buf: &[u8], idx: &mut usize) -> Result<i32, Box<dyn std::error::Error>> {
            ensure(buf, *idx, 4)?;
            let a: [u8;4] = buf[*idx..*idx+4].try_into().map_err(|_| "slice to array failed")?; *idx += 4; Ok(i32::from_le_bytes(a))
        }

        fn read_f32(buf: &[u8], idx: &mut usize) -> Result<f32, Box<dyn std::error::Error>> {
            ensure(buf, *idx, 4)?;
            let a: [u8;4] = buf[*idx..*idx+4].try_into().map_err(|_| "slice to array failed")?; *idx += 4; Ok(f32::from_le_bytes(a))
        }

        fn read_array<const N: usize>(buf: &[u8], idx: &mut usize) -> Result<[u8; N], Box<dyn std::error::Error>> {
            ensure(buf, *idx, N)?;
            let a: [u8; N] = buf[*idx..*idx+N].try_into().map_err(|_| "slice to array failed")?; *idx += N; Ok(a)
        }

        // Start parsing fields in order
        let status_flags = read_u8(buf, &mut i)?;
        // Bits[2:0] = STATUS_MODE; Bits[7:3] reserved
        let status_mode_raw = status_flags & 0x07;
        let status_mode = match enums::STATUSMODE_E::from_u8(status_mode_raw) {
            Some(m) => m,
            None => return Err(format!("unknown STATUSMODE_E code {:#x}", status_mode_raw).into()),
        };
        let status_output_raw = read_u8(buf, &mut i)?;
        let status_output = STATUS_BITS_T::from_bits_truncate(status_output_raw);

        let uart_main_baud_raw = read_u8(buf, &mut i)?;
        let uart_main_baud = match BAUDRATE_E::from_u8(uart_main_baud_raw) {
            Some(b) => b,
            None => return Err(format!("unknown uart_main_baud code {:#x}", uart_main_baud_raw).into()),
        };

        let uart_aux_baud_raw = read_u8(buf, &mut i)?;
        let uart_aux_baud = match BAUDRATE_E::from_u8(uart_aux_baud_raw) {
            Some(b) => b,
            None => return Err(format!("unknown uart_aux_baud code {:#x}", uart_aux_baud_raw).into()),
        };

        // MAC address (6 bytes)
        let mac_arr = read_array::<6>(buf, &mut i)?;
        let net_mac_addr = MACADDR_T::from_bytes(mac_arr);

        // IP addresses (each 4 bytes)
        let ip_arr = read_array::<4>(buf, &mut i)?; let net_ip_addr = IPADDR_T::from_bytes(ip_arr);
        let ip_arr = read_array::<4>(buf, &mut i)?; let net_ip_subnet = IPADDR_T::from_bytes(ip_arr);
        let ip_arr = read_array::<4>(buf, &mut i)?; let net_ip_gateway = IPADDR_T::from_bytes(ip_arr);
        let ip_arr = read_array::<4>(buf, &mut i)?; let net_ip_dns = IPADDR_T::from_bytes(ip_arr);

        let net_tcp_port = read_u16(buf, &mut i)?;
        let env_flags = read_u8(buf, &mut i)?;
        let env_pressure_ofs = read_i32(buf, &mut i)?;
        let env_salinity = read_u16(buf, &mut i)?;
        let env_vos = read_u16(buf, &mut i)?;
        let ahrs_flags = read_u8(buf, &mut i)?;

        // AHRSCAL_T fields
        let acc_min_x = read_i16(buf, &mut i)?;
        let acc_min_y = read_i16(buf, &mut i)?;
        let acc_min_z = read_i16(buf, &mut i)?;
        let acc_max_x = read_i16(buf, &mut i)?;
        let acc_max_y = read_i16(buf, &mut i)?;
        let acc_max_z = read_i16(buf, &mut i)?;
        let mag_valid = { let v = read_u8(buf, &mut i)?; v != 0 };
        let mag_hard_x = read_f32(buf, &mut i)?;
        let mag_hard_y = read_f32(buf, &mut i)?;
        let mag_hard_z = read_f32(buf, &mut i)?;
        let mag_soft_x = read_f32(buf, &mut i)?;
        let mag_soft_y = read_f32(buf, &mut i)?;
        let mag_soft_z = read_f32(buf, &mut i)?;
        let mag_field = read_f32(buf, &mut i)?;
        let mag_error = read_f32(buf, &mut i)?;
        let gyro_offset_x = read_i16(buf, &mut i)?;
        let gyro_offset_y = read_i16(buf, &mut i)?;
        let gyro_offset_z = read_i16(buf, &mut i)?;

        let ahrs_cal = AHRSCAL_T {
            acc_min_x,
            acc_min_y,
            acc_min_z,
            acc_max_x,
            acc_max_y,
            acc_max_z,
            mag_valid,
            mag_hard_x,
            mag_hard_y,
            mag_hard_z,
            mag_soft_x,
            mag_soft_y,
            mag_soft_z,
            mag_field,
            mag_error,
            gyro_offset_x,
            gyro_offset_y,
            gyro_offset_z,
        };

        let ahrs_yaw_offset = read_u16(buf, &mut i)?;
        let ahrs_pitch_offset = read_u16(buf, &mut i)?;
        let ahrs_roll_offset = read_u16(buf, &mut i)?;
        let xcvr_flags = read_u8(buf, &mut i)?;

        let xcvr_beacon_id_raw = read_u8(buf, &mut i)?;
        let xcvr_beacon_id = match enums::BID_E::from_u8(xcvr_beacon_id_raw) {
            Some(b) => b,
            None => return Err(format!("unknown BID_E code {:#x}", xcvr_beacon_id_raw).into()),
        };

        let xcvr_range_tmo = read_u16(buf, &mut i)?;
        let xcvr_resp_time = read_u16(buf, &mut i)?;
        let xcvr_yaw = read_u16(buf, &mut i)?;
        let xcvr_pitch = read_u16(buf, &mut i)?;
        let xcvr_roll = read_u16(buf, &mut i)?;
        let xcvr_posflt_vel = read_u8(buf, &mut i)?;
        let xcvr_posflt_ang = read_u8(buf, &mut i)?;
        let xcvr_posflt_tmo = read_u8(buf, &mut i)?;

        Ok(SETTINGS_T {
            status_flags,
            status_mode,
            status_output,
            uart_main_baud,
            uart_aux_baud,
            net_mac_addr,
            net_ip_addr,
            net_ip_subnet,
            net_ip_gateway,
            net_ip_dns,
            net_tcp_port,
            env_flags,
            env_pressure_ofs,
            env_salinity,
            env_vos,
            ahrs_flags,
            ahrs_cal,
            ahrs_yaw_offset,
            ahrs_pitch_offset,
            ahrs_roll_offset,
            xcvr_flags,
            xcvr_beacon_id,
            xcvr_range_tmo,
            xcvr_resp_time,
            xcvr_yaw,
            xcvr_pitch,
            xcvr_roll,
            xcvr_posflt_vel,
            xcvr_posflt_ang,
            xcvr_posflt_tmo,
        })
    }

    /// Serialize SETTINGS_T into a Vec<u8> using the same layout as from_bytes.
    pub fn to_bytes(&self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let mut v: Vec<u8> = Vec::new();
        // Merge status_mode into the low 3 bits of status_flags when serializing.
        let mut status_flags_out = self.status_flags & 0xF8; // preserve reserved upper bits
        status_flags_out |= self.status_mode.to_u8() & 0x07;
        v.push(status_flags_out);
        v.push(self.status_output.bits());
        v.push(self.uart_main_baud.to_u8());
        v.push(self.uart_aux_baud.to_u8());
        v.extend_from_slice(&self.net_mac_addr.to_bytes());
        v.extend_from_slice(&self.net_ip_addr.to_bytes());
        v.extend_from_slice(&self.net_ip_subnet.to_bytes());
        v.extend_from_slice(&self.net_ip_gateway.to_bytes());
        v.extend_from_slice(&self.net_ip_dns.to_bytes());
        v.extend_from_slice(&self.net_tcp_port.to_le_bytes());
        v.push(self.env_flags);
        v.extend_from_slice(&self.env_pressure_ofs.to_le_bytes());
        v.extend_from_slice(&self.env_salinity.to_le_bytes());
        v.extend_from_slice(&self.env_vos.to_le_bytes());
        v.push(self.ahrs_flags);

        // AHRSCAL_T
        v.extend_from_slice(&self.ahrs_cal.acc_min_x.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.acc_min_y.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.acc_min_z.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.acc_max_x.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.acc_max_y.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.acc_max_z.to_le_bytes());
        v.push(self.ahrs_cal.mag_valid as u8);
        v.extend_from_slice(&self.ahrs_cal.mag_hard_x.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.mag_hard_y.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.mag_hard_z.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.mag_soft_x.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.mag_soft_y.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.mag_soft_z.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.mag_field.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.mag_error.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.gyro_offset_x.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.gyro_offset_y.to_le_bytes());
        v.extend_from_slice(&self.ahrs_cal.gyro_offset_z.to_le_bytes());

        v.extend_from_slice(&self.ahrs_yaw_offset.to_le_bytes());
        v.extend_from_slice(&self.ahrs_pitch_offset.to_le_bytes());
        v.extend_from_slice(&self.ahrs_roll_offset.to_le_bytes());
        v.push(self.xcvr_flags);
        v.push(self.xcvr_beacon_id.to_u8());
        v.extend_from_slice(&self.xcvr_range_tmo.to_le_bytes());
        v.extend_from_slice(&self.xcvr_resp_time.to_le_bytes());
        v.extend_from_slice(&self.xcvr_yaw.to_le_bytes());
        v.extend_from_slice(&self.xcvr_pitch.to_le_bytes());
        v.extend_from_slice(&self.xcvr_roll.to_le_bytes());
        v.push(self.xcvr_posflt_vel);
        v.push(self.xcvr_posflt_ang);
        v.push(self.xcvr_posflt_tmo);

        Ok(v)
    }
}


#[derive(Debug, Clone, PartialEq)]
pub struct SETTINGS_T {
    pub status_flags: u8,                // STATUS_FLAGS
    pub status_mode: enums::STATUSMODE_E, // STATUS_MODE (bits[2:0] of status_flags)
    pub status_output: STATUS_BITS_T,    // STATUS_OUTPUT
    pub uart_main_baud: BAUDRATE_E,      // UART_MAIN_BAUD - only applied after reboot
    pub uart_aux_baud: BAUDRATE_E,       // UART_AUX_BAUD (reserved) - only applied after reboot
    pub net_mac_addr: MACADDR_T,         // NET_MAC_ADDR - only applied after reboot
    pub net_ip_addr: IPADDR_T,           // NET_IP_ADDR - only applied after reboot
    pub net_ip_subnet: IPADDR_T,         // NET_IP_SUBNET - only applied after reboot
    pub net_ip_gateway: IPADDR_T,        // NET_IP_GATEWAY - only applied after reboot
    pub net_ip_dns: IPADDR_T,            // NET_IP_DNS - only applied after reboot
    pub net_tcp_port: u16,               // NET_TCP_PORT - only applied after reboot
    pub env_flags: u8,                   // ENV_FLAGS - Bit[1]=AUTO_PRESSURE_OFS, Bit[0]=AUTO_VOS
    pub env_pressure_ofs: i32,
    pub env_salinity: u16,
    pub env_vos: u16,
    pub ahrs_flags: u8,                  // AHRS_FLAGS - Bits[7:1] = RESERVED, Bit[0]=AUTO_MAG_CAL
    pub ahrs_cal: AHRSCAL_T,
    pub ahrs_yaw_offset: u16,
    pub ahrs_pitch_offset: u16,
    pub ahrs_roll_offset: u16,
    pub xcvr_flags: u8,                 // XCVR_FLAGS - 
                                        /*  Bit[7] = XCVR_DIAG_MSGS,
                                            Bit[6] = XCVR_FIX_MSGS,
                                            Bit[5] = XCVR_USBL_MSGS,
                                            Bits[4:3] = XCVR_TX_MSGCTRL,
                                            Bit[2] = XCVR_BASELINES_MSGS,
                                            Bit[1] = XCVR_POSFLT_ENABLE,
                                            Bit[0] = USBL_USE_AHRS
                                        */
    pub xcvr_beacon_id: enums::BID_E,   // Valid values are from 1 to 100 (0x1 to 0xF). A value of 0 (BEACON_ALL) should not be used.
    pub xcvr_range_tmo: u16,            // Values are encoded in metres. Valid values are in the range 100m to 3000m.
    pub xcvr_resp_time: u16,            // Values are encoded in milliseconds. Valid values are in the range 10ms to 1000ms.
    pub xcvr_yaw: u16,                  // Values are encoded as deci-degrees, so divide the value by 10 to obtain a value in degrees. Valid values are cyclically wrapped to the range 0° to 359.9°.
    pub xcvr_pitch: u16,                // Values are encoded as deci-degrees, so divide the value by 10 to obtain a value in degrees. Valid values are in the range -90.0° to +90.0°.
    pub xcvr_roll: u16,                 // Values are encoded as deci-degrees, so divide the value by 10 to obtain a value in degrees. Valid values are in the range -180.0° to +180.0°.
    pub xcvr_posflt_vel: u8,
    pub xcvr_posflt_ang: u8,
    pub xcvr_posflt_tmo: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MACADDR_T {
    pub addr: u64,
}

impl MACADDR_T {
    /// Create from bytes in reverse order: [byte0, byte1, ..., byte5]
    pub fn from_bytes(bytes: [u8; 6]) -> Self {
        let mut addr: u64 = 0;
        for i in 0..6 {
            addr |= (bytes[i] as u64) << (8 * i);
        }
        MACADDR_T { addr }
    }

    /// Convert to bytes in reverse order: [byte0, byte1, ..., byte5]
    pub fn to_bytes(self) -> [u8; 6] {
        let mut bytes = [0u8; 6];
        for i in 0..6 {
            bytes[i] = ((self.addr >> (8 * i)) & 0xFF) as u8;
        }
        bytes
    }

/*     /// Format as human-readable MAC string: "XX-XX-XX-XX-XX-XX"
    pub fn to_string(self) -> String {
        let b = self.to_bytes();
        format!("{:02X}-{:02X}-{:02X}-{:02X}-{:02X}-{:02X}", b[5], b[4], b[3], b[2], b[1], b[0])
    } */
}


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IPADDR_T {
    pub addr: u32,
}

impl IPADDR_T {
    /// Create from bytes in reverse order: [byte0, byte1, byte2, byte3]
    pub fn from_bytes(bytes: [u8; 4]) -> Self {
        let addr = (bytes[3] as u32) << 24
                 | (bytes[2] as u32) << 16
                 | (bytes[1] as u32) << 8
                 | (bytes[0] as u32);
        IPADDR_T { addr }
    }

    /// Convert to bytes in reverse order: [byte0, byte1, byte2, byte3]
    pub fn to_bytes(self) -> [u8; 4] {
        [
            (self.addr & 0x000000FF) as u8,
            ((self.addr & 0x0000FF00) >> 8) as u8,
            ((self.addr & 0x00FF0000) >> 16) as u8,
            ((self.addr & 0xFF000000) >> 24) as u8,
        ]
    }
}

bitflags::bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct STATUS_BITS_T: u8 {
        const ENVIRONMENT     = 0b0000_0001; // Bit 0
        const ATTITUDE        = 0b0000_0010; // Bit 1
        const MAG_CAL         = 0b0000_0100; // Bit 2
        const ACC_CAL         = 0b0000_1000; // Bit 3
        const AHRS_RAW_DATA   = 0b0001_0000; // Bit 4
        const AHRS_COMP_DATA  = 0b0010_0000; // Bit 5
        // Bits 6 and 7 are reserved and treated as 0
    }
}

/* 
Example usage of STATUS_BITS_T:
fn main() {
    // Combine flags
    let status = STATUS_BITS_T::ENVIRONMENT | STATUS_BITS_T::ATTITUDE | STATUS_BITS_T::AHRS_COMP_DATA;

    // Check if a flag is set
    if status.contains(STATUS_BITS_T::ATTITUDE) {
        println!("Attitude data will be included.");
    }

    // Print raw value
    println!("Raw status byte: 0x{:02X}", status.bits());
}
 */