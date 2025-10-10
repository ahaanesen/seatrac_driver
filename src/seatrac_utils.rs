// Import required modules for serial communication and protocol handling
// use crate::modem::identifiers::CommandIdentificationCode; // Enum for command identification codes
// use crate::modem::record::DAT_SEND; // Structure for DAT messages
// use serialport::SerialPort; // Trait for serial port functionality
use std::io::{self}; // I/O utilities
use std::error::Error; // Error handling utilities


/// Calculates CRC16 checksum for a given buffer
/// 
/// # Arguments
/// * `buf` - The byte buffer for which the checksum is calculated
/// * `len` - The length of the buffer
/// 
/// # Returns
/// * The computed CRC16 checksum
fn calc_crc16(buf: &[u8], len: u16) -> u16 {
    let crc_poly = 0xA001; // CRC polynomial for checksum calculation
    let mut crc: u16 = 0; // Initial CRC value
    for b in 0..len {
        let mut v: u8 = buf[b as usize]; // Current byte to process
        for _ in 0..8 {
            // Process each bit of the byte
            if (v as u16 & 0x01) ^ (crc & 0x01) != 0 {
                crc >>= 1; // Shift CRC right
                crc ^= crc_poly; // XOR with the polynomial if bits differ
            } else {
                crc >>= 1; // Just shift if bits are the same
            }
            v >>= 1; // Shift byte to the right
        }
    }
    crc // Return computed CRC16
}


/// Converts a byte array to a hexadecimal string
/// 
/// # Arguments
/// * `buf` - The byte array to convert
/// 
/// # Returns
/// * A hexadecimal string representation of the byte array
fn bytes_to_hex(buf: &[u8]) -> String {
    hex::encode(buf) // Use hex crate to encode bytes to hex
}

// From 5.2 Message Format
// pub fn build_command_msg
// pub fn parse_response_msg