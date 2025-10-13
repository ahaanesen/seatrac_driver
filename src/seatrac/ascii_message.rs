// Import required modules for serial communication and protocol handling
// use crate::modem::identifiers::CommandIdentificationCode; // Enum for command identification codes
// use crate::modem::record::DAT_SEND; // Structure for DAT messages
// use serialport::SerialPort; // Trait for serial port functionality
use crate::seatrac::enums::{self}; // Replace with actual path
use hex;
use std::error::Error;


const CRC_POLY: u16 = 0xA001; // CRC polynomial for checksum calculation

/// Calculates CRC16 checksum for a given buffer
/// 
/// # Arguments
/// * `buf` - The byte buffer for which the checksum is calculated
/// 
/// # Returns
/// * The computed CRC16 checksum
/// Calculate CRC16 checksum for a buffer
fn calc_crc16(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in buf {
        let mut v = byte;
        for _ in 0..8 {
            if (v as u16 & 0x01) ^ (crc & 0x01) != 0 {
                crc >>= 1;
                crc ^= CRC_POLY;
            } else {
                crc >>= 1;
            }
            v >>= 1;
        }
    }
    crc
}


// From 5.2 Message Format
/// Create ASCII command message
pub fn make_command_message(cid: enums::CID_E, payload: &[u8]) -> Vec<u8> {
    // CID as two ASCII hex chars
    let cid_str = format!("{:02X}", cid.to_u8());
    // Encode payload as hex
    let payload_hex = hex::encode(payload);
    // Build message without checksum/delimiters
    let content = format!("{}{}", cid_str, payload_hex);
    // ASCII bytes (for CRC calculation)
    let content_bytes = content.as_bytes();
    let checksum = calc_crc16(content_bytes);
    let checksum_str = format!("{:04X}", checksum); // 4 chars ASCII hex
    // Full message: #CIDPayloadCSUM<CR><LF>
    let mut msg = Vec::new();
    msg.push(b'#');
    msg.extend_from_slice(content_bytes);
    msg.extend_from_slice(checksum_str.as_bytes());
    msg.push(b'\r');
    msg.push(b'\n');
    msg
}



/// Parse ASCII response message
pub fn parse_response_message(msg: &[u8]) -> Result<(enums::CID_E, Vec<u8>, u16), Box<dyn Error>> {
    // Check start byte and length
    if msg.len() < 9 || msg[0] != b'$' {
        return Err("Invalid message format".into());
    }
    // Find <CR><LF>
    if msg[msg.len()-2] != b'\r' || msg[msg.len()-1] != b'\n' {
        return Err("Missing CRLF".into());
    }
    // CID: 2 chars after $
    let cid = enums::CID_E::from_u8(u8::from_str_radix(std::str::from_utf8(&msg[1..3])?, 16)?)
        .ok_or("Unknown CID")?;
    // Payload: from index 3 to len-6 (excluding CSUM and CRLF)
    let payload_hex = &msg[3..msg.len()-6];
    let payload = hex::decode(payload_hex)?;
    // CSUM: last 4 chars before CRLF
    let csum_str = &msg[msg.len()-6..msg.len()-2];
    let csum = u16::from_str_radix(std::str::from_utf8(csum_str)?, 16)?;
    // Check CRC
    let content_bytes = &msg[1..msg.len()-6];
    let computed_crc = calc_crc16(content_bytes);
    if computed_crc != csum {
        return Err("Checksum mismatch".into());
    }
    Ok((cid, payload, csum))
}

/* pub trait SeaTracCommand {
    fn cid(&self) -> CID_E;
    fn payload(&self) -> Vec<u8>;

    fn to_bytes(&self) -> Vec<u8> {
        let cid_str = format!("{:02X}", self.cid().to_u8());
        let payload_hex = hex::encode(self.payload());
        let content = format!("{}{}", cid_str, payload_hex);
        let checksum = calc_crc16(content.as_bytes());
        let checksum_str = format!("{:04X}", checksum);
        let mut msg = Vec::new();
        msg.push(b'#');
        msg.extend_from_slice(content.as_bytes());
        msg.extend_from_slice(checksum_str.as_bytes());
        msg.extend_from_slice(b"\r\n");
        msg
    }
}

pub trait SeaTracResponse: Sized {
    fn cid() -> CID_E;
    fn from_payload(payload: &[u8]) -> Option<Self>;
}

#[derive(Debug)]
pub enum SeaTracParseError {
    InvalidStart,
    MissingCRLF,
    UnknownCID,
    ChecksumMismatch,
    HexDecodeError,
    Utf8Error,
}

pub fn parse_message(msg: &[u8]) -> Result<(CID_E, Vec<u8>), SeaTracParseError> {
    if msg.len() < 9 {
        return Err(SeaTracParseError::InvalidStart);
    }

    let start = msg[0];
    if start != b'$' && start != b'#' {
        return Err(SeaTracParseError::InvalidStart);
    }

    if msg[msg.len()-2] != b'\r' || msg[msg.len()-1] != b'\n' {
        return Err(SeaTracParseError::MissingCRLF);
    }

    let cid_str = std::str::from_utf8(&msg[1..3]).map_err(|_| SeaTracParseError::Utf8Error)?;
    let cid_val = u8::from_str_radix(cid_str, 16).map_err(|_| SeaTracParseError::UnknownCID)?;
    let cid = CID_E::from_u8(cid_val).ok_or(SeaTracParseError::UnknownCID)?;

    let payload_hex_end = msg.len().saturating_sub(6);
    let payload_hex = if payload_hex_end > 3 { &msg[3..payload_hex_end] } else { &[] };
    let payload = hex::decode(payload_hex).map_err(|_| SeaTracParseError::HexDecodeError)?;

    let csum_str = &msg[msg.len()-6..msg.len()-2];
    let csum = u16::from_str_radix(std::str::from_utf8(csum_str).map_err(|_| SeaTracParseError::Utf8Error)?, 16)
        .map_err(|_| SeaTracParseError::ChecksumMismatch)?;

    let content = &msg[1..msg.len()-6];
    if calc_crc16(content) != csum {
        return Err(SeaTracParseError::ChecksumMismatch);
    }

    Ok((cid, payload))
}
 */