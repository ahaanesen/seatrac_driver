// Import required modules for serial communication and protocol handling
// use crate::modem::identifiers::CommandIdentificationCode; // Enum for command identification codes
// use crate::modem::record::DAT_SEND; // Structure for DAT messages
// use serialport::SerialPort; // Trait for serial port functionality
use crate::seatrac::enums::{self, CID_E};
use crate::seatrac::structs; // Replace with actual path
use hex;
use std::error::Error;
use std::io;
// use crate::seatrac::structs::{SETTINGS_T, XCVR_USBL, XCVR_BASELINES};


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


/// Prepares a message by adding header, CID, payload, and CRC
/// 
/// # Arguments
/// * `cid` - The command identification code for the message
/// * `msg` - The DAT_SEND structure containing the message data
/// 
/// # Returns
/// * A vector of bytes representing the prepared message
pub fn prepare_message(cid: CID_E, msg: structs::DAT_SEND) -> Vec<u8> {
    let mut message = Vec::new(); // Initialize message vector
    
    // Add start delimiter
    message.push(b'#'); // Start of the message
    
    // Add Command Identifier (CID) in hexadecimal format
    let cid_str = format!("{:02X}", cid.to_u8());
    message.extend_from_slice(cid_str.as_bytes()); // Append CID
    let dest_str = format!("{:02X}", msg.dest_id as u8);
    message.extend_from_slice(dest_str.as_bytes()); // Append destination ID
    let msg_type = format!("{:02X}", msg.msg_type as u8);
    message.extend_from_slice(msg_type.as_bytes()); // Append message type
    let msg_length = format!("{:02X}", msg.packet_len as u8);
    message.extend_from_slice(msg_length.as_bytes()); // Append message length
    let packet_as_bytes = hex::encode(&msg.packet_data); // Convert packet data to hex
    message.extend_from_slice(packet_as_bytes.as_bytes()); // Append packet data

    // Convert message content (excluding the start delimiter) to bytes
    let byte_repr = hex::decode(&message[1..]).unwrap(); // Decode hex representation
    
    // Calculate and append CRC16
    let checksum = calc_crc16(&byte_repr); // Compute checksum
    let checksum_bytes = checksum.to_le_bytes(); // Convert checksum to bytes
    let checksum_str = format!("{:02X}{:02X}", checksum_bytes[0], checksum_bytes[1]);
    message.extend_from_slice(checksum_str.as_bytes()); // Append checksum to message
    
    // Add end delimiter
    message.extend_from_slice(b"\r\n"); // End of the message

    
    message // Return the prepared message
}

// From 5.2 Message Format
/// Create ASCII command message
/// # Arguments
/// * `cid` - The command identification code
/// * `payload` - The payload data as a byte slice
/// # Returns
/// * A vector of bytes representing the complete ASCII command message
pub fn make_command(cid: enums::CID_E, payload: &[u8]) -> Vec<u8> {
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

// TODO: unify with above function?
/// Create ASCII command message with u16 payload
/// # Arguments
/// * `cid` - The command identification code
/// * `payload` - The payload data as a u16 (only used for reboot?)
pub fn make_command_u16(cid: enums::CID_E, payload: u16) -> Vec<u8> {
    let cid_str = format!("{:02X}", cid.to_u8());
    let payload_hex = hex::encode(payload.to_le_bytes());
    let content = format!("{}{}", cid_str, payload_hex);
    let content_bytes = content.as_bytes();
    let checksum = calc_crc16(content_bytes);
    let checksum_str = format!("{:04X}", checksum); // 4 chars ASCII hex
    let mut msg = Vec::new();
    msg.push(b'#');
    msg.extend_from_slice(content_bytes);
    msg.extend_from_slice(checksum_str.as_bytes());
    msg.extend_from_slice(b"\r\n");
    msg
}

// TODO: check if this third option can work instead of both of the two above
/* pub fn make_command_message(cid: enums::CID_E, payload: impl AsRef<[u8]>) -> Vec<u8> {
    // CID as two ASCII hex chars
    let cid_str = format!("{:02X}", cid.to_u8());
    // Encode payload as hex
    let payload_bytes = payload.as_ref();
    let payload_hex = hex::encode(payload_bytes);
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
} */

/// Parse ASCII response message
/// # Arguments
/// * `msg` - The byte slice containing the ASCII response message
/// # Returns
/// * A tuple containing the command ID, the hex encoded payload, checksum, and the binary representation of the message
/// * An error if the message format is invalid or checksum does not match
pub fn parse_response(msg: &[u8]) -> Result<(enums::CID_E, Vec<u8>, u16), Box<dyn Error>> {
    // Basic validation
    if msg.len() < 8 || msg[0] != b'$' {
        return Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "Invalid response format")));
    }

    // The message payload (hex) is everything after the '$' up to the last 4 ASCII hex chars
    // which represent the checksum. This mirrors the reference implementation provided.
    let hex_str = &msg[1..msg.len() - 4];
    if hex_str.len() % 2 != 0 {
        return Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "OddLength")));
    }

    // Decode the ASCII hex into binary bytes
    let byte_repr = hex::decode(hex_str)?;
    if byte_repr.is_empty() {
        return Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "Empty payload")));
    }

    // First byte of decoded bytes is the CID
    let cid = enums::CID_E::from_u8(byte_repr[0]).ok_or_else(|| {
        Box::new(io::Error::new(io::ErrorKind::InvalidData, "Unknown CID")) as Box<dyn Error>
    })?;

    // Payload is the remaining bytes
    let payload = if byte_repr.len() > 1 { byte_repr[1..].to_vec() } else { Vec::new() };

    // Compute CRC over the binary bytes
    let computed_checksum = calc_crc16(&byte_repr);

    // Received checksum is the last 4 ASCII hex chars of the message
    let received_checksum_bytes = hex::decode(&msg[msg.len() - 4..msg.len()])?;
    if received_checksum_bytes.len() != 2 {
        return Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "Bad checksum length")));
    }
    let received_checksum = u16::from_le_bytes([received_checksum_bytes[0], received_checksum_bytes[1]]);

    if computed_checksum != received_checksum {
        return Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "Checksum mismatch")));
    }

    Ok((cid, payload, received_checksum))
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