use crate::seatrac::enums::{self, CID_E};
use crate::seatrac::structs;
use hex;
use std::error::Error;
use std::io;


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
    // crc = (crc & 0xFF) << 8 | (crc >> 8 & 0xFF);
    crc
}

#[test]
fn crc_append_roundtrip() {
    let data: Vec<u8> = vec![0x15]; // example bytes from spec
    let crc = calc_crc16(&data);
    println!("Calculated CRC: {:04X}", crc);
    // assert_eq!(calc_crc16(&data), 0, "CRC over data+CRC should be 0");
}

// From Lisa
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


pub fn make_command(cid: enums::CID_E, payload: &[u8]) -> Vec<u8> {
    // Compute CRC over binary bytes: CID byte followed by payload bytes
    let cid_byte = cid.to_u8();
    let mut crc_buf = Vec::with_capacity(1 + payload.len());
    crc_buf.push(cid_byte);
    crc_buf.extend_from_slice(payload);
    let checksum = calc_crc16(&crc_buf);
    let checksum_bytes = checksum.to_le_bytes();
    let checksum_str = format!("{:02X}{:02X}", checksum_bytes[0], checksum_bytes[1]);

    // Build ASCII hex fields for the message
    let cid_str = format!("{:02X}", cid_byte);
    // Use uppercase hex for consistency (hex::encode returns lowercase by default)
    let payload_hex = hex::encode(payload).to_uppercase();
    // let checksum_str = format!("{:04X}", checksum); // 4 ASCII hex chars, uppercase

    // Assemble full message: #CIDPayloadCSUM<CR><LF>
    let mut msg = Vec::with_capacity(1 + cid_str.len() + payload_hex.len() + checksum_str.len() + 2);
    msg.push(b'#');
    msg.extend_from_slice(cid_str.as_bytes());
    msg.extend_from_slice(payload_hex.as_bytes());
    msg.extend_from_slice(checksum_str.as_bytes());
    msg.push(b'\r');
    msg.push(b'\n');
    msg
}

#[cfg(test)]
#[test]
fn test_make_command() {
    let solution = b"#15C1CF\r\n";
    let cid = enums::CID_E::from_u8(0x15).unwrap();
    let payload = &[];
    let command = make_command(cid, payload);
    assert!(command == solution, "Generated: {} Expected: {}", String::from_utf8_lossy(&command), String::from_utf8_lossy(solution));
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

//From Lisa's implementation
pub fn parse_message(response: Vec<u8>) -> Result<(CID_E, Vec<u8>, u16, Vec<u8>), Box<dyn Error>> {
    // Basic validation of the response
    if response.len() < 8 || response[0] != b'$' {
        return Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "Invalid response format"))); // Check format
    }
    
    let hex_str = &response[1..response.len() - 4];
    if hex_str.len() % 2 != 0 {
        return Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "OddLength")));
    }
    let byte_repr = hex::decode(hex_str)?;
    // let byte_repr = hex::decode(&response[1..response.len() - 4]).unwrap(); // Decode the message excluding checksum

    let cid = CID_E::from_u8(byte_repr[0]); // Extract Command Identifier (CID)

    let payload = byte_repr[1..byte_repr.len()].to_vec(); // Extract payload

    // Validate and compuse std::io::Error;ute CRC
    let computed_checksum = calc_crc16(&byte_repr); // Compute checksum
    let received_checksum_bytes = hex::decode(&response[response.len() - 4..response.len()]).unwrap(); // Decode received checksum
    let received_checksum = u16::from_le_bytes([received_checksum_bytes[0], received_checksum_bytes[1]]); // Convert to u16
    
    // Check if the computed checksum matches the received one
    if computed_checksum != received_checksum {
        return Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "Checksum mismatch"))); // Handle checksum mismatch
    }

    match cid {
        Some(cid) => Ok((cid, payload, received_checksum, byte_repr)), // Return parsed message components
        None => Err(Box::new(io::Error::new(io::ErrorKind::InvalidData, "Invalid command identification code"))), // Handle invalid CID
    }
}
