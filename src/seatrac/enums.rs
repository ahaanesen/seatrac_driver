// References to the "SeaTrac Developer Guide (for Beacon firmware version 3.7)"
// are indicated in comments throughout this file.

// Chapter 6. Message field type
// 6.3. Enumerations & Constants

// NOTE: The enums in this file use identifier names from the SeaTrac Developer Guide
// to make it easy to cross-reference the C firmware documentation. The guide uses
// names with an `_E` suffix (for example `AMSGTYPE_E`, `APAYLOAD_E`, `BAUDRATE_E`,
// `BID_E`, `CID_E`, `CST_E`, `STATUSMODE_E`, `XCVR_TXMSGCTRL_E`). We preserve
// those identifier names here so the Rust types map directly to the documented
// constants and enums. The variant names remain uppercase to match the spec.

#![allow(non_camel_case_types)]

#[derive(Debug, Clone, PartialEq, Eq, Copy)]
pub enum AMSGTYPE_E { // Acoustic Message Type
    MSG_OWAY = 0x0,
    MSG_OWAYU = 0x1,
    MSG_REQ = 0x2,
    MSG_RESP = 0x3,
    MSG_REQU = 0x4,
    MSG_RESPU = 0x5,
    MSG_REQX = 0x6,
    MSG_RESPX = 0x7,
    MSG_UNKNOWN = 0xFF,
}

impl AMSGTYPE_E {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x0 => Some(AMSGTYPE_E::MSG_OWAY),
            0x1 => Some(AMSGTYPE_E::MSG_OWAYU),
            0x2 => Some(AMSGTYPE_E::MSG_REQ),
            0x3 => Some(AMSGTYPE_E::MSG_RESP),
            0x4 => Some(AMSGTYPE_E::MSG_REQU),
            0x5 => Some(AMSGTYPE_E::MSG_RESPU),
            0x6 => Some(AMSGTYPE_E::MSG_REQX),
            0x7 => Some(AMSGTYPE_E::MSG_RESPX),
            0xFF => Some(AMSGTYPE_E::MSG_UNKNOWN),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum APAYLOAD_E { // Acoustic Payload Identifier (APAYLOAD_E)
    PLOAD_PING = 0x0,
    PLOAD_ECHO = 0x1,
    PLOAD_NAV = 0x2,
    PLOAD_DAT = 0x3,
    PLOAD_DEX = 0x4,
}

impl APAYLOAD_E {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x0 => Some(APAYLOAD_E::PLOAD_PING),
            0x1 => Some(APAYLOAD_E::PLOAD_ECHO),
            0x2 => Some(APAYLOAD_E::PLOAD_NAV),
            0x3 => Some(APAYLOAD_E::PLOAD_DAT),
            0x4 => Some(APAYLOAD_E::PLOAD_DEX),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BAUDRATE_E { // Serial Port Baud Rate
    BAUD_4800 = 0x07,
    BAUD_9600 = 0x08,
    BAUD_14400 = 0x09,
    BAUD_19200 = 0x0A,
    BAUD_38400 = 0x0B,
    BAUD_57600 = 0x0C,
    BAUD_115200 = 0x0D,
}

impl BAUDRATE_E {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x07 => Some(BAUDRATE_E::BAUD_4800),
            0x08 => Some(BAUDRATE_E::BAUD_9600),
            0x09 => Some(BAUDRATE_E::BAUD_14400),
            0x0A => Some(BAUDRATE_E::BAUD_19200),
            0x0B => Some(BAUDRATE_E::BAUD_38400),
            0x0C => Some(BAUDRATE_E::BAUD_57600),
            0x0D => Some(BAUDRATE_E::BAUD_115200),
            _ => None,
        }
    }

    pub fn to_u8(&self) -> u8 {
        match *self {
            BAUDRATE_E::BAUD_4800 => 0x07,
            BAUDRATE_E::BAUD_9600 => 0x08,
            BAUDRATE_E::BAUD_14400 => 0x09,
            BAUDRATE_E::BAUD_19200 => 0x0A,
            BAUDRATE_E::BAUD_38400 => 0x0B,
            BAUDRATE_E::BAUD_57600 => 0x0C,
            BAUDRATE_E::BAUD_115200 => 0x0D,
        }
    }
    pub fn from_u32(value: u32) -> Option<Self> {
        match value {
            4800 => Some(BAUDRATE_E::BAUD_4800),
            9600 => Some(BAUDRATE_E::BAUD_9600),
            14400 => Some(BAUDRATE_E::BAUD_14400),
            19200 => Some(BAUDRATE_E::BAUD_19200),
            38400 => Some(BAUDRATE_E::BAUD_38400),
            57600 => Some(BAUDRATE_E::BAUD_57600),
            115200 => Some(BAUDRATE_E::BAUD_115200),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BID_E { // Beacon Identification Code
    BEACON_ALL = 0x0,
    BEACON_ID_1 = 0x1,
    BEACON_ID_2 = 0x2,
    BEACON_ID_3 = 0x3,
    BEACON_ID_4 = 0x4,
    BEACON_ID_5 = 0x5,
    BEACON_ID_6 = 0x6,
    BEACON_ID_7 = 0x7,
    BEACON_ID_8 = 0x8,
    BEACON_ID_9 = 0x9,
    BEACON_ID_10 = 0xA,
    BEACON_ID_11 = 0xB,
    BEACON_ID_12 = 0xC,
    BEACON_ID_13 = 0xD,
    BEACON_ID_14 = 0xE,
    BEACON_ID_15 = 0xF,
}

impl BID_E {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x0 => Some(BID_E::BEACON_ALL),
            0x1 => Some(BID_E::BEACON_ID_1),
            0x2 => Some(BID_E::BEACON_ID_2),
            0x3 => Some(BID_E::BEACON_ID_3),
            0x4 => Some(BID_E::BEACON_ID_4),
            0x5 => Some(BID_E::BEACON_ID_5),
            0x6 => Some(BID_E::BEACON_ID_6),
            0x7 => Some(BID_E::BEACON_ID_7),
            0x8 => Some(BID_E::BEACON_ID_8),
            0x9 => Some(BID_E::BEACON_ID_9),
            0xA => Some(BID_E::BEACON_ID_10),
            0xB => Some(BID_E::BEACON_ID_11),
            0xC => Some(BID_E::BEACON_ID_12),
            0xD => Some(BID_E::BEACON_ID_13),
            0xE => Some(BID_E::BEACON_ID_14),
            0xF => Some(BID_E::BEACON_ID_15),
            _ => None,
        }
    }

    pub fn to_u8(&self) -> u8 {
        match *self {
            BID_E::BEACON_ALL => 0x0,
            BID_E::BEACON_ID_1 => 0x1,
            BID_E::BEACON_ID_2 => 0x2,
            BID_E::BEACON_ID_3 => 0x3,
            BID_E::BEACON_ID_4 => 0x4,
            BID_E::BEACON_ID_5 => 0x5,
            BID_E::BEACON_ID_6 => 0x6,
            BID_E::BEACON_ID_7 => 0x7,
            BID_E::BEACON_ID_8 => 0x8,
            BID_E::BEACON_ID_9 => 0x9,
            BID_E::BEACON_ID_10 => 0xA,
            BID_E::BEACON_ID_11 => 0xB,
            BID_E::BEACON_ID_12 => 0xC,
            BID_E::BEACON_ID_13 => 0xD,
            BID_E::BEACON_ID_14 => 0xE,
            BID_E::BEACON_ID_15 => 0xF,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CAL_ACTION_E { // CalibrationAction
    CAL_ACC_DEFAULTS = 0x00,
    CAL_ACC_RESET = 0x01,
    CAL_ACC_CALC = 0x02,
    CAL_MAG_DEFAULTS = 0x03,
    CAL_MAG_RESET = 0x04,
    CAL_MAG_CALC = 0x05,
    CAL_PRES_OFFSET_RESET = 0x06,
    CAL_PRES_OFFSET_CALC = 0x07,
}

impl CAL_ACTION_E {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x00 => Some(CAL_ACTION_E::CAL_ACC_DEFAULTS),
            0x01 => Some(CAL_ACTION_E::CAL_ACC_RESET),
            0x02 => Some(CAL_ACTION_E::CAL_ACC_CALC),
            0x03 => Some(CAL_ACTION_E::CAL_MAG_DEFAULTS),
            0x04 => Some(CAL_ACTION_E::CAL_MAG_RESET),
            0x05 => Some(CAL_ACTION_E::CAL_MAG_CALC),
            0x06 => Some(CAL_ACTION_E::CAL_PRES_OFFSET_RESET),
            0x07 => Some(CAL_ACTION_E::CAL_PRES_OFFSET_CALC),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CID_E { // Command Identification Codes
    CID_SYS_ALIVE = 0x01,
    CID_SYS_INFO = 0x02,
    CID_SYS_REBOOT = 0x03,
    CID_SYS_ENGINEERING = 0x04,

    CID_PROG_INIT = 0x0D,
    CID_PROG_BLOCK = 0x0E,
    CID_PROG_UPDATE = 0x0F,

    CID_STATUS = 0x10,
    CID_STATUS_CFG_GET = 0x11,
    CID_STATUS_CFG_SET = 0x12,

    CID_SETTINGS_GET = 0x15, // Request to get the current settings from the device.
    CID_SETTINGS_SET = 0x16, // Command to set new settings on the device.
    CID_SETTINGS_LOAD = 0x17,
    CID_SETTINGS_SAVE = 0x18, // Command to save the current settings to non-volatile memory.
    CID_SETTINGS_RESET = 0x19,

    CID_CAL_ACTION = 0x20,
    CID_AHRS_CAL_GET = 0x21,
    CID_AHRS_CAL_SET = 0x22,

    CID_XCVR_ANALYSE = 0x30,
    CID_XCVR_TX_MSG = 0x31,
    CID_XCVR_RX_ERR = 0x32,
    CID_XCVR_RX_MSG = 0x33,
    CID_XCVR_RX_REQ = 0x34,
    CID_XCVR_RX_RESP = 0x35,
    CID_XCVR_RX_UNHANDLED = 0x37,
    CID_XCVR_USBL = 0x38, // Message generated when a USBL signal is decoded into an angular bearing.
    CID_XCVR_FIX = 0x39, // Message generated when the transceiver gets a position/range fix on a beacon from a request/response (any message type?).
    CID_XCVR_STATUS = 0x3A,
    CID_XCVR_TX_MSGCTRL_SET = 0x3B,
    CID_XCVR_USBL_TIME = 0x3C,
    CID_XCVR_BASELINE = 0x3D,

    CID_PING_SEND = 0x40, // Command sent to initiate a ping.
    CID_PING_REQ = 0x41, // Message generated when a ping is received.
    CID_PING_RESP = 0x42, // Message generated when a ping request is received and a ping response is sent.
    CID_PING_ERROR = 0x43, // Message generated if no response is received to a ping within the timeout period.
    
    CID_ECHO_SEND = 0x48,
    CID_ECHO_REQ = 0x49,
    CID_ECHO_RESP = 0x4A,
    CID_ECHO_ERROR = 0x4B,

    CID_NAV_QUERY_SEND = 0x50,
    CID_NAV_QUERY_REQ = 0x51,
    CID_NAV_QUERY_RESP = 0x52,
    CID_NAV_ERROR = 0x53,
    CID_NAV_QUEUE_SET = 0x58,
    CID_NAV_QUEUE_CLR = 0x59,
    CID_NAV_QUEUE_STATUS = 0x5A,
    CID_NAV_STATUS_SEND = 0x5B,
    CID_NAV_STATUS_RECEIVE = 0x5C,
    CID_NAV_QUEUE_GET = 0x5D,

    CID_DAT_SEND = 0x60,
    CID_DAT_RECEIVE = 0x61,
    CID_DAT_ERROR = 0x63,
    CID_DAT_QUEUE_SET = 0x64,
    CID_DAT_QUEUE_CLR = 0x65,
    CID_DAT_QUEUE_STATUS = 0x66,
    CID_DAT_QUEUE_GET = 0x67,

    CID_CFG_BEACON_GET = 0x80,
    CID_CFG_BEACON_SET = 0x81,
    CID_CFG_BEACON_RESP = 0x82,
    CID_CFG_REQ_SEND = 0x83,
    CID_CFG_REQ_RECEIVE = 0x84,
    CID_CFG_RESP_SEND = 0x85,
    CID_CFG_RESP_RECEIVE = 0x86,
    // TODO: complete the rest of the CID_E values
}

impl CID_E {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x01 => Some(CID_E::CID_SYS_ALIVE),
            0x02 => Some(CID_E::CID_SYS_INFO),
            0x03 => Some(CID_E::CID_SYS_REBOOT),
            0x04 => Some(CID_E::CID_SYS_ENGINEERING),
            0x0D => Some(CID_E::CID_PROG_INIT),
            0x0E => Some(CID_E::CID_PROG_BLOCK),
            0x0F => Some(CID_E::CID_PROG_UPDATE),
            0x10 => Some(CID_E::CID_STATUS),
            0x11 => Some(CID_E::CID_STATUS_CFG_GET),
            0x12 => Some(CID_E::CID_STATUS_CFG_SET),
            0x15 => Some(CID_E::CID_SETTINGS_GET),
            0x16 => Some(CID_E::CID_SETTINGS_SET),
            0x17 => Some(CID_E::CID_SETTINGS_LOAD),
            0x18 => Some(CID_E::CID_SETTINGS_SAVE),
            0x19 => Some(CID_E::CID_SETTINGS_RESET),
            0x20 => Some(CID_E::CID_CAL_ACTION),
            0x21 => Some(CID_E::CID_AHRS_CAL_GET),
            0x22 => Some(CID_E::CID_AHRS_CAL_SET),
            0x30 => Some(CID_E::CID_XCVR_ANALYSE),
            0x31 => Some(CID_E::CID_XCVR_TX_MSG),
            0x32 => Some(CID_E::CID_XCVR_RX_ERR),
            0x33 => Some(CID_E::CID_XCVR_RX_MSG),
            0x34 => Some(CID_E::CID_XCVR_RX_REQ),
            0x35 => Some(CID_E::CID_XCVR_RX_RESP),
            0x37 => Some(CID_E::CID_XCVR_RX_UNHANDLED),
            0x38 => Some(CID_E::CID_XCVR_USBL),
            0x39 => Some(CID_E::CID_XCVR_FIX),
            0x3A => Some(CID_E::CID_XCVR_STATUS),
            0x3B => Some(CID_E::CID_XCVR_TX_MSGCTRL_SET),
            0x3C => Some(CID_E::CID_XCVR_USBL_TIME),
            0x3D => Some(CID_E::CID_XCVR_BASELINE),
            0x40 => Some(CID_E::CID_PING_SEND),
            0x41 => Some(CID_E::CID_PING_REQ),
            0x42 => Some(CID_E::CID_PING_RESP),
            0x43 => Some(CID_E::CID_PING_ERROR),
            0x48 => Some(CID_E::CID_ECHO_SEND),
            0x49 => Some(CID_E::CID_ECHO_REQ),
            0x4A => Some(CID_E::CID_ECHO_RESP),
            0x4B => Some(CID_E::CID_ECHO_ERROR),
            0x50 => Some(CID_E::CID_NAV_QUERY_SEND),
            0x51 => Some(CID_E::CID_NAV_QUERY_REQ),
            0x52 => Some(CID_E::CID_NAV_QUERY_RESP),
            0x53 => Some(CID_E::CID_NAV_ERROR),
            0x58 => Some(CID_E::CID_NAV_QUEUE_SET),
            0x59 => Some(CID_E::CID_NAV_QUEUE_CLR),
            0x5A => Some(CID_E::CID_NAV_QUEUE_STATUS),
            0x5B => Some(CID_E::CID_NAV_STATUS_SEND),
            0x5C => Some(CID_E::CID_NAV_STATUS_RECEIVE),
            0x5D => Some(CID_E::CID_NAV_QUEUE_GET),
            0x60 => Some(CID_E::CID_DAT_SEND),
            0x61 => Some(CID_E::CID_DAT_RECEIVE),
            0x63 => Some(CID_E::CID_DAT_ERROR),
            0x64 => Some(CID_E::CID_DAT_QUEUE_SET),
            0x65 => Some(CID_E::CID_DAT_QUEUE_CLR),
            0x66 => Some(CID_E::CID_DAT_QUEUE_STATUS),
            0x67 => Some(CID_E::CID_DAT_QUEUE_GET),
            0x80 => Some(CID_E::CID_CFG_BEACON_GET),
            0x81 => Some(CID_E::CID_CFG_BEACON_SET),
            0x82 => Some(CID_E::CID_CFG_BEACON_RESP),
            0x83 => Some(CID_E::CID_CFG_REQ_SEND),
            0x84 => Some(CID_E::CID_CFG_REQ_RECEIVE),
            0x85 => Some(CID_E::CID_CFG_RESP_SEND),
            0x86 => Some(CID_E::CID_CFG_RESP_RECEIVE),
            _ => None,
        }
    }

    /// Convert this CID_E variant to its numeric u8 code.
    pub fn to_u8(&self) -> u8 {
        match *self {
            CID_E::CID_SYS_ALIVE => 0x01,
            CID_E::CID_SYS_INFO => 0x02,
            CID_E::CID_SYS_REBOOT => 0x03,
            CID_E::CID_SYS_ENGINEERING => 0x04,
            CID_E::CID_PROG_INIT => 0x0D,
            CID_E::CID_PROG_BLOCK => 0x0E,
            CID_E::CID_PROG_UPDATE => 0x0F,
            CID_E::CID_STATUS => 0x10,
            CID_E::CID_STATUS_CFG_GET => 0x11,
            CID_E::CID_STATUS_CFG_SET => 0x12,
            CID_E::CID_SETTINGS_GET => 0x15,
            CID_E::CID_SETTINGS_SET => 0x16,
            CID_E::CID_SETTINGS_LOAD => 0x17,
            CID_E::CID_SETTINGS_SAVE => 0x18,
            CID_E::CID_SETTINGS_RESET => 0x19,
            CID_E::CID_CAL_ACTION => 0x20,
            CID_E::CID_AHRS_CAL_GET => 0x21,
            CID_E::CID_AHRS_CAL_SET => 0x22,
            CID_E::CID_XCVR_ANALYSE => 0x30,
            CID_E::CID_XCVR_TX_MSG => 0x31,
            CID_E::CID_XCVR_RX_ERR => 0x32,
            CID_E::CID_XCVR_RX_MSG => 0x33,
            CID_E::CID_XCVR_RX_REQ => 0x34,
            CID_E::CID_XCVR_RX_RESP => 0x35,
            CID_E::CID_XCVR_RX_UNHANDLED => 0x37,
            CID_E::CID_XCVR_USBL => 0x38,
            CID_E::CID_XCVR_FIX => 0x39,
            CID_E::CID_XCVR_STATUS => 0x3A,
            CID_E::CID_XCVR_TX_MSGCTRL_SET => 0x3B,
            CID_E::CID_XCVR_USBL_TIME => 0x3C,
            CID_E::CID_XCVR_BASELINE => 0x3D,
            CID_E::CID_PING_SEND => 0x40,
            CID_E::CID_PING_REQ => 0x41,
            CID_E::CID_PING_RESP => 0x42,
            CID_E::CID_PING_ERROR => 0x43,
            CID_E::CID_ECHO_SEND => 0x48,
            CID_E::CID_ECHO_REQ => 0x49,
            CID_E::CID_ECHO_RESP => 0x4A,
            CID_E::CID_ECHO_ERROR => 0x4B,
            CID_E::CID_NAV_QUERY_SEND => 0x50,
            CID_E::CID_NAV_QUERY_REQ => 0x51,
            CID_E::CID_NAV_QUERY_RESP => 0x52,
            CID_E::CID_NAV_ERROR => 0x53,
            CID_E::CID_NAV_QUEUE_SET => 0x58,
            CID_E::CID_NAV_QUEUE_CLR => 0x59,
            CID_E::CID_NAV_QUEUE_STATUS => 0x5A,
            CID_E::CID_NAV_STATUS_SEND => 0x5B,
            CID_E::CID_NAV_STATUS_RECEIVE => 0x5C,
            CID_E::CID_NAV_QUEUE_GET => 0x5D,
            CID_E::CID_DAT_SEND => 0x60,
            CID_E::CID_DAT_RECEIVE => 0x61,
            CID_E::CID_DAT_ERROR => 0x63,
            CID_E::CID_DAT_QUEUE_SET => 0x64,
            CID_E::CID_DAT_QUEUE_CLR => 0x65,
            CID_E::CID_DAT_QUEUE_STATUS => 0x66,
            CID_E::CID_DAT_QUEUE_GET => 0x67,
            CID_E::CID_CFG_BEACON_GET => 0x80,
            CID_E::CID_CFG_BEACON_SET => 0x81,
            CID_E::CID_CFG_BEACON_RESP => 0x82,
            CID_E::CID_CFG_REQ_SEND => 0x83,
            CID_E::CID_CFG_REQ_RECEIVE => 0x84,
            CID_E::CID_CFG_RESP_SEND => 0x85,
            CID_E::CID_CFG_RESP_RECEIVE => 0x86,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CST_E { // Command Status Codes
    CST_OK = 0x00,
    CST_FAIL = 0x01,
    CST_EEPROM_ERROR = 0x03,
    CST_CMD_PARAM_MISSING = 0x04,
    CST_CMD_PARAM_INVALID = 0x05,
    CST_PROG_FLASH_ERROR = 0x0A,
    CST_PROG_FIRMWARE_ERROR = 0x0B,
    CST_PROG_SECTION_ERROR = 0x0C,
    CST_PROG_LENGTH_ERROR = 0x0D,
    CST_PROG_DATA_ERROR = 0x0E,
    CST_PROG_CHECKSUM_ERROR = 0x0F,
    CST_XCVR_BUSY = 0x30,
    CST_XCVR_ID_REJECTED = 0x31,
    CST_XCVR_CSUM_ERROR = 0x32,
    CST_XCVR_LENGTH_ERROR = 0x33,
    CST_XCVR_RESP_TIMEOUT = 0x34,
    CST_XCVR_RESP_ERROR = 0x35,
    CST_XCVR_RESP_WRONG = 0x36,
    CST_XCVR_PLOAD_ERROR = 0x37,
    CST_XCVR_STATE_STOPPED = 0x3A,
    CST_XCVR_STATE_IDLE = 0x3B,
    CST_XCVR_STATE_TX = 0x3C,
    CST_XCVR_STATE_REQ = 0x3D,
    CST_XCVR_STATE_RX = 0x3E,
    CST_XCVR_STATE_RESP = 0x3F,
    CST_DEX_SOCKET_ERROR = 0x70,
    CST_DEX_RX_SYNC = 0x71,
    CST_DEX_RX_DATA = 0x72,
    CST_DEX_RX_SEQ_ERROR = 0x73,
    CST_DEX_RX_MSG_ERROR = 0x74,
    CST_DEX_REQ_ERROR = 0x75,
    CST_DEX_RESP_TMO_ERROR = 0x76,
    CST_DEX_RESP_MSG_ERROR = 0x77,
    CST_DEX_RESP_REMOTE_ERROR = 0x78,
    CST_CFG_RESP_NOT_ALLOWED = 0x80,
    CST_CFG_ALREADY_LOCKED = 0x81,
    CST_CFG_NOT_LOCKED = 0x82,
    CST_INTERRUPT_LOCKED = 0xB0
}

impl CST_E {
    pub fn to_u8(&self) -> u8 {
        match *self {
            CST_E::CST_OK => 0x00,
            CST_E::CST_FAIL => 0x01,
            CST_E::CST_EEPROM_ERROR => 0x03,
            CST_E::CST_CMD_PARAM_MISSING => 0x04,
            CST_E::CST_CMD_PARAM_INVALID => 0x05,
            CST_E::CST_PROG_FLASH_ERROR => 0x0A,
            CST_E::CST_PROG_FIRMWARE_ERROR => 0x0B,
            CST_E::CST_PROG_SECTION_ERROR => 0x0C,
            CST_E::CST_PROG_LENGTH_ERROR => 0x0D,
            CST_E::CST_PROG_DATA_ERROR => 0x0E,
            CST_E::CST_PROG_CHECKSUM_ERROR => 0x0F,
            CST_E::CST_XCVR_BUSY => 0x30,
            CST_E::CST_XCVR_ID_REJECTED => 0x31,
            CST_E::CST_XCVR_CSUM_ERROR => 0x32,
            CST_E::CST_XCVR_LENGTH_ERROR => 0x33,
            CST_E::CST_XCVR_RESP_TIMEOUT => 0x34,
            CST_E::CST_XCVR_RESP_ERROR => 0x35,
            CST_E::CST_XCVR_RESP_WRONG => 0x36,
            CST_E::CST_XCVR_PLOAD_ERROR => 0x37,
            CST_E::CST_XCVR_STATE_STOPPED => 0x3A,
            CST_E::CST_XCVR_STATE_IDLE => 0x3B,
            CST_E::CST_XCVR_STATE_TX => 0x3C,
            CST_E::CST_XCVR_STATE_REQ => 0x3D,
            CST_E::CST_XCVR_STATE_RX => 0x3E,
            CST_E::CST_XCVR_STATE_RESP => 0x3F,
            CST_E::CST_DEX_SOCKET_ERROR => 0x70,
            CST_E::CST_DEX_RX_SYNC => 0x71,
            CST_E::CST_DEX_RX_DATA => 0x72,
            CST_E::CST_DEX_RX_SEQ_ERROR => 0x73,
            CST_E::CST_DEX_RX_MSG_ERROR => 0x74,
            CST_E::CST_DEX_REQ_ERROR => 0x75,
            CST_E::CST_DEX_RESP_TMO_ERROR => 0x76,
            CST_E::CST_DEX_RESP_MSG_ERROR => 0x77,
            CST_E::CST_DEX_RESP_REMOTE_ERROR => 0x78,
            CST_E::CST_CFG_RESP_NOT_ALLOWED => 0x80,
            CST_E::CST_CFG_ALREADY_LOCKED => 0x81,
            CST_E::CST_CFG_NOT_LOCKED => 0x82,
            CST_E::CST_INTERRUPT_LOCKED => 0xB0,
        }
    }

    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x00 => Some(CST_E::CST_OK),
            0x01 => Some(CST_E::CST_FAIL),
            0x03 => Some(CST_E::CST_EEPROM_ERROR),
            0x04 => Some(CST_E::CST_CMD_PARAM_MISSING),
            0x05 => Some(CST_E::CST_CMD_PARAM_INVALID),
            0x0A => Some(CST_E::CST_PROG_FLASH_ERROR),
            0x0B => Some(CST_E::CST_PROG_FIRMWARE_ERROR),
            0x0C => Some(CST_E::CST_PROG_SECTION_ERROR),
            0x0D => Some(CST_E::CST_PROG_LENGTH_ERROR),
            0x0E => Some(CST_E::CST_PROG_DATA_ERROR),
            0x0F => Some(CST_E::CST_PROG_CHECKSUM_ERROR),
            0x30 => Some(CST_E::CST_XCVR_BUSY),
            0x31 => Some(CST_E::CST_XCVR_ID_REJECTED),
            0x32 => Some(CST_E::CST_XCVR_CSUM_ERROR),
            0x33 => Some(CST_E::CST_XCVR_LENGTH_ERROR),
            0x34 => Some(CST_E::CST_XCVR_RESP_TIMEOUT),
            0x35 => Some(CST_E::CST_XCVR_RESP_ERROR),
            0x36 => Some(CST_E::CST_XCVR_RESP_WRONG),
            0x37 => Some(CST_E::CST_XCVR_PLOAD_ERROR),
            0x3A => Some(CST_E::CST_XCVR_STATE_STOPPED),
            0x3B => Some(CST_E::CST_XCVR_STATE_IDLE),
            0x3C => Some(CST_E::CST_XCVR_STATE_TX),
            0x3D => Some(CST_E::CST_XCVR_STATE_REQ),
            0x3E => Some(CST_E::CST_XCVR_STATE_RX),
            0x3F => Some(CST_E::CST_XCVR_STATE_RESP),
            0x70 => Some(CST_E::CST_DEX_SOCKET_ERROR),
            0x71 => Some(CST_E::CST_DEX_RX_SYNC),
            0x72 => Some(CST_E::CST_DEX_RX_DATA),
            0x73 => Some(CST_E::CST_DEX_RX_SEQ_ERROR),
            0x74 => Some(CST_E::CST_DEX_RX_MSG_ERROR),
            0x75 => Some(CST_E::CST_DEX_REQ_ERROR),
            0x76 => Some(CST_E::CST_DEX_RESP_TMO_ERROR),
            0x77 => Some(CST_E::CST_DEX_RESP_MSG_ERROR),
            0x78 => Some(CST_E::CST_DEX_RESP_REMOTE_ERROR),
            0x80 => Some(CST_E::CST_CFG_RESP_NOT_ALLOWED),
            0x81 => Some(CST_E::CST_CFG_ALREADY_LOCKED),
            0x82 => Some(CST_E::CST_CFG_NOT_LOCKED),
            0xB0 => Some(CST_E::CST_INTERRUPT_LOCKED),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum STATUSMODE_E { // Status Output Mode
    STATUS_MODE_MANUAL = 0x0,
    STATUS_MODE_1HZ = 0x1,
    STATUS_MODE_2HZ5 = 0x2,
    STATUS_MODE_5HZ = 0x3,
    STATUS_MODE_10HZ = 0x4,
    STATUS_MODE_25HZ = 0x5,
}

impl STATUSMODE_E {
    pub fn to_u8(&self) -> u8 {
        match *self {
            STATUSMODE_E::STATUS_MODE_MANUAL => 0x0,
            STATUSMODE_E::STATUS_MODE_1HZ => 0x1,
            STATUSMODE_E::STATUS_MODE_2HZ5 => 0x2,
            STATUSMODE_E::STATUS_MODE_5HZ => 0x3,
            STATUSMODE_E::STATUS_MODE_10HZ => 0x4,
            STATUSMODE_E::STATUS_MODE_25HZ => 0x5,
        }
    }

    /// Convert a raw 3-bit value (bits[2:0]) into the corresponding STATUSMODE_E
    /// Returns None for values that are not defined in the spec.
    pub fn from_u8(value: u8) -> Option<Self> {
        match value & 0x07 {
            0x0 => Some(STATUSMODE_E::STATUS_MODE_MANUAL),
            0x1 => Some(STATUSMODE_E::STATUS_MODE_1HZ),
            0x2 => Some(STATUSMODE_E::STATUS_MODE_2HZ5),
            0x3 => Some(STATUSMODE_E::STATUS_MODE_5HZ),
            0x4 => Some(STATUSMODE_E::STATUS_MODE_10HZ),
            0x5 => Some(STATUSMODE_E::STATUS_MODE_25HZ),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum XCVR_TXMSGCTRL_E { // Transmit Message Control
    XCVR_TXMSG_ALLOW_ALL = 0x0,
    XCVR_TXMSG_BLOCK_RESP = 0x1,
    XCVR_TXMSG_BLOCK_ALL = 0x3,
}

impl XCVR_TXMSGCTRL_E {
    pub fn to_u8(&self) -> u8 {
        match *self {
            XCVR_TXMSGCTRL_E::XCVR_TXMSG_ALLOW_ALL => 0x0,
            XCVR_TXMSGCTRL_E::XCVR_TXMSG_BLOCK_RESP => 0x1,
            XCVR_TXMSGCTRL_E::XCVR_TXMSG_BLOCK_ALL => 0x3,
        }
    }
}
