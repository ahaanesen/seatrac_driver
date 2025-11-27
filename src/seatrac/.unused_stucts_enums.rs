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


#[derive(Debug, Clone, PartialEq)]
pub struct XCVR_USBL {
    pub xcor_sig_peakfloat: f32,
    pub xcor_threshold: f32,
    pub xcor_cross_point: u16,
    pub xcor_cross_mag: f32,
    pub xcor_detect: u16,
    pub xcor_length: u16,
    pub xcor_data: Vec<f32>,
    pub channels: u8,
    pub channel_rssi: Vec<i16>,
    pub baselines: u8,
    pub phase_angle: Vec<f32>,
    pub signal_azimuth: i16,
    pub signal_elevation: i16,
    pub signal_fit_error: f32,
    pub beacon_dest_id: enums::BID_E,
    pub beacon_src_id: enums::BID_E,
}

impl XCVR_USBL {
    pub fn from_bytes(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {

        if data.len() < 20 {
            return Err("Buffer too short to parse XCVR_USBL".into());
        }

        let xcor_sig_peakfloat = f32::from_le_bytes(data[0..4].try_into()?);
        let xcor_threshold = f32::from_le_bytes(data[4..8].try_into()?);
        let xcor_cross_point = u16::from_le_bytes(data[8..10].try_into()?);
        let xcor_cross_mag = f32::from_le_bytes(data[10..14].try_into()?);
        let xcor_detect = u16::from_le_bytes(data[14..16].try_into()?);
        let xcor_length = u16::from_le_bytes(data[16..18].try_into()?);

        let num_floats = (xcor_length as usize).min((data.len() - 18) / 4);
        let mut xcor_data = Vec::with_capacity(num_floats);
        for i in 0..num_floats {
            let start = 18 + i * 4;
            let end = start + 4;
            if end <= data.len() {
                xcor_data.push(f32::from_le_bytes(data[start..end].try_into()?));
            }
        }

        // After xcor_data, we have channels and channel_rssi
        let offset_after_xcor = 18 + num_floats * 4;
        if offset_after_xcor + 1 > data.len() {
            return Err("Buffer too short to read channels".into());
        }
        let channels = data[offset_after_xcor];
        let mut channel_rssi = Vec::with_capacity(channels as usize);
        for i in 0..channels {
            let start = offset_after_xcor + 1 + (i as usize) * 2;
            let end = start + 2;
            if end <= data.len() {
                channel_rssi.push(i16::from_le_bytes(data[start..end].try_into()?));
            }
        }

        // After channel_rssi, we have baselines and phase_angle
        let offset_after_rssi = offset_after_xcor + 1 + (channels as usize) * 2;
        if offset_after_rssi + 1 > data.len() {
            return Err("Buffer too short to read baselines count".into());
        }
        let baselines = data[offset_after_rssi];
        let mut phase_angle: Vec<f32> = Vec::with_capacity(baselines as usize);
        let mut offset_after_phase = offset_after_rssi + 1;
        // read baselines number of f32 angles
        for i in 0..(baselines as usize) {
            let start = offset_after_phase + i * 4;
            let end = start + 4;
            if end > data.len() {
                return Err("Buffer too short to read phase_angle values".into());
            }
            phase_angle.push(f32::from_le_bytes(data[start..end].try_into()?));
        }
        offset_after_phase += (baselines as usize) * 4;

        // Now read signal azimuth, elevation, fit_error
        if offset_after_phase + 2 > data.len() {
            return Err("Buffer too short to read signal_azimuth".into());
        }
        let signal_azimuth = i16::from_le_bytes(data[offset_after_phase..offset_after_phase + 2].try_into()?);
        offset_after_phase += 2;
        if offset_after_phase + 2 > data.len() {
            return Err("Buffer too short to read signal_elevation".into());
        }
        let signal_elevation = i16::from_le_bytes(data[offset_after_phase..offset_after_phase + 2].try_into()?);
        offset_after_phase += 2;
        if offset_after_phase + 4 > data.len() {
            return Err("Buffer too short to read signal_fit_error".into());
        }
        let signal_fit_error = f32::from_le_bytes(data[offset_after_phase..offset_after_phase + 4].try_into()?);
        offset_after_phase += 4;

        // Finally beacon destination and source IDs (each 1 byte)
        if offset_after_phase + 2 > data.len() {
            return Err("Buffer too short to read beacon ids".into());
        }
        let beacon_dest_id = enums::BID_E::from_u8(data[offset_after_phase]).ok_or("Invalid BID_E for beacon_dest_id")?;
        offset_after_phase += 1;
        let beacon_src_id = enums::BID_E::from_u8(data[offset_after_phase]).ok_or("Invalid BID_E for beacon_src_id")?;

        Ok(Self {
            xcor_sig_peakfloat,
            xcor_threshold,
            xcor_cross_point,
            xcor_cross_mag,
            xcor_detect,
            xcor_length,
            xcor_data,
            channels,
            channel_rssi,
            baselines,
            phase_angle,
            signal_azimuth,
            signal_elevation,
            signal_fit_error,
            beacon_dest_id,
            beacon_src_id,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct XCVR_BASELINES {
    pub beacon_src_id: enums::BID_E,
    pub baseline_count: u8,
    pub baselines: Vec<f32>,
}
impl XCVR_BASELINES {
    pub fn from_bytes(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use std::convert::TryInto;
        if data.len() < 2 {
            return Err("Buffer too short to parse XCVR_BASELINES".into());
        }
        let beacon_src_id = enums::BID_E::from_u8(data[0]).ok_or("Invalid BID_E for beacon_src_id")?;
        let baseline_count = data[1];
        let mut baselines: Vec<f32> = Vec::with_capacity(baseline_count as usize);
        let mut offset: usize = 2;
        for _ in 0..(baseline_count as usize) {
            if offset + 4 > data.len() {
                return Err("Buffer too short to read baseline values".into());
            }
            let v = f32::from_le_bytes(data[offset..offset + 4].try_into()?);
            baselines.push(v);
            offset += 4;
        }
        Ok(Self { beacon_src_id, baseline_count, baselines })

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