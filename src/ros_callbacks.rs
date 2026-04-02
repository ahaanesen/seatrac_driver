// TODO: create subscription callback (remove from main)

// TODO: create the ros messages for publishing
use std::sync::{Arc, Mutex};
use std::time::{Duration};

use futures::stream::StreamExt;
use r2r::builtin_interfaces;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, LivelinessPolicy, ReliabilityPolicy};
use r2r::{blueboat_interfaces, QosProfile};

use nalgebra::{Vector3};
use crate::comms::message_types::{ReceivedMsg, UsblData};


# [derive(Debug, Clone)]
pub struct AcousticCommSend {
    pub header: builtin_interfaces::msg::Time,
    pub dest_node_id: u8,
    pub position: Vector3<f64>,
}

pub async fn acoustic_send_callback(
    arc_node: Arc<Mutex<r2r::Node>>,
    topic_name: &str,
    tx_to_tdma: tokio::sync::mpsc::Sender<AcousticCommSend>,
) -> Result<(), r2r::Error> {
    let qos_profile = create_custom_sensor_qos_profile();

    let mut sub = arc_node
        .lock()
        .unwrap()
        .subscribe::<r2r::blueboat_interfaces::msg::AcousticCommSend>(
            topic_name, qos_profile,
    );

    loop {
        match sub.next().await {
            Some(msg) => {
                let dest_id = msg.dest_node_id;
                let position = Vector3::new(msg.position.x, msg.position.y, msg.position.z);
                let acoustic_msg = AcousticCommSend {
                    header: msg.header,
                    dest_node_id: dest_id,
                    position,
                };
                let _ = tx_to_tdma.try_send(acoustic_msg);
            }
            None => break, // continue,
        }
    }

    Ok(())
}

pub fn create_acoustic_receive_msg(
    acoustic_receive: &ReceivedMsg,
) -> r2r::blueboat_interfaces::msg::AcousticCommReceive {

    let t_now = std::time::SystemTime::now();
    let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

    let header_msg = r2r::std_msgs::msg::Header {
        stamp: builtin_interfaces::msg::Time {
            sec: since_epoch.as_secs() as i32,
            nanosec: since_epoch.subsec_nanos(),
        },
        frame_id: "".to_string(),
    };

    r2r::blueboat_interfaces::msg::AcousticCommReceive {
        header: header_msg,
        node_id: acoustic_receive.node_id,
        message_index: acoustic_receive.message_index,
        position: r2r::geometry_msgs::msg::Vector3 {
            x: acoustic_receive.position.x,
            y: acoustic_receive.position.y,
            z: acoustic_receive.position.z,
        },
        t_sent: acoustic_receive.t_sent,
        t_received: acoustic_receive.t_received,
    }
}


pub fn create_usbl_msg(
    usbl_data: &UsblData,
) -> r2r::blueboat_interfaces::msg::UsblMeasurement {

    let t_now = std::time::SystemTime::now();
    let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

    let header_msg = r2r::std_msgs::msg::Header {
        stamp: builtin_interfaces::msg::Time {
            sec: since_epoch.as_secs() as i32,
            nanosec: since_epoch.subsec_nanos(),
        },
        frame_id: "".to_string(),
    };

    r2r::blueboat_interfaces::msg::UsblMeasurement {
        header: header_msg,
        rov_id: usbl_data.node_id,
        time: usbl_data.time,
        channels: usbl_data.channels,
        rssi: usbl_data.rssi.clone(),
        azimuth: usbl_data.azimuth,
        elevation: usbl_data.elevation,
        fit_error: usbl_data.fit_error,
    }
}

fn create_custom_sensor_qos_profile() -> r2r::QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 1,
        reliability: ReliabilityPolicy::BestEffort,
        durability: DurabilityPolicy::Volatile,
        avoid_ros_namespace_conventions: false,
        deadline: Duration::ZERO,
        lifespan: Duration::ZERO,
        liveliness: LivelinessPolicy::SystemDefault,
        liveliness_lease_duration: Duration::ZERO,
    }
}
