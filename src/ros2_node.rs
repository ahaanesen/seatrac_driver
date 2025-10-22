use rclrs;
use std::sync::{Arc, Mutex};
use std_msgs::msg::String;

#[derive(Clone)]
pub struct RosBridge {
    pub outgoing: Arc<Mutex<Vec<String>>>,
    pub publisher: rclrs::Publisher<String>,
}

impl RosBridge {
    pub fn new(context: &rclrs::Context, node_name: &str) -> anyhow::Result<Self> {
        let node = rclrs::create_node(context, node_name)?;
        let publisher = node.create_publisher::<String>("/seatrac/output", rclrs::QOS_PROFILE_DEFAULT)?;
        let outgoing = Arc::new(Mutex::new(Vec::new()));
        let outgoing_clone = outgoing.clone();

        node.create_subscription::<String>(
            "/seatrac/input",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: String| {
                outgoing_clone.lock().unwrap().push(msg.data.clone());
            },
        )?;

        // spin node in background
        let node_bg = node.clone();
        std::thread::spawn(move || {
            rclrs::spin(&node_bg).unwrap();
        });

        Ok(Self { outgoing, publisher })
    }

    pub fn next_outgoing(&self) -> Option<String> {
        self.outgoing.lock().unwrap().pop()
    }

    pub fn publish_incoming(&self, msg: String) {
        let ros_msg = String { data: msg };
        self.publisher.publish(ros_msg).unwrap();
    }
}
