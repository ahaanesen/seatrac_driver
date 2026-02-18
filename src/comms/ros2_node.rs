use r2r::std_msgs::msg::String as StringMsg;
use r2r::{Publisher, QosProfile};
use std::sync::{Arc, Mutex};
use futures::stream::StreamExt;
use tokio::task;


/// Queues shared between ROS callbacks and TDMA/modem handler
#[derive(Clone)]
pub struct MessageQueues {
    pub to_modem: Arc<Mutex<Vec<String>>>,     // Messages from ROS to modem
    pub from_modem: Arc<Mutex<Vec<String>>>,   // Messages from modem to ROS
}


pub struct RosBridge {
    publisher: Publisher<StringMsg>,
    pub queues: MessageQueues,
}

impl RosBridge {
    pub fn new(arc_node: &Arc<Mutex<r2r::Node>>, node_name: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let queues = MessageQueues {
            to_modem: Arc::new(Mutex::new(Vec::new())),
            from_modem: Arc::new(Mutex::new(Vec::new())),
        };

        // Publisher: outgoing data to /seatrac_x/output
        let out_topic = format!("/{}/output", node_name);
        let publisher = arc_node.lock().unwrap()
            .create_publisher::<StringMsg>(&out_topic, QosProfile::sensor_data())?;

        // Subscription: push incoming ROS data into send queue
        let in_topic = format!("/{}/input", node_name);
        let mut sub = arc_node.lock().unwrap()
            .subscribe::<StringMsg>(&in_topic, QosProfile::sensor_data())?;

        // Spawn a tokio task to poll the subscription stream and push into the queue
        let to_modem_clone = Arc::clone(&queues.to_modem);
        task::spawn(async move {
            loop {
                match sub.next().await {
                    Some(msg) => {
                        let mut queue = to_modem_clone.lock().unwrap();
                        queue.push(msg.data);
                    }
                    None => break,
                }
            }
        });

        Ok(Self { publisher, queues })
    }


    /// Publish all messages currently waiting in the receive queue
    pub fn publish_from_queue(&self) -> Result<(), Box<dyn std::error::Error>> {
        let mut out_queue = self.queues.from_modem.lock().unwrap();
        for msg in out_queue.drain(..) {
            let ros_msg = StringMsg { data: msg };
            self.publisher.publish(&ros_msg)?;
        }
        Ok(())
    }
}