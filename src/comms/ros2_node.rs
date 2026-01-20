use rclrs::*;
use std::sync::{Arc, Mutex};
//use std::{thread, time::Duration};
use std_msgs::msg::String as StringMsg;


/// Queues shared between ROS callbacks and TDMA/modem handler
#[derive(Clone)]
pub struct MessageQueues {
    pub to_modem: Arc<Mutex<Vec<String>>>,     // Messages from ROS to modem
    pub from_modem: Arc<Mutex<Vec<String>>>,   // Messages from modem to ROS
}


#[derive(Clone)]
#[allow(unused)]
pub struct RosBridge {
    worker: Worker<Option<String>>,
    publisher: Publisher<StringMsg>,
    subscription: Subscription<StringMsg>,
    pub queues: MessageQueues,
}

impl RosBridge {
        pub fn new(executor: &Executor, node_name: &str) -> Result<Self, RclrsError> {
            let node = executor.create_node(node_name)?;
            let worker = node.create_worker(None);

            let queues = MessageQueues {
                to_modem: Arc::new(Mutex::new(Vec::new())),
                from_modem: Arc::new(Mutex::new(Vec::new())),
            };

            // Publisher: outgoing data to /seatrac_x/output
            let out_topic = format!("/{}/output", node_name);
            let publisher = node.create_publisher::<StringMsg>(out_topic.as_str())?;

            // Subscription: push incoming ROS data into send queue
            let in_topic = format!("/{}/input", node_name);
            let to_modem_clone = Arc::clone(&queues.to_modem);
            let subscription = node.create_subscription::<StringMsg, _>(
                in_topic.as_str(),
                move |msg: StringMsg| {
                    // Handle incoming message from subscription
                    let mut queue = to_modem_clone.lock().unwrap();
                    queue.push(msg.data);
                },
            )?;

            Ok(Self { worker, publisher, queues, subscription })
        }


    /// Publish all messages currently waiting in the receive queue
    pub fn publish_from_queue(&self) -> Result<(), RclrsError> {
        let publisher   = self.publisher.clone();
        let mut out_queue = self.queues.from_modem.lock().unwrap();
        for msg in out_queue.drain(..) {
            publisher.publish(StringMsg { data: msg })?;
        }
        Ok(())
    }
}
