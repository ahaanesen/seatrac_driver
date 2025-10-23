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
pub struct RosBridge {
    worker: Worker<Option<String>>,
    publisher: Publisher<StringMsg>,
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

        // Publisher for outgoing data to /seatrac/output
        let publisher = node.create_publisher::<StringMsg>("/seatrac/output")?;

        // Subscription: push incoming ROS data into send queue
        worker.create_subscription::<StringMsg, _>(
            "/seatrac/input",
            move |data: &mut Option<String>, msg: StringMsg| {
                *data = Some(msg.data);
            },
        )?;

        Ok(Self { worker, publisher, queues })
    }

    /// Publish all messages currently waiting in the receive queue
    pub fn queue_from_subscription(&self) -> Result<(), RclrsError> {
        // move a clone of the Arc<Mutex<Vec<String>>> into the closure
        let queue_arc = Arc::clone(&self.queues.to_modem);

        // run a closure that takes ownership of the queue Arc
        // the closure receives &mut Option<String> and moves the message out with take()
        drop(
            self.worker.run(move |data: &mut Option<String>| -> Result<(), RclrsError> {
            if let Some(msg) = data.take() {
                let mut queue = queue_arc.lock().unwrap(); // or map_err on poisoning if you prefer
                queue.push(msg);
                Ok(())
            } else {
                // no message available — treat as no-op or return an error if that's required
                Ok(())
            }
            })
        );
        Ok(())
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
