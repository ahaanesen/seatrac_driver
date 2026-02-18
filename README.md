# Seatrac driver
Acoustic communication using Seatrac X010 and USBL capabilities using X150.
Divided in comms and seatrac (hw driver), with modem_driver a uniform trait facilitating flexibility in chosen hardware.

## Description
For acoustic communication between agents of AUVs and ASV. 
In the ROS2 network, each agent becomes visible as nodes called "seatrac_<node_id>". Each of the aforementioned nodes connect to two topics, /seatrac_<node_id>/output and /seatrac_<node_id>/input. The node publishes messages received acoustically to /output, and (for now) also usbl data if the modem is a X150 modem. Additionally, the node subscribes to the /input topic and will send the messages acoustically.


### Current status
- reading config file (json because serde_yaml is decpricated)
- setting beacon id, salinity, baudrate (baudrate might need a checkup)
- usbl flag + usbl messages
    - by sending DAT messages with MSG_OWAYU, azimuth and elevation is availible in DAT_RECIEVE if the recieving modem is a usbl! Because of this, switched to only DAT messages again
- disabled: broadcasting default message with some function of sin and cos for x and y, but retrieving depth for z
    - takes in ack handler, so message is automatically resent. might not want this as is broadcast every 5 seconds and always want the newest message
- existing ros2 bridge that connects topics to queues to bridge communication from the ros2 network (topics) to acoustic communication (queues for in and out)
- acoustically received messages sent on ros2 topics
- messages received on the to_modem topic sent acoustically


### TODOs
- make a third topic to_ekf
- message types for whatever goes on the topics (for now only strings as this has not been specified yet)
- need better time precision for ranging (nb might increase dat message, so might need to the change packet length index)
    - most clock stuff might be exchanged when implementing clock synchronization anyway
- clean seatrac driver

## Getting Started

### Dependencies
#### ROS 2 + r2r setup
This project uses the [r2r](https://github.com/sequenceplanner/r2r) crate for ROS 2 integration. Unlike ros2_rust, no Python virtual environment or colcon build is required.

1. Install ROS 2 (e.g., Humble or Jazzy)
2. Source your ROS 2 environment:
```
source /opt/ros/<distro>/setup.bash
```
3. Build with cargo:
```
cargo build
```


### Installing

* How/where to download your program
* Any modifications needed to be made to files/folders

### Executing program

* All modems should have unique node_id's between 0 and 15
* Modems are either Seatrac X010 or Seatrac X150 (USBL)
* Make sure both config_comms and config_modem are correct
* Run with
```
cargo run
```
* With broadcasting disabled, it is necessary to publish the messages to the /input topic with this format:
```
ros2 topic pub /seatrac_<node_id>/input std_msgs/msg/String'{data: "destination_id:<node_id> x:<> y:<> z:<>"}' -1 
```

## Help

Any advise for common problems or issues.
```
command to run if program contains helper info
```

## Authors


## Version History


## License



## Acknowledgments
