# Seatrac driver
## Current status
- reading config file (json because serde_yaml is decpricated)
- setting beacon id, salinity
- usbl flag + usbl messages
    - for seatrac usbl enabling XCVR_FIX_MSGS and XCVR_BASELINES_MSGS (can also enable XCVR_USBL_MSGS, but is very large)
    - CHANGE: by sending DAT messages with MSG_OWAYU, azimuth and elevation is availible in DAT_RECIEVE if the recieving modem is a usbl! Because of this, switched to only DAT messages again
- broadcasting default message with some function of sin and cos for x and y, but retrieving depth for z
    - takes in ack handler, so message is automatically resent. might not want this as is broadcast every 5 seconds and always want the newest message
- existing ros2 bridge that connects topics to queues to bridge communication from the ros2 network (topics) to acoustic communication (queues for in and out)

## TODOs
- finish implementation to change baudrate for seatrac modem (might work now, but not checked)
- logging (ros2 bagging)
- not implemented what to do when something is published to the to_modem topic
- make a third topic to_ekf
- make own file in comms that makes and sends all the messages, currently in tdma_scheduler, but not sure if it makes sense
- 