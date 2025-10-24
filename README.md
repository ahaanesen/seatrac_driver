# Seatrac driver
## Current status
- reading config file (json because serde_yaml is decpricated)
- setting beacon id, salinity
- usbl flag + usbl messages
    - for seatrac usbl enabling XCVR_FIX_MSGS and XCVR_BASELINES_MSGS (can also enable XCVR_USBL_MSGS, but is very large)
    - CHANGE: by sending DAT messages with MSG_OWAYU, azimuth and elevation is availible in DAT_RECIEVE if the recieving modem is a usbl! Because of this, switched to only DAT messages again

## TODOs
- finish implementation to change baudrate for seatrac modem (might work now, but not checked)
- logging (ros2 bagging)
- ros2 thread?
- make own file in comms that makes and sends all the messages, currently in tdma_scheduler, but not sure if it makes sense