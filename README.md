# Seatrac driver
## Current status
- reading config file (json because serde_yaml is decpricated)
- setting beacon id, salinity
- usbl flag + usbl messages
    - for seatrac usbl enabling XCVR_FIX_MSGS and XCVR_BASELINES_MSGS (can also enable XCVR_USBL_MSGS, but is very large)

## TODOs
- finish implementation to change baudrate for seatrac modem (might work now, but not checked)
- hashmap to sturcture messages?? (or keep it simple with match on cid)
- tdma (thread)
- send thread?
- listen tread?
- logging (what type, same type as lisa or ros2 bagging?)
- ros2 thread?
- move PositionalCoordinates struct (in message manager) to a message_types file to gather all different message types?