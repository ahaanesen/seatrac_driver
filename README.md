# Seatrac driver
## Current status
- reading config file (json because serde_yaml is decpricated)
- setting beacon id, salinity
## TODOs
- finish implementation to change baudrate for seatrac modem
- usbl flag + usbl messages
    - for seatrac usbl only enabling XCVR_FIX_MSGS and XCVR_BASELINES_MSGS (as the XCVR_USBL_MSGS is very large)