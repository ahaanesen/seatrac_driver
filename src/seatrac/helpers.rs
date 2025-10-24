/// Calculate the propagation time based on the size of the message and propagation speed.
pub fn calculate_propagation_time(acoustic_msg: &[u8], propagation_time: u64) -> u64 {
    let message_size = acoustic_msg.len() as u64;
    let wait_time_ms = propagation_time * message_size;
    (wait_time_ms / 1000) as u64
}