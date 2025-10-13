use std::process::{Command, Stdio};
use std::io::{Write};

/// Executes a shell command and returns the output as a vector of bytes.
fn execute_command(command: &str) -> Result<Vec<u8>, String> {
    let output = Command::new("sh")
        .arg("-c")
        .arg(command)
        .stdout(Stdio::piped())
        .output()
        .map_err(|e| e.to_string())?;

    if output.status.success() {
        Ok(output.stdout)
    } else {
        Err(String::from_utf8_lossy(&output.stderr).to_string())
    }

}

/// Encodes the given input string using DCCL via command line.
pub fn encode_input(input: &str) -> Result<Vec<u8>, String> {
    let command = format!(
        "echo \"{}\" | dccl --encode --proto_file src/payload.proto",
        input
    );
    execute_command(&command)
}

/// Decodes the given encoded byte output using DCCL via command line.
pub fn decode_output(encoded_output: &[u8]) -> Result<String, String> {
    let mut decode_process = Command::new("sh")
        .arg("-c")
        .arg("dccl --decode -f src/payload.proto --omit_prefix")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .map_err(|e| e.to_string())?;

    {
        let stdin = decode_process.stdin.as_mut().ok_or("Failed to open stdin")?;
        stdin.write_all(encoded_output).map_err(|e| e.to_string())?;
    }

    let decoded_output = decode_process.wait_with_output().map_err(|e| e.to_string())?;

    if decoded_output.status.success() {
        Ok(String::from_utf8_lossy(&decoded_output.stdout).to_string())
    } else {
        Err(String::from_utf8_lossy(&decoded_output.stderr).to_string())
    }
}

/// Extracts integers from the decoded string result of DCCL.
pub fn extract_values(decoded_result: &str) -> Vec<i32> {

    decoded_result.split_whitespace()
        .filter_map(|pair| {
            let parts: Vec<&str> = pair.split(':').collect();
            if parts.len() == 1 {
                parts[0].trim().parse::<i32>().ok()
            } else {
                None
            }
        })
        .collect()
}
