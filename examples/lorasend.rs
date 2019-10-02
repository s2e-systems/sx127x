use lorahatlib::{LoraHat, TransceiverMode};

fn main() {
    println!("Initializing LoraHat");

    let lorahat = LoraHat::new().unwrap();

    let hatversion = lorahat.version().unwrap();

    println!("Lora HAT version {}", hatversion);

    lorahat.set_transceiver_mode(TransceiverMode::Sleep).unwrap();
}