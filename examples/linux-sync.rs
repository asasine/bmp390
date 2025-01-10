//! This example demonstrates how to use the synchronous BMP390 driver on Linux.
//!
//! # Arguments
//! The program accepts an optional argument that specifies the I2C device to use. If not provided, the default device
//! `/dev/i2c-1` is used.

use bmp390::{sync::Bmp390, Address, Configuration};
use clap::Parser;
use linux_embedded_hal::{Delay, I2cdev};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    /// Which I2C device to use.
    #[clap(default_value = "/dev/i2c-1")]
    device: String,
}

fn main() {
    let args = Args::parse();
    eprintln!("Using I2C device: {}", args.device);
    let i2c = I2cdev::new(args.device)
        .map_err(bmp390::Error::I2c)
        .expect("Failed to create I2C device");

    let config = Configuration::default();
    let mut sensor = Bmp390::try_new(i2c, Address::Up, Delay, &config)
        .expect("Failed to initialize BMP390 sensor");

    let measurement = sensor.measure().expect("Failed to measure BMP390 data");
    println!("Measurement: {}", measurement);
}
