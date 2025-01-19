use bmp390::{sync::Bmp390, Address, Configuration};
use clap::Parser;
use embedded_hal::delay::DelayNs;
use linux_embedded_hal::{Delay, I2cdev};

/// This example demonstrates how to use the synchronous BMP390 driver on Linux.
///
/// By default, this will print measurements from the `/dev/i2c-1` device continuously.
#[derive(Parser)]
#[command(version)]
struct Args {
    /// Which I2C device to use.
    #[clap(default_value = "/dev/i2c-1")]
    device: String,
}

fn main() {
    let args = Args::parse();
    eprintln!("Using I2C device: {}", args.device);
    let i2c = I2cdev::new(&args.device)
        .map_err(bmp390::Error::I2c)
        .expect("Failed to create I2C device");

    let config = Configuration {
        fifo: bmp390::FifoConfig {
            fifo_mode: true,
            fifo_stop_on_full: false,
            fifo_time_en: true,
            fifo_press_en: true,
            fifo_temp_en: true,
            fifo_subsampling: bmp390::FifoSubsampling::X32,
            data_select: bmp390::FifoDataSelect::Filtered,
        },
        ..Default::default()
    };

    let mut sensor = Bmp390::try_new(i2c, Address::Up, Delay, &config)
        .expect("Failed to initialize BMP390 sensor");

    sensor.flush_fifo().expect("Failed to flush FIFO");

    let mut delay = Delay;
    let delay_ms = 1000;

    eprintln!("Measuring forever...");
    loop {
        let iter = sensor.iter_fifo().expect("Failed to read FIFO data");
        for frame in iter {
            eprintln!("Frame: {:?}", frame);
        }

        delay.delay_ms(delay_ms);
    }
}
