use bmp390::{sync::Bmp390, Address, Configuration};
use clap::Parser;
use embedded_hal::delay::DelayNs;
use linux_embedded_hal::{Delay, I2cdev};

/// This example demonstrates how to use the synchronous BMP390 driver on Linux.
///
/// By default, this will print one measurement from the `/dev/i2c-1` device. The device can be
/// changed with the first positional argument. The program can also print multiple times with the
/// `--count` argument, or made to repeat `--forever`. To speed up the program, use
/// `--frequency <FREQ>`. The BMP390 is configured to 50 Hz by default; any value above this will
/// yield repeated measurements.
#[derive(Parser)]
#[command(version)]
#[command(group = clap::ArgGroup::new("repetition").multiple(false))]
struct Args {
    /// Which I2C device to use.
    #[clap(default_value = "/dev/i2c-1")]
    device: String,

    /// How many measurements to take before exiting. Exclusive with `forever`.
    #[clap(short, long, default_value_t = 1, group = "repetition")]
    count: usize,

    /// Whether to perform measurements continuously. Exclusive with `count`.
    #[clap(long, default_value_t = false, group = "repetition")]
    forever: bool,

    /// How many measurements to take per second.
    #[clap(short, long, default_value_t = 1.0)]
    frequency: f32,
}

impl Args {
    fn delay_ms(&self) -> u32 {
        (1000.0 / self.frequency) as u32
    }
}

fn main() {
    let args = Args::parse();
    eprintln!("Using I2C device: {}", args.device);
    let i2c = I2cdev::new(&args.device)
        .map_err(bmp390::Error::I2c)
        .expect("Failed to create I2C device");

    let config = Configuration::default();
    let mut sensor = Bmp390::try_new(i2c, Address::Up, Delay, &config)
        .expect("Failed to initialize BMP390 sensor");

    let mut delay = Delay;
    let delay_ms = args.delay_ms();

    if args.forever {
        eprintln!("Measuring forever...");
        for i in 1usize.. {
            let measurement = sensor.measure().expect("Failed to measure BMP390 data");
            eprintln!("{i}: {measurement}");
            delay.delay_ms(delay_ms);
        }
    } else {
        let count = args.count;
        for i in 1..=count {
            let measurement = sensor.measure().expect("Failed to measure BMP390 data");
            eprintln!("{i}/{count}: {measurement}");
            if i != count {
                delay.delay_ms(delay_ms);
            }
        }
    }
}

