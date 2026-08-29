use bmp390::{
    Bmp390, Command, ConfigurationBuilder, IirFilterCoefficient, MeasurementMode, OdrSel,
    Oversampling,
    field_sets::{Config, Odr, Osr, PwrCtrl},
    interfaces::{I2cInterface, Polling, Sdo},
};
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
        (1000.0 / self.frequency).floor() as u32
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    eprintln!("Using I2C device: {}", args.device);
    let i2c = I2cdev::new(&args.device)?;

    let i2c = Polling {
        interface: I2cInterface {
            bus: i2c,
            address: Sdo::Up,
        },
        delay: Delay,
    };

    let mut bmp390 = Bmp390::new(i2c);

    let chip_id = bmp390.device().chip_id().read()?;
    eprintln!("Chip ID: {:#04X}", chip_id.value());
    let rev_id = bmp390.device().rev_id().read()?;
    eprintln!("Rev ID: {:#04X}", rev_id.value());

    // read event and interrupt status registers to clear any pending interrupts
    bmp390.device().event().read()?;
    bmp390.device().int_status().read()?;

    // configure
    let builder = ConfigurationBuilder::new()
        .config({
            let mut c = Config::new();
            c.set_iir_filter(IirFilterCoefficient::Coef15);
            c
        })
        .odr({
            let mut odr = Odr::new();
            odr.set_odr_sel(OdrSel::Odr50);
            odr
        })
        .osr({
            let mut osr = Osr::new();
            osr.set_pressure(Oversampling::X8);
            osr.set_temperature(Oversampling::X1);
            osr
        })
        .pwr_ctrl({
            let mut pwr = PwrCtrl::new();
            pwr.set_pressure_enable(true);
            pwr.set_temperature_enable(true);
            pwr.set_mode(MeasurementMode::Normal);
            pwr
        });

    bmp390.configure(builder)?;

    // check for errors after writing config
    let err_reg = bmp390.device().err_reg().read()?;
    if err_reg.conf_err() {
        panic!("Error: configuration was invalid");
    }

    let mut delay = Delay;
    let delay_ms = args.delay_ms();

    let result = bmp390
        .device()
        .cmd()
        .dispatch(|cmd| cmd.set_cmd(Command::SoftReset));

    eprintln!("Soft reset: {:?}", result);

    if args.forever {
        eprintln!("Measuring forever...");
        for i in 1usize.. {
            let measurement = bmp390.measure()?;
            eprintln!("{i}: {measurement}");
            delay.delay_ms(delay_ms);
        }
    } else {
        let count = args.count;
        for i in 1..=count {
            let measurement = bmp390.measure()?;
            eprintln!("{i}/{count}: {measurement}");
            if i != count {
                delay.delay_ms(delay_ms);
            }
        }
    }

    Ok(())
}
