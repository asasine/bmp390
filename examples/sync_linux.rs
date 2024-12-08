use bmp390::{sync::Bmp390, Address, Configuration};
use linux_embedded_hal::{Delay, I2cdev};
use uom::si::length::meter;

fn main() {
    let i2c = I2cdev::new("/dev/i2c-1")
        .map_err(bmp390::Error::I2c)
        .expect("Failed to create I2C device");
    let mut delay = Delay;

    let config = Configuration::default();
    let mut sensor = Bmp390::try_new(i2c, Address::Up, &mut delay, &config)
        .expect("Failed to initialize BMP390 sensor");

    let measurement = sensor.measure().expect("Failed to measure BMP390 data");

    println!(
        "Measurement: Pressure: {} hPa, Temperature: {} °C, Altitude: {} m",
        measurement.pressure.get::<uom::si::pressure::hectopascal>(),
        measurement
            .temperature
            .get::<uom::si::thermodynamic_temperature::degree_celsius>(),
        measurement.altitude.get::<meter>()
    );
}
