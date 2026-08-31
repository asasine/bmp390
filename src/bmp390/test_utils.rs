//! Test utilities for unit tests on [`crate::Bmp390`].

extern crate std;

use crate::raw::Device;
use device_driver::FieldSet;
use device_driver_mock::{LinearMemory, Recording};
use std::vec;

const CALIBRATION_BYTES: [u8; 22] = [
    0x00, 0x98, 0x6c, 0xa9, 0x4a, 0xf9, 0xe3, 0x1c, 0x61, 0x16, 0x06, 0x01, 0x51, 0x4a, 0xde, 0x5d,
    0x03, 0xfa, 0xf9, 0x0e, 0x06, 0xf5,
];

/// Create a [`Recording`] interface with the pressure, temperature, and calibration data preloaded.
pub fn interface() -> Recording<LinearMemory> {
    let interface = LinearMemory::default();
    let mut d = Device::new(interface);
    let Ok(()) = d.pressure_data().write(|w| w.set_value(0x6b_b3_cb));
    let Ok(()) = d.temperature_data().write(|w| w.set_value(0x82_ba_d1));
    let Ok(()) = d.calibration_data().write(|w| {
        w.get_inner_buffer_mut().copy_from_slice(&CALIBRATION_BYTES);
    });

    Recording {
        interface: d.interface,
        operations: vec![],
    }
}
