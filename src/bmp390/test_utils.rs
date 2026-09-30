//! Test utilities for unit tests on [`crate::Bmp390`].

extern crate std;

use crate::{
    raw::{Device, field_sets::CalibrationData},
    registers::Coefficients,
};
use device_driver::FieldSet;
use device_driver_mock::{LinearMemory, Recording};
use std::vec;

const CALIBRATION_BYTES: [u8; 22] = [
    0x00, 0x98, 0x6c, 0xa9, 0x4a, 0xf9, 0xe3, 0x1c, 0x61, 0x16, 0x06, 0x01, 0x51, 0x4a, 0xde, 0x5d,
    0x03, 0xfa, 0xf9, 0x0e, 0x06, 0xf5,
];

pub fn coefficients() -> Coefficients {
    let mut calibration = CalibrationData::new_zero();
    calibration
        .get_inner_buffer_mut()
        .copy_from_slice(&CALIBRATION_BYTES);
    calibration.into()
}

/// Create a [`Recording`] interface with the pressure, temperature, and calibration data preloaded.
pub fn interface() -> Recording<LinearMemory> {
    interface_with_data_ready(false)
}

/// Create a [`Recording`] interface with measurement data and the requested data-ready state.
pub fn interface_with_data_ready(data_ready: bool) -> Recording<LinearMemory> {
    let interface = LinearMemory::default();
    let mut d = Device::new(interface);
    let Ok(()) = d.data_with_interrupt_status().write(|w| {
        w.set_pressure(0x6b_b3_cb);
        w.set_temperature(0x82_ba_d1);
        w.set_data_ready(data_ready);
    });
    let Ok(()) = d.calibration_data().write(|w| {
        w.get_inner_buffer_mut().copy_from_slice(&CALIBRATION_BYTES);
    });

    Recording {
        interface: d.interface,
        operations: vec![],
    }
}
