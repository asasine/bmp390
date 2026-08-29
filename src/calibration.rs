use crate::field_sets::CalibrationData;
use uom::si::{
    f32::{Pressure, ThermodynamicTemperature},
    pressure::pascal,
    thermodynamic_temperature::degree_celsius,
};

/// Output from the BMP390 consists of ADC outputs.
///
/// These must be compensated using formulas from the datasheet to obtain the actual temperature and pressure values,
/// using coefficients stored in non-volatile memory (NVM).
///
/// # Datasheet
/// - Section 3.11 Output compensation.
/// - Appendix A: Computation formulae reference implementation.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Coefficients {
    par_t1: f32,
    par_t2: f32,
    par_t3: f32,
    par_p1: f32,
    par_p2: f32,
    par_p3: f32,
    par_p4: f32,
    par_p5: f32,
    par_p6: f32,
    par_p7: f32,
    par_p8: f32,
    par_p9: f32,
    par_p10: f32,
    par_p11: f32,
}

impl Coefficients {
    /// Compensate a temperature reading according to calibration coefficients.
    ///
    /// # Datasheet
    /// Apendix A, Section 8.5
    pub fn compensate_temperature(
        &self,
        temperature_uncompensated: u32,
    ) -> ThermodynamicTemperature {
        // This could be done in fewer expressions, but it's broken down for clarity and to match the datasheet
        let uncompensated = temperature_uncompensated as f32;
        let partial_data1 = uncompensated - self.par_t1;
        let partial_data2 = partial_data1 * self.par_t2;
        let temperature = partial_data2 + (partial_data1 * partial_data1) * self.par_t3;
        ThermodynamicTemperature::new::<degree_celsius>(temperature)
    }

    /// Compensate a pressure reading according to calibration coefficients.
    ///
    /// # Datasheet
    /// Apendix A, Section 8.6
    pub fn compensate_pressure(
        &self,
        temperature: ThermodynamicTemperature,
        pressure_uncompensated: u32,
    ) -> Pressure {
        // This could be done in fewer expressions, but it's broken down for clarity and to match the datasheet
        let uncompensated = pressure_uncompensated as f32;
        let temperature = temperature.get::<degree_celsius>();
        let partial_data1 = self.par_p6 * temperature;
        let partial_data2 = self.par_p7 * temperature * temperature;
        let partial_data3 = self.par_p8 * temperature * temperature * temperature;
        let partial_out1 = self.par_p5 + partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = self.par_p2 * temperature;
        let partial_data2 = self.par_p3 * temperature * temperature;
        let partial_data3 = self.par_p4 * temperature * temperature * temperature;
        let partial_out2 =
            uncompensated * (self.par_p1 + partial_data1 + partial_data2 + partial_data3);

        let partial_data1 = uncompensated * uncompensated;
        let partial_data2 = self.par_p9 + self.par_p10 * temperature;
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 =
            partial_data3 + uncompensated * uncompensated * uncompensated * self.par_p11;

        let pressure = partial_out1 + partial_out2 + partial_data4;
        Pressure::new::<pascal>(pressure)
    }
}

impl From<CalibrationData> for Coefficients {
    fn from(value: CalibrationData) -> Self {
        #[cfg(feature = "defmt")]
        defmt::trace!("NVM_PAR: {}", value);
        let nvm_par_t1 = value.nvm_par_t_1();
        let nvm_par_t2 = value.nvm_par_t_2();
        let nvm_par_t3 = value.nvm_par_t_3();
        let nvm_par_p1 = value.nvm_par_p_1();
        let nvm_par_p2 = value.nvm_par_p_2();
        let nvm_par_p3 = value.nvm_par_p_3();
        let nvm_par_p4 = value.nvm_par_p_4();
        let nvm_par_p5 = value.nvm_par_p_5();
        let nvm_par_p6 = value.nvm_par_p_6();
        let nvm_par_p7 = value.nvm_par_p_7();
        let nvm_par_p8 = value.nvm_par_p_8();
        let nvm_par_p9 = value.nvm_par_p_9();
        let nvm_par_p10 = value.nvm_par_p_10();
        let nvm_par_p11 = value.nvm_par_p_11();

        Self {
            par_t1: (nvm_par_t1 as f32) / 0.003_906_25,    // 2^-8
            par_t2: (nvm_par_t2 as f32) / 1_073_741_824.0, // 2^30
            par_t3: (nvm_par_t3 as f32) / 281_474_976_710_656.0, // 2^48
            par_p1: ((nvm_par_p1 as f32) - 16_384.0) / 1_048_576.0, // 2^14 / 2^20
            par_p2: ((nvm_par_p2 as f32) - 16_384.0) / 536_870_912.0, // 2^14 / 2^29
            par_p3: (nvm_par_p3 as f32) / 4_294_967_296.0, // 2^32
            par_p4: (nvm_par_p4 as f32) / 137_438_953_472.0, // 2^37
            par_p5: (nvm_par_p5 as f32) / 0.125,           // 2^-3
            par_p6: (nvm_par_p6 as f32) / 64.0,            // 2^6
            par_p7: (nvm_par_p7 as f32) / 256.0,           // 2^8
            par_p8: (nvm_par_p8 as f32) / 32768.0,         // 2^15
            par_p9: (nvm_par_p9 as f32) / 281_474_976_710_656.0, //2^48
            par_p10: (nvm_par_p10 as f32) / 281_474_976_710_656.0, // 2^48
            par_p11: (nvm_par_p11 as f32) / 36_893_488_147_419_103_232.0, // 2^65
        }
    }
}
