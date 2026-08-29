use libm::powf;
use uom::si::{
    f32::{Length, Pressure, ThermodynamicTemperature},
    length::{foot, meter},
    pressure::{hectopascal, pascal},
    thermodynamic_temperature::degree_celsius,
};

/// A single measurement from the [`Bmp390`] barometer.
///
/// Measurements utilize the [`uom`] crate to provide automatic, type-safe, and zero-cost units of measurement.
///
/// # Example
/// ```
/// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
/// # use uom::si::pressure::pascal;
/// # use uom::si::length::meter;
/// # use uom::si::thermodynamic_temperature::degree_celsius;
/// let measurement = bmp390::Measurement {
///    pressure: Pressure::new::<pascal>(90_240.81),
///    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
///    altitude: Length::new::<meter>(1000.0),
/// };
///
/// defmt::info!("Measurement: {}", measurement);
/// ```
///
/// Note: these examples show creation of [`Measurement`] structs directly. In practice you would receive these from
/// [`Bmp390::measure`].
///
/// Conversion between units is easy with the [`uom`] crate. For example, to convert to imperial units:
/// ```
/// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
/// # use uom::si::pressure::pascal;
/// # use uom::si::length::meter;
/// # use uom::si::thermodynamic_temperature::degree_celsius;
/// # let measurement = bmp390::Measurement {
/// #    pressure: Pressure::new::<pascal>(90_240.81),
/// #    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
/// #    altitude: Length::new::<meter>(1000.0),
/// # };
/// use uom::si::pressure::millimeter_of_mercury;
/// use uom::si::thermodynamic_temperature::degree_fahrenheit;
/// use uom::si::length::foot;
///
/// // "Pressure: 676.9753 mmHg, Temperature: 77 °F, Altitude: 3280.84 feet"
/// defmt::info!("Pressure: {} mmHg, temperature: {} °F, altitude: {} feet",
///     measurement.pressure.get::<millimeter_of_mercury>(),
///     measurement.temperature.get::<degree_fahrenheit>(),
///     measurement.altitude.get::<foot>());
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// The pressure as a [`Pressure`], allowing for easy conversion to any unit of pressure.
    ///
    /// # Example
    /// ```
    /// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
    /// # use uom::si::pressure::pascal;
    /// # use uom::si::length::meter;
    /// # use uom::si::thermodynamic_temperature::degree_celsius;
    /// use uom::si::pressure::millimeter_of_mercury;
    /// let measurement = bmp390::Measurement {
    ///    pressure: Pressure::new::<pascal>(90_240.81),
    ///    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
    ///    altitude: Length::new::<meter>(1000.0),
    /// };
    ///
    /// // "Pressure: 676.9753 mmHg"
    /// defmt::info!("Pressure: {} mmHg", measurement.pressure.get::<millimeter_of_mercury>());
    /// ```
    pub pressure: Pressure,

    /// The temperature as a [`ThermodynamicTemperature`], allowing for easy conversion to any unit of temperature.
    ///
    /// # Example
    /// ```
    /// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
    /// # use uom::si::pressure::pascal;
    /// # use uom::si::length::meter;
    /// # use uom::si::thermodynamic_temperature::degree_celsius;
    /// use uom::si::thermodynamic_temperature::degree_fahrenheit;
    /// let measurement = bmp390::Measurement {
    ///    pressure: Pressure::new::<pascal>(90_240.81),
    ///    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
    ///    altitude: Length::new::<meter>(1000.0),
    /// };
    ///
    /// // "Temperature: 77 °F"
    /// defmt::info!("Temperature: {} °F", measurement.temperature.get::<degree_fahrenheit>());
    /// ```
    pub temperature: ThermodynamicTemperature,

    /// The altitude as a [`Length`], allowing for easy conversion to any unit of length.
    ///
    /// # Example
    /// ```
    /// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
    /// # use uom::si::pressure::pascal;
    /// # use uom::si::length::meter;
    /// # use uom::si::thermodynamic_temperature::degree_celsius;
    /// use uom::si::length::foot;
    /// let measurement = bmp390::Measurement {
    ///    pressure: Pressure::new::<pascal>(90_240.81),
    ///    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
    ///    altitude: Length::new::<meter>(1000.0),
    /// };
    ///
    /// // "Length: 3280.84 feet"
    /// defmt::info!("Length: {} feet", measurement.altitude.get::<foot>());
    /// ```
    pub altitude: Length,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Measurement {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Pressure: {} Pa, Temperature: {} °C, Altitude: {} m",
            self.pressure.get::<pascal>(),
            self.temperature.get::<degree_celsius>(),
            self.altitude.get::<meter>()
        );
    }
}

impl core::fmt::Display for Measurement {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "Pressure: {} Pa, Temperature: {} °C, Altitude: {} m",
            self.pressure.get::<pascal>(),
            self.temperature.get::<degree_celsius>(),
            self.altitude.get::<meter>(),
        )
    }
}

/// Calculate the altitude based on the pressure, sea level pressure, and the reference altitude.
///
/// The altitude is calculating following the [NOAA formula](https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf).
pub fn calculate_altitude(pressure: Pressure, altitude_reference: Length) -> Length {
    let sea_level = Pressure::new::<hectopascal>(1013.25);
    let above_sea_level =
        Length::new::<foot>(145366.45 * (1.0 - powf((pressure / sea_level).value, 0.190284)));

    above_sea_level - altitude_reference
}
