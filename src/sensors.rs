use defmt::Format;
use serde::{Deserialize, Serialize};

/// Sensors file for data types
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct AccelData{
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct GyroData{
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct MagData{
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Ms5611Sample {
    // pressure measured in millibars
    pub pressure_mbar: f32,
    // temperature in degrees celcius
    pub temp_c: f32,
}

#[derive(Debug, Format, Clone, Default, Serialize, Deserialize)]
pub struct GpsRmc {
    /// UTC time as hhmmss (fractional seconds dropped), e.g., 123519.
    pub utc_time: u32,
    /// Date as ddmmyy, e.g., 230394.
    pub date: u32,
    /// Data validity: 'A' = valid, 'V' = invalid.
    pub status: u8,
    /// Latitude in signed decimal degrees (positive for North, negative for South).
    pub latitude: f32,
    /// Longitude in signed decimal degrees (positive for East, negative for West).
    pub longitude: f32,
    /// Speed over ground in knots.
    pub speed: f32,
    /// Course over ground in degrees.
    pub course: f32,
}