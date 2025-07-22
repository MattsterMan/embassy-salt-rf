use embassy_stm32::spi::{Spi, Error as SpiError};
use embassy_stm32::gpio::AnyPin;
use embedded_hal::digital::{InputPin, OutputPin};
use defmt::*;

/// LLCC68 API command opcodes (see Section 11.1)
pub mod opcode {
    pub const SET_STANDBY: u8          = 0x80;
    pub const SET_PACKET_TYPE: u8      = 0x8A;
    pub const SET_RF_FREQUENCY: u8     = 0x86;
    pub const SET_PA_CONFIG: u8        = 0x95;
    pub const SET_TX_PARAMS: u8        = 0x8E;
    pub const SET_BUFFER_BASE_ADDRESS: u8 = 0x8F;
    pub const WRITE_BUFFER: u8         = 0x0E;
    pub const SET_MODULATION_PARAMS: u8= 0x8B;
    pub const SET_PACKET_PARAMS: u8    = 0x8C;
    pub const SET_DIO_IRQ_PARAMS: u8   = 0x08;
    pub const WRITE_REGISTER: u8       = 0x0D;
    pub const READ_REGISTER: u8       = 0x1D;
    pub const SET_TX: u8               = 0x83;
    pub const SET_RX: u8               = 0x82;
    pub const GET_IRQ_STATUS: u8       = 0x12;
    pub const CLEAR_IRQ_STATUS: u8     = 0x02;
    pub const GET_RX_BUFFER_STATUS: u8 = 0x13;
    pub const READ_BUFFER: u8          = 0x1E;
    pub const GET_DEVICE_ERRORS: u8 = 0x17;
    pub const CLEAR_DEVICE_ERRORS: u8 = 0x07;
    pub const GET_STATUS: u8 = 0xC0;
    pub const GET_PACKET_STATUS: u8 = 0x14;
}

// maximum payload length for register setting
pub const MAX_PAYLOAD_LEN: usize = 8;

/// Module providing high‑level configuration types for the LLCC68 radio.
pub mod config {
    /// RF frequency in Hz. Valid range: 150_000_000 ..= 960_000_000
    #[derive(Debug, Clone, Copy)]
    pub struct RfFrequency(pub u32);

    impl RfFrequency {
        /// FRF = freq_hz * (2^25 / XTAL_HZ)
        /// Returns the 4‑byte big‑endian register value.
        pub fn to_bytes(self) -> [u8; 4] {
            const XTAL_HZ: u64 = 32_000_000;
            const FRF_SCALE: u64 = 1 << 25;   // 2^25

            // Compute full 32‑bit FRF register
            let frf: u64 = ((self.0 as u64) * FRF_SCALE) / XTAL_HZ;
            let frf32 = frf as u32;  // drop anything >32 bits

            [
                ((frf32 >> 24) & 0xFF) as u8,
                ((frf32 >> 16) & 0xFF) as u8,
                ((frf32 >>  8) & 0xFF) as u8,
                ((frf32 >>  0) & 0xFF) as u8,
            ]
        }
    }

    /// Packet type selection for LLCC68
    #[repr(u8)]
    #[derive(Debug, Clone, Copy)]
    pub enum PacketType {
        /// GFSK modulation
        Gfsk = 0x00,
        /// LoRa modulation
        LoRa = 0x01,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct PaConfig {
        /// Output power in dBm: –9…+22
        pub output_power: i8,
        /// Ramp time for SET_TX_PARAMS
        pub ramp_time: RampTime,
    }

    impl PaConfig {
        /// 2‑byte payload for SET_TX_PARAMS
        pub fn tx_params_bytes(&self) -> [u8; 2] {
            [(self.output_power & 0x1F) as u8, self.ramp_time as u8]
        }

        /// 4‑byte payload for SET_PA_CONFIG using the “optimal” table
        pub fn pa_config_bytes(&self) -> [u8; 4] {
            // pick the paDutyCycle & hpMax that match your power
            let (pa_duty, hp_max) = match self.output_power {
                22 => (0x04, 0x07),
                20 => (0x03, 0x05),
                17 => (0x02, 0x03),
                14 => (0x02, 0x02),
                p  => {
                    // default / lower‑power path (PA_BOOST)
                    let p = p.clamp(-9, 22) as u8;
                    (0x01, 0x01) // a safe generic duty/hp for any p
                }
            };
            [pa_duty, hp_max, 0x00, 0x01]
        }
    }


    /// Transmit ramp times.
    #[repr(u8)]
    #[derive(Debug, Clone, Copy)]
    pub enum RampTime {
        /// 10 µs
        Ramp10us = 0x00,
        /// 20 µs
        Ramp20us = 0x01,
        /// 40 µs
        Ramp40us = 0x02,
        /// 80 µs
        Ramp80us = 0x03,
        /// 200 µs
        Ramp200us = 0x04,
        /// 800 µs
        Ramp800us = 0x05,
        /// 1700 µs
        Ramp1700us = 0x06,
        /// 3400 µs
        Ramp3400us = 0x07,
    }

    /// Modulation parameters for LoRa.
    #[derive(Debug, Clone, Copy)]
    pub struct ModulationParams {
        pub spreading_factor: SpreadingFactor,
        pub bandwidth: Bandwidth,
        pub coding_rate: CodingRate,
        pub low_data_rate_optimize: bool,
    }

    impl ModulationParams {
        pub fn to_bytes(self) -> [u8; 4] {
            let sf = self.spreading_factor as u8;
            let bw = self.bandwidth as u8;
            let cr = self.coding_rate as u8;
            let ldro = if self.low_data_rate_optimize { 1 } else { 0 };
            [sf, bw, cr, ldro]
        }
    }

    /// LoRa spreading factors (SF5 .. SF12).
    #[repr(u8)]
    #[derive(Debug, Clone, Copy)]
    pub enum SpreadingFactor {
        Sf5  = 0x05,
        Sf6  = 0x06,
        Sf7  = 0x07,
        Sf8  = 0x08,
        Sf9  = 0x09,
        Sf10 = 0x0A,
        Sf11 = 0x0B,
        Sf12 = 0x0C,
    }

    /// Bandwidth options for LoRa.
    #[repr(u8)]
    #[derive(Debug, Clone, Copy)]
    pub enum Bandwidth {
        Bw125kHz = 0x04,
        Bw250kHz = 0x05,
        Bw500kHz = 0x06,
    }

    /// Coding rates for LoRa (4/5 .. 4/8).
    #[repr(u8)]
    #[derive(Debug, Clone, Copy)]
    pub enum CodingRate {
        Cr4_5 = 0x01,
        Cr4_6 = 0x02,
        Cr4_7 = 0x03,
        Cr4_8 = 0x04,
    }

    /// Packet parameters (preamble, header, payload, CRC, IQ inversion).
    #[derive(Debug, Clone, Copy)]
    pub struct PacketParams {
        pub preamble_length: u16,
        pub header_type: HeaderType,
        pub payload_length: u8,
        pub crc_on: bool,
        pub invert_iq: bool,
    }

    impl PacketParams {
        pub fn to_bytes(self) -> [u8; 7] {
            let pre = self.preamble_length.to_be_bytes();
            let hdr = self.header_type as u8;
            let payload = self.payload_length;
            let crc = if self.crc_on { 1 } else { 0 };
            let inv = if self.invert_iq { 1 } else { 0 };
            [pre[0], pre[1], hdr, payload, crc, inv, 0x00]
        }
    }

    /// Implicit or explicit header.
    #[repr(u8)]
    #[derive(Debug, Clone, Copy)]
    pub enum HeaderType {
        Explicit = 0x00,
        Implicit = 0x01,
    }
}

/// Bit positions in the 0x12/0x13 IRQ registers (Table 13‑29 LLCC68)
pub struct IrqStatus {
    pub tx_done:            bool, // bit 0
    pub rx_done:            bool, // bit 1
    pub preamble_detected:  bool, // bit 2
    pub sync_word_valid:    bool, // bit 3
    pub header_valid:       bool, // bit 4
    pub header_err:         bool, // bit 5
    pub crc_err:            bool, // bit 6
    pub cad_done:           bool, // bit 7
    pub cad_detected:       bool, // bit 8
    pub timeout:            bool, // bit 9
}

impl IrqStatus {
    pub fn from_bits(bits: u16) -> Self {
        Self {
            tx_done:           bits & (1 << 0) != 0,
            rx_done:           bits & (1 << 1) != 0,
            preamble_detected: bits & (1 << 2) != 0,
            sync_word_valid:   bits & (1 << 3) != 0,
            header_valid:      bits & (1 << 4) != 0,
            header_err:        bits & (1 << 5) != 0,
            crc_err:           bits & (1 << 6) != 0,
            cad_done:          bits & (1 << 7) != 0,
            cad_detected:      bits & (1 << 8) != 0,
            timeout:           bits & (1 << 9) != 0,
        }
    }

    /// Print any flags that are set.
    pub fn log(&self) {
        if self.tx_done           { info!("IRQ: TxDone"); }
        if self.rx_done           { info!("IRQ: RxDone"); }
        if self.preamble_detected { info!("IRQ: PreambleDetected"); }
        if self.sync_word_valid   { info!("IRQ: SyncWordValid"); }
        if self.header_valid      { info!("IRQ: HeaderValid"); }
        if self.header_err        { info!("IRQ: HeaderErr"); }
        if self.crc_err           { info!("IRQ: CrcErr"); }
        if self.cad_done          { info!("IRQ: CadDone"); }
        if self.cad_detected      { info!("IRQ: CadDetected"); }
        if self.timeout           { warn!("IRQ: Timeout"); }
    }
}

/// Bit positions for the “OpError” flags (Table 13‑85 LLCC68)
pub struct DeviceErrors {
    pub rc64k_calib_err: bool, // bit 0
    pub rc13m_calib_err: bool, // bit 1
    pub pll_calib_err:   bool, // bit 2
    pub adc_calib_err:   bool, // bit 3
    pub img_calib_err:   bool, // bit 4
    pub xosc_start_err:  bool, // bit 5
    pub pll_lock_err:    bool, // bit 6
    pub pa_ramp_err:     bool, // bit 8
}

impl DeviceErrors {
    pub fn from_bits(bits: u16) -> Self {
        Self {
            rc64k_calib_err: bits & (1 << 0) != 0,
            rc13m_calib_err: bits & (1 << 1) != 0,
            pll_calib_err:   bits & (1 << 2) != 0,
            adc_calib_err:   bits & (1 << 3) != 0,
            img_calib_err:   bits & (1 << 4) != 0,
            xosc_start_err:  bits & (1 << 5) != 0,
            pll_lock_err:    bits & (1 << 6) != 0,
            pa_ramp_err:     bits & (1 << 8) != 0,
        }
    }

    /// Logs any errors that are set
    pub fn log(&self) {
        let mut any = false;
        if self.rc64k_calib_err { warn!("DeviceError: RC64K_CALIB_ERR"); }
        if self.rc13m_calib_err { warn!("DeviceError: RC13M_CALIB_ERR"); }
        if self.pll_calib_err   { warn!("DeviceError: PLL_CALIB_ERR"); }
        if self.adc_calib_err   { warn!("DeviceError: ADC_CALIB_ERR"); }
        if self.img_calib_err   { warn!("DeviceError: IMG_CALIB_ERR"); }
        if self.xosc_start_err  { warn!("DeviceError: XOSC_START_ERR"); }
        if self.pll_lock_err    { warn!("DeviceError: PLL_LOCK_ERR"); }
        if self.pa_ramp_err     { warn!("DeviceError: PA_RAMP_ERR"); }
        
        if !any {
            info!("DeviceError: none");
        }
    }
}

/// The 3‑bit command status field from GetStatus (bits 3:1).
#[repr(u8)]
#[derive(Debug, Clone, Copy, Format)]
pub enum CommandStatus {
    None             = 0x0,
    DataAvailable    = 0x2,
    CmdTimeout       = 0x3,
    ProcError        = 0x4,
    Failure          = 0x5,
    CmdTxDone        = 0x6,
    Reserved(u8),
}

impl From<u8> for CommandStatus {
    fn from(v: u8) -> Self {
        match v & 0x07 {
            0x2 => CommandStatus::DataAvailable,
            0x3 => CommandStatus::CmdTimeout,
            0x4 => CommandStatus::ProcError,
            0x5 => CommandStatus::Failure,
            0x6 => CommandStatus::CmdTxDone,
            0x0 => CommandStatus::None,
            other => CommandStatus::Reserved(other),
        }
    }
}

/// The decoded GetStatus fields.
#[derive(Debug)]
pub struct Status {
    pub mode:        u8,
    pub cmd_status:  CommandStatus,
}

impl Status {
    pub fn from_byte(b: u8) -> Self {
        let mode   = (b >> 4) & 0x07;   // bits 6:4
        let cs_raw = (b >> 1) & 0x07;   // bits 3:1
        Status { mode, cmd_status: cs_raw.into() }
    }

    pub fn log(&self) {
        use defmt::info;
        let m = match self.mode {
            0x2 => "STBY_RC",
            0x3 => "STBY_XOSC",
            0x4 => "FS",
            0x5 => "RX",
            0x6 => "TX",
            _  => "RFU/Unused",
        };
        info!("  • Chip mode      = {}", m);
        info!("  • Command status = {}", self.cmd_status);
    }
}

/// Holds the values returned by GetPacketStatus
#[derive(Debug)]
pub struct PacketStatus {
    /// RSSI of the last received packet (in dBm)
    pub rssi: f32,
    /// SNR of the last received packet (in dB)
    pub snr: f32,
}
