//! rf.rs — LLCC68 driver using Embassy STM32 SPI

use defmt::{info, unwrap, warn};
use defmt::export::write;
use embassy_stm32::gpio::{Input, Output, Level, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::{peripherals, Peri};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::spi::{Spi, Error as SpiError};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pubsub::PubSubBehavior;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use heapless::String;
use postcard::to_slice;
use static_cell::StaticCell;
use crate::{SensorPacket, LATEST_PACKET, NEW_PACKET, RADIO_RX_CHANNEL};
use crate::can::TOTAL_BYTES;
use crate::rf::{opcode, config::*, MAX_PAYLOAD_LEN, IrqStatus, DeviceErrors, Status, PacketStatus, CommandStatus};

pub static IRQ_TRIG_CELL: StaticCell<Signal<NoopRawMutex, ()>> = StaticCell::new();

#[embassy_executor::task]
pub async fn led_task(
    dio1_trigger: &'static Signal<NoopRawMutex, ()>,
    mut led: Output<'static>,
) {
    // wait for a tx complete trigger (or timeout)
    if dio1_trigger.signaled() {
        // toggle led for 100ms
        led.set_low();
        Timer::after_millis(100).await;
        led.set_high();   
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
pub async fn radio_irq_task(
    mut dio1: ExtiInput<'static>,
    dio1_trigger: &'static Signal<NoopRawMutex, ()>,
) {
    loop {
        info!("Waiting for radio tx interrupt...");
        dio1.wait_for_rising_edge().await;
        info!("DIO1 IRQ Triggered");
        dio1_trigger.signal(());
    }
}

#[embassy_executor::task]
pub async fn radio_tx_task(
    mut radio: Llcc68<'static>,
    dio1_trigger: &'static Signal<NoopRawMutex, ()>,
) {
    let freq = RfFrequency(915_000_000);
    let pa_config = PaConfig {
        output_power: 22,
        ramp_time: RampTime::Ramp800us,
    };
    let mod_params = ModulationParams {
        spreading_factor: SpreadingFactor::Sf9,
        bandwidth: Bandwidth::Bw125kHz,
        coding_rate: CodingRate::Cr4_5,
        low_data_rate_optimize: false,
    };
    let pkt_params = PacketParams {
        preamble_length: 8,
        header_type: HeaderType::Explicit,
        payload_length: 12,
        crc_on: true,
        invert_iq: false,
    };
    static SYNC_WORD_CUSTOM: [u8; 2] = [0xAB, 0xCD];
    let sync_word = (0x0740, &SYNC_WORD_CUSTOM);

    // bit0 = TxDone, bit9 = Timeout
    let irq_mask:  u16 = 0x0201;
    let dio1_mask: u16 = 0x0201;

    if let Err(e) = radio.setup_lora_tx(freq, pa_config, mod_params, pkt_params, irq_mask, dio1_mask, sync_word).await {
        defmt::error!("Radio setup failed: {:?}", e);
        return;
    }

    let raw = radio.get_status().await.unwrap();
    info!("GET_STATUS = 0x{:02X}", raw);
    let st = Status::from_byte(raw);
    st.log();
    
    // Confirm sync word wrote correctly
    let mut read_back = [0u8; 2];
    if let Err(e) = radio.read_register(sync_word.0, &mut read_back).await {
        defmt::error!("Failed to read sync word back: {:?}", e);
    } else {
        info!("Sync word read back = {:02X}{:02X}", read_back[0], read_back[1]);
    }

    let _ = radio.clear_device_errors().await;

    // scratch buffer for serialization
    let mut tx_buf = [0u8; TOTAL_BYTES];

    loop {
        // wait for a new sensor data packet
        NEW_PACKET.wait().await;
        // grab out sensor packet
        let payload = {
            let guard = LATEST_PACKET.lock().await;
            guard.as_ref().expect("We signaled only after Some").clone()
        };
        info!("Pre-slice gyroZ: {}", payload.gyro.z);
        // slice SensorPacket into tx_buf via postcard
        let slice = to_slice(&payload, &mut tx_buf).unwrap();
        
        let pkt_params = PacketParams {
            preamble_length: 8,
            header_type: HeaderType::Explicit,
            payload_length: slice.len() as u8,
            crc_on: true,
            invert_iq: false,
        };
        
        // program packet length
        if let Err(e) = radio.set_packet_params(pkt_params).await {
            defmt::error!("Write buffer failed: {:?}", e);
            return;
        }
        
        // write the slice of the payload into fifo
        if let Err(e) = radio.write_buffer(0x00, slice).await {
            defmt::error!("Write buffer failed: {:?}", e);
            return;
        }
        
        let _ = radio.clear_irq_status(irq_mask).await; // Clear all before TX

        // drive RF switch into TX power mode
        radio.enter_tx();

        if let Err(e) = radio.set_tx(1000).await {
            defmt::error!("Failed to enter TX mode: {:?}", e);
            return;
        }
        
        // Wait for tx to finish or timeout to occur
        dio1_trigger.wait().await;
        
        // Check what mode we are in
        let raw = radio.get_status().await.unwrap();
        info!("GET_STATUS = 0x{:02X}", raw);
        let st = Status::from_byte(raw);
        st.log();
        
        // Turn off Tx/Rx switch until next loop
        radio.enter_idle();
        
        // no timer needed at the bottom since we wait on CAN
    }
}

#[embassy_executor::task]
pub async fn radio_rx_task(
    mut radio: Llcc68<'static>,
    dio1_trigger: &'static Signal<NoopRawMutex, ()>,
) {
    let freq = RfFrequency(915_000_000);
    let mod_params = ModulationParams {
        spreading_factor: SpreadingFactor::Sf9,
        bandwidth: Bandwidth::Bw125kHz,
        coding_rate: CodingRate::Cr4_5,
        low_data_rate_optimize: false,
    };
    let pkt_params = PacketParams {
        preamble_length: 8,
        header_type: HeaderType::Explicit,
        payload_length: 12,
        crc_on: true,
        invert_iq: false,
    };
    static SYNC_WORD_CUSTOM: [u8; 2] = [0xAB, 0xCD];
    let sync_word = (0x0740, &SYNC_WORD_CUSTOM);

    // bit1 = RxDone, bit9 = Timeout
    let irq_mask:  u16 = 0x0202;
    let dio1_mask: u16 = 0x0202;

    if let Err(e) = radio.setup_lora_rx(freq, mod_params, pkt_params, irq_mask, dio1_mask, sync_word).await {
        defmt::error!("Radio setup failed: {:?}", e);
        return;
    }

    let raw = radio.get_status().await.unwrap();
    info!("GET_STATUS = 0x{:02X}", raw);
    let st = Status::from_byte(raw);
    st.log();

    // Confirm sync word wrote correctly
    let mut read_back = [0u8; 2];
    if let Err(e) = radio.read_register(sync_word.0, &mut read_back).await {
        defmt::error!("Failed to read sync word back: {:?}", e);
    } else {
        info!("Sync word read back = {:02X}{:02X}", read_back[0], read_back[1]);
    }

    loop {
        info!("Waiting for a packet...");
        let _ = radio.clear_irq_status(0xFFFF).await; // Clear all before TX
        
        radio.enter_rx();  // steer RF switch into RX

        // set RX (0 = continuous, or pass ms for a timeout)
        if let Err(e) = radio.set_rx(0).await {
            defmt::error!("Failed to enter RX mode: {:?}", e);
            return;
        }

        // Wait for rx to finish or timeout to occur
        dio1_trigger.wait().await;

        let raw = radio.get_status().await.unwrap();
        info!("POST‑SETUP GET_STATUS = 0x{:02X}", raw);
        let st = Status::from_byte(raw);
        st.log();
        
        match st.cmd_status {
            CommandStatus::DataAvailable => {
                let ps = radio.get_packet_status().await.unwrap();
                info!("  RSSI={} dBm, SNR={} dB", ps.rssi, ps.snr);

                let (len, offset) = radio.get_rx_buffer_status().await.unwrap();
                info!("Packet Len = {}, Offset = {}", len, offset);
                let mut buf = [0u8; 256];
                radio.read_buffer(offset, &mut buf[..len as usize]).await.unwrap();
                
                match postcard::from_bytes::<SensorPacket>(&buf[..len as usize]) {
                    Ok(packet) => {
                        RADIO_RX_CHANNEL.publish_immediate(packet);
                    }
                    Err(_) => {
                        warn!("SensorPacket decode failed");
                    }
                }
                info!("Received {} bytes: {}", len, buf);
            }
            CommandStatus::CmdTimeout => {
                warn!("RX timeout");
            }
            other => {
                warn!("Unexpected command status: {:?}", other);
            }
        }
    }
}

pub struct Llcc68<'d> {
    spi: Spi<'d, Async>,
    cs: Output<'d>,
    busy: Input<'d>,
    rxen: Output<'d>,
    txen: Output<'d>,
}

impl<'d> Llcc68<'d> {
    pub fn new(
        spi: Spi<'d, Async>,
        cs: Peri<'d, peripherals::PA4>,
        busy: Peri<'d, peripherals::PB1>,
        rxen: Peri<'d, peripherals::PB12>,
        txen: Peri<'d, peripherals::PB13>,
    ) -> Self {
        let cs = Output::new(cs, Level::High, Speed::Low);
        let busy = Input::new(busy, Pull::None);

        // Default both low (RF bypass)
        let rxen = Output::new(rxen, Level::Low,  Speed::Low);
        let txen = Output::new(txen, Level::Low,  Speed::Low);

        Self { spi, cs, busy, rxen, txen }
    }

    async fn wait_busy(&self) {
        while self.busy.is_high() {}
        //info!("BUSY LOW");
    }

    async fn send_command(&mut self, cmd: u8, payload: &[u8]) -> Result<(), SpiError> {
        self.wait_busy().await;
        self.cs.set_low();

        self.spi.write(&[cmd]).await?;
        if !payload.is_empty() {
            self.spi.write(payload).await?;
        }

        self.cs.set_high();
        self.wait_busy().await;
        Ok(())
    }

    /// Read `len` bytes starting at `addr`.
    pub async fn read_register(
        &mut self,
        addr: u16,
        buf: &mut [u8],
    ) -> Result<(), SpiError> {
        let addr_bytes = addr.to_be_bytes();

        // 1) Wait for BUSY to go low
        self.wait_busy().await;

        // 2) Pull CS low and send READ_REGISTER + address
        self.cs.set_low();
        self.spi.write(&[opcode::READ_REGISTER, addr_bytes[0], addr_bytes[1]]).await?;

        // 1) throw away the status byte
        let mut status = [0u8; 1];
        self.spi.read(&mut status).await?;
        // 2) read exactly buf.len() bytes into the user buffer
        self.spi.read(buf).await?;

        // 4) Raise CS and wait BUSY again
        self.cs.set_high();
        self.wait_busy().await;

        Ok(())
    }

    pub async fn get_device_errors(&mut self) -> Result<u16, SpiError> {
        // 1) Wait until BUSY is low
        self.wait_busy().await;

        // 2) Assert chip‑select and send opcode
        self.cs.set_low();
        // 0x17 = GET_DEVICE_ERRORS
        self.spi.write(&[opcode::GET_DEVICE_ERRORS]).await?;

        // 3) Read back 4 bytes: RFU, Status, ErrMSB, ErrLSB
        let mut resp = [0u8; 4];
        self.spi.read(&mut resp).await?;
        self.cs.set_high();

        // 4) Extract the 2‑byte error field
        let err = u16::from_be_bytes([resp[2], resp[3]]);

        // (Optional) Check resp[1] – the Status byte – for other command flags

        Ok(err)
    }


    /// Clear _all_ recorded device errors (opcode 0x07, Table 13‑86)
    pub async fn clear_device_errors(&mut self) -> Result<(), SpiError> {
        self.wait_busy().await;
        self.cs.set_low();
        self.spi.write(&[opcode::CLEAR_DEVICE_ERRORS, 0x00, 0x00]).await?;
        self.cs.set_high();
        self.wait_busy().await;
        Ok(())
    }

    /// Read the 1‑byte status register via the GetStatus command (0xC0).
    pub async fn get_status(&mut self) -> Result<u8, SpiError> {
        self.wait_busy().await;
        self.cs.set_low();
        // Send GET_STATUS opcode
        self.spi.write(&[opcode::GET_STATUS]).await?;
        // Read back two bytes, second byte is the Status
        let mut status = [0u8; 2];
        self.spi.read(&mut status).await?;
        self.cs.set_high();
        Ok(status[1])
    }
    
    /// Read packet‐level stats after an RxDone.
    pub async fn get_packet_status(&mut self) -> Result<PacketStatus, SpiError> {
        self.wait_busy().await;
        self.cs.set_low();
        self.spi.write(&[opcode::GET_PACKET_STATUS]).await?;
        let mut buf = [0u8; 5];
        self.spi.read(&mut buf).await?;
        self.cs.set_high();
        info!("Raw packet status buffer: {}", buf);

        // buf = [RFU, Status, RssiPkt, SnrPkt, SignalRssiPkt]
        let rssi_pkt = buf[2];
        let snr_pkt  = buf[3] as i8;  // signed

        let rssi = -(rssi_pkt as f32) / 2.0;    // average RSSI in dBm
        let snr  = (snr_pkt  as f32) / 4.0;     // SNR in dB

        Ok(PacketStatus { rssi, snr, })
    }

    /// After an RxDone, returns (payload_length, rx_start_buffer_offset).
    pub async fn get_rx_buffer_status(&mut self) -> Result<(u8, u8), SpiError> {
        self.wait_busy().await;
        self.cs.set_low();
        // 0x13 = GET_RX_BUFFER_STATUS, then three NOPs
        self.spi.write(&[opcode::GET_RX_BUFFER_STATUS]).await?;

        // 3) Read back the four‑byte response
        let mut resp = [0u8; 4];
        self.spi.read(&mut resp).await?;
        self.cs.set_high();
        
        info!("Raw RX buffer status buffer: {}", resp);

        // resp = [ RFU, Status, PayloadLengthRx, RxStartBufferPointer ]
        let length = resp[1];
        let offset = resp[2];
        Ok((length, offset))
    }


    /// Read `buf.len()` bytes starting at the given buffer `offset`.
    pub async fn read_buffer(&mut self, offset: u8, buf: &mut [u8]) -> Result<(), SpiError> {
        // 1) Wait for BUSY low
        self.wait_busy().await;

        // 2) CS↓ + opcode + offset
        self.cs.set_low();
        self.spi.write(&[opcode::READ_BUFFER, offset]).await?;

        // 3) Discard the one RFU
        let mut _dummy = [0u8; 1];
        self.spi.read(&mut _dummy).await?;

        // 4) Now read the real payload
        self.spi.read(buf).await?;

        // 5) CS↑
        self.cs.set_high();

        Ok(())
    }

    /// Put RF switch into RX (high impedance on TX path)
    pub fn enter_rx(&mut self) {
        self.rxen.set_high();
        self.txen.set_low();
    }

    /// Put RF switch into TX (high impedance on RX path)
    pub fn enter_tx(&mut self) {
        self.txen.set_high();
        self.rxen.set_low();
    }

    /// Bypass both (idle/standby)
    pub fn enter_idle(&mut self) {
        self.rxen.set_low();
        self.txen.set_low();
    }

    pub async fn set_standby_rc(&mut self) -> Result<(), SpiError> {
        self.send_command(opcode::SET_STANDBY, &[0x00]).await
    }

    pub async fn set_standby_xosc(&mut self) -> Result<(), SpiError> {
        self.send_command(opcode::SET_STANDBY, &[0x01]).await
    }

    pub async fn set_packet_type(&mut self, pkt: PacketType) -> Result<(), SpiError> {
        self.send_command(opcode::SET_PACKET_TYPE, &[pkt as u8]).await
    }

    pub async fn set_rf_frequency(&mut self, freq: RfFrequency) -> Result<(), SpiError> {
        self.send_command(opcode::SET_RF_FREQUENCY, &freq.to_bytes()).await
    }

    pub async fn set_pa_config(&mut self, cfg: PaConfig) -> Result<(), SpiError> {
        self.send_command(opcode::SET_PA_CONFIG, &cfg.pa_config_bytes()).await
    }

    pub async fn set_tx_params(&mut self, cfg: PaConfig) -> Result<(), SpiError> {
        self.send_command(opcode::SET_TX_PARAMS, &cfg.tx_params_bytes()).await
    }

    pub async fn set_buffer_base(&mut self, tx_base: u8, rx_base: u8) -> Result<(), SpiError> {
        self.send_command(opcode::SET_BUFFER_BASE_ADDRESS, &[tx_base, rx_base]).await
    }

    pub async fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), SpiError> {
        let mut buf = [0u8; 256];
        buf[0] = offset;
        buf[1..=data.len()].copy_from_slice(data);
        self.send_command(opcode::WRITE_BUFFER, &buf[..=data.len()]).await
    }

    pub async fn set_modulation(&mut self, params: ModulationParams) -> Result<(), SpiError> {
        self.send_command(opcode::SET_MODULATION_PARAMS, &params.to_bytes()).await
    }

    pub async fn set_packet_params(&mut self, params: PacketParams) -> Result<(), SpiError> {
        self.send_command(opcode::SET_PACKET_PARAMS, &params.to_bytes()).await
    }

    pub async fn set_dio_irq(&mut self, irq_mask: u16, dio1_mask: u16) -> Result<(), SpiError> {
        let irq = irq_mask.to_be_bytes();
        let dio1 = dio1_mask.to_be_bytes();
        let dio2 = [0x00, 0x00]; // Not used
        let dio3 = [0x00, 0x00]; // Not used

        let mut payload = [0u8; 8];
        payload[0..2].copy_from_slice(&irq);
        payload[2..4].copy_from_slice(&dio1);
        payload[4..6].copy_from_slice(&dio2);
        payload[6..8].copy_from_slice(&dio3);

        self.send_command(opcode::SET_DIO_IRQ_PARAMS, &payload).await
    }


    /// Write `data` bytes starting at the 16‑bit register `addr`.
    pub async fn write_register(&mut self, addr: u16, data: &[u8]) -> Result<(), SpiError> {
        let mut payload = [0u8; 2 + MAX_PAYLOAD_LEN]; // MAX_PAYLOAD_LEN ≥ your largest data.len()
        let addr_bytes = addr.to_be_bytes();
        payload[0] = addr_bytes[0];
        payload[1] = addr_bytes[1];
        payload[2..2 + data.len()].copy_from_slice(data);

        // Single SPI transaction:
        self.wait_busy().await;
        self.cs.set_low();
        // 1-byte opcode + (2addr + data.len()) payload
        self.spi.write(&[opcode::WRITE_REGISTER]).await?;
        self.spi.write(&payload[..2 + data.len()]).await?;
        self.cs.set_high();
        self.wait_busy().await;
        Ok(())
    }

    /// Start a TX.  
    /// - `timeout_ms = 0` ⇒ disable timeout (single‑mode TX).  
    /// - otherwise ⇒ HW timeout = timeout_ms × 64 (15.625µs steps).
    pub async fn set_tx(&mut self, timeout_ms: u32) -> Result<(), SpiError> {
        // 1) Compute the 24‑bit “steps” value
        let steps = if timeout_ms == 0 {
            0u32
        } else {
            // convert ms→steps (1ms = 64 steps), clamp to 0x00FF_FFFF
            let s = timeout_ms.saturating_mul(64);
            s.min(0x00FF_FFFF)
        };
        let b = steps.to_be_bytes(); // [b3, b2, b1, b0]

        // 2) Wait until BUSY is low
        self.wait_busy().await;

        // 3) Assert CS and send opcode + 3‑byte timeout
        self.cs.set_low();
        self.spi.write(&[opcode::SET_TX, b[1], b[2], b[3]]).await?;
        self.cs.set_high();

        Ok(())
    }

    /// Start an RX session.
    ///
    /// - `timeout_ms == 0` ⇒ single‑mode RX (no timeout; returns to STBY_RC on RxDone)
    /// - `timeout_ms == u32::MAX` ⇒ continuous RX (0xFFFFFF; never times out)
    /// - otherwise ⇒ timeout_ms → steps (ms * 64) in 15.625 µs units, clamped to 0x00FF_FFFF
    pub async fn set_rx(&mut self, timeout_ms: u32) -> Result<(), SpiError> {
        // 1) Compute the 24‑bit timeout field
        let steps = if timeout_ms == 0 {
            0u32
        } else if timeout_ms == u32::MAX {
            0x00FF_FFFF
        } else {
            let s = timeout_ms.saturating_mul(64);
            s.min(0x00FF_FFFF)
        };
        let b = steps.to_be_bytes(); // [b3, b2, b1, b0]

        // 2) Wait until BUSY is low
        self.wait_busy().await;

        // 3) Assert CS and send SET_RX + 3‑byte timeout
        self.cs.set_low();
        self.spi.write(&[opcode::SET_RX, b[1], b[2], b[3]]).await?;
        self.cs.set_high();

        Ok(())
    }


    pub async fn get_irq_status(&mut self) -> Result<u16, SpiError> {
        self.wait_busy().await;
        self.cs.set_low();
        self.spi.write(&[opcode::GET_IRQ_STATUS]).await?;
        let mut buf = [0u8; 2];
        self.spi.read(&mut buf).await?;
        self.cs.set_high();
        Ok(u16::from_be_bytes(buf))
    }

    pub async fn clear_irq_status(&mut self, flags: u16) -> Result<(), SpiError> {
        self.send_command(opcode::CLEAR_IRQ_STATUS, &flags.to_be_bytes()).await
    }

    /// Full TX initialization routine
    pub async fn setup_lora_tx(
        &mut self,
        freq: RfFrequency,
        pa: PaConfig,
        mod_params: ModulationParams,
        pkt_params: PacketParams,
        irq_mask: u16,
        dio1: u16,
        sync_word: (u16, &[u8; 2]),
    ) -> Result<(), SpiError> {
        self.set_standby_rc().await?;
        self.set_packet_type(PacketType::LoRa).await?;
        self.set_modulation(mod_params).await?;
        self.set_packet_params(pkt_params).await?;
        self.set_rf_frequency(freq).await?;
        self.set_pa_config(pa).await?;
        self.set_tx_params(pa).await?;
        self.set_buffer_base(0x00, 0x00).await?;
        self.set_dio_irq(irq_mask, dio1).await?;
        self.write_register(sync_word.0, sync_word.1).await?;
        self.enter_idle();
        Ok(())
    }

    /// Full RX initialization routine
    pub async fn setup_lora_rx(
        &mut self,
        freq: RfFrequency,
        mod_params: ModulationParams,
        pkt_params: PacketParams,
        irq_mask: u16,
        dio1: u16,
        sync_word: (u16, &[u8; 2]),
    ) -> Result<(), SpiError> {
        self.set_standby_xosc().await?;
        // self.set_standby_rc().await?;
        self.set_packet_type(PacketType::LoRa).await?;
        self.set_modulation(mod_params).await?;
        self.set_packet_params(pkt_params).await?;
        self.set_rf_frequency(freq).await?;
        self.set_buffer_base(0x00, 0x00).await?;
        self.set_dio_irq(irq_mask, dio1).await?;
        self.write_register(sync_word.0, sync_word.1).await?;
        self.enter_idle();
        Ok(())
    }
}
