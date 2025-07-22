#![no_std]
#![no_main]

mod rf;
mod usb_serial;
mod rfdriver;
mod can;
mod sensors;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, spi, usb, Config};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::time::{mhz, Hertz};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::can::{Can, Fifo, Rx0InterruptHandler, Rx1InterruptHandler, RxBuf, SceInterruptHandler, StandardId, TxBuf, TxInterruptHandler};
use embassy_stm32::can::Fifo::Fifo0;
use embassy_stm32::can::filter::{BankConfig, Mask32};
use embassy_stm32::can::util::{calc_can_timings, NominalBitTiming};
use embassy_stm32::peripherals::{CAN, USB};
use embassy_stm32::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use serde::{Deserialize, Serialize};
use crate::can::{can_rx_task};
use crate::rfdriver::{led_task, radio_irq_task, radio_rx_task, radio_tx_task, Llcc68, IRQ_TRIG_CELL};
use crate::usb_serial::*;
use crate::sensors::*;
use embassy_stm32::rcc::*;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;

enum BoardRole {
    Tx,
    Rx,
}
///CHANGE THIS FOR EACH BOARD TYPE ---------------------------------
const ROLE: BoardRole = BoardRole::Tx;

// Combine all sensor data into one struct for packetization
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct SensorPacket {
    pub time: u64,
    pub lsm_accel: AccelData,
    pub gyro: GyroData,
    pub mag: MagData,
    pub baro: Ms5611Sample,
    pub adxl_1: AccelData,
    pub adxl_2: AccelData,
    pub lis_1: AccelData,
    pub lis_2: AccelData,
    pub gps: GpsRmc,
}

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => Rx0InterruptHandler<CAN>, InterruptHandler<USB>;
    CAN1_RX1        => Rx1InterruptHandler<CAN>;
    CAN1_SCE        => SceInterruptHandler<CAN>;
    USB_HP_CAN1_TX  => TxInterruptHandler<CAN>;
});

pub static RADIO_RX_CHANNEL: PubSubChannel<CriticalSectionRawMutex, SensorPacket, 4, 1, 1> = PubSubChannel::new();
// mutex to store the latest received sensor packet from CAN
pub static LATEST_PACKET: Mutex<CriticalSectionRawMutex, Option<SensorPacket>> = Mutex::new(None);
// signal to notify rf task when a new packet is ready to transmit
pub static NEW_PACKET: Signal<CriticalSectionRawMutex, ()> = Signal::new();
// buffer to store CAN frames
static RX_BUF: StaticCell<RxBuf<16>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll = Some(Pll {
        src: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL9,  // 72 Mhz
    });
    config.rcc.sys = Sysclk::PLL1_P; // 72 Mhz
    config.rcc.ahb_pre = AHBPrescaler::DIV1; // 72 Mhz
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 36 Mhz
    config.rcc.apb2_pre = APBPrescaler::DIV1; // 72 Mhz

    let mut p = embassy_stm32::init(config);

    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(1);

    let spi = spi::Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, spi_config);

    // Disable JTAG so that PA15 is usable
    embassy_stm32::pac::AFIO.mapr().modify(|w| {
        w.set_swj_cfg(0b0000_0001); // Change to 0b0000_0010 to disable JTAG and SWD, but enable the use of LED1 (blue)
    });

    let led1 = Output::new(p.PA15, Level::High, Speed::Low);
    let mut led2 = Output::new(p.PB6, Level::High, Speed::Low);
    
    // Set up radio
    let radio = Llcc68::new(spi, p.PA4, p.PB1, p.PB12, p.PB13);  // CS on PA4, BUSY on PB1
    let dio1 = ExtiInput::new(p.PA3, p.EXTI3, Pull::Down);
    let mut radio_rst = Output::new(p.PB0, Level::High, Speed::Medium);
    let sig = IRQ_TRIG_CELL.init(Signal::new());
    
    // Reset module before initializing
    radio_rst.set_low();
    Timer::after_millis(100).await;
    radio_rst.set_high();
    // Wait for PLL to stabilize
    Timer::after_millis(100).await;

    // can configuration
    embassy_stm32::pac::AFIO.mapr().modify(|w| w.set_can1_remap(2)); // Set alternate pin mapping to B8/B9
    let mut can = Can::new(p.CAN, p.PB8, p.PB9, Irqs);
    let mut can_stby = Output::new(p.PC13, Level::Low, Speed::Medium);  // set low to enable CAN
    can_stby.set_low();
    can.modify_filters().enable_bank(0, Fifo0, Mask32::accept_all());
    can.modify_config()
        .set_bitrate(250_000);
    can.set_automatic_wakeup(true);  // wake up module on packet start sensed
    info!("Enabling CAN");
    can.enable().await;
    let rx_buf = RX_BUF.init(RxBuf::new());
    // split into tx/rx
    let (tx, rx) = can.split();
    let rx = rx.buffered(rx_buf);
    
    match ROLE {
        BoardRole::Tx => {
            info!("Starting interrupt task");
            spawner
                .spawn(radio_irq_task(dio1, sig))
                .unwrap();
            info!("Starting tx task");
            spawner
                .spawn(radio_tx_task(radio, sig))
                .unwrap();
            info!("Starting CAN task");
            spawner.spawn(can_rx_task(rx)).unwrap();
            // spawn led tx confirmation task
            // spawner.spawn(led_task(sig, led1)).unwrap();
        }
        BoardRole::Rx => {
            spawner
                .spawn(radio_irq_task(dio1, sig))
                .unwrap();
            spawner
                .spawn(radio_rx_task(radio, sig))
                .unwrap();
            // spawn led rx confirmation task
            // spawner.spawn(led_task(sig, led1)).unwrap();
        }
    }

    // USB driver and task
    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);
    spawner.spawn(usb_task(driver)).unwrap();  // usb serial printing

    loop {
        // Show the program is running with led2
        led2.set_high();
        Timer::after_millis(500).await;
        led2.set_low();
        Timer::after_millis(500).await;
    }
}