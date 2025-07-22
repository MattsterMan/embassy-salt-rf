use defmt::{panic, *};
use defmt_rtt as _; // global logger
use embassy_futures::join::join;
use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usb, Peri};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use panic_probe as _;
use core::fmt::{Write};
use embassy_stm32::peripherals::USB;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, WaitResult};
use embassy_time::Timer;
use heapless::String;
use crate::{RADIO_RX_CHANNEL};

async fn usb_print<'a, T: Instance>(
    class: &mut CdcAcmClass<'a, Driver<'a, T>>,
    args: core::fmt::Arguments<'_>,
) {
    let mut buffer: String<256> = String::new(); // adjust size as needed
    let _ = buffer.write_fmt(args);
    
    // Send in chunks of 64 or less
    let mut i = 0;
    while i < buffer.len() {
        let end = (i + 64).min(buffer.len());
        let chunk = &buffer.as_bytes()[i..end];
        if let Err(e) = class.write_packet(chunk).await {
            warn!("USB write error: {:?}", e);
            break;
        }
        i = end;
    }

    // If last chunk was exactly 64 bytes, send a ZLP
    if buffer.len() % 64 == 0 {
        let _ = class.write_packet(&[]).await;
    }
}

// Handy macro
#[macro_export]
macro_rules! usb_write {
    ($usb:expr, $($arg:tt)*) => {
        usb_print($usb, format_args!($($arg)*)).await
    };
}

#[embassy_executor::task]
pub async fn usb_task(
    driver: Driver<'static, USB>,
) {
    // Create embassy-usb Config.
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("SALT-TX-BOARD");

    // Buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();
    
    let mut usb_sub = RADIO_RX_CHANNEL.subscriber().unwrap();

    // The echo task, which waits for a connection and then forwards sensor data.
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("USB Connected");

            loop {
                // wait for a new sensor packet to be received
                match usb_sub.next_message().await {
                    WaitResult::Message(packet) => {
                        info!("USB received new packet");
                        usb_write!(
                            &mut class,
                            "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\r\n",
                            packet.time,
                            packet.lsm_accel.x, packet.lsm_accel.y, packet.lsm_accel.z,
                            packet.gyro.x,    packet.gyro.y,    packet.gyro.z,
                            packet.mag.x, packet.mag.y, packet.mag.z,
                            packet.baro.pressure_mbar, packet.baro.temp_c,
                            packet.adxl_1.x,  packet.adxl_1.y,  packet.adxl_1.z,
                            packet.adxl_2.x,  packet.adxl_2.y,  packet.adxl_2.z,
                            packet.lis_1.x,   packet.lis_1.y,   packet.lis_1.z,
                            packet.lis_2.x,   packet.lis_2.y,   packet.lis_2.z,
                            packet.gps.status as char, packet.gps.utc_time, packet.gps.date,
                            packet.gps.latitude, packet.gps.longitude, packet.gps.course,
                            packet.gps.speed
                        );
                    }
                    WaitResult::Lagged(e) => {
                        warn!("USB Lagged {:?}", e);
                    }
                }
                
            }
        } 
    };
    // Run both the USB device and the echo task concurrently.
    join(usb_fut, echo_fut).await;
}
