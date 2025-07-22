use defmt::*;
use embassy_stm32::{bind_interrupts, can};
use embassy_stm32::can::{BufferedCanRx, CanRx, CanTx, Frame, Id, Rx1InterruptHandler};
use embassy_stm32::peripherals::CAN;
use embassy_time::Timer;
use postcard::from_bytes;
use crate::{SensorPacket, LATEST_PACKET, NEW_PACKET};

const BASE_ID:     u16   = 0x300;          // first frame’s ID
pub const TOTAL_BYTES: usize = 125;            // fixed serialized size
const SEGMENTS:    usize = (TOTAL_BYTES + 7) / 8; // == 16

#[embassy_executor::task]
pub async fn can_rx_task(mut rx: BufferedCanRx<'static, 16>) {
    let mut buf = [0u8; TOTAL_BYTES];

    loop {
        // ————————————————————————————————————————————
        // 1) Wait for the *first* frame (segment 0) to start a new packet
        // ————————————————————————————————————————————
        info!("Waiting for a CAN message...");
        let envelope = rx.read().await.unwrap();
        let (frame, _ts) = envelope.parts();

        // Only proceed if this is the first segment (ID == BASE_ID)
        let raw0 = match frame.id() {
            Id::Standard(sid) => sid.as_raw(),
            Id::Extended(eid) => eid.as_raw() as u16,
        };
        if raw0 != BASE_ID {
            // out-of-sync: ignore and try again
            continue;
        }

        // Copy bytes 0–7 into buf[0..8]
        let data0 = frame.data();
        buf[0..data0.len()].copy_from_slice(data0);

        // ————————————————————————————————————————————
        // 2) Read the remaining SEGMENTS−1 frames
        // ————————————————————————————————————————————
        for _ in 1..SEGMENTS {
            let envelope = rx.read().await.unwrap();
            let (frame, _) = envelope.parts();
            
            // Compute this segment’s index from its ID
            let raw = match frame.id() {
                Id::Standard(sid) => sid.as_raw(),
                Id::Extended(eid) => eid.as_raw() as u16,
            };
            let idx = raw.wrapping_sub(BASE_ID) as usize;  // segment number

            // Guard in case something weird happens
            if idx >= SEGMENTS {
                // bad segment → abort this packet
                break;
            }

            // Copy into buf[idx*8 .. idx*8 + data.len()]
            let data = frame.data();
            let start = idx * 8;
            let end   = core::cmp::min(start + data.len(), TOTAL_BYTES);
            buf[start..end].copy_from_slice(&data[..end - start]);
        }

        // ————————————————————————————————————————————
        // 3) Deserialize once the full 125 bytes are in `buf`
        // ————————————————————————————————————————————
        let packet: SensorPacket = from_bytes(&buf).unwrap();
        info!("Got packet @ {} ms", packet.time);
        info!("Packet Control AccelZ: {}", packet.lsm_accel.z);
        {
            let mut guard = LATEST_PACKET.lock().await;
            *guard = Some(packet);
        }
        NEW_PACKET.signal(());  // signal to the rf task we have new data
    }
}

#[embassy_executor::task]
pub async fn can_loopback_test(mut tx: CanTx<'static>, mut rx: CanRx<'static>) {
    // give the bus a moment to settle
    Timer::after_millis(10).await;

    // prepare a single 8-byte frame
    const TEST_ID: u16 = 0x123;
    let payload = [0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44];
    
    loop {
        let frame = Frame::new_standard(TEST_ID, &payload).unwrap();

        // transmit
        info!("Loopback test: sending frame ID=0x{:X}", TEST_ID);
        tx.write(&frame).await;
        info!("Frame sent, awaiting loopback…");

        // receive it back
        let envelope = rx.read().await.unwrap();
        let (rx_frame, _ts) = envelope.parts();
        let rx_id = match rx_frame.id() {
            Id::Standard(sid) => sid.as_raw(),
            Id::Extended(eid)  => eid.as_raw() as u16,
        };

        info!("Loopback received ID=0x{:X}, data={:?}", rx_id, rx_frame.data());

        // you can now assert (in logs) that rx_id == TEST_ID && rx_frame.data() == payload   
    }
}