#![deny(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use defmt::{error, info, println, unwrap, Format};

use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either, Either3};
use embassy_rp::bind_interrupts;
use embassy_rp::config::Config;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2C_Config, InterruptHandler as I2C_IH};
use embassy_rp::peripherals::{I2C0, USB};
use embassy_rp::spi::{Config as SPI_Config, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::Timer;
use embassy_usb::class::midi::MidiClass;
use embassy_usb::{Builder, Config as USB_Config};
use embedded_hal_async::i2c::I2c;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2C_IH<I2C0>;
    USBCTRL_IRQ => InterruptHandler<USB>;
});

const LED_DEFAULT: u32 = 0x0101_01FF;
const LED_NUM: usize = 16;

const BUF_NUM: usize = LED_NUM + 2;
const BUF_BYTE_SIZE: usize = BUF_NUM * 4;

mod tca9555 {
    pub const ADDR: u8 = 0x20;

    #[allow(unused)]
    #[repr(u8)]
    pub enum Reg {
        InputP0 = 0x00,
        InputP1,
        OutputP0,
        OutputP1,
        PolInvP0,
        PolInvP1,
        CfgP0,
        CfgP1,
    }
}

#[embassy_executor::task]
async fn usb_runner(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    loop {
        info!("init usb runner");
        usb.run().await;
    }
}

#[derive(Format)]
pub struct Event {
    key: u8,
    velos: u8,
}

#[derive(Format)]
pub enum MidiEvent {
    NoteOn(Event),
    NoteOff(Event),
}

#[embassy_executor::task]
async fn midi_runner(
    mut class: MidiClass<'static, Driver<'static, USB>>,
    send_keys: Receiver<'static, NoopRawMutex, u8, 16>,
    midi_events: Sender<'static, NoopRawMutex, MidiEvent, 4>,
) {
    let mut buf = [0; 32];

    loop {
        class.wait_connection().await;
        info!("Connected");
        loop {
            match embassy_futures::select::select(class.read_packet(&mut buf), send_keys.receive())
                .await
            {
                Either::First(ret_read) => match ret_read {
                    Ok(n) => {
                        let data = &buf[0..n];
                        // info!("New: {:02x}", data);
                        for data in data.chunks_exact(4) {
                            let event: Option<MidiEvent> = match data[0] & 0x0F {
                                0x08 => Some(MidiEvent::NoteOff(Event {
                                    key: data[2],
                                    velos: data[3],
                                })),
                                0x09 => Some(MidiEvent::NoteOn(Event {
                                    key: data[2],
                                    velos: data[3],
                                })),
                                _ => None,
                            };
                            if let Some(event) = event {
                                // info!("Send Event: {}", event);
                                if midi_events.try_send(event).is_err() {
                                    error!("Can´t send midi events");
                                }
                            }
                        }
                    }
                    Err(_) => break,
                },
                Either::Second(key) => {
                    buf[0] = 0x09;
                    buf[1] = 0x90;
                    // println!("Key {:02x} press {:02x}", key & 0xF, key & 0xF0);
                    buf[2] = key & 0x0F; // + 0x3C;
                    buf[3] = if key & 0xF0 != 0x00 { 0x7F } else { 0x00 };

                    // let pos = class.read_packet(&mut buf).await.unwrap();
                    let data = &buf[0..4];
                    // info!("data: {:02x}", data);
                    if class.write_packet(data).await.is_err() {
                        break;
                    };
                }
            };
        }
        info!("Disconnect");
    }
}

// Create embassy-usb DeviceBuilder using the driver and config.
// It needs some buffers for building the descriptors.
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
// static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();

static KEY_CHAN: StaticCell<Channel<NoopRawMutex, u8, 16>> = StaticCell::new();
static EVENTS_CHAN: StaticCell<Channel<NoopRawMutex, MidiEvent, 4>> = StaticCell::new();

#[derive(Clone, Copy)]
enum LedMode {
    Idle,
    Midi((u8, bool)),
}

impl PartialEq for LedMode {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Midi(_l0), Self::Midi(_r0)) => true,
            _ => core::mem::discriminant(self) == core::mem::discriminant(other),
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // I2C pins
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let mut key_int = Input::new(p.PIN_3, Pull::None);

    // SPI pins
    let spi_mosi = p.PIN_19;
    let spi_clk = p.PIN_18;
    let _spi_cs = Output::new(p.PIN_17, Level::Low);

    info!("set up spi");
    let mut spi = Spi::new_txonly(p.SPI0, spi_clk, spi_mosi, p.DMA_CH1, SPI_Config::default());

    // LED buffer with start and end frame
    let mut led_buff: [u32; BUF_NUM] = [
        // First u32 = start
        0x0000_0000, // Led data = 0b111 + 5-bit bright, 3x 8-bit B G R
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        LED_DEFAULT,
        // Last u32 = end
        0xFFFF_FFFF,
    ];

    let buf: [u8; BUF_BYTE_SIZE] = unsafe { core::mem::transmute(led_buff) };
    spi.write(&buf).await.unwrap();

    info!("set up i2c ");
    let mut i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, I2C_Config::default());

    info!("init tca9555 config for keyboard");
    // // All pins as input
    i2c.write(tca9555::ADDR, &[tca9555::Reg::CfgP0 as u8, 0xFF, 0xFF])
        .await
        .expect("Should able to talk to GPIO");

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = USB_Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-MIDI PICO RGB KEYPAD");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [], // no msos descriptors
        CONTROL_BUF.init([0; 128]),
    );

    // Create classes on the builder.
    let class = MidiClass::new(&mut builder, 1, 1, 64);

    // Build the builder.
    let usb = builder.build();

    // Run the USB device.
    unwrap!(spawner.spawn(usb_runner(usb)));
    // Timer::after_millis(10).await;

    let keys = embassy_sync::channel::Channel::new();
    let events = embassy_sync::channel::Channel::new();

    let keys = KEY_CHAN.init(keys);
    let events = EVENTS_CHAN.init(events);

    unwrap!(spawner.spawn(midi_runner(class, keys.receiver(), events.sender())));

    let mut led_status = false;
    let mut lvl = 0xFF;

    let mut tick = Timer::after_millis(10);

    let mut i2c_input: [u8; 2] = [0xFF, 0xFF];
    let mut old: u16 = 0xFFFF;

    let mut led_mode = LedMode::Idle;
    let mut led_old_mode = LedMode::Idle;

    loop {
        match select3(key_int.wait_for_low(), &mut tick, events.receive()).await {
            Either3::First(()) => {
                let _ = i2c
                    .write_read_async(
                        u16::from(tca9555::ADDR),
                        [tca9555::Reg::InputP0 as u8],
                        &mut i2c_input,
                    )
                    .await;

                let inp: u16 = u16::from_ne_bytes(i2c_input);

                let mut diff = inp ^ old;
                while diff != 0 {
                    #[allow(clippy::cast_possible_truncation)]
                    let bit = diff.trailing_zeros() as u8;

                    let mask = 1 << bit;

                    diff ^= mask;

                    let is_pressed = inp & mask == 0x00;

                    let data = bit | if is_pressed { 0x80 } else { 0x00 };

                    // println!("Key {:02x} press {:02x}", data & 0xF, data & 0xF0);
                    if keys.try_send(data).is_err() {
                        error!("Error Send");
                    };
                }

                old = inp;
            }
            Either3::Second(()) => {
                led_mode = LedMode::Idle;
            }
            Either3::Third(event) => {
                let (led, is_pressed) = match event {
                    MidiEvent::NoteOn(event) => (event.key, true),
                    MidiEvent::NoteOff(event) => (event.key, false),
                };

                led_mode = LedMode::Midi((led, is_pressed));
            }
        }

        if led_mode != led_old_mode {
            println!("Led mode changed!");
            let last = led_buff.len() - 1;
            for led in &mut led_buff[1..last] {
                *led = LED_DEFAULT;
            }
            lvl = 0;
        }

        match led_mode {
            LedMode::Idle => {
                // led_buff[1] = led_0;
                led_buff[2] = u32::from_le_bytes([0xFF, lvl, 0x00, 0x00]);
                led_buff[3] = u32::from_le_bytes([0xFF, 0x00, lvl, 0x00]);
                led_buff[4] = u32::from_le_bytes([0xFF, 0x00, 0x00, lvl]);
                led_buff[8] = u32::from_le_bytes([0xFF, 0x00, lvl, lvl]);
                led_buff[12] = u32::from_le_bytes([0xFF, lvl, 0x00, lvl]);
                led_buff[16] = u32::from_le_bytes([0xFF, lvl, lvl, 0x00]);

                lvl = lvl.wrapping_add(16);
                led_status = !led_status;

                tick = Timer::after_millis(250);
            }
            LedMode::Midi((key, value)) => {
                let pos = 1 + usize::from(key & 0x0f);
                led_buff[pos] = if value { 0x0111_01FF } else { LED_DEFAULT };
                tick = Timer::after_millis(5000);
            }
        }

        let buf: [u8; BUF_BYTE_SIZE] = unsafe { core::mem::transmute(led_buff) };
        spi.write(&buf).await.unwrap();

        led_old_mode = led_mode;
    }
}
