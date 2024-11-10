#![deny(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![no_std]
#![no_main]

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver};

use defmt::{error, info, println, unwrap};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::config::Config;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2C_Config, InterruptHandler as I2C_IH};
use embassy_rp::peripherals::{I2C0, USB};
use embassy_rp::spi::{Config as SPI_Config, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use embassy_usb::class::midi::MidiClass;
use embassy_usb::{Builder, Config as USB_Config};
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

#[embassy_executor::task]
async fn midi_runner(
    mut class: MidiClass<'static, Driver<'static, USB>>,
    keys: Receiver<'static, NoopRawMutex, u8, 16>,
) {
    let mut buf = [0; 64];

    loop {
        class.wait_connection().await;
        info!("Connected");
        loop {
            match embassy_futures::select::select(class.read_packet(&mut buf), keys.receive()).await
            {
                Either::First(ret_read) => match ret_read {
                    Ok(n) => info!("New: {:02x}", &buf[0..n]),
                    Err(_) => break,
                },
                Either::Second(key) => {
                    buf[0] = 0x09;
                    buf[1] = 0x90;
                    println!("Key {} press {}", key & 0xF, key & 0xF0);
                    buf[2] = key & 0x0F; // + 0x3C;
                    buf[3] = if key & 0xF0 != 0x00 { 0x7F } else { 0x00 };

                    // let pos = class.read_packet(&mut buf).await.unwrap();
                    let data = &buf[0..4];
                    info!("data: {:x}", data);
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
        .unwrap();

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = USB_Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-MIDI example");
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

    let keys: Channel<NoopRawMutex, u8, 16> = embassy_sync::channel::Channel::new();

    let keys = KEY_CHAN.init(keys);

    unwrap!(spawner.spawn(midi_runner(class, keys.receiver())));

    let mut led_status = false;
    let mut lvl = 0xFF;

    let mut tick = Timer::after_millis(10);

    let mut i2c_input: [u8; 2] = [0xFF, 0xFF];
    let mut old: u16 = 0xFFFF;

    loop {
        match select(&mut tick, key_int.wait_for_low()).await {
            Either::First(()) => {
                tick = Timer::after_millis(500);
                let led_0: u32 = if led_status {
                    0xFF00_0008_u32.to_be()
                } else {
                    LED_DEFAULT
                };

                led_buff[1] = led_0;
                led_buff[2] = u32::from_le_bytes([0xFF, lvl, 0x00, 0x00]);
                led_buff[3] = u32::from_le_bytes([0xFF, 0x00, lvl, 0x00]);
                led_buff[4] = u32::from_le_bytes([0xFF, 0x00, 0x00, lvl]);
                led_buff[8] = u32::from_le_bytes([0xFF, 0x00, lvl, lvl]);
                led_buff[12] = u32::from_le_bytes([0xFF, lvl, 0x00, lvl]);
                led_buff[16] = u32::from_le_bytes([0xFF, lvl, lvl, 0x00]);

                lvl = lvl.wrapping_add(16);
                led_status = !led_status;
            }
            Either::Second(()) => {
                let _ = i2c
                    .write_read_async(
                        u16::from(tca9555::ADDR),
                        [tca9555::Reg::InputP0 as u8],
                        &mut i2c_input,
                    )
                    .await;

                let inp: u16 = u16::from_ne_bytes(i2c_input);

                let diff = inp ^ old;
                for bit in 0..16_u16 {
                    if diff & (1 << bit) != 0x00 {
                        let data = (bit as u8) | if inp & (1 << bit) == 0x00 { 0x80 } else { 0x00 };

                        println!("Key {} press {}", data & 0xF, data & 0xF0);
                        if keys.try_send(data).is_err() {
                            error!("Error Send");
                        };

                        let pos: usize = 1 + usize::from(bit);

                        let led_0: u32 = if inp & (1 << bit) == 0x00 {
                            0xFF00_4000_u32.to_be()
                        } else {
                            LED_DEFAULT
                        };
                        led_buff[pos] = led_0;
                    }
                }

                old = inp;

                tick = Timer::after_secs(5);
                lvl = 0;
            }
        }

        let buf: [u8; BUF_BYTE_SIZE] = unsafe { core::mem::transmute(led_buff) };
        spi.write(&buf).await.unwrap();
    }
}
