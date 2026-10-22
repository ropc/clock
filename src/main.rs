#![no_std]
#![no_main]

use core::cell::RefCell;

use cyw43::{Control, JoinOptions, aligned_bytes};
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::{Config, Stack, StackResources};
use embassy_rp::{
    bind_interrupts, clocks::RoscRng, dma,
    gpio::{Level, Output}, i2c::{self, I2c},
    peripherals::{DMA_CH0, I2C0, PIO0},
    pio::{self, Pio},
};
use embassy_time::{Delay, Duration, Instant, Timer};
use chrono::{DateTime, Timelike};
use embedded_hal_bus::i2c::RefCellDevice;
use periph::chips::light::Apds9960Full;
use four_letter_phat_rs::{Dimming, Display, FourLetterPhat};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const NETWORK_SSID: &str = env!("NETWORK_SSID");
const NETWORK_PASSWORD: &str = env!("NETWORK_PASSWORD");

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

static I2C_STATIC_CELL: StaticCell<I2c<'static, I2C0, i2c::Async>> = StaticCell::new();
static I2C_REF_CELL: StaticCell<RefCell<&mut I2c<'static, I2C0, i2c::Async>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download ./cyw43-firmware/43439A0.bin --binary-format bin --chip RP235x --base-address 0x10100000
    //     probe-rs download ./cyw43-firmware/43439A0_clm.bin --binary-format bin --chip RP235x --base-address 0x10140000
    // let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    // let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let fw = aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        // SPI communication won't work if the speed is too high, so we use a divider larger than `DEFAULT_CLOCK_DIVIDER`.
        // See: https://github.com/embassy-rs/embassy/issues/3960.
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH0, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = Config::dhcpv4(Default::default());
    // debug!("default dhcpv4 config: {}", config);
    let mut rng = RoscRng;
    let seed = rng.next_u64();

    // Init network stack
    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    unwrap!(spawner.spawn(net_task(runner)));
    unwrap!(spawner.spawn(time_sync_task(control, stack)));

    let scl = p.PIN_13;
    let sda = p.PIN_12;

    let i2c = I2C_STATIC_CELL.init(I2c::new_async(p.I2C0, scl, sda, Irqs, Default::default()));
    let i2c_ref_cell = I2C_REF_CELL.init(RefCell::new(i2c));

    let mut delay = Delay {};
    let sensor = Apds9960Full::new(RefCellDevice::new(i2c_ref_cell), 0x39, &mut delay).unwrap();
    let display = FourLetterPhat::new(RefCellDevice::new(i2c_ref_cell), 0x70).unwrap();

    

    unwrap!(spawner.spawn(display_task(display)));

    human_interface_task(sensor).await;
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn time_sync_task(mut control: Control<'static>, stack: Stack<'static>) {
    loop {
        while let Err(err) = control
            .join(NETWORK_SSID, JoinOptions::new(NETWORK_PASSWORD.as_bytes()))
            .await
        {
            info!("join failed: {:?}", err);
        }

        info!("waiting for link...");
        stack.wait_link_up().await;

        info!("waiting for DHCP...");
        stack.wait_config_up().await;

        // And now we can use it!
        info!("Stack is up!");

        loop {
            Timer::after_secs(10).await;
            info!("network still up: {}", stack.is_link_up());
        }
    }
}

fn to_char(num: u32) -> char {
    // defmt::info!("converting {} to char", num);
    match num {
        1 => '1',
        2 => '2',
        3 => '3',
        4 => '4',
        5 => '5',
        6 => '6',
        7 => '7',
        8 => '8',
        9 => '9',
        _ => '0',
    }
}

#[embassy_executor::task]
async fn display_task(mut display: FourLetterPhat<RefCellDevice<'static, &'static mut I2c<'static, I2C0, i2c::Async>>>) {
    let mut set_decimal = true;
    // TODO: since display.set_digit() clears the display before setting the new one, there is a
    // flicker when updating the display, since it goes display '0' -> blank -> display '1'
    // a hacky solution would be to keep previous state and only change the digits that are needed
    // a better solution would be to figure out the minimum segment change for the digit and execute those

    // let start = Instant::now();
    let offset: u64 = 1792679109000;
    loop {
        let now = Instant::now().as_millis() + offset;
        let now = DateTime::from_timestamp_millis(now.try_into().unwrap()).unwrap();
        let now = now.with_timezone(&chrono_tz::US::Eastern);

        // defmt::info!("now is {}", now);

        // display.set_dimming(Dimming::BRIGHTNESS_MAX).unwrap();
        display.set_digit(0, to_char(now.hour() / 10)).unwrap();
        display.set_digit(1, to_char(now.hour() % 10)).unwrap();

        display.set_digit(2, to_char(now.minute() / 10)).unwrap();
        display.set_digit(3, to_char(now.minute() % 10)).unwrap();
        
        if set_decimal {
            display.set_decimal(1).unwrap();
        } else {
            display.clear_decimal(1).unwrap();
        }

        Timer::after_secs(1).await;
        set_decimal = !set_decimal;
    }
}
// #[embassy_executor::task]
async fn human_interface_task(mut sensor: Apds9960Full<RefCellDevice<'_, &mut I2c<'_, I2C0, i2c::Async>>>) -> ! {
    // TODO: implement time-sharing bc gesture engine and ambient light detection
    // use the same hardware components

    // sensor.enable_gesture(true).unwrap();
    // sensor.enable_gesture_interrupt(true).unwrap();

    sensor.enable_proximity(true).unwrap();
    sensor.enable_proximity_interrupt(true).unwrap();

    loop {
        // defmt::info!("Blink");
        // Timer::after(Duration::from_millis(20)).await;
        Timer::after(Duration::from_millis(100)).await;

        let mut gesture_data_buf = [(0, 0, 0, 0); 1];
        
        let gesture_data = sensor.read_gesture_fifo(&mut gesture_data_buf).map_or_default(|_| Option::Some(gesture_data_buf));
        let proximity_data = sensor.proximity().unwrap_or_default();
        let light_data = sensor.color_clear().map_or_default(|level| Option::Some(level));
        
        // defmt::info!("gesture: {:?}\tprox: {:?}\tlight: {:?}", gesture_data, proximity_data, light_data);
    }
}
