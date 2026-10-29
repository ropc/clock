#![no_std]
#![no_main]

use core::{cell::RefCell, net::{IpAddr, SocketAddr}, str::FromStr};

use cyw43::{Control, JoinOptions, aligned_bytes};
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_net::{Config, Stack, StackResources, dns::DnsQueryType, udp::PacketMetadata};
use embassy_rp::{
    bind_interrupts, clocks::RoscRng, dma,
    gpio::{Level, Output}, i2c::{self, I2c},
    peripherals::{DMA_CH0, I2C0, PIO0},
    pio::{self, Pio},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal, watch::Watch};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer };
use chrono::{DateTime, Datelike, Timelike, Weekday};
use embedded_hal_bus::i2c::RefCellDevice;
use periph::chips::light::Apds9960Full;
use four_letter_phat_rs::FourLetterPhat;
use sntpc::{NtpContext, get_time};
use sntpc_net_embassy::UdpSocketWrapper;
use sntpc_time_embassy::EmbassyTimestampGenerator;
use embassy_net::udp::UdpSocket;
use thiserror::Error;
use static_cell::StaticCell;
use crate::ClockState::TimeSynced;

use {defmt_rtt as _, panic_probe as _};

const NETWORK_SSID: &str = env!("NETWORK_SSID");
const NETWORK_PASSWORD: &str = env!("NETWORK_PASSWORD");
const TIMEZONE: &str = env!("TIMEZONE");
const NTP_SERVER: &str = "pool.ntp.org";
const NTP_SYNC_DELAY_SECONDS: u64 = 60 * 60 * 12;  // 12 hours in seconds
const AMBIENT_LIGHT_BRIGHT_THRESHOLD: u16 = 20;
const AMBIENT_LIGHT_DARK_THRESHOLD: u16 = 3;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

static I2C_STATIC_CELL: StaticCell<I2c<'static, I2C0, i2c::Async>> = StaticCell::new();
static I2C_REF_CELL: StaticCell<RefCell<&mut I2c<'static, I2C0, i2c::Async>>> = StaticCell::new();
static CLOCK_STATE: Watch<CriticalSectionRawMutex, ClockState, 1> = Watch::new_with(ClockState::PoweredOn);
static AMBIENT_LIGHT_STATE: Watch<CriticalSectionRawMutex, AmbientLightState, 2> = Watch::new();
static WAKEUP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[derive(Debug, Format, Clone, Copy, Default, PartialEq, Eq)]
enum AmbientLightState {
    #[default]
    BRIGHT,
    DARK,
}

#[derive(Debug, Format, Clone, Copy)]
enum ClockState {
    PoweredOn,
    WifiConnected,
    TimeSynced(u64),  // stores offset
}

#[derive(Error, Debug, Format)]
enum AppError {
    #[error("Empty DNS response")]
    EMPTY_DNS,
    #[error("DNS error")]
    DNS_ERROR(embassy_net::dns::Error),
}

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

    let scl = p.PIN_1;
    let sda = p.PIN_0;

    let i2c = I2C_STATIC_CELL.init(I2c::new_async(p.I2C0, scl, sda, Irqs, Default::default()));
    let i2c_ref_cell = I2C_REF_CELL.init(RefCell::new(i2c));

    let mut delay = Delay {};
    let sensor = Apds9960Full::new(RefCellDevice::new(i2c_ref_cell), 0x39, &mut delay).unwrap();
    let displays = [
        FourLetterPhat::new(RefCellDevice::new(i2c_ref_cell), 0x70).unwrap(),
        FourLetterPhat::new(RefCellDevice::new(i2c_ref_cell), 0x72).unwrap(),
        FourLetterPhat::new(RefCellDevice::new(i2c_ref_cell), 0x71).unwrap(),
    ];

    unwrap!(spawner.spawn(display_task(displays)));

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
            Timer::after_secs(1).await;
        }

        info!("waiting for link...");
        stack.wait_link_up().await;
        
        info!("waiting for DHCP...");
        stack.wait_config_up().await;
        
        // And now we can use it!
        info!("Stack is up!");
        CLOCK_STATE.sender().send(ClockState::WifiConnected);

        loop {
            let mut rx_meta = [PacketMetadata::EMPTY; 16];
            let mut rx_buffer = [0; 4096];
            let mut tx_meta = [PacketMetadata::EMPTY; 16];
            let mut tx_buffer = [0; 4096];
            let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
            socket.bind(123).unwrap();
            let socket = UdpSocketWrapper::from(socket);
            
            let ntp_context = NtpContext::new(EmbassyTimestampGenerator::default());

            let Ok(ntp_addrs) = stack
                .dns_query(NTP_SERVER, DnsQueryType::A)
                .await
                .map_err(|e: embassy_net::dns::Error| AppError::DNS_ERROR(e))
                .and_then(|addr| if addr.is_empty() { Err(AppError::EMPTY_DNS) } else { Ok(addr) })
            else {
                warn!("failed to resolve ntp server ip");
                continue;
            };
            let addr: IpAddr = ntp_addrs[0].into();
            let result = get_time(SocketAddr::from((addr, 123)), &socket, ntp_context).await;

            match result {
                Ok(time) => {
                    info!("time is {}", time);
                    CLOCK_STATE.sender().send(ClockState::TimeSynced(time.offset().try_into().expect("ntp offset should be positive")));
                },
                Err(e) => {
                    error!("ntp error: {}", e);
                    let is_synced = if let Some(TimeSynced(_)) = CLOCK_STATE.try_get() { true } else { false }
                    if !is_synced {
                        Timer::after_secs(1).await;
                        continue;
                    }
                },
            };

            Timer::after_secs(NTP_SYNC_DELAY_SECONDS).await;
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

fn format_weekday(weekday: Weekday) -> &'static str {
    match weekday {
        Weekday::Sun => "Sun",
        Weekday::Mon => "Mon",
        Weekday::Tue => "Tue",
        Weekday::Wed => "Wed",
        Weekday::Thu => "Thu",
        Weekday::Fri => "Fri",
        Weekday::Sat => "Sat",
    }
}

fn format_month(month: u32) -> &'static str {
    match month {
        1 => "Jan",
        2 => "Feb",
        3 => "Mar",
        4 => "Apr",
        5 => "May",
        6 => "Jun",
        7 => "Jul",
        8 => "Aug",
        9 => "Sep",
        10 => "Oct",
        11 => "Nov",
        12 => "Dec",
        _ => "  ",
    }
}

#[embassy_executor::task]
async fn display_task(displays: [FourLetterPhat<RefCellDevice<'static, &'static mut I2c<'static, I2C0, i2c::Async>>>; 3]) {
    let mut set_decimal = true;
    let timezone = chrono_tz::Tz::from_str(TIMEZONE).expect("invalid timezone");
    let mut wakeup_timeout: Option<u64> = None;
    let mut clock_state_receiver = CLOCK_STATE.receiver().unwrap();
    let mut ticker = Ticker::every(Duration::from_secs(1));

    let [mut display0, mut display1, mut display2] = displays;

    // show that display is powered on
    display0.set_decimal(0).unwrap();

    loop {
        let state = match select3(clock_state_receiver.changed(), ticker.next(), WAKEUP_SIGNAL.wait()).await {
            Either3::First(state) => state,
            Either3::Second(_) => {
                set_decimal = !set_decimal;
                clock_state_receiver.get().await
            },
            Either3::Third(_) => {
                info!("wakeup received");
                // timeout 3 seconds from now
                wakeup_timeout = Some(Instant::now().as_micros() + 3_000_000);
                clock_state_receiver.get().await
            },
        };

        match state {
            ClockState::PoweredOn => {
                // initial state; everything else will be off
                display0.set_decimal(0).unwrap();
            },
            ClockState::WifiConnected => {
                // same here, nothing besides the first decimal will have been written to
                display0.set_decimal(0).unwrap();
                display0.set_decimal(1).unwrap();
            },
            ClockState::TimeSynced(offset) => {
                let now = Instant::now().as_micros();

                display0.clear_decimal(0).unwrap();

                let ambient_light;
                if let Some(timeout) = wakeup_timeout && now < timeout {
                    ambient_light = AmbientLightState::BRIGHT;
                } else {
                    wakeup_timeout = None;
                    ambient_light = AMBIENT_LIGHT_STATE.try_get().unwrap_or_default();
                }

                match ambient_light {
                    AmbientLightState::DARK => {
                        display0.clear_display().unwrap();
                        display1.clear_display().unwrap();
                        display2.clear_display().unwrap();
                    },
                    AmbientLightState::BRIGHT => {
                        let now = DateTime::from_timestamp_micros((now + offset).try_into().unwrap()).unwrap();
                        let now = now.with_timezone(&timezone);

                        // time display
                        display0.set_digit(0, to_char(now.hour() / 10)).unwrap();
                        display0.set_digit(1, to_char(now.hour() % 10)).unwrap();

                        display0.set_digit(2, to_char(now.minute() / 10)).unwrap();
                        display0.set_digit(3, to_char(now.minute() % 10)).unwrap();

                        if set_decimal {
                            display0.set_decimal(1).unwrap();
                        } else {
                            display0.clear_decimal(1).unwrap();
                        }

                        // date displays
                        let weekday = format_weekday(now.weekday());
                        let month = format_month(now.month());

                        for (pos, char) in weekday.chars().chain(month.chars()).enumerate() {
                            let display = if pos < 4 { &mut display1 } else { &mut display2 };
                            display.set_digit(pos as u8 % 4, char).unwrap();
                        }

                        display2.set_digit(2, to_char(now.day() / 10)).unwrap();
                        display2.set_digit(3, to_char(now.day() % 10)).unwrap();
                    }
                }
            }
        }
    }
}

// #[embassy_executor::task]
async fn human_interface_task(mut sensor: Apds9960Full<RefCellDevice<'_, &mut I2c<'_, I2C0, i2c::Async>>>) -> ! {
    // TODO: implement time-sharing bc gesture engine and ambient light detection
    // use the same hardware components

    // sensor.enable_gesture(true).unwrap();
    // sensor.enable_gesture_interrupt(true).unwrap();

    sensor.enable_proximity(true).unwrap();
    // sensor.enable_proximity_interrupt(true).unwrap();

    let mut wakeup_signaled = false;

    loop {
        // defmt::info!("Blink");
        // Timer::after(Duration::from_millis(20)).await;
        Timer::after_millis(50).await;

        let mut gesture_data_buf = [(0, 0, 0, 0); 1];
        
        let gesture_data = sensor.read_gesture_fifo(&mut gesture_data_buf).ok();
        let proximity_data = sensor.proximity().ok();
        let light_data = sensor.color_clear().ok();

        // info!("gesture data: {}\tproximity data: {}\tlight data: {}", gesture_data, proximity_data, light_data);

        if let Some(value) = proximity_data {
            if value > 5 && !wakeup_signaled {
                WAKEUP_SIGNAL.signal(());
                wakeup_signaled = true;
            }
            if value <= 5 && wakeup_signaled {
                wakeup_signaled = false;
            }
        }

        if let Some(current_ambient_light) = light_data {
            let change_to_state = if current_ambient_light > AMBIENT_LIGHT_BRIGHT_THRESHOLD {
                Some(AmbientLightState::BRIGHT)
            } else if current_ambient_light < AMBIENT_LIGHT_DARK_THRESHOLD {
                Some(AmbientLightState::DARK)
            } else {
                None
            };

            let current_state = AMBIENT_LIGHT_STATE.try_get().unwrap_or_default();

            if let Some(new_state) = change_to_state && new_state != current_state {
                AMBIENT_LIGHT_STATE.sender().send(new_state);
            }
        }
    }
}
