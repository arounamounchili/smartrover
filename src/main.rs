#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

use core::sync::atomic::AtomicU32;
use embassy_executor::Spawner;
use embassy_net::{Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::mcpwm::operator::PwmPinConfig;
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::peripherals::{ADC1, MCPWM0};
use esp_hal::ram;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::{
    AuthenticationMethod, Config as WifiConfig, ControllerConfig, ap::AccessPointConfig,
    sta::StationConfig,
};
use log::{error, info};

use smartrover::adapters::{
    ControlTaskContext, SensorContext, SharedRoverState, control_task, sensor_task,
};
use smartrover::camera::{CameraContext, SharedFrame, camera_task};
use smartrover::config::WEB_WORKER_COUNT;
use smartrover::config::network::{AP_CHANNEL, AP_IP_OCTETS, DEFAULT_AP_PASS, DEFAULT_AP_SSID};
use smartrover::drivers::{Arducam, BatteryPin, MotorController};
use smartrover::network::{
    WebBuffers, WebContext, WifiModeInfo, connection_task, dhcp_task, net_task, web_task,
};

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // 160 MHz keeps startup and radio peak current modest on a single 1S LiPo cell.
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_160MHz);
    let peripherals = esp_hal::init(config);

    // Reclaim bootloader DRAM2 plus internal SRAM for esp-radio and esp-rtos allocations.
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 32 * 1024);

    // Start the esp-rtos scheduler and Embassy time driver on Core 0.
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    esp_println::logger::init_logger_from_env();

    const MCPWM_PERIOD_TICKS: u16 = 1999;
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
    let mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

    let mut timer0 = mcpwm.timer0;
    timer0.start(
        clock_cfg
            .timer_clock_with_frequency(
                MCPWM_PERIOD_TICKS,
                PwmWorkingMode::Increase,
                Rate::from_khz(16),
            )
            .unwrap(),
    );

    let mut oper_left = mcpwm.operator0;
    let mut oper_right = mcpwm.operator1;
    oper_left.set_timer(&timer0);
    oper_right.set_timer(&timer0);

    let (pwm_left_a, pwm_left_b) = oper_left.with_pins(
        peripherals.GPIO12,
        PwmPinConfig::UP_ACTIVE_HIGH,
        peripherals.GPIO11,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );
    let (pwm_right_a, pwm_right_b) = oper_right.with_pins(
        peripherals.GPIO14,
        PwmPinConfig::UP_ACTIVE_HIGH,
        peripherals.GPIO13,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    let motors = MotorController::new(
        pwm_left_a,
        pwm_left_b,
        pwm_right_a,
        pwm_right_b,
        MCPWM_PERIOD_TICKS,
    );
    let motors = mk_static!(MotorController<'static, MCPWM0<'static>, 0, 1>, motors);

    let led1 = Output::new(peripherals.GPIO3, Level::Low, Default::default());
    let led2 = Output::new(peripherals.GPIO4, Level::Low, Default::default());
    let led3 = Output::new(peripherals.GPIO5, Level::Low, Default::default());
    let led4 = Output::new(peripherals.GPIO6, Level::Low, Default::default());
    let leds = mk_static!([Output<'static>; 4], [led1, led2, led3, led4]);

    let state = &*mk_static!(SharedRoverState, SharedRoverState::new());
    let frame = &*mk_static!(SharedFrame, SharedFrame::new());
    let stream_session = &*mk_static!(AtomicU32, AtomicU32::new(0));

    let rng = esp_hal::rng::Rng::new();
    let seed = ((rng.random() as u64) << 32) | (rng.random() as u64);

    let mut i2c = match I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    ) {
        Ok(i2c) => i2c.with_sda(peripherals.GPIO8).with_scl(peripherals.GPIO9),
        Err(e) => {
            error!("CRITICAL: Failed to initialize I2C: {e:?}");
            core::future::pending::<()>().await;
            unreachable!();
        }
    };

    let mut spi_cam_hw = match Spi::new(
        peripherals.SPI2,
        SpiConfig::default().with_frequency(Rate::from_mhz(8)),
    ) {
        Ok(spi) => spi
            .with_sck(peripherals.GPIO36)
            .with_mosi(peripherals.GPIO35)
            .with_miso(peripherals.GPIO37),
        Err(e) => {
            error!("CRITICAL: Failed to initialize SPI: {e:?}");
            core::future::pending::<()>().await;
            unreachable!();
        }
    };
    let cam_cs = Output::new(peripherals.GPIO34, Level::High, Default::default());
    let mut arducam_hw = Arducam::new(cam_cs);

    // Initial hardware probe with staggered delay to mitigate startup brownout.
    Timer::after(Duration::from_millis(100)).await;
    info!("Initializing Arducam...");
    let camera_ready = match arducam_hw.init(&mut i2c, &mut spi_cam_hw) {
        Ok(_) => {
            info!("Arducam: OK");
            true
        }
        Err(e) => {
            error!("Arducam: FAIL ({e:?}); camera task disabled");
            false
        }
    };

    // Start Core 1 for dedicated camera SPI capture only when Arducam init succeeds.
    if camera_ready {
        let cpu1_stack = mk_static!(Stack<16384>, Stack::new());
        esp_rtos::start_second_core(
            peripherals.CPU_CTRL,
            sw_int.software_interrupt1,
            cpu1_stack,
            move || {
                let executor = mk_static!(
                    esp_rtos::embassy::Executor,
                    esp_rtos::embassy::Executor::new()
                );
                executor.run(|spawner| {
                    spawner.spawn(
                        camera_task(CameraContext {
                            arducam: arducam_hw,
                            spi: spi_cam_hw,
                            frame,
                        })
                        .unwrap(),
                    );
                });
            },
        );
    }

    let env_ssid = option_env!("WIFI_SSID").filter(|s| !s.is_empty());
    let env_pass = option_env!("WIFI_PASS").unwrap_or("");

    let wifi_initial_cfg = if let Some(ssid) = env_ssid {
        WifiConfig::Station(
            StationConfig::default()
                .with_ssid(ssid)
                .with_password(env_pass.into()),
        )
    } else {
        WifiConfig::AccessPoint(
            AccessPointConfig::default()
                .with_ssid(DEFAULT_AP_SSID)
                .with_password(DEFAULT_AP_PASS.into())
                .with_auth_method(AuthenticationMethod::Wpa2Personal)
                .with_channel(AP_CHANNEL)
                .with_max_connections(4),
        )
    };

    let (controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(wifi_initial_cfg),
    )
    .expect("Failed to create Wi-Fi controller");

    let (wifi_mode, stack, runner) = if let Some(ssid) = env_ssid {
        let config = embassy_net::Config::dhcpv4(Default::default());
        let (stack, runner) = embassy_net::new(
            interfaces.station,
            config,
            mk_static!(StackResources<6>, StackResources::<6>::new()),
            seed,
        );
        (WifiModeInfo::Station { ssid }, stack, runner)
    } else {
        let ap_ip = Ipv4Address::from_octets(AP_IP_OCTETS);
        let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(ap_ip, 24),
            gateway: Some(ap_ip),
            dns_servers: Default::default(),
        });
        let (stack, runner) = embassy_net::new(
            interfaces.access_point,
            config,
            mk_static!(StackResources<6>, StackResources::<6>::new()),
            seed,
        );
        (
            WifiModeInfo::SoftAp {
                ssid: DEFAULT_AP_SSID,
            },
            stack,
            runner,
        )
    };

    spawner.spawn(net_task(stack, runner).unwrap());
    spawner.spawn(connection_task(controller, wifi_mode, state).unwrap());
    if matches!(wifi_mode, WifiModeInfo::SoftAp { .. }) {
        spawner.spawn(dhcp_task(stack).unwrap());
    }

    let web_buffers = mk_static!(
        [WebBuffers; WEB_WORKER_COUNT],
        [const { WebBuffers::new() }; WEB_WORKER_COUNT]
    );
    for buf in web_buffers.iter_mut() {
        spawner.spawn(
            web_task(
                stack,
                WebContext {
                    state,
                    frame,
                    stream_session,
                },
                buf,
            )
            .unwrap(),
        );
    }

    spawner.spawn(
        control_task(ControlTaskContext {
            state,
            motors,
            leds,
        })
        .unwrap(),
    );

    // Initialize ADC1 after Wi-Fi PHY init so radio calibration does not disturb SAR registers.
    let mut adc1_config = AdcConfig::new();
    let batt_pin = adc1_config.enable_pin_with_cal::<_, AdcCalCurve<ADC1<'static>>>(
        peripherals.GPIO1,
        Attenuation::_11dB,
    );
    let adc1: Adc<'_, _, esp_hal::Blocking> = Adc::new(peripherals.ADC1, adc1_config);
    let batt_pin = mk_static!(BatteryPin, batt_pin);

    spawner.spawn(
        sensor_task(SensorContext {
            i2c,
            adc1,
            batt_pin,
            state,
        })
        .unwrap(),
    );

    core::future::pending::<()>().await;
}
