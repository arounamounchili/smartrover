//! 20 Hz sensor polling task for IMU (BMI323), ToF (VL53L0X), and battery ADC.

use crate::adapters::SharedRoverState;
use crate::config::control::{TICK_RATE_MS, TOF_CLEAR_DISTANCE_MM};
use crate::drivers::{BatteryMonitor, BatteryPin, Bmi323, DistanceFilter, Vl53l0x};
use embassy_time::{Duration, Ticker, Timer};
use esp_hal::Blocking;
use esp_hal::analog::adc::Adc;
use esp_hal::i2c::master::I2c;
use esp_hal::peripherals::ADC1;
use log::{error, info, warn};

pub struct SensorContext {
    pub i2c: I2c<'static, Blocking>,
    pub adc1: Adc<'static, ADC1<'static>, Blocking>,
    pub batt_pin: &'static mut BatteryPin,
    pub state: &'static SharedRoverState,
}

#[embassy_executor::task]
pub async fn sensor_task(mut ctx: SensorContext) {
    let imu = Bmi323::new();
    let mut tof = Vl53l0x::new();
    let mut dist_filter = DistanceFilter::new();
    let mut battery = BatteryMonitor::new();

    // Stagger sensor initialization to avoid I2C bus contention and peak current spikes
    Timer::after(Duration::from_millis(150)).await;
    info!("Initializing IMU...");
    let mut imu_healthy = match imu.init(&mut ctx.i2c).await {
        Ok(()) => {
            info!("IMU: OK");
            true
        }
        Err(e) => {
            error!("IMU: FAIL ({e:?})");
            false
        }
    };

    Timer::after(Duration::from_millis(100)).await;
    info!("Initializing ToF...");
    let mut tof_healthy = match tof.init(&mut ctx.i2c).await {
        Ok(()) => {
            info!("ToF: OK");
            true
        }
        Err(e) => {
            error!("ToF: FAIL ({e:?})");
            false
        }
    };

    let mut ticker = Ticker::every(Duration::from_millis(TICK_RATE_MS));
    let mut tick_count = 0u32;
    let mut last_ax = 0.0;
    let mut last_ay = 0.0;
    let mut last_az = 1.0;
    let mut last_dist_mm = TOF_CLEAR_DISTANCE_MM;
    let mut last_batt_mv = 0u16;
    let mut last_batt_pct = 100i32;
    let mut imu_err_streak = 0u8;
    let mut tof_err_streak = 0u8;

    loop {
        if imu_healthy {
            match imu.read_accel(&mut ctx.i2c) {
                Ok((ax, ay, az)) => {
                    last_ax = ax;
                    last_ay = ay;
                    last_az = az;
                    imu_err_streak = 0;
                }
                Err(_) => {
                    imu_err_streak = imu_err_streak.saturating_add(1);
                    if imu_err_streak >= 4 {
                        imu_healthy = false;
                        warn!("IMU communication lost");
                    }
                }
            }
        } else if tick_count.is_multiple_of(40) && imu.init(&mut ctx.i2c).await.is_ok() {
            imu_healthy = true;
            imu_err_streak = 0;
            info!("IMU recovered");
        }

        if tof_healthy {
            match tof.poll_distance(&mut ctx.i2c) {
                Ok(Some(dist)) => {
                    last_dist_mm = dist_filter.update(dist);
                    tof_err_streak = 0;
                }
                Ok(None) => {
                    tof_err_streak = tof_err_streak.saturating_add(1);
                    if tof_err_streak >= 10 {
                        tof_healthy = false;
                        warn!("ToF measurement stalled");
                    }
                }
                Err(_) => {
                    tof_err_streak = tof_err_streak.saturating_add(2);
                    if tof_err_streak >= 10 {
                        tof_healthy = false;
                        warn!("ToF communication lost");
                    }
                }
            }
        } else if tick_count % 40 == 20 && tof.init(&mut ctx.i2c).await.is_ok() {
            tof_healthy = true;
            tof_err_streak = 0;
            dist_filter.reset();
            last_dist_mm = TOF_CLEAR_DISTANCE_MM;
            info!("ToF recovered");
        }

        ctx.state.publish_imu_and_tof(
            last_dist_mm,
            last_ax,
            last_ay,
            last_az,
            tof_healthy,
            imu_healthy,
        );

        if tick_count.is_multiple_of(10) {
            let (v_batt, pct) = battery.read_voltage_filtered(&mut ctx.adc1, ctx.batt_pin);
            last_batt_mv = (v_batt * 1000.0 + 0.5) as u16;
            last_batt_pct = pct;
            ctx.state.publish_battery(pct);
        }

        if tick_count.is_multiple_of(100) {
            info!(
                "Heartbeat: Dist={}mm (tof_ok={}, imu_ok={}), Batt={}mV ({}%), Heap={}/{}KB free",
                last_dist_mm,
                tof_healthy,
                imu_healthy,
                last_batt_mv,
                last_batt_pct,
                esp_alloc::HEAP.free() / 1024,
                (esp_alloc::HEAP.used() + esp_alloc::HEAP.free()) / 1024
            );
        }

        tick_count = tick_count.wrapping_add(1);
        ticker.next().await;
    }
}
