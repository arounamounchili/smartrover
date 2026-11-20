//! 20 Hz control loop task.

use crate::adapters::SharedRoverState;
use crate::adapters::leds::GpioLedAdapter;
use crate::config::control::{TICK_RATE_MS, WATCHDOG_TIMEOUT_TICKS};
use crate::domain::rover::RoverController;
use crate::drivers::MotorController;
use embassy_time::{Duration, Ticker};
use esp_hal::gpio::Output;
use esp_hal::peripherals::MCPWM0;

pub struct ControlTaskContext {
    pub state: &'static SharedRoverState,
    pub motors: &'static mut MotorController<'static, MCPWM0<'static>, 0, 1>,
    pub leds: &'static mut [Output<'static>; 4],
}

#[embassy_executor::task]
pub async fn control_task(ctx: ControlTaskContext) {
    let mut controller = RoverController::new();
    let mut led_adapter = GpioLedAdapter { leds: ctx.leds };
    let mut ticker = Ticker::every(Duration::from_millis(TICK_RATE_MS));
    let mut last_cmd_seq: u32 = 0;
    let mut ticks_since_cmd: u32 = WATCHDOG_TIMEOUT_TICKS;

    loop {
        let sensors = ctx.state.load_sensor_frame();
        let (cmd, seq) = ctx.state.load_command_with_seq();
        let tick_age = if seq != last_cmd_seq {
            last_cmd_seq = seq;
            ticks_since_cmd = 1;
            0
        } else {
            let current = ticks_since_cmd;
            ticks_since_cmd = current.saturating_add(1);
            current
        };

        let telemetry = controller.step(sensors, cmd, tick_age, ctx.motors, &mut led_adapter);

        ctx.state.publish_telemetry(&telemetry);

        ticker.next().await;
    }
}
