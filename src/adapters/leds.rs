//! GPIO LED status indicator implementing `StatusPort`.
//!
//! Hardware mapping (`Microcontroller.kicad_sch` / `BOM.csv`):
//! - `leds[0]` (`GPIO3`): Front-Left White
//! - `leds[1]` (`GPIO4`): Front-Right White
//! - `leds[2]` (`GPIO5`): Rear-Left Red
//! - `leds[3]` (`GPIO6`): Rear-Right Red

use crate::domain::ports::{RoverMode, StatusPort, WheelOutput};
use embedded_hal::digital::OutputPin;

pub(super) struct GpioLedAdapter<'a, P> {
    pub(super) leds: &'a mut [P; 4],
}

impl<'a, P: OutputPin> GpioLedAdapter<'a, P> {
    fn set_pattern(&mut self, fl: bool, fr: bool, rl: bool, rr: bool) {
        let states = [fl, fr, rl, rr];
        for (led, on) in self.leds.iter_mut().zip(states) {
            if on {
                let _ = led.set_high();
            } else {
                let _ = led.set_low();
            }
        }
    }
}

impl<'a, P: OutputPin> StatusPort for GpioLedAdapter<'a, P> {
    fn indicate(&mut self, mode: RoverMode, wheel_output: WheelOutput, tick: u32) {
        match mode {
            RoverMode::Idle => {
                // Slow heartbeat on Front-Left white LED
                let pulse = (tick / 20).is_multiple_of(2);
                self.set_pattern(pulse, false, false, false);
            }
            RoverMode::ManualDrive => {
                if wheel_output
                    .left_duty
                    .saturating_add(wheel_output.right_duty)
                    < 0
                {
                    // Reversing: Rear Red taillights ON
                    self.set_pattern(false, false, true, true);
                } else {
                    // Forward / Neutral / Pivot: Front White headlights ON
                    self.set_pattern(true, true, false, false);
                }
            }
            RoverMode::Braking => {
                // Front headlights + solid Rear Red brake lights
                self.set_pattern(true, true, true, true);
            }
            RoverMode::AdasBraking => {
                // Rapid Rear Red ADAS alert flash with Front headlights ON
                let rear_flash = (tick / 2).is_multiple_of(2);
                self.set_pattern(true, true, rear_flash, rear_flash);
            }
            RoverMode::Disconnected => {
                // Alternating Front White search blink
                let alt = (tick / 8).is_multiple_of(2);
                self.set_pattern(alt, !alt, false, false);
            }
            RoverMode::Tipped | RoverMode::LowBattery => {
                // All 4 hazard blink
                let on = (tick / 4).is_multiple_of(2);
                self.set_pattern(on, on, on, on);
            }
        }
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::digital::ErrorType;

    #[derive(Default, Clone, Copy)]
    struct DummyPin {
        high: bool,
    }

    impl ErrorType for DummyPin {
        type Error = Infallible;
    }

    impl OutputPin for DummyPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.high = false;
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.high = true;
            Ok(())
        }
    }

    #[test]
    fn test_led_adapter_patterns_across_all_modes() {
        let mut pins = [DummyPin::default(); 4];
        let mut adapter = GpioLedAdapter { leds: &mut pins };

        let coast = WheelOutput::coast();
        let fwd = WheelOutput {
            left_duty: 500,
            right_duty: 500,
            active_brake: false,
        };
        let rev = WheelOutput {
            left_duty: -500,
            right_duty: -500,
            active_brake: false,
        };

        adapter.indicate(RoverMode::Idle, coast, 0);
        assert_eq!(adapter.leds.map(|p| p.high), [true, false, false, false]);
        adapter.indicate(RoverMode::Idle, coast, 20);
        assert_eq!(adapter.leds.map(|p| p.high), [false, false, false, false]);

        adapter.indicate(RoverMode::ManualDrive, fwd, 0);
        assert_eq!(adapter.leds.map(|p| p.high), [true, true, false, false]);

        adapter.indicate(RoverMode::ManualDrive, rev, 0);
        assert_eq!(adapter.leds.map(|p| p.high), [false, false, true, true]);

        adapter.indicate(RoverMode::Braking, coast, 0);
        assert_eq!(adapter.leds.map(|p| p.high), [true, true, true, true]);

        adapter.indicate(RoverMode::AdasBraking, coast, 0);
        assert_eq!(adapter.leds.map(|p| p.high), [true, true, true, true]);
        adapter.indicate(RoverMode::AdasBraking, coast, 2);
        assert_eq!(adapter.leds.map(|p| p.high), [true, true, false, false]);

        adapter.indicate(RoverMode::Disconnected, coast, 0);
        assert_eq!(adapter.leds.map(|p| p.high), [true, false, false, false]);
        adapter.indicate(RoverMode::Disconnected, coast, 8);
        assert_eq!(adapter.leds.map(|p| p.high), [false, true, false, false]);

        adapter.indicate(RoverMode::Tipped, coast, 0);
        assert_eq!(adapter.leds.map(|p| p.high), [true, true, true, true]);
        adapter.indicate(RoverMode::LowBattery, coast, 4);
        assert_eq!(adapter.leds.map(|p| p.high), [false, false, false, false]);
    }
}
