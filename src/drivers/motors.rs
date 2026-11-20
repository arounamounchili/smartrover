//! Differential H-bridge motor driver (`MCPWM0`).

use crate::domain::ports::WheelOutput;

#[cfg(target_arch = "xtensa")]
use crate::domain::ports::MotorPort;
#[cfg(target_arch = "xtensa")]
use esp_hal::mcpwm::{PwmPeripheral, operator::PwmPin};

/// Pure function that converts a `WheelOutput` (`-1000..=1000` or `active_brake`)
/// into the 4 H-Bridge PWM compare timestamps `(left_a, left_b, right_a, right_b)`.
fn compute_pwm_timestamps(period: u16, output: WheelOutput) -> (u16, u16, u16, u16) {
    if output.active_brake {
        let brk = period.saturating_add(1);
        return (brk, brk, brk, brk);
    }

    let max = period as u32;

    let left_mag = output.left_duty.unsigned_abs().min(1000);
    let left_ts = (max * left_mag / 1000) as u16;
    let (la, lb) = if output.left_duty >= 0 {
        (0, left_ts)
    } else {
        (left_ts, 0)
    };

    let right_mag = output.right_duty.unsigned_abs().min(1000);
    let right_ts = (max * right_mag / 1000) as u16;
    let (ra, rb) = if output.right_duty >= 0 {
        (0, right_ts)
    } else {
        (right_ts, 0)
    };

    (la, lb, ra, rb)
}

#[cfg(target_arch = "xtensa")]
pub struct MotorController<'d, PWM, const OP0: u8, const OP1: u8> {
    left_a: PwmPin<'d, PWM, OP0, true>,
    left_b: PwmPin<'d, PWM, OP0, false>,
    right_a: PwmPin<'d, PWM, OP1, true>,
    right_b: PwmPin<'d, PWM, OP1, false>,
    period: u16,
}

#[cfg(target_arch = "xtensa")]
impl<'d, PWM, const OP0: u8, const OP1: u8> MotorController<'d, PWM, OP0, OP1>
where
    PWM: PwmPeripheral,
{
    pub fn new(
        left_a: PwmPin<'d, PWM, OP0, true>,
        left_b: PwmPin<'d, PWM, OP0, false>,
        right_a: PwmPin<'d, PWM, OP1, true>,
        right_b: PwmPin<'d, PWM, OP1, false>,
        period: u16,
    ) -> Self {
        Self {
            left_a,
            left_b,
            right_a,
            right_b,
            period,
        }
    }
}

#[cfg(target_arch = "xtensa")]
impl<'d, PWM, const OP0: u8, const OP1: u8> MotorPort for MotorController<'d, PWM, OP0, OP1>
where
    PWM: PwmPeripheral,
{
    fn apply(&mut self, output: WheelOutput) {
        let (la, lb, ra, rb) = compute_pwm_timestamps(self.period, output);
        self.left_a.set_timestamp(la);
        self.left_b.set_timestamp(lb);
        self.right_a.set_timestamp(ra);
        self.right_b.set_timestamp(rb);
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_compute_pwm_timestamps_forward_reverse_brake_and_coast() {
        // Coast
        assert_eq!(
            compute_pwm_timestamps(2000, WheelOutput::coast()),
            (0, 0, 0, 0)
        );

        // Active electromagnetic brake shorts H-bridge with period + 1
        assert_eq!(
            compute_pwm_timestamps(2000, WheelOutput::brake()),
            (2001, 2001, 2001, 2001)
        );

        // Differential turn: left +500 (forward), right -500 (reverse)
        assert_eq!(
            compute_pwm_timestamps(
                2000,
                WheelOutput {
                    left_duty: 500,
                    right_duty: -500,
                    active_brake: false,
                }
            ),
            (0, 1000, 1000, 0)
        );

        // Clamping beyond 1000
        assert_eq!(
            compute_pwm_timestamps(
                1999,
                WheelOutput {
                    left_duty: 1500,
                    right_duty: -1500,
                    active_brake: false,
                }
            ),
            (0, 1999, 1999, 0)
        );
    }

    #[test]
    fn test_compute_pwm_timestamps_boundaries_and_extremes() {
        // Full forward and full reverse
        assert_eq!(
            compute_pwm_timestamps(
                2000,
                WheelOutput {
                    left_duty: 1000,
                    right_duty: 1000,
                    active_brake: false,
                }
            ),
            (0, 2000, 0, 2000)
        );
        assert_eq!(
            compute_pwm_timestamps(
                2000,
                WheelOutput {
                    left_duty: -1000,
                    right_duty: -1000,
                    active_brake: false,
                }
            ),
            (2000, 0, 2000, 0)
        );

        // Zero duty
        assert_eq!(
            compute_pwm_timestamps(
                2000,
                WheelOutput {
                    left_duty: 0,
                    right_duty: 0,
                    active_brake: false,
                }
            ),
            (0, 0, 0, 0)
        );

        // Positive and negative asymmetric duty
        assert_eq!(
            compute_pwm_timestamps(
                2000,
                WheelOutput {
                    left_duty: 250,
                    right_duty: -750,
                    active_brake: false,
                }
            ),
            (0, 500, 1500, 0)
        );

        // Period = 0 (drive vs brake)
        assert_eq!(
            compute_pwm_timestamps(
                0,
                WheelOutput {
                    left_duty: 1000,
                    right_duty: -1000,
                    active_brake: false,
                }
            ),
            (0, 0, 0, 0)
        );
        assert_eq!(
            compute_pwm_timestamps(0, WheelOutput::brake()),
            (1, 1, 1, 1)
        );

        // Extreme i32::MIN / i32::MAX duty clamping and large period without u32 overflow
        assert_eq!(
            compute_pwm_timestamps(
                65_534,
                WheelOutput {
                    left_duty: i32::MAX,
                    right_duty: i32::MIN,
                    active_brake: false,
                }
            ),
            (0, 65_534, 65_534, 0)
        );
        assert_eq!(
            compute_pwm_timestamps(65_534, WheelOutput::brake()),
            (65_535, 65_535, 65_535, 65_535)
        );
        assert_eq!(
            compute_pwm_timestamps(u16::MAX, WheelOutput::brake()),
            (u16::MAX, u16::MAX, u16::MAX, u16::MAX)
        );
    }
}
