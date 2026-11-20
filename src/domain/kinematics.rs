//! Differential drive kinematics, input shaping, and slew-rate limiting.

use crate::config::control::{PWM_SLEW_RATE_PER_TICK, STICK_DEADZONE, STICK_EXPO};
use crate::domain::ports::SpeedGear;

/// Clamps a normalized float to `[-1.0, 1.0]`, returning `0.0` on non-finite (`NaN`/`Inf`) inputs.
#[inline]
pub(crate) const fn sanitize_unit(val: f32) -> f32 {
    if !val.is_finite() {
        0.0
    } else if val > 1.0 {
        1.0
    } else if val < -1.0 {
        -1.0
    } else {
        val
    }
}

/// Applies a symmetric deadzone and rescales the remaining range to `[-1.0, 1.0]`.
fn apply_deadzone(input: f32, deadzone: f32) -> f32 {
    let clamped = sanitize_unit(input);
    let dz = if deadzone.is_finite() {
        deadzone.clamp(0.0, 0.99)
    } else {
        0.0
    };
    let abs = clamped.abs();
    if abs <= dz {
        0.0
    } else {
        let sign = if clamped >= 0.0 { 1.0 } else { -1.0 };
        sign * ((abs - dz) / (1.0 - dz))
    }
}

/// Applies a cubic response curve: `expo * x^3 + (1 - expo) * x`.
fn apply_expo(input: f32, expo: f32) -> f32 {
    let x = sanitize_unit(input);
    let e = if expo.is_finite() {
        expo.clamp(0.0, 1.0)
    } else {
        0.0
    };
    (e * x * x * x) + ((1.0 - e) * x)
}

/// Mixes normalized throttle and steering into `(left_duty, right_duty)` in `-1000..=1000`.
pub(crate) fn mix_arcade_drive(
    raw_throttle: f32,
    raw_steering: f32,
    gear: SpeedGear,
) -> (i32, i32) {
    let shaped_throttle = apply_expo(apply_deadzone(raw_throttle, STICK_DEADZONE), STICK_EXPO);
    let shaped_steering = apply_expo(apply_deadzone(raw_steering, STICK_DEADZONE), STICK_EXPO);

    if shaped_throttle == 0.0 && shaped_steering == 0.0 {
        return (0, 0);
    }

    let mut left = shaped_throttle + shaped_steering;
    let mut right = shaped_throttle - shaped_steering;

    let max_mag = left.abs().max(right.abs());
    if max_mag > 1.0 {
        left /= max_mag;
        right /= max_mag;
    }

    let scale = gear.power_scale() * 1000.0;
    let left_duty = (left * scale) as i32;
    let right_duty = (right * scale) as i32;

    (left_duty.clamp(-1000, 1000), right_duty.clamp(-1000, 1000))
}

/// Limits per-tick PWM duty changes to reduce H-bridge current spikes.
pub(crate) fn slew_limit(current: i32, target: i32) -> i32 {
    let delta = (target as i64) - (current as i64);
    let step = PWM_SLEW_RATE_PER_TICK as i64;
    if delta > step {
        current.saturating_add(PWM_SLEW_RATE_PER_TICK)
    } else if delta < -step {
        current.saturating_sub(PWM_SLEW_RATE_PER_TICK)
    } else {
        target
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_deadzone_eliminates_stick_drift() {
        assert_eq!(apply_deadzone(0.05, 0.08), 0.0);
        assert_eq!(apply_deadzone(-0.07, 0.08), 0.0);
        assert!((apply_deadzone(1.0, 0.08) - 1.0).abs() < 1e-5);
        assert!((apply_deadzone(-1.0, 0.08) + 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_arcade_drive_gears_and_directions() {
        assert_eq!(mix_arcade_drive(1.0, 0.0, SpeedGear::Sport), (1000, 1000));
        assert_eq!(mix_arcade_drive(-1.0, 0.0, SpeedGear::Crawl), (-450, -450));
        assert_eq!(mix_arcade_drive(0.0, 1.0, SpeedGear::Normal), (750, -750));
        assert_eq!(mix_arcade_drive(1.0, 1.0, SpeedGear::Sport), (1000, 0));
    }

    #[test]
    fn test_slew_rate_limiter() {
        assert_eq!(slew_limit(0, 1000), PWM_SLEW_RATE_PER_TICK);
        assert_eq!(slew_limit(0, -1000), -PWM_SLEW_RATE_PER_TICK);
        assert_eq!(slew_limit(200, 300), 300);
    }

    #[test]
    fn test_non_finite_inputs_sanitize_to_zero() {
        assert_eq!(apply_deadzone(f32::NAN, 0.08), 0.0);
        assert_eq!(apply_expo(f32::INFINITY, 0.35), 0.0);
        assert_eq!(
            mix_arcade_drive(f32::NAN, 1.0, SpeedGear::Sport),
            (1000, -1000)
        );
        assert_eq!(
            mix_arcade_drive(f32::NAN, 0.5, SpeedGear::Sport),
            mix_arcade_drive(0.0, 0.5, SpeedGear::Sport)
        );
        assert_eq!(
            mix_arcade_drive(f32::NEG_INFINITY, f32::NAN, SpeedGear::Sport),
            (0, 0)
        );
    }

    #[test]
    fn test_apply_expo_curves_and_edge_cases() {
        // expo = 0.0 -> strictly linear
        for &x in &[-1.0, -0.5, 0.0, 0.5, 1.0] {
            assert!((apply_expo(x, 0.0) - x).abs() < 1e-6);
        }

        // expo = 1.0 -> pure cubic x^3
        assert_eq!(apply_expo(-1.0, 1.0), -1.0);
        assert_eq!(apply_expo(0.0, 1.0), 0.0);
        assert_eq!(apply_expo(1.0, 1.0), 1.0);
        assert!((apply_expo(0.5, 1.0) - 0.125).abs() < 1e-6);
        assert!((apply_expo(-0.5, 1.0) + 0.125).abs() < 1e-6);

        // expo = 0.35 at intermediate stick positions and endpoints
        let expected_half = 0.35 * 0.125 + 0.65 * 0.5;
        assert!((apply_expo(0.5, 0.35) - expected_half).abs() < 1e-6);
        assert!((apply_expo(-0.5, 0.35) + expected_half).abs() < 1e-6);
        assert_eq!(apply_expo(1.0, 0.35), 1.0);
        assert_eq!(apply_expo(-1.0, 0.35), -1.0);
        assert_eq!(apply_expo(0.0, 0.35), 0.0);

        // Non-finite expo falls back to 0.0 (linear)
        assert!((apply_expo(0.5, f32::NAN) - 0.5).abs() < 1e-6);
        assert!((apply_expo(0.5, f32::INFINITY) - 0.5).abs() < 1e-6);
        assert!((apply_expo(-0.5, f32::NEG_INFINITY) + 0.5).abs() < 1e-6);

        // Out-of-range expo clamps to [0.0, 1.0]
        assert!((apply_expo(0.5, -0.5) - 0.5).abs() < 1e-6);
        assert!((apply_expo(0.5, 2.5) - 0.125).abs() < 1e-6);
    }

    #[test]
    fn test_apply_deadzone_boundaries_and_rescaling() {
        let dz = 0.08;
        // Exact boundary and immediately below -> 0.0
        assert_eq!(apply_deadzone(dz, dz), 0.0);
        assert_eq!(apply_deadzone(-dz, dz), 0.0);
        assert_eq!(apply_deadzone(0.0799, dz), 0.0);
        assert_eq!(apply_deadzone(-0.0799, dz), 0.0);

        // Immediately above deadzone -> positive/negative small rescaled value
        let pos_above = apply_deadzone(0.081, dz);
        let neg_above = apply_deadzone(-0.081, dz);
        assert!(pos_above > 0.0);
        assert!(neg_above < 0.0);
        assert!((pos_above + neg_above).abs() < 1e-6);

        // Mid-range rescaling: (0.54 - 0.08) / (1.0 - 0.08) = 0.5
        assert!((apply_deadzone(0.54, dz) - 0.5).abs() < 1e-5);
        assert!((apply_deadzone(-0.54, dz) + 0.5).abs() < 1e-5);

        // Zero deadzone and non-finite deadzone -> pass-through (clamped to [-1.0, 1.0])
        assert!((apply_deadzone(0.25, 0.0) - 0.25).abs() < 1e-6);
        assert_eq!(apply_deadzone(0.0, 0.0), 0.0);
        assert!((apply_deadzone(0.25, f32::NAN) - 0.25).abs() < 1e-6);
        assert!((apply_deadzone(-0.25, f32::INFINITY) + 0.25).abs() < 1e-6);
        assert!((apply_deadzone(0.25, f32::NEG_INFINITY) - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_mix_arcade_drive_asymmetric_and_reverse() {
        // Full throttle + full steering in all quadrants (Sport)
        assert_eq!(mix_arcade_drive(1.0, -1.0, SpeedGear::Sport), (0, 1000));
        assert_eq!(mix_arcade_drive(-1.0, 1.0, SpeedGear::Sport), (0, -1000));
        assert_eq!(mix_arcade_drive(-1.0, -1.0, SpeedGear::Sport), (-1000, 0));

        // Forward with steering (Normal gear: scale 750)
        let (lf, rf) = mix_arcade_drive(0.6, -0.4, SpeedGear::Normal);
        assert!(lf > 0 && rf > lf && rf <= 750);

        // Reverse with steering (Sport gear)
        let (lr, rr) = mix_arcade_drive(-0.6, 0.4, SpeedGear::Sport);
        assert!(lr < 0 && rr < lr && rr >= -1000);

        // Asymmetric wheel normalization when max(|left|, |right|) > 1.0
        let (la, ra) = mix_arcade_drive(0.9, 0.8, SpeedGear::Sport);
        assert_eq!(la, 1000);
        assert!(ra > 0 && ra < 250);
    }

    #[test]
    fn test_slew_limit_boundaries_and_zero_crossing() {
        // Exact positive and negative step limits
        assert_eq!(
            slew_limit(100, 100 + PWM_SLEW_RATE_PER_TICK),
            100 + PWM_SLEW_RATE_PER_TICK
        );
        assert_eq!(
            slew_limit(100, 100 - PWM_SLEW_RATE_PER_TICK),
            100 - PWM_SLEW_RATE_PER_TICK
        );

        // One unit beyond step limit gets clamped to +/- PWM_SLEW_RATE_PER_TICK
        assert_eq!(
            slew_limit(100, 100 + PWM_SLEW_RATE_PER_TICK + 1),
            100 + PWM_SLEW_RATE_PER_TICK
        );
        assert_eq!(
            slew_limit(100, 100 - PWM_SLEW_RATE_PER_TICK - 1),
            100 - PWM_SLEW_RATE_PER_TICK
        );

        // Zero-crossing within step limit vs beyond step limit
        assert_eq!(slew_limit(-30, 50), 50);
        assert_eq!(slew_limit(30, -50), -50);
        assert_eq!(
            slew_limit(-PWM_SLEW_RATE_PER_TICK, PWM_SLEW_RATE_PER_TICK),
            0
        );
        assert_eq!(
            slew_limit(PWM_SLEW_RATE_PER_TICK, -PWM_SLEW_RATE_PER_TICK),
            0
        );

        // Target already reached
        assert_eq!(slew_limit(420, 420), 420);
        assert_eq!(slew_limit(0, 0), 0);

        // Extreme i32 boundaries without signed overflow
        assert_eq!(
            slew_limit(i32::MIN, i32::MAX),
            i32::MIN + PWM_SLEW_RATE_PER_TICK
        );
        assert_eq!(
            slew_limit(i32::MAX, i32::MIN),
            i32::MAX - PWM_SLEW_RATE_PER_TICK
        );
    }
}
