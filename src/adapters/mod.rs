//! Runtime adapters connecting the domain core to hardware peripherals and shared state.

#[cfg(target_arch = "xtensa")]
mod control_task;
mod leds;
#[cfg(target_arch = "xtensa")]
mod sensor_task;
pub(crate) mod state;

#[cfg(target_arch = "xtensa")]
pub use control_task::{ControlTaskContext, control_task};
#[cfg(target_arch = "xtensa")]
pub use sensor_task::{SensorContext, sensor_task};
pub use state::SharedRoverState;
