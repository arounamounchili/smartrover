//! Camera frame capture pipeline and double-buffered JPEG sharing (Core 1).

use core::sync::atomic::{AtomicU32, Ordering};

/// Reference-counted active `/stream` viewer tracker.
///
/// Ensures Core 1 camera capture runs only while at least one client is streaming
/// and stays active across overlapping session preemptions without underflow.
struct ViewerCounter {
    count: AtomicU32,
}

impl ViewerCounter {
    const fn new() -> Self {
        Self {
            count: AtomicU32::new(0),
        }
    }

    /// Registers a viewer and returns the new active viewer count.
    fn enter(&self) -> u32 {
        self.count.fetch_add(1, Ordering::AcqRel).saturating_add(1)
    }

    /// Unregisters a viewer (saturating at `0`) and returns the remaining viewer count.
    fn leave(&self) -> u32 {
        let mut cur = self.count.load(Ordering::Acquire);
        loop {
            if cur == 0 {
                return 0;
            }
            let next = cur - 1;
            match self
                .count
                .compare_exchange_weak(cur, next, Ordering::AcqRel, Ordering::Acquire)
            {
                Ok(_) => return next,
                Err(actual) => cur = actual,
            }
        }
    }

    fn has_viewers(&self) -> bool {
        self.count.load(Ordering::Acquire) > 0
    }
}

/// Validates JPEG SOI (`0xFF 0xD8 0xFF`) within the first 16 bytes (skipping any Arducam SPI
/// turnaround dummy byte) and trims trailing FIFO padding after JPEG EOI (`0xFF 0xD9`).
/// Returns `Some((start, end))` bounds for the clean JPEG slice, or `None` if invalid.
fn validated_jpeg_bounds(data: &[u8]) -> Option<(usize, usize)> {
    let soi_limit = data.len().min(16);
    let start = data[..soi_limit]
        .windows(3)
        .position(|w| w == [0xFF, 0xD8, 0xFF])?;

    let mut i = data.len();
    while i >= start + 4 {
        if data[i - 2] == 0xFF && data[i - 1] == 0xD9 {
            return Some((start, i));
        }
        i -= 1;
    }
    None
}

#[cfg(target_arch = "xtensa")]
mod target_impl {
    use super::{ViewerCounter, validated_jpeg_bounds};
    use crate::config::MAX_JPEG_FRAME_BYTES;
    use crate::drivers::Arducam;
    use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::mutex::{Mutex, MutexGuard};
    use embassy_sync::signal::Signal;
    use embassy_time::{Duration, Timer};
    use esp_hal::Blocking;
    use esp_hal::spi::master::Spi;

    /// RAII guard tracking an active `/stream` viewer; decrements the viewer count on drop.
    pub(crate) struct StreamViewerGuard<'a> {
        frame: &'a SharedFrame,
    }

    impl Drop for StreamViewerGuard<'_> {
        fn drop(&mut self) {
            self.frame.viewers.leave();
        }
    }

    #[repr(align(4))]
    pub struct SharedFrame {
        slots: [Mutex<CriticalSectionRawMutex, heapless::Vec<u8, MAX_JPEG_FRAME_BYTES>>; 2],
        active_slot: AtomicU8,
        version: AtomicU32,
        viewers: ViewerCounter,
        capture_wake: Signal<CriticalSectionRawMutex, ()>,
        frame_ready: Signal<CriticalSectionRawMutex, ()>,
    }

    impl Default for SharedFrame {
        fn default() -> Self {
            Self::new()
        }
    }

    impl SharedFrame {
        pub const fn new() -> Self {
            Self {
                slots: [
                    Mutex::new(heapless::Vec::new()),
                    Mutex::new(heapless::Vec::new()),
                ],
                active_slot: AtomicU8::new(0),
                version: AtomicU32::new(0),
                viewers: ViewerCounter::new(),
                capture_wake: Signal::new(),
                frame_ready: Signal::new(),
            }
        }

        pub(crate) fn version(&self) -> u32 {
            self.version.load(Ordering::Acquire)
        }

        /// Registers an active `/stream` client and wakes Core 1 camera capture immediately.
        pub(crate) fn acquire_viewer(&self) -> StreamViewerGuard<'_> {
            self.viewers.enter();
            self.capture_wake.signal(());
            StreamViewerGuard { frame: self }
        }

        fn has_viewers(&self) -> bool {
            self.viewers.has_viewers()
        }

        /// Suspends Core 1 `camera_task` in hardware low-power wait until a `/stream` client connects.
        async fn wait_for_viewer(&self) {
            self.capture_wake.wait().await;
        }

        /// Wakes any `/stream` task waiting for a new frame (used on session preemption).
        pub(crate) fn notify_stream_preempt(&self) {
            self.frame_ready.signal(());
        }

        /// Waits for Core 1 to publish a new frame (or for a preemption notification).
        pub(crate) async fn wait_frame_signal(&self) {
            self.frame_ready.wait().await;
        }

        fn inactive_slot_index(&self) -> usize {
            (self.active_slot.load(Ordering::Acquire) ^ 1) as usize
        }

        async fn lock_write_slot(
            &self,
            idx: usize,
        ) -> MutexGuard<'_, CriticalSectionRawMutex, heapless::Vec<u8, MAX_JPEG_FRAME_BYTES>>
        {
            self.slots[idx].lock().await
        }

        fn publish_slot(&self, idx: usize) {
            self.active_slot.store(idx as u8, Ordering::Release);
            self.version.fetch_add(1, Ordering::Release);
            self.frame_ready.signal(());
        }

        /// Locks the currently active completed JPEG frame slot without cross-frame tearing.
        pub(crate) async fn lock_active_frame(
            &self,
        ) -> MutexGuard<'_, CriticalSectionRawMutex, heapless::Vec<u8, MAX_JPEG_FRAME_BYTES>>
        {
            loop {
                let idx = self.active_slot.load(Ordering::Acquire) as usize;
                let slot = self.slots[idx].lock().await;
                if self.active_slot.load(Ordering::Acquire) as usize == idx {
                    return slot;
                }
            }
        }
    }

    /// Context owning the camera and SPI peripheral exclusively on Core 1.
    pub struct CameraContext {
        pub arducam: Arducam,
        pub spi: Spi<'static, Blocking>,
        pub frame: &'static SharedFrame,
    }

    enum CaptureWait {
        Done,
        Timeout,
        SpiError,
        ViewerLeft,
    }

    #[embassy_executor::task]
    pub async fn camera_task(mut ctx: CameraContext) {
        loop {
            // Sleep Core 1 completely when no browser is streaming `/stream`.
            if !ctx.frame.has_viewers() {
                ctx.frame.wait_for_viewer().await;
                continue;
            }

            if let Err(e) = ctx
                .arducam
                .clear_fifo_flag(&mut ctx.spi)
                .and_then(|_| ctx.arducam.start_capture(&mut ctx.spi))
            {
                log::warn!("Camera SPI start failed: {e:?}");
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }

            let mut wait_result = CaptureWait::Timeout;
            for _ in 0..100 {
                if !ctx.frame.has_viewers() {
                    wait_result = CaptureWait::ViewerLeft;
                    break;
                }
                match ctx.arducam.is_capture_done(&mut ctx.spi) {
                    Ok(true) => {
                        wait_result = CaptureWait::Done;
                        break;
                    }
                    Ok(false) => {}
                    Err(e) => {
                        log::warn!("Camera SPI poll failed: {e:?}");
                        wait_result = CaptureWait::SpiError;
                        break;
                    }
                }
                Timer::after(Duration::from_millis(10)).await;
            }

            match wait_result {
                CaptureWait::Done => {}
                CaptureWait::ViewerLeft => {
                    let _ = ctx.arducam.flush_fifo(&mut ctx.spi);
                    let _ = ctx.arducam.clear_fifo_flag(&mut ctx.spi);
                    continue;
                }
                CaptureWait::Timeout => {
                    log::warn!("Camera capture timeout; flushing FIFO");
                    let _ = ctx.arducam.flush_fifo(&mut ctx.spi);
                    let _ = ctx.arducam.clear_fifo_flag(&mut ctx.spi);
                    Timer::after(Duration::from_millis(100)).await;
                    continue;
                }
                CaptureWait::SpiError => {
                    let _ = ctx.arducam.flush_fifo(&mut ctx.spi);
                    let _ = ctx.arducam.clear_fifo_flag(&mut ctx.spi);
                    Timer::after(Duration::from_millis(100)).await;
                    continue;
                }
            }

            let len = match ctx.arducam.get_fifo_length(&mut ctx.spi) {
                Ok(l) => l as usize,
                Err(e) => {
                    log::warn!("Camera FIFO length read failed: {e:?}");
                    let _ = ctx.arducam.flush_fifo(&mut ctx.spi);
                    Timer::after(Duration::from_millis(100)).await;
                    continue;
                }
            };

            if len > 0 && len < MAX_JPEG_FRAME_BYTES {
                // Read one extra byte so any leading SPI turnaround dummy byte precedes a full frame.
                let read_len = len + 1;
                let write_idx = ctx.frame.inactive_slot_index();
                let mut valid = false;
                {
                    let mut slot = ctx.frame.lock_write_slot(write_idx).await;
                    let burst_ok = ctx
                        .arducam
                        .read_fifo_burst(&mut ctx.spi, &mut slot, read_len);

                    if !burst_ok {
                        log::warn!("Camera SPI burst read failed; dropping frame");
                    } else if let Some((start, end)) = validated_jpeg_bounds(&slot) {
                        slot.truncate(end);
                        if start > 0 {
                            slot.copy_within(start.., 0);
                            slot.truncate(end - start);
                        }
                        valid = true;
                    } else {
                        let head_len = slot.len().min(8);
                        let tail_start = slot.len().saturating_sub(8);
                        log::warn!(
                            "Dropping malformed JPEG frame ({len} bytes, head={:02X?}, tail={:02X?})",
                            &slot[..head_len],
                            &slot[tail_start..]
                        );
                    }
                }

                if valid {
                    ctx.frame.publish_slot(write_idx);
                } else {
                    let _ = ctx.arducam.flush_fifo(&mut ctx.spi);
                }
            } else {
                if len >= MAX_JPEG_FRAME_BYTES {
                    log::warn!(
                        "Frame too large ({len} + 1 > {MAX_JPEG_FRAME_BYTES} bytes); flushing FIFO"
                    );
                }
                let _ = ctx.arducam.flush_fifo(&mut ctx.spi);
                let _ = ctx.arducam.clear_fifo_flag(&mut ctx.spi);
            }

            Timer::after(Duration::from_millis(10)).await;
        }
    }
}

#[cfg(target_arch = "xtensa")]
pub use target_impl::{CameraContext, SharedFrame, camera_task};

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_jpeg_validation_and_fifo_padding_trim() {
        assert_eq!(
            validated_jpeg_bounds(&[0xFF, 0xD8, 0xFF, 0xD9]),
            Some((0, 4))
        );
        assert_eq!(
            validated_jpeg_bounds(&[0xFF, 0xD8, 0xFF, 0xE0, 0x22, 0xFF, 0xD9, 0x00, 0x00]),
            Some((0, 7))
        );
        // Leading Arducam SPI turnaround dummy byte + >18-byte trailing FIFO padding
        let mut padded = [0u8; 40];
        padded[0] = 0x00;
        padded[1..7].copy_from_slice(&[0xFF, 0xD8, 0xFF, 0xE0, 0xFF, 0xD9]);
        assert_eq!(validated_jpeg_bounds(&padded), Some((1, 7)));

        assert_eq!(validated_jpeg_bounds(&[0x00, 0xD8, 0xFF, 0xD9]), None);
        assert_eq!(
            validated_jpeg_bounds(&[0xFF, 0xD8, 0xFF, 0xE0, 0x11, 0x22]),
            None
        );
        assert_eq!(validated_jpeg_bounds(&[0xFF, 0xD8]), None);
    }

    #[test]
    fn test_jpeg_validation_edge_cases_and_boundaries() {
        // Empty, 1-byte, and <4-byte slices
        assert_eq!(validated_jpeg_bounds(&[]), None);
        assert_eq!(validated_jpeg_bounds(&[0xFF]), None);
        assert_eq!(validated_jpeg_bounds(&[0xFF, 0xD8, 0xFF]), None);
        assert_eq!(validated_jpeg_bounds(&[0xFF, 0xD8, 0xD9]), None);

        // Minimal 4-byte valid JPEG marker sequence
        assert_eq!(
            validated_jpeg_bounds(&[0xFF, 0xD8, 0xFF, 0xD9]),
            Some((0, 4))
        );

        // SOI at offset 13 (valid: bytes 13..16 fit in 16-byte search window)
        let mut soi_13 = [0x00u8; 22];
        soi_13[13..16].copy_from_slice(&[0xFF, 0xD8, 0xFF]);
        soi_13[16..18].copy_from_slice(&[0xFF, 0xD9]);
        assert_eq!(validated_jpeg_bounds(&soi_13), Some((13, 18)));

        // SOI at offset 14 (invalid: 3rd SOI byte falls outside first 16 bytes)
        let mut soi_14 = [0x00u8; 22];
        soi_14[14..17].copy_from_slice(&[0xFF, 0xD8, 0xFF]);
        soi_14[17..19].copy_from_slice(&[0xFF, 0xD9]);
        assert_eq!(validated_jpeg_bounds(&soi_14), None);

        // Multiple EOI markers -> trims trailing padding after the last EOI marker
        let multi_eoi = [
            0xAA, 0xFF, 0xD8, 0xFF, 0xE0, 0xFF, 0xD9, 0x12, 0x34, 0xFF, 0xD9, 0x00, 0x00,
        ];
        assert_eq!(validated_jpeg_bounds(&multi_eoi), Some((1, 11)));

        // Missing SOI or missing EOI
        assert_eq!(
            validated_jpeg_bounds(&[0x00, 0x00, 0xFF, 0xE0, 0xFF, 0xD9]),
            None
        );
        assert_eq!(
            validated_jpeg_bounds(&[0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x00]),
            None
        );
    }

    #[test]
    fn test_viewer_counter_handles_preemption_overlap_and_underflow() {
        let viewers = ViewerCounter::new();
        assert!(!viewers.has_viewers());

        // Underflow guard: leaving at 0 stays at 0
        assert_eq!(viewers.leave(), 0);
        assert!(!viewers.has_viewers());

        // Session 1 enters
        assert_eq!(viewers.enter(), 1);
        assert!(viewers.has_viewers());

        // Session 2 preempts while Session 1 is still releasing -> count reaches 2
        assert_eq!(viewers.enter(), 2);
        assert!(viewers.has_viewers());

        // Session 1 exits -> Session 2 must keep camera capture active
        assert_eq!(viewers.leave(), 1);
        assert!(viewers.has_viewers());

        // Session 2 exits -> capture goes idle
        assert_eq!(viewers.leave(), 0);
        assert!(!viewers.has_viewers());
    }
}
