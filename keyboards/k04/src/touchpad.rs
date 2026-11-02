use embassy_futures::select::{Either, Either3, select, select3};
use embassy_nrf::gpio::Input;
use embassy_nrf::twim::Twim;
use embassy_time::{Duration, Instant, Timer};
use rmk::core_traits::Runnable;
use rmk::event::{Axis, AxisEvent, AxisValType, EventSubscriber, PointingEvent, publish_event};
use rmk::processor::Processor;

use crate::module_settings;

/// Touchpad diagnostics over the USB CDC serial logger (`usb_debug`). The
/// no-op expansion keeps the report loop free of logging otherwise.
#[cfg(feature = "usb_debug")]
macro_rules! tp_log {
    ($($arg:tt)*) => {
        ::log::info!($($arg)*)
    };
}
#[cfg(not(feature = "usb_debug"))]
macro_rules! tp_log {
    ($($arg:tt)*) => {
        ()
    };
}

const IQS5XX_ADDR: u8 = 0x74;
const REG_PRODUCT_NUMBER: u16 = 0x0000;
const REG_PREVIOUS_CYCLE_TIME: u16 = 0x000c;
const REG_BOTTOM_BETA: u16 = 0x0637;
const REG_FILTER_SETTINGS: u16 = 0x0632;
const REG_STATIONARY_THRESH: u16 = 0x0672;
const REG_SYSTEM_CONTROL_0: u16 = 0x0431;
const REG_SYSTEM_CONTROL_1: u16 = 0x0432;
const REG_REPORT_RATE_ACTIVE: u16 = 0x057a;
const REG_REPORT_RATE_IDLE_TOUCH: u16 = 0x057c;
const REG_REPORT_RATE_IDLE: u16 = 0x057e;
const REG_REPORT_RATE_LP1: u16 = 0x0580;
const REG_REPORT_RATE_LP2: u16 = 0x0582;
const REG_ACTIVE_MODE_TIMEOUT: u16 = 0x0584;
const REG_IDLE_TOUCH_MODE_TIMEOUT: u16 = 0x0585;
const REG_IDLE_MODE_TIMEOUT: u16 = 0x0586;
const REG_LP1_MODE_TIMEOUT: u16 = 0x0587;
const REG_SYSTEM_CONFIG_0: u16 = 0x058e;
const REG_SYSTEM_CONFIG_1: u16 = 0x058f;
const REG_XY_CONFIG_0: u16 = 0x0669;
const REG_SINGLE_FINGER_GESTURES: u16 = 0x06b7;
const REG_MULTI_FINGER_GESTURES: u16 = 0x06b8;
const REG_SCROLL_INITIAL_DISTANCE: u16 = 0x06c8;
const REG_END_COMMS: u16 = 0xeeee;

const REPORT_RATE_ACTIVE_MS: u16 = 4;
const REPORT_RATE_IDLE_TOUCH_MS: u16 = 4;
const REPORT_RATE_IDLE_MS: u16 = 8;
const REPORT_RATE_LP1_MS: u16 = 8;
const REPORT_RATE_LP2_MS: u16 = 8;
const ACTIVE_MODE_TIMEOUT_SECS: u8 = 255;
const IDLE_TOUCH_MODE_TIMEOUT_SECS: u8 = 255;
const IDLE_MODE_TIMEOUT_SECS: u8 = 255;
const LP1_MODE_TIMEOUT_20_SECS: u8 = 255;

const TOUCH_EVENT_BLOCK_LEN: usize = 10;
const TOUCH_INIT_RETRY_INTERVAL: Duration = Duration::from_millis(500);
const TOUCH_READ_FAILURE_REINIT_THRESHOLD: u8 = 4;
const TOUCH_SCROLL_CONFIRM_SAMPLES: u8 = 1;
const TOUCH_MOTION_ACCUM_LIMIT: i32 = (i8::MAX as i32) * 2;
const SCROLL_DIVISOR: i16 = 8;
// A tap holds its click back for this long after the lift: a second touch
// landing inside the window turns the gesture into a double tap (two clicks
// or a drag hold), and with no second touch the click fires as a pulse.
const DOUBLE_TAP_WINDOW: Duration = Duration::from_millis(220);
// The second touch of a double tap confirms the drag hold by staying down
// and still for this long: the button then presses and holds exactly while
// the finger is down, and the held-back click is dropped so the drag never
// clicks. Lifting before the confirm reads as a double click; moving before
// it reads as ordinary cursor motion after a click.
const HOLD_CONFIRM: Duration = Duration::from_millis(15);

// Fixed-point motion filter. All filter state is kept in 1/256 of a sensor
// count; the result is scaled into 1/256 pixel units before accumulation so a
// fractional remainder can never turn into a multi-pixel jump when the scaled
// DPI is high. The high resolution also keeps integer truncation from biasing
// the EMA away from the true mean for slow motion.
const FILTER_SHIFT: u32 = 8;
const FILTER_ONE: i32 = 1 << FILTER_SHIFT;
const SLOW_FILTER_ALPHA: i32 = FILTER_ONE / 8;
const MEDIUM_FILTER_ALPHA: i32 = FILTER_ONE / 2;
const FAST_FILTER_ALPHA: i32 = FILTER_ONE;
// Raw counts per report above which the filter stops smoothing. Fast strokes
// stay raw so the cursor never feels laggy.
const MEDIUM_MOTION_SPEED: u16 = 3;
const FAST_MOTION_SPEED: u16 = 10;
// Velocity deadzone in 1/256 count per report. The EMA pushes zero-mean noise
// below this while a deliberate slow drag converges to the raw delta and passes.
const VELOCITY_DEADZONE_ENTER: i32 = 12;
const VELOCITY_DEADZONE_EXIT: i32 = 4;

const BUTTON_LEFT: u8 = 1 << 0;
const BUTTON_RIGHT: u8 = 1 << 1;
/// Set on a drag-start press so the central holds the button until the
/// matching release event instead of treating it as a one-shot tap click.
const BUTTON_DRAG_FLAG: u8 = 0x80;
const GESTURE_0_SINGLE_TAP: u8 = 1 << 0;
const GESTURE_1_TWO_FINGER_TAP: u8 = 1 << 0;
const GESTURE_1_SCROLL: u8 = 1 << 1;

const SYSTEM_INFO_0_SHOW_RESET: u8 = 1 << 7;
const SYSTEM_INFO_0_CHARGING_MODE: u8 = 0b111;
const CHARGING_MODE_ACTIVE: u8 = 0b000;
const CHARGING_MODE_IDLE_TOUCH: u8 = 0b001;
const SYSTEM_INFO_0_ATI_ERROR: u8 = 1 << 3;
const SYSTEM_INFO_0_REATI_OCCURRED: u8 = 1 << 4;
const SYSTEM_INFO_1_TP_MOVEMENT: u8 = 1 << 0;
const SYSTEM_CONTROL_0_ACK_RESET: u8 = 1 << 7;
const SYSTEM_CONTROL_1_WAKE: u8 = 0;
const SYSTEM_CONTROL_1_SUSPEND: u8 = 1 << 0;
const TOUCH_WAKE_SETTLE: Duration = Duration::from_millis(100);
// Liveness heartbeat for the USB-serial diagnostics. Without `usb_debug` the
// interval is long enough to never wake the loop in production builds.
#[cfg(feature = "usb_debug")]
const TP_HEARTBEAT: Duration = Duration::from_secs(5);
#[cfg(not(feature = "usb_debug"))]
const TP_HEARTBEAT: Duration = Duration::from_secs(3600);

/// Exponential-moving-average filter for the per-report relative delta.
///
/// The sensor reports integer counts, so at low speeds a single count of noise
/// is indistinguishable from a single count of intent. The EMA averages the
/// noise towards zero and the hysteresis deadzone discards what is left.
/// Filtering the velocity (rather than the accumulated position) keeps the
/// cursor from lagging behind the finger, and the adaptive time constant lets a
/// fast stroke decay immediately when it stops instead of coasting.
struct MotionFilter {
    x: i32,
    y: i32,
    active: bool,
}

impl MotionFilter {
    const fn new() -> Self {
        Self {
            x: 0,
            y: 0,
            active: false,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    /// Feed one raw delta and return the smoothed delta in 1/256 count units.
    fn update(&mut self, dx: i16, dy: i16) -> (i32, i32) {
        let raw_x = i32::from(dx) << FILTER_SHIFT;
        let raw_y = i32::from(dy) << FILTER_SHIFT;

        // Use the larger of the raw and filtered speed so a fast stroke keeps
        // its fast time constant while it decelerates, instead of suddenly
        // decaying slowly (which would coast) once the raw delta reaches zero.
        let raw_speed = dx.unsigned_abs().max(dy.unsigned_abs());
        let filtered_speed = (self.x.abs().max(self.y.abs()) >> FILTER_SHIFT) as u16;
        let speed = raw_speed.max(filtered_speed);
        let alpha = if speed >= FAST_MOTION_SPEED {
            FAST_FILTER_ALPHA
        } else if speed >= MEDIUM_MOTION_SPEED {
            MEDIUM_FILTER_ALPHA
        } else {
            SLOW_FILTER_ALPHA
        };

        if alpha == FAST_FILTER_ALPHA {
            self.x = raw_x;
            self.y = raw_y;
        } else {
            self.x += ema_step(raw_x - self.x, alpha);
            self.y += ema_step(raw_y - self.y, alpha);
        }

        // Hysteresis keeps a slow stroke moving once it has started without
        // letting resting-finger noise restart it.
        let threshold = if self.active {
            VELOCITY_DEADZONE_EXIT
        } else {
            VELOCITY_DEADZONE_ENTER
        };
        if self.x.abs().max(self.y.abs()) < threshold {
            self.active = false;
            return (0, 0);
        }
        self.active = true;
        (self.x, self.y)
    }
}

pub struct Touchpad {
    i2c: Twim<'static>,
    rdy: Input<'static>,
    device_id: u8,
    side: u8,
    ready: bool,
    suspended: bool,
    read_failures: u8,
    multi_finger_samples: u8,
    filter: MotionFilter,
    // Sub-pixel motion, 1/256 pixel.
    sub_x: i32,
    sub_y: i32,
    // Whole pixels waiting to be reported.
    acc_x: i32,
    acc_y: i32,
    // Two-finger scroll accumulators, 1/256 wheel unit. The remainder
    // survives across reports so slow scrolls that never reach the divisor
    // within a single report still emit units at a steady rate.
    scroll_sub_h: i32,
    scroll_sub_v: i32,
    scroll_active: bool,
    // Double-tap-and-drag state. A tap holds its click back
    // (`pending_tap_click`): a second touch landing inside the window arms
    // the hold confirm (`hold_confirm_since`) — the finger staying down and
    // still for HOLD_CONFIRM converts the click into the drag hold
    // (`drag_active`), which holds the button exactly while the finger is
    // down; lifting before the confirm delivers both clicks (a double
    // click); moving before it cancels the confirm and fires the click as
    // ordinary cursor motion after a tap.
    drag_active: bool,
    hold_confirm_since: Option<Instant>,
    pending_tap_click: Option<Instant>,
    touch_moved: bool,
    touch_multi: bool,
    was_touching: bool,
}

impl Touchpad {
    pub fn new(device_id: u8, i2c: Twim<'static>, rdy: Input<'static>) -> Self {
        Self {
            i2c,
            rdy,
            device_id,
            side: side_for_device_id(device_id),
            ready: false,
            suspended: false,
            read_failures: 0,
            multi_finger_samples: 0,
            filter: MotionFilter::new(),
            sub_x: 0,
            sub_y: 0,
            acc_x: 0,
            acc_y: 0,
            scroll_sub_h: 0,
            scroll_sub_v: 0,
            scroll_active: false,
            drag_active: false,
            hold_confirm_since: None,
            pending_tap_click: None,
            touch_moved: false,
            touch_multi: false,
            was_touching: false,
        }
    }

    async fn run_loop(&mut self) -> ! {
        loop {
            let selection = module_settings::module_selection(self.side);
            if selection != module_settings::ModuleSelection::Touchpad {
                tp_log!("tp: parked, selection={:?}", selection);
                self.deactivate().await;
                let _ = module_settings::wait_for_module_selection_change(self.side, selection).await;
                continue;
            }

            let sleeping = module_settings::module_sleeping();
            if sleeping {
                self.enter_sleep().await;
                match select(
                    module_settings::wait_for_module_selection_change(
                        self.side,
                        module_settings::ModuleSelection::Touchpad,
                    ),
                    module_settings::wait_for_module_sleep_change(sleeping),
                )
                .await
                {
                    Either::First(_) => self.deactivate().await,
                    Either::Second(_) => self.resume_from_sleep().await,
                }
                continue;
            }

            if self.suspended {
                self.resume_from_sleep().await;
            }

            if !self.ready {
                if !self.init().await {
                    Timer::after(TOUCH_INIT_RETRY_INTERVAL).await;
                }
            } else {
                let loop_wait = self.next_loop_wait();
                match select3(
                    self.rdy.wait_for_low(),
                    module_settings::wait_for_module_selection_change(
                        self.side,
                        module_settings::ModuleSelection::Touchpad,
                    ),
                    Timer::after(loop_wait),
                )
                .await
                {
                    Either3::First(_) => {
                        if module_settings::module_sleeping() {
                            self.enter_sleep().await;
                        } else {
                            self.poll_once().await;
                        }
                    }
                    Either3::Second(_) => self.deactivate().await,
                    Either3::Third(_) => {
                        if self.pending_tap_click.is_none() {
                            tp_log!("tp: alive");
                        }
                        self.release_deferred_tap();
                    }
                }
            }
        }
    }

    async fn poll_once(&mut self) {
        if !self.ready && !self.init().await {
            return;
        }

        self.release_deferred_tap();

        match self.read_motion().await {
            TouchReadResult::Motion { x, y } => {
                self.read_failures = 0;
                self.queue_motion(x, y);
            }
            TouchReadResult::Button { buttons } => {
                self.read_failures = 0;
                self.send_button(buttons);
            }
            TouchReadResult::Drag { pressed } => {
                self.read_failures = 0;
                self.send_button(if pressed { BUTTON_LEFT | BUTTON_DRAG_FLAG } else { 0 });
            }
            TouchReadResult::Scroll { h, v } => {
                self.read_failures = 0;
                self.send_scroll(h, v);
            }
            TouchReadResult::Contact => {
                self.read_failures = 0;
                // Multi-finger gestures do not move the cursor, but feeding a
                // zero delta lets the EMA decay instead of holding stale speed.
                self.queue_motion(0, 0);
            }
            TouchReadResult::NoMotion => {
                self.read_failures = 0;
                // Start the next touch with a clean filter so a previous fast
                // stroke cannot leak into the new one.
                self.filter.reset();
            }
            TouchReadResult::ReadFailed => {
                self.multi_finger_samples = 0;
                self.read_failures = self.read_failures.saturating_add(1);
                if self.read_failures >= TOUCH_READ_FAILURE_REINIT_THRESHOLD {
                    self.reset();
                }
                return;
            }
        }
        self.send_accumulated_motion();
    }

    async fn init(&mut self) -> bool {
        let product = self.read_u16(REG_PRODUCT_NUMBER).await.unwrap_or(0);
        if product == 0 {
            tp_log!("tp: init fail (no sensor)");
            return false;
        }

        let _ = self.write_u8(REG_SYSTEM_CONTROL_1, SYSTEM_CONTROL_1_WAKE).await;
        let _ = self.end_session().await;
        Timer::after(Duration::from_millis(100)).await;

        if self.read_u16(REG_PRODUCT_NUMBER).await.unwrap_or(0) == 0 {
            return false;
        }

        let mut ok = true;
        ok &= self.write_u16(REG_REPORT_RATE_ACTIVE, REPORT_RATE_ACTIVE_MS).await;
        ok &= self
            .write_u16(REG_REPORT_RATE_IDLE_TOUCH, REPORT_RATE_IDLE_TOUCH_MS)
            .await;
        ok &= self.write_u16(REG_REPORT_RATE_IDLE, REPORT_RATE_IDLE_MS).await;
        ok &= self.write_u16(REG_REPORT_RATE_LP1, REPORT_RATE_LP1_MS).await;
        ok &= self.write_u16(REG_REPORT_RATE_LP2, REPORT_RATE_LP2_MS).await;
        ok &= self.write_u8(REG_ACTIVE_MODE_TIMEOUT, ACTIVE_MODE_TIMEOUT_SECS).await;
        ok &= self
            .write_u8(REG_IDLE_TOUCH_MODE_TIMEOUT, IDLE_TOUCH_MODE_TIMEOUT_SECS)
            .await;
        ok &= self.write_u8(REG_IDLE_MODE_TIMEOUT, IDLE_MODE_TIMEOUT_SECS).await;
        ok &= self.write_u8(REG_LP1_MODE_TIMEOUT, LP1_MODE_TIMEOUT_20_SECS).await;
        ok &= self.write_u8(REG_SYSTEM_CONFIG_1, 0x46).await;
        ok &= self.write_u8(REG_BOTTOM_BETA, 2).await;
        ok &= self.write_u8(REG_STATIONARY_THRESH, 2).await;
        // IIR + MAV + ALP count filter, matching the reference IQS5xx driver.
        ok &= self.write_u8(REG_FILTER_SETTINGS, 0x0b).await;
        ok &= self.write_u8(REG_XY_CONFIG_0, 0x05).await;
        // Enable single tap and two-finger tap/scroll. Single-tap clicks come
        // from the sensor's gesture bit; the double-tap drag is tracked from
        // touch edges in software.
        ok &= self.write_u8(REG_SINGLE_FINGER_GESTURES, GESTURE_0_SINGLE_TAP).await;
        ok &= self
            .write_u8(REG_MULTI_FINGER_GESTURES, GESTURE_1_TWO_FINGER_TAP | GESTURE_1_SCROLL)
            .await;
        ok &= self.write_u16(REG_SCROLL_INITIAL_DISTANCE, 0x0001).await;
        ok &= self.write_u8(REG_SYSTEM_CONFIG_0, 0x60).await;
        ok &= self.end_session().await;

        if ok {
            self.ready = true;
            self.suspended = false;
            self.read_failures = 0;
            self.clear_motion_state();
            tp_log!("tp: init ok");
        } else {
            tp_log!("tp: init fail (config)");
        }
        ok
    }

    async fn enter_sleep(&mut self) {
        if self.suspended {
            return;
        }

        self.clear_motion_state();
        if self.ready {
            self.suspended = self.set_suspend(true).await;
            if !self.suspended {
                self.ready = false;
            }
        }
    }

    async fn resume_from_sleep(&mut self) {
        if self.suspended {
            let woke = self.set_suspend(false).await;
            self.suspended = false;
            if woke && self.ready {
                self.read_failures = 0;
                self.clear_motion_state();
                Timer::after(TOUCH_WAKE_SETTLE).await;
                return;
            }
            self.ready = false;
        }
    }

    async fn set_suspend(&mut self, suspend: bool) -> bool {
        let mut control = [0u8; 1];
        if !self.read(REG_SYSTEM_CONTROL_1, &mut control).await {
            return false;
        }
        let value = if suspend {
            control[0] | SYSTEM_CONTROL_1_SUSPEND
        } else {
            control[0] & !SYSTEM_CONTROL_1_SUSPEND
        };
        let wrote = self.write_u8(REG_SYSTEM_CONTROL_1, value).await;
        let ended = self.end_session().await;
        wrote && ended
    }

    async fn read_motion(&mut self) -> TouchReadResult {
        let mut data = [0u8; TOUCH_EVENT_BLOCK_LEN];
        if !self.read(REG_PREVIOUS_CYCLE_TIME, &mut data).await {
            return TouchReadResult::ReadFailed;
        }

        let previous_cycle_time = data[0];
        let gesture_1 = data[2];
        let system_info_0 = data[3];
        let system_info_1 = data[4];
        let number_of_fingers = data[5];
        let movement_or_scroll =
            (system_info_1 & SYSTEM_INFO_1_TP_MOVEMENT) != 0 || (gesture_1 & GESTURE_1_SCROLL) != 0;
        let x = if movement_or_scroll {
            i16::from_be_bytes([data[6], data[7]])
        } else {
            0
        };
        let y = if movement_or_scroll {
            i16::from_be_bytes([data[8], data[9]])
        } else {
            0
        };

        if (system_info_0 & SYSTEM_INFO_0_SHOW_RESET) != 0 {
            let _ = self.write_u8(REG_SYSTEM_CONTROL_0, SYSTEM_CONTROL_0_ACK_RESET).await;
            let _ = self.end_session().await;
            self.reset();
            return TouchReadResult::NoMotion;
        }

        if !self.end_session().await {
            return TouchReadResult::ReadFailed;
        }

        let charging_mode = system_info_0 & SYSTEM_INFO_0_CHARGING_MODE;
        let scanning_for_touch = charging_mode == CHARGING_MODE_ACTIVE || charging_mode == CHARGING_MODE_IDLE_TOUCH;
        if !scanning_for_touch || (system_info_0 & (SYSTEM_INFO_0_ATI_ERROR | SYSTEM_INFO_0_REATI_OCCURRED)) != 0 {
            // Idle and transition frames report untrustworthy finger data, so
            // drop the touch state.
            self.multi_finger_samples = 0;
            self.filter.reset();
            self.sub_x = 0;
            self.sub_y = 0;
            self.acc_x = 0;
            self.acc_y = 0;
            self.touch_multi = false;
            self.hold_confirm_since = None;
            self.was_touching = false;
            self.clear_scroll_state();
            self.release_drag();
            return TouchReadResult::NoMotion;
        }

        self.multi_finger_samples = if number_of_fingers >= 2 {
            self.multi_finger_samples.saturating_add(1)
        } else {
            0
        };
        let two_fingers_settled = self.multi_finger_samples >= TOUCH_SCROLL_CONFIRM_SAMPLES;
        if self.scroll_active && !two_fingers_settled {
            // The two-finger gesture ended; the next scroll starts from zero
            // instead of carrying a stale remainder.
            self.clear_scroll_state();
        }

        let gestures_enabled = module_settings::touch_gestures_enabled(self.side);
        let touching = number_of_fingers > 0;
        let touch_start_edge = touching && !self.was_touching;
        let touch_end_edge = !touching && self.was_touching;
        self.was_touching = touching;

        if touch_start_edge {
            self.touch_moved = false;
            // A second touch landing while a tap's click is pending arms the
            // hold confirm: the finger staying down and still for
            // HOLD_CONFIRM turns the gesture into a drag hold, lifting early
            // into a double click, and moving early into ordinary cursor
            // motion after a click.
            self.hold_confirm_since = if gestures_enabled && number_of_fingers == 1 && self.pending_tap_click.is_some()
            {
                Some(Instant::now())
            } else {
                None
            };
            tp_log!("tp: touch start fingers={}", number_of_fingers);
        }

        if touching {
            self.touch_multi |= self.multi_finger_samples >= 2;
            // The hold confirm: the second touch of a double tap must stay
            // down and still for HOLD_CONFIRM to become the drag hold — the
            // button then presses and holds exactly while the finger is down,
            // and the held-back click is dropped so the drag never clicks.
            // Movement cancels the confirm (the touch was ordinary cursor
            // motion after a tap) and fires the held-back click so the tap
            // still clicks.
            if let Some(since) = self.hold_confirm_since {
                if self.touch_multi || x != 0 || y != 0 {
                    self.hold_confirm_since = None;
                    if self.pending_tap_click.take().is_some() {
                        tp_log!("tp: tap click (hold cancelled by motion)");
                        self.send_button(BUTTON_LEFT);
                    }
                } else if since.elapsed() >= HOLD_CONFIRM {
                    self.hold_confirm_since = None;
                    self.pending_tap_click = None;
                    self.drag_active = true;
                    tp_log!("tp: drag press (hold confirmed)");
                    return TouchReadResult::Drag { pressed: true };
                }
            }
            if x != 0 || y != 0 {
                self.touch_moved = true;
            }
        } else {
            if touch_end_edge {
                if self.drag_active {
                    self.drag_active = false;
                    tp_log!("tp: drag release");
                    return TouchReadResult::Drag { pressed: false };
                }
                if self.hold_confirm_since.take().is_some() {
                    // The second touch lifted before the hold confirmed: a
                    // double tap — deliver the held-back click and this tap's
                    // click back to back.
                    self.pending_tap_click = None;
                    tp_log!("tp: double tap click");
                    self.send_button(BUTTON_LEFT);
                    return TouchReadResult::Button { buttons: BUTTON_LEFT };
                }
                if gestures_enabled && !self.touch_moved && !self.touch_multi {
                    // A still single-finger touch that lifted: a tap — hold
                    // the click back for the double-tap window, so a second
                    // touch can confirm it into a drag hold or complete a
                    // double click. With no second touch the click fires as
                    // a pulse when the window expires.
                    self.pending_tap_click = Some(Instant::now() + DOUBLE_TAP_WINDOW);
                    tp_log!("tp: tap click (deferred)");
                    return TouchReadResult::NoMotion;
                }
            }
        }

        if gestures_enabled && (gesture_1 & GESTURE_1_TWO_FINGER_TAP) != 0 && previous_cycle_time != 0 {
            self.release_drag();
            tp_log!("tp: two-finger tap, right click");
            return TouchReadResult::Button { buttons: BUTTON_RIGHT };
        }

        if gestures_enabled && two_fingers_settled && ((gesture_1 & GESTURE_1_SCROLL) != 0 || x != 0 || y != 0) {
            self.release_drag();
            self.scroll_active = true;
            // Accumulate raw counts in 1/256 wheel-unit units and only emit
            // whole units. Truncating each report against the divisor alone
            // (the old behavior) threw away every remainder, so a slow
            // two-finger scroll produced nothing until a single report
            // happened to carry a full divisor worth of motion.
            self.scroll_sub_h = self.scroll_sub_h.saturating_add(i32::from(x) << FILTER_SHIFT);
            self.scroll_sub_v = self.scroll_sub_v.saturating_add(i32::from(y) << FILTER_SHIFT);
            let unit = i32::from(SCROLL_DIVISOR) << FILTER_SHIFT;
            let h = (self.scroll_sub_h / unit).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            let v = (self.scroll_sub_v / unit).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            self.scroll_sub_h -= i32::from(h) * unit;
            self.scroll_sub_v -= i32::from(v) * unit;
            if h != 0 || v != 0 {
                #[cfg(feature = "usb_debug")]
                {
                    // Throttled scroll diagnostics; the report rate would
                    // flood the CDC writer with a per-event log.
                    static LAST_SCROLL_LOG_MS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
                    let now_ms = Instant::now().as_millis() as u32;
                    let last = LAST_SCROLL_LOG_MS.load(core::sync::atomic::Ordering::Relaxed);
                    if now_ms.wrapping_sub(last) >= 1000
                        && LAST_SCROLL_LOG_MS
                            .compare_exchange(
                                last,
                                now_ms,
                                core::sync::atomic::Ordering::Relaxed,
                                core::sync::atomic::Ordering::Relaxed,
                            )
                            .is_ok()
                    {
                        tp_log!(
                            "tp: scroll h={} v={} pending=({},{})",
                            h,
                            v,
                            self.scroll_sub_h,
                            self.scroll_sub_v
                        );
                    }
                }
                return TouchReadResult::Scroll { h, v };
            }
            return TouchReadResult::Contact;
        }

        if number_of_fingers == 0 {
            return TouchReadResult::NoMotion;
        }
        if number_of_fingers != 1 {
            return TouchReadResult::Contact;
        }

        // A stationary single finger still feeds the filter so its EMA can
        // decay instead of holding stale velocity until the next movement.
        TouchReadResult::Motion {
            x,
            y: y.saturating_neg(),
        }
    }

    fn reset(&mut self) {
        tp_log!("tp: reset");
        self.ready = false;
        self.suspended = false;
        self.read_failures = 0;
        self.clear_motion_state();
    }

    async fn deactivate(&mut self) {
        if self.ready && !self.suspended {
            self.suspended = self.set_suspend(true).await;
        }
        self.ready = false;
        self.read_failures = 0;
        self.clear_motion_state();
    }

    fn clear_motion_state(&mut self) {
        self.multi_finger_samples = 0;
        self.filter.reset();
        self.sub_x = 0;
        self.sub_y = 0;
        self.acc_x = 0;
        self.acc_y = 0;
        self.pending_tap_click = None;
        self.hold_confirm_since = None;
        self.touch_moved = false;
        self.touch_multi = false;
        self.was_touching = false;
        self.clear_scroll_state();
        self.release_drag();
    }

    fn clear_scroll_state(&mut self) {
        self.scroll_active = false;
        self.scroll_sub_h = 0;
        self.scroll_sub_v = 0;
    }

    fn release_drag(&mut self) {
        if self.drag_active {
            self.drag_active = false;
            self.send_button(0);
        }
    }

    /// Fires the held-back tap click once its double-tap window has passed
    /// with no second touch: a press/release pulse the host sees as the
    /// click. While a hold confirm is armed the second touch owns this
    /// click's fate, so the deadline stands down.
    fn release_deferred_tap(&mut self) {
        if self.hold_confirm_since.is_some() {
            return;
        }
        if !self
            .pending_tap_click
            .is_some_and(|deadline| Instant::now() >= deadline)
        {
            return;
        }
        self.pending_tap_click = None;
        tp_log!("tp: tap click (deferred sent)");
        self.send_button(BUTTON_LEFT);
    }

    /// How long the report loop may sleep before its next deadline: the
    /// pending click's fire time when one is waiting (and no hold confirm
    /// owns it), otherwise the liveness heartbeat.
    fn next_loop_wait(&self) -> Duration {
        if self.hold_confirm_since.is_some() {
            return TP_HEARTBEAT;
        }
        match self.pending_tap_click {
            Some(deadline) => TP_HEARTBEAT.min(deadline.saturating_duration_since(Instant::now())),
            None => TP_HEARTBEAT,
        }
    }

    /// Feed smoothed counts through the DPI scaling, accumulate sub-pixels and
    /// hand whole pixels to the report accumulator.
    fn queue_motion(&mut self, x: i16, y: i16) {
        let (fx, fy) = self.filter.update(x, y);
        if fx == 0 && fy == 0 {
            return;
        }

        self.sub_x = self
            .sub_x
            .saturating_add(module_settings::scale_touch_delta_fixed(fx, self.side));
        self.sub_y = self
            .sub_y
            .saturating_add(module_settings::scale_touch_delta_fixed(fy, self.side));

        let dx = self.sub_x >> FILTER_SHIFT;
        let dy = self.sub_y >> FILTER_SHIFT;
        self.sub_x -= dx << FILTER_SHIFT;
        self.sub_y -= dy << FILTER_SHIFT;

        self.acc_x = clamp_motion_accum(self.acc_x.saturating_add(dx));
        self.acc_y = clamp_motion_accum(self.acc_y.saturating_add(dy));
    }

    fn send_button(&self, buttons: u8) {
        publish_event(PointingEvent {
            device_id: self.device_id,
            axes: [
                relative_axis(Axis::X, 0),
                relative_axis(Axis::Y, 0),
                relative_axis(Axis::Z, i16::from(buttons)),
            ],
        });
    }

    fn send_scroll(&self, h: i16, v: i16) {
        publish_event(PointingEvent {
            device_id: self.device_id,
            axes: [
                relative_axis(Axis::H, h),
                relative_axis(Axis::V, v),
                relative_axis(Axis::Z, 0),
            ],
        });
    }

    fn send_accumulated_motion(&mut self) {
        if self.acc_x == 0 && self.acc_y == 0 {
            return;
        }

        let report_x = self.acc_x.clamp(i8::MIN as i32, i8::MAX as i32) as i16;
        let report_y = self.acc_y.clamp(i8::MIN as i32, i8::MAX as i32) as i16;
        self.acc_x -= report_x as i32;
        self.acc_y -= report_y as i32;

        publish_event(PointingEvent {
            device_id: self.device_id,
            axes: [
                relative_axis(Axis::X, report_x),
                relative_axis(Axis::Y, report_y),
                relative_axis(Axis::Z, 0),
            ],
        });
    }

    async fn read_u16(&mut self, reg: u16) -> Option<u16> {
        let mut buf = [0u8; 2];
        if self.read(reg, &mut buf).await {
            Some(u16::from_be_bytes(buf))
        } else {
            None
        }
    }

    async fn read(&mut self, reg: u16, out: &mut [u8]) -> bool {
        let address = reg.to_be_bytes();
        if self.i2c.write_read(IQS5XX_ADDR, &address, out).await.is_ok() {
            return true;
        }

        Timer::after(Duration::from_micros(200)).await;
        self.i2c.write_read(IQS5XX_ADDR, &address, out).await.is_ok()
    }

    async fn write_u8(&mut self, reg: u16, val: u8) -> bool {
        let r = reg.to_be_bytes();
        self.i2c.write(IQS5XX_ADDR, &[r[0], r[1], val]).await.is_ok()
    }

    async fn write_u16(&mut self, reg: u16, val: u16) -> bool {
        let r = reg.to_be_bytes();
        let v = val.to_be_bytes();
        self.i2c.write(IQS5XX_ADDR, &[r[0], r[1], v[0], v[1]]).await.is_ok()
    }

    async fn __end_session_bytes(&self) -> [u8; 3] {
        let r = REG_END_COMMS.to_be_bytes();
        [r[0], r[1], 0]
    }

    async fn end_session(&mut self) -> bool {
        let bytes = self.__end_session_bytes().await;
        self.i2c.write(IQS5XX_ADDR, &bytes).await.is_ok()
    }
}

fn side_for_device_id(device_id: u8) -> u8 {
    match device_id {
        2 => 0,
        3 => 1,
        _ => device_id.min(1),
    }
}

enum TouchReadResult {
    Motion { x: i16, y: i16 },
    Button { buttons: u8 },
    Drag { pressed: bool },
    Scroll { h: i16, v: i16 },
    Contact,
    NoMotion,
    ReadFailed,
}

fn relative_axis(axis: Axis, value: i16) -> AxisEvent {
    AxisEvent {
        typ: AxisValType::Rel,
        axis,
        value,
    }
}

/// One EMA increment. The nudge keeps integer truncation from stalling the
/// filter below the deadzone for very slow motion.
fn ema_step(delta: i32, alpha: i32) -> i32 {
    let step = (delta * alpha) / FILTER_ONE;
    if step == 0 && delta != 0 { delta.signum() } else { step }
}

struct NeverSub;
pub struct NeverEvent;

impl EventSubscriber for NeverSub {
    type Event = NeverEvent;

    async fn next_event(&mut self) -> NeverEvent {
        core::future::pending().await
    }
}

impl Runnable for Touchpad {
    async fn run(&mut self) -> ! {
        self.run_loop().await
    }
}

impl Processor for Touchpad {
    type Event = NeverEvent;

    fn subscriber() -> impl EventSubscriber<Event = NeverEvent> {
        NeverSub
    }

    async fn process(&mut self, _: NeverEvent) {}

    async fn process_loop(&mut self) -> ! {
        self.run().await
    }
}

fn clamp_motion_accum(value: i32) -> i32 {
    value.clamp(-TOUCH_MOTION_ACCUM_LIMIT, TOUCH_MOTION_ACCUM_LIMIT)
}
