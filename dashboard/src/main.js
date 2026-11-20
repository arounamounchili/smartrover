import './style.css';
import {
  MODE_MAP,
  createControlState,
  resetTransientInputs,
  sampleControlInputs,
  setGear as updateGearState,
  toggleAdas as updateAdasState,
} from './controls.js';

const JOY_SIZE = 176;
const TARGET_LOOP_MS = 50;
const LOW_BATT_WARN_PCT = 20;
const ADAS_SLOW_DISTANCE_MM = 320;
const DIST_METER_MAX_MM = 1000;
const DIST_CLEAR_MM = 2000;
const NO_GAMEPAD_TEXT = 'No controller detected (WASD or pointer)';

const controlState = createControlState();

const joyCanvas = document.getElementById('joystick-canvas');
const joyCtx = joyCanvas.getContext('2d');
const gamepadText = document.getElementById('gamepad-text');
const ebrakeBtn = document.getElementById('ebrake-btn');
const adasBtn = document.getElementById('adas-btn');
const modePill = document.getElementById('mode-pill');
const sensorPill = document.getElementById('sensor-pill');
const battPill = document.getElementById('batt-pill');
const gearHint = document.getElementById('gear-hint');
const thrVal = document.getElementById('thr-val');
const strVal = document.getElementById('str-val');
const inputSourceTag = document.getElementById('input-source-tag');
const distEl = document.getElementById('dist');
const distBar = document.getElementById('dist-bar');
const adasTag = document.getElementById('adas-tag');
const leftPwm = document.getElementById('left-pwm');
const rightPwm = document.getElementById('right-pwm');
const leftBar = document.getElementById('left-bar');
const rightBar = document.getElementById('right-bar');
const linkPill = document.getElementById('link-pill');
const linkText = document.getElementById('link-text');
const gearBtns = document.querySelectorAll('.gear-btn');

let lastUiGear = controlState.gear;
let lastUiAdas = controlState.adasEnabled;
let lastUiBrake = false;

function syncBrakeUi(active = controlState.pointerBrake || controlState.keyBrake) {
  if (active === lastUiBrake) return;
  lastUiBrake = active;
  ebrakeBtn.classList.toggle('active', active);
  ebrakeBtn.setAttribute('aria-pressed', String(active));
}

function setGear(gear) {
  const next = updateGearState(controlState, gear);
  lastUiGear = next;
  gearBtns.forEach(btn => {
    const isMatch = Number(btn.dataset.gear) === next;
    btn.classList.toggle('active', isMatch);
    btn.setAttribute('aria-pressed', String(isMatch));
  });
}

function toggleAdas(forceState) {
  const next = updateAdasState(controlState, forceState);
  lastUiAdas = next;
  adasBtn.classList.toggle('active', next);
  adasBtn.setAttribute('aria-pressed', String(next));
  adasBtn.textContent = next ? 'Assist: On' : 'Assist: Off';
}

gearBtns.forEach(btn => {
  btn.onclick = () => setGear(Number(btn.dataset.gear));
});

adasBtn.onclick = () => toggleAdas();

ebrakeBtn.addEventListener('pointerdown', (e) => {
  controlState.pointerBrake = true;
  ebrakeBtn.setPointerCapture?.(e.pointerId);
  syncBrakeUi();
});

const releasePointerBrake = () => {
  controlState.pointerBrake = false;
  syncBrakeUi();
};
ebrakeBtn.addEventListener('pointerup', releasePointerBrake);
ebrakeBtn.addEventListener('pointercancel', releasePointerBrake);

const PREVENT_SCROLL_KEYS = new Set([
  'Space',
  'ArrowUp',
  'ArrowDown',
  'ArrowLeft',
  'ArrowRight',
]);

window.addEventListener('keydown', (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (PREVENT_SCROLL_KEYS.has(e.code)) {
    e.preventDefault();
  }
  controlState.keysPressed.add(e.code);
  if (e.code === 'Digit1') setGear(1);
  if (e.code === 'Digit2') setGear(2);
  if (e.code === 'Digit3') setGear(3);
  if (e.code === 'Space') {
    controlState.keyBrake = true;
    syncBrakeUi();
  }
});

window.addEventListener('keyup', (e) => {
  controlState.keysPressed.delete(e.code);
  if (e.code === 'Space') {
    e.preventDefault();
    controlState.keyBrake = false;
    syncBrakeUi();
  }
});

const handleSafetyReset = () => {
  resetTransientInputs(controlState);
  syncBrakeUi();
};
window.addEventListener('blur', handleSafetyReset);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) handleSafetyReset();
});

function updateTouchStick(clientX, clientY) {
  const rect = joyCanvas.getBoundingClientRect();
  const maxR = rect.width / 2 - 16;
  if (maxR <= 0) return;

  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  let dx = (clientX - cx) / maxR;
  let dy = (cy - clientY) / maxR;
  const mag = Math.hypot(dx, dy);
  if (mag > 1.0) {
    dx /= mag;
    dy /= mag;
  }
  controlState.touchSteering = dx;
  controlState.touchThrottle = dy;
}

joyCanvas.addEventListener('pointerdown', (e) => {
  controlState.touchActive = true;
  joyCanvas.setPointerCapture?.(e.pointerId);
  updateTouchStick(e.clientX, e.clientY);
});

joyCanvas.addEventListener('pointermove', (e) => {
  if (controlState.touchActive) updateTouchStick(e.clientX, e.clientY);
});

const releaseTouch = () => {
  controlState.touchActive = false;
  controlState.touchThrottle = 0.0;
  controlState.touchSteering = 0.0;
};
joyCanvas.addEventListener('pointerup', releaseTouch);
joyCanvas.addEventListener('pointercancel', releaseTouch);

let lastGamepadId = null;

function syncGamepadUi(gamepadId) {
  const next = gamepadId || null;
  if (next === lastGamepadId) return;
  lastGamepadId = next;
  if (next) {
    gamepadText.classList.add('active');
    gamepadText.textContent = next.slice(0, 32);
    gearHint.textContent = /dualshock|dualsense|sony|054c|playstation/i.test(next)
      ? 'L1 / R1'
      : 'LB / RB';
  } else {
    gamepadText.classList.remove('active');
    gamepadText.textContent = NO_GAMEPAD_TEXT;
    gearHint.textContent = 'LB / RB';
  }
}

window.addEventListener('gamepadconnected', (e) => {
  syncGamepadUi(e.gamepad?.id || 'Gamepad');
});

window.addEventListener('gamepaddisconnected', () => {
  syncGamepadUi(null);
});

let lastCanvasDpr = 0;
let lastJoySteering = null;
let lastJoyThrottle = null;

function syncCanvasResolution() {
  const dpr = Math.max(1, window.devicePixelRatio || 1);
  if (dpr === lastCanvasDpr) return false;
  lastCanvasDpr = dpr;
  joyCanvas.width = Math.round(JOY_SIZE * dpr);
  joyCanvas.height = Math.round(JOY_SIZE * dpr);
  joyCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return true;
}

function drawJoystick(steering, throttle) {
  const resized = syncCanvasResolution();
  if (!resized && steering === lastJoySteering && throttle === lastJoyThrottle) return;
  lastJoySteering = steering;
  lastJoyThrottle = throttle;
  const center = JOY_SIZE / 2;
  const outerR = center - 10;
  const travelR = outerR - 12;

  joyCtx.clearRect(0, 0, JOY_SIZE, JOY_SIZE);

  joyCtx.beginPath();
  joyCtx.arc(center, center, outerR, 0, Math.PI * 2);
  joyCtx.fillStyle = '#18181b';
  joyCtx.fill();
  joyCtx.strokeStyle = '#27272a';
  joyCtx.lineWidth = 1;
  joyCtx.stroke();

  joyCtx.beginPath();
  joyCtx.arc(center, center, travelR * 0.5, 0, Math.PI * 2);
  joyCtx.setLineDash([2, 3]);
  joyCtx.strokeStyle = '#27272a';
  joyCtx.stroke();
  joyCtx.setLineDash([]);

  joyCtx.beginPath();
  joyCtx.moveTo(center - outerR, center);
  joyCtx.lineTo(center + outerR, center);
  joyCtx.moveTo(center, center - outerR);
  joyCtx.lineTo(center, center + outerR);
  joyCtx.strokeStyle = '#27272a';
  joyCtx.stroke();

  const px = center + steering * travelR;
  const py = center - throttle * travelR;

  if (Math.hypot(steering, throttle) > 0.02) {
    joyCtx.beginPath();
    joyCtx.moveTo(center, center);
    joyCtx.lineTo(px, py);
    joyCtx.strokeStyle = '#71717a';
    joyCtx.lineWidth = 1.5;
    joyCtx.stroke();
  }

  joyCtx.beginPath();
  joyCtx.arc(px, py, 10, 0, Math.PI * 2);
  joyCtx.fillStyle = '#f4f4f5';
  joyCtx.fill();
}

let failCount = 0;
let isOffline = false;

async function loop() {
  const startedAt = performance.now();
  const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
  const input = sampleControlInputs(controlState, gamepads);

  syncGamepadUi(input.gamepadId);
  if (input.gear !== lastUiGear) {
    setGear(input.gear);
  }
  if (input.adas !== lastUiAdas) {
    toggleAdas(input.adas);
  }
  syncBrakeUi(input.brake);
  drawJoystick(input.steering, input.throttle);

  const tMilli = Math.round(input.throttle * 1000);
  const sMilli = Math.round(input.steering * 1000);
  const bFlag = input.brake ? 1 : 0;
  const aFlag = input.adas ? 1 : 0;

  thrVal.textContent = `${Math.round(input.throttle * 100)}%`;
  strVal.textContent = `${Math.round(input.steering * 100)}%`;
  inputSourceTag.textContent = input.source;

  try {
    const r = await fetch(`/control?t=${tMilli}&s=${sMilli}&g=${input.gear}&b=${bFlag}&a=${aFlag}`, {
      method: 'POST',
      cache: 'no-store',
      signal: AbortSignal.timeout(250),
    });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const d = await r.json();

    const tofOk = Boolean(d.tof_ok);
    const imuOk = Boolean(d.imu_ok);

    distEl.textContent = !tofOk
      ? 'FAULT'
      : (d.dist >= DIST_CLEAR_MM ? 'Out of range' : `${d.dist} mm`);
    battPill.textContent = `${d.batt}%`;
    battPill.className = d.batt < LOW_BATT_WARN_PCT ? 'kv-val fault' : 'kv-val';

    const modeLabel = MODE_MAP[d.state] || 'UNKNOWN';
    modePill.textContent = modeLabel;
    modePill.className = (d.state === 5 || d.state === 6)
      ? 'kv-val fault'
      : (d.state === 2 || d.state === 3 ? 'kv-val warn' : 'kv-val');

    if (!tofOk && !imuOk) {
      sensorPill.textContent = 'TOF+IMU ERR';
      sensorPill.className = 'kv-val fault';
    } else if (!tofOk) {
      sensorPill.textContent = 'TOF ERR';
      sensorPill.className = 'kv-val warn';
    } else if (!imuOk) {
      sensorPill.textContent = 'IMU ERR';
      sensorPill.className = 'kv-val fault';
    } else {
      sensorPill.textContent = 'OK';
      sensorPill.className = 'kv-val ok';
    }

    const leftPct = Math.round((d.left_pwm / 1000) * 100);
    const rightPct = Math.round((d.right_pwm / 1000) * 100);
    leftPwm.textContent = `${leftPct}%`;
    rightPwm.textContent = `${rightPct}%`;
    leftBar.style.width = `${Math.abs(leftPct)}%`;
    rightBar.style.width = `${Math.abs(rightPct)}%`;
    leftBar.style.background = leftPct < 0 ? 'var(--amber)' : '#e4e4e7';
    rightBar.style.background = rightPct < 0 ? 'var(--amber)' : '#e4e4e7';

    distBar.style.width = tofOk ? `${Math.min(100, (d.dist / DIST_METER_MAX_MM) * 100)}%` : '0%';

    if (!tofOk) {
      adasTag.className = 'tag tag-warn';
      adasTag.textContent = 'SENSOR FAULT';
      distBar.style.background = 'var(--amber)';
    } else if (!controlState.adasEnabled) {
      adasTag.className = 'tag tag-neutral';
      adasTag.textContent = 'ASSIST OFF';
      distBar.style.background = '#e4e4e7';
    } else if (d.state === 3) {
      adasTag.className = 'tag tag-stop';
      adasTag.textContent = 'STOPPED';
      distBar.style.background = 'var(--red)';
    } else if (d.adas_active || d.dist < ADAS_SLOW_DISTANCE_MM) {
      adasTag.className = 'tag tag-warn';
      adasTag.textContent = 'LIMITED';
      distBar.style.background = 'var(--amber)';
    } else {
      adasTag.className = 'tag tag-ok';
      adasTag.textContent = 'CLEAR';
      distBar.style.background = '#e4e4e7';
    }

    linkPill.className = 'conn-badge online';
    linkText.textContent =
      d.rssi === 0 ? 'Online (AP)' : `Online (${d.rssi} dBm)`;
    failCount = 0;
    isOffline = false;
  } catch {
    if (failCount < 4) {
      failCount++;
    }
    if (failCount === 4 && !isOffline) {
      isOffline = true;
      linkPill.className = 'conn-badge offline';
      linkText.textContent = 'Offline';
      modePill.textContent = 'STANDBY';
      modePill.className = 'kv-val';
      sensorPill.textContent = '--';
      sensorPill.className = 'kv-val';
      battPill.textContent = '--%';
      battPill.className = 'kv-val';
      distEl.textContent = '-- mm';
      distBar.style.width = '0%';
      adasTag.className = 'tag tag-neutral';
      adasTag.textContent = 'OFFLINE';
      leftPwm.textContent = '0%';
      rightPwm.textContent = '0%';
      leftBar.style.width = '0%';
      rightBar.style.width = '0%';
    }
  }

  const elapsed = performance.now() - startedAt;
  setTimeout(loop, Math.max(10, TARGET_LOOP_MS - elapsed));
}

const videoFeed = document.getElementById('video-feed');
const videoPlaceholder = document.getElementById('video-placeholder');
const videoTitle = document.getElementById('video-title');
const videoSub = document.getElementById('video-sub');
const streamBtn = document.getElementById('stream-toggle');
let isStreaming = false;

function stopStreamUi(reasonTitle = 'Stream inactive', reasonSub = 'Click "Start Stream" to connect to /stream') {
  videoFeed.removeAttribute('src');
  videoFeed.style.display = 'none';
  videoPlaceholder.style.display = 'block';
  videoTitle.textContent = reasonTitle;
  videoSub.textContent = reasonSub;
  streamBtn.textContent = 'Start Stream';
  streamBtn.setAttribute('aria-pressed', 'false');
  isStreaming = false;
}

videoFeed.onerror = () => {
  if (isStreaming) {
    stopStreamUi('Stream disconnected', 'Connection lost; click "Start Stream" to reconnect');
  }
};

streamBtn.onclick = () => {
  if (!isStreaming) {
    videoFeed.src = `/stream?t=${Date.now()}`;
    videoFeed.style.display = 'block';
    videoPlaceholder.style.display = 'none';
    streamBtn.textContent = 'Stop Stream';
    streamBtn.setAttribute('aria-pressed', 'true');
    isStreaming = true;
  } else {
    stopStreamUi();
  }
};

drawJoystick(0, 0);
loop();
