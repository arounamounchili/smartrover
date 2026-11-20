import test from 'node:test';
import assert from 'node:assert/strict';
import { register } from 'node:module';

// Register a minimal loader hook so importing `./style.css` in `main.js` succeeds under Node.
register(
  `data:text/javascript,${encodeURIComponent(
    'export async function load(url, context, nextLoad) { if (url.endsWith(".css")) return { format: "module", shortCircuit: true, source: "" }; return nextLoad(url, context); }',
  )}`,
);

function createMockElement(id, extra = {}) {
  const attrs = new Map();
  const listeners = new Map();
  const classes = new Set();

  const el = {
    id,
    textContent: '',
    style: {},
    dataset: {},
    ...extra,
    className: '',
    classList: {
      add: (c) => classes.add(c),
      remove: (c) => classes.delete(c),
      toggle: (c, force) => {
        if (force === undefined) {
          if (classes.has(c)) classes.delete(c);
          else classes.add(c);
        } else if (force) {
          classes.add(c);
        } else {
          classes.delete(c);
        }
      },
      contains: (c) => classes.has(c),
    },
    setAttribute: (k, v) => attrs.set(k, String(v)),
    getAttribute: (k) => attrs.get(k) ?? null,
    removeAttribute: (k) => {
      attrs.delete(k);
      if (k === 'src') delete el.src;
    },
    addEventListener: (type, fn) => {
      if (!listeners.has(type)) listeners.set(type, []);
      listeners.get(type).push(fn);
    },
    dispatch: (type, event = {}) => {
      for (const fn of listeners.get(type) || []) fn(event);
    },
  };
  return el;
}

function createCanvasCtx() {
  const noop = () => {};
  const transforms = [];
  return {
    transforms,
    setTransform: (...args) => transforms.push(args),
    clearRect: noop,
    beginPath: noop,
    arc: noop,
    fill: noop,
    stroke: noop,
    setLineDash: noop,
    moveTo: noop,
    lineTo: noop,
    fillStyle: '',
    strokeStyle: '',
    lineWidth: 1,
  };
}

let caseCounter = 0;

async function bootDashboard(fetchImpl, { devicePixelRatio = 1 } = {}) {
  const ids = [
    'joystick-canvas',
    'gamepad-text',
    'ebrake-btn',
    'adas-btn',
    'mode-pill',
    'sensor-pill',
    'batt-pill',
    'gear-hint',
    'thr-val',
    'str-val',
    'input-source-tag',
    'dist',
    'dist-bar',
    'adas-tag',
    'left-pwm',
    'right-pwm',
    'left-bar',
    'right-bar',
    'link-pill',
    'link-text',
    'video-feed',
    'video-placeholder',
    'video-title',
    'video-sub',
    'stream-toggle',
  ];

  const elements = new Map();
  for (const id of ids) {
    elements.set(id, createMockElement(id));
  }
  const joyCanvas = elements.get('joystick-canvas');
  const joyCtx = createCanvasCtx();
  joyCanvas.width = 176;
  joyCanvas.height = 176;
  joyCanvas.getContext = () => joyCtx;
  joyCanvas.getBoundingClientRect = () => ({ left: 0, top: 0, width: 176, height: 176 });
  joyCanvas.setPointerCapture = () => {};

  const gearButtons = ['1', '2', '3'].map((g) => {
    const btn = createMockElement(`gear-${g}`, { dataset: { gear: g } });
    return btn;
  });

  const windowListeners = new Map();
  const docListeners = new Map();

  globalThis.document = {
    hidden: false,
    getElementById: (id) => elements.get(id),
    querySelectorAll: (sel) => (sel === '.gear-btn' ? gearButtons : []),
    addEventListener: (type, fn) => {
      if (!docListeners.has(type)) docListeners.set(type, []);
      docListeners.get(type).push(fn);
    },
    dispatch: (type, event = {}) => {
      for (const fn of docListeners.get(type) || []) fn(event);
    },
  };

  globalThis.window = {
    devicePixelRatio,
    addEventListener: (type, fn) => {
      if (!windowListeners.has(type)) windowListeners.set(type, []);
      windowListeners.get(type).push(fn);
    },
    dispatch: (type, event = {}) => {
      for (const fn of windowListeners.get(type) || []) fn(event);
    },
  };

  Object.defineProperty(globalThis, 'navigator', {
    value: { getGamepads: () => [] },
    configurable: true,
  });

  let scheduledLoop = null;
  globalThis.setTimeout = (fn) => {
    scheduledLoop = fn;
    return 1;
  };

  let activeFetchPromise = null;
  globalThis.fetch = (...args) => {
    activeFetchPromise = Promise.resolve().then(() => fetchImpl(...args));
    return activeFetchPromise;
  };

  await import(`../src/main.js?case=${++caseCounter}`);
  // Flush the initial loop() call's fetch microtasks.
  await activeFetchPromise?.catch(() => {});
  await new Promise((r) => setImmediate(r));

  async function tick() {
    assert.ok(scheduledLoop, 'expected loop() to schedule next tick');
    const next = scheduledLoop;
    scheduledLoop = null;
    next();
    if (activeFetchPromise) {
      await activeFetchPromise.catch(() => {});
    }
    await new Promise((r) => setImmediate(r));
  }

  return {
    elements,
    gearButtons,
    joyCtx,
    tick,
    el: (id) => elements.get(id),
  };
}

function okResponse(payload) {
  return {
    ok: true,
    status: 200,
    json: async () => payload,
  };
}

test('main.js differentiates STOPPED, LIMITED (ADAS & distance), and CLEAR states', async () => {
  let payload = {
    state: 3, // ADAS STOP
    dist: 120,
    batt: 80,
    tof_ok: true,
    imu_ok: true,
    adas_active: true,
    left_pwm: 0,
    right_pwm: 0,
    rssi: 0,
  };

  const { el, tick } = await bootDashboard(() => okResponse(payload));

  // STOPPED state (d.state === 3)
  assert.equal(el('adas-tag').textContent, 'STOPPED');
  assert.equal(el('adas-tag').className, 'tag tag-stop');
  assert.equal(el('dist-bar').style.background, 'var(--red)');
  assert.equal(el('mode-pill').textContent, 'ADAS STOP');
  assert.equal(el('link-text').textContent, 'Online (AP)');

  // ADAS-active LIMITED state (d.state !== 3, adas_active === true, dist >= 320)
  payload = { ...payload, state: 1, dist: 600, adas_active: true, rssi: -55 };
  await tick();
  assert.equal(el('adas-tag').textContent, 'LIMITED');
  assert.equal(el('adas-tag').className, 'tag tag-warn');
  assert.equal(el('dist-bar').style.background, 'var(--amber)');
  assert.equal(el('link-text').textContent, 'Online (-55 dBm)');

  // Distance-based LIMITED state (adas_active === false, dist < 320)
  payload = { ...payload, state: 1, dist: 250, adas_active: false };
  await tick();
  assert.equal(el('adas-tag').textContent, 'LIMITED');
  assert.equal(el('adas-tag').className, 'tag tag-warn');

  // CLEAR state (dist >= 320, adas_active === false)
  payload = { ...payload, state: 1, dist: 500, adas_active: false };
  await tick();
  assert.equal(el('adas-tag').textContent, 'CLEAR');
  assert.equal(el('adas-tag').className, 'tag tag-ok');
  assert.equal(el('dist').textContent, '500 mm');
  assert.equal(el('dist-bar').style.background, '#e4e4e7');

  // Out-of-range clear sentinel (>= 2000 mm) displays "Out of range"
  payload = { ...payload, state: 1, dist: 2000, adas_active: false };
  await tick();
  assert.equal(el('dist').textContent, 'Out of range');
  assert.equal(el('dist-bar').style.width, '100%');

  // With Assist disabled, close distance displays ASSIST OFF instead of LIMITED
  el('adas-btn').onclick();
  payload = { ...payload, state: 1, dist: 150, adas_active: false };
  await tick();
  assert.equal(el('adas-tag').textContent, 'ASSIST OFF');
  assert.equal(el('adas-tag').className, 'tag tag-neutral');
  assert.equal(el('dist-bar').style.background, '#e4e4e7');
});

test('main.js indicates ToF, IMU, and combined sensor faults', async () => {
  let payload = {
    state: 1,
    dist: 500,
    batt: 75,
    tof_ok: false,
    imu_ok: true,
    adas_active: false,
    left_pwm: 200,
    right_pwm: -200,
    rssi: 0,
  };

  const { el, tick } = await bootDashboard(() => okResponse(payload));

  // ToF fault overrides ADAS tag and distance readout
  assert.equal(el('sensor-pill').textContent, 'TOF ERR');
  assert.equal(el('sensor-pill').className, 'kv-val warn');
  assert.equal(el('dist').textContent, 'FAULT');
  assert.equal(el('dist-bar').style.width, '0%');
  assert.equal(el('adas-tag').textContent, 'SENSOR FAULT');
  assert.equal(el('adas-tag').className, 'tag tag-warn');
  assert.equal(el('left-pwm').textContent, '20%');
  assert.equal(el('right-pwm').textContent, '-20%');
  assert.equal(el('right-bar').style.background, 'var(--amber)');

  // IMU-only fault
  payload = { ...payload, tof_ok: true, imu_ok: false };
  await tick();
  assert.equal(el('sensor-pill').textContent, 'IMU ERR');
  assert.equal(el('sensor-pill').className, 'kv-val fault');
  assert.equal(el('dist').textContent, '500 mm');

  // Combined ToF + IMU fault
  payload = { ...payload, tof_ok: false, imu_ok: false };
  await tick();
  assert.equal(el('sensor-pill').textContent, 'TOF+IMU ERR');
  assert.equal(el('sensor-pill').className, 'kv-val fault');
});

test('main.js transitions to Offline when failCount > 3 and recovers on telemetry', async () => {
  let shouldFail = false;
  const healthyPayload = {
    state: 1,
    dist: 700,
    batt: 90,
    tof_ok: true,
    imu_ok: true,
    adas_active: false,
    left_pwm: 500,
    right_pwm: 500,
    rssi: -48,
  };

  const { el, tick } = await bootDashboard(() => {
    if (shouldFail) throw new Error('network timeout');
    return okResponse(healthyPayload);
  });

  // Initially online
  assert.equal(el('link-pill').className, 'conn-badge online');
  assert.equal(el('link-text').textContent, 'Online (-48 dBm)');

  // Failures 1, 2, 3 must NOT flip UI to Offline yet (failCount <= 3)
  shouldFail = true;
  await tick(); // failCount = 1
  await tick(); // failCount = 2
  await tick(); // failCount = 3
  assert.equal(el('link-pill').className, 'conn-badge online');
  assert.equal(el('adas-tag').textContent, 'CLEAR');

  // Failure 4 (failCount > 3) transitions UI to Offline
  await tick(); // failCount = 4
  assert.equal(el('link-pill').className, 'conn-badge offline');
  assert.equal(el('link-text').textContent, 'Offline');
  assert.equal(el('mode-pill').textContent, 'STANDBY');
  assert.equal(el('sensor-pill').textContent, '--');
  assert.equal(el('batt-pill').textContent, '--%');
  assert.equal(el('dist').textContent, '-- mm');
  assert.equal(el('dist-bar').style.width, '0%');
  assert.equal(el('adas-tag').textContent, 'OFFLINE');
  assert.equal(el('adas-tag').className, 'tag tag-neutral');
  assert.equal(el('left-pwm').textContent, '0%');
  assert.equal(el('right-pwm').textContent, '0%');

  // Successful telemetry resets failCount and restores live UI
  shouldFail = false;
  await tick();
  assert.equal(el('link-pill').className, 'conn-badge online');
  assert.equal(el('link-text').textContent, 'Online (-48 dBm)');
  assert.equal(el('mode-pill').textContent, 'DRIVE');
  assert.equal(el('sensor-pill').textContent, 'OK');
  assert.equal(el('batt-pill').textContent, '90%');
  assert.equal(el('dist').textContent, '700 mm');
  assert.equal(el('adas-tag').textContent, 'CLEAR');
  assert.equal(el('adas-tag').className, 'tag tag-ok');
});

test('main.js toggles camera stream UI and handles stream error recovery', async () => {
  const { el } = await bootDashboard(() =>
    okResponse({
      state: 0,
      dist: 800,
      batt: 85,
      tof_ok: true,
      imu_ok: true,
      adas_active: false,
      left_pwm: 0,
      right_pwm: 0,
      rssi: 0,
    }),
  );

  const streamBtn = el('stream-toggle');
  const videoFeed = el('video-feed');
  const placeholder = el('video-placeholder');

  // Start stream
  streamBtn.onclick();
  assert.match(videoFeed.src, /^\/stream\?t=\d+$/);
  assert.equal(videoFeed.style.display, 'block');
  assert.equal(placeholder.style.display, 'none');
  assert.equal(streamBtn.textContent, 'Stop Stream');
  assert.equal(streamBtn.getAttribute('aria-pressed'), 'true');

  // Stop stream via toggle
  streamBtn.onclick();
  assert.equal(videoFeed.src, undefined);
  assert.equal(videoFeed.style.display, 'none');
  assert.equal(placeholder.style.display, 'block');
  assert.equal(el('video-title').textContent, 'Stream inactive');
  assert.equal(streamBtn.textContent, 'Start Stream');
  assert.equal(streamBtn.getAttribute('aria-pressed'), 'false');

  // Start again, then simulate stream error
  streamBtn.onclick();
  videoFeed.onerror();
  assert.equal(videoFeed.src, undefined);
  assert.equal(videoFeed.style.display, 'none');
  assert.equal(placeholder.style.display, 'block');
  assert.equal(el('video-title').textContent, 'Stream disconnected');
  assert.equal(streamBtn.textContent, 'Start Stream');
  assert.equal(streamBtn.getAttribute('aria-pressed'), 'false');

  // Calling onerror when already stopped is a safe no-op
  videoFeed.onerror();
  assert.equal(el('video-title').textContent, 'Stream disconnected');
});

test('main.js handles gear buttons, ADAS toggle, pointer brake, keyboard shortcuts, and safety blur/visibility resets', async () => {
  const urls = [];
  const { el, gearButtons, tick } = await bootDashboard((url) => {
    urls.push(url);
    return okResponse({
      state: 2, // BRAKE
      dist: 600,
      batt: 15, // < 20% -> fault class
      tof_ok: true,
      imu_ok: true,
      adas_active: false,
      left_pwm: -400,
      right_pwm: 400,
      rssi: 0,
    });
  });

  // Verify low-battery (<20%), negative left_pwm bar, and state=2 (BRAKE warn)
  assert.equal(el('batt-pill').className, 'kv-val fault');
  assert.equal(el('mode-pill').textContent, 'BRAKE');
  assert.equal(el('mode-pill').className, 'kv-val warn');
  assert.equal(el('left-bar').style.background, 'var(--amber)');

  // Gear button clicks
  gearButtons[0].onclick();
  assert.equal(gearButtons[0].getAttribute('aria-pressed'), 'true');
  assert.equal(gearButtons[1].getAttribute('aria-pressed'), 'false');

  // ADAS button click toggles Assist Off -> On
  const adasBtn = el('adas-btn');
  adasBtn.onclick();
  assert.equal(adasBtn.textContent, 'Assist: Off');
  assert.equal(adasBtn.getAttribute('aria-pressed'), 'false');
  adasBtn.onclick();
  assert.equal(adasBtn.textContent, 'Assist: On');
  assert.equal(adasBtn.getAttribute('aria-pressed'), 'true');

  // Pointer brake down, up, and cancel
  const ebrakeBtn = el('ebrake-btn');
  let capturedId = null;
  ebrakeBtn.setPointerCapture = (id) => {
    capturedId = id;
  };
  ebrakeBtn.dispatch('pointerdown', { pointerId: 7 });
  assert.equal(capturedId, 7);
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'true');
  ebrakeBtn.dispatch('pointerup');
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'false');
  ebrakeBtn.dispatch('pointerdown', { pointerId: 8 });
  ebrakeBtn.dispatch('pointercancel');
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'false');

  // Keyboard shortcuts: Digit1, Digit2, Digit3, Space (preventDefault), ArrowUp, KeyW
  let prevented = 0;
  const preventDefault = () => {
    prevented++;
  };
  globalThis.window.dispatch('keydown', { code: 'Digit1', preventDefault });
  assert.equal(gearButtons[0].getAttribute('aria-pressed'), 'true');
  globalThis.window.dispatch('keydown', { code: 'Digit2', preventDefault });
  assert.equal(gearButtons[1].getAttribute('aria-pressed'), 'true');
  globalThis.window.dispatch('keydown', { code: 'Digit3', preventDefault });
  assert.equal(gearButtons[2].getAttribute('aria-pressed'), 'true');
  globalThis.window.dispatch('keydown', { code: 'Space', preventDefault });
  globalThis.window.dispatch('keydown', { code: 'ArrowUp', preventDefault });
  assert.equal(prevented, 2);
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'true');

  await tick();
  assert.match(urls.at(-1), /\/control\?t=1000&s=0&g=3&b=1&a=1$/);

  // Keyup Space (also calls preventDefault to avoid Firefox focused-button activation) and ArrowUp
  globalThis.window.dispatch('keyup', { code: 'Space', preventDefault });
  globalThis.window.dispatch('keyup', { code: 'ArrowUp', preventDefault });
  assert.equal(prevented, 3);
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'false');

  // Modified keystrokes (browser shortcuts) must not shift gears or stick drive keys
  globalThis.window.dispatch('keydown', { code: 'Digit1', metaKey: true, preventDefault });
  globalThis.window.dispatch('keydown', { code: 'KeyW', ctrlKey: true, preventDefault });
  globalThis.window.dispatch('keydown', { code: 'Space', altKey: true, preventDefault });
  assert.equal(gearButtons[2].getAttribute('aria-pressed'), 'true');
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'false');
  assert.equal(prevented, 3);

  // Window blur and document visibilitychange reset held inputs
  globalThis.window.dispatch('keydown', { code: 'Space', preventDefault });
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'true');
  globalThis.window.dispatch('blur');
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'false');

  globalThis.window.dispatch('keydown', { code: 'Space', preventDefault });
  globalThis.document.hidden = false;
  globalThis.document.dispatch('visibilitychange');
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'true');
  globalThis.document.hidden = true;
  globalThis.document.dispatch('visibilitychange');
  assert.equal(ebrakeBtn.getAttribute('aria-pressed'), 'false');
});

test('main.js handles touch joystick clamping, HiDPI scaling, gamepad events/silent drops, fault modes, and HTTP error status', async () => {
  let statusMode = 5; // TILT FAULT
  let httpOk = true;
  const { el, joyCtx, tick } = await bootDashboard(
    () => {
      if (!httpOk) return { ok: false, status: 503, json: async () => ({}) };
      return okResponse({
        state: statusMode,
        dist: 400,
        batt: 60,
        tof_ok: true,
        imu_ok: true,
        adas_active: false,
        left_pwm: 0,
        right_pwm: 0,
        rssi: 0,
      });
    },
    { devicePixelRatio: 2 },
  );

  // HiDPI canvas scales backing buffer once and sets transform
  const joyCanvas = el('joystick-canvas');
  assert.equal(joyCanvas.width, 352);
  assert.equal(joyCanvas.height, 352);
  assert.deepEqual(joyCtx.transforms, [[2, 0, 0, 2, 0, 0]]);

  // State 5 (TILT FAULT), State 6 (LOW BATT), and unknown state (99 -> UNKNOWN)
  assert.equal(el('mode-pill').textContent, 'TILT FAULT');
  assert.equal(el('mode-pill').className, 'kv-val fault');
  statusMode = 6;
  await tick();
  assert.equal(el('mode-pill').textContent, 'LOW BATT');
  assert.equal(el('mode-pill').className, 'kv-val fault');
  statusMode = 99;
  await tick();
  assert.equal(el('mode-pill').textContent, 'UNKNOWN');

  // Touch joystick ignores collapsed canvas (maxR <= 0) without producing NaN
  joyCanvas.getBoundingClientRect = () => ({ left: 0, top: 0, width: 0, height: 0 });
  joyCanvas.dispatch('pointerdown', { pointerId: 0, clientX: 50, clientY: 50 });
  await tick();
  assert.equal(el('thr-val').textContent, '0%');
  assert.equal(el('str-val').textContent, '0%');
  joyCanvas.dispatch('pointerup');

  // Touch joystick pointerdown, pointermove (inside & outside radius), pointerup, pointercancel
  joyCanvas.getBoundingClientRect = () => ({ left: 0, top: 0, width: 176, height: 176 });
  joyCanvas.dispatch('pointermove', { clientX: 120, clientY: 50 }); // ignored while inactive
  joyCanvas.dispatch('pointerdown', { pointerId: 1, clientX: 88, clientY: 50 });
  joyCanvas.dispatch('pointermove', { clientX: 300, clientY: -200 }); // mag > 1.0 clamped
  await tick();
  assert.equal(el('input-source-tag').textContent, 'Pointer');
  joyCanvas.dispatch('pointerup');
  joyCanvas.dispatch('pointercancel');

  // Gamepad connected/disconnected events + Sony DualSense L1/R1 hint & gear/ADAS sync in loop()
  globalThis.window.dispatch('gamepadconnected', {
    gamepad: { id: 'Sony DualSense Wireless Controller' },
  });
  assert.equal(el('gamepad-text').textContent, 'Sony DualSense Wireless Controll');
  assert.equal(el('gamepad-text').classList.contains('active'), true);
  assert.equal(el('gear-hint').textContent, 'L1 / R1');
  globalThis.window.dispatch('gamepaddisconnected');
  assert.equal(el('gamepad-text').textContent, 'No controller detected (WASD or pointer)');
  assert.equal(el('gamepad-text').classList.contains('active'), false);
  assert.equal(el('gear-hint').textContent, 'LB / RB');

  // Poll a connected PlayStation gamepad that holds brake, increments gear, and toggles ADAS
  globalThis.navigator.getGamepads = () => [
    {
      connected: true,
      id: 'Sony DualShock 4',
      axes: [0.2, -0.3, 0.0],
      buttons: {
        1: { pressed: true }, // B/Circle activates brake UI
        3: { pressed: true }, // Y/Triangle toggles ADAS
        5: { pressed: true }, // RB/R1 increments gear 2 -> 3
      },
    },
  ];
  await tick();
  assert.equal(el('gear-hint').textContent, 'L1 / R1');
  assert.equal(el('adas-btn').textContent, 'Assist: Off');
  assert.equal(el('ebrake-btn').getAttribute('aria-pressed'), 'true');
  assert.equal(el('ebrake-btn').classList.contains('active'), true);

  // Poll a non-PlayStation gamepad for LB / RB hint and released brake
  globalThis.navigator.getGamepads = () => [
    {
      connected: true,
      id: 'Xbox Wireless Controller',
      axes: [0, 0, 0],
      buttons: {},
    },
  ];
  await tick();
  assert.equal(el('gear-hint').textContent, 'LB / RB');
  assert.equal(el('gamepad-text').classList.contains('active'), true);
  assert.equal(el('ebrake-btn').getAttribute('aria-pressed'), 'false');
  assert.equal(el('ebrake-btn').classList.contains('active'), false);

  // Silent gamepad drop (no gamepaddisconnected event) & HTTP !r.ok throws
  globalThis.navigator.getGamepads = undefined;
  httpOk = false;
  await tick();
  assert.equal(el('gamepad-text').textContent, 'No controller detected (WASD or pointer)');
  assert.equal(el('gamepad-text').classList.contains('active'), false);
  assert.equal(el('gear-hint').textContent, 'LB / RB');
});
