export const MODE_MAP = {
  0: 'READY',
  1: 'DRIVE',
  2: 'BRAKE',
  3: 'ADAS STOP',
  4: 'STANDBY',
  5: 'TILT FAULT',
  6: 'LOW BATT',
};

export function clampUnit(v) {
  if (!Number.isFinite(v)) return 0;
  return Math.max(-1, Math.min(1, v));
}

export function createControlState() {
  return {
    gear: 2,
    adasEnabled: true,
    pointerBrake: false,
    keyBrake: false,
    touchActive: false,
    touchThrottle: 0.0,
    touchSteering: 0.0,
    keysPressed: new Set(),
    prevLb: false,
    prevRb: false,
    prevY: false,
  };
}

export function setGear(state, gear) {
  const numeric = Number(gear);
  if (!Number.isFinite(numeric)) return state.gear;
  state.gear = Math.max(1, Math.min(3, Math.trunc(numeric)));
  return state.gear;
}

export function toggleAdas(state, forceState) {
  state.adasEnabled = typeof forceState === 'boolean' ? forceState : !state.adasEnabled;
  return state.adasEnabled;
}

export function resetTransientInputs(state) {
  state.keysPressed.clear();
  state.pointerBrake = false;
  state.keyBrake = false;
  state.touchActive = false;
  state.touchThrottle = 0.0;
  state.touchSteering = 0.0;
}

export function sampleControlInputs(state, gamepads = []) {
  let throttle = 0.0;
  let steering = 0.0;
  let gamepadBrake = false;
  let source = 'Idle';

  const gp = Array.from(gamepads || []).find((g) => g && g.connected);

  if (gp) {
    const leftY = -(gp.axes?.[1] || 0);
    const r2 = gp.buttons?.[7]?.value || 0;
    const l2 = gp.buttons?.[6]?.value || 0;
    const trig = r2 - l2;
    throttle = Math.abs(trig) > Math.abs(leftY) ? trig : leftY;

    const leftX = gp.axes?.[0] || 0;
    const rightX = gp.axes?.[2] || 0;
    steering = Math.abs(rightX) > Math.abs(leftX) ? rightX : leftX;

    if (gp.buttons?.[1]?.pressed) gamepadBrake = true;

    const lb = Boolean(gp.buttons?.[4]?.pressed);
    const rb = Boolean(gp.buttons?.[5]?.pressed);
    if (lb && !state.prevLb) setGear(state, state.gear - 1);
    if (rb && !state.prevRb) setGear(state, state.gear + 1);
    state.prevLb = lb;
    state.prevRb = rb;

    const yBtn = Boolean(gp.buttons?.[3]?.pressed);
    if (yBtn && !state.prevY) toggleAdas(state);
    state.prevY = yBtn;

    source = 'Gamepad';
  } else {
    state.prevLb = false;
    state.prevRb = false;
    state.prevY = false;
  }

  if (state.touchActive) {
    throttle = state.touchThrottle;
    steering = state.touchSteering;
    source = 'Pointer';
  }

  let kbThrottle = 0.0;
  let kbSteer = 0.0;
  if (state.keysPressed.has('KeyW') || state.keysPressed.has('ArrowUp')) kbThrottle += 1.0;
  if (state.keysPressed.has('KeyS') || state.keysPressed.has('ArrowDown')) kbThrottle -= 1.0;
  if (state.keysPressed.has('KeyD') || state.keysPressed.has('ArrowRight')) kbSteer += 1.0;
  if (state.keysPressed.has('KeyA') || state.keysPressed.has('ArrowLeft')) kbSteer -= 1.0;

  if (kbThrottle !== 0 || kbSteer !== 0) {
    throttle = kbThrottle;
    steering = kbSteer;
    source = 'Keyboard';
  }

  return {
    throttle: clampUnit(throttle),
    steering: clampUnit(steering),
    gear: state.gear,
    brake: Boolean(state.pointerBrake || state.keyBrake || gamepadBrake),
    adas: state.adasEnabled,
    source,
    gamepadId: gp ? String(gp.id || 'Gamepad') : null,
  };
}

