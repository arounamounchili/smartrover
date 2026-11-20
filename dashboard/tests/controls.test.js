import test from 'node:test';
import assert from 'node:assert/strict';
import {
  clampUnit,
  createControlState,
  resetTransientInputs,
  sampleControlInputs,
  setGear,
  toggleAdas,
} from '../src/controls.js';

test('keyboard WASD and arrow keys produce normalized throttle and steering', () => {
  const state = createControlState();
  state.keysPressed.add('KeyW');
  state.keysPressed.add('KeyD');

  const out = sampleControlInputs(state);
  assert.equal(out.throttle, 1.0);
  assert.equal(out.steering, 1.0);
  assert.equal(out.source, 'Keyboard');
});

test('resetTransientInputs clears held keys, pointer stick, and independent brakes', () => {
  const state = createControlState();
  state.keysPressed.add('KeyW');
  state.pointerBrake = true;
  state.keyBrake = true;
  state.touchActive = true;
  state.touchThrottle = 0.8;
  state.touchSteering = -0.5;

  resetTransientInputs(state);

  const out = sampleControlInputs(state);
  assert.equal(out.throttle, 0);
  assert.equal(out.steering, 0);
  assert.equal(out.brake, false);
  assert.equal(out.source, 'Idle');
});

test('pointer brake and key brake do not cancel each other', () => {
  const state = createControlState();
  state.pointerBrake = true;
  state.keyBrake = true;
  assert.equal(sampleControlInputs(state).brake, true);

  state.pointerBrake = false;
  assert.equal(sampleControlInputs(state).brake, true);

  state.keyBrake = false;
  assert.equal(sampleControlInputs(state).brake, false);
});

test('gamepad triggers, gear edge toggles, and ADAS toggle behave correctly', () => {
  const state = createControlState();
  const gp = {
    connected: true,
    id: 'Test Pad',
    axes: [0.0, 0.0, -0.4],
    buttons: {
      1: { pressed: false },
      3: { pressed: true },
      4: { pressed: false },
      5: { pressed: true },
      6: { value: 0.1 },
      7: { value: 0.8 },
    },
  };

  const first = sampleControlInputs(state, [gp]);
  assert.ok(Math.abs(first.throttle - 0.7) < 1e-6);
  assert.ok(Math.abs(first.steering + 0.4) < 1e-6);
  assert.equal(first.gear, 3);
  assert.equal(first.adas, false);

  // Holding RB and Y on next tick must not double-toggle
  const second = sampleControlInputs(state, [gp]);
  assert.equal(second.gear, 3);
  assert.equal(second.adas, false);
});

test('setGear and toggleAdas clamp bounds cleanly', () => {
  const state = createControlState();
  assert.equal(setGear(state, 99), 3);
  assert.equal(setGear(state, 0), 1);
  assert.equal(toggleAdas(state, false), false);
  assert.equal(toggleAdas(state), true);
});

test('clampUnit handles NaN, infinities, and out-of-range values', () => {
  assert.equal(clampUnit(NaN), 0);
  assert.equal(clampUnit(Infinity), 0);
  assert.equal(clampUnit(-Infinity), 0);
  assert.equal(clampUnit(2.5), 1);
  assert.equal(clampUnit(-2.5), -1);
  assert.equal(clampUnit(0.42), 0.42);
  assert.equal(clampUnit(-1), -1);
  assert.equal(clampUnit(1), 1);
});

test('setGear ignores NaN and non-finite inputs while truncating valid numbers', () => {
  const state = createControlState(); // gear = 2
  assert.equal(setGear(state, NaN), 2);
  assert.equal(setGear(state, undefined), 2);
  assert.equal(setGear(state, 'invalid'), 2);
  assert.equal(setGear(state, Infinity), 2);
  assert.equal(setGear(state, -Infinity), 2);
  assert.equal(setGear(state, '3'), 3);
  assert.equal(setGear(state, 1.9), 1);
});

test('opposing keyboard inputs cancel each other out', () => {
  const state = createControlState();
  state.keysPressed.add('KeyW');
  state.keysPressed.add('KeyS');
  state.keysPressed.add('KeyA');
  state.keysPressed.add('KeyD');

  const out = sampleControlInputs(state);
  assert.equal(out.throttle, 0);
  assert.equal(out.steering, 0);
  assert.equal(out.source, 'Idle');

  state.keysPressed.clear();
  state.keysPressed.add('ArrowUp');
  state.keysPressed.add('ArrowDown');
  state.keysPressed.add('ArrowLeft');
  state.keysPressed.add('ArrowRight');

  const arrowOut = sampleControlInputs(state);
  assert.equal(arrowOut.throttle, 0);
  assert.equal(arrowOut.steering, 0);
  assert.equal(arrowOut.source, 'Idle');
});

test('input priority follows Keyboard > Pointer > Gamepad', () => {
  const state = createControlState();
  const gp = {
    connected: true,
    id: 'Xbox Controller',
    axes: [0.3, -0.4, 0.0],
    buttons: {},
  };

  // Gamepad alone
  let out = sampleControlInputs(state, [gp]);
  assert.equal(out.source, 'Gamepad');
  assert.equal(out.gamepadId, 'Xbox Controller');
  assert.ok(Math.abs(out.throttle - 0.4) < 1e-6);

  // Pointer overrides Gamepad
  state.touchActive = true;
  state.touchThrottle = 0.6;
  state.touchSteering = -0.2;
  out = sampleControlInputs(state, [gp]);
  assert.equal(out.source, 'Pointer');
  assert.equal(out.throttle, 0.6);
  assert.equal(out.steering, -0.2);

  // Keyboard overrides Pointer and Gamepad
  state.keysPressed.add('KeyS');
  out = sampleControlInputs(state, [gp]);
  assert.equal(out.source, 'Keyboard');
  assert.equal(out.throttle, -1);
  assert.equal(out.steering, 0);

  // Releasing keyboard falls back to Pointer
  state.keysPressed.delete('KeyS');
  out = sampleControlInputs(state, [gp]);
  assert.equal(out.source, 'Pointer');

  // Deactivating pointer falls back to Gamepad even if stale touch values remain
  state.touchActive = false;
  out = sampleControlInputs(state, [gp]);
  assert.equal(out.source, 'Gamepad');
  assert.ok(Math.abs(out.throttle - 0.4) < 1e-6);
});

test('gamepad gear decrement/increment boundaries and brake button work cleanly', () => {
  const state = createControlState(); // starts at gear = 2
  const makePad = (lb, rb, brake) => ({
    connected: true,
    id: 'Pad',
    axes: [0, 0, 0],
    buttons: {
      1: { pressed: brake },
      4: { pressed: lb },
      5: { pressed: rb },
    },
  });

  // LB decrements 2 -> 1, and clamps at 1 on subsequent press
  assert.equal(sampleControlInputs(state, [makePad(true, false, true)]).gear, 1);
  assert.equal(sampleControlInputs(state, [makePad(false, false, true)]).brake, true);
  assert.equal(sampleControlInputs(state, [makePad(true, false, false)]).gear, 1);
  assert.equal(sampleControlInputs(state, [makePad(false, false, false)]).brake, false);

  // RB increments 1 -> 2 -> 3, and clamps at 3
  assert.equal(sampleControlInputs(state, [makePad(false, true, false)]).gear, 2);
  sampleControlInputs(state, [makePad(false, false, false)]);
  assert.equal(sampleControlInputs(state, [makePad(false, true, false)]).gear, 3);
  sampleControlInputs(state, [makePad(false, false, false)]);
  assert.equal(sampleControlInputs(state, [makePad(false, true, false)]).gear, 3);

  // resetTransientInputs preserves gear & adasEnabled while clearing transient inputs
  state.keysPressed.add('KeyW');
  state.keyBrake = true;
  state.touchActive = true;
  toggleAdas(state, false);
  resetTransientInputs(state);
  const afterReset = sampleControlInputs(state, null);
  assert.equal(afterReset.gear, 3);
  assert.equal(afterReset.adas, false);
  assert.equal(afterReset.source, 'Idle');
  assert.equal(afterReset.brake, false);

  // Fallback gamepadId when id is empty string
  const unnamedOut = sampleControlInputs(state, [{ connected: true, id: '' }]);
  assert.equal(unnamedOut.gamepadId, 'Gamepad');
});

