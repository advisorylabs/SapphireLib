// Tests for the camera's side of the runs onto the Pins: target_approach.cpp's camera math and
// approach controller (ported in vision.js and robot.js), the simulated AI Vision sensor, and
// vision.cpp's PinSight. The math and the controller's run are the same numbers as
// tests/motion/target_approach_test.cpp's. Run:
// node --test tools/sim/test/*.test.js
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const V = require('../js/vision.js');
const R = require('../js/robot.js');
const L = require('../js/mcl.js');
const F = require('../js/field.js');
const { PID } = require('../../analyzer/js/model.js');

const near = (actual, expected, tolerance, label) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: ${actual} vs ${expected}`);

test('camera math matches target_approach_test.cpp', () => {
  const camera = V.cameraMount();
  // 320 across, 74 degrees: the middle is straight ahead, the edges 37 either side
  near(V.cameraBearingDeg(camera, 160), 0, 1e-9, 'middle');
  near(V.cameraBearingDeg(camera, 320), 37, 1e-9, 'right edge');
  near(V.cameraBearingDeg(camera, 0), -37, 1e-9, 'left edge');
  const robot = { xIn: 0, yIn: 0, headingDeg: 90 }; // facing +x
  near(V.cameraBearingToDeg(camera, robot, { xIn: 10, yIn: 0 }), 0, 1e-9, 'straight ahead');
  near(V.cameraBearingToDeg(camera, robot, { xIn: 10, yIn: -10 }), 45, 1e-9, 'right of it (−y, facing +x)');
  assert.equal(V.cameraBearingToDeg(camera, robot, { xIn: -5, yIn: 0 }), null, 'behind');
  // seen 10 degrees right, a target 20in out keeps its 20in and swings onto that line
  const moved = V.onCameraRay(camera, robot, 10, { xIn: 20, yIn: 0 });
  near(Math.hypot(moved.xIn, moved.yIn), 20, 1e-9, 'range kept');
  near(V.cameraBearingToDeg(camera, robot, moved), 10, 1e-9, 'on the line');
  // a camera 6in ahead and 2in right, turned 5 degrees right: the lens is the ray's origin
  const offset = V.cameraMount({ forwardIn: 6, rightIn: 2, yawDeg: 5 });
  const pose = { xIn: 0, yIn: 0, headingDeg: 0 };
  const lens = { xIn: 2, yIn: 6 };
  const onRay = V.onCameraRay(offset, pose, -5, { xIn: 2, yIn: 30 });
  near(onRay.xIn, lens.xIn, 1e-9, 'axis 5 right, seen 5 left: straight ahead of the lens, x');
  near(onRay.yIn, 30, 1e-9, 'and y');
  near(V.cameraBearingToDeg(offset, pose, { xIn: 2, yIn: 30 }), -5, 1e-9, 'and back');
});

test('approachSpeedInPerS: cruise far off, the stop\'s sqrt(2ad) near the end, 0 past it', () => {
  near(R.approachSpeedInPerS(20, 18, 80), 18, 1e-12, 'far off');
  near(R.approachSpeedInPerS(1, 18, 80), Math.sqrt(160), 1e-12, '1in off');
  near(R.approachSpeedInPerS(0, 18, 80), 0, 1e-12, 'there');
  near(R.approachSpeedInPerS(-1, 18, 80), 0, 1e-12, 'past');
  near(R.approachSpeedInPerS(0.1, 18, 0), 18, 1e-12, 'no decel: cruise to the end');
});

// The robot's axis models and tracking gains (devices.cpp), and a plant of them: forward and turn
// on their own models, each wheel capped at 12V. target_approach_test.cpp runs the same one
const kForward = { kS: 0.30, kV: 0.1698, kA: 0.0383 };
const kTurn = { kS: 1.10, kV: 0.0318, kA: 0.0019 };

function axisStep(model, volts, v, dtS) {
  if (v === 0 && Math.abs(volts) <= model.kS) return 0;
  const friction = model.kS * Math.sign(v !== 0 ? v : volts);
  const next = v + ((volts - friction - model.kV * v) / model.kA) * dtS;
  // friction stops it rather than reversing it
  return v !== 0 && Math.sign(next) !== Math.sign(v) && Math.abs(volts) <= model.kS ? 0 : next;
}

function runApproach(start, target, config) {
  const forward = new PID({ gains: { kP: 2.086, kI: 0, kD: 0.396 }, outputLimit: 12 });
  const turn = new PID({ gains: { kP: 0.405, kI: 0, kD: 0.024 }, outputLimit: 12 });
  const approach = new R.TargetApproach(forward, turn);
  const s = { xIn: start.xIn, yIn: start.yIn, headingDeg: start.headingDeg, v: start.speedInPerS, w: 0 };
  approach.begin(config, start.speedInPerS, 0);
  const ticks = [];
  for (let ms = 0; ms <= 5000; ms += 10) {
    const h = s.headingDeg * Math.PI / 180;
    const tick = approach.update({ xIn: s.xIn, yIn: s.yIn, headingDeg: s.headingDeg },
      { xInPerS: s.v * Math.sin(h), yInPerS: s.v * Math.cos(h), headingDegPerS: s.w }, target, kForward, kTurn, ms);
    ticks.push(Object.assign({ ms, speedInPerS: s.v, pose: { xIn: s.xIn, yIn: s.yIn, headingDeg: s.headingDeg } }, tick));
    if (approach.reason !== 'running') break;
    const largest = Math.max(Math.abs(tick.forwardVolts) + Math.abs(tick.turnVolts), 12) / 12;
    for (let i = 0; i < 10; ++i) {
      s.v = axisStep(kForward, tick.forwardVolts / largest, s.v, 0.001);
      s.w = axisStep(kTurn, tick.turnVolts / largest, s.w, 0.001);
      const a = s.headingDeg * Math.PI / 180;
      s.headingDeg += s.w * 0.001;
      s.xIn += s.v * Math.sin(a) * 0.001;
      s.yIn += s.v * Math.cos(a) * 0.001;
    }
  }
  return { ticks, result: approach.result(), end: s };
}

test('TargetApproach: from 24in/s, onto a target 4in to the side, it turns to face it and meets it at cruise', () => {
  // the Triple Pin's: 18in/s through a Pin that seats 5.65in ahead, to 3in past that
  const config = { cruiseInPerS: 18, accelInPerS2: 60, decelInPerS2: 80, endShortIn: 2.65, steerMinIn: 7.65, timeoutMs: 3000 };
  const target = { xIn: 4, yIn: 36 };
  const { ticks, result, end } = runApproach({ xIn: 0, yIn: 0, headingDeg: 0, speedInPerS: 24 }, target, config);
  assert.equal(result.reason, 'settled');
  // the reference eases down from 24 to cruise, never faster than it started
  assert.ok(ticks.every((t) => t.speedRefInPerS <= 24 + 1e-9));
  assert.ok(ticks.some((t) => Math.abs(t.speedRefInPerS - 18) < 1e-9));
  // where the target would seat (5.65in ahead), the robot is at cruise
  const seat = ticks.find((t) => t.distanceIn <= 5.65);
  near(seat.speedInPerS, 18, 1.0, 'speed as the target seats');
  // and facing it: the target within a quarter inch of the robot's centerline
  const a = seat.pose.headingDeg * Math.PI / 180;
  const across = (target.xIn - seat.pose.xIn) * Math.cos(a) - (target.yIn - seat.pose.yIn) * Math.sin(a);
  assert.ok(Math.abs(across) < 0.25, `${across.toFixed(3)}in off the centerline`);
  // it ends at its end, slowing for it: braking from there is the drivetrain's stop()
  near(Math.hypot(target.xIn - end.xIn, target.yIn - end.yIn), 2.65, 0.75, 'where it ends, from the target');
  assert.ok(end.v < 12, `${end.v.toFixed(1)} in/s at the end`);
  // turn's static friction: no steady heading error while it re-aims
  const steering = ticks.filter((t) => t.steering && t.ms > 400);
  assert.ok(steering.every((t) => Math.abs(t.headingErrorDeg) < 1.5), 'within 1.5 degrees once turned');
});

test('TargetApproach: a target that moves (the camera) moves where it steers, with no kick', () => {
  const config = { cruiseInPerS: 18, accelInPerS2: 60, decelInPerS2: 80, endShortIn: 2.65, steerMinIn: 7.65 };
  const forward = new PID({ gains: { kP: 2.086, kI: 0, kD: 0.396 }, outputLimit: 12 });
  const turn = new PID({ gains: { kP: 0.405, kI: 0, kD: 0.024 }, outputLimit: 12 });
  const approach = new R.TargetApproach(forward, turn);
  approach.begin(config, 18, 0);
  const pose = { xIn: 0, yIn: 0, headingDeg: 0 };
  const velocity = { xInPerS: 0, yInPerS: 18, headingDegPerS: 0 };
  approach.update(pose, velocity, { xIn: 0, yIn: 30 }, kForward, kTurn, 0);
  const steady = approach.update(pose, velocity, { xIn: 0, yIn: 30 }, kForward, kTurn, 10);
  near(steady.turnVolts, 0, 1e-9, 'on it: no turn');
  // 2in right, 30in out: about 3.8 degrees, and the turn's volts are what that error asks for,
  // with D on the robot's own turn rate (still 0), not the target's jump
  const moved = approach.update(pose, velocity, { xIn: 2, yIn: 30 }, kForward, kTurn, 20);
  const errorDeg = Math.atan2(2, 30) * 180 / Math.PI;
  near(moved.headingErrorDeg, errorDeg, 1e-9, 'heading error');
  near(moved.turnVolts, 1.10 + 0.405 * errorDeg, 1e-9, 'kS and P, no D kick');
});

test('the simulated AI Vision: a Pin straight ahead is a tall detection in the image\'s middle; one lying down is wider than tall', () => {
  const field = F.override();
  field.objects = [];
  const world = { timeMs: 0, truth: { xIn: 0, yIn: 0, headingDeg: 0 }, pastTruth: () => world.truth };
  let pins = [{ color: 'yellow', xIn: 0, yIn: 30, state: 'standing' }];
  const camera = new V.AiVision(world, () => pins, field, { noisePx: 0, forwardIn: 0 });
  const names = camera.classNames();
  const [pin] = camera.objects();
  // an AI object detection, labeled the yellow/yellow Pin, sure of it
  assert.equal(pin.type, 'object');
  assert.equal(names[pin.id], 'YellowPin');
  assert.ok(pin.element.score >= V.kMinScore, `score ${pin.element.score}`);
  near(pin.element.xoffset + pin.element.width / 2, 160, 1, 'the middle');
  assert.ok(pin.element.height > 3 * pin.element.width, `${pin.element.width}x${pin.element.height}`);
  // a red Pin is a Red/Yellow Pin; one in the claw isn't seen at all
  pins = [{ color: 'red', xIn: 0, yIn: 30, state: 'standing' }, { color: 'yellow', xIn: 0, yIn: 30, state: 'held' }];
  world.timeMs = 100;
  assert.deepEqual(camera.objects().map((o) => names[o.id]), ['RedYellowPin']);
  // a cross on the Autonomous Line, 30in ahead, across the view: a Pin lying down, wider than tall
  field.objects = [{ kind: 'lyingPin', color: 'red', xIn: 0, yIn: 30, headingDeg: 270, innerIn: 1.6 }];
  pins = [];
  world.timeMs = 200;
  const [lying] = camera.objects();
  assert.equal(names[lying.id], 'RedYellowPin');
  assert.ok(lying.element.width > lying.element.height, `${lying.element.width}x${lying.element.height}`);
  // unplugged: no frame and no class names at all
  const unplugged = new V.AiVision(world, () => pins, field, { unplugged: true });
  assert.equal(unplugged.objects(), null);
  assert.equal(unplugged.classNames(), null);
});

test('the yellow/yellow Pin\'s class is found by its name, and no other Pin\'s is', () => {
  for (const name of ['Yellow/Yellow Pin', 'YellowPin', 'yellow_yellow_pin', 'YELLOW YELLOW PIN']) {
    assert.ok(V.isPinClassName(name), name);
  }
  for (const name of ['RedYellowPin', 'Blue/Yellow Pin', 'RedBluePin', 'Cup', 'Yellow Cup']) {
    assert.ok(!V.isPinClassName(name), name);
  }
  assert.equal(V.pinClassIn(['Cup', 'RedBluePin', 'BlueYellowPin', 'RedYellowPin', 'YellowPin']), 4);
  assert.equal(V.pinClassIn(['Cup', 'Yellow/Yellow Pin', 'Red/Yellow Pin']), 1);
  assert.equal(V.pinClassIn(['Cup', 'RedYellowPin']), -1);
  assert.equal(V.pinClassIn(null), -1);
});

test('PinSight moves the target onto the Pin it sees, and never onto another yellow Pin outside its gate', () => {
  const field = F.override();
  field.objects = [];
  const world = { timeMs: 0, truth: { xIn: 0, yIn: 0, headingDeg: 0 }, pastTruth: () => world.truth };
  // the Pin really is 3in right of where odometry has it, and another stands 30 degrees left
  const pins = [{ color: 'yellow', xIn: 3, yIn: 24, state: 'standing' },
    { color: 'yellow', xIn: -24 * Math.tan(30 * Math.PI / 180), yIn: 24, state: 'standing' }];
  const camera = new V.AiVision(world, () => pins, field, { noisePx: 0, forwardIn: 0 });
  const sight = new V.PinSight({ xIn: 0, yIn: 24 }, camera, { mount: { forwardIn: 0 }, clock: () => world.timeMs });
  const target = { xIn: 0, yIn: 24 };
  for (let ms = 0; ms <= 400; ms += 10) {
    world.timeMs = ms;
    sight.refine(world.truth, target);
  }
  assert.ok(sight.locked);
  // onto the line to the real Pin, at the range odometry had
  const bearing = V.cameraBearingToDeg(V.cameraMount(), world.truth, target);
  near(bearing, Math.atan2(3, 24) * 180 / Math.PI, 0.3, 'bearing to the target');
  near(Math.hypot(target.xIn, target.yIn), 24, 0.05, 'range kept');
  // a Pin 30 degrees off the target is past even the first sighting's gate
  const lonely = new V.PinSight({ xIn: 0, yIn: 24 }, new V.AiVision(world, () => [pins[1]], field, { noisePx: 0, forwardIn: 0 }),
    { mount: { forwardIn: 0 }, clock: () => world.timeMs });
  const still = { xIn: 0, yIn: 24 };
  for (let ms = 500; ms <= 900; ms += 10) {
    world.timeMs = ms;
    lonely.refine(world.truth, still);
  }
  assert.ok(!lonely.locked);
  assert.deepEqual(still, { xIn: 0, yIn: 24 });
});

test('PinSight takes only the yellow/yellow Pin\'s class: a Red/Yellow Pin right on the target is never it', () => {
  const field = F.override();
  field.objects = [];
  const world = { timeMs: 0, truth: { xIn: 0, yIn: 0, headingDeg: 0 }, pastTruth: () => world.truth };
  // a red Pin standing where odometry has the yellow/yellow one, which really is 3in right of it
  const pins = [{ color: 'red', xIn: 0, yIn: 24, state: 'standing' }, { color: 'yellow', xIn: 3, yIn: 24, state: 'standing' }];
  const camera = new V.AiVision(world, () => pins, field, { noisePx: 0, forwardIn: 0 });
  const sight = new V.PinSight({ xIn: 0, yIn: 24 }, camera, { mount: { forwardIn: 0 }, clock: () => world.timeMs });
  const target = { xIn: 0, yIn: 24 };
  for (let ms = 0; ms <= 400; ms += 10) {
    world.timeMs = ms;
    sight.refine(world.truth, target);
  }
  near(V.cameraBearingToDeg(V.cameraMount(), world.truth, target), Math.atan2(3, 24) * 180 / Math.PI, 0.3,
    'onto the yellow/yellow Pin');
  // a model that names no yellow/yellow Pin: nothing to look for, and the target stays put
  const named = new V.AiVision(world, () => pins, field, { noisePx: 0, forwardIn: 0, classNames: ['Cup', 'RedYellowPin'] });
  const blind = new V.PinSight({ xIn: 0, yIn: 24 }, named, { mount: { forwardIn: 0 }, clock: () => world.timeMs });
  const still = { xIn: 0, yIn: 24 };
  for (let ms = 500; ms <= 900; ms += 10) {
    world.timeMs = ms;
    blind.refine(world.truth, still);
  }
  assert.ok(!blind.locked);
  assert.deepEqual(still, { xIn: 0, yIn: 24 });
});
