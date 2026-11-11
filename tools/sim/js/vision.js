/*
 * SapphireLib simulator: vision.js
 *
 * The AI Vision sensor, and what the robot program can do with it: steer a run onto a Pin by
 * camera here as on the robot (no routine does now; the Triple Pin's runs hand their last third
 * to an approach() without a camera):
 *
 *   cameraBearingDeg(), cameraBearingToDeg(),
 *   onCameraRay()                    src/sapphirelib/motion/target_approach.cpp
 *   PinSight                         src/robot/vision.cpp
 *   AiVision                         the sensor: what it reports of the Pins in its view
 *
 * The simulated sensor is a pinhole camera with square pixels, 320 across and 240 down, seeing
 * horizontalFovDeg across, running its AI model of the game elements: every Pin and Cup in its
 * view, standing on the floor or in a Goal, is an object detection (its AI classifications) the
 * size of its outline in the image, labeled with its class (classNames: the names VEX's Override
 * playground gives the elements, YellowPin the yellow/yellow Pin) and a score. A Pin lying down
 * (the Autonomous Line's crosses) is wider than tall. It takes a frame every periodMs, of where the
 * robot was latencyMs before, with noisePx of noise on each detection's edges. Nothing hides
 * anything (no occlusion), and every number in SENSOR is made up, the class names included, since
 * nothing measures them yet: change them here once the robot is measured. world.settings.camera,
 * if a run gives one, overrides them: { unplugged: true } for a robot without the sensor, a
 * different mount for one measured wrong, other classNames for a model named differently.
 *
 * Plain script: window.SIM.vision in a browser, require('./vision.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./mcl.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.vision = factory(root.SIM.mcl);
  }
})(function (L) {
  'use strict';

  const kDegToRad = Math.PI / 180;
  const kRadToDeg = 180 / Math.PI;

  // --- camera math (target_approach.cpp) ----------------------------------------------

  /** CameraMount's defaults: an AI Vision Sensor looking straight ahead from the tracking center. */
  function cameraMount(c = {}) {
    return Object.assign({ imageWidthPx: 320, horizontalFovDeg: 74, forwardIn: 0, rightIn: 0, yawDeg: 0 }, c);
  }

  function focalPx(camera) {
    const halfFovRad = Math.min(Math.max(camera.horizontalFovDeg, 1), 179) * 0.5 * kDegToRad;
    return camera.imageWidthPx * 0.5 / Math.tan(halfFovRad);
  }

  // where the lens is on the field, and which way it looks
  function lensOf(camera, robot) {
    const h = robot.headingDeg * kDegToRad;
    return {
      xIn: robot.xIn + camera.forwardIn * Math.sin(h) + camera.rightIn * Math.cos(h),
      yIn: robot.yIn + camera.forwardIn * Math.cos(h) - camera.rightIn * Math.sin(h),
      axisDeg: robot.headingDeg + camera.yawDeg,
    };
  }

  /** cameraBearingDeg(): degrees off the camera's axis of a column of its image, positive right. */
  function cameraBearingDeg(camera, columnPx) {
    return Math.atan((columnPx - camera.imageWidthPx * 0.5) / focalPx(camera)) * kRadToDeg;
  }

  /** cameraFocalPx(): pixels across for 1in across, 1in ahead of the lens. */
  const cameraFocalPx = (camera) => focalPx(camera);

  /** cameraDepthIn(): how far ahead of the lens a field point is, along its axis; null behind it. */
  function cameraDepthIn(camera, robot, point) {
    const lens = lensOf(camera, robot);
    const a = lens.axisDeg * kDegToRad;
    const along = (point.xIn - lens.xIn) * Math.sin(a) + (point.yIn - lens.yIn) * Math.cos(a);
    return along <= 1e-6 ? null : along;
  }

  /** cameraBearingToDeg(): where a field point should show up, off the axis; null behind the lens. */
  function cameraBearingToDeg(camera, robot, point) {
    const lens = lensOf(camera, robot);
    const a = lens.axisDeg * kDegToRad;
    const dx = point.xIn - lens.xIn;
    const dy = point.yIn - lens.yIn;
    const along = dx * Math.sin(a) + dy * Math.cos(a);
    const across = dx * Math.cos(a) - dy * Math.sin(a);
    if (along <= 1e-6) return null;
    return Math.atan2(across, along) * kRadToDeg;
  }

  /** onCameraRay(): the target moved onto the line the camera sees it along, its range kept. */
  function onCameraRay(camera, robot, bearingDeg, prior) {
    const lens = lensOf(camera, robot);
    const rangeIn = Math.hypot(prior.xIn - lens.xIn, prior.yIn - lens.yIn);
    const ray = (lens.axisDeg + bearingDeg) * kDegToRad;
    return { xIn: lens.xIn + rangeIn * Math.sin(ray), yIn: lens.yIn + rangeIn * Math.cos(ray) };
  }

  // --- the simulated sensor -----------------------------------------------------------

  // made up: see the file comment
  const SENSOR = {
    imageWidthPx: 320,
    imageHeightPx: 240,
    horizontalFovDeg: 74,
    // where it really is on the robot: config.hpp's kVision*
    forwardIn: 6.0,
    rightIn: 0.0,
    yawDeg: 0.0,
    // the lens above the floor, looking level
    lensHeightIn: 8.0,
    periodMs: 33,
    latencyMs: 35,
    noisePx: 0.7,
    seed: 11,
    unplugged: false,
    // the AI model's classes, in id order, and how sure it is of each detection (0-100)
    classNames: ['Cup', 'RedBluePin', 'BlueYellowPin', 'RedYellowPin', 'YellowPin'],
    score: 90,
  };
  const kPinLengthIn = 6.5;
  const kPinAcrossIn = 1.6;
  const kCupAcrossIn = 3.15;
  const kCupHeightIn = 3.5;
  // where a Pin stands on a Goal: on top of the Goal, a Pin's length up for each below it
  const kGoalTopIn = { red: 3.25, blue: 3.25, neutral: 5.77, center: 8.77 };
  // each Pin's class by its color (field.js's and macros.js's one color: yellow is yellow/yellow,
  // an alliance's Pins half that color and half yellow)
  const kPinClassByColor = { yellow: 'YellowPin', red: 'RedYellowPin', blue: 'BlueYellowPin' };

  class AiVision {
    /**
     * `world` the world; `pins()` the Pins as macros.js keeps them ({ color, xIn, yIn, state, goal,
     * level }); `field` field.js's field, for its Goals and its lying Pins; `settings` SENSOR's
     * fields to change.
     */
    constructor(world, pins, field, settings = {}) {
      this.world = world;
      this.pins = pins;
      this.field = field;
      this.s = Object.assign({}, SENSOR, settings);
      this.rng = new L.Rng(this.s.seed);
      this.frameAtMs = null;
      this.cached = [];
    }

    /** The mount, as CameraMount has it. */
    mount() {
      return cameraMount({ imageWidthPx: this.s.imageWidthPx, horizontalFovDeg: this.s.horizontalFovDeg,
        forwardIn: this.s.forwardIn, rightIn: this.s.rightIn, yawDeg: this.s.yawDeg });
    }

    /** get_class_name(): the model's class names, in id order, or null with no sensor. */
    classNames() {
      return this.s.unplugged ? null : this.s.classNames.slice();
    }

    /**
     * get_all_objects(): the latest frame's detections, [{ id, type: 'object', element: { xoffset,
     * yoffset, width, height, score } }] with id the class, or null with no sensor
     * (get_object_count()'s PROS_ERR).
     */
    objects() {
      if (this.s.unplugged) return null;
      const nowMs = this.world.timeMs;
      const atMs = Math.floor(nowMs / this.s.periodMs) * this.s.periodMs;
      if (atMs !== this.frameAtMs) {
        this.frameAtMs = atMs;
        const pose = this.world.pastTruth(nowMs - atMs + this.s.latencyMs) || this.world.truth;
        this.cached = this.look(pose);
      }
      return this.cached;
    }

    /** What it sees from `pose`: every Pin's and Cup's outline in the image, labeled with its class. */
    look(pose) {
      const s = this.s;
      const camera = this.mount();
      const f = focalPx(camera);
      const lens = lensOf(camera, pose);
      const a = lens.axisDeg * kDegToRad;
      const toCamera = (xIn, yIn) => {
        const dx = xIn - lens.xIn;
        const dy = yIn - lens.yIn;
        return { along: dx * Math.sin(a) + dy * Math.cos(a), across: dx * Math.cos(a) - dy * Math.sin(a) };
      };
      const column = (p) => s.imageWidthPx / 2 + f * p.across / p.along;
      const row = (zIn, along) => s.imageHeightPx / 2 - f * (zIn - s.lensHeightIn) / along;
      const out = [];
      const add = (className, left, right, top, bottom) => {
        const id = s.classNames.indexOf(className);
        if (id < 0) return;
        const noise = () => this.rng.gaussian() * s.noisePx;
        const l = Math.max(0, left + noise());
        const r = Math.min(s.imageWidthPx, right + noise());
        const t = Math.max(0, top + noise());
        const b = Math.min(s.imageHeightPx, bottom + noise());
        if (r - l < 1 || b - t < 1) return;
        const score = Math.round(Math.min(100, Math.max(0, s.score + this.rng.gaussian() * 3)));
        out.push({ id, type: 'object', element: { xoffset: Math.round(l), yoffset: Math.round(t),
          width: Math.round(r - l), height: Math.round(b - t), score } });
      };
      // something standing: its middle, how wide and tall, and what it stands on
      const standing = (className, xIn, yIn, acrossIn, heightIn, baseIn) => {
        const c = toCamera(xIn, yIn);
        if (c.along < 1.0) return;
        const half = (f * acrossIn / 2) / c.along;
        add(className, column(c) - half, column(c) + half, row(baseIn + heightIn, c.along), row(baseIn, c.along));
      };
      // the Pins standing, on the floor or in a Goal
      for (const p of this.pins()) {
        if (p.state === 'held') continue;
        let baseIn = 0;
        if (p.state === 'scored' && p.goal) {
          const goal = this.field.goals.find((g) => g.name === p.goal);
          baseIn = (goal ? kGoalTopIn[goal.kind] || 0 : 0) + (Math.max(p.level, 1) - 1) * kPinLengthIn;
        }
        standing(kPinClassByColor[p.color] || 'YellowPin', p.xIn, p.yIn, kPinAcrossIn, kPinLengthIn, baseIn);
      }
      for (const o of this.field.objects) {
        if (o.kind === 'cup') standing('Cup', o.xIn, o.yIn, kCupAcrossIn, kCupHeightIn, 0);
        if (o.kind !== 'lyingPin') continue;
        // a Pin lying down, from its inner end to its outer one
        const h = o.headingDeg * kDegToRad;
        const ends = [o.innerIn, o.innerIn + kPinLengthIn].map((d) => toCamera(o.xIn + d * Math.sin(h), o.yIn + d * Math.cos(h)));
        if (ends.some((e) => e.along < 1.0)) continue;
        const columns = ends.flatMap((e) => [column(e) - (f * kPinAcrossIn / 2) / e.along, column(e) + (f * kPinAcrossIn / 2) / e.along]);
        const near = Math.min(...ends.map((e) => e.along));
        add(kPinClassByColor[o.color] || 'YellowPin', Math.min(...columns), Math.max(...columns), row(kPinAcrossIn, near), row(0, near));
      }
      return out;
    }
  }

  // --- robot/vision.cpp ---------------------------------------------------------------

  // the yellow/yellow Pin's class: the one whose name, letters only and lower case, is one of these
  const kPinClassNames = ['yellowyellowpin', 'yellowpin'];
  const kMinScore = 40;

  /** vision.cpp's isPinClassName(): "Yellow/Yellow Pin" and "YellowPin" are; "RedYellowPin" isn't. */
  function isPinClassName(name) {
    return kPinClassNames.includes(String(name).toLowerCase().replace(/[^a-z]/g, ''));
  }

  /** vision.cpp's pinClassId() lookup: the yellow/yellow Pin's class among `names`, -1 if none. */
  function pinClassIn(names) {
    return names ? names.findIndex(isPinClassName) : -1;
  }
  const kMinPinHeightPx = 12;
  const kMinPinTallness = 1.3;
  const kAcquireGateDeg = 25.0;
  const kPinHeightIn = 6.5;
  const kMinPinHeightFraction = 0.55;
  const kTrackGateDeg = 8.0;
  const kLostFrames = 4;
  const kSmoothingMs = 60.0;
  const kFrameLatencyMs = 50;
  const kHistory = 16;

  /**
   * PinSight: one run's look at one Pin, moving the approach's target onto the yellow Pin the
   * camera sees (vision.cpp). `sensor` an AiVision; `mount` where the program thinks the camera is
   * (CameraMount); `clock()` the time; `log(text)` the terminal. trace has every frame it looked at.
   */
  class PinSight {
    constructor(pin, sensor, { mount, clock, log = () => {} }) {
      this.plan = { xIn: pin.xIn, yIn: pin.yIn };
      this.current = { xIn: pin.xIn, yIn: pin.yIn };
      this.sensor = sensor;
      this.camera = cameraMount(mount);
      this.clock = clock;
      this.log = log;
      this.locked = false;
      this.frames = 0;
      this.history = [];
      this.lastRefineMs = 0;
      this.lastFrameKey = null;
      // what the latest frame showed of the Pin: { bearingDeg, seenFrom }, null if it didn't
      this.sighting = null;
      // where the last frame that showed it put it, and how many new frames since have missed it
      this.seenPin = null;
      this.missedFrames = 0;
      this.warned = false;
      // the yellow/yellow Pin's class, once the sensor names it
      this.pinClass = -1;
      this.trace = [];
    }

    pin() {
      return { xIn: this.current.xIn, yIn: this.current.yIn };
    }

    movedIn() {
      return Math.hypot(this.current.xIn - this.plan.xIn, this.current.yIn - this.plan.yIn);
    }

    poseAt(ms) {
      let best = null;
      for (const entry of this.history) {
        if (!best || Math.abs(entry.ms - ms) < Math.abs(best.ms - ms)) best = entry;
      }
      return best ? best.pose : { xIn: 0, yIn: 0, headingDeg: 0 };
    }

    /** The approach's refiner: `target` ({ xIn, yIn }) moved onto the Pin the camera sees. */
    refine(pose, target) {
      const now = this.clock();
      const sinceMs = this.history.length ? now - this.lastRefineMs : 0;
      this.lastRefineMs = now;
      this.history.push({ ms: now, pose: Object.assign({}, pose) });
      if (this.history.length > kHistory) this.history.shift();
      const pin = this.pin();
      target.xIn = pin.xIn;
      target.yIn = pin.yIn;

      // the yellow/yellow Pin's class: without one, there's nothing to look for
      if (this.pinClass < 0) this.pinClass = pinClassIn(this.sensor.classNames());
      const objects = this.pinClass < 0 ? null : this.sensor.objects();
      if (objects === null) {
        if (!this.warned) {
          this.warned = true;
          // a sensor that answers but names no such class, or none answering at all
          this.log(this.pinClass < 0 && this.sensor.objects() !== null
            ? 'AI Vision names no yellow/yellow Pin class: steering by odometry'
            : 'no AI Vision sensor answering: steering by odometry');
        }
        return;
      }
      // a new frame: look for the Pin in it; the same frame as last tick keeps what it showed.
      // Every detection keys the frame, so one with no Pin in it still counts when anything changes
      const detections = objects.filter((o) => o.type === 'object' && o.id === this.pinClass);
      const key = JSON.stringify(objects.map((o) => [o.id, o.type, o.element]));
      if (key !== this.lastFrameKey) {
        this.lastFrameKey = key;
        this.sighting = null;
        const pins = detections.map((o) => o.element)
          .filter((e) => e.score >= kMinScore && e.height >= kMinPinHeightPx && e.height >= kMinPinTallness * e.width)
          .map((e) => ({ centerPx: e.xoffset + e.width / 2, heightPx: e.height, score: e.score }));
        const seenFrom = this.poseAt(now - kFrameLatencyMs);
        // following the Pin from the last frame that showed it, or looking for it around the target
        const tracking = this.seenPin !== null && this.missedFrames < kLostFrames;
        const expected = cameraBearingToDeg(this.camera, seenFrom, tracking ? this.seenPin : pin);
        const gate = tracking ? kTrackGateDeg : kAcquireGateDeg;
        // looking: at least kMinPinHeightFraction as tall as a Pin as far off as the target
        const depth = cameraDepthIn(this.camera, seenFrom, pin);
        const minHeightPx = tracking || depth === null ? 0 : kMinPinHeightFraction * focalPx(this.camera) * kPinHeightIn / depth;
        let best = null;
        let bestBearing = 0;
        let bestOff = gate;
        for (const p of expected === null ? [] : pins) {
          const bearing = cameraBearingDeg(this.camera, p.centerPx);
          const off = Math.abs(bearing - expected);
          if (off > gate || p.heightPx < minHeightPx) continue;
          // tracking: nearest the last sighting; looking: the tallest, the nearest Pin
          if (!best || (tracking ? off < bestOff : p.heightPx > best.heightPx)) {
            best = p;
            bestBearing = bearing;
            bestOff = off;
          }
        }
        if (best) {
          this.sighting = { bearingDeg: bestBearing, seenFrom };
          this.seenPin = onCameraRay(this.camera, seenFrom, bestBearing, pin);
          this.missedFrames = 0;
          this.frames++;
          if (!tracking) {
            this.locked = true;
            this.log(`locked on a Pin at ${best.centerPx.toFixed(0)}px, ${bestBearing.toFixed(1)} deg ` +
              `(${(bestBearing - expected).toFixed(1)} off where odometry had it)`);
          }
        } else if (this.seenPin) {
          this.missedFrames++;
        }
        // the Pin, or with none taken, the tallest sure Pin the gates turned down
        const shown = best || pins.reduce((t, p) => (!t || p.heightPx > t.heightPx ? p : t), null);
        this.trace.push({ tMs: now, seen: !!best, px: shown ? shown.centerPx : -1,
          bearing: best ? bestBearing : shown ? cameraBearingDeg(this.camera, shown.centerPx) : 0,
          expected, pinXIn: pin.xIn, pinYIn: pin.yIn, blobs: pins.length, pins: detections.length,
          heightPx: shown ? shown.heightPx : 0, score: shown ? shown.score : 0, objects: objects.length,
          classes: objects.reduce((bits, o) => (o.type === 'object' && o.id < 32 ? bits | (1 << o.id) : bits), 0) >>> 0 });
      }
      // every tick, swing the target toward the line the latest sighting showed it on, keeping its
      // range from the lens
      const nowDeg = this.sighting ? cameraBearingToDeg(this.camera, this.sighting.seenFrom, pin) : null;
      if (nowDeg !== null) {
        const pull = 1 - Math.exp(-Math.min(sinceMs, 100) / kSmoothingMs);
        this.current = onCameraRay(this.camera, this.sighting.seenFrom, nowDeg + pull * (this.sighting.bearingDeg - nowDeg), pin);
        target.xIn = this.current.xIn;
        target.yIn = this.current.yIn;
      }
    }
  }

  return {
    cameraMount, cameraBearingDeg, cameraFocalPx, cameraDepthIn, cameraBearingToDeg, onCameraRay,
    isPinClassName, pinClassIn, kMinScore,
    SENSOR, AiVision, PinSight,
    kAcquireGateDeg, kTrackGateDeg, kLostFrames, kMinPinHeightFraction, kSmoothingMs, kFrameLatencyMs, kMinPinHeightPx, kMinPinTallness,
  };
});
