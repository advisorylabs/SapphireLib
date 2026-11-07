/*
 * SapphireLib simulator: macros.js
 *
 * The robot's lift, claw and extender for autonomous routines: the calls
 * src/robot/autons.cpp makes from include/robot/macros.hpp, against a
 * simulated lift and Override's Pins, so a routine in routines.js can score
 * the way the robot does.
 *
 *   C++ (autons.cpp)                                    simulator (routines.js)
 *   macros::setLiftLevel(macros::Goal::alliance, 2);    m.setLiftLevel(m.Goal.alliance, 2);
 *   macros::setLiftDeg(macros::liftScoreDeg());         m.setLiftDeg(m.liftScoreDeg());
 *   macros::waitForLift(1000);                          yield* m.waitForLift(1000);
 *   macros::setClawClosed(false);                       m.setClawClosed(false);
 *   macros::setAutoClaw(true);                          m.setAutoClaw(true);
 *   if (!macros::clawClosed()) return;                  if (!m.clawClosed()) return;
 *
 * What's ported from src/robot/macros.cpp, constant for constant: the lift's
 * ladder (ALLIANCE, MEDIUM, CENTER), its loop (kP 0.1 V/deg, kLiftGravityVolts
 * up, never more than kLiftMaxDownVolts down, the seat at the bottom, a 20ms
 * task), liftScoreDeg() (kScoreExtraDipDeg past halfway to the level below),
 * waitForLift() (within kLiftToleranceDeg), and autoClaw (its own 10ms task:
 * armed once the claw's sensor has seen the claw empty for kAutoClawClearMs,
 * a grab once the reading, less odometry's speed toward the claw times
 * kClawLeadMs, its own 10ms tick and the reading's age, is at kGrabMm, with
 * the lift down at the bottom; the speed limit at kClawSpeedVolts until
 * kClawSpeedOffDelayMs after the grab). The lead rule itself is
 * clawtiming.js's ruleFires(), which the claw timing page (claw.html)
 * simulates.
 *
 * What's made up, since no file measures it, so change it here when the
 * robot is measured: the lift's model is Auto-Tune's (kS 0.71, kV 0.0096, kA
 * 0.0004, kG 1.08, from the comment on kLiftGains), but without the motors'
 * current limits; the pistons take kPistonMs to move; the claw holds a
 * Pin kClawReachIn ahead of the tracking center, where the Triple Pin
 * routine's grab of its second Pin put it (moveToPose(-27.5, -20, 133) for
 * the Pin at (-23.5, -23.5)); and its distance sensor looks forward from
 * kClawSensorForwardIn ahead of the tracking center, where a Pin a little
 * behind the jaws' middle reads kGrabMm, measuring every kClawSensorPeriodMs
 * and kClawSensorDelayMs late, as world.js's distance sensors do.
 *
 * The Pins: every standing Pin of field.override(), the ones standing in the
 * neutral and center Goals counting as those Goals' first, and the preload in
 * the claw. The claw closing with a Pin between its jaws picks it up; opening
 * puts it down where the claw is: on a Goal if it's within kGoalCatchIn of
 * the Goal's middle with the lift above the bottom, at the next level of that
 * Goal's stack, and standing on the floor otherwise. scored() says what went
 * where, and whether each went on at the level the lift was at (a stack's
 * n-th Pin is its Ln).
 *
 * The lift runs whether or not anything drives the chassis: create() adds it
 * to the robot's tasks (robot.tasks, run every 10ms tick).
 *
 * Plain script: window.SIM.macros in a browser, require('./macros.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./mcl.js'), require('./field.js'), require('./clawtiming.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.macros = factory(root.SIM.mcl, root.SIM.field, root.SIM.clawtiming);
  }
})(function (L, F, CT) {
  'use strict';

  // --- src/robot/macros.cpp ---
  const kPieceDetectMm = 125;
  const kGrabMm = 60;
  const kClawLeadMs = 75.0;
  const kGrabMaxMm = 250;
  const kClawReadingMaxAgeMs = 50;
  const kAutoClawClearMs = 400;
  const kScoreExtraDipDeg = 30.0;
  const kLiftKp = 0.1;
  const kLiftOutputLimit = 12.7;
  const kLiftMaxVolts = 12.0;
  const kLiftMaxDownVolts = 9.0;
  const kLiftGravityVolts = 1.08;
  const kLiftRestDeg = 2.0;
  const kLiftToleranceDeg = 15.0;
  const kLiftPeriodMs = 20;
  const kClawSpeedVolts = 5.0;
  const kClawSpeedOffDelayMs = 350;
  const kAutonClawPollMs = 10;
  // the ladder, in motor degrees: each goal's bottom, then its levels (firstLevel is the L number
  // of its entry 1)
  const LADDER = {
    alliance: { name: 'ALLIANCE', deg: [0, 225, 370, 635, 910, 1170, 1430], firstLevel: 1 },
    medium: { name: 'MEDIUM', deg: [0, 425, 705, 975, 1245, 1520], firstLevel: 2 },
    center: { name: 'CENTER', deg: [0, 630, 915, 1170, 1450], firstLevel: 2 },
  };
  const Goal = { alliance: 'alliance', medium: 'medium', center: 'center' };

  // --- the simulated mechanisms (made up: see the file comment) ---
  // Auto-Tune's lift model, volts per motor degree (/s, /s^2), and gravity's pull in volts
  const kLiftModel = { kS: 0.71, kV: 0.0096, kA: 0.0004, kG: 1.08 };
  const kLiftTopDeg = 1550.0;
  const kPistonMs = 60;
  const kClawReachIn = 5.3;
  const kClawSensorForwardIn = 1.9;
  const kClawSensorPeriodMs = 33;
  const kClawSensorDelayMs = 15;
  // autoClaw's rule, in clawtiming.js's terms
  const kClawRule = { rule: 'leadAge', grabMm: kGrabMm, leadMs: kClawLeadMs, loopMs: kAutonClawPollMs,
    maxReachMm: kGrabMaxMm, maxAgeMs: kClawReadingMaxAgeMs };
  // how far off the claw's middle a Pin can be and still be between its jaws: either side, short of
  // it, and past it, where the back of the claw stops it
  const kJawHalfWidthIn = 1.2;
  const kJawFrontIn = 1.0;
  const kJawBackIn = 1.5;
  // how far off a Goal's middle a Pin let go of still lands on it
  const kGoalCatchIn = 1.25;
  // the lift above this counts as up, for landing on a Goal rather than dropping on the floor
  const kLiftUpDeg = 50.0;

  /** sapphirelib::TimedFlag: a value, and since when it's been that. */
  class TimedFlag {
    constructor(value, nowMs) {
      this.value = value;
      this.sinceMs = nowMs;
    }

    set(value, nowMs) {
      if (value !== this.value) {
        this.value = value;
        this.sinceMs = nowMs;
      }
    }

    falseFor(ms, nowMs) {
      return !this.value && nowMs - this.sinceMs >= ms;
    }
  }

  /** The Pins on `field` (field.override()): standing on the floor, or already in a Goal. */
  function startingPins(field) {
    const pins = [];
    for (const o of field.objects) {
      if (o.kind !== 'pin') continue;
      const goal = field.goals.find((g) => Math.hypot(g.xIn - o.xIn, g.yIn - o.yIn) < 0.5);
      pins.push({ color: o.color, xIn: o.xIn, yIn: o.yIn, state: goal ? 'scored' : 'standing',
        goal: goal ? goal.name : null, level: goal ? 1 : 0, byRoutine: false });
    }
    return pins;
  }

  /**
   * One robot's macros. `auton` is its auton functions (auton.js's create()), for setSpeedLimit()
   * and for waiting with the background motions running. `options.field` is the field (Override's
   * at the world's size by default); `options.preload` false starts with the claw empty and open;
   * `options.clawRule` changes autoClaw's rule, in clawtiming.js's terms, for trying another (the
   * old one: { rule: 'fixed', fixedMm: 125 }).
   */
  function create(robot, auton, options = {}) {
    const world = robot.world;
    const field = options.field || F.override(world.settings.fieldSizeIn);
    const now = () => world.timeMs;
    const preload = options.preload !== false;
    const clawRule = { ...kClawRule, ...(options.clawRule || {}) };

    const pins = startingPins(field);
    // the preload, in the alliance color of the side the robot starts on
    if (preload) {
      pins.push({ color: F.sideOf(world.truth.xIn, world.truth.yIn), xIn: 0, yIn: 0, state: 'held', goal: null, level: 0,
        byRoutine: false });
    }
    // how many Pins each Goal's stack has
    const stacks = new Map(field.goals.map((g) => [g.name, pins.filter((p) => p.goal === g.name).length]));

    const state = {
      table: 'alliance',
      level: 0, // the ladder's entry, 0 the bottom
      target: 0.0,
      commandGen: 0,
      // the lift, as the motors' encoders read it, and how fast it's going
      liftDeg: 0.0,
      liftDegPerS: 0.0,
      liftVolts: 0.0,
      lastLiftStepMs: -kLiftPeriodMs,
      settled: false,
      settledGen: -1,
      // the pistons: what they were told, and when they get there
      clawClosed: preload,
      clawMovedAtMs: -kPistonMs,
      jawsClosed: preload,
      extended: false,
      autoClawArmed: false,
      autoClaw: { on: false, grabMs: null },
      lastClawPollMs: -kAutonClawPollMs,
      // the claw sensor's latest sample (when it measured, what it read), and readClawSensor()'s
      // last reading and when it changed
      clawSample: { atMs: null, mm: 9999 },
      clawReading: { mm: -1, changedMs: 0 },
    };
    const clawLoaded = new TimedFlag(false, now());
    // what the C++ would print on the terminal, or log as a "macro" event
    const log = (text) => robot.log('macros', text);

    const ladder = () => LADDER[state.table];
    const levelDeg = (entry = state.level) => ladder().deg[entry];
    const levelNumber = (entry = state.level) => (entry === 0 ? 0 : ladder().firstLevel + entry - 1);

    // where the claw holds a Pin, and the robot's forward and right, from where the robot really is
    // (or was: `t` a pose from world.pastTruth())
    function clawFrame(t = world.truth) {
      const a = t.headingDeg * L.kDegToRad;
      const forward = [Math.sin(a), Math.cos(a)];
      const right = [Math.cos(a), -Math.sin(a)];
      return { t, forward, right, xIn: t.xIn + kClawReachIn * forward[0], yIn: t.yIn + kClawReachIn * forward[1] };
    }

    const held = () => pins.find((p) => p.state === 'held') || null;

    /**
     * The claw's distance sensor, in mm: its latest sample, taken every kClawSensorPeriodMs of where
     * the robot was kClawSensorDelayMs before.
     */
    function clawSensorMm() {
      const nowMs = now();
      const atMs = Math.floor(nowMs / kClawSensorPeriodMs) * kClawSensorPeriodMs;
      if (atMs !== state.clawSample.atMs) {
        const pose = world.pastTruth(nowMs - atMs + kClawSensorDelayMs) || world.truth;
        state.clawSample = { atMs, mm: measureClaw(pose) };
      }
      return state.clawSample.mm;
    }

    /** What the claw's sensor reads with the robot at `pose`: the nearest Pin in front, or the one held. */
    function measureClaw(pose) {
      const pinRadius = field.pinIn / 2;
      if (held()) return Math.round((kClawReachIn - kClawSensorForwardIn - pinRadius) * 25.4);
      const f = clawFrame(pose);
      let best = Infinity;
      for (const p of pins) {
        // a Pin on a Goal is as visible as one on the floor
        if (p.state === 'held') continue;
        const dx = p.xIn - f.t.xIn;
        const dy = p.yIn - f.t.yIn;
        const along = dx * f.forward[0] + dy * f.forward[1] - kClawSensorForwardIn;
        const across = dx * f.right[0] + dy * f.right[1];
        if (along <= 0 || Math.abs(across) > kJawHalfWidthIn) continue;
        best = Math.min(best, along - pinRadius);
      }
      return Number.isFinite(best) && best * 25.4 <= 2000 ? Math.max(0, Math.round(best * 25.4)) : 9999;
    }

    const pieceInClaw = (mm) => mm <= kPieceDetectMm;
    const liftDown = () => state.liftDeg <= levelDeg(0) + kLiftToleranceDeg;

    // readClawSensor(): the reading, noting when it last changed
    function readClawSensor(nowMs) {
      const mm = clawSensorMm();
      if (mm !== state.clawReading.mm) state.clawReading = { mm, changedMs: nowMs };
      return mm;
    }

    // clawApproachMmPerMs(): odometry's speed along the way the robot faces, never below 0
    function clawApproachMmPerMs() {
      const s = robot.odometry.snapshot();
      const a = s.rawPose.headingDeg * L.kDegToRad;
      const inPerS = s.rawVelocity.xInPerS * Math.sin(a) + s.rawVelocity.yInPerS * Math.cos(a);
      return Math.max(0, (inPerS * 25.4) / 1000);
    }

    // autoClawAim(): where the Pin will be as the jaws shut if the claw fires now, and whether to
    function autoClawAim(mm, nowMs) {
      const ageMs = Math.min(nowMs - state.clawReading.changedMs, kClawReadingMaxAgeMs);
      const approachMmPerMs = clawApproachMmPerMs();
      const r = CT.ruleFires(clawRule, mm, ageMs, approachMmPerMs);
      return { readingMm: mm, ageMs, approachMmPerMs, aimMm: r.predictedMm, fires: r.fires };
    }

    // --- the claw ---

    // the jaws shutting on whatever's between them, or letting go of what they held
    function jawsClose() {
      state.jawsClosed = true;
      if (held()) return;
      const f = clawFrame();
      for (const p of pins) {
        if (p.state !== 'standing') continue;
        const dx = p.xIn - f.xIn;
        const dy = p.yIn - f.yIn;
        const along = dx * f.forward[0] + dy * f.forward[1];
        const across = dx * f.right[0] + dy * f.right[1];
        if (along >= -kJawBackIn && along <= kJawFrontIn && Math.abs(across) <= kJawHalfWidthIn) {
          p.state = 'held';
          log(`claw closed on the ${p.color} pin at (${p.xIn.toFixed(1)}, ${p.yIn.toFixed(1)})`);
          return;
        }
      }
      log('claw closed on nothing');
    }

    function jawsOpen() {
      state.jawsClosed = false;
      const p = held();
      if (!p) return;
      const f = clawFrame();
      p.xIn = f.xIn;
      p.yIn = f.yIn;
      p.byRoutine = true;
      const goal = field.goals.find((g) => Math.hypot(g.xIn - f.xIn, g.yIn - f.yIn) <= kGoalCatchIn);
      if (goal && state.liftDeg > kLiftUpDeg) {
        const level = stacks.get(goal.name) + 1;
        stacks.set(goal.name, level);
        Object.assign(p, { state: 'scored', goal: goal.name, level, xIn: goal.xIn, yIn: goal.yIn,
          liftLevel: levelNumber(), offIn: Math.hypot(goal.xIn - f.xIn, goal.yIn - f.yIn), atMs: now() });
        log(`scored on the ${goal.name}: its L${level}, ${p.offIn.toFixed(2)}in off its middle` +
          (p.liftLevel !== level ? ` (the lift was at L${p.liftLevel})` : ''));
      } else {
        p.state = 'standing';
        log(`let go of a pin on the floor at (${f.xIn.toFixed(1)}, ${f.yIn.toFixed(1)})`);
      }
    }

    // --- the lift ---

    function sendLift(deg) {
      if (Number.isNaN(deg)) return;
      state.target = deg;
      state.commandGen++;
    }

    // the lift's loop, as computePositionCommand() runs it with the lift's config
    function liftCommand() {
      const target = state.target;
      const position = state.liftDeg;
      let volts;
      if (target > 0.0) {
        volts = Math.min(Math.max(kLiftKp * (target - position), -kLiftOutputLimit), kLiftOutputLimit) + kLiftGravityVolts;
      } else if (position > kLiftRestDeg) {
        volts = Math.min(Math.min(Math.max(kLiftKp * (target - position), -kLiftOutputLimit), kLiftOutputLimit), -0.0);
      } else {
        volts = 0.0;
      }
      return Math.min(Math.max(volts, -kLiftMaxDownVolts), kLiftMaxVolts);
    }

    // one millisecond of the lift's physics: the motors' volts against friction, back-EMF and gravity
    function liftPhysicsMs() {
      const m = kLiftModel;
      const dt = 0.001;
      const v = state.liftDegPerS;
      const push = state.liftVolts - m.kG;
      const moving = Math.abs(v) > 1e-6;
      if (!moving && Math.abs(push) <= m.kS) {
        // stuck: static friction holds it
        state.liftDegPerS = 0.0;
      } else {
        const friction = Math.sign(moving ? v : push) * m.kS;
        const next = v + ((push - friction - m.kV * v) / m.kA) * dt;
        // through zero, it stops for this step, and static friction decides the next
        state.liftDegPerS = moving && Math.sign(next) !== Math.sign(v) ? 0.0 : next;
      }
      state.liftDeg += state.liftDegPerS * dt;
      if (state.liftDeg <= 0.0) {
        state.liftDeg = 0.0;
        state.liftDegPerS = Math.max(0.0, state.liftDegPerS);
      } else if (state.liftDeg >= kLiftTopDeg) {
        state.liftDeg = kLiftTopDeg;
        state.liftDegPerS = Math.min(0.0, state.liftDegPerS);
      }
    }

    function liftStep(nowMs) {
      state.liftVolts = liftCommand();
      // settleTimeMs 0: settled the first step it's within tolerance of this command's target
      state.settled = Math.abs(state.liftDeg - state.target) <= kLiftToleranceDeg;
      state.settledGen = state.commandGen;
      state.lastLiftStepMs = nowMs;
    }

    // --- autoClaw (autonClawStep()) ---

    function endAutoClaw(why) {
      if (!state.autoClaw.on) return;
      state.autoClaw.on = false;
      state.autoClaw.grabMs = null;
      auton.setSpeedLimit(12.0);
      log(`autoClaw off (${why})`);
    }

    function grabReady(aim) {
      return !state.clawClosed && state.autoClawArmed && state.level === 0 && liftDown() && aim.fires;
    }

    function autoClawStep(nowMs) {
      if (!state.autoClaw.on) return;
      const mm = readClawSensor(nowMs);
      clawLoaded.set(pieceInClaw(mm), nowMs);
      if (clawLoaded.falseFor(kAutoClawClearMs, nowMs)) state.autoClawArmed = true;
      const aim = autoClawAim(mm, nowMs);
      if (state.autoClaw.grabMs === null && grabReady(aim)) {
        setPiston(true, nowMs);
        state.autoClaw.grabMs = nowMs;
        state.lastGrab = aim;
        log(`autoClaw grabbed (sensor ${mm} mm, ${(aim.approachMmPerMs * 1000 / 25.4).toFixed(1)} in/s, ` +
          `aimed at ${Math.round(aim.aimMm)} mm)`);
      }
      if (state.autoClaw.grabMs !== null && nowMs - state.autoClaw.grabMs >= kClawSpeedOffDelayMs) endAutoClaw('grabbed');
    }

    function setPiston(closed, nowMs) {
      if (closed !== state.clawClosed) state.clawMovedAtMs = nowMs;
      state.clawClosed = closed;
    }

    // every 10ms tick, after the world has moved
    function task(nowMs) {
      for (let i = 0; i < 10; ++i) liftPhysicsMs();
      if (nowMs - state.lastLiftStepMs >= kLiftPeriodMs) liftStep(nowMs);
      if (nowMs - state.lastClawPollMs >= kAutonClawPollMs) {
        state.lastClawPollMs = nowMs;
        autoClawStep(nowMs);
      }
      if (state.clawClosed !== state.jawsClosed && nowMs - state.clawMovedAtMs >= kPistonMs) {
        if (state.clawClosed) jawsClose();
        else jawsOpen();
      }
      // a held Pin rides in the claw
      const p = held();
      if (p) {
        const f = clawFrame();
        p.xIn = f.xIn;
        p.yIn = f.yIn;
      }
    }
    robot.tasks.push(task);

    const m = {
      Goal,

      setClawClosed(closed) {
        if (state.clawClosed && !closed) {
          state.autoClawArmed = false;
          clawLoaded.set(pieceInClaw(clawSensorMm()), now());
          state.autoClaw.grabMs = null;
        }
        setPiston(closed, now());
        log(`setClawClosed(${closed})`);
      },

      clawClosed: () => state.clawClosed,

      setAutoClaw(on) {
        if (!on) {
          endAutoClaw('off');
          return;
        }
        if (state.autoClaw.on) return;
        state.autoClaw.on = true;
        state.autoClaw.grabMs = null;
        auton.setSpeedLimit(kClawSpeedVolts);
        log('autoClaw on');
      },

      autoClaw: () => state.autoClaw.on,

      setExtended(out) {
        state.extended = out;
        log(`setExtended(${out})`);
      },

      extended: () => state.extended,

      setLiftLevel(goal, level) {
        if (!LADDER[goal]) throw new Error(`setLiftLevel(): no goal "${goal}"`);
        state.table = goal;
        const t = ladder();
        const top = t.deg.length - 1;
        let entry = level <= 0 ? 0 : level - t.firstLevel + 1;
        entry = Math.min(Math.max(entry, level > 0 ? 1 : 0), top);
        if (levelNumber(entry) !== level) log(`setLiftLevel(): ${t.name} has no L${level}, so L${levelNumber(entry)}`);
        state.level = entry;
        sendLift(levelDeg());
      },

      setLiftDeg(deg) {
        sendLift(deg);
      },

      liftScoreDeg() {
        if (state.level === 0) return levelDeg(0);
        const below = levelDeg(state.level - 1);
        return Math.max((levelDeg() + below) / 2 - kScoreExtraDipDeg, below);
      },

      /** waitForLift(timeoutMs), with the background motions running meanwhile. */
      *waitForLift(timeoutMs) {
        const gen = state.commandGen;
        return yield* auton.waitUntil(() => state.settledGen === gen && gen === state.commandGen && state.settled, timeoutMs);
      },

      // only R2's scores count, and update() never runs in autonomous
      scoreCount: () => 0,
      pinInClaw: () => pieceInClaw(clawSensorMm()),
      liftAtBottom: () => state.liftDeg <= 0.5,

      // --- for the page and the tests ---

      /** The lift, the claw and the goal and level: what the field draws by the robot. */
      status() {
        return {
          liftDeg: state.liftDeg, targetDeg: state.target, goal: ladder().name, level: levelNumber(),
          clawClosed: state.clawClosed, extended: state.extended, autoClaw: state.autoClaw.on,
          sensorMm: clawSensorMm(), claw: clawFrame(), lastGrab: state.lastGrab || null,
        };
      },

      pins: () => pins,

      /** The Pins the routine put on a Goal, in the order it did: { goal, level, liftLevel, offIn, atMs }. */
      scored: () => pins.filter((p) => p.state === 'scored' && p.byRoutine)
        .sort((a, b) => a.atMs - b.atMs)
        .map((p) => ({ goal: p.goal, level: p.level, liftLevel: p.liftLevel, offIn: p.offIn, atMs: p.atMs })),
    };
    robot.macros = m;
    return m;
  }

  return {
    create,
    Goal,
    LADDER,
    startingPins,
    kClawReachIn,
    kClawSensorForwardIn,
    kPieceDetectMm,
    kGrabMm,
    kClawLeadMs,
    kAutonClawPollMs,
    kClawSpeedVolts,
    kPistonMs,
  };
});
