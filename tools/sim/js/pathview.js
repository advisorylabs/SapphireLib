/*
 * SapphireLib simulator: pathview.js
 *
 * The Paths tab: draw a trajectory chain on the field, split it into
 * sections with their own limits, see the speed the planner gives it, drive
 * it in the simulator, and copy it out as C++ for src/robot/autons.cpp (and
 * paste that back in to keep editing). The robot plans the same chain with
 * the same planner (trajectory.js is its port), so what this shows is what
 * the robot drives, given the same axis models. Planned on Override's field
 * (field.js): it says where the routine starts against SG1 and what the
 * robot's footprint runs into on the way, and turns a routine into the other
 * alliance's or the other Quadrant's.
 *
 * The path document, and its C++ in both directions, are plain functions
 * that also load under Node for the tests. build() wires the tab, in a
 * browser only.
 *
 * A document is { name, waypoints: [{ xIn, yIn, headingDeg }] (headingDeg
 * null for none), breaks, sections }: section k ends at waypoint breaks[k]
 * (each section after the first starts from the previous one's last point),
 * and sections[k] is { constraints, stopAtEnd }. A waypoint may also say what
 * the robot's claw and extender do on reaching it: claw 'open' or 'close',
 * extender 'out' or 'in' (see ACTIONS), absent for nothing. They go out as a
 * second C++ list beside the sections, for robot::followPath()
 * (include/robot/paths.hpp).
 *
 * Plain script: window.SIM.pathview in a browser, require('./pathview.js') in
 * Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory(require('./trajectory.js'), require('./field.js'));
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.pathview = factory(root.SIM.trajectory, root.SIM.field);
  }
})(function (T, F) {
  'use strict';

  const kStorageKey = 'sapphire-sim.paths.v1';
  // the footprint, kept apart from the paths: it's the robot's, not any one path's
  const kRobotStorageKey = 'sapphire-sim.robot.v1';
  const kSnapIn = 0.5;
  const kFieldHalfIn = 70.25;
  const AXES = [['forward', 'Fwd', 'in'], ['strafe', 'Strafe', 'in'], ['turn', 'Turn', '°']];
  // each section's color, by the page's CSS tokens (render.js reads the same names)
  const SECTION_COLORS = ['series-1', 'series-3', 'series-7', 'series-5', 'series-4', 'series-2'];
  // What a point can have the robot's claw and extender do: robot::PathAction's kinds
  // (include/robot/paths.hpp), in the same order, so keep the two the same. `field` is the
  // waypoint's field for that mechanism and `value` what it holds for this one.
  const ACTIONS = [
    { field: 'claw', value: 'open', cpp: 'clawOpen', label: 'claw open' },
    { field: 'claw', value: 'close', cpp: 'clawClose', label: 'claw close' },
    { field: 'extender', value: 'out', cpp: 'extenderOut', label: 'ext out' },
    { field: 'extender', value: 'in', cpp: 'extenderIn', label: 'ext in' },
  ];
  // each mechanism's field and name, in ACTIONS' order
  const MECHANISMS = [['claw', 'Claw'], ['extender', 'Extender']];

  // --- The document -----------------------------------------------------------------------------

  function clone(value) {
    return JSON.parse(JSON.stringify(value));
  }

  function emptyDoc(name = 'path') {
    return { name, waypoints: [], breaks: [], sections: [{ constraints: T.trajectoryConstraints(), stopAtEnd: false }] };
  }

  /** A document from a chain of sections (TrajectorySection-shaped objects). */
  function fromSections(sections, name = 'path') {
    const doc = { name, waypoints: [], breaks: [], sections: [] };
    sections.forEach((section, k) => {
      for (const w of section.waypoints) {
        doc.waypoints.push({ xIn: w.xIn, yIn: w.yIn, headingDeg: w.headingDeg === undefined ? null : w.headingDeg });
      }
      if (k + 1 < sections.length) doc.breaks.push(doc.waypoints.length - 1);
      doc.sections.push({ constraints: T.trajectoryConstraints(section.constraints), stopAtEnd: !!section.stopAtEnd });
    });
    if (doc.sections.length === 0) doc.sections.push({ constraints: T.trajectoryConstraints(), stopAtEnd: false });
    return doc;
  }

  /** The chain generateTrajectory() takes. */
  function toSections(doc) {
    const out = [];
    let start = 0;
    for (let k = 0; k < doc.sections.length; ++k) {
      const end = k < doc.breaks.length ? doc.breaks[k] : doc.waypoints.length - 1;
      const waypoints = doc.waypoints.slice(start, end + 1).map((w) => {
        const point = { xIn: w.xIn, yIn: w.yIn };
        if (w.headingDeg !== null && w.headingDeg !== undefined) point.headingDeg = w.headingDeg;
        return point;
      });
      out.push({ waypoints, constraints: clone(doc.sections[k].constraints), stopAtEnd: doc.sections[k].stopAtEnd });
      start = end + 1;
    }
    return out;
  }

  /** The section waypoint j is in. A section's last point (the junction) is its own. */
  function sectionOf(doc, j) {
    let k = 0;
    while (k < doc.breaks.length && doc.breaks[k] < j) ++k;
    return k;
  }

  /** Merge section k + 1 into section k: the joined section keeps k's limits and k + 1's stop. */
  function mergeSections(doc, k) {
    if (k < 0 || k >= doc.breaks.length) return;
    doc.breaks.splice(k, 1);
    doc.sections[k].stopAtEnd = doc.sections[k + 1].stopAtEnd;
    doc.sections.splice(k + 1, 1);
  }

  /** Breaks must sit strictly inside the path; drop any that don't, merging their sections. */
  function tidyBreaks(doc) {
    for (let k = doc.breaks.length - 1; k >= 0; --k) {
      const b = doc.breaks[k];
      if (b < 1 || b > doc.waypoints.length - 2 || (k > 0 && doc.breaks[k - 1] >= b)) mergeSections(doc, k);
    }
  }

  function insertWaypoint(doc, index, point) {
    const w = { xIn: point.xIn, yIn: point.yIn, headingDeg: point.headingDeg === undefined ? null : point.headingDeg };
    for (const [field] of MECHANISMS) if (point[field]) w[field] = point[field];
    // the first point is where the robot starts: it always has a heading
    if (doc.waypoints.length === 0 && w.headingDeg === null) w.headingDeg = 0;
    doc.waypoints.splice(index, 0, w);
    for (let k = 0; k < doc.breaks.length; ++k) if (doc.breaks[k] >= index) doc.breaks[k]++;
    tidyBreaks(doc);
    return index;
  }

  function addWaypoint(doc, point) {
    return insertWaypoint(doc, doc.waypoints.length, point);
  }

  function removeWaypoint(doc, index) {
    if (index < 0 || index >= doc.waypoints.length) return;
    const k = doc.breaks.indexOf(index);
    if (k >= 0) mergeSections(doc, k);
    doc.waypoints.splice(index, 1);
    for (let i = 0; i < doc.breaks.length; ++i) if (doc.breaks[i] > index) doc.breaks[i]--;
    if (index === 0 && doc.waypoints.length && doc.waypoints[0].headingDeg === null) doc.waypoints[0].headingDeg = 0;
    tidyBreaks(doc);
  }

  /** End a section at waypoint j; the new section after it copies the limits it splits off from. */
  function splitAt(doc, j) {
    if (j < 1 || j > doc.waypoints.length - 2 || doc.breaks.includes(j)) return false;
    const k = sectionOf(doc, j);
    doc.breaks.splice(k, 0, j);
    const original = doc.sections[k];
    doc.sections.splice(k + 1, 0, { constraints: clone(original.constraints), stopAtEnd: original.stopAtEnd });
    original.stopAtEnd = false;
    return true;
  }

  /**
   * The same chain driven backward: the points in reverse, each junction's stop moved to the
   * section that now ends there, and the start facing the way the old end did.
   */
  function reverse(doc) {
    const n = doc.waypoints.length;
    if (n === 0) return;
    // the heading the old end faced: its own, or the last one set before it
    let endHeading = 0;
    for (const w of doc.waypoints) if (w.headingDeg !== null) endHeading = w.headingDeg;
    const points = doc.waypoints.slice().reverse();
    if (points[0].headingDeg === null) points[0].headingDeg = endHeading;
    const breaks = doc.breaks.map((b) => n - 1 - b).reverse();
    // reversed section k ends where old section (count - 2 - k) did
    const sections = doc.sections.slice().reverse().map((s, k, all) => ({
      constraints: s.constraints,
      stopAtEnd: k + 1 < all.length ? all[k + 1].stopAtEnd : false,
    }));
    Object.assign(doc, { waypoints: points, breaks, sections });
  }

  const wrapHeading = (deg) => ((deg % 360) + 360) % 360;

  /**
   * The same routine for the other alliance. Override's field turns into itself under a half turn
   * about the middle, red for blue, so every point turns half a turn and every heading with it.
   */
  function rotate180(doc) {
    for (const w of doc.waypoints) {
      // || 0 so a point on an axis doesn't come back as -0
      w.xIn = -w.xIn || 0;
      w.yIn = -w.yIn || 0;
      if (w.headingDeg !== null) w.headingDeg = wrapHeading(w.headingDeg + 180);
    }
  }

  /**
   * The same routine from the alliance's other Quadrant: mirrored across the y = x diagonal, which
   * swaps left for bottom and top for right. The Goals, the tape, the Toggles and the objects along
   * the walls land on their twins; the Loaders don't, being on the side walls only.
   */
  function mirrorDiagonal(doc) {
    for (const w of doc.waypoints) {
      [w.xIn, w.yIn] = [w.yIn, w.xIn];
      // facing (sin h, cos h) becomes (cos h, sin h): 90 - h
      if (w.headingDeg !== null) w.headingDeg = wrapHeading(90 - w.headingDeg);
    }
  }

  /** Index to insert a point clicked near the path at: after the nearest chord's start. */
  function insertionIndex(doc, xIn, yIn) {
    let best = doc.waypoints.length;
    let bestDistance = Infinity;
    for (let j = 0; j + 1 < doc.waypoints.length; ++j) {
      const a = doc.waypoints[j];
      const b = doc.waypoints[j + 1];
      const dx = b.xIn - a.xIn;
      const dy = b.yIn - a.yIn;
      const lengthSq = dx * dx + dy * dy;
      const f = lengthSq > 0 ? Math.min(1, Math.max(0, ((xIn - a.xIn) * dx + (yIn - a.yIn) * dy) / lengthSq)) : 0;
      const d = Math.hypot(a.xIn + f * dx - xIn, a.yIn + f * dy - yIn);
      if (d < bestDistance) {
        bestDistance = d;
        best = j + 1;
      }
    }
    return best;
  }

  /** Set what waypoint j has `field`'s mechanism do: one of ACTIONS' values, or null for nothing. */
  function setAction(doc, j, field, value) {
    const w = doc.waypoints[j];
    if (!w) return;
    if (value) w[field] = value;
    else delete w[field];
  }

  /** Every action on the path, as robot::PathAction lists them: by point, then ACTIONS' order. */
  function actionsOf(doc) {
    const out = [];
    doc.waypoints.forEach((w, j) => {
      for (const action of ACTIONS) if (w[action.field] === action.value) out.push({ waypoint: j, action });
    });
    return out;
  }

  /** What waypoint w has the claw and the extender do, as short labels for the field. */
  function actionLabels(w) {
    return ACTIONS.filter((action) => w[action.field] === action.value).map((action) => action.label);
  }

  // --- C++ out --------------------------------------------------------------------------------

  function num(value) {
    const rounded = Math.round(value * 100) / 100;
    return String(Object.is(rounded, -0) ? 0 : rounded);
  }

  /** A C++ identifier from the path's name: "left rush" is kLeftRush. */
  function identifier(name) {
    const words = String(name || 'path').replace(/[^A-Za-z0-9]+/g, ' ').trim().split(/\s+/).filter(Boolean);
    const camel = words.map((w) => w.charAt(0).toUpperCase() + w.slice(1)).join('') || 'Path';
    return `k${/^[0-9]/.test(camel) ? `Path${camel}` : camel}`;
  }

  function waypointLiteral(w) {
    const heading = w.headingDeg !== undefined && w.headingDeg !== null ? `, .headingDeg = ${num(w.headingDeg)}` : '';
    return `{.xIn = ${num(w.xIn)}, .yIn = ${num(w.yIn)}${heading}}`;
  }

  /** The non-default constraints, in declaration order, or '' for all defaults. */
  function constraintsLiteral(c) {
    const defaults = T.trajectoryConstraints();
    const fields = [];
    if (c.voltageBudget !== defaults.voltageBudget) fields.push(`.voltageBudget = ${num(c.voltageBudget)}`);
    for (const [axis] of AXES) {
      const limits = c[axis];
      const parts = [];
      if (limits.maxVelocity > 0) parts.push(`.maxVelocity = ${num(limits.maxVelocity)}`);
      if (limits.maxAcceleration > 0) parts.push(`.maxAcceleration = ${num(limits.maxAcceleration)}`);
      if (parts.length) fields.push(`.${axis} = {${parts.join(', ')}}`);
    }
    return fields.length ? `{${fields.join(', ')}}` : '';
  }

  /**
   * The chain as C++ to paste into autons.cpp. `about` is the comment's
   * description of the plan, e.g. "3.42 s planned with the Robot tab's models".
   * A path with actions gets a second list after it, k<Name>Actions, for
   * robot::followPath().
   */
  function toCpp(doc, about = '') {
    const name = identifier(doc.name);
    const lines = [`// Sapphire Sim path "${doc.name}"${about ? `: ${about}` : ''}`,
      `const std::vector<sapphirelib::motion::TrajectorySection> ${name}{`];
    const sections = toSections(doc);
    sections.forEach((section) => {
      const open = '    {.waypoints = {';
      const pad = ' '.repeat(open.length);
      const points = section.waypoints.map(waypointLiteral);
      const body = [`${open}${points.join(`,\n${pad}`)}}`];
      const constraints = constraintsLiteral(section.constraints);
      if (constraints) body.push(`     .constraints = ${constraints}`);
      if (section.stopAtEnd) body.push('     .stopAtEnd = true');
      lines.push(`${body.join(',\n')}},`);
    });
    lines.push('};');
    const actions = actionsOf(doc);
    if (actions.length) {
      lines.push(`// what the claw and the extender do along "${doc.name}", by point (robot::followPath())`,
        `const std::vector<robot::PathAction> ${name}Actions{`,
        ...actions.map(({ waypoint, action }) => `    {.waypoint = ${waypoint}, .action = robot::PathAction::${action.cpp}},`),
        '};');
    }
    return lines.join('\n');
  }

  // --- C++ in ---------------------------------------------------------------------------------

  /**
   * Reads C++ that toCpp() wrote (or hand-edited the same way): a
   * std::vector of TrajectorySection, or of TrajectoryWaypoint for a single
   * section, and the k<Name>Actions list after it if there is one. Returns
   * { ok, error, doc }.
   */
  function parseCpp(text) {
    const source = String(text).replace(/\/\*[\s\S]*?\*\//g, ' ').replace(/\/\/[^\n]*/g, ' ');
    const nameMatch = source.match(/\bk([A-Za-z0-9_]+)\s*\{/);
    const start = nameMatch ? source.indexOf('{', nameMatch.index) : source.indexOf('{');
    if (start < 0) return { ok: false, error: 'no { ... } to read', doc: null };
    let i = start;
    // names (robot::PathAction::clawOpen) only in the actions list: in the sections, one is a mistake
    let allowNames = false;
    const fail = (message) => { throw new Error(`${message} (character ${i})`); };
    const skip = () => { while (i < source.length && /\s/.test(source[i])) ++i; };
    const value = () => {
      skip();
      if (source[i] === '{') {
        ++i;
        const items = [];
        const named = {};
        let anyNamed = false;
        skip();
        while (source[i] !== '}') {
          if (i >= source.length) fail('missing }');
          skip();
          if (source[i] === '.') {
            ++i;
            const m = /^[A-Za-z_]\w*/.exec(source.slice(i));
            if (!m) fail('expected a field name after .');
            i += m[0].length;
            skip();
            if (source[i] !== '=') fail(`expected = after .${m[0]}`);
            ++i;
            named[m[0]] = value();
            anyNamed = true;
          } else {
            items.push(value());
          }
          skip();
          if (source[i] === ',') ++i;
          skip();
        }
        ++i;
        if (anyNamed && items.length) fail('mixed named and unnamed values');
        return anyNamed ? named : items;
      }
      const m = /^(true|false|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[fF]?/.exec(source.slice(i));
      if (!m && allowNames) {
        const name = /^[A-Za-z_]\w*(?:\s*::\s*[A-Za-z_]\w*)*/.exec(source.slice(i));
        if (!name) fail('expected a number, a name, or {');
        i += name[0].length;
        // the last part: robot::PathAction::clawOpen is clawOpen
        return { name: name[0].split('::').pop().trim() };
      }
      if (!m) fail('expected a number, true, false, or {');
      i += m[0].length;
      if (m[1] === 'true') return true;
      if (m[1] === 'false') return false;
      return Number(m[1]);
    };
    try {
      const top = value();
      if (!Array.isArray(top) || top.length === 0) throw new Error('expected a list of sections');
      const isWaypoint = (o) => o && typeof o === 'object' && !Array.isArray(o) && 'xIn' in o;
      const sections = isWaypoint(top[0]) ? [{ waypoints: top }] : top;
      const chain = sections.map((s, k) => {
        if (!s || Array.isArray(s) || !Array.isArray(s.waypoints)) throw new Error(`section ${k} has no .waypoints`);
        for (const w of s.waypoints) {
          if (!isWaypoint(w) || typeof w.xIn !== 'number' || typeof w.yIn !== 'number') {
            throw new Error(`section ${k}: every waypoint needs .xIn and .yIn`);
          }
        }
        return { waypoints: s.waypoints, constraints: s.constraints || {}, stopAtEnd: s.stopAtEnd === true };
      });
      const name = nameMatch ? nameMatch[1].replace(/([a-z0-9])([A-Z])/g, '$1 $2').toLowerCase() : 'pasted';
      const doc = fromSections(chain, name);
      // what the claw and the extender do along it, if that was pasted too
      const actionsMatch = source.slice(i).match(/\bk[A-Za-z0-9_]*Actions\s*\{/);
      if (actionsMatch) {
        i += actionsMatch.index + actionsMatch[0].length - 1;
        allowNames = true;
        const list = value();
        if (!Array.isArray(list)) throw new Error('expected a list of actions');
        list.forEach((a, k) => {
          if (!a || Array.isArray(a) || typeof a !== 'object' || !a.action || !a.action.name) {
            throw new Error(`action ${k} needs .waypoint and .action`);
          }
          const action = ACTIONS.find((candidate) => candidate.cpp === a.action.name);
          if (!action) throw new Error(`action ${k}: there's no PathAction::${a.action.name}`);
          if (!Number.isInteger(a.waypoint) || a.waypoint < 0 || a.waypoint >= doc.waypoints.length) {
            throw new Error(`action ${k}: the path has no point ${a.waypoint}`);
          }
          doc.waypoints[a.waypoint][action.field] = action.value;
        });
      }
      return { ok: true, error: '', doc };
    } catch (error) {
      return { ok: false, error: error.message, doc: null };
    }
  }

  // --- The robot's logged models ---------------------------------------------------------------

  const kLoggedAxes = [['fwd', 'forward'], ['strafe', 'strafe'], ['turn', 'turn']];

  /**
   * What to plan with from the robot's logs: `logged` is slt.axisModels()'s
   * { fwd, strafe, turn } (each log's #meta model.* lines, and any Auto-Tune
   * since). { models: { forward, strafe, turn }, files, missing: [] } with
   * the files they came from, or { models: null, files, missing } naming the
   * axes no log had, since a plan needs all three.
   */
  function loggedPlanModels(logged) {
    const missing = kLoggedAxes.filter(([key]) => !logged[key]).map(([key]) => key);
    const files = [...new Set(kLoggedAxes.filter(([key]) => logged[key]).map(([key]) => logged[key].file))];
    if (missing.length) return { models: null, files, missing };
    const models = {};
    for (const [key, axis] of kLoggedAxes) models[axis] = { kS: logged[key].kS, kV: logged[key].kV, kA: logged[key].kA };
    return { models, files, missing };
  }

  // --- Against the field ------------------------------------------------------------------------

  /**
   * What the tab says about a path on Override's field: where it starts against SG1, and what a
   * widthIn × lengthIn footprint runs into along the plan (field.planHits()). { text, warn, hits },
   * with hits for drawing; text is '' for a path with no points.
   */
  function fieldCheckText(doc, plan, widthIn, lengthIn, field) {
    if (!doc.waypoints.length) return { text: '', warn: false, hits: [] };
    const first = doc.waypoints[0];
    const start = F.startCheck({ xIn: first.xIn, yIn: first.yIn, headingDeg: first.headingDeg || 0 }, widthIn, lengthIn, field);
    const parts = [];
    const problems = [];
    if (start.side) {
      parts.push(`Starts on ${start.side}'s side, in the ${start.quadrant} Quadrant.`);
    } else {
      problems.push('across the Autonomous Line');
    }
    if (!start.touchingWall) problems.push('off the perimeter');
    if (start.loadZone) problems.push(`in the ${start.loadZone}`);
    if (problems.length) parts.push(`SG1: it starts ${problems.join(', ')}.`);
    if (start.oversize) parts.push('SG1: it\'s bigger than 18 × 18 in.');
    let hits = [];
    if (plan && plan.ok) {
      hits = F.planHits(plan.trajectory.samples, widthIn, lengthIn, field);
      parts.push(hits.length ? `Runs into ${hits.map((h) => `the ${h.name} at ${h.tS.toFixed(2)} s`).join(', ')}.`
        : 'Clear of the Goals, Loaders and walls.');
    }
    return { text: parts.join(' '), warn: problems.length > 0 || start.oversize || hits.length > 0, hits };
  }

  // --- The tab (browser only) ------------------------------------------------------------------

  /**
   * Wires the Paths tab. `ctx` is the page's: { $, el, fmt, segmented,
   * setSegment, slt, options, renderer, runPath(routine), tuneModels(),
   * defaultChain } where tuneModels() is the axis models of the TUNE.CFG the
   * Tune tab has open, or null, slt is the analyzer's log reader, and
   * defaultChain is the path a first visit starts with.
   */
  function build(ctx) {
    const { $, el, fmt } = ctx;
    const state = {
      paths: [],
      current: 0,
      selected: -1,
      modelsSource: 'robot',
      // useLogs(): loggedPlanModels() of the robot's logs opened last, or null
      logged: null,
      plan: null,
      planDirty: true,
      // fieldCheckText() of the plan: what it says under it, and what the field draws in red
      fieldCheck: { text: '', warn: false, hits: [] },
      drag: null,
      active: false,
    };

    let fieldModel = null;
    function field() {
      const sizeIn = ctx.options.world.fieldSizeIn;
      if (!fieldModel || fieldModel.sizeIn !== sizeIn) fieldModel = F.override(sizeIn);
      return fieldModel;
    }

    /** The robot's footprint, from the last visit; the simulated robot takes it too. */
    function loadRobot() {
      try {
        const stored = JSON.parse(localStorage.getItem(kRobotStorageKey) || 'null');
        if (stored && stored.widthIn > 0 && stored.lengthIn > 0) {
          ctx.options.world.robotWidthIn = stored.widthIn;
          ctx.options.world.robotLengthIn = stored.lengthIn;
        }
      } catch (error) {
        // no storage: the default footprint
      }
      $('path-robot-width').value = String(ctx.options.world.robotWidthIn);
      $('path-robot-length').value = String(ctx.options.world.robotLengthIn);
    }

    function setRobot(key, value) {
      if (!(value >= 6 && value <= 24)) return;
      ctx.options.world[key] = value;
      try {
        localStorage.setItem(kRobotStorageKey, JSON.stringify({ widthIn: ctx.options.world.robotWidthIn, lengthIn: ctx.options.world.robotLengthIn }));
      } catch (error) {
        // it just won't be remembered
      }
      state.planDirty = true;
      replan();
    }

    function load() {
      try {
        const stored = JSON.parse(localStorage.getItem(kStorageKey) || 'null');
        if (stored && Array.isArray(stored.paths) && stored.paths.length) {
          state.paths = stored.paths;
          state.current = Math.min(stored.current || 0, stored.paths.length - 1);
          return;
        }
      } catch (error) {
        // no storage here (a private window, a file:// page that refuses it): start fresh
      }
      state.paths = [fromSections(ctx.defaultChain, 'tour')];
      state.current = 0;
    }

    function save() {
      try {
        localStorage.setItem(kStorageKey, JSON.stringify({ paths: state.paths, current: state.current }));
      } catch (error) {
        // drafts just don't outlive the page
      }
    }

    const doc = () => state.paths[state.current];

    const loggedModels = () => (state.logged && state.logged.models ? state.logged : null);

    function models() {
      if (state.modelsSource === 'tune') {
        const m = ctx.tuneModels();
        if (m) return m;
      }
      if (state.modelsSource === 'log' && loggedModels()) return loggedModels().models;
      return ctx.options.world.models;
    }

    function modelsLabel() {
      if (state.modelsSource === 'tune' && ctx.tuneModels()) return 'the open TUNE.CFG\'s models';
      if (state.modelsSource === 'log' && loggedModels()) return `the models ${loggedModels().files.join(' and ')} ran with`;
      return 'the Robot tab\'s models';
    }

    /**
     * The models the robot's logs say it ran, for "Logged models": from the
     * Tune tab's logs as well as this tab's own. `select` switches to them,
     * when the logs were opened here for that.
     */
    function logStatus(text, warn = false) {
      const box = $('path-log-status');
      box.hidden = false;
      box.className = warn ? 'result warn' : 'result';
      box.textContent = text;
    }

    function useLogs(logs, { select = false } = {}) {
      if (!logs.length) return;
      const found = loggedPlanModels(ctx.slt.axisModels(logs));
      if (!found.models) {
        const names = logs.map((log) => log.meta.file || log.fileName).join(', ');
        logStatus(found.files.length
          ? `No ${found.missing.join(' or ')} model in ${names}: a plan needs all three.`
          : `No axis models in ${names}: logs from before the robot logged them (model.* lines), or none measured.`, true);
        return;
      }
      state.logged = found;
      logStatus(`Logged models from ${found.files.join(' and ')}.`);
      if (select) {
        state.modelsSource = 'log';
        ctx.setSegment($('path-models-seg'), 'log');
      }
      if (state.modelsSource !== 'log') return;
      state.planDirty = true;
      if (state.active) renderAll();
    }

    async function openLogs(files) {
      const logs = [];
      const problems = [];
      for (const file of files) {
        try {
          logs.push(ctx.slt.parse(await file.text(), file.name));
        } catch (error) {
          problems.push(`${file.name}: ${error.message}`);
        }
      }
      if (problems.length) logStatus(problems.join(' '), true);
      useLogs(logs, { select: true });
    }

    function changed({ list = true } = {}) {
      state.planDirty = true;
      save();
      if (list) renderAll();
    }

    function replan() {
      if (!state.planDirty) return state.plan;
      state.planDirty = false;
      const d = doc();
      state.plan = d.waypoints.length >= 2 ? T.generateTrajectory(toSections(d), models())
        : { ok: false, error: 'click the field to add points (at least 2)', trajectory: new T.Trajectory() };
      const w = ctx.options.world;
      state.fieldCheck = fieldCheckText(d, state.plan, w.robotWidthIn, w.robotLengthIn, field());
      renderPlan();
      return state.plan;
    }

    // -- panels --

    function renderSelect() {
      const select = $('path-select');
      select.replaceChildren(...state.paths.map((p, i) => el('option', { value: String(i), text: p.name || `path ${i + 1}` })));
      select.value = String(state.current);
      $('path-name').value = doc().name;
    }

    function numberInput(value, step, onChange, attrs = {}) {
      const input = el('input', Object.assign({ type: 'number', step, value: String(value), class: 'num-input' }, attrs));
      input.addEventListener('change', () => {
        const v = Number(input.value);
        if (Number.isFinite(v)) onChange(v);
      });
      return input;
    }

    function renderPoints() {
      const d = doc();
      const table = $('path-points');
      const head = el('thead', {}, el('tr', {}, ['#', 'x', 'y', 'Heading', 'Sec', ''].map((t) => el('th', { text: t }))));
      const rows = d.waypoints.map((w, j) => {
        const section = sectionOf(d, j);
        const isBreak = d.breaks.includes(j);
        const hasHeading = w.headingDeg !== null;
        const check = el('input', { type: 'checkbox', title: j === 0 ? 'The first point is where the robot starts: it needs a heading' : 'Face a heading here' });
        check.checked = hasHeading;
        check.disabled = j === 0;
        check.addEventListener('change', () => {
          w.headingDeg = check.checked ? Math.round(plannedHeadingAt(j)) : null;
          changed();
        });
        const heading = numberInput(hasHeading ? w.headingDeg : '', 1, (v) => { w.headingDeg = v; changed(); },
          { disabled: !hasHeading, 'aria-label': `Waypoint ${j} heading` });
        const swatch = el('i', { class: 'sec-sw' });
        swatch.style.background = sectionColor(section);
        // a dot for a point where the claw or the extender does something (renderPointActions())
        const labels = actionLabels(w);
        const dot = labels.length ? el('i', { class: 'act-dot', title: labels.join(', ') }) : null;
        const actions = el('td', { class: 'actions' }, [
          j >= 1 && j <= d.waypoints.length - 2 && !isBreak
            ? el('button', { class: 'mini', type: 'button', text: 'Split', title: 'End a section here: the rest of the path gets its own limits',
              onclick: () => { splitAt(d, j); changed(); } }) : null,
          el('button', { class: 'mini', type: 'button', text: '✕', title: 'Remove this point', 'aria-label': `Remove waypoint ${j}`,
            onclick: () => { removeWaypoint(d, j); state.selected = -1; changed(); } }),
        ]);
        const row = el('tr', { class: j === state.selected ? 'selected' : '' }, [
          el('td', {}, [String(j), dot]),
          el('td', {}, numberInput(w.xIn, kSnapIn, (v) => { w.xIn = clampField(v); changed(); }, { 'aria-label': `Waypoint ${j} x` })),
          el('td', {}, numberInput(w.yIn, kSnapIn, (v) => { w.yIn = clampField(v); changed(); }, { 'aria-label': `Waypoint ${j} y` })),
          el('td', { class: 'heading-cell' }, [check, heading]),
          el('td', {}, [swatch, String(section)]),
          actions,
        ]);
        row.addEventListener('click', (event) => {
          if (event.target.closest('input, button, select')) return;
          state.selected = j;
          renderPoints();
        });
        return row;
      });
      table.replaceChildren(head, el('tbody', {}, rows));
      renderPointActions();
    }

    /** Under the table: what the claw and the extender do as the robot reaches the selected point. */
    function renderPointActions() {
      const d = doc();
      const j = state.selected;
      const w = d.waypoints[j];
      const box = $('path-point-actions');
      if (!w) {
        box.replaceChildren(el('span', { class: 'hint', text: 'Select a point to have the claw or the extender do something as the robot reaches it.' }));
        return;
      }
      box.replaceChildren(el('strong', { text: `Point ${j}` }), ...MECHANISMS.map(([field, label]) => {
        const select = el('select', {
          class: w[field] ? 'mech-select set' : 'mech-select',
          title: `What the ${field} does as the robot reaches point ${j} (robot::followPath())`,
          'aria-label': `Point ${j} ${field}`,
        }, [el('option', { value: '', text: '—' }),
          ...ACTIONS.filter((a) => a.field === field).map((a) => el('option', { value: a.value, text: a.value }))]);
        select.value = w[field] || '';
        select.addEventListener('change', () => {
          setAction(d, j, field, select.value || null);
          changed();
        });
        return el('label', { class: 'inline' }, [label, select]);
      }));
    }

    function renderSections() {
      const d = doc();
      const box = $('path-sections');
      box.replaceChildren(...d.sections.map((section, k) => {
        const c = section.constraints;
        const first = k === 0 ? 0 : d.breaks[k - 1];
        const last = k < d.breaks.length ? d.breaks[k] : d.waypoints.length - 1;
        const swatch = el('i', { class: 'sec-sw' });
        swatch.style.background = sectionColor(k);
        const grid = el('div', { class: 'limit-grid' }, [
          el('span'), el('span', { class: 'h', text: 'top speed' }), el('span', { class: 'h', text: 'top accel' }),
          ...AXES.flatMap(([axis, label, unit]) => [
            el('span', { text: `${label} (${unit})` }),
            numberInput(c[axis].maxVelocity, 1, (v) => { c[axis].maxVelocity = Math.max(0, v); changed(); },
              { min: 0, title: `${unit}/s. 0: only the voltage budget limits it`, 'aria-label': `Section ${k} ${label} top speed` }),
            numberInput(c[axis].maxAcceleration, 10, (v) => { c[axis].maxAcceleration = Math.max(0, v); changed(); },
              { min: 0, title: `${unit}/s². 0: only the voltage budget limits it`, 'aria-label': `Section ${k} ${label} top acceleration` }),
          ]),
        ]);
        const budget = numberInput(c.voltageBudget, 0.5, (v) => { c.voltageBudget = Math.min(12, Math.max(0, v)); changed(); },
          { min: 0, max: 12, 'aria-label': `Section ${k} voltage budget` });
        const isLast = k === d.sections.length - 1;
        const stop = el('input', { type: 'checkbox' });
        stop.checked = isLast || section.stopAtEnd;
        stop.disabled = isLast;
        stop.addEventListener('change', () => { section.stopAtEnd = stop.checked; changed(); });
        return el('div', { class: 'section-card' }, [
          el('header', {}, [swatch, el('strong', { text: `Section ${k}` }), el('span', { class: 'hint', text: `points ${first}–${last}` }),
            !isLast ? el('button', { class: 'mini', type: 'button', text: 'Join next', title: 'Merge the next section into this one',
              onclick: () => { mergeSections(d, k); changed(); } }) : null]),
          el('label', { class: 'inline', title: 'The most any wheel may be planned to need; the rest of 12V is headroom' },
            ['Voltage budget (V)', budget]),
          grid,
          el('label', { class: 'inline', title: isLast ? 'The last section always stops' : 'Come to rest at this section\'s last point: the path may turn sharply there' },
            [stop, 'Stop at its end']),
        ]);
      }));
    }

    function renderPlan() {
      const plan = state.plan;
      const status = $('path-status');
      const summary = $('path-summary');
      summary.replaceChildren();
      $('path-models-hint').textContent = `planned with ${modelsLabel()}`;
      const check = $('path-field');
      check.hidden = !state.fieldCheck.text;
      check.className = state.fieldCheck.warn ? 'result warn' : 'result';
      check.textContent = state.fieldCheck.text;
      if (!plan || !plan.ok) {
        status.textContent = plan ? plan.error : '';
        status.className = 'result warn';
        status.hidden = !plan;
        drawPlanChart(null);
        $('path-code').textContent = toCpp(doc(), 'not plannable yet');
        return;
      }
      const t = plan.trajectory;
      status.hidden = false;
      status.className = 'result';
      status.textContent = `${fmt(t.durationS(), 2)} s over ${fmt(t.lengthIn(), 0)} in.`;
      const row = (label, text) => summary.append(el('span', { text: label }), el('span', { text }));
      if (t.sectionCount() > 1) {
        row('Sections start', Array.from({ length: t.sectionCount() }, (_, k) => `${k}: ${fmt(t.sectionStart(k).tS, 2)} s`).join('  '));
      }
      const peak = (key) => Math.max(...t.samples.map((s) => Math.abs(s[key])));
      row('Top speed', `${fmt(peak('speedInPerS'), 1)} in/s`);
      row('Peaks', `fwd ${fmt(peak('forwardInPerS'), 0)}  strafe ${fmt(peak('strafeInPerS'), 0)} in/s  turn ${fmt(peak('omegaDegPerS'), 0)}°/s`);
      drawPlanChart(t);
      $('path-code').textContent = toCpp(doc(), `${fmt(t.durationS(), 2)} s planned with ${modelsLabel()}`);
    }

    function drawPlanChart(trajectory) {
      const canvas = $('path-chart');
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);
      const g = canvas.getContext('2d');
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      g.clearRect(0, 0, rect.width, rect.height);
      if (!trajectory || trajectory.empty() || rect.width === 0) return;
      const colors = ctx.renderer.colors;
      const left = 34;
      const right = rect.width - 30;
      const top = 18;
      const bottom = rect.height - 16;
      const duration = trajectory.durationS();
      const m = models();
      const volts = trajectory.samples.map((s) => {
        const v = T.trajectoryFeedforward(m, s);
        return Math.abs(v.forward) + Math.abs(v.strafe) + Math.abs(v.turn);
      });
      const vMax = Math.max(10, ...trajectory.samples.map((s) => s.speedInPerS)) * 1.1;
      const xOf = (t) => left + (t / duration) * (right - left);
      const ySpeed = (v) => bottom - (v / vMax) * (bottom - top);
      const yVolts = (v) => bottom - (v / 12) * (bottom - top);
      g.font = '11px ui-monospace, Consolas, monospace';
      g.fillStyle = colors.muted;
      g.strokeStyle = colors.grid;
      g.lineWidth = 1;
      for (let k = 1; k < trajectory.sectionCount(); ++k) {
        const x = xOf(trajectory.sectionStart(k).tS);
        g.beginPath();
        g.moveTo(x, top);
        g.lineTo(x, bottom);
        g.stroke();
      }
      g.beginPath();
      g.moveTo(left, bottom);
      g.lineTo(right, bottom);
      g.stroke();
      g.fillText('0', 6, bottom + 4);
      g.fillText(`${Math.round(vMax)}`, 6, top + 4);
      g.textAlign = 'right';
      g.fillText('12V', rect.width - 2, top + 4);
      g.fillText(`${fmt(duration, 1)}s`, right, rect.height - 2);
      g.textAlign = 'left';
      const line = (values, color, y, dash) => {
        g.strokeStyle = color;
        g.lineWidth = 2;
        g.setLineDash(dash);
        g.beginPath();
        trajectory.samples.forEach((s, i) => {
          const px = xOf(s.tS);
          const py = y(values[i]);
          if (i === 0) g.moveTo(px, py);
          else g.lineTo(px, py);
        });
        g.stroke();
        g.setLineDash([]);
      };
      line(trajectory.samples.map((s) => s.speedInPerS), colors.accent, ySpeed, []);
      line(volts, colors['series-2'], yVolts, [4, 3]);
      g.font = '11px system-ui, sans-serif';
      g.fillStyle = colors.accent;
      g.fillText('speed (in/s)', left + 4, 11);
      g.fillStyle = colors['series-2'];
      g.fillText('worst wheel (V)', left + 90, 11);
    }

    function renderAll() {
      renderSelect();
      renderPoints();
      renderSections();
      state.planDirty = true;
      replan();
    }

    // -- headings and colors --

    function plannedHeadingAt(j) {
      // the nearest keyframe at or before it: where the robot faces there until told otherwise
      const d = doc();
      for (let i = j; i >= 0; --i) if (d.waypoints[i].headingDeg !== null) return d.waypoints[i].headingDeg;
      return 0;
    }

    function sectionColor(k) {
      return `var(--${SECTION_COLORS[k % SECTION_COLORS.length]})`;
    }

    function clampField(v) {
      return Math.max(-kFieldHalfIn, Math.min(kFieldHalfIn, v));
    }

    const snap = (v) => Math.round(v / kSnapIn) * kSnapIn;

    // -- the field --

    const kHandlePx = 22;

    function hit(px, py) {
      const d = doc();
      const r = ctx.renderer;
      for (let j = d.waypoints.length - 1; j >= 0; --j) {
        const w = d.waypoints[j];
        if (w.headingDeg === null) continue;
        const [x, y] = r.toPx(w.xIn, w.yIn);
        const a = w.headingDeg * Math.PI / 180;
        if (Math.hypot(px - (x + Math.sin(a) * kHandlePx), py - (y - Math.cos(a) * kHandlePx)) <= 10) return { kind: 'heading', j };
      }
      for (let j = d.waypoints.length - 1; j >= 0; --j) {
        const [x, y] = r.toPx(d.waypoints[j].xIn, d.waypoints[j].yIn);
        if (Math.hypot(px - x, py - y) <= 8) return { kind: 'point', j };
      }
      return null;
    }

    function nearPath(px, py) {
      const plan = state.plan;
      if (!plan || !plan.ok) return false;
      return plan.trajectory.samples.some((s) => {
        const [x, y] = ctx.renderer.toPx(s.xIn, s.yIn);
        return Math.hypot(px - x, py - y) <= 6;
      });
    }

    function pointerDown(px, py, event) {
      const d = doc();
      const h = hit(px, py);
      if (event.button === 2 && h && h.kind === 'point') {
        removeWaypoint(d, h.j);
        state.selected = -1;
        changed();
        return true;
      }
      if (h) {
        state.drag = h;
        state.selected = h.j;
        renderPoints();
        return true;
      }
      const p = ctx.renderer.toField(px, py);
      const point = { xIn: clampField(snap(p.xIn)), yIn: clampField(snap(p.yIn)), headingDeg: null };
      const index = nearPath(px, py) ? insertionIndex(d, p.xIn, p.yIn) : d.waypoints.length;
      insertWaypoint(d, index, point);
      state.selected = index;
      state.drag = { kind: 'point', j: index };
      changed();
      return true;
    }

    function pointerMove(px, py, event) {
      if (!state.drag) {
        const h = hit(px, py);
        return h ? (h.kind === 'heading' ? 'alias' : 'grab') : 'copy';
      }
      const w = doc().waypoints[state.drag.j];
      if (!w) return '';
      const p = ctx.renderer.toField(px, py);
      if (state.drag.kind === 'point') {
        w.xIn = clampField(snap(p.xIn));
        w.yIn = clampField(snap(p.yIn));
      } else {
        let heading = Math.atan2(p.xIn - w.xIn, p.yIn - w.yIn) * 180 / Math.PI;
        heading = event.shiftKey ? Math.round(heading / 15) * 15 : Math.round(heading);
        w.headingDeg = (heading + 360) % 360;
      }
      // the list catches up when the drag ends; the plan every frame
      changed({ list: false });
      return 'grabbing';
    }

    function pointerUp() {
      if (!state.drag) return;
      state.drag = null;
      renderAll();
    }

    function deleteSelected() {
      if (state.selected < 0) return false;
      removeWaypoint(doc(), state.selected);
      state.selected = -1;
      changed();
      return true;
    }

    /** What render.js draws over the field while the tab is open. */
    function editorView() {
      replan();
      return {
        doc: doc(), plan: state.plan, selected: state.selected, handlePx: kHandlePx, sectionOf: (j) => sectionOf(doc(), j),
        hits: state.fieldCheck.hits,
      };
    }

    // -- the buttons --

    $('path-select').addEventListener('change', (event) => {
      state.current = Number(event.target.value);
      state.selected = -1;
      changed();
    });
    $('path-name').addEventListener('change', (event) => {
      doc().name = event.target.value.trim() || 'path';
      changed();
    });
    $('path-new-btn').addEventListener('click', () => {
      state.paths.push(emptyDoc(`path ${state.paths.length + 1}`));
      state.current = state.paths.length - 1;
      state.selected = -1;
      changed();
    });
    $('path-delete-btn').addEventListener('click', () => {
      state.paths.splice(state.current, 1);
      if (state.paths.length === 0) state.paths.push(emptyDoc('path 1'));
      state.current = Math.min(state.current, state.paths.length - 1);
      state.selected = -1;
      changed();
    });
    $('path-reverse-btn').addEventListener('click', () => {
      reverse(doc());
      state.selected = -1;
      changed();
    });
    $('path-rotate-btn').addEventListener('click', () => {
      rotate180(doc());
      changed();
    });
    $('path-mirror-btn').addEventListener('click', () => {
      mirrorDiagonal(doc());
      changed();
    });
    for (const [id, key] of [['path-robot-width', 'robotWidthIn'], ['path-robot-length', 'robotLengthIn']]) {
      $(id).addEventListener('change', (event) => setRobot(key, Number(event.target.value)));
    }
    $('path-run-btn').addEventListener('click', () => {
      const plan = replan();
      if (!plan || !plan.ok) return;
      const d = doc();
      const first = d.waypoints[0];
      const planned = models();
      const sections = toSections(d);
      ctx.runPath({
        name: `Path: ${d.name}`,
        start: { xIn: first.xIn, yIn: first.yIn, headingDeg: first.headingDeg },
        *run(robot) {
          robot.odometry.setPose(this.start);
          // planned and followed with the same models, as the robot plans with its TUNE.CFG's
          robot.drivetrain.setAxisModels(planned);
          const result = T.generateTrajectory(sections, planned);
          if (!result.ok) return;
          yield* robot.drivetrain.followTrajectory(result.trajectory);
        },
      });
    });
    ctx.segmented($('path-models-seg'), (value) => {
      state.modelsSource = value;
      if (value === 'tune' && !ctx.tuneModels()) {
        flash('No TUNE.CFG with axis models is open: open one in the Tune tab. Planning with the Robot tab\'s models.');
      }
      if (value === 'log' && !loggedModels()) {
        flash('No log with the robot\'s axis models is open: open an SL*.CSV below, or in the Tune tab. ' +
          'Planning with the Robot tab\'s models.');
      }
      changed();
    });
    $('path-log-files').addEventListener('change', (event) => {
      openLogs([...event.target.files]);
      event.target.value = '';
    });
    $('path-log-btn').addEventListener('click', () => $('path-log-files').click());
    const logDrop = $('path-log-drop');
    logDrop.addEventListener('dragover', (event) => {
      event.preventDefault();
      logDrop.classList.add('over');
    });
    logDrop.addEventListener('dragleave', () => logDrop.classList.remove('over'));
    logDrop.addEventListener('drop', (event) => {
      event.preventDefault();
      logDrop.classList.remove('over');
      openLogs([...event.dataTransfer.files]);
    });
    $('path-copy-btn').addEventListener('click', async () => {
      const text = $('path-code').textContent;
      try {
        await navigator.clipboard.writeText(text);
        flash('Copied. Paste it into src/robot/autons.cpp.');
      } catch (error) {
        // no clipboard (a file:// page in some browsers): select it for Ctrl+C instead
        const range = document.createRange();
        range.selectNodeContents($('path-code'));
        const selection = window.getSelection();
        selection.removeAllRanges();
        selection.addRange(range);
        flash('Selected: press Ctrl+C (⌘C) to copy.');
      }
    });
    $('path-paste-btn').addEventListener('click', () => {
      $('path-paste').hidden = false;
      $('path-paste-row').hidden = false;
      $('path-paste').focus();
    });
    $('path-paste-cancel').addEventListener('click', () => {
      $('path-paste').hidden = true;
      $('path-paste-row').hidden = true;
    });
    $('path-read-btn').addEventListener('click', () => {
      const parsed = parseCpp($('path-paste').value);
      if (!parsed.ok) {
        flash(`Couldn't read it: ${parsed.error}.`, true);
        return;
      }
      state.paths.push(parsed.doc);
      state.current = state.paths.length - 1;
      state.selected = -1;
      $('path-paste').value = '';
      $('path-paste').hidden = true;
      $('path-paste-row').hidden = true;
      flash(`Read "${parsed.doc.name}": ${parsed.doc.waypoints.length} points in ${parsed.doc.sections.length} section(s).`);
      changed();
    });

    let flashTimer = null;
    function flash(text, warn = false) {
      const box = $('path-code-status');
      box.hidden = false;
      box.className = warn ? 'result warn' : 'result';
      box.textContent = text;
      clearTimeout(flashTimer);
      flashTimer = setTimeout(() => { box.hidden = true; }, 6000);
    }

    loadRobot();
    load();
    renderAll();

    return {
      setActive(active) {
        state.active = active;
        if (active) {
          state.planDirty = true;
          renderAll();
        }
      },
      isActive: () => state.active,
      modelsChanged() {
        state.planDirty = true;
      },
      useLogs,
      /** How the last run ended, under the Run button. */
      showRunResult(text) {
        const box = $('path-run-result');
        box.hidden = false;
        box.textContent = text;
      },
      editorView,
      pointerDown,
      pointerMove,
      pointerUp,
      deleteSelected,
    };
  }

  return {
    SECTION_COLORS,
    ACTIONS,
    emptyDoc,
    fromSections,
    toSections,
    sectionOf,
    insertWaypoint,
    addWaypoint,
    removeWaypoint,
    splitAt,
    mergeSections,
    reverse,
    rotate180,
    mirrorDiagonal,
    fieldCheckText,
    insertionIndex,
    setAction,
    actionsOf,
    actionLabels,
    identifier,
    toCpp,
    parseCpp,
    loggedPlanModels,
    build,
  };
});
