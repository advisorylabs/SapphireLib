// Tests for js/pathview.js: the Paths tab's path document, and its C++ both
// ways. Run: node --test tools/sim/test/*.test.js
//
// What Copy C++ writes is what src/robot/autons.cpp compiles, so the
// snippet's exact text is pinned here, and reading it back must give the same
// chain.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const P = require('../js/pathview.js');
const T = require('../js/trajectory.js');
const F = require('../js/field.js');
const { TOUR_CHAIN } = require('../js/routines.js');

test('a chain survives the document and back', () => {
  const doc = P.fromSections(TOUR_CHAIN, 'tour');
  assert.equal(doc.waypoints.length, 8);
  assert.deepEqual(doc.breaks, [2, 4]);
  assert.equal(doc.sections.length, 3);
  const back = P.toSections(doc);
  assert.equal(back.length, 3);
  back.forEach((section, k) => {
    assert.deepEqual(section.waypoints, TOUR_CHAIN[k].waypoints);
    assert.deepEqual(section.constraints, T.trajectoryConstraints(TOUR_CHAIN[k].constraints));
    assert.equal(section.stopAtEnd, !!TOUR_CHAIN[k].stopAtEnd);
  });
  // a waypoint at a junction belongs to the section it ends
  assert.equal(P.sectionOf(doc, 2), 0);
  assert.equal(P.sectionOf(doc, 3), 1);
  assert.equal(P.sectionOf(doc, 7), 2);
});

test('Logged models: all three axes from the robot\'s logs, or which are missing', () => {
  const fwd = { kS: 0.3, kV: 0.1698, kA: 0.0383, delayS: null, from: 'logged', file: 'SL000010.CSV' };
  const strafe = { kS: 2.21, kV: 0.3234, kA: 0.0838, delayS: null, from: 'logged', file: 'SL000010.CSV' };
  const turn = { kS: 0.9, kV: 0.031, kA: 0.002, delayS: 0.045, from: 'autotune', file: 'SL000011.CSV' };
  const found = P.loggedPlanModels({ fwd, strafe, turn });
  assert.deepEqual(found.models, {
    forward: { kS: 0.3, kV: 0.1698, kA: 0.0383 },
    strafe: { kS: 2.21, kV: 0.3234, kA: 0.0838 },
    turn: { kS: 0.9, kV: 0.031, kA: 0.002 },
  });
  assert.deepEqual(found.files, ['SL000010.CSV', 'SL000011.CSV']);
  assert.deepEqual(found.missing, []);
  // and they plan
  assert.ok(T.generateTrajectory(TOUR_CHAIN, found.models).ok);

  const partial = P.loggedPlanModels({ fwd });
  assert.equal(partial.models, null);
  assert.deepEqual(partial.missing, ['strafe', 'turn']);
  assert.deepEqual(partial.files, ['SL000010.CSV']);
  assert.deepEqual(P.loggedPlanModels({}).files, []);
});

test('editing keeps the sections in step with the points', () => {
  const doc = P.emptyDoc('edit');
  P.addWaypoint(doc, { xIn: 0, yIn: 0 });
  assert.equal(doc.waypoints[0].headingDeg, 0, 'the start always gets a heading');
  for (const y of [10, 20, 30, 40]) P.addWaypoint(doc, { xIn: 0, yIn: y });
  assert.ok(P.splitAt(doc, 2));
  assert.ok(!P.splitAt(doc, 2), 'already a junction');
  assert.ok(!P.splitAt(doc, 0) && !P.splitAt(doc, 4), 'not at either end');
  doc.sections[0].constraints.voltageBudget = 6;
  assert.deepEqual(doc.breaks, [2]);
  assert.equal(doc.sections[1].constraints.voltageBudget, 10, 'the new section copies the old limits');

  // inserting before the junction moves it along
  P.insertWaypoint(doc, 1, { xIn: 5, yIn: 5 });
  assert.deepEqual(doc.breaks, [3]);
  // removing the junction joins the sections, keeping the first's limits
  doc.sections[1].stopAtEnd = false;
  P.removeWaypoint(doc, 3);
  assert.deepEqual(doc.breaks, []);
  assert.equal(doc.sections.length, 1);
  assert.equal(doc.sections[0].constraints.voltageBudget, 6);

  // a junction left at the very end by a deletion is dropped
  P.splitAt(doc, 3);
  P.removeWaypoint(doc, 4);
  assert.deepEqual(doc.breaks, []);
  assert.equal(doc.sections.length, 1);

  // deleting the start hands the heading on
  doc.waypoints[1].headingDeg = null;
  P.removeWaypoint(doc, 0);
  assert.equal(doc.waypoints[0].headingDeg, 0);

  // a click near the second chord inserts there, not at the end
  const line = P.emptyDoc();
  for (const [x, y] of [[0, 0], [0, 20], [20, 20]]) P.addWaypoint(line, { xIn: x, yIn: y });
  assert.equal(P.insertionIndex(line, 10, 21), 2);
  assert.equal(P.insertionIndex(line, 1, 10), 1);
});

test('reversing drives the same chain backward, and twice is where it started', () => {
  const doc = P.fromSections(TOUR_CHAIN, 'tour');
  P.reverse(doc);
  assert.deepEqual(doc.waypoints[0], { xIn: -48, yIn: -60, headingDeg: 0 });
  assert.deepEqual(doc.breaks, [3, 5]);
  // the stop was at the end of the old middle section; reversed, the middle still ends there
  assert.equal(doc.sections[1].stopAtEnd, false);
  assert.equal(doc.sections[0].stopAtEnd, true);
  assert.equal(doc.sections[1].constraints.voltageBudget, 7);
  P.reverse(doc);
  assert.deepEqual(P.toSections(doc), P.toSections(P.fromSections(TOUR_CHAIN, 'tour')));
});

test('the other alliance\'s routine is a half turn, the other Quadrant\'s a mirror, and twice is where it started', () => {
  const doc = P.fromSections(TOUR_CHAIN, 'tour');
  const original = P.toSections(P.fromSections(TOUR_CHAIN, 'tour'));
  P.rotate180(doc);
  assert.deepEqual(doc.waypoints[0], { xIn: 48, yIn: 60, headingDeg: 180 });
  assert.deepEqual(doc.waypoints[1], { xIn: 50, yIn: 24, headingDeg: null });
  // a point on an axis stays a plain 0, so the C++ never says -0
  assert.ok(Object.is(doc.waypoints[3].xIn, 0));
  P.rotate180(doc);
  assert.deepEqual(P.toSections(doc), original);

  P.mirrorDiagonal(doc);
  // (-48, -60) facing +y becomes (-60, -48) facing +x; facing +x becomes facing +y
  assert.deepEqual(doc.waypoints[0], { xIn: -60, yIn: -48, headingDeg: 90 });
  assert.equal(doc.waypoints[2].headingDeg, 0);
  assert.equal(doc.waypoints[4].headingDeg, 270);
  P.mirrorDiagonal(doc);
  assert.deepEqual(P.toSections(doc), original);
});

test('the field check finds the same hits, red for blue, on the other alliance\'s routine', () => {
  const field = F.override();
  const doc = P.fromSections(TOUR_CHAIN, 'tour');
  const models = { forward: { kS: 1.0, kV: 0.2, kA: 0.045 }, strafe: { kS: 1.3, kV: 0.24, kA: 0.055 }, turn: { kS: 0.6, kV: 0.045, kA: 0.0065 } };
  const check = P.fieldCheckText(doc, T.generateTrajectory(P.toSections(doc), models), 15, 15, field);
  // the demo chain predates Override: it starts off the wall and cuts through three Goals
  assert.ok(check.warn);
  assert.match(check.text, /^Starts on red's side, in the bottom Quadrant\. SG1: it starts off the perimeter\./);
  assert.deepEqual(check.hits.map((h) => h.name), ['red goal (left)', 'center goal', 'red goal (bottom)']);

  P.rotate180(doc);
  const turned = P.fieldCheckText(doc, T.generateTrajectory(P.toSections(doc), models), 15, 15, field);
  assert.match(turned.text, /^Starts on blue's side, in the top Quadrant\./);
  assert.deepEqual(turned.hits.map((h) => h.name), ['blue goal (right)', 'center goal', 'blue goal (top)']);
  turned.hits.forEach((h, i) => assert.ok(Math.abs(h.tS - check.hits[i].tS) < 1e-6));

  assert.equal(P.fieldCheckText(P.emptyDoc(), null, 15, 15, field).text, '');
});

const SMALL = [
  { waypoints: [{ xIn: -48, yIn: -60, headingDeg: 0 }, { xIn: -40, yIn: -30.25 }] },
  { waypoints: [{ xIn: -24, yIn: -12, headingDeg: 45 }],
    constraints: { voltageBudget: 7, strafe: { maxVelocity: 30 }, turn: { maxAcceleration: 400 } },
    stopAtEnd: true },
  { waypoints: [{ xIn: 0, yIn: 0 }] },
];

test('Copy C++ writes what autons.cpp compiles', () => {
  const text = P.toCpp(P.fromSections(SMALL, 'left rush'), '3.42 s planned with the Robot tab\'s models');
  assert.equal(text, [
    '// Sapphire Sim path "left rush": 3.42 s planned with the Robot tab\'s models',
    'const std::vector<sapphirelib::motion::TrajectorySection> kLeftRush{',
    '    {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0},',
    '                   {.xIn = -40, .yIn = -30.25}}},',
    '    {.waypoints = {{.xIn = -24, .yIn = -12, .headingDeg = 45}},',
    '     .constraints = {.voltageBudget = 7, .strafe = {.maxVelocity = 30}, .turn = {.maxAcceleration = 400}},',
    '     .stopAtEnd = true},',
    '    {.waypoints = {{.xIn = 0, .yIn = 0}}},',
    '};',
  ].join('\n'));
  assert.equal(P.identifier('2nd try!'), 'kPath2ndTry');
});

test('Paste C++ reads it back, comments and all', () => {
  const doc = P.fromSections(SMALL, 'left rush');
  const parsed = P.parseCpp(`/* from the sim */\n${P.toCpp(doc)}\n// trailing comment`);
  assert.ok(parsed.ok, parsed.error);
  assert.equal(parsed.doc.name, 'left rush');
  assert.deepEqual(P.toSections(parsed.doc), P.toSections(doc));

  // the plan's own example, written by hand
  const hand = P.parseCpp(`const std::vector<sapphirelib::motion::TrajectorySection> kRush{
      {.waypoints = {{.xIn = -48, .yIn = -60, .headingDeg = 0}, {.xIn = -40, .yIn = -30}},
       .constraints = {.voltageBudget = 10.0}},
      {.waypoints = {{.xIn = -24, .yIn = -12, .headingDeg = 45}},
       .constraints = {.voltageBudget = 7.0}}, // slower through the intake
  };`);
  assert.ok(hand.ok, hand.error);
  assert.equal(hand.doc.waypoints.length, 3);
  assert.equal(hand.doc.sections[1].constraints.voltageBudget, 7);

  // one section, as a list of waypoints
  const single = P.parseCpp('const std::vector<sapphirelib::motion::TrajectoryWaypoint> kLine{{.xIn = 0, .yIn = 0, .headingDeg = 90}, {.xIn = 24, .yIn = 0}};');
  assert.ok(single.ok, single.error);
  assert.equal(single.doc.sections.length, 1);
  assert.equal(single.doc.waypoints[0].headingDeg, 90);

  assert.ok(!P.parseCpp('nothing here').ok);
  assert.match(P.parseCpp('kX{{.waypoints = {{.yIn = 1}}}};').error, /needs \.xIn and \.yIn/);
  assert.match(P.parseCpp('kX{{.waypoints = {{.xIn = 1, .yIn = 2}}').error, /missing \}/);
  // a name is only for the actions: in the sections it's still a mistake
  assert.match(P.parseCpp('kX{{.waypoints = {{.xIn = kA, .yIn = 2}}}};').error, /expected a number, true, false, or \{/);
});

test('a point\'s claw and extender actions go out as robot::PathAction, and come back', () => {
  const doc = P.fromSections(SMALL, 'left rush');
  P.setAction(doc, 1, 'extender', 'out');
  P.setAction(doc, 2, 'claw', 'open');
  P.setAction(doc, 2, 'extender', 'in');
  P.setAction(doc, 3, 'claw', 'close');
  assert.deepEqual(P.actionLabels(doc.waypoints[2]), ['claw open', 'ext in']);
  // the sections the planner gets don't change
  assert.deepEqual(P.toSections(doc), P.toSections(P.fromSections(SMALL, 'left rush')));
  const text = P.toCpp(doc, '3.42 s planned with the Robot tab\'s models');
  assert.equal(text.split('\n').slice(9).join('\n'), [
    '// what the claw and the extender do along "left rush", by point (robot::followPath())',
    'const std::vector<robot::PathAction> kLeftRushActions{',
    '    {.waypoint = 1, .action = robot::PathAction::extenderOut},',
    '    {.waypoint = 2, .action = robot::PathAction::clawOpen},',
    '    {.waypoint = 2, .action = robot::PathAction::extenderIn},',
    '    {.waypoint = 3, .action = robot::PathAction::clawClose},',
    '};',
  ].join('\n'));

  // Paste C++ puts them back on their points
  const parsed = P.parseCpp(text);
  assert.ok(parsed.ok, parsed.error);
  assert.deepEqual(parsed.doc.waypoints, doc.waypoints);
  assert.equal(P.toCpp(parsed.doc, '3.42 s planned with the Robot tab\'s models'), text);
  // ...written by hand, with the namespace left off
  const hand = P.parseCpp(`${P.toCpp(P.fromSections(SMALL, 'left rush'))}
    const std::vector<PathAction> kLeftRushActions{{.waypoint = 0, .action = PathAction::extenderOut}};`);
  assert.ok(hand.ok, hand.error);
  assert.equal(hand.doc.waypoints[0].extender, 'out');
  // an action that isn't one, or on a point the path doesn't have
  const sections = P.toCpp(P.fromSections(SMALL, 'left rush'));
  assert.match(P.parseCpp(`${sections} kLeftRushActions{{.waypoint = 0, .action = PathAction::intakeOn}};`).error,
    /no PathAction::intakeOn/);
  assert.match(P.parseCpp(`${sections} kLeftRushActions{{.waypoint = 9, .action = PathAction::clawOpen}};`).error,
    /no point 9/);

  // clearing one takes it off; with none left, no list at all
  for (const j of [1, 2, 3]) {
    P.setAction(doc, j, 'claw', null);
    P.setAction(doc, j, 'extender', null);
  }
  assert.deepEqual(P.actionsOf(doc), []);
  assert.ok(!P.toCpp(doc).includes('PathAction'));
});

test('actions stay on their points through editing and reversing', () => {
  const doc = P.fromSections(SMALL, 'left rush');
  P.setAction(doc, 2, 'claw', 'open');
  // a point inserted before it moves it along, and removing another doesn't take it
  P.insertWaypoint(doc, 1, { xIn: -44, yIn: -45 });
  assert.equal(doc.waypoints[3].claw, 'open');
  P.removeWaypoint(doc, 1);
  assert.equal(doc.waypoints[2].claw, 'open');
  // driven backward, it's still where it was on the field
  P.reverse(doc);
  const at = doc.waypoints.findIndex((w) => w.claw === 'open');
  assert.deepEqual([doc.waypoints[at].xIn, doc.waypoints[at].yIn], [-24, -12]);
  assert.deepEqual(P.actionsOf(doc).map(({ waypoint, action }) => [waypoint, action.cpp]), [[at, 'clawOpen']]);
});

test('ACTIONS matches robot::PathAction, in its order', () => {
  // include/robot/paths.hpp: followPath() and the C++ that Copy C++ writes both go by these names
  const header = require('node:fs').readFileSync(require('node:path').join(__dirname, '../../../include/robot/paths.hpp'), 'utf8');
  const kinds = header.match(/enum Kind[^{]*\{([^}]*)\}/)[1].replace(/\/\/[^\n]*/g, '').split(',').map((s) => s.trim()).filter(Boolean);
  assert.deepEqual(P.ACTIONS.map((a) => a.cpp), kinds);
});
