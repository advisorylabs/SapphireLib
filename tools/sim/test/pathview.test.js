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
});
