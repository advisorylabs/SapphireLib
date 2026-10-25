// Tests for js/field.js: Override's field, and the footprint checks the Paths
// tab makes with it. Run: node --test tools/sim/test/*.test.js
//
// The positions are pinned to the game manual's Appendix A drawings (v2.1),
// which measure from the bottom left corner of a 140.41 in field: a route
// planned against a misplaced Goal is a route that hits the real one.
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const F = require('../js/field.js');

const field = F.override();
// the manual's frame: inches from the left wall and the near (audience) wall
const kManualHalfIn = 140.41 / 2;
const manual = (v) => v + kManualHalfIn;
const near = (a, b, tolerance = 0.03) => Math.abs(a - b) <= tolerance;

test('the Goals, Loaders and Toggles are where the manual draws them', () => {
  const at = (kind, x, y) => field.goals.some((g) => g.kind === kind && near(manual(g.xIn), x) && near(manual(g.yIn), y));
  assert.equal(field.goals.length, 9);
  assert.ok(at('center', 70.2, 70.2));
  for (const [x, y] of [[46.66, 117.3], [23.11, 93.75], [117.3, 46.66], [93.75, 23.11]]) assert.ok(at('neutral', x, y), `neutral ${x},${y}`);
  for (const [x, y] of [[23.11, 46.66], [46.66, 23.11]]) assert.ok(at('red', x, y), `red ${x},${y}`);
  for (const [x, y] of [[93.75, 117.3], [117.3, 93.75]]) assert.ok(at('blue', x, y), `blue ${x},${y}`);
  // the center Goal is the tall one
  assert.equal(field.goals.find((g) => g.kind === 'center').heightIn, 8.77);

  // red's two on the left wall, blue's on the right, 11.44 in from the end walls, 3.74 in deep
  const ofManual = F.override(140.41);
  assert.equal(ofManual.loaders.length, 4);
  for (const loader of ofManual.loaders) {
    const xs = loader.outline.map((p) => manual(p[0]));
    assert.ok(loader.alliance === 'red' ? near(Math.max(...xs), 3.74) : near(Math.min(...xs), 140.41 - 3.74), loader.name);
    assert.ok(near(manual(loader.yIn), 11.44) || near(manual(loader.yIn), 128.97), loader.name);
  }
  assert.deepEqual(field.toggles.map((t) => t.quadrant).sort(), ['bottom', 'left', 'right', 'top']);
});

test('a match starts with 24 Cups and 37 Pins on the field, 16 of them lying in the crosses', () => {
  const count = (kind) => field.objects.filter((o) => o.kind === kind).length;
  assert.equal(count('cup'), 24);
  assert.equal(count('pin') + count('lyingPin'), 37);
  assert.equal(count('lyingPin'), 16);
  // a yellow Pin in every neutral Goal, the alliance Goals empty
  for (const g of field.goals) {
    const pinned = field.objects.some((o) => o.kind === 'pin' && o.xIn === g.xIn && o.yIn === g.yIn);
    assert.equal(pinned, g.kind === 'neutral' || g.kind === 'center', g.name);
  }
});

test('the field turns into itself under a half turn, red for blue', () => {
  const swap = { red: 'blue', blue: 'red' };
  for (const g of field.goals) {
    const twin = field.goals.find((o) => near(o.xIn, -g.xIn, 1e-9) && near(o.yIn, -g.yIn, 1e-9));
    assert.ok(twin, g.name);
    assert.equal(twin.kind, swap[g.kind] || g.kind);
  }
  for (const l of field.loaders) {
    assert.ok(field.loaders.some((o) => o.alliance === swap[l.alliance] && near(o.yIn, -l.yIn, 1e-9) && near(o.xIn, -l.xIn, 1e-9)), l.name);
  }
  for (const o of field.objects) {
    const twin = field.objects.find((t) => t.kind === o.kind && near(t.xIn, -o.xIn, 1e-9) && near(t.yIn, -o.yIn, 1e-9) &&
      t.color === (swap[o.color] || o.color) && (o.kind !== 'lyingPin' || (t.headingDeg - o.headingDeg + 360) % 360 === 180));
    assert.ok(twin, `${o.kind} at ${o.xIn},${o.yIn}`);
  }
});

test('a footprint hits a Goal it overlaps, not one it only touches, and a wall only past it', () => {
  const g = field.goals.find((o) => o.name === 'red goal (left)');
  const across = 5.61 / 2 + 15 / 2;
  // straight below the Goal, the robot's front edge flush with the Goal's bottom flat
  const flush = F.footprint({ xIn: g.xIn, yIn: g.yIn - across, headingDeg: 0 }, 15, 15);
  assert.deepEqual(F.footprintHits(flush, field), []);
  const into = F.footprint({ xIn: g.xIn, yIn: g.yIn - across + 0.1, headingDeg: 0 }, 15, 15);
  assert.deepEqual(F.footprintHits(into, field), ['red goal (left)']);
  // turned 45°, its corner reaches farther
  const turned = F.footprint({ xIn: g.xIn, yIn: g.yIn - across - 2, headingDeg: 45 }, 15, 15);
  assert.deepEqual(F.footprintHits(turned, field), ['red goal (left)']);
  // a robot against the near wall is fine; an inch past it isn't
  const half = field.halfIn;
  assert.deepEqual(F.footprintHits(F.footprint({ xIn: 0, yIn: -half + 7.5, headingDeg: 0 }, 15, 15), field), []);
  assert.deepEqual(F.footprintHits(F.footprint({ xIn: 0, yIn: -half + 6.5, headingDeg: 0 }, 15, 15), field), ['wall']);
  // width is across: an 18 × 12 robot facing +x is 18 in tall on the field
  const wide = F.footprint({ xIn: 0, yIn: 0, headingDeg: 90 }, 18, 12);
  assert.ok(near(Math.max(...wide.map((p) => p[1])), 9, 1e-9));
  assert.ok(near(Math.max(...wide.map((p) => p[0])), 6, 1e-9));
});

test('a plan reports the first moment it hits each thing, in order', () => {
  // straight up the middle from the near wall: the center Goal, then on into the far wall
  const samples = [];
  for (let i = 0; i <= 140; ++i) samples.push({ tS: i / 10, xIn: 0, yIn: -62 + i, headingDeg: 0 });
  const hits = F.planHits(samples, 15, 15, field);
  assert.deepEqual(hits.map((h) => h.name), ['center goal', 'wall']);
  assert.ok(hits[0].tS < hits[1].tS);
  assert.ok(near(hits[0].pose.yIn, -10.3, 0.6), `first touches the goal at y ${hits[0].pose.yIn}`);
});

test('a start is checked against SG1', () => {
  // against red's wall, in the left Quadrant, clear of the Load Zones
  assert.deepEqual(F.startCheck({ xIn: -62.75, yIn: -35, headingDeg: 90 }, 15, 15, field),
    { side: 'red', quadrant: 'left', touchingWall: true, loadZone: null, oversize: false });
  // in the near red Load Zone
  assert.equal(F.startCheck({ xIn: -62.75, yIn: -55, headingDeg: 90 }, 15, 15, field).loadZone, 'red load zone (near)');
  // across the Autonomous Line, off the wall
  const across = F.startCheck({ xIn: -30, yIn: 30, headingDeg: 0 }, 15, 15, field);
  assert.equal(across.side, null);
  assert.equal(across.touchingWall, false);
  assert.equal(F.startCheck({ xIn: 60, yIn: 0, headingDeg: 0 }, 18.5, 15, field).oversize, true);
});

test('the cursor readout names what is under it', () => {
  assert.equal(F.describe(field, 0, 0), 'center goal');
  assert.equal(F.describe(field, 10, 0), 'Midfield');
  assert.equal(F.describe(field, -65, 60), 'red load zone (far)');
  assert.equal(F.describe(field, -69, -58.8), 'red loader (near)');
  assert.equal(F.describe(field, -40, 5), 'left quadrant, red side');
  assert.equal(F.describe(field, 80, 0), '');
});

test('the world gets the Goals and Loaders as boxes', () => {
  const boxes = F.obstacleBoxes(field);
  assert.equal(boxes.length, 13);
  const center = boxes.find((b) => b.name === 'center goal');
  assert.ok(near(center.maxXIn - center.minXIn, 5.61, 1e-9));
  assert.ok(boxes.every((b) => b.minXIn < b.maxXIn && b.minYIn < b.maxYIn));
});
