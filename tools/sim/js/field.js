/*
 * SapphireLib simulator: field.js
 *
 * The VEX V5 Robotics Competition field for Override (2026-27), to scale, so
 * routines can be planned on it: the tile seams, the tape lines and zones,
 * the nine Goals, the four Loaders and four Toggles, and where the Cups and
 * Pins start a Head-to-Head match. And what the Paths tab checks with it:
 * whether the robot's footprint hits a Goal, a Loader or a wall anywhere
 * along a plan, and where a routine starts (which alliance's side of the
 * Autonomous Line, which Quadrant).
 *
 * Every number is from the game manual's Appendix A drawings (v2.1: Field
 * Element Locations, Tape Line Specifications, Scoring Object Locations and
 * Goal Specifications), moved into odom::Pose's frame: origin in the middle,
 * +x right, +y downfield (away from the audience), headings clockwise from
 * +y. Seen the way the manual draws it, from the audience: red's Alliance
 * Station along the left wall, blue's along the right. The Goals and the
 * objects sit on tile corners, so they're measured from the middle; the
 * Loaders and the objects along the walls are measured from the walls, so
 * they follow the field size given.
 *
 * Plain script: window.SIM.field in a browser, require('./field.js') in Node.
 *
 * Team 96671H: Hitmen
 */
(function (factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    const root = typeof self !== 'undefined' ? self : this;
    root.SIM = root.SIM || {};
    root.SIM.field = factory();
  }
})(function () {
  'use strict';

  const kDegToRad = Math.PI / 180;

  // Tile seam to tile seam. The manual's seams are 23.11, 46.66, 70.20, 93.75 and 117.30 in from
  // the left wall of a 140.41 in field: the edge tiles lose a little under the walls.
  const kTilePitchIn = 23.545;
  const T = kTilePitchIn;
  // How far the tiles' puzzle teeth reach either side of a seam (the load zone tape sits past them).
  const kTeethIn = 0.45;
  // White and alliance tape. The Autonomous Line is two of them with a 1 in gap, 2.50 in across,
  // and the Midfield's outer edge is 34.05 in along: both say 3/4 in tape.
  const kTapeIn = 0.75;
  const kAutoLineGapIn = 1.0;

  // Goals: a chamfered square 5.61 in across the flats, 3.22 in to a corner (Goal Specifications).
  const kGoalAcrossIn = 5.61;
  const kGoalCornerIn = 3.22;
  // Loaders reach 3.74 in in from the wall and are 4.02 in wide, centered 11.44 in from the end walls.
  const kLoaderDepthIn = 3.74;
  const kLoaderWidthIn = 4.02;
  const kLoaderFromEndWallIn = 11.44;
  // Toggles: a bar 25.99 in long on top of the middle of each wall.
  const kToggleLengthIn = 25.99;

  // Cups 3.15 in across, Pins 1.6 in across and 6.5 in long (Appendix A).
  const kCupIn = 3.15;
  const kPinIn = 1.6;
  const kPinLengthIn = 6.5;
  // The groups along the walls: a Pin between two Cups, 3.16 in apart, 1.58 in off the wall.
  const kWallGroupSpacingIn = 3.16;
  const kWallGroupFromWallIn = 1.58;
  // The four crosses on the Autonomous Line: Pins lying with their inner ends 1.6 in from the Cup's
  // middle.
  const kCrossInnerIn = 1.6;

  /** The goal's outline, corners in order around it. */
  function goalOutline(xIn, yIn) {
    const h = kGoalAcrossIn / 2;
    const c = Math.sqrt(kGoalCornerIn * kGoalCornerIn - h * h);
    return [[h, c], [c, h], [-c, h], [-h, c], [-h, -c], [-c, -h], [c, -h], [h, -c]]
      .map(([x, y]) => [xIn + x, yIn + y]);
  }

  function boxOutline(minXIn, minYIn, maxXIn, maxYIn) {
    return [[minXIn, minYIn], [maxXIn, minYIn], [maxXIn, maxYIn], [minXIn, maxYIn]];
  }

  /**
   * Which of the four triangular Quadrants a point is in: the two diagonals split the field, and
   * each Quadrant is named for its wall. Red owns left and bottom, blue top and right.
   */
  function quadrantOf(xIn, yIn) {
    if (Math.abs(xIn) >= Math.abs(yIn)) return xIn < 0 ? 'left' : 'right';
    return yIn < 0 ? 'bottom' : 'top';
  }

  /** Whose side of the Autonomous Line (the diagonal from the far left corner to the near right one). */
  function sideOf(xIn, yIn) {
    return xIn + yIn < 0 ? 'red' : 'blue';
  }

  /**
   * The whole field for a wall-to-wall size (140.5 in, the simulator's, by default):
   * { sizeIn, halfIn, tilePitchIn, seams, tape, zones, goals, loaders, toggles, objects }. Tape is
   * { color, widthIn, points, closed } center lines; zones are { name, alliance, outline } (the
   * Load Zones and the Midfield); goals and loaders carry their outline; objects are { kind: 'cup' |
   * 'pin' | 'lyingPin', color, xIn, yIn } with a lying Pin's headingDeg pointing at its colored end.
   */
  function override(sizeIn = 140.5) {
    const half = sizeIn / 2;
    const seams = [-2, -1, 0, 1, 2].map((k) => k * T);

    // -- tape --
    const tape = [];
    const white = (points, closed = false) => tape.push({ color: 'white', widthIn: kTapeIn, points, closed });
    // the Midfield: a square standing on a corner, its tape centered on the four tile corners
    white([[-T, 0], [0, T], [T, 0], [0, -T]], true);
    // each Load Zone's tape: its outside edge on the middle of the corner tile, then along the seam
    // just past the teeth
    const zoneEdge = 2.5 * T;
    const zoneSeam = 2 * T - kTeethIn - kTapeIn;
    const zones = [];
    for (const [alliance, sx] of [['red', -1], ['blue', 1]]) {
      for (const sy of [1, -1]) {
        const x = sx * (zoneEdge + kTapeIn / 2);
        const y = sy * (zoneSeam + kTapeIn / 2);
        tape.push({ color: alliance, widthIn: kTapeIn, points: [[x, sy * half], [x, y], [sx * half, y]], closed: false });
        zones.push({
          name: `${alliance} load zone (${sy > 0 ? 'far' : 'near'})`,
          alliance,
          outline: boxOutline(Math.min(sx * zoneEdge, sx * half), Math.min(sy * zoneSeam, sy * half),
            Math.max(sx * zoneEdge, sx * half), Math.max(sy * zoneSeam, sy * half)),
        });
      }
    }
    zones.push({ name: 'Midfield', alliance: null, outline: [[-T, 0], [0, T], [T, 0], [0, -T]] });
    // the y = x diagonal, one line from each Load Zone to the Midfield
    white([[-zoneEdge, -zoneEdge], [-T / 2, -T / 2]]);
    white([[T / 2, T / 2], [zoneEdge, zoneEdge]]);
    // the Autonomous Line along the other diagonal: two lines, the gap between them on y = -x
    for (const offset of [-1, 1]) {
      const d = (offset * (kAutoLineGapIn + kTapeIn)) / 2;
      const r = Math.SQRT2 * d;
      white([[-zoneEdge, zoneEdge + r], [(r - T) / 2, (r + T) / 2]]);
      white([[(r + T) / 2, (r - T) / 2], [zoneEdge, -zoneEdge + r]]);
    }

    // -- goals: the tall one in the middle, then each Quadrant's pair --
    const goal = (kind, xIn, yIn, heightIn) => ({
      kind, xIn, yIn, heightIn,
      name: kind === 'center' ? 'center goal' : `${kind} goal (${quadrantOf(xIn, yIn)})`,
      outline: goalOutline(xIn, yIn),
    });
    const goals = [
      goal('center', 0, 0, 8.77),
      goal('neutral', -T, 2 * T, 5.77),
      goal('neutral', -2 * T, T, 5.77),
      goal('neutral', 2 * T, -T, 5.77),
      goal('neutral', T, -2 * T, 5.77),
      goal('red', -2 * T, -T, 3.25),
      goal('red', -T, -2 * T, 3.25),
      goal('blue', T, 2 * T, 3.25),
      goal('blue', 2 * T, T, 3.25),
    ];

    // -- loaders: on red's wall and blue's, near each end --
    const loaders = [];
    for (const [alliance, sx] of [['red', -1], ['blue', 1]]) {
      for (const sy of [1, -1]) {
        const yIn = sy * (half - kLoaderFromEndWallIn);
        const inner = sx * (half - kLoaderDepthIn);
        loaders.push({
          alliance, xIn: sx * (half - kLoaderDepthIn / 2), yIn,
          name: `${alliance} loader (${sy > 0 ? 'far' : 'near'})`,
          outline: boxOutline(Math.min(inner, sx * half), yIn - kLoaderWidthIn / 2, Math.max(inner, sx * half), yIn + kLoaderWidthIn / 2),
        });
      }
    }

    // -- toggles: the middle of every wall, each one its Quadrant's --
    const toggles = [[0, half, 0], [0, -half, 0], [-half, 0, 90], [half, 0, 90]].map(([xIn, yIn, angleDeg]) => ({
      xIn, yIn, angleDeg, lengthIn: kToggleLengthIn, quadrant: quadrantOf(xIn, yIn),
    }));

    // -- the Cups and Pins a Head-to-Head match starts with --
    const objects = [];
    const cup = (xIn, yIn) => objects.push({ kind: 'cup', color: null, xIn, yIn });
    const pin = (color, xIn, yIn) => objects.push({ kind: 'pin', color, xIn, yIn });
    // a Pin standing in every neutral Goal
    for (const g of goals) if (g.kind === 'center' || g.kind === 'neutral') pin('yellow', g.xIn, g.yIn);
    // a Cup holding a red or blue Pin at each corner of the Midfield, colored for the side it's on
    for (const [xIn, yIn] of [[-T, 0], [0, -T], [0, T], [T, 0]]) {
      cup(xIn, yIn);
      pin(sideOf(xIn, yIn), xIn, yIn);
    }
    // four crosses on the Autonomous Line: a Cup, and four Pins lying around it, each one's yellow
    // half inward and its alliance half pointing at that alliance's walls
    for (const k of [-2, -1, 1, 2]) {
      const xIn = k * T;
      const yIn = -k * T;
      cup(xIn, yIn);
      for (const [headingDeg, color] of [[0, 'blue'], [90, 'blue'], [180, 'red'], [270, 'red']]) {
        objects.push({ kind: 'lyingPin', color, xIn, yIn, headingDeg, innerIn: kCrossInnerIn });
      }
    }
    // yellow Pins standing on the other diagonal's tile corners
    for (const k of [-2, -1, 1, 2]) pin('yellow', k * T, k * T);
    // along every wall, twice: Cup, yellow Pin, Cup
    const fromWall = half - kWallGroupFromWallIn;
    for (const along of [-T, T]) {
      for (const [ax, ay, nx, ny] of [[1, 0, 0, 1], [1, 0, 0, -1], [0, 1, -1, 0], [0, 1, 1, 0]]) {
        for (const step of [-1, 0, 1]) {
          const xIn = ax * (along + step * kWallGroupSpacingIn) + nx * fromWall;
          const yIn = ay * (along + step * kWallGroupSpacingIn) + ny * fromWall;
          if (step === 0) pin('yellow', xIn, yIn);
          else cup(xIn, yIn);
        }
      }
    }

    return {
      name: 'VEX V5 Override (2026-27)',
      sizeIn, halfIn: half, tilePitchIn: T, seams, tape, zones, goals, loaders, toggles, objects,
      cupIn: kCupIn, pinIn: kPinIn, pinLengthIn: kPinLengthIn,
    };
  }

  /**
   * The Goals and Loaders as boxes, for the simulated world's collisions and the localizer's map
   * (World and FieldMap only know boxes). A Goal's box is its flats, so its chamfers are filled in.
   */
  function obstacleBoxes(field) {
    const box = (name, outline) => ({
      name,
      minXIn: Math.min(...outline.map((p) => p[0])), minYIn: Math.min(...outline.map((p) => p[1])),
      maxXIn: Math.max(...outline.map((p) => p[0])), maxYIn: Math.max(...outline.map((p) => p[1])),
    });
    return [...field.goals, ...field.loaders].map((e) => box(e.name, e.outline));
  }

  // -- the robot's footprint --

  /** The robot's four corners at a pose: widthIn across, lengthIn front to back, centered on the pose. */
  function footprint(pose, widthIn, lengthIn) {
    const a = pose.headingDeg * kDegToRad;
    const forward = [Math.sin(a), Math.cos(a)];
    const right = [Math.cos(a), -Math.sin(a)];
    return [[1, 1], [1, -1], [-1, -1], [-1, 1]].map(([r, f]) => [
      pose.xIn + (r * widthIn * right[0] + f * lengthIn * forward[0]) / 2,
      pose.yIn + (r * widthIn * right[1] + f * lengthIn * forward[1]) / 2,
    ]);
  }

  /** Whether two convex outlines overlap (separating axes); touching isn't overlapping. */
  function overlaps(a, b) {
    for (const outline of [a, b]) {
      for (let i = 0; i < outline.length; ++i) {
        const p = outline[i];
        const q = outline[(i + 1) % outline.length];
        const nx = q[1] - p[1];
        const ny = p[0] - q[0];
        let aMin = Infinity;
        let aMax = -Infinity;
        let bMin = Infinity;
        let bMax = -Infinity;
        for (const v of a) {
          const d = v[0] * nx + v[1] * ny;
          aMin = Math.min(aMin, d);
          aMax = Math.max(aMax, d);
        }
        for (const v of b) {
          const d = v[0] * nx + v[1] * ny;
          bMin = Math.min(bMin, d);
          bMax = Math.max(bMax, d);
        }
        // a hair of slack, in the axis's units, so a footprint flush against something isn't a hit
        const slack = 1e-6 * Math.hypot(nx, ny);
        if (aMax <= bMin + slack || bMax <= aMin + slack) return false;
      }
    }
    return true;
  }

  /**
   * What a footprint (footprint()'s corners) hits: the names of the Goals and Loaders it overlaps,
   * and 'wall' if a corner is past one. A robot flush against a wall is fine.
   */
  function footprintHits(corners, field) {
    const hits = [];
    const limit = field.halfIn + 0.05;
    if (corners.some(([x, y]) => Math.abs(x) > limit || Math.abs(y) > limit)) hits.push('wall');
    for (const e of [...field.goals, ...field.loaders]) if (overlaps(corners, e.outline)) hits.push(e.name);
    return hits;
  }

  /**
   * Everything a plan's footprint runs into: [{ name, tS, pose }], the first moment it hits each
   * one, in the order it gets there. `samples` are trajectory samples ({ tS, xIn, yIn, headingDeg }).
   */
  function planHits(samples, widthIn, lengthIn, field) {
    const first = new Map();
    for (const s of samples) {
      for (const name of footprintHits(footprint(s, widthIn, lengthIn), field)) {
        if (!first.has(name)) first.set(name, { name, tS: s.tS, pose: { xIn: s.xIn, yIn: s.yIn, headingDeg: s.headingDeg } });
      }
    }
    return [...first.values()].sort((a, b) => a.tS - b.tS);
  }

  /**
   * Where a routine starts, against SG1 (at most 18 × 18, touching the tiles and the perimeter on
   * its alliance's side of the Autonomous Line, clear of the Load Zones, one robot per Quadrant):
   * { side: 'red' | 'blue' | null (across the line), quadrant, touchingWall, loadZone (the name of
   * one it's in, or null), oversize }. Goals and Loaders are planHits()'s, at 0 s.
   */
  function startCheck(pose, widthIn, lengthIn, field) {
    const corners = footprint(pose, widthIn, lengthIn);
    const sides = new Set(corners.map(([x, y]) => sideOf(x, y)));
    const touchingWall = corners.some(([x, y]) => Math.max(Math.abs(x), Math.abs(y)) >= field.halfIn - 0.5);
    const zone = field.zones.find((z) => z.alliance && overlaps(corners, z.outline));
    return {
      side: sides.size === 1 ? [...sides][0] : null,
      quadrant: quadrantOf(pose.xIn, pose.yIn),
      touchingWall,
      loadZone: zone ? zone.name : null,
      oversize: widthIn > 18 || lengthIn > 18,
    };
  }

  function inside(outline, xIn, yIn) {
    let sign = 0;
    for (let i = 0; i < outline.length; ++i) {
      const p = outline[i];
      const q = outline[(i + 1) % outline.length];
      const cross = (q[0] - p[0]) * (yIn - p[1]) - (q[1] - p[1]) * (xIn - p[0]);
      if (cross === 0) continue;
      if (sign === 0) sign = Math.sign(cross);
      else if (Math.sign(cross) !== sign) return false;
    }
    return true;
  }

  /** What's at a point, in a few words, for the field's cursor readout. */
  function describe(field, xIn, yIn) {
    if (Math.abs(xIn) > field.halfIn || Math.abs(yIn) > field.halfIn) return '';
    for (const e of [...field.goals, ...field.loaders]) if (inside(e.outline, xIn, yIn)) return e.name;
    for (const z of field.zones) if (inside(z.outline, xIn, yIn)) return z.name;
    return `${quadrantOf(xIn, yIn)} quadrant, ${sideOf(xIn, yIn)} side`;
  }

  return {
    kTilePitchIn,
    override,
    obstacleBoxes,
    quadrantOf,
    sideOf,
    footprint,
    overlaps,
    footprintHits,
    planHits,
    startCheck,
    describe,
  };
});
