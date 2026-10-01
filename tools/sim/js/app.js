/*
 * SapphireLib simulator: app.js
 *
 * The page: the frame loop (real time × speed, in 10ms simulation ticks),
 * keyboard driving, the controls in each tab, the help card, and the
 * readouts. The simulation itself is sim.js; this only drives it and shows
 * it.
 *
 * Browser only: window.SIM.app.
 *
 * Team 96671H: Hitmen
 */
(function () {
  'use strict';

  const root = typeof self !== 'undefined' ? self : this;
  const SIM = root.SIM;
  const L = SIM.mcl;
  const W = SIM.world;
  const R = SIM.robot;
  const S = SIM.sim;
  const M = root.SA.model;
  const TF = root.SA.tunefile;
  const { ROUTINES } = SIM.routines;

  const kTickMs = S.kTickMs;
  const kMaxTicksPerFrame = 400;

  let options;
  let sim;
  let renderer;
  let playing = true;
  let speed = 1;
  let accumulatorMs = 0;
  let lastFrameMs = null;
  let lastDomMs = 0;
  let kidnapArmed = false;
  let draggingDefender = false;
  let selectedRoutine = ROUTINES[0];
  const keys = new Set();

  // what the field draws; the toggles in the Show tab
  const view = {
    particles: true, weightColor: true, weightSize: true, ellipse: true,
    beams: true, expected: true,
    raw: true, corrected: true, estimate: true, correctionArrow: true, trails: true, target: true,
    mounts: true, grid: true,
    zoom: 1,
  };

  const $ = (id) => document.getElementById(id);

  function el(tag, attrs = {}, children = []) {
    const node = document.createElement(tag);
    for (const [key, value] of Object.entries(attrs)) {
      if (key === 'text') node.textContent = value;
      else if (key === 'html') node.innerHTML = value;
      else if (key.startsWith('on')) node.addEventListener(key.slice(2), value);
      else if (value === true) node.setAttribute(key, '');
      else if (value !== false && value != null) node.setAttribute(key, value);
    }
    for (const child of [].concat(children)) {
      if (child == null) continue;
      node.append(child.nodeType ? child : document.createTextNode(child));
    }
    return node;
  }

  function fmt(value, digits = 1) {
    return Number.isFinite(value) ? value.toFixed(digits) : '–';
  }

  // --- setup ---------------------------------------------------------------------------

  function start() {
    options = S.defaultOptions();
    // the example elements always exist; the Events tab puts them in the world and the map
    options.world.elements = allElements(false);
    renderer = new SIM.render.FieldRenderer($('field'));
    buildTabs();
    buildTransport();
    buildDrive();
    buildShow();
    buildEvents();
    buildRobot();
    buildMcl();
    buildTune();
    buildLegend();
    buildHelp();
    bindField();
    bindKeyboard();

    new ResizeObserver(() => renderer.resize()).observe($('field-wrap'));
    renderer.resize();

    // open on something moving, like the analyzer's demo
    sim = new S.Simulation(options);
    runRoutine(ROUTINES[0]);
    updateGainsReadout();
    updateMclCode();
    scheduleFrame();
  }

  function buildTabs() {
    const tabs = $('tabs');
    tabs.addEventListener('click', (event) => {
      const button = event.target.closest('button[data-tab]');
      if (!button) return;
      for (const b of tabs.querySelectorAll('button')) b.setAttribute('aria-selected', String(b === button));
      for (const body of document.querySelectorAll('.tab-body')) body.hidden = body.dataset.body !== button.dataset.tab;
    });
  }

  function buildTransport() {
    $('play-btn').addEventListener('click', () => setPlaying(!playing));
    $('reset-btn').addEventListener('click', () => sim.reset(selectedRoutine.start));
    $('speed-select').addEventListener('change', (event) => { speed = +event.target.value; });
    $('theme-btn').addEventListener('click', () => {
      const docEl = document.documentElement;
      const dark = docEl.dataset.theme ? docEl.dataset.theme === 'dark'
        : window.matchMedia('(prefers-color-scheme: dark)').matches;
      docEl.dataset.theme = dark ? 'light' : 'dark';
      renderer.refreshColors();
    });
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => renderer.refreshColors());
  }

  function setPlaying(value) {
    playing = value;
    $('play-btn').textContent = playing ? 'Pause' : 'Play';
  }

  function buildHelp() {
    $('help-btn').addEventListener('click', () => toggleHelp());
  }

  function toggleHelp(open = $('help').hidden) {
    $('help').hidden = !open;
    $('help-btn').setAttribute('aria-expanded', String(open));
  }

  // --- Drive tab ---------------------------------------------------------------------------

  function buildDrive() {
    const select = $('routine-select');
    for (const routine of ROUTINES) select.append(el('option', { value: routine.id, text: routine.name }));
    select.addEventListener('change', () => {
      selectedRoutine = ROUTINES.find((r) => r.id === select.value);
      $('routine-desc').textContent = selectedRoutine.description;
    });
    $('routine-desc').textContent = selectedRoutine.description;
    $('run-btn').addEventListener('click', () => runRoutine(selectedRoutine));
    $('stop-btn').addEventListener('click', () => sim.stopProgram());
    $('stick').addEventListener('input', (event) => { $('stick-out').textContent = `${event.target.value}%`; });

    segmented($('correct-seg'), (value) => {
      options.correctOdometry = value === 'on';
      sim.robot.localizer.setCorrectionEnabled(options.correctOdometry);
    });
    segmented($('gains-seg'), (value) => {
      options.useAutoTune = value === 'auto';
      sim.applyGains();
      updateGainsReadout();
    });
  }

  function segmented(container, onChange) {
    container.addEventListener('click', (event) => {
      const button = event.target.closest('button[data-value]');
      if (!button) return;
      for (const b of container.querySelectorAll('button')) b.setAttribute('aria-pressed', String(b === button));
      onChange(button.dataset.value);
    });
  }

  function setSegment(container, value) {
    for (const b of container.querySelectorAll('button')) b.setAttribute('aria-pressed', String(b.dataset.value === value));
  }

  function runRoutine(routine) {
    selectedRoutine = routine;
    $('routine-select').value = routine.id;
    $('routine-desc').textContent = routine.description;
    sim.runRoutine(routine);
    $('routine-result').hidden = true;
    if (!playing) setPlaying(true);
  }

  function updateGainsReadout() {
    const g = sim.gains;
    const box = $('gains-readout');
    box.replaceChildren();
    const row = (label, text) => box.append(el('span', { text: label }), el('span', { text }));
    row('Drive', `kP ${fmt(g.drive.kP, 3)}  kD ${fmt(g.drive.kD, 4)}  (V/in)`);
    row('Turn', `kP ${fmt(g.turn.kP, 3)}  kD ${fmt(g.turn.kD, 5)}  (V/°)`);
    if (options.useAutoTune && sim.designs) {
      const d = sim.designs;
      row('Settle', `${fmt(d.drive.settleTimeS, 2)} s / ${fmt(d.turn.settleTimeS, 2)} s`);
      row('Margin', `${fmt(d.drive.phaseMarginDeg, 0)}° / ${fmt(d.turn.phaseMarginDeg, 0)}°`);
    }
  }

  // --- Show tab ---------------------------------------------------------------------------

  // [view key, label, tooltip, swatch]
  const SHOW = {
    'show-particles': [
      ['particles', 'Particles', 'Every guess at where the robot is', 'accent'],
      ['weightColor', 'Color by fit', 'Amber explains the last readings best, grey worst', 'warning'],
      ['weightSize', 'Size by weight', 'Bigger counts more in the estimate', null],
      ['ellipse', 'Uncertainty ellipse (2σ)', 'Where the cloud puts the robot', 'accent'],
    ],
    'show-sensors': [
      ['beams', 'Sensor beams', 'What each sensor really sees, from the real robot', 'beam'],
      ['expected', 'Expected vs measured', 'From the estimate: dashed to where the map says the wall is, a tick at ' +
        'the reading. Green agrees (within 3σ), red doesn\'t', 'good'],
    ],
    'show-poses': [
      ['raw', 'Odometry alone', 'Tracking wheels and IMU only', 'raw'],
      ['corrected', 'Corrected pose', 'What the motions drive by: odometry plus the correction', 'corrected'],
      ['estimate', 'MCL estimate (×)', 'The particles\' weighted mean', 'accent'],
      ['correctionArrow', 'Correction', 'From odometry alone to the corrected pose', 'corrected'],
      ['trails', 'Trails', 'Real (grey), odometry alone, corrected', null],
      ['target', 'Motion target', 'The running motion\'s point, pose, or path', 'series-4'],
    ],
    'show-field': [
      ['mounts', 'Tracking wheels and sensors', 'Drawn on the real robot', 'series-7'],
      ['grid', 'Tiles and origin', '', null],
    ],
  };

  function buildShow() {
    segmented($('zoom-seg'), (value) => { view.zoom = +value; });
    for (const [containerId, items] of Object.entries(SHOW)) {
      const container = $(containerId);
      for (const [key, label, tip, swatch] of items) {
        const input = el('input', { type: 'checkbox' });
        input.checked = view[key];
        input.addEventListener('change', () => { view[key] = input.checked; });
        const sw = el('span', { class: 'sw' });
        sw.style.background = swatch ? `var(--${swatch})` : 'transparent';
        container.append(el('label', { title: tip || null }, [input, sw, el('span', { text: label })]));
      }
    }
  }

  function buildLegend() {
    const items = [
      ['var(--robot-line)', '', 'Where the robot really is'],
      ['var(--raw)', 'dash', 'Odometry alone'],
      ['var(--corrected)', '', 'Corrected pose'],
      ['var(--warning)', 'dot', 'Heavy particle'],
      ['var(--muted)', 'dot', 'Light particle'],
      ['var(--beam)', '', 'Sensor beam'],
    ];
    const legend = $('legend');
    for (const [color, kind, label] of items) {
      const swatch = el('i', { class: kind });
      swatch.style.color = color;
      legend.append(el('span', {}, [swatch, label]));
    }
  }

  // --- Events tab ---------------------------------------------------------------------------

  const SCENARIOS = [
    { name: 'Clean robot', apply: (o) => Object.assign(o.world, pickErrors(W.defaultSettings())) },
    { name: 'Worn tracking wheels', apply: (o) => Object.assign(o.world, { wheelDiameterErrorPct: 3, wheelSlipPct: 2, imuDriftDegPerMin: 3 }) },
    { name: 'Mis-measured offsets', apply: (o) => Object.assign(o.world, { offsetErrorIn: 1.0 }) },
    // the vertical wheel is really on the right, but its offset was typed as the positive number
    // OdometryConfig's old docs suggested: every turn drags the pose sideways
    { name: 'Vertical offset sign wrong', apply: (o) => Object.assign(o.world, { verticalWheelFlipped: true }) },
    { name: 'Noisy sensors', apply: (o) => Object.assign(o.world, { sensorNoiseScale: 3, sensorDropoutPct: 15 }) },
    {
      name: 'Crowded field',
      apply: (o) => {
        for (const e of o.world.elements) e.inWorld = true;
        Object.assign(o.world.defender, { enabled: true, patrol: true, xIn: 0, yIn: 30 });
      },
    },
  ];

  function allElements(inWorld) {
    return W.exampleElements().map((e) => Object.assign(e, { inWorld }));
  }

  function pickErrors(settings) {
    const out = {};
    for (const key of ['wheelDiameterErrorPct', 'offsetErrorIn', 'verticalWheelFlipped', 'wheelSlipPct', 'imuDriftDegPerMin', 'imuScaleErrorPct',
      'imuNoiseDeg', 'sensorNoiseScale', 'sensorDelayMs', 'sensorDropoutPct']) out[key] = settings[key];
    return out;
  }

  function buildEvents() {
    $('bump-btn').addEventListener('click', () => {
      const angle = Math.random() * 2 * Math.PI;
      sim.world.bump(6 * Math.cos(angle), 6 * Math.sin(angle));
    });
    $('kidnap-btn').addEventListener('click', () => {
      kidnapArmed = true;
      const hint = $('field-hint');
      hint.textContent = 'Click the field where the robot should be put down';
      hint.hidden = false;
    });
    $('relocalize-btn').addEventListener('click', () => sim.robot.localizer.relocalizeGlobally());

    const d = options.world.defender;
    checkboxList($('defender-checks'), [
      ['On the field', () => d.enabled, (v) => { d.enabled = v; sim.refreshMaps(); },
        'Drag it on the field. The sensors see it; the map doesn\'t'],
      ['Patrol', () => d.patrol, (v) => { d.patrol = v; }, 'Drive back and forth'],
    ]);
    // one list of elements, each really there or not, and the map has all of them or none
    checkboxList($('element-checks'), [
      ['In the world', () => options.world.elements.some((e) => e.inWorld), (v) => {
        for (const e of options.world.elements) e.inWorld = v;
        sim.refreshMaps();
      }, 'The sensors see them (filled boxes)'],
      ['In the map', () => options.elementsInMap, (v) => {
        options.elementsInMap = v;
        sim.refreshMaps();
      }, 'The localizer expects them (dashed outlines). In the map but not the world is the worse mistake'],
    ]);
    const buttons = $('scenario-buttons');
    for (const scenario of SCENARIOS) {
      buttons.append(el('button', {
        class: 'btn', type: 'button', text: scenario.name,
        onclick: () => {
          Object.assign(options.world, pickErrors(W.defaultSettings()));
          options.world.elements = allElements(false);
          options.elementsInMap = false;
          options.world.defender.enabled = false;
          options.world.defender.patrol = false;
          scenario.apply(options);
          syncSliders();
          refreshChecks();
          runRoutine(selectedRoutine);
        },
      }));
    }
  }

  const checkRefreshers = [];

  function checkboxList(container, items) {
    for (const [label, get, set, tip] of items) {
      const input = el('input', { type: 'checkbox' });
      input.checked = get();
      input.addEventListener('change', () => set(input.checked));
      checkRefreshers.push(() => { input.checked = get(); });
      container.append(el('label', { title: tip || null }, [input, el('span', { text: label })]));
    }
  }

  function refreshChecks() {
    for (const refresh of checkRefreshers) refresh();
  }

  // --- Robot tab ---------------------------------------------------------------------------

  const WORLD_SLIDERS = [
    ['wheelDiameterErrorPct', 'Tracking wheel diameter error', -5, 5, 0.1, '%', 'The wheels roll this much farther than configured'],
    ['wheelSlipPct', 'Tracking wheel slip', 0, 5, 0.1, '%', 'Random, as a percent of travel'],
    ['offsetErrorIn', 'Tracking wheel offset error', -2, 2, 0.1, 'in', 'Offsets measured wrong: turning drags the pose'],
    ['imuDriftDegPerMin', 'IMU drift', 0, 10, 0.1, '°/min', ''],
    ['imuScaleErrorPct', 'IMU scale error', -2, 2, 0.1, '%', 'Before calibrateHeadingScale()'],
    ['sensorNoiseScale', 'Distance sensor noise', 0, 5, 0.1, '×', '1 is half the V5 spec (±15mm, ±5%) as one standard deviation'],
    ['sensorDelayMs', 'Distance sensor delay', 0, 100, 1, 'ms', 'On top of its ~30Hz sampling'],
    ['sensorDropoutPct', 'Distance sensor dropouts', 0, 50, 1, '%', 'Readings of "no object"'],
    ['sensorOutlierPct', 'Distance sensor outliers', 0, 30, 0.5, '%', 'Readings off something nearer than the wall that no map has: a game element, a robot'],
  ];

  const sliderSyncers = [];

  function slider(container, { label, min, max, step, unit, hint, get, set }) {
    const input = el('input', { type: 'range', min, max, step });
    input.value = get();
    const out = el('output', { text: `${get()}${unit}` });
    input.addEventListener('input', () => {
      set(+input.value);
      out.textContent = `${input.value}${unit}`;
    });
    sliderSyncers.push(() => {
      input.value = get();
      out.textContent = `${get()}${unit}`;
    });
    container.append(el('label', { class: 'slider', title: hint || null }, [label, out, input]));
  }

  function syncSliders() {
    for (const sync of sliderSyncers) sync();
  }

  function buildRobot() {
    const box = $('world-sliders');
    for (const [key, label, min, max, step, unit, hint] of WORLD_SLIDERS) {
      slider(box, { label, min, max, step, unit, hint, get: () => options.world[key], set: (v) => { options.world[key] = v; } });
    }
    const grid = $('model-grid');
    // per inch for Forward and Strafe, per degree for Turn
    grid.append(el('span'), el('span', { class: 'h', text: 'kS (V)' }),
      el('span', { class: 'h', text: 'kV (V·s/u)', title: 'volts per unit/s: inches, or degrees for Turn' }),
      el('span', { class: 'h', text: 'kA (V·s²/u)', title: 'volts per unit/s²: inches, or degrees for Turn' }));
    for (const [axis, label] of [['forward', 'Forward'], ['strafe', 'Strafe'], ['turn', 'Turn']]) {
      grid.append(el('span', { text: label }));
      for (const term of ['kS', 'kV', 'kA']) {
        const input = el('input', { type: 'number', step: 'any', value: options.world.models[axis][term] });
        // a calibrated world (Tune tab) can bring the robot's own models
        sliderSyncers.push(() => { input.value = options.world.models[axis][term]; });
        input.addEventListener('change', () => {
          const value = +input.value;
          if (!Number.isFinite(value) || value < 0) return;
          options.world.models[axis][term] = value;
          // the running chassis too: PlantSim reads its model every step
          const m = options.world.models[axis];
          sim.world.plants[axis].model = M.mechanismModel(M.feedforward(m.kS, m.kV, m.kA));
        });
        grid.append(input);
      }
    }
    grid.append(el('span', { text: 'Delay (s)', title: 'Between commanding volts and the chassis responding' }));
    const delay = el('input', { type: 'number', step: 'any', value: options.world.delayS });
    sliderSyncers.push(() => { delay.value = options.world.delayS; });
    delay.addEventListener('change', () => {
      if (!(+delay.value >= 0)) return;
      options.world.delayS = +delay.value;
      for (const plant of Object.values(sim.world.plants)) plant.delayS = options.world.delayS;
    });
    grid.append(delay, el('span'), el('span'));
    $('autotune-btn').addEventListener('click', () => {
      options.useAutoTune = true;
      setSegment($('gains-seg'), 'auto');
      sim.applyGains();
      updateGainsReadout();
    });
  }

  // --- MCL tab ---------------------------------------------------------------------------

  // [path in LocalizerConfig, label, min, max, step, unit, hint]
  const MCL_SLIDERS = [
    ['filter.particleCount', 'Particles', 50, 3000, 50, '', 'More is steadier and slower'],
    ['periodMs', 'Update period', 20, 200, 10, 'ms', 'startTask(periodMs)'],
    ['filter.motionNoise.perInch', 'Motion noise per inch', 0, 0.2, 0.005, '', 'How much odometry is doubted as it moves'],
    ['filter.motionNoise.baseIn', 'Motion noise, standing still', 0, 0.2, 0.005, 'in', ''],
    ['filter.motionNoise.perDegreeIn', 'Motion noise per degree turned', 0, 0.05, 0.001, 'in', ''],
    ['filter.beam.outlierProbability', 'Outlier probability', 0, 0.5, 0.01, '', 'How often a reading is expected to be junk'],
    ['filter.beam.sigmaFraction', 'Sensor noise, fraction of distance', 0.01, 0.2, 0.005, '', ''],
    ['filter.recovery.triggerRatio', 'Recovery trigger', 0.3, 1, 0.05, '', 'Recover once the fit drops below this share of its usual level'],
    ['filter.recovery.radiusIn', 'Recovery radius', 0, 72, 1, 'in', '0 scatters recovery particles over the whole field'],
    ['sensorLatencyMs', 'Latency compensation', 0, 100, 5, 'ms', ''],
    ['maxCorrectionSpreadIn', 'Correct only below spread', 0.5, 10, 0.5, 'in', ''],
    ['minAgreeingSensors', 'Correct only with agreeing sensors', 1, 4, 1, '', ''],
    ['maxCorrectionRateInPerS', 'Correction speed', 0, 20, 0.5, 'in/s', '0 jumps; the PIDs see it as a spike'],
  ];

  const MCL_CHECKS = [
    ['filter.recovery.enabled', 'Recovery', 'Scatter fresh particles when readings suddenly stop matching'],
    ['waitForSetPose', 'Wait for setPose()', 'Correct nothing until odometry has a field pose'],
  ];

  function defaultMcl() {
    const config = R.localizerConfig({});
    config.periodMs = 50;
    return config;
  }

  let mclConfig = defaultMcl();

  function getPath(object, path) {
    return path.split('.').reduce((o, k) => o[k], object);
  }

  function setPath(object, path, value) {
    const keysOf = path.split('.');
    const last = keysOf.pop();
    keysOf.reduce((o, k) => o[k], object)[last] = value;
  }

  function buildMcl() {
    const box = $('mcl-sliders');
    for (const [path, label, min, max, step, unit, hint] of MCL_SLIDERS) {
      slider(box, { label, min, max, step, unit, hint, get: () => getPath(mclConfig, path), set: (v) => {
        setPath(mclConfig, path, v);
        applyMcl();
      } });
    }
    const checks = $('mcl-checks');
    for (const [path, label, hint] of MCL_CHECKS) {
      const input = el('input', { type: 'checkbox' });
      input.checked = getPath(mclConfig, path);
      input.addEventListener('change', () => {
        setPath(mclConfig, path, input.checked);
        applyMcl();
      });
      checkRefreshers.push(() => { input.checked = getPath(mclConfig, path); });
      checks.append(el('label', { title: hint }, [input, el('span', { text: label })]));
    }
    $('mcl-defaults-btn').addEventListener('click', () => {
      mclConfig = defaultMcl();
      syncSliders();
      refreshChecks();
      applyMcl();
    });
  }

  let mclTimer = null;

  /** Rebuild the localizer with the new settings, a moment after the last change. */
  function applyMcl() {
    updateMclCode();
    clearTimeout(mclTimer);
    mclTimer = setTimeout(() => {
      const { periodMs, ...config } = mclConfig;
      options.localizer = JSON.parse(JSON.stringify(config));
      options.localizerPeriodMs = periodMs;
      const robot = sim.robot;
      robot.localizer = new R.MonteCarloLocalizer(robot.odometry, sim.world.settings.sensors.map((_, i) => () => sim.world.distanceSensor(i)),
        options.localizerMounts || sim.world.settings.sensors, sim.localizerMap(), options.localizer, robot.clock);
      robot.localizer.setCorrectionEnabled(options.correctOdometry);
      robot.localizerPeriodMs = periodMs;
      robot.localizer.update();
    }, 150);
  }

  /**
   * The non-default settings two ways: TUNE.CFG lines (the Tune tab saves
   * them, and the robot loads them without a rebuild), and the C++ for the
   * LocalizerConfig in localizerSettings() (src/robot/tune.cpp), nested in
   * declaration order the way designated initializers need.
   */
  function updateMclCode() {
    const flat = mclFlat();
    const changed = TF.LOCALIZER_SETTINGS.filter((setting) => flat[setting.key] !== setting.default);
    const cfg = changed.map((setting) => `mcl.${setting.key}=${TF.num(flat[setting.key])}`);
    if (flat.periodMs !== TF.DEFAULT_PERIOD_MS) cfg.unshift(`mcl.periodMs=${flat.periodMs}`);
    $('mcl-cfg').textContent = cfg.length ? cfg.join('\n') : '# the defaults: nothing to add';

    const literal = (setting, value) => {
      if (['filter.recovery.enabled', 'correctOdometry', 'waitForSetPose'].includes(setting.key)) return value ? 'true' : 'false';
      if (setting.whole) return String(value);
      return Number.isInteger(value) ? `${value}.0` : TF.num(value);
    };
    // { name, fields: [...] } groups, in the order the structs declare them
    const tree = { fields: [] };
    for (const setting of changed) {
      const parts = setting.key.split('.');
      let node = tree;
      for (const part of parts.slice(0, -1)) {
        let child = node.fields.find((f) => f.name === part && f.fields);
        if (!child) {
          child = { name: part, fields: [] };
          node.fields.push(child);
        }
        node = child;
      }
      node.fields.push({ name: parts[parts.length - 1], value: literal(setting, flat[setting.key]) });
    }
    const render = (node) => node.fields.map((f) => (f.fields ? `.${f.name} = {${render(f)}}` : `.${f.name} = ${f.value}`)).join(', ');
    let code = changed.length ? `LocalizerConfig{\n    ${tree.fields.map((f) => render({ fields: [f] })).join(',\n    ')},\n}`
      : 'LocalizerConfig{}  // the defaults';
    if (flat.periodMs !== TF.DEFAULT_PERIOD_MS) code += `\n\n// and in initDevices()\nlocalizer().startTask(${flat.periodMs});`;
    $('mcl-code').textContent = code;
  }

  /** The MCL tab's settings as a flat { path: value } config, like TUNE.CFG's mcl.* lines. */
  function mclFlat() {
    const out = {};
    for (const setting of TF.LOCALIZER_SETTINGS) {
      const value = getPath(mclConfig, setting.key);
      out[setting.key] = typeof value === 'boolean' ? (value ? 1 : 0) : value;
    }
    out.periodMs = mclConfig.periodMs;
    return out;
  }

  /** Sets the MCL tab, and the running localizer, from a flat config. */
  function setMclFlat(flat) {
    mclConfig = defaultMcl();
    for (const setting of TF.LOCALIZER_SETTINGS) {
      if (flat[setting.key] === undefined) continue;
      const isSwitch = typeof getPath(mclConfig, setting.key) === 'boolean';
      setPath(mclConfig, setting.key, isSwitch ? flat[setting.key] !== 0 : flat[setting.key]);
    }
    if (flat.periodMs !== undefined) mclConfig.periodMs = flat.periodMs;
    syncSliders();
    refreshChecks();
    applyMcl();
  }

  // --- Tune tab -------------------------------------------------------------------------------

  function buildTune() {
    SIM.tuneview.build({
      $, el, fmt, segmented, setSegment,
      getMclFlat: mclFlat,
      setMclFlat,
      setPlaying: (value) => setPlaying(value),
      applyWorld,
      saveText,
    });
    // the chart sizes itself when the tab is first shown
    $('tabs').addEventListener('click', () => requestAnimationFrame(() => SIM.tuneview.drawHistory()));
  }

  /** The world's errors from a calibration (Tune tab), and a fresh run in it. */
  function applyWorld(world) {
    const { models, ...rest } = world;
    Object.assign(options.world, rest);
    if (models) options.world.models = JSON.parse(JSON.stringify(models));
    syncSliders();
    runRoutine(selectedRoutine);
  }

  /** Saves text as a file: a save dialog where the browser has one, a download otherwise. */
  async function saveText(name, text) {
    if (window.showSaveFilePicker) {
      try {
        const handle = await window.showSaveFilePicker({ suggestedName: name,
          types: [{ description: 'SapphireLib tuning profile', accept: { 'text/plain': ['.cfg', '.CFG'] } }] });
        const writable = await handle.createWritable();
        await writable.write(text);
        await writable.close();
        return true;
      } catch (error) {
        if (error && error.name === 'AbortError') return false; // they cancelled
        // anything else (a file:// page some browsers refuse): fall back to a download
      }
    }
    const link = el('a', { href: URL.createObjectURL(new Blob([text], { type: 'text/plain' })), download: name });
    document.body.append(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
    return true;
  }

  // --- the field: selecting, dragging, kidnapping ---------------------------------------------------------------------------

  function bindField() {
    const canvas = $('field');
    const at = (event) => {
      const rect = canvas.getBoundingClientRect();
      return [event.clientX - rect.left, event.clientY - rect.top];
    };
    const onDefender = (px, py) => {
      const d = options.world.defender;
      if (!d.enabled) return false;
      const p = renderer.toField(px, py);
      return Math.abs(p.xIn - d.xIn) <= d.sizeIn / 2 && Math.abs(p.yIn - d.yIn) <= d.sizeIn / 2;
    };
    canvas.addEventListener('pointerdown', (event) => {
      const [px, py] = at(event);
      if (kidnapArmed) {
        const p = renderer.toField(px, py);
        sim.world.kidnap(p.xIn, p.yIn);
        kidnapArmed = false;
        $('field-hint').hidden = true;
        return;
      }
      if (onDefender(px, py)) {
        draggingDefender = true;
        canvas.classList.add('dragging');
        canvas.setPointerCapture(event.pointerId);
        return;
      }
    });
    canvas.addEventListener('pointermove', (event) => {
      const [px, py] = at(event);
      if (draggingDefender) {
        const p = renderer.toField(px, py);
        const d = options.world.defender;
        const limit = options.world.fieldSizeIn / 2 - d.sizeIn / 2;
        d.xIn = Math.max(-limit, Math.min(limit, p.xIn));
        d.yIn = Math.max(-limit, Math.min(limit, p.yIn));
        d.patrol = false;
        refreshChecks();
        sim.world.rebuildMap();
      } else {
        canvas.style.cursor = onDefender(px, py) ? 'grab' : '';
      }
    });
    const end = () => {
      draggingDefender = false;
      canvas.classList.remove('dragging');
    };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
  }

  function bindKeyboard() {
    const typing = (event) => /^(INPUT|SELECT|TEXTAREA)$/.test(event.target.tagName) && event.target.type !== 'checkbox' &&
      event.target.type !== 'range';
    window.addEventListener('keydown', (event) => {
      if (typing(event)) return;
      const key = event.key.toLowerCase();
      if (key === ' ') {
        event.preventDefault();
        setPlaying(!playing);
        return;
      }
      if (key === '?') {
        toggleHelp();
        return;
      }
      if (key === 'escape') {
        toggleHelp(false);
        return;
      }
      if (['w', 'a', 's', 'd', 'q', 'e', 'arrowleft', 'arrowright', 'arrowup', 'arrowdown', 'shift'].includes(key)) {
        if (key.startsWith('arrow')) event.preventDefault();
        keys.add(key);
        // taking the sticks stops a running routine, like a driver grabbing the controller
        if (key !== 'shift' && sim.program) sim.stopProgram();
      }
    });
    window.addEventListener('keyup', (event) => keys.delete(event.key.toLowerCase()));
    window.addEventListener('blur', () => keys.clear());
  }

  function driverInput() {
    const stick = (+$('stick').value / 100) * (keys.has('shift') ? 0.4 : 1);
    const axis = (plus, minus) => (plus.some((k) => keys.has(k)) ? 1 : 0) - (minus.some((k) => keys.has(k)) ? 1 : 0);
    return {
      throttle: stick * axis(['w', 'arrowup'], ['s', 'arrowdown']),
      strafe: stick * axis(['d'], ['a']),
      turn: stick * 0.6 * axis(['e', 'arrowright'], ['q', 'arrowleft']),
    };
  }

  // --- the loop ---------------------------------------------------------------------------

  function frame(nowMs) {
    const elapsed = lastFrameMs === null ? 0 : Math.min(100, nowMs - lastFrameMs);
    lastFrameMs = nowMs;
    if (playing) {
      accumulatorMs += elapsed * speed;
      let ticks = 0;
      while (accumulatorMs >= kTickMs && ticks < kMaxTicksPerFrame) {
        const d = driverInput();
        sim.setDriverInput(d.throttle, d.strafe, d.turn);
        sim.tick();
        accumulatorMs -= kTickMs;
        ticks++;
        if (sim.programResult) showResult();
      }
      if (ticks === kMaxTicksPerFrame) accumulatorMs = 0;
    }
    renderer.draw(sim, view);
    if (nowMs - lastDomMs > 120) {
      updateDom();
      lastDomMs = nowMs;
    }
    scheduleFrame();
  }

  // requestAnimationFrame, with a timer behind it for pages it's starved in (some embedded
  // views never fire it). The token lets whichever fires first run the frame and cancels the other
  let frameToken = 0;

  function scheduleFrame() {
    const token = ++frameToken;
    requestAnimationFrame((nowMs) => {
      if (token === frameToken) frame(nowMs);
    });
    setTimeout(() => {
      if (token === frameToken) frame(performance.now());
    }, 50);
  }

  function showResult() {
    const r = sim.programResult;
    sim.programResult = null;
    const box = $('routine-result');
    box.hidden = false;
    const source = options.correctOdometry ? 'odometry + MCL' : 'odometry alone';
    box.textContent = `${r.name}, by ${source}: done at ${fmt(r.atMs / 1000, 1)} s, ${fmt(r.errors.corrected, 2)} in ` +
      `off (odometry alone ${fmt(r.errors.raw, 2)} in).`;
  }

  // --- readouts ---------------------------------------------------------------------------

  function tile(label, value, foot, color, cls = '') {
    const labelEl = el('span', { class: 'label' }, [color ? el('i') : null, label]);
    if (color) labelEl.firstChild.style.color = color;
    return el('div', { class: `tile ${cls}` }, [labelEl, el('span', { class: 'value num', text: value }), el('span', { class: 'foot', text: foot })]);
  }

  /** Why the last update didn't correct odometry, in a few words: blockedBy's first reason. */
  function blockedText(loc) {
    const b = loc.status.blockedBy;
    const B = R.BLOCKED;
    const c = loc.config;
    if (b & B.correctionOff) return 'off (Drive tab)';
    if (b & B.noSetPose) return 'no setPose() yet';
    if (b & B.spinning) return 'spinning: readings skipped';
    if (b & B.noReadings) return 'no readings';
    if (b & B.tooSpread) return `spread over ${fmt(c.maxCorrectionSpreadIn, 1)} in`;
    if (b & B.tooFewAgree) return `${loc.status.sensorsAgreeing} agree, needs ${c.minAgreeingSensors}`;
    if (b & B.refused) return 'setPose() mid-update';
    return 'holding';
  }

  function updateDom() {
    $('clock').textContent = `${fmt(sim.world.timeMs / 1000, 1)}s`;
    const e = sim.errors();
    const loc = sim.robot.localizer;
    const st = loc.status;
    const n = loc.filter.particles.length;
    $('tiles').replaceChildren(
      tile('Odometry alone', `${fmt(e.raw, 2)} in`, 'from the truth', 'var(--raw)'),
      tile('Corrected pose', `${fmt(e.corrected, 2)} in`, 'from the truth', 'var(--corrected)'),
      tile('Particle spread', `${fmt(st.spreadIn, 2)} in`, `neff ${fmt((100 * st.effectiveParticles) / n, 0)}% of ${n}`,
        'var(--accent)'),
      tile('Sensors agree', `${st.sensorsAgreeing} / ${st.sensorsUsed}`, `agree / reading, of ${loc.sensors.length}`,
        'var(--good)'),
      tile('Correcting', st.correcting ? 'Yes' : 'No', st.correcting
        ? `by ${fmt(Math.hypot(st.correctionXIn, st.correctionYIn), 2)} in` : blockedText(loc), null,
      st.correcting ? 'good' : 'off'),
    );
    drawChart();
  }

  function drawChart() {
    SIM.render.drawChart($('chart'), sim.history, renderer.colors);
  }

  root.SIM.app = { start };
})();
