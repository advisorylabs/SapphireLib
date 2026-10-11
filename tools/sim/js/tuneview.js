/*
 * SapphireLib simulator: tuneview.js
 *
 * The Tune tab: auto-tuning the localizer (tuner.js) and calibrating the
 * simulated world from the robot's logs (mclcal.js), with TUNE.CFG in and
 * out so what it finds reaches the robot without a rebuild.
 *
 * The search runs on the page's own thread in slices of a few tens of
 * milliseconds between frames, so the page stays usable and a file:// page
 * needs no worker. The live simulation pauses while it runs.
 *
 * Browser only: window.SIM.tuneview. app.js hands it a context with the
 * few things it needs from the rest of the page.
 *
 * Team 96671H: Hitmen
 */
(function () {
  'use strict';

  const root = typeof self !== 'undefined' ? self : this;
  const SIM = root.SIM;
  const T = SIM.tuner;
  const REC = SIM.recorder;
  const slt = root.SA.slt;
  const TF = root.SA.tunefile;

  const kSliceMs = 35;
  const kHiddenSliceMs = 900;
  const kSensorNames = ['Front', 'Right', 'Back', 'Left'];
  const BUDGETS = [['Quick', 40], ['Normal', 100], ['Thorough', 200]];

  // a simulated robot to calibrate from when there are no real logs yet: the Demo button
  const MYSTERY_WORLD = { sensorNoiseScale: 1.8, sensorDropoutPct: 5, sensorOutlierPct: 4, sensorDelayMs: 45,
    wheelDiameterErrorPct: 2 };

  let ctx;
  let $;
  let el;
  let fmt;

  const state = {
    suite: 'standard',
    robot: null, // calibrateWorld()'s result, once logs are in
    robotCases: null,
    profile: TF.emptyProfile(),
    profileName: null,
    running: null, // { kind: 'search' | 'calibrate', stop }
    result: null,
    partial: null,
    history: [],
    mystery: null,
  };

  // --- Setup ------------------------------------------------------------------------------------

  function build(context) {
    ctx = context;
    $ = ctx.$;
    el = ctx.el;
    fmt = ctx.fmt;

    // the suite to search on
    ctx.segmented($('tune-suite'), (value) => {
      state.suite = value;
      renderCases();
    });
    renderCases();

    const params = $('tune-params');
    for (const p of T.PARAMS) {
      const input = el('input', { type: 'checkbox', 'data-key': p.key });
      input.checked = p.on;
      const why = p.key === 'sensorLatencyMs' ? 'Off by default: worth a try once the world is calibrated from logs'
        : p.key === 'filter.particleCount' || p.key === 'periodMs' ? 'Costs CPU on the brain; the score charges for it'
          : p.key === 'maxCorrectionRateInPerS' ? 'Faster corrections are a harder push on the motions, which the score can\'t see'
            : '';
      params.append(el('label', { title: why ? `${p.key}. ${why}` : p.key }, [input, el('span', { text: p.label })]));
    }

    const budget = $('tune-budget');
    for (const [label, n] of BUDGETS) budget.append(el('option', { value: n, text: `${label}: ${n} tries` }));
    budget.value = '100';

    $('tune-start-btn').addEventListener('click', () => (state.running ? stop() : startSearch()));
    $('tune-files').addEventListener('change', (event) => loadFiles([...event.target.files]));
    $('tune-open-btn').addEventListener('click', () => $('tune-files').click());
    $('tune-demo-btn').addEventListener('click', demoLogs);
    $('tune-use-world-btn').addEventListener('click', useWorld);
    $('tune-use-btn').addEventListener('click', useResult);
    $('tune-save-btn').addEventListener('click', saveTuneFile);
    $('tune-from-file-btn').addEventListener('click', () => {
      ctx.setMclFlat(TF.localizerSettings(state.profile));
      flash($('tune-file-status'), 'The MCL tab now has this file\'s settings; searches start from them.');
    });

    const drop = $('tune-drop');
    drop.addEventListener('dragover', (event) => {
      event.preventDefault();
      drop.classList.add('over');
    });
    drop.addEventListener('dragleave', () => drop.classList.remove('over'));
    drop.addEventListener('drop', (event) => {
      event.preventDefault();
      drop.classList.remove('over');
      loadFiles([...event.dataTransfer.files]);
    });
    renderProfile();
    renderProgress(null);
  }

  function flash(node, text) {
    node.textContent = text;
    node.hidden = false;
  }

  // --- The suite --------------------------------------------------------------------------------

  function currentCases() {
    return state.suite === 'robot' && state.robotCases ? state.robotCases : T.standardCases();
  }

  function renderCases() {
    const box = $('tune-cases');
    box.replaceChildren();
    const robotButton = $('tune-suite').querySelector('[data-value="robot"]');
    robotButton.disabled = !state.robotCases;
    robotButton.title = state.robotCases ? '' : 'Calibrate from the robot\'s logs first (below)';
    if (state.suite === 'robot' && !state.robotCases) {
      state.suite = 'standard';
      ctx.setSegment($('tune-suite'), 'standard');
    }
    for (const c of currentCases()) {
      const input = el('input', { type: 'checkbox', 'data-case': c.id });
      input.checked = true;
      box.append(el('label', {}, [input, el('span', { text: c.name })]));
    }
    $('tune-suite-hint').textContent = state.suite === 'robot'
      ? 'Your calibrated world: two routines and your robot\'s own paths.'
      : 'The simulator\'s scenarios.';
  }

  function checkedCases() {
    const ids = [...$('tune-cases').querySelectorAll('input:checked')].map((i) => i.dataset.case);
    return currentCases().filter((c) => ids.includes(c.id));
  }

  function checkedParams() {
    return [...$('tune-params').querySelectorAll('input:checked')].map((i) => i.dataset.key);
  }

  // --- Running a generator in slices --------------------------------------------------------------

  /**
   * Steps `generator` a slice at a time between frames, calling onStep for
   * each value and onDone with the result. Returns a stop function.
   */
  function pump(generator, onStep, onDone) {
    let stopped = false;
    const slice = () => {
      if (stopped) return;
      // a hidden page's timers fire about once a second, so a short slice would crawl; with no one
      // watching, there's nothing to stay responsive for
      const until = performance.now() + (document.hidden ? kHiddenSliceMs : kSliceMs);
      while (performance.now() < until) {
        const step = generator.next();
        if (step.done) {
          onDone(step.value);
          return;
        }
        onStep(step.value);
      }
      setTimeout(slice, 0);
    };
    setTimeout(slice, 0);
    return () => {
      stopped = true;
    };
  }

  function setRunning(kind, stopFn) {
    state.running = kind ? { kind, stop: stopFn } : null;
    $('tune-start-btn').textContent = state.running && state.running.kind === 'search' ? 'Stop' : 'Start search';
    $('tune-start-btn').disabled = !!state.running && state.running.kind !== 'search';
    for (const id of ['tune-open-btn', 'tune-demo-btn', 'tune-use-world-btn']) $(id).disabled = !!state.running;
  }

  function stop() {
    if (!state.running) return;
    state.running.stop();
    const kind = state.running.kind;
    setRunning(null);
    ctx.setPlaying(true);
    if (kind === 'search' && state.partial) renderStopped();
  }

  // --- Searching -----------------------------------------------------------------------------------

  function startSearch() {
    const cases = checkedCases();
    const params = checkedParams();
    if (!cases.length || !params.length) {
      flash($('tune-progress-text'), 'Pick at least one case and one setting to search.');
      return;
    }
    const start = ctx.getMclFlat();
    const budget = +$('tune-budget').value;
    ctx.setPlaying(false);
    state.result = null;
    state.partial = { start, best: start, bestScore: 1, evaluations: 0 };
    state.history = [];
    $('tune-result').hidden = true;
    const began = performance.now();
    let runs = 0;
    const generator = T.search({ start, cases, params, budget });
    const stopFn = pump(generator, (event) => {
      if (event.type === 'prepare' || event.type === 'run') runs++;
      if (event.type === 'eval') {
        state.partial = { start, best: event.best, bestScore: event.bestScore, evaluations: event.evaluations };
        state.history.push({ evaluation: event.evaluations, score: event.score, best: event.bestScore,
          accepted: event.accepted });
        renderProgress({ event, budget, began, runs });
      }
      if (event.type === 'holdout') renderProgress({ holdout: true, budget, began, runs });
    }, (result) => {
      setRunning(null);
      ctx.setPlaying(true);
      state.result = result;
      renderProgress({ done: result, budget, began, runs });
      renderResult(result);
    });
    setRunning('search', stopFn);
    renderProgress({ starting: true, budget, began, runs });
  }

  function scoreText(score) {
    const pct = (1 - score) * 100;
    return pct >= 0.5 ? `${fmt(pct, 0)}% less error` : pct <= -0.5 ? `${fmt(-pct, 0)}% more error` : 'no change';
  }

  function renderProgress(p) {
    const bar = $('tune-bar');
    const text = $('tune-progress-text');
    if (!p) {
      bar.style.setProperty('--done', '0');
      text.textContent = 'Not started. A Normal search takes a minute or two.';
      drawHistory();
      return;
    }
    text.hidden = false;
    const elapsed = (performance.now() - p.began) / 1000;
    if (p.starting) {
      text.textContent = 'Driving each case once to record the path every candidate will replay…';
    } else if (p.holdout) {
      bar.style.setProperty('--done', '1');
      text.textContent = 'Checking the winner on seeds the search never saw…';
    } else if (p.done) {
      bar.style.setProperty('--done', '1');
      text.textContent = `Done: ${p.done.evaluations} settings tried, ${p.runs} simulated runs in ${fmt(elapsed, 0)}s.`;
    } else {
      const e = p.event;
      bar.style.setProperty('--done', String(Math.min(1, e.evaluations / p.budget)));
      const eta = e.evaluations > 1 ? (elapsed / e.evaluations) * (p.budget - e.evaluations) : NaN;
      const trying = e.key ? `${labelOf(e.key)} ${TF.num(e.value)}: ${e.accepted ? 'kept' : 'no better'}` : 'the start';
      text.textContent = `Try ${e.evaluations} of ${p.budget} (${trying}). Best so far: ${scoreText(e.bestScore)}. ` +
        `${fmt(elapsed, 0)}s${Number.isFinite(eta) ? `, about ${fmt(eta, 0)}s left at most` : ''}.`;
    }
    drawHistory();
  }

  function labelOf(key) {
    const p = T.PARAMS.find((x) => x.key === key);
    return p ? p.label : key;
  }

  /** Every try's score (dots) and the best so far (line), 1.0 being the start. */
  function drawHistory() {
    const canvas = $('tune-chart');
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (!width || !height) return;
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    const g = canvas.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    const pad = { l: 34, r: 8, t: 8, b: 18 };
    const h = state.history;
    const n = Math.max(10, h.length);
    const scores = h.map((x) => x.score).concat([1]);
    const lo = Math.max(0, Math.min(...scores, ...h.map((x) => x.best)) - 0.05);
    const hi = Math.min(2, Math.max(1.05, ...scores.filter((s) => s < 2)));
    const x = (i) => pad.l + ((i - 1) / Math.max(1, n - 1)) * (width - pad.l - pad.r);
    const y = (s) => pad.t + ((hi - Math.min(hi, s)) / (hi - lo)) * (height - pad.t - pad.b);
    g.clearRect(0, 0, width, height);
    g.font = '11px ui-monospace, monospace';
    g.fillStyle = css('--muted');
    g.strokeStyle = css('--grid');
    g.lineWidth = 1;
    for (const s of [lo, 1, hi]) {
      g.beginPath();
      g.moveTo(pad.l, y(s));
      g.lineTo(width - pad.r, y(s));
      g.stroke();
      g.fillText(s.toFixed(2), 2, y(s) + 4);
    }
    g.fillText('tries', width - pad.r - 30, height - 4);
    for (const point of h) {
      g.fillStyle = point.accepted ? css('--accent') : css('--axis');
      g.beginPath();
      g.arc(x(point.evaluation), y(point.score), point.accepted ? 3 : 2, 0, 2 * Math.PI);
      g.fill();
    }
    if (h.length) {
      g.strokeStyle = css('--accent');
      g.lineWidth = 2;
      g.beginPath();
      h.forEach((point, i) => (i ? g.lineTo(x(point.evaluation), y(point.best)) : g.moveTo(x(point.evaluation), y(point.best))));
      g.stroke();
    }
  }

  // --- Results ------------------------------------------------------------------------------------

  function verdictText(result) {
    const h = result.holdout;
    const gain = `${fmt(h.gainPct, 0)}% ± ${fmt(h.sePct, 0)}%`;
    if (!result.changes.length) {
      return { cls: 'neutral', text: 'Nothing beat the starting settings by more than chance: they\'re already as good as this suite can tell. Keep them.' };
    }
    if (h.verdict === 'better') {
      return { cls: 'good', text: `${scoreText(result.bestScore)} than the starting settings on the search's own runs, and ${gain} less on seeds it never saw. Worth running on the robot.` };
    }
    if (h.verdict === 'marginal') {
      return { cls: 'neutral', text: `${scoreText(result.bestScore)} on the search's runs, and ${gain} on seeds it never saw: small enough to be noise. About as good as the start; try them if you like.` };
    }
    return { cls: 'bad', text: `${scoreText(result.bestScore)} on the search's runs, but no better (${gain}) on seeds it never saw: it found its own seeds' luck. Keep the starting settings.` };
  }

  function renderResult(result) {
    $('tune-result').hidden = false;
    const verdict = result.stopped
      ? { cls: 'neutral', text: `Stopped after ${result.evaluations} tries: ${scoreText(result.bestScore)} on the ` +
          'search\'s runs, not yet checked on runs it never saw. Let one finish before trusting it on the robot.' }
      : verdictText(result);
    const box = $('tune-verdict');
    box.className = `verdict ${verdict.cls}`;
    box.textContent = verdict.text;

    const changes = $('tune-changes');
    const rows = [el('tr', {}, ['Setting', 'Was', 'Now'].map((t) => el('th', { text: t })))];
    for (const c of result.changes) {
      rows.push(el('tr', {}, [el('td', { text: c.label }), el('td', { text: TF.num(c.from) }), el('td', { text: TF.num(c.to) })]));
    }
    if (!result.changes.length) rows.push(el('tr', {}, [el('td', { text: 'No changes', colspan: 3 })]));
    changes.replaceChildren(...rows);

    const perCase = $('tune-percase');
    perCase.hidden = !result.perCase;
    const head = el('tr', {}, ['Case', 'Mean error', 'New seeds'].map((t) => el('th', { text: t })));
    const caseRows = [head];
    for (const c of result.perCase || []) {
      const arrow = (a, b) => `${fmt(a, 2)} → ${fmt(b, 2)}`;
      caseRows.push(el('tr', {}, [
        el('td', { text: c.name }),
        el('td', { text: arrow(c.start.meanIn, c.best.meanIn) }),
        el('td', { text: arrow(c.holdoutStart.meanIn, c.holdoutBest.meanIn) }),
      ]));
    }
    perCase.replaceChildren(...caseRows);
    $('tune-use-btn').disabled = !result.changes.length;
    $('tune-save-btn').disabled = !result.changes.length;
    $('tune-save-status').hidden = true;
  }

  /** A stopped search: the best it had found, unchecked, and no per-case table. */
  function renderStopped() {
    const p = state.partial;
    state.result = { start: p.start, best: p.best, bestScore: p.bestScore, evaluations: p.evaluations, stopped: true,
      perCase: null,
      changes: T.PARAMS.filter((x) => p.best[x.key] !== p.start[x.key])
        .map((x) => ({ key: x.key, label: x.label, from: p.start[x.key], to: p.best[x.key] })) };
    renderResult(state.result);
  }

  function useResult() {
    if (!state.result) return;
    ctx.setMclFlat(state.result.best);
    flash($('tune-save-status'), 'The simulator now runs these settings (see the MCL tab, which also has the C++). Run a routine to see them.');
  }

  // --- TUNE.CFG -------------------------------------------------------------------------------------

  function renderProfile() {
    const p = state.profile;
    $('tune-file-name').textContent = state.profileName
      ? `${state.profileName}: revision ${p.revision}${p.note ? `, "${p.note}"` : ''}`
      : 'None open: saving starts a new one (revision 1).';
    const mcl = Object.entries(p.mcl);
    const lines = [...TF.entries(p).entries()].filter(([k]) => k.startsWith('mcl.')).map(([k, v]) => `${k}=${v}`);
    $('tune-file-mcl').textContent = lines.length ? lines.join('\n') : '(no mcl.* lines: the robot runs LocalizerConfig\'s defaults)';
    $('tune-from-file-btn').disabled = !mcl.length && !('mcl.periodMs' in p.values);
    const log = $('tune-file-log');
    log.replaceChildren(...p.changelog.slice(0, 8).map((line) => el('li', { text: line })));
    log.hidden = !p.changelog.length;
  }

  async function saveTuneFile() {
    if (!state.result) return;
    const note = state.suite === 'robot' ? 'MCL tuned on the world calibrated from the robot\'s logs'
      : 'MCL tuned on the simulator\'s standard suite';
    const revised = TF.revise(state.profile, T.tuneChanges(state.result.best, state.profile), { source: 'sim tuner', note });
    if (revised.error) {
      flash($('tune-save-status'), `Can't write it: ${revised.error}`);
      return;
    }
    if (!revised.changes.length) {
      flash($('tune-save-status'), 'TUNE.CFG already has these settings.');
      return;
    }
    const text = TF.format(revised.profile);
    const saved = await ctx.saveText('TUNE.CFG', text);
    if (!saved) return;
    state.profile = revised.profile;
    state.profileName = 'TUNE.CFG';
    renderProfile();
    flash($('tune-save-status'), `Saved revision ${revised.profile.revision}: ${revised.changes.length} change${revised.changes.length === 1 ? '' : 's'}. ` +
      'Copy it into the sl folder on the robot\'s SD card; the Home page shows the revision it loaded.');
  }

  // --- Calibrating from logs ------------------------------------------------------------------------

  async function loadFiles(files) {
    if (state.running) return;
    const logs = [];
    const problems = [];
    for (const file of files) {
      const text = await file.text();
      const isProfile = /\.cfg$/i.test(file.name) || (/^\s*(#|format\s*=)/.test(text) && !text.startsWith('#SLT'));
      if (isProfile) {
        const parsed = TF.parse(text);
        if (parsed.ok) {
          state.profile = parsed.profile;
          state.profileName = file.name;
          renderProfile();
        } else {
          problems.push(`${file.name}: line ${parsed.errorLine}: ${parsed.error} (the robot would reject it too)`);
        }
        continue;
      }
      try {
        logs.push(slt.parse(text, file.name));
      } catch (error) {
        problems.push(error.message);
      }
    }
    state.mystery = null;
    $('tune-cal-problems').textContent = problems.join(' ');
    $('tune-cal-problems').hidden = !problems.length;
    if (logs.length) calibrate(logs);
  }

  function demoLogs() {
    if (state.running) return;
    // three runs of a robot whose world only this page knows, logged the way the robot logs
    const logs = ['tour', 'laps', 'sprints'].map((routine, i) =>
      T.recordedLog({ id: routine, routine, world: MYSTERY_WORLD }, T.defaultConfig(), 61 + i, 970001 + i));
    state.mystery = MYSTERY_WORLD;
    $('tune-cal-problems').hidden = true;
    calibrate(logs);
  }

  function calibrate(logs) {
    ctx.setPlaying(false);
    $('tune-cal').hidden = false;
    $('tune-cal-status').textContent = `Reading ${logs.length} log${logs.length === 1 ? '' : 's'}…`;
    $('tune-cal-table').replaceChildren();
    $('tune-use-world-btn').hidden = true;
    let runs = 0;
    const stopFn = pump(T.calibrateWorld(logs), () => {
      runs++;
      $('tune-cal-status').textContent = `Matching the sensors' delay by simulating your robot's paths (${runs} runs)…`;
    }, (result) => {
      setRunning(null);
      ctx.setPlaying(true);
      state.robot = result;
      renderCalibration(result);
    });
    setRunning('calibrate', stopFn);
  }

  function renderCalibration(r) {
    const c = r.calibration;
    const status = $('tune-cal-status');
    if (!c.ok) {
      status.textContent = c.problems.join(' ') || 'Not enough MCL data in these logs: record a few minutes of driving with the localizer running.';
      return;
    }
    status.textContent = `${c.logs} log${c.logs === 1 ? '' : 's'}, ${c.readings.total} sensor readings, ` +
      `${r.trajectories.length} path${r.trajectories.length === 1 ? '' : 's'} to replay.`;
    const truth = state.mystery;
    const rows = [el('tr', {}, ['Measured', 'Value', truth ? 'Really' : 'Means'].map((t) => el('th', { text: t })))];
    const row = (label, value, means, really) => rows.push(el('tr', {}, [el('td', { text: label }), el('td', { text: value }),
      el('td', { text: truth ? really : means })]));
    if (c.noise.ok) {
      const at = (d) => fmt(Math.max(c.noise.minSigmaIn, c.noise.sigmaFraction * d), 1);
      row('Sensor noise', `${fmt(c.noise.simNoiseScale, 2)}× the sim's default`,
        `±${at(12)} / ±${at(36)} / ±${at(60)} in at 12/36/60 in`, `${truth && truth.sensorNoiseScale}×`);
    }
    if (c.outliers.ok) {
      // the world's rate: some blocked readings land too near the wall to tell apart from noise
      row('Blocked readings', `${fmt(c.outliers.simPct, 1)}%`,
        `${fmt(100 * c.outliers.rate, 1)}% clearly off the wall, ${fmt(100 * c.outliers.shortShare, 0)}% of those short`,
        `${truth && truth.sensorOutlierPct}%`);
    }
    if (c.dropouts.ok) row('Missing readings', `${fmt(100 * c.dropouts.rate, 1)}%`, 'with a wall in range', `${truth && truth.sensorDropoutPct}%`);
    if (r.latency.ok) {
      row('Sensor delay', `${fmt(r.latency.delayMs, 0)} ± ${fmt(r.latency.stderrMs, 0)} ms`,
        'matched by simulating your paths', `${truth && truth.sensorDelayMs} ms`);
    } else if (c.latency.ok) {
      row('Sensor delay', `≥ ${fmt(c.latency.measuredMs, 0)} ms`, 'couldn\'t match it; a lower bound', '');
    }
    if (c.odometry.ok) {
      const v = c.odometry.verticalReadsLongPct;
      const h = c.odometry.horizontalReadsLongPct;
      const wheel = (pct, scale) => (Number.isFinite(pct) ? `${pct >= 0 ? '+' : ''}${fmt(pct, 1)}%` : '–') +
        (Number.isFinite(scale) && Math.abs(pct) >= 0.5 ? ` (diameter ×${fmt(scale, 3)})` : '');
      row('Tracking wheels read', `${wheel(v, c.odometry.verticalDiameterScale)} / ${wheel(h, c.odometry.horizontalDiameterScale)}`,
        'vertical / horizontal; long is +', `+${truth && truth.wheelDiameterErrorPct}%`);
    }
    for (const s of c.sensors) {
      const flag = Math.abs(s.medianIn) >= 0.3;
      row(`${kSensorNames[s.index] || `Sensor ${s.index}`} sensor bias`, `${s.medianIn >= 0 ? '+' : ''}${fmt(s.medianIn, 2)} in`,
        flag ? `reads ${s.medianIn > 0 ? 'long' : 'short'}: check its mount (off by about ${fmt(2 * Math.abs(s.medianIn), 1)} in)` : 'fine',
        '0 in');
    }
    if (Number.isFinite(c.cpu.mclPct)) {
      row('Localizer CPU', `${fmt(c.cpu.mclUpdateUs.mean, 0)} µs per update`, `${fmt(c.cpu.mclPct, 1)}% of the brain, at most`, '');
    }
    if (Number.isFinite(c.cpu.loggerPct)) row('Logger CPU', `${fmt(c.cpu.loggerPct, 1)}%`, 'sampling and formatting, at most', '');
    $('tune-cal-table').replaceChildren(...rows);
    $('tune-use-world-btn').hidden = false;
  }

  function useWorld() {
    const r = state.robot;
    if (!r) return;
    ctx.applyWorld(r.world);
    // the search starts from what the robot was really running
    ctx.setMclFlat(r.start);
    state.robotCases = T.robotCases(r.world, r.trajectories);
    state.suite = 'robot';
    ctx.setSegment($('tune-suite'), 'robot');
    renderCases();
    flash($('tune-cal-status'), 'The simulated robot now has these errors (Robot tab), and the MCL tab has the settings the robot ran. ' +
      'Search on "Your robot" above.');
  }

  root.SIM.tuneview = { build, drawHistory };
})();
