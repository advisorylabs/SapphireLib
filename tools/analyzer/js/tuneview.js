/*
 * SapphireLib telemetry analyzer — tuneview.js
 *
 * The Tune tab: tuning from a log. The analyzer can't drive the robot, so it
 * works from what the robot already recorded — Auto-Tune's characterization
 * runs (char.* channels), or ordinary driving in a match — and does the rest
 * the way the robot would (tuning.js, with model.js's ports of the robot's
 * math): fit the model, design each controller, replay the log's real targets
 * through the model with the gains the robot had and with the new ones, and
 * write the C++ to paste. Then flash, and the next log checks the result.
 *
 * Browser only: SA.views.tune.
 *
 * Team 96671H — Hitmen
 */
(function () {
  'use strict';

  const { h, clear, fmt, sig } = SA.ui;
  const { tuning: T, model: M, slt } = SA;

  function numberInput(id, value, step, onChange, label) {
    const input = h('input', { id, type: 'number', step: String(step), value: String(value) });
    input.addEventListener('change', () => {
      const v = Number(input.value);
      if (Number.isFinite(v)) onChange(v);
    });
    return h('label', { class: 'field', for: id }, label, input);
  }

  /** Every Auto-Tune run of `system`'s axis across all loaded logs. */
  function autotuneSources(app, system) {
    const out = [];
    app.runs.forEach((run, runIndex) => {
      const other = T.systems(run).find((s) => s.id === system.id);
      if (!other) return;
      for (const r of other.autotuneRuns) {
        out.push({ key: `auto:${runIndex}:${r.index}`, label: `Auto-Tune run ${r.index + 1} in ${run.files[0]}`,
          run, system: other, autotune: r });
      }
    });
    return out;
  }

  const view = {
    choice: null, // remembered across re-renders: { systemId, sourceKey }

    render(section, app) {
      const systems = T.systems(app.run);
      section.append(h('section', { class: 'panel' },
        h('h3', null, 'Tuning from a log'),
        h('p', null, 'The analyzer can\'t drive the robot, and doesn\'t need to: a log already holds what ' +
          'Auto-Tune measures. Its characterization runs (the PID page\'s Auto-Tune, logged as char.*) are ' +
          'refit here with the robot\'s own math, and ordinary match driving works too — the volts every ' +
          'system was sent, next to where it went. From the model, each controller is designed the way the ' +
          'robot designs it, then judged by replaying this log\'s real targets through the model: with the ' +
          'gains the robot had, and with the new ones. Copy the C++, flash, and the next log shows whether ' +
          'it worked.'),
        h('p', { class: 'muted' }, 'For fresh data: select a controller on the robot\'s PID page and tap ' +
          'Auto-Tune (Lift measures only the lift) with the SD card in, then open that log here.')));
      section.lastChild.querySelectorAll('p').forEach((p) => p.classList.add('note'));
      if (!systems.length) {
        section.append(h('p', { class: 'muted' }, 'Nothing tunable in this log: it needs chassis + odom channels, ' +
          'char.* Auto-Tune runs, or a mechanism\'s X.act channel.'));
        return;
      }

      const choice = view.choice || {};
      let system = systems.find((s) => s.id === choice.systemId) ||
        systems.find((s) => s.kind === 'mechanism') || systems[0];
      const systemPicker = h('div', { class: 'segments' });
      const sourceSel = h('select', { class: 'select', id: 'tune-source', 'aria-label': 'Data to fit' });
      const gravitySel = h('select', { class: 'select', id: 'tune-gravity', 'aria-label': 'Gravity' },
        h('option', { value: 'constant' }, 'Gravity: constant (elevator)'),
        h('option', { value: 'cosine' }, 'Gravity: cosine (arm)'),
        h('option', { value: 'none' }, 'No gravity'));
      const horizontal = h('input', { id: 'tune-horizontal', type: 'number', step: '1', value: '0' });
      const horizontalField = h('label', { class: 'field', for: 'tune-horizontal' }, 'Arm level at', horizontal);
      const body = h('div', { class: 'tune-grid' });
      section.append(h('section', { class: 'panel' },
        h('div', { class: 'field-row' }, systemPicker, sourceSel, gravitySel, horizontalField)),
      body);

      const renderSystems = () => {
        clear(systemPicker);
        for (const s of systems) {
          systemPicker.append(h('button', { type: 'button', 'aria-pressed': String(s === system),
            onclick: () => {
              system = s;
              renderSystems();
              renderSources();
            } }, s.label));
        }
      };

      let sources = [];
      const renderSources = () => {
        sources = autotuneSources(app, system);
        sources.push({ key: 'passive', label: `Driving in ${app.session() ? app.session().label : 'this run'}`,
          run: app.run, system });
        clear(sourceSel);
        for (const s of sources) sourceSel.append(h('option', { value: s.key }, s.label));
        const remembered = sources.find((s) => s.key === choice.sourceKey && choice.systemId === system.id);
        sourceSel.value = remembered ? remembered.key : sources[0].key;
        gravitySel.hidden = system.kind !== 'mechanism';
        horizontalField.hidden = system.kind !== 'mechanism' || gravitySel.value !== 'cosine';
        renderBody();
      };
      sourceSel.addEventListener('change', () => renderBody());
      gravitySel.addEventListener('change', () => {
        horizontalField.hidden = gravitySel.value !== 'cosine';
        renderBody();
      });
      horizontal.addEventListener('change', () => renderBody());

      let charts = [];
      const renderBody = () => {
        for (const c of charts) c.destroy();
        charts = [];
        clear(body);
        const source = sources.find((s) => s.key === sourceSel.value) || sources[0];
        view.choice = { systemId: system.id, sourceKey: source.key };
        const gravity = system.kind === 'mechanism'
          ? M.gravityShape(gravitySel.value, Number(horizontal.value) || 0, 1) : M.gravityShape('none');
        const fit = fitFor(app, system, source, gravity);
        const left = h('div', { class: 'side-stack' });
        const right = h('div', { class: 'side-stack' });
        body.append(left, right);
        left.append(modelPanel(system, source, fit));
        if (fit.model && M.ffValid(fit.model.motion)) {
          left.append(modelCheckPanel(app, system, source, fit, charts));
          for (const controller of system.controllers) {
            right.append(controllerPanel(app, system, source, fit, controller, charts));
          }
          if (!system.controllers.length) {
            right.append(h('section', { class: 'panel' }, h('p', { class: 'muted' },
              'No controller in this log runs on this axis (Strafe has none on this robot), so there\'s ' +
              'nothing to design — but the model is used by the driver stick mode; see the code below.')));
          }
          right.append(codePanel(app, systems, system, source, fit));
        }
      };

      renderSystems();
      renderSources();
    },
  };

  function fitFor(app, system, source, gravity) {
    try {
      if (source.autotune) return T.fitAutotune(source.system, source.autotune, gravity);
      const [t0, t1] = app.range;
      const samples = system.kind === 'mechanism' ? T.mechanismSamples(app.run, system, t0, t1)
        : T.driveSamples(app.run, system, t0, t1);
      return T.fitPassive(system, samples, gravity);
    } catch (error) {
      return { ok: false, model: null, rSquared: 0, samplesUsed: 0, delayS: 0, source: source.label, error };
    }
  }

  function modelPanel(system, source, fit) {
    const cells = [];
    const cell = (k, v, r) => h('div', { class: 'cell' }, h('div', { class: 'k' }, k), h('div', { class: 'v' }, v),
      r ? h('div', { class: 'r' }, r) : null);
    const reported = source.autotune && source.autotune.reported;
    const m = fit.model;
    if (m) {
      cells.push(cell('kS (V)', sig(m.motion.kS), reported ? `robot: ${sig(reported.kS)}` : null));
      cells.push(cell(`kV (V per ${system.unit}/s)`, sig(m.motion.kV), reported ? `robot: ${sig(reported.kV)}` : null));
      cells.push(cell(`kA (V per ${system.unit}/s²)`, sig(m.motion.kA), reported ? `robot: ${sig(reported.kA)}` : null));
      if (m.gravity.kind !== 'none') cells.push(cell('kG (V)', sig(m.kG), reported ? `robot: ${sig(reported.kG)}` : null));
      cells.push(cell('Delay', `${Math.round(fit.delayS * 1000)} ms`, reported ? `robot: ${Math.round(reported.delayS * 1000)} ms` : null));
      cells.push(cell('R²', fit.rSquared.toFixed(3), `${fit.samplesUsed} intervals`));
    }
    const notes = [];
    if (!fit.ok) {
      notes.push(h('p', null, SA.ui.severityBadge('warning'), ' ', fit.model && M.ffValid(fit.model.motion)
        ? 'A weak fit (R² under the robot\'s 0.8 floor, or too few moving samples): treat these numbers as rough.'
        : 'No usable model from this data: the axis barely moved, or the volts and motion don\'t line up ' +
          '(a reversed sensor?). Try an Auto-Tune run.'));
    }
    if (source.key === 'passive') {
      notes.push(h('p', null, system.kind === 'mechanism'
        ? 'Fitted from match driving. If what it carries changes weight (game pieces), kG is the average ' +
          'load; if its motors overheated, fit a range before they did (pick a session, or Practice).'
        : 'Fitted from match driving. Collisions, wheel slip and saturated autonomous volts all blur it; an ' +
          'Auto-Tune run is cleaner.'));
    }
    if (reported) {
      notes.push(h('p', { class: 'muted' }, 'The robot fitted the same run on-board; the "robot" numbers are ' +
        'from its tune event. Logged values are rounded, so the last digit can differ.'));
    }
    return h('section', { class: 'panel' },
      h('header', null, h('h3', null, `${system.label} model`), h('span', { class: 'hint' }, fit.source || source.label)),
      h('div', { class: 'model-grid' }, cells), h('div', { class: 'note', style: { marginTop: '8px' } }, notes));
  }

  /** Measured position against the model's prediction from the same volts. */
  function modelCheckPanel(app, system, source, fit, charts) {
    const box = h('div');
    const panel = h('section', { class: 'panel' },
      h('header', null, h('h3', null, 'Does the model match?'),
        h('span', { class: 'hint' }, 'The recorded volts played through the model, against where it really went.')),
      box);
    let samples;
    if (source.autotune) {
      // The longest segment of the run, on its own time base.
      const runs = source.autotune.data.ramps.concat(source.autotune.data.steps);
      const longest = runs.reduce((a, b) => (b.length > a.length ? b : a), runs[0]);
      samples = longest.map((s) => ({ t: s.timeMs / 1000, volts: s.volts, position: s.position }));
    } else {
      const [t0, t1] = app.range;
      const all = system.kind === 'mechanism' ? T.mechanismSamples(app.run, system, t0, t1)
        : T.driveSamples(app.run, system, t0, t1);
      // Twenty seconds with the most motion.
      let best = 0;
      let bestStart = 0;
      for (let i = 0; i < all.length; i += 50) {
        let travel = 0;
        for (let j = i + 1; j < all.length && all[j].t - all[i].t < 20; ++j) {
          if (Number.isFinite(all[j].position) && Number.isFinite(all[j - 1].position)) {
            travel += Math.abs(all[j].position - all[j - 1].position);
          }
        }
        if (travel > best) {
          best = travel;
          bestStart = i;
        }
      }
      samples = all.filter((s) => s.t >= all[bestStart].t && s.t < all[bestStart].t + 20);
    }
    if (!samples || samples.length < 10) {
      box.append(h('p', { class: 'muted' }, 'Not enough data to check.'));
      return panel;
    }
    const predicted = M.replayOpenLoop(fit.model, fit.delayS, samples, 0.1);
    const t = Float64Array.from(samples, (s) => s.t);
    requestAnimationFrame(() => {
      const group = new SA.charts.ChartGroup({ full: [t[0], t[t.length - 1]],
        formatTime: (x) => (source.autotune ? `${x.toFixed(1)}s` : app.shortLabel(x)) });
      const chart = new SA.charts.TimeChart(box, {
        title: `Position (${system.unit})`,
        height: 170,
        series: [
          { label: 'measured', color: '--series-1', t, y: Float64Array.from(samples, (s) => s.position) },
          { label: 'model', color: '--series-2', t, y: predicted },
        ],
      });
      group.add(chart);
      group.redraw();
      charts.push(chart);
    });
    return panel;
  }

  function controllerPanel(app, system, source, fit, controller, charts) {
    const log = app.run;
    const [t0, t1] = app.range;
    const current = T.gainsAt(log, controller, t0) || { kP: 0, kI: 0, kD: 0 };
    const spec = Object.assign({}, controller.spec);
    const isMech = system.kind === 'mechanism';
    const inferred = isMech ? T.inferredGravityVolts(log, system, t0, t1) : NaN;
    const currentGravity = Number.isFinite(inferred) ? inferred : 0;
    let design = T.design(fit, controller, spec);
    const custom = { gains: Object.assign({}, design.ok ? design.gains : current),
      gravity: isMech ? fit.model.kG : 0 };

    const table = h('div', { class: 'table-wrap' });
    const chartBox = h('div');
    const metricsBox = h('div', { class: 'table-wrap' });
    const stepSel = h('select', { class: 'select', id: `tune-step-${controller.name}`, 'aria-label': 'Response to replay' });
    const panel = h('section', { class: 'panel' },
      h('header', null, h('h3', null, `${controller.name} controller`),
        h('span', { class: 'hint' }, `the "${controller.pid}" PID · loop ${Math.round((controller.loopS || 0.01) * 1000)} ms`)),
      h('div', { class: 'field-row' },
        numberInput(`tune-settle-${controller.name}`, spec.settleTimeS, 0.05, (v) => {
          spec.settleTimeS = v;
          redesign();
        }, 'Settle time (s)'),
        numberInput(`tune-zeta-${controller.name}`, spec.dampingRatio, 0.1, (v) => {
          spec.dampingRatio = v;
          redesign();
        }, 'Damping (1 = no overshoot)'),
        numberInput(`tune-pm-${controller.name}`, spec.minPhaseMarginDeg, 5, (v) => {
          spec.minPhaseMarginDeg = v;
          redesign();
        }, 'Min phase margin (°)')),
      h('div', { style: { height: '10px' } }), table,
      isMech ? null : h('div', { class: 'field-row', style: { marginTop: '10px' } }, stepSel),
      h('div', { style: { height: '8px' } }), chartBox, metricsBox);

    const gainCell = (row, key) => {
      if (row !== 'custom') return h('td', { class: 'num' }, sig((row === 'current' ? current : design.gains)[key]));
      const input = h('input', { type: 'number', step: 'any', value: String(Number(custom.gains[key].toPrecision(4))),
        id: `tune-${controller.name}-${key}`, 'aria-label': `Custom ${key}`, style: { width: '90px' } });
      input.addEventListener('change', () => {
        const v = Number(input.value);
        if (Number.isFinite(v) && v >= 0) {
          custom.gains[key] = v;
          replay();
        }
      });
      return h('td', { class: 'num' }, input);
    };
    const renderTable = () => {
      clear(table);
      const pm = (g) => M.phaseMarginDeg(fit.model.motion, g, fit.delayS + 0.5 * Math.max(0, (controller.loopS || 0.01) - 0.01));
      const band = (g) => (g.kP > 0 ? fit.model.motion.kS / g.kP : Infinity);
      const gravityCell = (row) => {
        if (!isMech) return null;
        if (row !== 'custom') {
          return h('td', { class: 'num' }, row === 'current' ? `${fmt(currentGravity, 2)}${Number.isFinite(inferred) ? '' : '?'}`
            : fmt(fit.model.kG, 2));
        }
        const input = h('input', { type: 'number', step: '0.1', value: String(Number(custom.gravity.toFixed(3))),
          id: `tune-${controller.name}-gravity`, 'aria-label': 'Custom gravity volts', style: { width: '80px' } });
        input.addEventListener('change', () => {
          const v = Number(input.value);
          if (Number.isFinite(v)) {
            custom.gravity = v;
            replay();
          }
        });
        return h('td', { class: 'num' }, input);
      };
      const row = (name, label, gains) => h('tr', null, h('td', null, label),
        gainCell(name, 'kP'), gainCell(name, 'kI'), gainCell(name, 'kD'), gravityCell(name),
        h('td', { class: 'num' }, `${fmt(pm(gains), 0)}°`),
        h('td', { class: 'num' }, `±${sig(band(gains), 2)}`));
      table.append(h('table', { class: 'data-table' },
        h('thead', null, h('tr', null, h('th', null, ''), h('th', { class: 'num' }, 'kP'), h('th', { class: 'num' }, 'kI'),
          h('th', { class: 'num' }, 'kD'), isMech ? h('th', { class: 'num' }, 'gravity V') : null,
          h('th', { class: 'num' }, 'phase margin'), h('th', { class: 'num' }, 'friction band'))),
        h('tbody', null,
          row('current', 'On the robot', current),
          design.ok ? row('designed', 'Designed', design.gains) : null,
          row('custom', 'Try your own', custom.gains))),
      design.ok ? h('p', { class: 'muted', style: { margin: '6px 0 0' } },
        `Designed to settle in ${design.settleTimeS.toFixed(2)}s` +
        (design.limitedByDelay ? ` (slowed from ${spec.settleTimeS}s: the ${Math.round(fit.delayS * 1000)} ms delay ` +
          'can\'t support faster at this phase margin)' : '') +
        `. Static friction can stop it up to ±${sig(design.staticErrorBound, 2)} short; ` +
        `the exit threshold here is ${controller.threshold}.`) : null);
    };

    const redesign = () => {
      design = T.design(fit, controller, spec);
      if (design.ok) custom.gains = Object.assign({}, design.gains);
      renderTable();
      replay();
    };

    let chart = null;
    const steps = isMech ? [] : T.replaySteps(log, system, fit, controller, { gains: current, t0, t1 });
    if (!isMech) {
      steps.forEach((s, i) => stepSel.append(h('option', { value: String(i) },
        `${app.shortLabel(s.start)}: from ${fmt(s.e0, 1)} ${system.unit}`)));
      if (!steps.length) stepSel.hidden = true;
      stepSel.addEventListener('change', () => replay());
    }

    const replay = () => {
      if (chart) {
        chart.destroy();
        charts.splice(charts.indexOf(chart), 1);
        chart = null;
      }
      clear(chartBox);
      clear(metricsBox);
      if (isMech) {
        const window = { t0, t1 };
        const runs = [
          ['On the robot', current, currentGravity, '--series-3'],
          ...(design.ok ? [['Designed', design.gains, fit.model.kG, '--series-2']] : []),
          ['Try your own', custom.gains, custom.gravity, '--series-7'],
        ].map(([label, gains, gravityVolts, color]) => ({
          label, color, result: T.replayMechanism(log, system, fit, Object.assign({ gains, gravityVolts }, window)),
        })).filter((r) => r.result);
        if (!runs.length) {
          chartBox.append(h('p', { class: 'muted' }, 'No targets to replay in this range.'));
          return;
        }
        const base = runs[0].result;
        chart = new SA.charts.TimeChart(chartBox, {
          title: `What if: this range's targets, replayed through the model (${system.unit})`,
          height: 200,
          series: [
            { label: 'target', color: '--series-1', t: base.t, y: base.target, step: true },
            { label: 'recorded', color: '--axis', t: base.t, y: base.recorded, width: 2 },
            ...runs.map((r) => ({ label: r.label, color: r.color, t: r.result.t, y: r.result.x })),
          ],
        });
        const group = new SA.charts.ChartGroup({ full: [base.t[0], base.t[base.t.length - 1]],
          formatTime: (x, long) => (long ? app.label(x) : app.shortLabel(x)) });
        group.add(chart);
        group.redraw();
        charts.push(chart);
        const better = (values, lowerIsBetter = true) => {
          const best = lowerIsBetter ? Math.min(...values.map(Math.abs)) : Math.max(...values);
          return (v) => ((lowerIsBetter ? Math.abs(v) : v) === best ? 'num metric-better' : 'num');
        };
        const rests = runs.map((r) => r.result.metrics.meanHoldError);
        const within = runs.map((r) => r.result.metrics.withinFraction);
        const restClass = better(rests.filter(Number.isFinite));
        const withinClass = better(within, false);
        metricsBox.append(h('table', { class: 'data-table' },
          h('thead', null, h('tr', null, h('th', null, 'Replay'), h('th', { class: 'num' }, 'Comes to rest'),
            h('th', { class: 'num' }, `Within ±${controller.threshold}`), h('th', { class: 'num' }, 'RMS error'),
            h('th', { class: 'num' }, 'At full volts'))),
          h('tbody', null, runs.map((r) => {
            const m = r.result.metrics;
            return h('tr', null, h('td', null, r.label),
              h('td', { class: restClass(m.meanHoldError) }, Number.isFinite(m.meanHoldError)
                ? `${Math.abs(m.meanHoldError).toFixed(1)} ${m.meanHoldError > 0 ? 'low' : 'high'}` : '—'),
              h('td', { class: withinClass(m.withinFraction) }, `${Math.round(m.withinFraction * 100)}%`),
              h('td', { class: 'num' }, fmt(m.rmsError, 1)),
              h('td', { class: 'num' }, `${Math.round(m.saturatedFraction * 100)}%`));
          }))));
        return;
      }
      const index = Number(stepSel.value) || 0;
      const recordedStep = steps[index];
      if (!recordedStep) {
        chartBox.append(h('p', { class: 'muted' }, controller.pid === 'hold'
          ? 'Heading hold runs all through driver control, following the driver\'s stick, so there\'s no ' +
            'step to replay. It\'s designed from the same turn model as Turn, to a softer spec.'
          : `No ${controller.pid} step responses in this range to replay (pick a match with motions, or ` +
            'the Practice session a Run Test happened in).'));
        return;
      }
      const sims = [
        ['On the robot', current, '--series-3'],
        ...(design.ok ? [['Designed', design.gains, '--series-2']] : []),
        ['Try your own', custom.gains, '--series-7'],
      ].map(([label, gains, color]) => ({
        label, color,
        sim: T.replaySteps(log, system, fit, controller, { gains, t0: recordedStep.start - 1e-6,
          t1: recordedStep.start + 1e-6, maxResponses: 1 })[0],
      })).filter((s) => s.sim);
      chart = new SA.charts.TimeChart(chartBox, {
        title: `What if: this step, from ${fmt(recordedStep.e0, 1)} ${system.unit} (error)`,
        height: 200,
        series: [
          { label: 'recorded', color: '--axis', t: recordedStep.recorded.t, y: recordedStep.recorded.err },
          ...sims.map((s) => ({ label: s.label, color: s.color, t: s.sim.sim.t, y: s.sim.sim.err })),
        ],
        guides: [{ y: controller.threshold, label: 'exit threshold' }, { y: -controller.threshold }],
      });
      const group = new SA.charts.ChartGroup({ full: [0, recordedStep.duration], formatTime: (x) => `${x.toFixed(2)}s` });
      group.add(chart);
      group.redraw();
      charts.push(chart);
      metricsBox.append(h('table', { class: 'data-table' },
        h('thead', null, h('tr', null, h('th', null, 'Replay'), h('th', { class: 'num' }, `Settles within ±${controller.threshold}`),
          h('th', { class: 'num' }, 'Overshoot'), h('th', { class: 'num' }, 'Left over'))),
        h('tbody', null, sims.map((s) => {
          const m = s.sim.metrics;
          return h('tr', null, h('td', null, s.label),
            h('td', { class: 'num' }, m.settled ? `${m.settleS.toFixed(2)}s` : 'not in time'),
            h('td', { class: 'num' }, `${fmt(m.overshoot, 2)} ${system.unit}`),
            h('td', { class: 'num' }, `${fmt(m.finalError, 2)} ${system.unit}`));
        }))));
    };

    renderTable();
    requestAnimationFrame(replay);
    return panel;
  }

  function codePanel(app, systems, system, source, fit) {
    const pre = h('pre', { class: 'code' });
    const sourceName = source.run.files.join(' + ');
    if (system.kind === 'mechanism') {
      const controller = system.controllers[0];
      const d = controller ? T.design(fit, controller) : null;
      pre.textContent = T.mechanismSnippet(system, fit, d && d.ok ? d.gains : { kP: 0, kI: 0, kD: 0 }, fit.model.kG,
        sourceName) + '\n// In macros.cpp: replace kLiftGains and kLiftGravityVolts with these.';
    } else {
      // Every drivetrain axis the same kind of source can fit, for one paste.
      const fits = {};
      const designs = {};
      for (const s of systems.filter((x) => x.kind === 'drive')) {
        let f;
        if (source.autotune) {
          const match = autotuneSources(app, s).find((x) => x.run === source.run);
          f = match ? T.fitAutotune(match.system, match.autotune) : null;
        } else {
          f = s === system ? fit : fitFor(app, s, { key: 'passive', run: app.run, system: s }, M.gravityShape('none'));
        }
        if (!f || !f.ok) continue;
        fits[s.axisName] = f;
        for (const c of s.controllers) designs[c.name] = T.design(f, c);
      }
      pre.textContent = T.driveSnippet(fits, designs, sourceName) +
        '\n// In devices.cpp: drivePIDConfig / turnPIDConfig; the heading-hold and axis-model lines go in initDevices().';
    }
    const copy = h('button', { class: 'btn small', type: 'button' }, 'Copy');
    copy.addEventListener('click', async () => {
      const ok = await SA.ui.copyText(pre.textContent, pre);
      copy.textContent = ok ? 'Copied' : 'Selected: copy it';
      setTimeout(() => {
        copy.textContent = 'Copy';
      }, 2000);
    });
    return h('section', { class: 'panel' },
      h('header', null, h('h3', null, 'Paste into the robot program'), copy),
      pre,
      h('p', { class: 'muted', style: { margin: '8px 0 0' } }, 'Designed gains only (not your custom ones): ' +
        'the numbers above are the evidence for them. Nothing is sent to the robot from here.'));
  }

  SA.views.tune = view;
})();
