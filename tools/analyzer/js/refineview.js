/*
 * SapphireLib telemetry analyzer: refineview.js
 *
 * The Refine tab: fine-tuning the drivetrain's PIDs from every loaded log
 * (refine.js), and the history of what each run ran and what changed it.
 * The analyzer proposes; accepting a proposal and saving TUNE.CFG is what
 * reaches the robot, and the next log says whether it worked.
 *
 * Works across every loaded run, not just the one picked at the top: the
 * more steps it has, the surer it is, and the history is about runs.
 *
 * Browser only: SA.views.refine.
 *
 * Team 96671H: Hitmen
 */
(function () {
  'use strict';

  const { h, clear, fmt, sig } = SA.ui;
  const { refine: RF, tunefile: TF, model: M } = SA;

  const pct = (fraction) => (Number.isFinite(fraction) ? `${Math.round(100 * fraction)}%` : '-');
  const sec = (s) => (Number.isFinite(s) ? `${s.toFixed(2)}s` : '-');
  const gainsText = (g) => (g ? `${sig(g.kP)} / ${sig(g.kD)}${g.kI ? ` / kI ${sig(g.kI)}` : ''}` : '-');

  /** A proposal's identity: new logs make a new proposal, which starts from its own default. */
  function keyOf(r) {
    if (!r.ok) return `${r.controller.pid}:none`;
    return [r.controller.pid, r.saves, r.proposed.kP, r.proposed.kD, r.identified.model.kA, r.identified.delayS].join(':');
  }

  const view = {
    accepted: new Map(), // proposal key -> accepted? (remembered across re-renders of the same proposal)

    render(section, app) {
      const runs = app.runs.slice();
      section.append(h('section', { class: 'panel' },
        h('header', null, h('h3', null, 'Refine from real motions'),
          h('span', { class: 'hint', title: 'Every logged motion replayed through the axis model; the model is fitted to ' +
            'what the robot did, and each controller designed again from it' },
          `${runs.length} run${runs.length === 1 ? '' : 's'} loaded · nothing changes until you save TUNE.CFG`))));

      const profile = app.tuneProfile ? app.tuneProfile.profile : TF.emptyProfile();
      const results = RF.CONTROLLERS.map((c) => RF.refine(runs, c, { profile }));
      for (const r of results) {
        if (!view.accepted.has(keyOf(r))) view.accepted.set(keyOf(r), r.ok && r.saves === 'gains');
      }

      const charts = [];
      section._cleanup = () => charts.forEach((c) => c.destroy());
      const grid = h('div', { class: 'two-col' });
      section.append(grid);
      for (const r of results) grid.append(proposalPanel(r, charts));
      const filePanel = h('section', { class: 'panel' });
      section.append(filePanel);
      const renderFile = () => {
        clear(filePanel);
        tuneFilePanel(filePanel, app, results, renderFile);
      };
      section.addEventListener('accepted', renderFile);
      renderFile();
      section.append(historyPanel(runs));
    },
  };

  // --- One controller's proposal ----------------------------------------------------------------

  function proposalPanel(r, charts) {
    const c = r.controller;
    const status = !r.ok ? h('span', { class: 'pill' }, 'Not enough data')
      : r.change === 'retune' ? h('span', { class: 'pill bad' }, 'Retune proposed')
        : h('span', { class: 'pill ok' }, 'No change');
    const panel = h('section', { class: 'panel' },
      h('header', null, h('h3', null, `${c.name}`), status));
    if (!r.ok) {
      panel.append(h('p', { class: 'muted' }, r.why));
      return panel;
    }
    panel.append(h('ol', { class: 'note reasons' }, r.reasons.map((text) => h('li', null, text))));

    const id = r.identified;
    const base = r.base.model;
    const modelText = (m, delayS) => `kS ${sig(m.kS)} kV ${sig(m.kV)} kA ${sig(m.kA)}, delay ${Math.round(1000 * delayS)}ms`;
    panel.append(h('dl', { class: 'kv' },
      h('dt', null, 'Gains (kP / kD)'), h('dd', null, r.change === 'retune'
        ? `${gainsText(r.current)} → ${gainsText(r.proposed)}` : `${gainsText(r.current)} (unchanged)`),
      h('dt', null, 'Model'), h('dd', null, r.modelOff
        ? `${modelText(base, r.base.delayS)} → ${modelText(id.model, id.delayS)}`
        : `${modelText(base, r.base.delayS)} ${r.unexplained ? '(doesn\'t explain these steps)' : '(fits them as measured)'}`),
      h('dt', null, 'From'), h('dd', null, r.base.source),
      h('dt', null, 'Steps used'), h('dd', null, `${r.responses.length} (the most recent, across the loaded runs)`)));

    const p = r.predicted;
    panel.append(h('div', { class: 'table-wrap' }, h('table', { class: 'data-table' },
      h('thead', null, h('tr', null, h('th', null, 'On these steps'), h('th', { class: 'num' }, 'Overshoot'),
        h('th', { class: 'num' }, 'Settles in'))),
      h('tbody', null,
        h('tr', null, h('td', null, 'Recorded'), h('td', { class: 'num' }, pct(r.measured.overshootFraction)),
          h('td', { class: 'num' }, sec(r.measured.settleS))),
        h('tr', null, h('td', null, 'What the old model promised'), h('td', { class: 'num' }, pct(p.promised.overshootFraction)),
          h('td', { class: 'num' }, sec(p.promised.settleS))),
        h('tr', null, h('td', null, 'Refined model, gains now'), h('td', { class: 'num' }, pct(p.now.overshootFraction)),
          h('td', { class: 'num' }, sec(p.now.settleS))),
        h('tr', null, h('td', null, 'Refined model, proposed gains'),
          h('td', { class: 'num' }, pct(p.proposed.overshootFraction)), h('td', { class: 'num' }, sec(p.proposed.settleS)))))));

    // one step at a time: what it did, and the replays (the proposed gains only when there are some)
    const select = h('select', { class: 'select', 'aria-label': 'Step' });
    r.responses.forEach((resp, i) => select.append(h('option', { value: String(i) },
      `${resp.file} at ${resp.start.toFixed(1)}s: ${fmt(resp.e0, 1)}${c.unit}`)));
    // the step that overshot most: the one that says most about what's wrong
    select.value = String(r.responses.reduce((best, resp, i, all) =>
      (resp.metrics.overshootFraction > all[best].metrics.overshootFraction ? i : best), 0));
    const chartBox = h('div');
    panel.append(h('div', { class: 'field-row' }, select), chartBox);
    let chart = null;
    const draw = () => {
      if (chart) {
        chart.destroy();
        charts.splice(charts.indexOf(chart), 1);
      }
      clear(chartBox);
      const resp = r.responses[Number(select.value)];
      const old = RF.replay(resp, base, r.base.delayS, c);
      const fitted = RF.replay(resp, id.model, id.delayS, c);
      const series = [
        { label: 'recorded', color: '--ink-2', t: resp.t, y: resp.err },
        { label: 'old model', color: '--series-2', t: old.t, y: old.err },
        { label: 'refined model', color: '--series-1', t: fitted.t, y: fitted.err },
      ];
      if (r.change === 'retune') {
        const next = RF.replay(resp, id.model, id.delayS, c, r.proposed);
        series.push({ label: 'proposed gains', color: '--series-3', t: next.t, y: next.err });
      }
      chart = new SA.charts.TimeChart(chartBox, {
        title: `Error, ${c.unit}`,
        height: 190,
        series,
        guides: [{ y: c.threshold, label: 'exit threshold' }].concat(c.signed ? [{ y: -c.threshold }] : []),
      });
      const end = Math.max(resp.t[resp.t.length - 1], fitted.t[fitted.t.length - 1]);
      const group = new SA.charts.ChartGroup({ full: [0, end], formatTime: (x) => `${x.toFixed(2)}s` });
      group.add(chart);
      group.redraw();
      charts.push(chart);
    };
    select.addEventListener('change', draw);
    // after layout, so the chart knows its width; a timer too, for views where frames are starved
    let drawn = false;
    const first = () => {
      if (drawn) return;
      drawn = true;
      draw();
    };
    requestAnimationFrame(first);
    setTimeout(first, 60);

    if (r.saves) {
      const checkbox = h('input', { type: 'checkbox' });
      checkbox.checked = view.accepted.get(keyOf(r));
      checkbox.addEventListener('change', () => {
        view.accepted.set(keyOf(r), checkbox.checked);
        panel.dispatchEvent(new CustomEvent('accepted', { bubbles: true }));
      });
      panel.append(h('label', { class: 'accept' }, checkbox, r.saves === 'gains'
        ? ` Put these ${c.name.toLowerCase()} gains and the refined ${c.axis} model in TUNE.CFG`
        : ` Put the refined ${c.axis} model in TUNE.CFG (the gains stay; driving in velocity mode uses the model)`));
    }
    return panel;
  }

  // --- TUNE.CFG -------------------------------------------------------------------------------------

  function tuneFilePanel(panel, app, results, rerender) {
    const loaded = app.tuneProfile;
    const profile = loaded ? loaded.profile : TF.emptyProfile();
    const chosen = results.filter((r) => r.ok && view.accepted.get(keyOf(r)));
    const revised = TF.revise(profile, RF.tuneChanges(chosen),
      { source: 'analyzer', note: chosen.length ? `PID refinement: ${chosen.map((r) => r.controller.name).join(', ')}` : null });

    const open = h('input', { type: 'file', accept: '.cfg,.CFG', hidden: true });
    open.addEventListener('change', async () => {
      const file = open.files[0];
      if (!file) return;
      app.setTuneProfile(file.name, await file.text());
      rerender();
    });
    panel.append(h('header', null, h('h3', null, 'TUNE.CFG'),
      h('span', { class: 'hint' }, 'loaded from the SD card at startup')));
    panel.append(h('p', { class: 'note' }, loaded
      ? `${loaded.name}: revision ${profile.revision}${profile.note ? `, "${profile.note}"` : ''}. Saving keeps its other ` +
        'lines (the simulator\'s MCL settings), adds one to the revision and notes the change at the bottom.'
      : 'None open: saving starts a new one. Open the one on the robot\'s card to keep its other lines (the simulator\'s ' +
        'MCL settings) and its history.'));
    if (app.tuneProfileError) panel.append(h('p', { class: 'banner' }, app.tuneProfileError));

    const changes = revised.changes;
    panel.append(changes.length
      ? h('div', { class: 'table-wrap' }, h('table', { class: 'data-table' },
        h('thead', null, h('tr', null, h('th', null, 'Line'), h('th', null, 'Was'), h('th', null, 'Becomes'))),
        h('tbody', null, changes.map((ch) => h('tr', null, h('td', { class: 'data' }, ch.key),
          h('td', { class: 'data' }, ch.from || '(the code\'s)'), h('td', { class: 'data' }, ch.to || '(the code\'s)'))))))
      : h('p', { class: 'muted' }, chosen.length ? 'TUNE.CFG already has these.' : 'Nothing accepted yet.'));

    const save = h('button', { class: 'btn primary', type: 'button', disabled: !changes.length }, 'Save TUNE.CFG');
    save.addEventListener('click', () => {
      const text = TF.format(revised.profile);
      const link = h('a', { href: URL.createObjectURL(new Blob([text], { type: 'text/plain' })), download: 'TUNE.CFG' });
      document.body.append(link);
      link.click();
      link.remove();
      app.setTuneProfile('TUNE.CFG', text);
      rerender();
    });
    const openBtn = h('button', { class: 'btn', type: 'button', onclick: () => open.click() }, 'Open TUNE.CFG…');
    panel.append(h('div', { class: 'field-row' }, openBtn, save, open));
    if (profile.changelog.length) {
      panel.append(h('details', null, h('summary', null, 'Its history'),
        h('ul', { class: 'note changelog' }, profile.changelog.slice(0, 12).map((line) => h('li', { class: 'data' }, line)))));
    }
    panel.append(h('p', { class: 'muted' }, 'Copy it to the sl folder on the SD card; the Home page shows the revision loaded.'));
  }

  // --- History --------------------------------------------------------------------------------------

  function historyPanel(runs) {
    const rows = RF.history(runs);
    const panel = h('section', { class: 'panel' },
      h('header', null, h('h3', null, 'Run by run'), h('span', { class: 'hint' }, 'gains, results, and what changed them')));
    if (!rows.length) {
      panel.append(h('p', { class: 'muted' }, 'No runs loaded.'));
      return panel;
    }
    const controllerCell = (row, pid) => {
      const c = row.controllers.find((x) => x.pid === pid);
      if (!c) return h('td', { class: 'muted' }, '-');
      const last = c.gains[c.gains.length - 1];
      const stats = c.responses ? `${c.responses} steps, ${pct(c.overshoot)} over, ${sec(c.settleS)}` : 'no steps';
      return h('td', null, h('div', { class: 'data' }, gainsText(last)),
        h('div', { class: 'muted' }, stats + (c.timeouts ? `, ${c.timeouts} timed out` : '')));
    };
    const body = [];
    for (const row of rows) {
      const tune = row.tune.status === 'loaded' ? `r${row.tune.rev}` : row.tune.status === 'rejected' ? 'REJECTED'
        : row.tune.status === null ? '(not logged)' : 'none';
      const mcl = row.mcl ? `${fmt(row.mcl.correctionIn, 2)} in, ${pct(row.mcl.correctingShare)} correcting` +
        (Number.isFinite(row.mcl.updateUs) ? `, ${Math.round(row.mcl.updateUs)} µs` : '') : '-';
      body.push(h('tr', null,
        h('td', null, h('div', { class: 'data' }, row.file), h('div', { class: 'muted' }, row.sessions.join(', ') || 'no enabled time')),
        h('td', { class: 'data' }, tune),
        controllerCell(row, 'turn'),
        controllerCell(row, 'drive'),
        h('td', null, mcl),
        h('td', { class: 'num' }, row.cpu ? `${fmt(row.cpu.loggerPct, 1)}%` : '-')));
      if (row.changes.length) {
        body.push(h('tr', { class: 'changes' }, h('td', { colspan: 6 },
          h('ul', null, row.changes.map((ch) => h('li', null, `${ch.controller}: `,
            h('span', { class: 'data' }, ch.from ? `${gainsText(ch.from)} → ${gainsText(ch.to)}` : `→ ${gainsText(ch.to)}`),
            ` (${ch.source})`))))));
      }
    }
    panel.append(h('div', { class: 'table-wrap' }, h('table', { class: 'data-table history' },
      h('thead', null, h('tr', null, ['Run', 'TUNE.CFG', 'Turn kP / kD', 'Drive kP / kD', 'Localizer', 'Logger CPU']
        .map((t, i) => h('th', { class: i === 5 ? 'num' : null }, t)))),
      h('tbody', null, body))));
    panel.append(h('p', { class: 'muted' }, 'Steps: median overshoot and settle time. Localizer: mean drift corrected, ' +
      'share of updates correcting, update time. Logger CPU: an upper bound.'));
    return panel;
  }

  SA.views.refine = view;
})();
