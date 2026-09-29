/*
 * SapphireLib telemetry analyzer — app.js
 *
 * The controller: opens logs (file picker, drag and drop, or the demo),
 * groups them into program runs, picks the run and the match being looked
 * at, and hands that range to the views. Also the Overview, PID responses and
 * Motions views; Replay, Charts and Tune live in their own files and register
 * themselves in SA.views.
 *
 * Browser only: window.SA.app.
 *
 * Team 96671H — Hitmen
 */
(function () {
  'use strict';

  const root = typeof self !== 'undefined' ? self : this;
  root.SA = root.SA || {};
  root.SA.views = root.SA.views || {};

  const { h, clear, severityBadge, fmt, sig, heatColor, heatInk } = SA.ui;
  const { slt, analysis: A } = SA;

  const app = {
    runs: [],
    run: null,
    runIndex: 0,
    sessions: [],
    sessionIndex: -1, // -1: the whole run
    range: [0, 1],
    report: null,
    cursor: 0,
    demo: false,
    tab: 'overview',
    listeners: new Map(),
    sections: new Map(),

    on(event, fn) {
      if (!this.listeners.has(event)) this.listeners.set(event, []);
      this.listeners.get(event).push(fn);
    },

    off(event, fn) {
      this.listeners.set(event, (this.listeners.get(event) || []).filter((f) => f !== fn));
    },

    emit(event, ...args) {
      for (const fn of this.listeners.get(event) || []) fn(...args);
    },

    // --- Time ---

    session() {
      return this.sessionIndex >= 0 ? this.sessions[this.sessionIndex] : null;
    },

    /** Full label for a moment: "Match 1, driver 1:13.7". */
    label(t) {
      return A.timeLabel(this.sessions, t);
    },

    /** Short label for chart ticks: "A0:07" in auton, "1:13" in driver. */
    shortLabel(t) {
      for (const s of this.sessions) {
        if (s.auton && t >= s.auton[0] - 1e-3 && t <= s.auton[1] + 1e-3) {
          return `A${A.clock(t - s.auton[0]).replace(/\.\d$/, '')}`;
        }
        if (s.driver && t >= s.driver[0] - 1e-3 && t <= s.driver[1] + 1e-3) {
          return A.clock(t - s.driver[0]).replace(/\.\d$/, '');
        }
        if (s.kind === 'practice' && t >= s.start && t <= s.end) {
          return A.clock(t - s.start).replace(/\.\d$/, '');
        }
      }
      return `${t.toFixed(0)}s`;
    },

    /** Phase bands for charts: autonomous and disabled time shaded. */
    bands() {
      return slt.periods(this.run).filter((p) => p.mode !== 'opcontrol').map((p) => ({
        start: p.start,
        end: p.end,
        fill: p.mode === 'autonomous' ? '--band-auton' : '--band-disabled',
      }));
    },

    findingMarkers() {
      return this.report.findings.filter((f) => f.t !== null && f.severity !== 'info').map((f) => ({
        t: f.t,
        color: f.severity === 'critical' ? '--critical' : '--warning',
        label: f.title,
      }));
    },

    // --- Loading ---

    async loadFiles(files) {
      const texts = [];
      for (const file of files) texts.push({ name: file.name, text: await file.text() });
      this.loadTexts(texts, false);
    },

    loadTexts(items, demo) {
      const logs = [];
      const errors = [];
      for (const { name, text } of items) {
        try {
          logs.push(slt.parse(text, name));
        } catch (error) {
          errors.push(`${name}: ${error.message}`);
        }
      }
      if (logs.length === 0) {
        this.showError(errors.length ? errors : ['No SLT telemetry files were given.']);
        return;
      }
      this.demo = demo;
      this.runs = slt.mergeRuns(logs);
      this.errors = errors;
      // The newest run first: that's usually the match just played.
      this.selectRun(this.runs.length - 1);
    },

    loadDemo() {
      const main = document.getElementById('main');
      clear(main).append(h('section', { class: 'drop' }, h('h2', null, 'Simulating a match…'),
        h('p', null, 'Building the demo logs by simulating the robot. A second or so.')));
      setTimeout(() => {
        this.loadTexts([
          { name: 'SL000041.CSV', text: SA.demo.pitSession() },
          { name: 'SL000042.CSV', text: SA.demo.match() },
        ], true);
      }, 30);
    },

    selectRun(index) {
      this.runIndex = index;
      this.run = this.runs[index];
      this.sessions = slt.sessions(this.run);
      // Prefer the last match, then the last session of any kind.
      const matches = this.sessions.map((s, i) => [s, i]).filter(([s]) => s.kind === 'match');
      this.sessionIndex = matches.length ? matches[matches.length - 1][1]
        : this.sessions.length ? this.sessions.length - 1 : -1;
      this.renderPickers();
      this.selectSession(this.sessionIndex);
    },

    selectSession(index) {
      for (const section of this.sections.values()) if (section._cleanup) section._cleanup();
      this.sections.clear();
      this.sessionIndex = index;
      const s = this.session();
      this.range = s ? [s.start, s.end] : [this.run.start, this.run.end];
      this.report = A.analyze(this.run, { range: this.range });
      // Just past the start: the logger only samples once the robot is enabled.
      this.cursor = Math.min(this.range[1], this.range[0] + 0.5);
      this.renderPickers();
      this.renderTabs();
      this.emit('range');
      this.show(this.tab, true);
    },

    seek(t, openReplay = false) {
      this.cursor = Math.min(Math.max(t, this.run.start), this.run.end);
      if (this.cursor < this.range[0] || this.cursor > this.range[1]) {
        // Outside the match: look at the whole run instead.
        const inside = this.sessions.findIndex((s) => this.cursor >= s.start && this.cursor <= s.end);
        this.selectSession(inside);
        this.cursor = Math.min(Math.max(t, this.run.start), this.run.end);
      }
      this.emit('cursor', this.cursor);
      if (openReplay) this.show('replay');
    },

    // --- Chrome ---

    renderPickers() {
      const runSelect = document.getElementById('run-select');
      clear(runSelect);
      this.runs.forEach((run, i) => {
        const matches = slt.sessions(run).filter((s) => s.kind === 'match').length;
        runSelect.append(h('option', { value: String(i) },
          `${run.files.join(' + ')} — ${run.meta.robot || 'robot'}, ${(run.end - run.start).toFixed(0)}s` +
          (matches ? `, ${matches} match${matches > 1 ? 'es' : ''}` : '')));
      });
      runSelect.value = String(this.runIndex);
      runSelect.hidden = this.runs.length < 2;

      const picker = clear(document.getElementById('session-picker'));
      const add = (label, index) => picker.append(h('button', {
        type: 'button',
        'aria-pressed': String(index === this.sessionIndex),
        onclick: () => this.selectSession(index),
      }, label));
      this.sessions.forEach((s, i) => add(s.label, i));
      add('Whole run', -1);

      const run = this.run;
      const label = document.getElementById('file-label');
      label.textContent = `${this.demo ? 'Demo · ' : ''}${run.files.join(' + ')} · ${run.meta.robot || '?'}`;
      label.title = `${run.meta.writer || ''} · kernel ${run.meta.kernel || '?'} · built ${run.meta.build || '?'}`;
    },

    renderTabs() {
      const critical = this.report.findings.filter((f) => f.severity === 'critical').length;
      for (const button of document.querySelectorAll('#tabs button')) {
        const existing = button.querySelector('.count');
        if (existing) existing.remove();
        if (button.dataset.tab === 'overview' && critical > 0) {
          button.append(h('span', { class: 'count', title: `${critical} critical findings` }, critical));
        }
      }
    },

    show(tab, rerender = false) {
      this.tab = tab;
      SA.ui.storage('tab', tab);
      for (const button of document.querySelectorAll('#tabs button')) {
        button.setAttribute('aria-selected', String(button.dataset.tab === tab));
      }
      const main = document.getElementById('main');
      if (!this.run) return;
      // Each view keeps its section; a new range rebuilds it.
      for (const [name, section] of this.sections) section.hidden = name !== tab;
      let section = this.sections.get(tab);
      if (!section || rerender || section.dataset.range !== this.rangeKey()) {
        if (section) {
          if (section._cleanup) section._cleanup();
          section.remove();
        }
        section = h('div', { class: 'view', dataset: { range: this.rangeKey() } });
        this.sections.set(tab, section);
        clear(main);
        for (const s of this.sections.values()) main.append(s);
        for (const [name, s] of this.sections) s.hidden = name !== tab;
        const view = SA.views[tab];
        if (view) view.render(section, this);
      }
      section.hidden = false;
      const view = SA.views[tab];
      if (view && view.shown) view.shown(this);
      if (!main.contains(section)) main.append(section);
    },

    rangeKey() {
      return `${this.runIndex}:${this.sessionIndex}`;
    },

    showError(lines) {
      const main = clear(document.getElementById('main'));
      main.append(h('section', { class: 'drop' },
        h('h2', null, 'Couldn\'t read those files'),
        lines.map((line) => h('p', null, line)),
        h('p', null, 'The analyzer reads SapphireLib telemetry logs: SLnnnnnn.CSV files from the ' +
          'sl folder on the robot\'s SD card, whose first line is #SLT,1.')));
    },

    start() {
      document.documentElement.style.setProperty('--topbar-h',
        `${document.getElementById('topbar').offsetHeight}px`);
      new ResizeObserver(() => document.documentElement.style.setProperty('--topbar-h',
        `${document.getElementById('topbar').offsetHeight}px`)).observe(document.getElementById('topbar'));

      document.getElementById('file-input').addEventListener('change', (e) => {
        if (e.target.files.length) this.loadFiles([...e.target.files]);
        e.target.value = '';
      });
      document.getElementById('demo-btn').addEventListener('click', () => this.loadDemo());
      document.getElementById('run-select').addEventListener('change', (e) => {
        this.selectRun(Number(e.target.value));
      });
      document.getElementById('theme-btn').addEventListener('click', () => {
        const rootEl = document.documentElement;
        const dark = rootEl.dataset.theme ? rootEl.dataset.theme === 'dark'
          : matchMedia('(prefers-color-scheme: dark)').matches;
        rootEl.dataset.theme = dark ? 'light' : 'dark';
        SA.ui.storage('theme', rootEl.dataset.theme);
        this.emit('theme');
      });
      const savedTheme = SA.ui.storage('theme');
      if (savedTheme === 'light' || savedTheme === 'dark') document.documentElement.dataset.theme = savedTheme;
      matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => this.emit('theme'));
      new MutationObserver(() => this.emit('theme')).observe(document.documentElement,
        { attributes: true, attributeFilter: ['data-theme'] });
      this.on('theme', () => {
        // Canvases pick their colors from tokens at draw time: redraw them.
        this.show(this.tab, true);
      });

      for (const button of document.querySelectorAll('#tabs button')) {
        button.addEventListener('click', () => this.show(button.dataset.tab));
      }
      const savedTab = SA.ui.storage('tab');
      if (savedTab && SA.views[savedTab]) this.tab = savedTab;
      const hash = location.hash.replace('#', '');
      if (hash && SA.views[hash]) this.tab = hash;

      // Drop files anywhere.
      let depth = 0;
      document.addEventListener('dragenter', (e) => {
        if (![...(e.dataTransfer?.types || [])].includes('Files')) return;
        depth++;
        document.body.classList.add('dragging');
      });
      document.addEventListener('dragleave', () => {
        depth = Math.max(0, depth - 1);
        if (depth === 0) document.body.classList.remove('dragging');
      });
      document.addEventListener('dragover', (e) => e.preventDefault());
      document.addEventListener('drop', (e) => {
        e.preventDefault();
        depth = 0;
        document.body.classList.remove('dragging');
        const files = [...(e.dataTransfer?.files || [])];
        if (files.length) this.loadFiles(files);
      });

      // Open in a working state: the demo, clearly marked, until real logs arrive.
      this.loadDemo();
    },
  };

  // --- Overview -------------------------------------------------------------------------

  function findingCard(f, app) {
    const actions = h('div', { class: 'actions' });
    if (f.t !== null) {
      actions.append(h('button', { class: 'btn small', type: 'button',
        onclick: () => app.seek(Math.max(app.run.start, f.t - 2), true) }, 'Replay'));
    }
    if (f.channels.some((c) => app.run.get(c))) {
      actions.append(h('button', { class: 'btn small', type: 'button', onclick: () => {
        SA.views.charts.focus(app, f.channels.filter((c) => app.run.get(c)), f.t, f.end);
      } }, 'Chart'));
    }
    return h('article', { class: `finding ${f.severity}` },
      h('div', { class: 'when' }, severityBadge(f.severity), h('br'),
        f.t !== null ? app.shortLabel(f.t) : 'whole log'),
      h('div', { class: 'body' },
        h('div', { class: 'title' }, f.title),
        h('div', { class: 'system' }, f.system, f.t !== null ? ` · ${app.label(f.t)}` : ''),
        h('div', { class: 'detail' }, f.detail)),
      actions);
  }

  function tiles(app) {
    const r = app.report;
    const count = (sev) => r.findings.filter((f) => f.severity === sev).length;
    const live = r.motors.filter((m) => !m.neverAnswered && Number.isFinite(m.peakTemp));
    const hottest = live.reduce((a, b) => (!a || b.peakTemp > a.peakTemp ? b : a), null);
    const motions = r.motions;
    const timeouts = motions.filter((m) => m.reason === 'timeout').length;
    const settled = motions.filter((m) => m.reason === 'settled').length;
    const b = r.battery;
    const mech = r.mechanisms.find((m) => Number.isFinite(m.sag));
    const tile = (label, value, foot, severity) => h('div', { class: 'tile' },
      h('span', { class: 'label' }, severity ? severityBadge(severity) : null, severity ? ' ' : null, label),
      h('span', { class: 'value' }, value),
      foot ? h('span', { class: 'foot' }, foot) : null);
    const out = [
      tile('Findings', `${count('critical')} critical`,
        `${count('warning')} warnings · ${count('info')} notes`,
        count('critical') ? 'critical' : count('warning') ? 'warning' : 'good'),
    ];
    if (hottest) {
      out.push(tile('Hottest motor', `${hottest.name} ${hottest.peakTemp.toFixed(0)}°C`,
        hottest.timeAboveDerate > 0 ? `at 55°C+ for ${hottest.timeAboveDerate.toFixed(0)}s (derating)`
          : 'never reached 55°C, where V5 motors derate',
        hottest.peakTemp >= 55 ? 'critical' : hottest.peakTemp >= 50 ? 'warning' : 'good'));
    }
    if (Number.isFinite(b.min)) {
      out.push(tile('Battery, lowest', `${b.min.toFixed(2)} V`,
        `${fmt(b.start, 1)} V → ${fmt(b.end, 1)} V · ${fmt(b.pctStart, 0)}% → ${fmt(b.pctEnd, 0)}%`,
        b.min < A.THRESHOLDS.batteryCriticalV ? 'critical' : b.min < A.THRESHOLDS.batteryWarnV ? 'warning' : 'good'));
    }
    if (motions.length) {
      const cut = motions.filter((m) => m.end === null).length;
      out.push(tile('Motions', `${settled} of ${motions.length} settled`,
        [timeouts ? `${timeouts} timed out` : 'none timed out', cut ? `${cut} cut short` : null]
          .filter(Boolean).join(' · '), timeouts ? 'warning' : 'good'));
    }
    if (mech) {
      out.push(tile(`${mech.name} at rest`, `${Math.abs(mech.sag).toFixed(1)}° ${mech.sag > 0 ? 'low' : 'high'}`,
        'median error holding a steady target', Math.abs(mech.sag) >= A.THRESHOLDS.sagMin ? 'warning' : 'good'));
    }
    out.push(tile('SD log', `${r.logger.drops} row${r.logger.drops === 1 ? '' : 's'} dropped`,
      `${r.logger.faults} write faults · ${app.run.files.length} file${app.run.files.length > 1 ? 's' : ''}`,
      r.logger.faults ? 'critical' : r.logger.drops > 100 ? 'warning' : 'good'));
    return h('div', { class: 'tiles' }, out);
  }

  function thermalStrip(app) {
    const r = app.report;
    const motors = r.motors.filter((m) => !m.neverAnswered);
    if (motors.length === 0) return null;
    const [t0, t1] = app.range;
    const rows = motors.slice().sort((a, b) => (b.peakTemp || 0) - (a.peakTemp || 0)).map((m) => {
      const canvas = h('canvas', { width: '600', height: '14', role: 'img',
        'aria-label': `${m.name} temperature over time, peak ${fmt(m.peakTemp, 0)}°C` });
      const draw = () => {
        const ch = app.run.get(m.channel);
        const dpr = window.devicePixelRatio || 1;
        const w = canvas.clientWidth || 600;
        canvas.width = Math.round(w * dpr);
        canvas.height = Math.round(14 * dpr);
        const ctx = canvas.getContext('2d');
        ctx.scale(dpr, dpr);
        ctx.fillStyle = SA.charts.token('--surface-3');
        ctx.fillRect(0, 0, w, 14);
        const [i0, i1] = ch.range(t0, t1);
        for (let i = i0; i < i1; ++i) {
          const x0 = ((ch.t[i] - t0) / (t1 - t0)) * w;
          const x1 = i + 1 < ch.length ? ((Math.min(ch.t[i + 1], t1) - t0) / (t1 - t0)) * w : w;
          const temp = ch.cols.temp[i];
          ctx.fillStyle = Number.isFinite(temp) ? heatColor(temp) : SA.charts.token('--surface-3');
          ctx.fillRect(x0, 0, Math.max(1, x1 - x0 + 0.5), 14);
        }
        // Phase boundaries as hairlines.
        ctx.fillStyle = SA.charts.token('--surface');
        for (const s of app.sessions) {
          for (const edge of [s.auton && s.auton[1], s.driver && s.driver[0]]) {
            if (!edge || edge <= t0 || edge >= t1) continue;
            ctx.fillRect(((edge - t0) / (t1 - t0)) * w, 0, 2, 14);
          }
        }
      };
      requestAnimationFrame(draw);
      canvas.addEventListener('click', (e) => {
        const rect = canvas.getBoundingClientRect();
        app.seek(t0 + ((e.clientX - rect.left) / rect.width) * (t1 - t0), true);
      });
      canvas.style.cursor = 'pointer';
      const peak = h('span', { class: 'peak' }, Number.isFinite(m.peakTemp) ? `${m.peakTemp.toFixed(0)}°` : '—');
      return h('div', { class: 'thermal-row' }, h('span', { class: 'name' }, m.name), canvas, peak);
    });
    return h('section', { class: 'panel' },
      h('header', null, h('h3', null, 'Motor temperatures'),
        h('span', { class: 'hint' }, 'Across the selected range, hottest first. V5 motors report in 5°C ' +
          'steps and cut their own power from 55°C. Click to replay that moment.')),
      h('div', { class: 'thermal' }, rows),
      SA.ui.heatLegend());
  }

  function motorTable(app) {
    const motors = app.report.motors;
    if (!motors.length) return null;
    const rows = motors.slice().sort((a, b) => (b.peakTemp || -1) - (a.peakTemp || -1)).map((m) => {
      const chip = Number.isFinite(m.peakTemp)
        ? h('span', { class: 'heat-chip', style: { background: heatColor(m.peakTemp), color: heatInk(m.peakTemp) } },
          `${m.peakTemp.toFixed(0)}°C`)
        : '—';
      const perVolt = Number.isFinite(m.rpmPerVoltEarly)
        ? `${m.rpmPerVoltEarly.toFixed(1)} → ${m.rpmPerVoltLate.toFixed(1)}` : '—';
      return h('tr', null,
        h('td', { class: 'data' }, m.name),
        h('td', { class: 'num' }, m.neverAnswered ? 'no motor' : chip),
        h('td', { class: 'num' }, m.timeAboveDerate > 0 ? `${m.timeAboveDerate.toFixed(0)}s` : '—'),
        h('td', { class: 'num' }, m.stallS > 0 ? `${m.stallS.toFixed(1)}s` : '—'),
        h('td', { class: 'num' }, Number.isFinite(m.meanAmps) ? `${m.meanAmps.toFixed(2)} A` : '—'),
        h('td', { class: 'num' }, perVolt),
        h('td', { class: 'num' }, m.disconnects.length || '—'));
    });
    return h('section', { class: 'panel' },
      h('header', null, h('h3', null, 'Motor health'),
        h('span', { class: 'hint' }, 'Speed per volt early → late in the range: a big drop is a motor ' +
          'derating (hot) or dragging (friction).')),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data-table' },
        h('thead', null, h('tr', null, h('th', null, 'Motor'), h('th', { class: 'num' }, 'Peak'),
          h('th', { class: 'num' }, '≥55°C'), h('th', { class: 'num' }, 'Stalled'),
          h('th', { class: 'num' }, 'Mean current'), h('th', { class: 'num' }, 'rpm/V'),
          h('th', { class: 'num' }, 'Dropouts'))),
        h('tbody', null, rows))));
  }

  SA.views.overview = {
    render(section, app) {
      const r = app.report;
      if (app.demo) {
        section.append(h('div', { class: 'banner' },
          h('strong', null, 'This is the demo.'),
          h('span', null, 'A simulated pit session and match with problems planted in them. Open your ' +
            'robot\'s logs (SLnnnnnn.CSV from the SD card\'s sl folder) to analyze your own.'),
          h('label', { class: 'btn small', for: 'file-input' }, 'Open logs')));
      }
      if (app.errors && app.errors.length) {
        section.append(h('div', { class: 'banner' }, h('strong', null, 'Skipped:'),
          app.errors.join('; ')));
      }
      const s = app.session();
      section.append(h('div', null,
        h('div', { class: 'eyebrow' }, s ? s.kind === 'match' ? 'Match' : s.kind : 'Whole run'),
        h('h2', null, s ? `${s.label}: ` + [s.auton ? `autonomous ${A.clock(s.auton[1] - s.auton[0])}` : null,
          s.driver ? `driver ${A.clock(s.driver[1] - s.driver[0])}` : null].filter(Boolean).join(', ') :
          `${app.run.files.join(' + ')}: ${(app.run.end - app.run.start).toFixed(0)}s`)));
      section.append(tiles(app));

      const severities = ['critical', 'warning', 'info'];
      const shown = new Set(['critical', 'warning']);
      const list = h('div', { class: 'findings' });
      const filters = h('div', { class: 'chart-toolbar' });
      const renderList = () => {
        clear(list);
        const items = r.findings.filter((f) => shown.has(f.severity));
        if (items.length === 0) {
          list.append(h('p', { class: 'muted' }, r.findings.length ? 'Nothing at this severity.' :
            'No findings in this range: nothing overheated, stalled, dropped out or timed out.'));
        }
        for (const f of items) list.append(findingCard(f, app));
      };
      for (const sev of severities) {
        const n = r.findings.filter((f) => f.severity === sev).length;
        const button = h('button', { class: 'btn small', type: 'button', 'aria-pressed': String(shown.has(sev)),
          onclick: () => {
            if (shown.has(sev)) shown.delete(sev);
            else shown.add(sev);
            button.setAttribute('aria-pressed', String(shown.has(sev)));
            renderList();
          } }, severityBadge(sev), ` ${n}`);
        filters.append(button);
      }
      renderList();
      section.append(h('section', { class: 'panel' },
        h('header', null, h('h3', null, 'What went wrong'),
          h('span', { class: 'hint' }, 'Most severe first. Replay jumps to two seconds before it.')),
        filters, h('div', { style: { height: '10px' } }), list));
      section.append(h('div', { class: 'two-col' }, thermalStrip(app), motorTable(app)));
    },
  };

  // --- PID responses ---------------------------------------------------------------------

  SA.views.responses = {
    render(section, app) {
      const pids = app.report.pids.filter((p) => p.responses.length);
      if (!pids.length) {
        section.append(h('p', { class: 'muted' }, 'No PID channels logged in this range.'));
        return;
      }
      let current = pids[0];
      const picker = h('div', { class: 'segments' });
      const tableBox = h('div', { class: 'table-wrap' });
      const detail = h('div', { class: 'chart-stack' });
      const motions = slt.motions(app.run);
      const gains = h('div');

      const pick = (pid) => {
        current = pid;
        for (const b of picker.children) b.setAttribute('aria-pressed', String(b.textContent === pid.name));
        renderTable();
      };
      for (const pid of pids) {
        picker.append(h('button', { type: 'button', onclick: () => pick(pid) }, pid.name));
      }
      let charts = [];
      const showResponse = (r) => {
        for (const c of charts) c.destroy();
        charts = [];
        clear(detail);
        const ch = app.run.get(current.name);
        const pad = Math.min(0.3, (r.end - r.start) * 0.1);
        const group = new SA.charts.ChartGroup({
          full: [r.start - pad, r.end + pad], formatTime: (t) => `${(t - r.start).toFixed(2)}s`,
        });
        const series = (col, label, token) => ({ label, color: token, t: ch.t, y: ch.cols[col] });
        const folded = ch.cols.meas[r.first] === 0 && ch.cols.meas[r.last] === 0;
        const top = new SA.charts.TimeChart(detail, {
          title: folded ? 'Error (this loop folds the target into the error)' : 'Target, measurement, error',
          height: 170,
          series: folded ? [series('err', 'error', '--series-1')]
            : [series('target', 'target', '--series-1'), series('meas', 'measured', '--series-2'),
              series('err', 'error', '--series-3')],
        });
        const terms = new SA.charts.TimeChart(detail, {
          title: 'What it commanded (volts)',
          height: 170,
          series: [series('p', 'P', '--series-1'), series('i', 'I', '--series-2'),
            series('d', 'D', '--series-3'), series('out', 'output', '--series-7')],
        });
        group.add(top);
        group.add(terms);
        charts = [top, terms];
        group.redraw();
      };
      const renderTable = () => {
        clear(tableBox);
        clear(gains);
        const ch = app.run.get(current.name);
        const rows = current.responses.map((r, i) => {
          const motion = motions.find((m) => r.start >= m.start - 0.05 && r.start <= (m.stop ?? m.start) + 0.05);
          const row = h('tr', { class: 'clickable', tabindex: '0' },
            h('td', { class: 'num' }, i + 1),
            h('td', { class: 'data' }, app.shortLabel(r.start)),
            h('td', null, motion ? motion.kind : r.end - r.start > 10 ? 'continuous' : '—'),
            h('td', { class: 'num' }, fmt(r.initialError, 2)),
            h('td', { class: 'num' }, fmt(r.finalError, 2)),
            h('td', { class: 'num' }, `${(r.end - r.start).toFixed(2)}s`),
            h('td', { class: 'num' }, `${Math.round(r.saturatedFraction * 100)}%`),
            h('td', { class: 'num' }, r.swings),
            h('td', { class: 'num' }, r.gains ? `${sig(r.gains.kP)} / ${sig(r.gains.kI)} / ${sig(r.gains.kD)}` : '—'));
          const open = () => {
            for (const other of row.parentNode.children) other.classList.remove('selected');
            row.classList.add('selected');
            showResponse(r);
          };
          row.addEventListener('click', open);
          row.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') open();
          });
          return row;
        });
        tableBox.append(h('table', { class: 'data-table' },
          h('thead', null, h('tr', null, h('th', { class: 'num' }, '#'), h('th', null, 'Start'), h('th', null, 'Motion'),
            h('th', { class: 'num' }, 'First error'), h('th', { class: 'num' }, 'Last error'),
            h('th', { class: 'num' }, 'Length'), h('th', { class: 'num' }, 'Saturated'),
            h('th', { class: 'num' }, 'Swings'), h('th', { class: 'num' }, 'kP / kI / kD'))),
          h('tbody', null, rows)));
        if (ch.gains.length) {
          gains.append(h('p', { class: 'note' }, `Gains logged for ${current.name}: `,
            ch.gains.map((g) => `${app.shortLabel(g.t)} → kP ${sig(g.kP)}, kI ${sig(g.kI)}, kD ${sig(g.kD)}`).join(' · ')));
        }
        const first = current.responses.find((r) => r.end - r.start < 10) || current.responses[0];
        if (first) {
          const index = current.responses.indexOf(first);
          rows[index].classList.add('selected');
          showResponse(first);
        }
      };
      section.append(h('section', { class: 'panel' },
        h('header', null, h('h3', null, 'Step responses'),
          h('span', { class: 'hint' }, 'Each response starts where the PID was reset (a new motion). ' +
            'Swings count zero crossings of the error; many that don\'t die out is oscillation.')),
        picker, h('div', { style: { height: '10px' } }), tableBox, gains));
      section.append(h('section', { class: 'panel' }, detail));
      pick(current);
    },
  };

  // --- Motions ------------------------------------------------------------------------------

  SA.views.motions = {
    render(section, app) {
      const list = app.report.motions;
      if (!list.length) {
        section.append(h('p', { class: 'muted' }, 'No drivetrain motions in this range.'));
        return;
      }
      const detail = h('div', { class: 'two-col' });
      let charts = [];
      const describe = (m) => {
        const p = m.params;
        if (m.kind === 'turnToHeading') return `to ${fmt(p.target_deg, 1)}°`;
        if (m.kind === 'driveDistance') return `${fmt(p.target_in, 1)} in`;
        if (m.kind === 'moveToPoint') return `to (${fmt(p.x, 1)}, ${fmt(p.y, 1)})`;
        if (m.kind === 'moveToPose') return `to (${fmt(p.x, 1)}, ${fmt(p.y, 1)}) at ${fmt(p.heading_deg, 0)}°`;
        if (m.kind === 'followPath') return `${p.waypoints} waypoints`;
        return '';
      };
      const show = (m) => {
        for (const c of charts) c.destroy();
        charts = [];
        clear(detail);
        const stop = m.stop ?? m.start + 1;
        const field = h('canvas', { class: 'field-canvas', role: 'img', 'aria-label': 'Path during the motion' });
        detail.append(h('section', { class: 'panel' }, h('header', null, h('h3', null, 'Path'),
          h('span', { class: 'hint' }, 'Odometry during the motion; the target where it has one.')), field));
        requestAnimationFrame(() => SA.replay.drawField(field, app, stop, { from: m.start, motion: m }));
        const stack = h('div', { class: 'chart-stack' });
        detail.append(h('section', { class: 'panel' }, stack));
        const group = new SA.charts.ChartGroup({ full: [m.start - 0.2, stop + 0.4],
          formatTime: (t) => `${(t - m.start).toFixed(2)}s` });
        for (const name of ['drive', 'turn']) {
          const ch = app.run.get(name);
          if (!ch) continue;
          const chart = new SA.charts.TimeChart(stack, {
            title: `${name} PID error`, height: 150,
            series: [{ label: `${name} error`, color: name === 'drive' ? '--series-1' : '--series-2', t: ch.t, y: ch.cols.err }],
            guides: m.params.threshold !== undefined ? [{ y: m.params.threshold, label: 'exit threshold' },
              { y: -m.params.threshold }] : [],
          });
          group.add(chart);
          charts.push(chart);
        }
        group.redraw();
      };
      const rows = list.map((m) => {
        const result = m.reason === 'settled' ? h('span', { class: 'pill ok' }, 'settled')
          : m.reason ? h('span', { class: 'pill bad' }, m.reason) : h('span', { class: 'pill' }, 'cut short');
        const row = h('tr', { class: 'clickable', tabindex: '0' },
          h('td', { class: 'data' }, app.shortLabel(m.start)),
          h('td', null, m.kind),
          h('td', { class: 'data' }, describe(m)),
          h('td', null, result),
          h('td', { class: 'num' }, m.error !== null ? fmt(m.error, 2) : '—'),
          h('td', { class: 'num' }, m.ms !== null ? `${m.ms} ms` : '—'),
          h('td', null, h('button', { class: 'btn small', type: 'button', onclick: (e) => {
            e.stopPropagation();
            app.seek(m.start - 0.5, true);
          } }, 'Replay')));
        const open = () => {
          for (const other of row.parentNode.children) other.classList.remove('selected');
          row.classList.add('selected');
          show(m);
        };
        row.addEventListener('click', open);
        row.addEventListener('keydown', (e) => {
          if (e.key === 'Enter') open();
        });
        return row;
      });
      section.append(h('section', { class: 'panel' },
        h('header', null, h('h3', null, 'Drivetrain motions'),
          h('span', { class: 'hint' }, 'Every blocking motion, from its start and end events. Final error is ' +
            'inches, or degrees for turns.')),
        h('div', { class: 'table-wrap' }, h('table', { class: 'data-table' },
          h('thead', null, h('tr', null, h('th', null, 'Start'), h('th', null, 'Motion'), h('th', null, 'Target'),
            h('th', null, 'Result'), h('th', { class: 'num' }, 'Final error'), h('th', { class: 'num' }, 'Took'),
            h('th', null, ''))),
          h('tbody', null, rows)))));
      section.append(detail);
      rows[0].classList.add('selected');
      show(list[0]);
    },
  };

  root.SA.app = app;
})();
