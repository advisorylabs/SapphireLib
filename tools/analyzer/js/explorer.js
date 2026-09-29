/*
 * SapphireLib telemetry analyzer — explorer.js
 *
 * The Charts tab: any column of any channel, stacked on one shared time axis.
 * Presets cover the usual questions (motor temperatures, the lift, each PID,
 * the battery); the picker covers the rest. Each chart holds one channel, so
 * a chart's series share their units and it keeps a single y-axis. Below the
 * charts, a table of every plotted value at the cursor, and the visible range
 * as CSV to copy.
 *
 * Browser only: SA.views.charts.
 *
 * Team 96671H — Hitmen
 */
(function () {
  'use strict';

  const { h, clear, fmt } = SA.ui;
  const { analysis: A } = SA;

  /** Preset selections: [{ channel, columns }] per preset, for this run. */
  function presets(run) {
    const has = (name) => run.get(name);
    const motors = A.motorChannels(run).map((c) => c.name);
    const out = [];
    if (motors.length) {
      out.push({ name: 'Motor temperatures', pick: motors.map((c) => ({ channel: c, columns: ['temp'] })), merge: 'temp' });
      out.push({ name: 'Motor current', pick: motors.map((c) => ({ channel: c, columns: ['amps'] })), merge: 'amps' });
    }
    for (const act of A.mechanismChannels(run)) {
      const base = act.name.replace(/\.act$/, '');
      const pick = [{ channel: act.name, columns: ['target', 'pos'] }, { channel: act.name, columns: ['volts'] }];
      for (const m of motors.filter((c) => c.toLowerCase().includes(base.toLowerCase()))) {
        pick.push({ channel: m, columns: ['temp'] });
      }
      out.push({ name: base[0].toUpperCase() + base.slice(1), pick, merge: 'temp' });
    }
    for (const pid of ['drive', 'turn', 'hold']) {
      if (has(pid)) {
        out.push({ name: `${pid} PID`, pick: [{ channel: pid, columns: ['err'] }, { channel: pid, columns: ['p', 'i', 'd', 'out'] }] });
      }
    }
    if (has('chassis')) out.push({ name: 'Chassis volts', pick: [{ channel: 'chassis', columns: ['fwd_v', 'strafe_v', 'turn_v'] }] });
    if (has('batt')) {
      const pick = [{ channel: 'batt', columns: ['volts'] }];
      if (has('batt').has('amps')) pick.push({ channel: 'batt', columns: ['amps'] });
      out.push({ name: 'Battery', pick });
    }
    if (has('driver')) out.push({ name: 'Driver sticks', pick: [{ channel: 'driver', columns: ['lx', 'ly', 'rx', 'ry'] }] });
    if (has('odom')) out.push({ name: 'Odometry', pick: [{ channel: 'odom', columns: ['x', 'y'] }, { channel: 'odom', columns: ['heading'] }] });
    return out;
  }

  const view = {
    state: null,

    /** Opens the Charts tab on `channels` around [t, end]. */
    focus(app, channels, t, end) {
      const pick = [];
      for (const name of channels) {
        const ch = app.run.get(name);
        if (!ch) continue;
        const columns = ch.kind === 'pid' ? ['err', 'out']
          : ch.has('temp') ? ['temp', 'amps'] : ch.has('pos') ? ['target', 'pos'] : ch.columns.slice(0, 4);
        for (const column of columns) pick.push({ channel: name, columns: [column] });
      }
      view.pending = { pick, window: t !== null && t !== undefined ? [t - 5, (end ?? t) + 5] : null };
      app.show('charts', true);
    },

    render(section, app) {
      const run = app.run;
      const group = new SA.charts.ChartGroup({
        full: app.range.slice(),
        formatTime: (t, long) => (long ? app.label(t) : app.shortLabel(t)),
        onSeek: (t) => {
          app.cursor = t;
          group.setCursor(t);
          renderValues();
        },
      });
      const stack = h('div', { class: 'chart-stack' });
      const values = h('div', { class: 'table-wrap' });
      let charts = [];
      // The selection: an ordered list of { channel, columns } — one chart each.
      let selection = [];
      const bands = app.bands();
      const markers = app.findingMarkers();
      let colorIndex = 0;

      const rebuild = () => {
        for (const c of charts) c.destroy();
        charts = [];
        clear(stack);
        colorIndex = 0;
        if (selection.length === 0) {
          stack.append(h('p', { class: 'muted' }, 'Pick a preset or some columns on the left.'));
        }
        for (const sel of selection) {
          const ch = run.get(sel.channel);
          if (!ch) continue;
          const series = sel.columns.map((col, k) => ({
            label: sel.merged ? A.shortName(ch.name) : col,
            color: `--series-${(k % 8) + 1}`,
            t: ch.t,
            y: ch.cols[col],
            step: col === 'temp' || col === 'target' || col === 'law' || col === 'flags',
          }));
          const chart = new SA.charts.TimeChart(stack, {
            title: sel.title || `${sel.channel}: ${sel.columns.join(', ')}`,
            height: 160,
            series: sel.extra ? series.concat(sel.extra) : series,
            bands,
            markers,
            guides: sel.columns.includes('temp') ? [{ y: 55, label: '55°C: derating' }] : [],
          });
          group.add(chart);
          charts.push(chart);
        }
        group.redraw();
        renderPicker();
        renderValues();
      };

      /** A merged selection: one column from several channels in one chart (all °C, say). */
      const applyPreset = (preset) => {
        selection = [];
        if (preset.merge) {
          const merged = preset.pick.filter((p) => p.columns.length === 1 && p.columns[0] === preset.merge);
          const rest = preset.pick.filter((p) => !merged.includes(p));
          for (const p of rest) selection.push({ channel: p.channel, columns: p.columns });
          if (merged.length) {
            // Several channels, one column: one chart, one series per channel, in
            // fixed slot order (never more than eight — the rest go to "more").
            const first = merged.slice(0, 8);
            selection.push({ channel: first[0].channel, columns: [preset.merge], merged: true,
              title: `${preset.name}${merged.length > 8 ? ' (first 8)' : ''}`,
              extra: first.slice(1).map((p, k) => ({
                label: A.shortName(p.channel),
                color: `--series-${k + 2}`,
                t: run.get(p.channel).t,
                y: run.get(p.channel).cols[preset.merge],
                step: preset.merge === 'temp',
              })) });
          }
        } else {
          selection = preset.pick.map((p) => ({ channel: p.channel, columns: p.columns.slice() }));
        }
        rebuild();
      };

      // Picker.
      const presetBar = h('div', { class: 'presets' });
      const picker = h('div', { class: 'picker' });
      const renderPicker = () => {
        clear(picker);
        for (const ch of run.channels) {
          if (ch.kind === 'events' || ch.length === 0) continue;
          const open = selection.some((s) => s.channel === ch.name);
          const details = h('details', { open: open ? true : null },
            h('summary', null, ch.name, h('span', { class: 'kind' }, `${ch.kind} · ${ch.length} rows`)));
          for (const col of ch.columns) {
            const checked = selection.some((s) => s.channel === ch.name && s.columns.includes(col) && !s.merged);
            const box = h('input', { type: 'checkbox', checked: checked ? true : null });
            box.addEventListener('change', () => {
              let sel = selection.find((s) => s.channel === ch.name && !s.merged);
              if (box.checked) {
                if (!sel) {
                  sel = { channel: ch.name, columns: [] };
                  selection.push(sel);
                }
                if (!sel.columns.includes(col)) sel.columns.push(col);
              } else if (sel) {
                sel.columns = sel.columns.filter((c) => c !== col);
                if (sel.columns.length === 0) selection = selection.filter((s) => s !== sel);
              }
              rebuild();
            });
            details.append(h('label', null, box, col));
          }
          picker.append(details);
        }
      };
      for (const preset of presets(run)) {
        presetBar.append(h('button', { class: 'btn small', type: 'button', onclick: () => applyPreset(preset) }, preset.name));
      }

      // Values at the cursor: the table view of every chart.
      const renderValues = () => {
        clear(values);
        const t = group.cursor ?? app.cursor;
        const rows = [];
        for (const chart of charts) {
          for (const s of chart.series) {
            const i = SA.charts.indexAt(s.t, t);
            rows.push(h('tr', null, h('td', null, chart.titleEl.textContent), h('td', null, s.label),
              h('td', { class: 'num' }, i >= 0 ? SA.charts.formatValue(s.y[i]) : '—'),
              h('td', { class: 'num' }, i >= 0 ? `${(t - s.t[i]).toFixed(3)}s` : '—')));
          }
        }
        values.append(h('table', { class: 'data-table' },
          h('caption', { class: 'muted', style: { textAlign: 'left', paddingBottom: '6px' } },
            `Values at ${app.label(t)}`),
          h('thead', null, h('tr', null, h('th', null, 'Chart'), h('th', null, 'Series'), h('th', { class: 'num' }, 'Value'),
            h('th', { class: 'num' }, 'Age'))),
          h('tbody', null, rows)));
      };

      const csvBox = h('pre', { class: 'code', hidden: true });
      const copyCsv = async () => {
        const [a, b] = group.range;
        const lines = [];
        for (const chart of charts) {
          for (const s of chart.series) {
            const [i0] = [Math.max(0, SA.charts.indexAt(s.t, a))];
            const i1 = SA.charts.indexAt(s.t, b);
            lines.push(`# ${chart.titleEl.textContent} / ${s.label}`);
            lines.push('t_s,value');
            for (let i = i0; i <= i1; ++i) lines.push(`${s.t[i].toFixed(6)},${s.y[i]}`);
          }
        }
        csvBox.textContent = lines.join('\n');
        csvBox.hidden = false;
        const ok = await SA.ui.copyText(csvBox.textContent, csvBox);
        copyBtn.textContent = ok ? 'Copied' : 'Selected: copy it';
        setTimeout(() => {
          copyBtn.textContent = 'Copy visible range as CSV';
        }, 2000);
      };
      const copyBtn = h('button', { class: 'btn small', type: 'button', onclick: copyCsv }, 'Copy visible range as CSV');

      section.append(h('div', { class: 'explorer' },
        h('aside', { class: 'panel' }, h('header', null, h('h3', null, 'Channels')), picker),
        h('div', { class: 'side-stack' },
          h('section', { class: 'panel' },
            h('div', { class: 'chart-toolbar' }, presetBar,
              h('button', { class: 'btn small', type: 'button', onclick: () => group.zoom(0.5) }, 'Zoom in'),
              h('button', { class: 'btn small', type: 'button', onclick: () => group.zoom(2) }, 'Zoom out'),
              h('button', { class: 'btn small', type: 'button', onclick: () => group.reset() }, 'Whole range'),
              h('span', { class: 'hint' }, 'Drag to zoom · Ctrl+wheel zooms · click sets the cursor')),
            h('div', { style: { height: '12px' } }), stack),
          h('section', { class: 'panel' }, h('header', null, h('h3', null, 'Values at the cursor'), copyBtn),
            values, csvBox))));

      group.cursor = app.cursor;
      const pending = view.pending;
      view.pending = null;
      if (pending && pending.pick.length) {
        selection = pending.pick;
        rebuild();
        if (pending.window) group.setRange(pending.window[0], pending.window[1]);
      } else {
        const list = presets(run);
        if (list.length) applyPreset(list[0]);
        else rebuild();
      }
    },
  };

  SA.views.charts = view;
})();
