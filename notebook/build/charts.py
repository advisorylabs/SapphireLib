"""Every chart and diagram on Section G's pages, drawn with matplotlib into img/.

Charts read the JSON that experiments.js wrote (data/); diagrams are drawn
from scratch. make_all(blocks) is called by build_section.py with the page
date ranges, which the Gantt charts are drawn from.
"""
import datetime as dt
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, 'img')
DATA = os.path.join(HERE, 'data')
os.makedirs(IMG, exist_ok=True)

plt.rcParams.update({
    'font.family': 'Arial', 'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': '#DDDDDD', 'grid.linewidth': 0.8, 'axes.axisbelow': True,
    'axes.titlesize': 11, 'axes.titleweight': 'bold', 'legend.frameon': False, 'figure.dpi': 200,
})

ORANGE, BLUE, GREEN, RED, GRAY, PURPLE, DARK = '#E65100', '#1565C0', '#2E7D32', '#C62828', '#757575', '#6A1B9A', '#212121'


def data(name):
    with open(os.path.join(DATA, name + '.json'), encoding='utf-8') as f:
        return json.load(f)


def save(fig, name):
    fig.savefig(os.path.join(IMG, name + '.png'), dpi=200, bbox_inches='tight', pad_inches=0.06, facecolor='white')
    plt.close(fig)


def blank(w, h, xlim, ylim):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect('equal')
    ax.axis('off')
    return fig, ax


def box(ax, x, y, w, h, text, fill='#E3F2FD', edge='#000000', size=9, bold=False, ink='#000000'):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.08', fc=fill, ec=edge, lw=1))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=size, color=ink,
            fontweight='bold' if bold else 'normal', linespacing=1.25)


def arrow(ax, a, b, text=None, color='#000000', size=8, lw=1.3, rad=0.0, off=(0, 0.12)):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=12, color=color, lw=lw,
                                 connectionstyle='arc3,rad=%g' % rad, shrinkA=0, shrinkB=0))
    if text:
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], text, ha='center', va='center', fontsize=size, color=color)


# ---------------------------------------------------------------- Gantt charts

def gantt(name, rows, start, end, title, planned=None, note=None):
    """rows / planned: [(label, date0, date1, color)]. Planned bars are drawn hollow above the actual ones."""
    labels = [r[0] for r in rows]
    fig, ax = plt.subplots(figsize=(7.9, 0.36 * len(rows) + 1.25))
    for i, (label, d0, d1, color) in enumerate(rows):
        y = len(rows) - 1 - i
        if planned and planned.get(label):
            p0, p1 = planned[label]
            ax.barh(y + 0.22, mdates.date2num(p1) - mdates.date2num(p0), left=mdates.date2num(p0), height=0.26,
                    color='white', edgecolor='#000000', hatch='////', linewidth=0.8)
        if d0 is not None:
            ax.barh(y - (0.12 if planned else 0), mdates.date2num(d1) - mdates.date2num(d0) + 1, left=mdates.date2num(d0),
                    height=0.36 if planned else 0.6, color=color, edgecolor='#000000', linewidth=0.8)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(labels[::-1], fontsize=9.5)
    ax.set_xlim(mdates.date2num(start), mdates.date2num(end))
    ax.xaxis.set_major_locator(mdates.MonthLocator())
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%b 1'))
    ax.xaxis.set_minor_locator(mdates.WeekdayLocator(byweekday=mdates.MO))
    ax.xaxis.tick_top()
    ax.grid(axis='y', visible=False)
    ax.grid(axis='x', which='major', color='#999999', linewidth=1)
    ax.grid(axis='x', which='minor', color='#E5E5E5', linewidth=0.6)
    ax.tick_params(axis='y', length=0)
    ax.spines['left'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.set_title(title, pad=22)
    if planned:
        ax.barh([-5], [0], color='white', edgecolor='#000000', hatch='////', label='Planned on 06/15')
        ax.barh([-5], [0], color='#888888', edgecolor='#000000', label='What actually happened')
        ax.set_ylim(-0.6, len(rows) - 0.4)
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.02), ncol=2, fontsize=9)
    if note:
        fig.text(0.5, -0.04 if planned else 0.0, note, ha='center', fontsize=8.5, style='italic', color='#444444')
    save(fig, name)


BLOCK_COLORS = {
    'Research and decision': '#FF00FF',
    'Core control: PID + chassis': '#4A86E8',
    'Odometry + pose motions': '#4A86E8',
    'Brain GUI + autotuners': '#00B050',
    'Asterisk drive, macros, telemetry': '#FF9900',
    'Monte Carlo localization': '#9900FF',
    'Competition robot + autons': '#000000',
}

PLAN = {
    'Research and decision': (dt.date(2026, 6, 1), dt.date(2026, 6, 15)),
    'Core control: PID + chassis': (dt.date(2026, 6, 16), dt.date(2026, 6, 26)),
    'Odometry + pose motions': (dt.date(2026, 6, 29), dt.date(2026, 7, 15)),
    'Brain GUI + autotuners': (dt.date(2026, 7, 16), dt.date(2026, 7, 31)),
    'Asterisk drive, macros, telemetry': (dt.date(2026, 8, 3), dt.date(2026, 8, 21)),
    'Competition robot + autons': (dt.date(2026, 8, 24), dt.date(2026, 9, 11)),
}


def gantts(blocks, mid_date=None, final_date=None):
    """blocks: {name: (first page date, last page date)} in order; mid_date: the progress-check page's date."""
    start, end = dt.date(2026, 5, 29), dt.date(2026, 11, 16)
    if not blocks:
        return
    order = list(BLOCK_COLORS)
    plan_rows = [(n, PLAN[n][0], PLAN[n][1], BLOCK_COLORS[n]) for n in order if n in PLAN]
    gantt('gantt_plan', plan_rows, start, dt.date(2026, 10, 5), 'SapphireLib plan, made 06/15/26',
          note='One bar per roadmap phase. Light vertical lines are Mondays.')
    mid_cut = mid_date or dt.date(2026, 9, 1)
    mid_rows = []
    for n in order:
        if n not in blocks:
            continue
        d0, d1 = blocks[n]
        if d0 > mid_cut:
            mid_rows.append((n, None, None, BLOCK_COLORS[n]))
        else:
            mid_rows.append((n, d0, min(d1, mid_cut), BLOCK_COLORS[n]))
    gantt('gantt_mid', mid_rows, start, dt.date(2026, 10, 5), 'Plan against progress on %s' % mid_cut.strftime('%m/%d/%y'), planned=PLAN)
    final_cut = final_date or dt.date(2026, 11, 11)
    final_rows = [(n, blocks[n][0], min(blocks[n][1], final_cut), BLOCK_COLORS[n]) for n in order if n in blocks]
    gantt('gantt_final', final_rows, start, end,
          'Plan against what happened, 06/01/26 to %s' % final_cut.strftime('%m/%d/%y'), planned=PLAN)


# ---------------------------------------------------------------- diagrams

def architecture():
    fig, ax = blank(7.9, 3.9, (0, 10), (0, 5))
    layers = [
        (4.1, 'Robot program (96671H only)', ['autons', 'driver macros', 'ports + devices'], '#FFE0B2'),
        (3.1, 'What a routine calls', ['auton functions', 'motions', 'trajectories', 'mechanisms'], '#C8E6C9'),
        (2.1, 'Knowing where the robot is', ['odometry', 'Monte Carlo localization', 'IMU scale'], '#E1BEE7'),
        (1.1, 'Control and tools', ['PID', 'Auto-Tune', 'telemetry', 'brain GUI'], '#BBDEFB'),
        (0.1, 'PROS kernel + V5 hardware', ['motors', 'rotation sensors', 'distance sensors', 'SD card'], '#E0E0E0'),
    ]
    for y, title, parts, fill in layers:
        ax.add_patch(Rectangle((0.1, y), 9.8, 0.85, fc=fill, ec='#000000', lw=1))
        ax.text(0.25, y + 0.42, title, fontsize=9.5, fontweight='bold', va='center')
        w = 6.3 / len(parts)
        for i, p in enumerate(parts):
            box(ax, 3.55 + i * w, y + 0.14, w - 0.12, 0.57, p, fill='#FFFFFF', size=8.5)
    for y in (4.1, 3.1, 2.1, 1.1):
        arrow(ax, (1.9, y), (1.9, y - 0.15), lw=1.5)
    save(fig, 'architecture')


def pid_loop():
    fig, ax = blank(7.9, 2.5, (0, 10), (0, 3.2))
    box(ax, 0.1, 1.3, 1.25, 0.7, 'Target\n(24 in)', '#FFF9C4', bold=True)
    box(ax, 1.9, 1.3, 1.1, 0.7, 'Error =\ntarget - now', '#FFFFFF')
    box(ax, 3.55, 2.35, 1.9, 0.6, 'P: how far off now', '#FFCDD2')
    box(ax, 3.55, 1.35, 1.9, 0.6, 'I: how long it has been off', '#C8E6C9')
    box(ax, 3.55, 0.35, 1.9, 0.6, 'D: how fast it is closing', '#BBDEFB')
    box(ax, 6.0, 1.3, 1.25, 0.7, 'Add up,\nlimit + slew', '#FFFFFF')
    box(ax, 7.8, 1.3, 1.1, 0.7, 'Motors\n(volts)', '#E0E0E0', bold=True)
    box(ax, 7.8, 0.1, 1.1, 0.6, 'Sensors', '#E0E0E0')
    arrow(ax, (1.35, 1.65), (1.9, 1.65))
    for y in (2.65, 1.65, 0.65):
        arrow(ax, (3.0, 1.65), (3.55, y))
        arrow(ax, (5.45, y), (6.0, 1.65))
    arrow(ax, (7.25, 1.65), (7.8, 1.65))
    arrow(ax, (8.35, 1.3), (8.35, 0.7))
    ax.plot([7.8, 2.45, 2.45], [0.4, 0.4, 1.3], color='#000000', lw=1.3)
    arrow(ax, (2.45, 1.2), (2.45, 1.3))
    ax.text(5.1, 0.12, 'where the robot is now, every 10 ms', fontsize=8, ha='center', color='#444444')
    save(fig, 'pid_loop')


def _robot(ax, cx, cy, arrows, title, center=True, scale=1.0):
    w, h = 1.5 * scale, 1.9 * scale
    ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, fc='#ECEFF1', ec='#000000', lw=1.2))
    ax.add_patch(Polygon([[cx - 0.15 * scale, cy + h / 2], [cx + 0.15 * scale, cy + h / 2], [cx, cy + h / 2 + 0.22 * scale]], fc='#000000'))
    spots = {'FL': (-1, 1), 'FR': (1, 1), 'BL': (-1, -1), 'BR': (1, -1), 'ML': (-1, 0), 'MR': (1, 0)}
    for key, (sx, sy) in spots.items():
        if key[0] == 'M' and not center:
            continue
        wx, wy = cx + sx * (w / 2 + 0.16 * scale), cy + sy * (h / 2 - 0.3 * scale)
        fill = '#FFB74D' if key[0] == 'M' else '#90A4AE'
        ax.add_patch(Rectangle((wx - 0.12 * scale, wy - 0.27 * scale), 0.24 * scale, 0.54 * scale, fc=fill, ec='#000000', lw=0.9))
        v = arrows.get(key, 0)
        if v:
            ax.add_patch(FancyArrowPatch((wx + sx * 0.3 * scale, wy - v * 0.3 * scale), (wx + sx * 0.3 * scale, wy + v * 0.3 * scale),
                                         arrowstyle='-|>', mutation_scale=11, color=GREEN if v > 0 else RED, lw=2))
        elif key[0] == 'M':
            ax.text(wx + sx * 0.32 * scale, wy, 'off', fontsize=7, ha='center', va='center', color=GRAY)
    ax.text(cx, cy - h / 2 - 0.4 * scale, title, ha='center', fontsize=9.5, fontweight='bold')


def wheel_mix():
    fig, ax = blank(7.9, 3.0, (0, 10.5), (0, 4))
    _robot(ax, 1.75, 2.25, {'FL': 1, 'FR': 1, 'BL': 1, 'BR': 1, 'ML': 1, 'MR': 1}, 'Forward: all six push')
    _robot(ax, 5.25, 2.25, {'FL': 1, 'FR': -1, 'BL': -1, 'BR': 1}, 'Strafe right: corners only')
    _robot(ax, 8.75, 2.25, {'FL': 1, 'FR': -1, 'BL': 1, 'BR': -1, 'ML': 1, 'MR': -1}, 'Turn right: all six push')
    ax.text(5.25, 0.2, 'Green = wheel spins forward, red = backward. Orange wheels are the two straight "asterisk" center wheels.',
            ha='center', fontsize=8.5, color='#444444')
    save(fig, 'wheel_mix')


def asterisk_turn():
    fig, ax = blank(7.9, 2.9, (0, 10.5), (0, 3.9))
    _robot(ax, 2.6, 2.2, {'FL': 1, 'FR': -1, 'BL': 1, 'BR': -1}, 'Before: center wheels coast in a turn')
    _robot(ax, 7.9, 2.2, {'FL': 1, 'FR': -1, 'BL': 1, 'BR': -1, 'ML': 1, 'MR': -1}, 'After: center wheels help turn')
    arrow(ax, (4.4, 2.2), (6.1, 2.2), lw=2)
    ax.text(5.25, 2.5, '4 motors -> 6', ha='center', fontsize=9, fontweight='bold')
    save(fig, 'asterisk_turn')


def joystick_curve():
    x = np.linspace(-1, 1, 201)
    fig, ax = plt.subplots(figsize=(3.4, 2.7))
    for curve, color, label in ((0.0, GRAY, 'curve 0 (straight line)'), (0.5, BLUE, 'curve 0.5'), (1.0, ORANGE, 'curve 1 (pure cubic)')):
        ax.plot(x, curve * x ** 3 + (1 - curve) * x, color=color, lw=2, label=label)
    ax.set_xlabel('stick position')
    ax.set_ylabel('command sent to the drive')
    ax.legend(fontsize=7.5, loc='upper left')
    ax.set_title('Joystick curve')
    save(fig, 'joystick_curve')


def odom_diagram():
    fig, ax = blank(3.6, 3.3, (-3.2, 3.6), (-2.6, 3.6))
    ax.add_patch(Rectangle((-1.5, -1.8), 3.0, 3.6, fc='#ECEFF1', ec='#000000', lw=1.2))
    ax.add_patch(Circle((0, 0), 0.09, fc='#000000'))
    ax.text(0.15, -0.35, 'tracking\ncenter', fontsize=7.5)
    ax.add_patch(Rectangle((-1.0, -0.45), 0.22, 0.9, fc=BLUE, ec='#000000'))
    ax.text(-1.05, 0.6, 'vertical\nwheel', fontsize=7.5, ha='center', color=BLUE)
    ax.add_patch(Rectangle((-0.45, -1.35), 0.9, 0.22, fc=ORANGE, ec='#000000'))
    ax.text(0.95, -1.3, 'horizontal\nwheel', fontsize=7.5, ha='center', color=ORANGE)
    ax.annotate('', xy=(-0.89, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='<->', color='#000000', lw=1))
    ax.text(-0.45, 0.12, 'offset', fontsize=7.5, ha='center')
    ax.annotate('', xy=(0, -1.24), xytext=(0, 0), arrowprops=dict(arrowstyle='<->', color='#000000', lw=1))
    ax.text(0.34, -0.8, 'offset', fontsize=7.5, ha='center')
    ax.add_patch(FancyArrowPatch((0, 1.8), (0, 3.1), arrowstyle='-|>', mutation_scale=14, color='#000000', lw=2))
    ax.text(0.15, 2.6, 'forward', fontsize=8)
    t = np.linspace(math.radians(200), math.radians(140), 40)
    ax.plot(2.9 * np.cos(t) + 0.0, 2.9 * np.sin(t), color=RED, lw=1.5, ls='--')
    ax.text(-3.1, 1.9, 'when the robot\nturns, each wheel\nrolls along an arc\nthat is not travel', fontsize=7.5, color=RED)
    save(fig, 'odom_diagram')


def offset_error():
    err = np.linspace(0, 1.0, 50)
    fig, ax = plt.subplots(figsize=(4.0, 2.7))
    for turns, color in ((0.25, BLUE), (1, ORANGE), (3, RED)):
        ax.plot(err, err * 2 * math.pi * turns, color=color, lw=2, label='%s turn%s' % ('a quarter' if turns == 0.25 else turns, '' if turns <= 1 else 's'))
    ax.set_xlabel('error in the wheel offset (in)')
    ax.set_ylabel('false travel added to the pose (in)')
    ax.set_title('Calculated: what a wrong offset costs')
    ax.legend(fontsize=8)
    save(fig, 'offset_error')


def pursuit_diagram():
    fig, ax = blank(3.7, 3.0, (-0.5, 7), (-0.6, 5.2))
    path = np.array([[0, 0], [1.5, 2.8], [4.5, 3.6], [6.5, 1.5]])
    ax.plot(path[:, 0], path[:, 1], color='#000000', lw=2, marker='o', ms=4)
    rx, ry, L = 1.1, 1.3, 2.3
    ax.add_patch(Circle((rx, ry), L, fc='none', ec=BLUE, lw=1.5, ls='--'))
    ax.add_patch(Rectangle((rx - 0.3, ry - 0.3), 0.6, 0.6, fc='#ECEFF1', ec='#000000'))
    # exit point of the circle on the second segment
    p1, p2 = path[1], path[2]
    d = p2 - p1
    f = p1 - np.array([rx, ry])
    a, b, c = d @ d, 2 * (f @ d), f @ f - L * L
    t2 = (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)
    target = p1 + d * t2
    ax.add_patch(Circle(tuple(target), 0.13, fc=ORANGE, ec='#000000'))
    arrow(ax, (rx, ry), tuple(target), color=ORANGE, lw=2)
    ax.text(target[0] + 0.15, target[1] + 0.3, 'lookahead\npoint', fontsize=8, color=ORANGE)
    ax.text(rx - 0.4, ry - 0.75, 'robot', fontsize=8)
    ax.text(rx + 0.2, ry - L + 0.2, 'lookahead circle', fontsize=8, color=BLUE)
    ax.text(5.0, 3.6, 'path', fontsize=8)
    save(fig, 'pursuit_diagram')


def boomerang_diagram():
    fig, ax = blank(3.7, 2.9, (-0.5, 7), (-0.8, 4.8))
    target = np.array([5.5, 3.5])
    ax.add_patch(Rectangle((target[0] - 0.3, target[1] - 0.3), 0.6, 0.6, fc='#C8E6C9', ec='#000000'))
    arrow(ax, tuple(target), (target[0] + 1.0, target[1]), color=GREEN, lw=2)
    ax.text(target[0] - 0.3, target[1] + 0.5, 'target pose', fontsize=8, color=GREEN)
    robot = np.array([0.5, 0.2])
    pts = [robot]
    p = robot.copy()
    for _ in range(60):
        dist = np.linalg.norm(target - p)
        carrot = target - np.array([1.0, 0]) * 0.6 * dist
        step = (carrot - p)
        n = np.linalg.norm(step)
        if n < 0.05:
            break
        p = p + step / n * min(0.15, n)
        pts.append(p.copy())
    pts = np.array(pts)
    ax.plot(pts[:, 0], pts[:, 1], color=BLUE, lw=2)
    ax.add_patch(Rectangle((robot[0] - 0.3, robot[1] - 0.3), 0.6, 0.6, fc='#ECEFF1', ec='#000000'))
    ax.text(robot[0] - 0.35, robot[1] - 0.75, 'robot', fontsize=8)
    for frac, lab in ((0.0, 'carrot at start'), (0.55, 'carrot later')):
        q = pts[int(frac * (len(pts) - 1))]
        dist = np.linalg.norm(target - q)
        carrot = target - np.array([1.0, 0]) * 0.6 * dist
        ax.add_patch(Circle(tuple(carrot), 0.12, fc=ORANGE, ec='#000000'))
        ax.text(carrot[0] - 0.2, carrot[1] + 0.25, lab, fontsize=7.5, color=ORANGE, ha='center')
    save(fig, 'boomerang_diagram')


def flow(name, steps, w=7.9, h=1.4, fills=None, size=8, wrap_rows=1):
    """A left-to-right chain of boxes (optionally on several rows)."""
    per = math.ceil(len(steps) / wrap_rows)
    fig, ax = blank(w, h, (0, 10), (0, 10 * h / w))
    bw = 10 / per
    rh = (10 * h / w) / wrap_rows
    for i, s in enumerate(steps):
        r, c = divmod(i, per)
        x = c * bw + 0.12
        y = (wrap_rows - 1 - r) * rh + 0.12
        box(ax, x, y, bw - 0.5, rh - 0.24, s, (fills or {}).get(i, '#E3F2FD'), size=size)
        if c < per - 1 and i < len(steps) - 1:
            arrow(ax, (x + bw - 0.5, y + (rh - 0.24) / 2), (x + bw - 0.02, y + (rh - 0.24) / 2))
        elif i < len(steps) - 1:
            ax.plot([x + (bw - 0.5) / 2, x + (bw - 0.5) / 2, 0.12 + (bw - 0.5) / 2], [y, y - 0.12, y - 0.12], color='#000000', lw=1.3)
            arrow(ax, (0.12 + (bw - 0.5) / 2, y - 0.12), (0.12 + (bw - 0.5) / 2, y - 0.24))
    save(fig, name)


def gui_sketch():
    fig, ax = blank(4.2, 2.3, (0, 480), (0, 240))
    ax.add_patch(Rectangle((0, 0), 480, 240, fc='#1B1B1B', ec='#000000', lw=2))
    ax.add_patch(Rectangle((0, 212), 480, 28, fc='#B71C1C', ec='none'))
    ax.text(10, 226, 'SapphireLib  96671H', color='white', fontsize=8.5, va='center', fontweight='bold')
    tabs = ['Home', 'Auton', 'Diag', 'PID', 'Odom']
    for i, t in enumerate(tabs):
        ax.add_patch(Rectangle((i * 96, 169), 96, 43, fc='#424242' if i != 3 else '#B71C1C', ec='#000000'))
        ax.text(i * 96 + 48, 190, t, color='white', fontsize=8.5, ha='center', va='center')
    ax.text(14, 140, 'Drive   kP 4.25   kI 0.00   kD 0.68', color='white', fontsize=8)
    for i, t in enumerate(['kP -', 'kP +', 'kD -', 'kD +']):
        ax.add_patch(Rectangle((14 + i * 68, 84), 60, 36, fc='#616161', ec='#000000'))
        ax.text(44 + i * 68, 102, t, color='white', fontsize=7.5, ha='center', va='center')
    for i, t in enumerate(['Run Test', 'Auto-Tune']):
        ax.add_patch(Rectangle((14 + i * 136, 30), 128, 40, fc='#2E7D32', ec='#000000'))
        ax.text(78 + i * 136, 50, t, color='white', fontsize=8, ha='center', va='center')
    for i, t in enumerate(['Drive', 'Turn', 'Hold', 'Lift']):
        ax.add_patch(Rectangle((392, 128 - i * 34), 80, 30, fc='#424242' if i else '#B71C1C', ec='#000000'))
        ax.text(432, 143 - i * 34, t, color='white', fontsize=7.5, ha='center', va='center')
    save(fig, 'gui_sketch')


def mcl_steps():
    rng = np.random.default_rng(4)
    fig, axes = plt.subplots(1, 4, figsize=(7.9, 2.35))
    truth = np.array([0.62, 0.42])
    odom = np.array([0.5, 0.52])
    titles = ['1. Guess: many dots near\nwhere odometry says', '2. Move: every dot moves\nas the wheels did (+ noise)',
              '3. Weigh: dots whose walls\nmatch the sensors grow', '4. Resample: keep the good\ndots, average = pose']
    clouds = [odom + rng.normal(0, 0.09, (120, 2)), odom + np.array([0.04, -0.03]) + rng.normal(0, 0.11, (120, 2)), None, truth + rng.normal(0, 0.03, (120, 2))]
    clouds[2] = clouds[1]
    for i, ax in enumerate(axes):
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)
        for s in ax.spines.values():
            s.set_visible(True)
            s.set_linewidth(2)
        pts = clouds[i]
        if i == 2:
            d = np.linalg.norm(pts - truth, axis=1)
            wgt = np.exp(-(d / 0.08) ** 2)
            ax.scatter(pts[:, 0], pts[:, 1], s=4 + 60 * wgt, c=wgt, cmap='RdYlGn', edgecolors='none')
            for ex, ey in ((1, truth[1]), (truth[0], 1), (0, truth[1]), (truth[0], 0)):
                ax.plot([truth[0], ex], [truth[1], ey], color=BLUE, lw=0.9, ls=':')
        else:
            ax.scatter(pts[:, 0], pts[:, 1], s=6, color=PURPLE, alpha=0.7, edgecolors='none')
        ax.add_patch(Rectangle((truth[0] - 0.05, truth[1] - 0.05), 0.1, 0.1, fc='none', ec='#000000', lw=1.4))
        if i == 3:
            est = pts.mean(axis=0)
            ax.plot(est[0], est[1], marker='*', ms=11, color=ORANGE, mec='#000000')
        ax.set_title(titles[i], fontsize=8.5, fontweight='normal')
    fig.text(0.5, 0.0, 'Black square = where the robot really is. Dotted lines = the four distance sensors. Star = the estimate.',
             ha='center', fontsize=8.5, color='#444444')
    fig.tight_layout(pad=0.4)
    save(fig, 'mcl_steps')


def ff_model():
    v = np.linspace(0, 50, 50)
    fig, ax = plt.subplots(figsize=(3.6, 2.6))
    ax.plot(v, 1.0 + 0.2 * v, color=BLUE, lw=2)
    ax.axhline(1.0, color=RED, ls='--', lw=1)
    ax.text(26, 1.35, 'kS: volts just to beat friction', fontsize=8, color=RED)
    ax.text(14, 8.3, 'slope = kV:\nvolts per in/s', fontsize=8, color=BLUE)
    ax.set_xlabel('speed (in/s)')
    ax.set_ylabel('volts needed to hold that speed')
    ax.set_ylim(0, 12)
    ax.set_title('The model Auto-Tune measures')
    save(fig, 'ff_model')


def thermal():
    curve = [(20, 1.0), (54, 1.0), (56, 0.5), (59, 0.5), (61, 0.25), (64, 0.25), (66, 0.125), (69, 0.125), (71, 0.0), (75, 0.0)]
    fig, ax = plt.subplots(figsize=(3.7, 2.6))
    ax.plot([c[0] for c in curve], [c[1] * 100 for c in curve], color=RED, lw=2.2)
    ax.fill_between([c[0] for c in curve], [c[1] * 100 for c in curve], color=RED, alpha=0.12)
    ax.set_xlabel('motor temperature (C)')
    ax.set_ylabel('power the motor still gives (%)')
    ax.set_xlim(40, 75)
    ax.set_title('V5 motor power against heat')
    save(fig, 'thermal')


# ---------------------------------------------------------------- data charts

def pid_charts():
    d = data('pid_windup')
    fig, ax = plt.subplots(figsize=(7.9, 2.6))
    ax.axvspan(0, d['wallUntilS'], color='#EEEEEE')
    ax.text(d['wallUntilS'] / 2, 60, 'robot held\nagainst a wall', ha='center', fontsize=8.5, color='#444444')
    ax.plot(d['none']['t'], d['none']['x'], color=ORANGE, lw=2, label='No anti-windup: overshoots %.1f in' % d['none']['overshootIn'])
    ax.plot(d['guard']['t'], d['guard']['x'], color=BLUE, lw=2, label='Our anti-windup: overshoots %.1f in' % d['guard']['overshootIn'])
    ax.axhline(d['target'], color='#000000', ls='--', lw=1)
    ax.text(8.9, d['target'] + 1.5, 'target 24 in', ha='right', fontsize=8.5)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('distance driven (in)')
    ax.set_xlim(0, 9)
    ax.legend(loc='upper right', fontsize=9)
    ax.set_title('Simulated: drive 24 in after being blocked for 1.5 s')
    save(fig, 'pid_windup')

    d = data('pid_kick')
    fig, ax = plt.subplots(figsize=(3.9, 2.6))
    ax.plot(d['onError']['t'], d['onError']['volts'], color=ORANGE, lw=2, label='D on error: %.0f V spike' % d['onError']['peakVolts'])
    ax.plot(d['onMeasurement']['t'], d['onMeasurement']['volts'], color=BLUE, lw=2, label='D on measurement: %.1f V' % d['onMeasurement']['peakVolts'])
    ax.axhline(12, color=RED, ls=':', lw=1)
    ax.text(1.18, 13.5, '12 V is all a motor has', fontsize=7.5, color=RED, ha='right')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('volts the PID asks for')
    ax.set_xlim(0, 1.2)
    ax.legend(fontsize=7.5)
    ax.set_title('Target nudged 5 degrees')
    save(fig, 'pid_kick')

    d = data('pid_period')
    fig, ax = plt.subplots(figsize=(3.9, 2.6))
    colors = [BLUE, GREEN, ORANGE, RED]
    styles = ['-', '--', '-', '-']
    for c, color, style in zip(d['cases'], colors, styles):
        ax.plot(c['t'], c['x'], color=color, lw=1.8, ls=style, label=c['name'])
    ax.axhline(24, color='#000000', ls=':', lw=1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('distance driven (in)')
    ax.set_ylim(15, 27)
    ax.set_xlim(0.2, 2.5)
    ax.legend(fontsize=6.8, loc='lower right')
    ax.set_title('Same gains, different loop speed')
    save(fig, 'pid_period')


def autotuner_charts():
    d = data('autotuners')
    h = d['twiddle']['history']
    fig, ax = plt.subplots(figsize=(3.9, 2.5))
    good = [(x['run'], x['cost']) for x in h if not x['bad']]
    bad = [(x['run'], x['cost']) for x in h if x['bad']]
    ax.scatter([g[0] for g in good], [g[1] for g in good], s=8, color=BLUE, label='stable test drive')
    ax.scatter([b[0] for b in bad], [min(b[1], 60) for b in bad], s=8, color=RED, label='overshot or never settled')
    ax.set_xlabel('test drive number')
    ax.set_ylabel('score (lower is better)')
    ax.set_ylim(0, 62)
    ax.legend(fontsize=7.5)
    ax.set_title('Twiddle: %d test drives' % d['twiddle']['runs'])
    save(fig, 'twiddle')

    fig, ax = plt.subplots(figsize=(3.9, 2.5))
    ax.plot(d['relay']['t'], d['relay']['x'], color=ORANGE, lw=1.8)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position (in)')
    ax.set_title('Relay: one %.1f s wobble, %.2f in each way' % (d['relay']['TuS'], d['relay']['amplitudeIn']))
    save(fig, 'relay')

    fig, axes = plt.subplots(1, 2, figsize=(7.9, 2.7), gridspec_kw={'width_ratios': [1.25, 1]})
    names = [('twiddle', 'Twiddle', GRAY), ('relay', 'Relay (Ziegler-Nichols)', ORANGE), ('model', 'Model + pole placement', BLUE)]
    ax = axes[0]
    for key, label, color in names:
        t = next(x for x in d[key]['tests'] if x['sizeIn'] == 24)
        ax.plot(t['t'], t['x'], color=color, lw=2, label=label)
    ax.axhline(24, color='#000000', ls=':', lw=1)
    ax.set_xlim(0, 2.5)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('distance driven (in)')
    ax.legend(fontsize=8, loc='lower right')
    ax.set_title('Simulated 24 in drive with each tuner\'s gains')
    ax = axes[1]
    sizes = [6, 24, 48]
    width = 0.26
    for i, (key, label, color) in enumerate(names):
        vals = [next(x for x in d[key]['tests'] if x['sizeIn'] == s)['overshootIn'] for s in sizes]
        bars = ax.bar(np.arange(3) + (i - 1) * width, vals, width, color=color)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.05, '%.1f' % v, ha='center', fontsize=7.5)
    ax.set_xticks(range(3))
    ax.set_xticklabels(['6 in drive', '24 in drive', '48 in drive'])
    ax.set_ylabel('overshoot (in)')
    ax.set_title('Overshoot at three distances')
    fig.tight_layout()
    save(fig, 'autotuner_compare')


def lookahead_chart():
    d = data('lookahead')
    fig, axes = plt.subplots(1, 2, figsize=(7.9, 3.0), gridspec_kw={'width_ratios': [1, 1.15]})
    ax = axes[0]
    path = np.array(d['path'])
    ax.plot(path[:, 0], path[:, 1], color='#000000', lw=1.2, ls='--', marker='o', ms=3, label='path')
    for L, color in ((3, RED), (12, BLUE), (24, ORANGE)):
        row = next(r for r in d['rows'] if r['lookaheadIn'] == L)
        tr = np.array(row['track'])
        ax.plot(tr[:, 0], tr[:, 1], color=color, lw=1.6, label='%d in lookahead' % L)
    ax.set_aspect('equal')
    ax.set_xlabel('field x (in)')
    ax.set_ylabel('field y (in)')
    ax.legend(fontsize=7.5, loc='lower left')
    ax.set_title('Where the robot drove')
    ax = axes[1]
    Ls = [r['lookaheadIn'] for r in d['rows']]
    ax.bar(np.arange(len(Ls)) - 0.2, [r['meanIn'] for r in d['rows']], 0.4, color=BLUE, label='mean distance off the path (in)')
    ax2 = ax.twinx()
    ax2.bar(np.arange(len(Ls)) + 0.2, [r['turningDeg'] for r in d['rows']], 0.4, color=ORANGE, label='total steering (degrees)')
    ax2.spines['right'].set_visible(True)
    ax2.grid(False)
    ax.set_xticks(range(len(Ls)))
    ax.set_xticklabels(['%d in' % L for L in Ls])
    ax.set_xlabel('lookahead distance')
    ax.set_ylabel('inches off the path', color=BLUE)
    ax2.set_ylabel('degrees of steering', color=ORANGE)
    ax.set_title('Accuracy against smoothness')
    lines = ax.get_legend_handles_labels()[0] + ax2.get_legend_handles_labels()[0]
    labels = ax.get_legend_handles_labels()[1] + ax2.get_legend_handles_labels()[1]
    ax.legend(lines, labels, fontsize=7.5, loc='upper center')
    fig.tight_layout()
    save(fig, 'lookahead')


def tank_boomerang():
    fig, ax = plt.subplots(figsize=(3.8, 2.5))
    bars = ax.bar(['Before\n(perfect odometry)', 'After\n(perfect odometry)', 'Before\n(noisy odometry)', 'After\n(noisy odometry)'],
                  [69, 299, 29, 297], color=[ORANGE, BLUE, ORANGE, BLUE])
    for b, v, lab in zip(bars, [69, 299, 29, 297], ['69', '299', '12 to 29', '297+']):
        ax.text(b.get_x() + b.get_width() / 2, v + 5, lab, ha='center', fontsize=8.5)
    ax.set_ylim(0, 335)
    ax.set_ylabel('poses settled, out of 300')
    ax.tick_params(axis='x', labelsize=7)
    ax.set_title('Tank moveToPose: desktop test')
    save(fig, 'tank_boomerang')


def mcl_charts():
    d = data('drift_series')
    fig, ax = plt.subplots(figsize=(7.9, 2.6))
    ax.plot([s['t'] for s in d['odom']], [s['e'] for s in d['odom']], color=ORANGE, lw=2, label='odometry alone')
    ax.plot([s['t'] for s in d['mcl']], [s['e'] for s in d['mcl']], color=BLUE, lw=2, label='odometry + MCL')
    ax.axvline(6, color='#000000', ls='--', lw=1)
    ax.text(6.2, ax.get_ylim()[1] * 0.9, '6 in bump', fontsize=8.5)
    ax.set_xlabel('time into the routine (s)')
    ax.set_ylabel('pose error (in)')
    ax.legend(fontsize=9, loc='center right')
    ax.set_title('Simulated Field tour, worn tracking wheels, bumped at 6 s (seed 1)')
    save(fig, 'drift_series')

    fig, ax = plt.subplots(figsize=(7.9, 2.2))
    ax.plot([s['t'] for s in d['odom']], [s['e'] for s in d['odom']], color=ORANGE, lw=2.2)
    ax.axvline(6, color='#000000', ls='--', lw=1)
    ax.text(6.2, max(s['e'] for s in d['odom']) * 0.85, 'another robot bumps ours 6 in', fontsize=8.5)
    ax.set_xlabel('time into the routine (s)')
    ax.set_ylabel('pose error (in)')
    ax.set_title('Simulated: how far off odometry gets in one 25 s routine')
    save(fig, 'drift_only')

    d = data('mcl_vs_odom')
    rows = d['rows']
    fig, ax = plt.subplots(figsize=(7.9, 2.8))
    x = np.arange(len(rows))
    b1 = ax.bar(x - 0.2, [r['odomMean'] for r in rows], 0.4, color=ORANGE, label='odometry alone')
    b2 = ax.bar(x + 0.2, [r['mclMean'] for r in rows], 0.4, color=BLUE, label='odometry + MCL')
    for bars in (b1, b2):
        for b in bars:
            ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.08, '%.2f' % b.get_height(), ha='center', fontsize=6.8)
    ax.set_xticks(x)
    ax.set_xticklabels(['%s\n%s' % (r['routine'].replace('Field tour', 'Tour').replace('Square laps', 'Laps'), r['condition']) for r in rows], fontsize=7)
    ax.set_ylabel('mean pose error (in)')
    ax.legend(fontsize=9, loc='upper left')
    ax.set_title('Simulated: mean error from the true pose, average of 5 seeds')
    for k in (2.5, 5.5, 8.5):
        ax.axvline(k, color='#BBBBBB', lw=1)
    save(fig, 'mcl_vs_odom')

    d = data('particle_count')
    fig, axes = plt.subplots(1, 2, figsize=(7.9, 2.4))
    labels = [str(r['particles']) for r in d['rows']]
    for ax, key, title, color in ((axes[0], 'mean', 'Mean pose error (in)', BLUE), (axes[1], 'recoverS', 'Seconds to get back within 1 in after the bump', PURPLE)):
        bars = ax.bar(labels, [r[key] for r in d['rows']], color=color, width=0.55)
        for b in bars:
            ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.03, '%.2f' % b.get_height(), ha='center', fontsize=8.5)
        ax.set_xlabel('particles')
        ax.set_title(title, fontsize=9.5)
        ax.set_ylim(0, max(r[key] for r in d['rows']) * 1.18)
    fig.tight_layout()
    save(fig, 'particle_count')

    fig, ax = plt.subplots(figsize=(3.8, 2.5))
    rates = ['instant\n(0.2 in jump)', '12 in/s', '4 in/s\n(ours)']
    volts = [0.7 * 20, 0.7 * 12, 0.7 * 4]
    bars = ax.bar(rates, volts, color=[RED, ORANGE, BLUE], width=0.55)
    for b, v in zip(bars, volts):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.3, '%.1f V' % v, ha='center', fontsize=9)
    ax.axhline(12, color=RED, ls=':', lw=1)
    ax.set_ylabel('voltage kick from the D term (V)')
    ax.set_ylim(0, 16)
    ax.set_title('Calculated: easing in a correction')
    save(fig, 'correction_rate')


def diagonal_chart():
    d = data('diagonal')
    fig, ax = plt.subplots(figsize=(3.9, 2.9))
    ax.plot([d['start'][0], d['end'][0]], [d['start'][1], d['end'][1]], color='#000000', ls='--', lw=1, label='straight line')
    for key, color, label in (('moveToPoint', ORANGE, 'moveToPoint: %.1f in off' % d['moveToPoint']['worstIn']),
                              ('trajectory', BLUE, 'followTrajectory: %.1f in off' % d['trajectory']['worstIn'])):
        tr = np.array(d[key]['track'])
        ax.plot(tr[:, 0], tr[:, 1], color=color, lw=2, label=label)
    ax.set_aspect('equal')
    ax.set_xlabel('field x (in)')
    ax.set_ylabel('field y (in)')
    ax.legend(fontsize=7.5, loc='upper left')
    ax.set_title('Simulated diagonal drive')
    save(fig, 'diagonal')


def claw_chart():
    d = data('claw')
    fig, ax = plt.subplots(figsize=(7.9, 2.7))
    colors = [ORANGE, GREEN, BLUE]
    for (name, rows), color in zip(d['rules'].items(), colors):
        ax.plot([r['speed'] for r in rows], [r['caught'] * 100 for r in rows], color=color, lw=2.2, marker='o', ms=4, label=name)
    ax.set_xlabel('robot speed when it reaches the Pin (in/s)')
    ax.set_ylabel('Pins caught (%)')
    ax.set_ylim(-3, 105)
    ax.legend(fontsize=9, loc='center right')
    ax.set_title('Simulated: 400 grabs at each speed')
    save(fig, 'claw')


def research_next():
    labels = ['Wheel-bias\nparticles', '+ sensor\nresetting,\n30 ms updates', '+ faster\ncorrection\nwhen far off', '1000\nparticles', 'Lower\nmotion noise']
    vals = [-14, -19.5, -30, 0, 0]
    fig, ax = plt.subplots(figsize=(7.9, 2.5))
    bars = ax.bar(labels, vals, color=[BLUE, BLUE, BLUE, GRAY, GRAY], width=0.6)
    for b, v, lab in zip(bars, vals, ['-13 to -15%', '-19.5%', '-30%', 'no help', 'no help']):
        ax.text(b.get_x() + b.get_width() / 2, v - 2.2 if v < 0 else 1, lab, ha='center', fontsize=9)
    ax.set_ylim(-36, 6)
    ax.set_ylabel('change in pose error')
    ax.axhline(0, color='#000000', lw=1)
    ax.tick_params(axis='x', labelsize=8)
    ax.set_title('Simulated on 10/27: ideas for a more accurate localizer (12 seeds, 10 scenarios)')
    save(fig, 'research_next')


def sensor_cone():
    fig, ax = blank(3.7, 2.6, (0, 8), (0, 5.6))
    ax.plot([0.3, 7.7], [5.2, 5.2], color='#000000', lw=4)
    ax.text(7.6, 4.85, 'field wall', fontsize=8, ha='right')
    ax.add_patch(Rectangle((4.3, 3.2), 1.1, 0.9, fc='#FFCC80', ec='#000000'))
    ax.text(5.55, 3.55, 'Loader', fontsize=8)
    ax.add_patch(Rectangle((3.2, 0.3), 1.0, 1.0, fc='#ECEFF1', ec='#000000'))
    ax.text(3.7, 0.05, 'robot', fontsize=8, ha='center', va='top')
    sx, sy = 3.7, 1.3
    ax.add_patch(Polygon([[sx, sy], [sx - 1.05, 5.2], [sx + 1.05, 5.2]], fc=BLUE, alpha=0.15, ec=BLUE, ls='--'))
    arrow(ax, (sx, sy), (sx, 5.2), color=BLUE, lw=1.6)
    ax.text(sx - 0.15, 2.6, 'center ray\nsays: wall', fontsize=7.5, color=BLUE, ha='right')
    arrow(ax, (sx, sy), (4.45, 3.2), color=RED, lw=1.6)
    ax.text(4.75, 2.2, 'the real reading:\nthe Loader, 3.7 in\ncloser', fontsize=7.5, color=RED)
    save(fig, 'sensor_cone')


def sim_field():
    d = data('drift_series')
    fig, ax = blank(3.5, 3.5, (-78, 78), (-78, 78))
    ax.add_patch(Rectangle((-70.25, -70.25), 140.5, 140.5, fc='#F5F5F5', ec='#000000', lw=2.5))
    for k in range(-2, 3):
        ax.plot([k * 23.4, k * 23.4], [-70.25, 70.25], color='#E0E0E0', lw=0.6)
        ax.plot([-70.25, 70.25], [k * 23.4, k * 23.4], color='#E0E0E0', lw=0.6)
    tr = np.array([[p['x'], p['y']] for p in d['track']])
    ax.plot(tr[:, 0], tr[:, 1], color=BLUE, lw=1.8)
    ax.add_patch(Rectangle((tr[0, 0] - 7.5, tr[0, 1] - 7.5), 15, 15, fc='#CFD8DC', ec='#000000'))
    ax.text(tr[0, 0], tr[0, 1] - 13, 'start', ha='center', fontsize=8)
    bump = min(d['track'], key=lambda p: abs(p['t'] - 6.0))
    ax.plot(bump['x'], bump['y'], marker='X', ms=10, color=RED, mec='#000000')
    ax.text(bump['x'] + 5, bump['y'] + 4, 'bump at 6 s', fontsize=8, color=RED)
    for ex, ey in ((70.25, 20), (20, 70.25)):
        ax.plot([20, ex], [20, ey], color=PURPLE, lw=0.9, ls=':')
    ax.text(0, 73, '12 ft field wall (140.5 in inside)', ha='center', fontsize=8)
    save(fig, 'sim_field')


def orbit():
    fig, ax = blank(3.0, 2.7, (-4, 4), (-3.6, 3.6))
    ax.add_patch(Circle((0, 0), 0.7, fc='#FFCC80', ec='#000000'))
    ax.text(0, 0, 'goal', ha='center', va='center', fontsize=8)
    ax.add_patch(Circle((0, 0), 2.3, fc='none', ec=BLUE, ls='--', lw=1.2))
    for ang in (200, 250, 300):
        a = math.radians(ang)
        cx, cy = 2.3 * math.cos(a), 2.3 * math.sin(a)
        rot = ang + 90
        ax.add_patch(Rectangle((cx - 0.45, cy - 0.45), 0.9, 0.9, angle=rot, rotation_point='center', fc='#ECEFF1', ec='#000000'))
        arrow(ax, (cx, cy), (cx * 0.55, cy * 0.55), color=GREEN, lw=1.6)
    t = np.linspace(math.radians(210), math.radians(290), 30)
    ax.plot(3.1 * np.cos(t), 3.1 * np.sin(t), color=ORANGE, lw=2)
    arrow(ax, (3.1 * math.cos(t[-2]), 3.1 * math.sin(t[-2])), (3.1 * math.cos(t[-1]), 3.1 * math.sin(t[-1])), color=ORANGE, lw=2)
    ax.text(0, -3.5, 'strafe around (orange), push in (green)', ha='center', fontsize=8)
    save(fig, 'orbit')


def make_all(blocks, mid_date=None):
    gantts(blocks, mid_date)
    sim_field()
    orbit()
    architecture()
    pid_loop()
    wheel_mix()
    asterisk_turn()
    joystick_curve()
    odom_diagram()
    offset_error()
    pursuit_diagram()
    boomerang_diagram()
    gui_sketch()
    mcl_steps()
    ff_model()
    thermal()
    pid_charts()
    autotuner_charts()
    lookahead_chart()
    tank_boomerang()
    mcl_charts()
    diagonal_chart()
    claw_chart()
    research_next()
    sensor_cone()
    flow('flow_motion', ['Motion starts:\nread the\nposition', 'PID turns the\nerror into\nmotor volts', 'Within the\nthreshold?', 'Stayed there\nfor the settle\ntime?', 'Return\n"settled"'],
         fills={2: '#FFF9C4', 3: '#FFF9C4', 4: '#C8E6C9'})
    flow('flow_autotune', ['1. Ramp the\nvolts up slowly,\neach way', '2. Step to a\nfixed voltage,\neach way', '3. Fit kS, kV,\nkA and the\ndelay', '4. Place the\npoles for each\ncontroller', '5. Check phase\nmargin; slow\ndown if needed'],
         fills={2: '#E1BEE7', 3: '#E1BEE7', 4: '#C8E6C9'})
    flow('flow_telemetry', ['Control loop\ncopies a 64-byte\nrecord', 'Lock-free ring\n(drops if full,\nand counts it)', 'Writer task\nmerges rows\nby timestamp', 'One big write\nevery 250 ms', 'SD card:\nSL000042.CSV'],
         fills={0: '#C8E6C9', 4: '#E0E0E0'})
    flow('flow_tuning', ['Robot drives,\nlogs to SD', 'Analyzer or\nsimulator reads\nthe logs', 'Tools propose\nnew values', 'TUNE.CFG on\nthe SD card', 'Robot loads it\nat startup:\nno rebuild'],
         fills={3: '#FFF9C4', 4: '#C8E6C9'})
    flow('flow_macro', ['Read every\nbutton once\nper loop', 'First rule\nthat applies\nwins', 'Set lift level,\nclaw, intake', 'Mechanisms\nrun their\nown PID', 'Wait 10 ms,\nrepeat'],
         fills={1: '#FFF9C4'})
    flow('flow_autocycle', ['Back off the\nloader', 'Turn to the\ngoal', 'Drive up; creep\nthe last 2 in\nat 6 V', 'Driver presses\nscore', 'Drive back to\nthe loader'],
         fills={3: '#FFE0B2'})


if __name__ == '__main__':
    make_all({})
