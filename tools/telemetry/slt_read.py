#!/usr/bin/env python3
"""Reference reader for SapphireLib telemetry files (SLT v1). Standard library only.

The format is specified in docs/TELEMETRY_FORMAT.md; this file is the executable
version of that spec, for the tuning app to copy from or import.

    python slt_read.py SL000042.CSV      summary: channels, events, motions
    python slt_read.py --selftest        parse the embedded golden file and check it

As a module:

    import slt_read
    log = slt_read.read("SL000042.CSV")
    turn = log["turn"]                       # a channel, by name
    for motion in slt_read.motions(log):     # drivetrain motions, start to end
        steps = motion.rows(turn)            # the turn PID's steps during it
        print(motion.kind, motion.params, motion.reason, len(steps))

Team 96671H - Hitmen
"""
import sys
from dataclasses import dataclass, field

FIRST_STEP = 8  # pid flags bit: a new response starts at this row
CONFIG_KEYS = ("integral_limit", "output_limit", "slew_rate",
               "derivative_on_measurement", "nominal_dt_s")


@dataclass
class Channel:
    id: int
    name: str
    kind: str      # "samples", "pid", or "events"
    decimals: int
    columns: list  # after the implicit t_us
    rows: list = field(default_factory=list)    # (t_us, [floats]); pid flags is last
    gains: list = field(default_factory=list)   # (t_us, kP, kI, kD)
    config: dict = field(default_factory=dict)
    resets: list = field(default_factory=list)  # t_us of each R row
    dropped: tuple = (0, 0)                     # (full, contended), latest D row

    def column(self, name):
        """[(t_us, value)] for one column."""
        i = self.columns.index(name)
        return [(t, v[i]) for t, v in self.rows]


@dataclass
class Log:
    meta: dict = field(default_factory=dict)
    channels: dict = field(default_factory=dict)  # id -> Channel
    events: list = field(default_factory=list)    # (t_us, tag, message)
    health: list = field(default_factory=list)    # (t_us, {key: int})
    skipped: int = 0                              # malformed lines ignored
    last_us: int = 0                              # latest t_us of any row

    def __getitem__(self, name):
        """A channel by name -- how to find one across files, since ids can differ."""
        for c in self.channels.values():
            if c.name == name:
                return c
        raise KeyError(f"no channel named {name!r}")

    def get(self, name, default=None):
        try:
            return self[name]
        except KeyError:
            return default


def read(path):
    with open(path, "rb") as f:
        return parse(f.read())


def parse(data):
    """A Log from a whole file's bytes (or text)."""
    if isinstance(data, bytes):
        data = data.decode("ascii", "replace")
    lines = data.split("\n")
    # The last piece is "" after a final newline, or a line torn by power
    # loss. Either way it isn't a record.
    lines.pop()
    if not lines or lines[0].split(",")[:2] != ["#SLT", "1"]:
        raise ValueError("not an SLT v1 file")
    log = Log()
    for line in lines[1:]:
        try:
            _parse(log, line)
        except (ValueError, KeyError, IndexError):
            log.skipped += 1  # skip a bad line; never abort the file
    return log


def _stamp(log, t):
    t = int(t)
    log.last_us = max(log.last_us, t)
    return t


def _parse(log, line):
    kind, _, rest = line.partition(",")
    ch = log.channels
    if kind == "S":
        cid, t, *vals = rest.split(",")
        c = ch[int(cid)]
        if len(vals) != len(c.columns):
            raise ValueError("row width")
        c.rows.append((_stamp(log, t), [float(v) for v in vals]))
    elif kind == "G":
        cid, t, kp, ki, kd = rest.split(",")
        ch[int(cid)].gains.append((_stamp(log, t), float(kp), float(ki), float(kd)))
    elif kind == "C":
        cid, t, *vals = rest.split(",")
        if len(vals) != len(CONFIG_KEYS):
            raise ValueError("config width")
        _stamp(log, t)
        ch[int(cid)].config = dict(zip(CONFIG_KEYS, map(float, vals)))
    elif kind == "R":
        cid, t = rest.split(",")
        ch[int(cid)].resets.append(_stamp(log, t))
    elif kind == "E":
        t, tag, *msg = rest.split(",", 2)
        log.events.append((_stamp(log, t), tag, msg[0] if msg else ""))
    elif kind == "D":
        t, cid, full, contended = rest.split(",")
        _stamp(log, t)
        ch[int(cid)].dropped = (int(full), int(contended))
    elif kind == "H":
        t, *pairs = rest.split(",")
        values = {k: int(v) for k, v in (p.split("=", 1) for p in pairs)}
        log.health.append((_stamp(log, t), values))
    elif kind == "#chan":
        cid, name, ckind, decimals, *cols = rest.split(",")
        ch[int(cid)] = Channel(int(cid), name, ckind, int(decimals), cols)
    elif kind == "#meta":
        key, _, value = rest.partition(",")
        log.meta[key] = value
    # Anything else (future row types, other '#' lines) is skipped by design.


def responses(channel):
    """A pid channel's rows split into step responses, each starting at a FIRST_STEP row."""
    out = []
    for t, v in channel.rows:
        if not out or int(v[-1]) & FIRST_STEP:
            out.append([])
        out[-1].append((t, v))
    return out


def gains_at(channel, t_us):
    """(kP, kI, kD) in effect at t_us, or None if the file hasn't said yet."""
    current = None
    for t, kp, ki, kd in channel.gains:
        if t > t_us:
            break
        current = (kp, ki, kd)
    return current


# --- Motions ---------------------------------------------------------------------


@dataclass
class Motion:
    """One drivetrain motion, from its `motion,start` event to its `motion,end`."""
    kind: str          # driveDistance, turnToHeading, moveToPoint, moveToPose, followPath
    start_us: int
    params: dict       # the start event's keys: target_deg, threshold, settle_ms, ...
    end_us: int = None      # None if it never ended (see stop_us)
    reason: str = None      # "settled", "timeout", "aborted"; None if it never ended
    error: float = None     # final error: inches, or degrees for turnToHeading
    ms: int = None          # duration as the robot measured it
    stop_us: int = None     # where its rows stop: end_us, or where it was cut short
    depth: int = 0          # motions open around it when it started (0 today)

    @property
    def ended(self):
        return self.end_us is not None

    def rows(self, channel):
        """The channel's rows recorded while this motion ran, start and stop inclusive."""
        return [(t, v) for t, v in channel.rows if self.start_us <= t <= self.stop_us]


def _number(text):
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return text


def motions(log):
    """The log's motion events paired into Motions, in start order.

    Each `end` closes the most recent open `start` of the same kind. A motion
    with no `end` was cut short -- PROS deletes the autonomous task wherever it
    is on a competition state change -- and stops at the first `phase` event
    after it started (or at the end of the file). It stays open until then all
    the same, since a motion run from a task of your own survives a phase
    change and ends normally.
    """
    done = []
    stack = []    # open motions, innermost last
    cut = {}      # id(motion) -> t_us of the first phase change while it was open

    def close_cut(motion):
        motion.stop_us = cut.get(id(motion), log.last_us)
        done.append(motion)

    for t, tag, message in sorted(log.events, key=lambda e: e[0]):
        if tag == "phase":
            for motion in stack:
                cut.setdefault(id(motion), t)
            continue
        if tag != "motion":
            continue
        parts = message.split(",")
        if len(parts) < 2:
            continue
        what, kind = parts[0], parts[1]
        keys = dict(p.split("=", 1) for p in parts[2:] if "=" in p)
        if what == "start":
            stack.append(Motion(kind, t, {k: _number(v) for k, v in keys.items()},
                                depth=len(stack)))
        elif what == "end":
            match = next((i for i in range(len(stack) - 1, -1, -1) if stack[i].kind == kind),
                         None)
            if match is None:
                continue  # its start is in an earlier file (the card went in mid-motion)
            # Anything opened after it and still open never ended.
            while len(stack) > match + 1:
                close_cut(stack.pop())
            motion = stack.pop()
            motion.end_us = motion.stop_us = t
            motion.reason = keys.get("reason")
            motion.error = _number(keys["error"]) if "error" in keys else None
            motion.ms = _number(keys["ms"]) if "ms" in keys else None
            done.append(motion)
    while stack:
        close_cut(stack.pop())
    return sorted(done, key=lambda m: m.start_us)


# --- Characterization --------------------------------------------------------------


def characterization_segments(channel, gap_us=50_000):
    """A char.* channel's rows split into the runner's segments.

    Each segment is a list of (time_ms, volts, position) -- the fields of
    tuning::CharacterizationSample, time_ms counted from the segment's first
    row. Within a segment rows are one sample period apart; between segments
    the runner waits at least 100 ms, which is what `gap_us` detects. One
    Auto-Tune run is four segments: ramp forward, ramp back, step forward,
    step back. See docs/TELEMETRY_FORMAT.md, "Refitting an axis offline".
    """
    segments = []
    start = last = None
    for t, (volts, position) in channel.rows:
        if last is None or t - last > gap_us:
            segments.append([])
            start = t
        segments[-1].append((round((t - start) / 1000), volts, position))
        last = t
    return segments


# --- Self-test ---------------------------------------------------------------------

# Byte for byte the golden file in tests/telemetry/csv_format_test.cpp
# (kGoldenFile): the encoder is held to it there, and this reader here.
SELFTEST_FILE = (
    "#SLT,1\n"
    "#meta,writer,sapphirelib 0.1.0\n"
    "#meta,kernel,4.2.2\n"
    "#meta,build,Sep 27 2026 14:02:11\n"
    "#meta,robot,96671H\n"
    "#meta,file,SL000042.CSV\n"
    "#meta,dir,/usd/sl\n"
    "#meta,open_us,2104331\n"
    "#meta,clock,us_since_program_start\n"
    "#chan,0,sys,events,0\n"
    "#chan,1,events,events,0\n"
    "#chan,2,drive,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags\n"
    "#chan,3,turn,pid,4,target,meas,err,p,i,d,u_raw,out,dt,flags\n"
    "#chan,5,odom,samples,4,x,y,heading\n"
    "#chan,6,batt,samples,2,volts,pct\n"
    "#chan,7,lift,pid,3,target,meas,err,p,i,d,u_raw,out,dt,flags\n"
    "#chan,8,lift.act,samples,3,target,pos,volts,law\n"
    "E,2104502,file,open,SL000042.CSV\n"
    "E,2104502,phase,disabled,comp=1,field=1\n"
    "H,3104771,rows=4,bytes=1034,writes=1,wmax_us=21873,wavg_us=21873,drops=0,unlogged=2,"
    "resyncs=0,breaks=0,faults=0\n"
    "E,15003114,phase,autonomous,comp=1,field=1\n"
    "S,5,15003201,0,0,0\n"
    "S,6,15003201,12.61,87\n"
    "E,15003390,auton,start,Turn Testing\n"
    "E,15003400,motion,start,turnToHeading,target_deg=90.000,threshold=2.000,settle_ms=200,"
    "timeout_ms=3000\n"
    "C,3,15003511,0,12,0,0,0.01\n"
    "G,3,15003512,0.35,0,0.0002\n"
    "S,3,15003514,90,0,90,31.5,0,0,31.5,12,0.01,13\n"
    "S,5,15013203,0,0,0.12\n"
    "S,3,15013604,89.59,0,89.59,31.3565,0,-0.0082,31.3483,12,0.01,5\n"
    "S,3,15023690,88.63,0,88.63,31.0205,0,-0.0192,31.0013,12,0.01,5\n"
    "S,3,15612874,1.84,0,1.84,0.644,0,-0.0012,0.6428,0.6428,0.01,0\n"
    "E,15612900,motion,end,turnToHeading,reason=settled,error=0.412,ms=609\n"
    "E,16871020,motion,start,turnToHeading,target_deg=0.000,threshold=2.000,settle_ms=200,"
    "timeout_ms=3000\n"
    "R,3,16871021\n"
    "S,3,16871027,-89.97,0,-89.97,-31.4895,0,0,-31.4895,-12,0.01,13\n"
    "E,30001022,phase,opcontrol,comp=1,field=1\n"
    "C,7,31540205,0,12.7,0,1,0.02\n"
    "G,7,31540206,0.2,0,0.01\n"
    "S,7,31540210,150,0.37,149.63,29.926,0,0,29.926,12.7,0.02,13\n"
    "S,8,31540236,150,0.37,12,0\n"
    "S,5,31540301,23.4512,-11.0833,91.87\n"
    "S,7,31560198,150,3.12,146.88,29.376,0,-1.375,28.001,12.7,0.02,5\n"
    "S,8,31560221,150,3.12,12,0\n"
    "E,48220114,mark,driver\n"
    "D,61000044,1,0,1\n"
)


def _check(condition, what):
    # Not `assert`: python -O strips those, and this is a test.
    if not condition:
        raise AssertionError(what)


def _selftest():
    # A torn final line (power cut mid-write) must be discarded, not parsed.
    log = parse(SELFTEST_FILE + "S,5,61000100,1,2")
    _check(log.skipped == 0, f"no line should be skipped, {log.skipped} were")
    _check(log.meta["file"] == "SL000042.CSV" and log.meta["robot"] == "96671H", "meta")
    _check(log.meta["build"] == "Sep 27 2026 14:02:11", "meta value runs to end of line")
    _check([c.name for c in log.channels.values()] ==
           ["sys", "events", "drive", "turn", "odom", "batt", "lift", "lift.act"], "channels")
    _check(log.last_us == 61000044, "last_us")

    turn = log["turn"]
    _check(turn.kind == "pid" and turn.id == 3 and turn.decimals == 4, "turn schema")
    _check(len(turn.rows) == 5, "turn rows")
    _check(turn.rows[1] == (15013604, [89.59, 0.0, 89.59, 31.3565, 0.0, -0.0082, 31.3483, 12.0,
                                       0.01, 5.0]), "turn row values")
    _check(turn.gains == [(15003512, 0.35, 0.0, 0.0002)], "turn gains")
    _check(turn.config == {"integral_limit": 0.0, "output_limit": 12.0, "slew_rate": 0.0,
                           "derivative_on_measurement": 0.0, "nominal_dt_s": 0.01}, "turn config")
    _check(turn.resets == [16871021], "turn resets")
    _check([len(r) for r in responses(turn)] == [4, 1], "turn responses split at flag 8")
    _check(gains_at(turn, 15003511) is None, "no gains before the first G")
    _check(gains_at(turn, 16871027) == (0.35, 0.0, 0.0002), "gains_at")

    _check(log["lift"].config["derivative_on_measurement"] == 1.0, "lift config")
    _check(log["lift.act"].column("volts") == [(31540236, 12.0), (31560221, 12.0)], "column()")
    _check(log["odom"].rows[-1] == (31540301, [23.4512, -11.0833, 91.87]), "odom")
    _check(log["events"].dropped == (0, 1) and log["turn"].dropped == (0, 0), "D rows")
    _check(log.get("nope") is None, "get() of a missing channel")

    _check(len(log.events) == 9, "events")
    _check(log.events[3] == (15003390, "auton", "start,Turn Testing"), "event message keeps commas")
    _check(log.health == [(3104771, {"rows": 4, "bytes": 1034, "writes": 1, "wmax_us": 21873,
                                     "wavg_us": 21873, "drops": 0, "unlogged": 2, "resyncs": 0,
                                     "breaks": 0, "faults": 0})], "H row")

    found = motions(log)
    _check(len(found) == 2, "two motions")
    first, second = found
    _check(first.kind == "turnToHeading" and first.ended, "first motion ended")
    _check(first.params == {"target_deg": 90.0, "threshold": 2.0, "settle_ms": 200,
                            "timeout_ms": 3000}, "start keys")
    _check((first.reason, first.error, first.ms) == ("settled", 0.412, 609), "end keys")
    _check([t for t, _ in first.rows(turn)] == [15003514, 15013604, 15023690, 15612874],
           "rows sliced by motion")
    _check(not second.ended and second.reason is None, "second motion never ended")
    _check(second.stop_us == 30001022, "cut short at the phase change")
    _check([t for t, _ in second.rows(turn)] == [16871027], "rows of the cut-short motion")

    # A motion run from a task that survives a phase change still pairs with
    # its end; ends whose start is in an earlier file are ignored; nesting is
    # tolerated.
    extra = parse(
        "#SLT,1\n"
        "E,5,motion,end,driveDistance,reason=timeout,error=3.000,ms=2000\n"
        "E,10,motion,start,followPath,waypoints=3,lookahead_in=12.000,cruise_v=8.000,"
        "timeout_ms=0\n"
        "E,20,phase,disabled,comp=0,field=0\n"
        "E,30,motion,start,moveToPoint,x=1.000,y=2.000\n"
        "E,40,motion,end,moveToPoint,reason=settled,error=0.500,ms=10\n"
        "E,50,motion,end,followPath,reason=settled,error=0.500,ms=40\n"
        "E,60,motion,start,turnToHeading\n"
        "E,70,motion,end,turnToHeading,reason=aborted,error=0.000,ms=0\n")
    paired = motions(extra)
    _check([(m.kind, m.start_us, m.end_us, m.depth) for m in paired] ==
           [("followPath", 10, 50, 0), ("moveToPoint", 30, 40, 1), ("turnToHeading", 60, 70, 0)],
           "pairing across a phase change, with nesting")
    _check(paired[2].params == {} and paired[2].reason == "aborted", "aborted motion")

    # Characterization rows split at the runner's waits between segments.
    char = Channel(9, "char.fwd", "samples", 4, ["volts", "pos"],
                   rows=[(1_000_000, [0.0, 0.0]), (1_010_000, [0.4, 0.0]),
                         (1_020_000, [0.8, 0.1]), (1_300_000, [0.0, 3.0]),
                         (1_310_000, [-0.4, 3.0])])
    _check(characterization_segments(char) ==
           [[(0, 0.0, 0.0), (10, 0.4, 0.0), (20, 0.8, 0.1)], [(0, 0.0, 3.0), (10, -0.4, 3.0)]],
           "characterization segments")

    # Bad lines are counted and skipped; the rest of the file still reads.
    messy = parse("#SLT,1\n#chan,2,x,samples,2,a,b\nS,2,1,1\nS,9,2,1,2\nG,2,3\nS,2,4,1,2\n"
                  "Q,future,row\n#future,directive\n")
    _check(messy.skipped == 3 and messy["x"].rows == [(4, [1.0, 2.0])], "malformed lines")

    for bad in ("", "#SLT,2\n", "time,value\n1,2\n"):
        try:
            parse(bad)
        except ValueError:
            continue
        raise AssertionError(f"{bad!r} should be refused")

    print("slt_read selftest: all checks passed")


def _summary(path):
    log = read(path)
    print(f"{log.meta.get('file')} ({log.meta.get('robot')}), {log.skipped} lines skipped")
    for c in log.channels.values():
        extra = f", {len(responses(c))} responses" if c.kind == "pid" else ""
        print(f"  {c.name:12} {c.kind:8} {len(c.rows):6} rows{extra}, dropped {c.dropped}")
    for t, tag, msg in log.events:
        if tag != "motion":
            print(f"  {t / 1e6:9.3f}s  {tag}: {msg}")
    for m in motions(log):
        outcome = f"{m.reason}, error {m.error}, {m.ms} ms" if m.ended else "cut short"
        print(f"  {m.start_us / 1e6:9.3f}s  {m.kind}: {outcome}")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        _selftest()
    elif len(sys.argv) == 2:
        _summary(sys.argv[1])
    else:
        print(__doc__)
        sys.exit(2)
