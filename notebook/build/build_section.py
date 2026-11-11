"""Build Section G of the notebook: SapphireLib Development.

    node notebook/build/experiments.js        (once, or when the library changes: writes data/)
    python notebook/build/build_section.py    -> notebook/SapphireLib_Section_G.pptx
                                                 notebook/preview/G-*.png

Pages come from pages_a/b/c.py in order. A page with more than a sheet's worth
of blocks is split into balanced "(continued)" pages. Dates are spread evenly:
the research pages across 06/01 to 06/15, everything after from 06/16 to
11/11, moved off weekends. Page numbers and [[key]] cross references are
filled in here.
"""
import datetime as dt
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import charts  # noqa: E402
import nb  # noqa: E402
import pages_a  # noqa: E402
import pages_b  # noqa: E402
import pages_c  # noqa: E402

RESEARCH = 'Research and decision'
START, RESEARCH_END, CODE_START, TODAY = dt.date(2026, 6, 1), dt.date(2026, 6, 15), dt.date(2026, 6, 16), dt.date(2026, 11, 11)


def weekday(d):
    if d.weekday() == 5:
        return d - dt.timedelta(days=1)
    if d.weekday() == 6:
        return d + dt.timedelta(days=1)
    return d


def spread(n, first, last):
    if n == 1:
        return [first]
    span = (last - first).days
    return [weekday(first + dt.timedelta(days=round(i * span / (n - 1)))) for i in range(n)]


def assign(pages):
    research = [p for p in pages if p.block == RESEARCH]
    rest = [p for p in pages if p.block != RESEARCH]
    dates = spread(len(research), START, RESEARCH_END) + spread(len(rest), CODE_START, TODAY)
    dates[-1] = TODAY
    for i in range(1, len(dates)):
        dates[i] = max(dates[i], dates[i - 1])
    for i, (p, d) in enumerate(zip(research + rest, dates)):
        p.day = d
        p.date = d.strftime('%m/%d/%y')
        p.number = 'G-%d' % (i + 1)


def resolve(value, keys):
    if isinstance(value, str):
        def sub(m):
            if m.group(1) not in keys:
                raise KeyError('no page with key ' + m.group(1))
            return keys[m.group(1)]
        return re.sub(r'\[\[(\w+)\]\]', sub, value)
    if isinstance(value, list):
        return [resolve(v, keys) for v in value]
    if isinstance(value, tuple):
        return tuple(resolve(v, keys) for v in value)
    return value


def main():
    source = pages_a.PAGES + pages_b.PAGES + pages_c.PAGES
    if '--no-charts' not in sys.argv:
        charts.make_all({})  # every image must exist before pages can be measured

    # page numbers shift cross references by at most a character, so one pass of numbering on
    # placeholder references, then the real ones
    def numbered(keys):
        pages = []
        for p in source:
            q = nb.Page(p.category, p.title, resolve(p.blocks, keys), resolve(p.sources, keys), p.block, p.author, p.key)
            pages.extend(nb.paginate(q))
        assign(pages)
        return pages

    class Guess(dict):
        def __contains__(self, key):
            return True

        def __getitem__(self, key):
            return 'G-00'

    pages = numbered(Guess())
    for _ in range(3):
        keys = {p.key: p.number for p in pages if p.key}
        pages = numbered(keys)
        if {p.key: p.number for p in pages if p.key} == keys:
            break

    blocks = {}
    for p in pages:
        first, last = blocks.get(p.block, (p.day, p.day))
        blocks[p.block] = (min(first, p.day), max(last, p.day))
    mid = next(p.day for p in pages if p.key == 'gantt_mid')
    if '--no-charts' not in sys.argv:
        charts.gantts(blocks, mid, next(p.day for p in pages if p.key == 'gantt_final'))

    out = os.path.join(HERE, '..', 'SapphireLib_Section_G.pptx')
    preview_dir = os.path.join(HERE, '..', 'preview')
    if os.path.isdir(preview_dir):
        for name in os.listdir(preview_dir):
            os.remove(os.path.join(preview_dir, name))
    report = nb.build(pages, os.path.join(HERE, 'template.pptx'), out, preview_dir)
    bad = 0
    for number, date, category, title, end, bottom in report:
        room = bottom - end
        flag = 'OVER ' if room < -0.005 else ('thin ' if room > 3.0 else '     ')
        bad += room < -0.005
        print('%s %-5s %s  %-34s %-58s room %+.2f' % (flag, number, date, category, title[:58], room))
    print('%d pages, %d overflow' % (len(pages), bad))
    for name, (a, b) in blocks.items():
        print('  %-36s %s to %s' % (name, a.strftime('%m/%d'), b.strftime('%m/%d')))


if __name__ == '__main__':
    main()
