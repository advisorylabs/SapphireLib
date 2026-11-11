"""Page builder for the notebook's Section G.

A page is a header category, a title and a list of blocks laid out top to
bottom. build() writes the .pptx on the notebook's own page layout and, from
the same layout pass, a PNG preview of every page (Pillow), since this
machine has no PowerPoint or LibreOffice to render with.
"""
import copy
import os
import re

from lxml import etree
from PIL import Image, ImageDraw, ImageFont
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Inches, Pt

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, 'img')

# the notebook's colour code (page A-1)
CATEGORY = {
    'Identify the Problem': ('FF0000', '000000'),
    'Brainstorm and Pick Best Solution': ('FF9900', '000000'),
    'CAD/Design and Prototype': ('FFFF00', '000000'),
    'Build': ('00FFFF', '000000'),
    'Test': ('FFFFFF', '000000'),
    'Monthly Gantt Charts': ('B7B7B7', '000000'),
    'Research': ('FF00FF', '000000'),
    'Programming': ('00FF00', '000000'),
    'Team Management': ('4A86E8', 'FFFFFF'),
    'Robot Iterations': ('000000', 'FFFFFF'),
}

LEFT, WIDTH = 0.30, 7.90
TOP = 0.90
SRC_FLOOR = 10.20
GAP = 0.10
BODY_PT, HEAD_PT, TABLE_PT, CODE_PT, SRC_PT = 12.5, 15, 10.5, 9, 9
FONT = 'Roboto'
MONO = 'Courier New'

GREEN, RED, YELLOW, GRAY, BLUE = 'B6D7A8', 'F4C7C3', 'FFF2CC', 'D9D9D9', 'CFE2F3'


# ---------------------------------------------------------------- block constructors

def H(text, size=HEAD_PT):
    return ('h', text, size)


def P(text, size=BODY_PT):
    return ('p', text, size)


def B(items, size=BODY_PT):
    return ('b', items, size)


def IMGB(name, w=WIDTH, caption=None):
    return ('img', name, w, caption)


def CODE(text, size=CODE_PT):
    return ('code', text.strip('\n'), size)


def BOX(text, fill='EFEFEF', size=BODY_PT):
    return ('box', text, fill, size)


def TABLE(rows, widths=None, fills=None, size=TABLE_PT, bold_first_col=False):
    return ('table', rows, widths, fills or {}, size, bold_first_col)


def SIDE(name, img_w, blocks, img_left=False, caption=None):
    return ('side', name, img_w, blocks, img_left, caption)


def MATRIX(criteria, options, scores, winner_note=None, first='Criteria'):
    """Weighted decision matrix. criteria: [(name, weight)], scores[i][j]: option j on criterion i, /5.

    Each cell shows the score and (score x weight); the best score in a row is
    green and the worst red, so the table reads without the numbers.
    """
    rows = [[first, 'Weight'] + list(options)]
    fills = {(0, c): GRAY for c in range(len(options) + 2)}
    totals = [0.0] * len(options)
    for i, (name, weight) in enumerate(criteria):
        row = [name, str(weight)]
        best, worst = max(scores[i]), min(scores[i])
        for j, s in enumerate(scores[i]):
            totals[j] += s * weight
            row.append('%g  (%g)' % (s, s * weight))
            if best != worst:
                fills[(i + 1, j + 2)] = GREEN if s == best else RED if s == worst else YELLOW
        fills[(i + 1, 0)] = BLUE
        rows.append(row)
    last = len(rows)
    rows.append(['Weighted total', ''] + ['%g' % t for t in totals])
    top = max(totals)
    for j, t in enumerate(totals):
        fills[(last, j + 2)] = GREEN if t == top else 'FFFFFF'
    fills[(last, 0)] = GRAY
    fills[(last, 1)] = GRAY
    n = len(options)
    first_w = 2.35 if n >= 4 else 2.6
    widths = [first_w, 0.7] + [(WIDTH - first_w - 0.7) / n] * n
    return ('table', rows, widths, fills, TABLE_PT, True, last)


def matrix_totals(criteria, scores):
    n = len(scores[0])
    return [sum(scores[i][j] * criteria[i][1] for i in range(len(criteria))) for j in range(n)]


class Page:
    def __init__(self, category, title, blocks, sources, block='', author='Cody Paschal', key=None):
        assert category in CATEGORY, category
        self.category = category
        self.title = title
        self.blocks = blocks
        self.sources = sources
        self.block = block
        self.author = author
        self.key = key
        self.date = None
        self.number = None


# ---------------------------------------------------------------- text measuring

_fonts = {}


def _font(size_pt, bold=False, mono=False):
    key = (round(size_pt * 4), bold, mono)
    if key not in _fonts:
        name = ('courbd.ttf' if bold else 'cour.ttf') if mono else ('arialbd.ttf' if bold else 'arial.ttf')
        _fonts[key] = ImageFont.truetype(os.path.join('C:\\Windows\\Fonts', name), int(round(size_pt * 100 / 72 * 4)))
    return _fonts[key]


def _width_in(text, size_pt, bold=False, mono=False):
    # measured at 4x and scaled back; Roboto runs about as wide as Arial, plus a safety margin
    return _font(size_pt, bold, mono).getlength(text) / 400.0 * 1.04


def runs_of(text):
    """'a **bold** b' -> [('a ', False), ('bold', True), (' b', False)]"""
    out = []
    for i, part in enumerate(re.split(r'\*\*', text)):
        if part:
            out.append((part, i % 2 == 1))
    return out


def wrap(text, size_pt, width_in, mono=False):
    """Wrapped lines of rich text, each a list of (text, bold)."""
    lines = []
    for para in text.split('\n'):
        words = []
        for chunk, bold in runs_of(para):
            for w in re.findall(r'\S+\s*|\s+', chunk):
                words.append((w, bold))
        line, used = [], 0.0
        for w, bold in words:
            ww = _width_in(w.rstrip(), size_pt, bold, mono)
            if line and used + ww > width_in:
                lines.append(line)
                line, used = [], 0.0
            line.append((w, bold))
            used += _width_in(w, size_pt, bold, mono)
        lines.append(line)
    return lines


def line_h(size_pt):
    return size_pt * 1.22 / 72.0


def text_h(text, size_pt, width_in, mono=False):
    return len(wrap(text, size_pt, width_in, mono)) * line_h(size_pt)


def img_size(name, w):
    with Image.open(os.path.join(IMG, name)) as im:
        return w, w * im.height / im.width


# ---------------------------------------------------------------- layout

PAD = 0.05


def table_layout(block, width):
    rows, widths = block[1], block[2]
    size = block[4]
    ncol = len(rows[0])
    if widths is None:
        widths = [width / ncol] * ncol
    scale = width / sum(widths)
    widths = [w * scale for w in widths]
    heights = []
    for r, row in enumerate(rows):
        h = 0
        for c, cell in enumerate(row):
            h = max(h, text_h(str(cell), size, widths[c] - 0.16))
        heights.append(h + 0.11)
    return widths, heights


def block_h(block, width):
    kind = block[0]
    if kind == 'h':
        return text_h(block[1], block[2], width - 2 * PAD) + 0.04
    if kind == 'p':
        return text_h(block[1], block[2], width - 2 * PAD) + 0.03
    if kind == 'b':
        return sum(text_h(i, block[2], width - 2 * PAD - 0.25) + 0.035 for i in block[1])
    if kind == 'img':
        h = img_size(block[1], min(block[2], width))[1]
        return h + (text_h(block[3], 10, width) + 0.03 if block[3] else 0)
    if kind == 'code':
        return len(block[1].split('\n')) * line_h(block[2]) + 0.14
    if kind == 'box':
        return text_h(block[1], block[3], width - 0.24) + 0.16
    if kind == 'table':
        return sum(table_layout(block, width)[1])
    if kind == 'side':
        iw = block[2]
        ih = img_size(block[1], iw)[1] + (text_h(block[5], 10, iw) + 0.03 if block[5] else 0)
        tw = width - iw - 0.15
        th = sum(block_h(b, tw) + GAP for b in block[3]) - GAP
        return max(ih, th)
    raise ValueError(kind)


def src_box(page):
    """(top, height) of the sources box: it grows upward from the footer to fit its text."""
    h = max(0.42, text_h('**Sources:** ' + page.sources, SRC_PT, 7.05 - 0.16) + 0.1)
    return SRC_FLOOR - h, h


def page_bottom(page):
    return src_box(page)[0] - 0.08


def scaled(block, f):
    """The block with its text f times the size (tables and code a little less), pictures 4% per step."""
    kind = block[0]
    if kind in ('h', 'p', 'b'):
        return (kind, block[1], round(block[2] * f, 2))
    if kind == 'box':
        return (kind, block[1], block[2], round(block[3] * f, 2))
    if kind == 'img':
        return (kind, block[1], min(WIDTH, block[2] * (0.5 + 0.5 * f)), block[3])
    if kind == 'table':
        return block[:4] + (min(12.0, max(9.0, round(block[4] * (0.4 + 0.6 * f), 2))),) + block[5:]
    if kind == 'code':  # never grown: its longest line is already sized to the page
        return (kind, block[1], max(8.0, round(block[2] * min(1.0, 0.5 + 0.5 * f), 2)))
    if kind == 'side':
        return (kind, block[1], block[2] * min(1.0, 0.5 + 0.5 * f), [scaled(b, f) for b in block[3]], block[4], block[5])
    return block


def fill_out(page):
    """Grow a page's text toward the notebook's 15 pt body while it still fits, so a short page isn't half empty."""
    cap = page_bottom(page) - TOP
    for f in (1.2, 1.16, 1.12, 1.08, 1.04):
        blocks = [scaled(b, f) for b in page.blocks]
        if sum(block_h(b, WIDTH) for b in blocks) + GAP * (len(blocks) - 1) <= cap - 0.1:
            page.blocks = blocks
            return page
    return page


def paginate(page):
    """One page if it fits (shrinking its text down to 88% if that is all it takes), else balanced continuation pages."""
    cap = page_bottom(page) - TOP
    for f in (1.0, 0.96, 0.92, 0.88):
        blocks = [scaled(b, f) for b in page.blocks] if f < 1.0 else page.blocks
        if sum(block_h(b, WIDTH) for b in blocks) + GAP * (len(blocks) - 1) <= cap:
            page.blocks = blocks
            return [fill_out(page) if f == 1.0 else page]
    heights = [block_h(b, WIDTH) for b in page.blocks]
    n = len(heights)

    def cost(i, j):  # blocks i..j-1 on one page
        return sum(heights[i:j]) + GAP * (j - i - 1)

    for k in range(2, n + 1):
        # best[c][j]: smallest achievable tallest-page height putting blocks 0..j-1 on c pages
        INF = float('inf')
        best = [[INF] * (n + 1) for _ in range(k + 1)]
        cut = [[0] * (n + 1) for _ in range(k + 1)]
        best[0][0] = 0
        for c in range(1, k + 1):
            for j in range(1, n + 1):
                for i in range(c - 1, j):
                    if page.blocks[j - 1][0] == 'h' and j < n:
                        continue  # never leave a heading at the bottom of a page
                    h = cost(i, j)
                    if h > cap or best[c - 1][i] == INF:
                        continue
                    m = max(best[c - 1][i], h)
                    if m < best[c][j]:
                        best[c][j] = m
                        cut[c][j] = i
        if best[k][n] < INF:
            cuts = [n]
            j = n
            for c in range(k, 0, -1):
                j = cut[c][j]
                cuts.append(j)
            cuts.reverse()
            parts = []
            for a, b in zip(cuts, cuts[1:]):
                part = Page(page.category, page.title if not parts else page.title + ' (continued)', page.blocks[a:b],
                            page.sources, page.block, page.author, page.key if not parts else None)
                parts.append(fill_out(part))
            return parts
    raise ValueError('cannot fit page: ' + page.title)


def layout(page):
    """[(block, x, y, w, h)], and the y the page ends at."""
    out = []
    y = TOP
    for block in page.blocks:
        h = block_h(block, WIDTH)
        out.append((block, LEFT, y, WIDTH, h))
        y += h + GAP
    return out, y - GAP


# ---------------------------------------------------------------- pptx writing

def _rgb(hexs):
    return RGBColor.from_string(hexs)


def _textbox(slide, x, y, w, h, wrap_text=True):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = wrap_text
    tf.margin_left = tf.margin_right = Inches(PAD)
    tf.margin_top = tf.margin_bottom = Inches(0.01)
    return box, tf


def _add_runs(paragraph, text, size, mono=False, color='000000', bold_all=False):
    for chunk, bold in runs_of(text):
        r = paragraph.add_run()
        r.text = chunk
        r.font.size = Pt(size)
        r.font.name = MONO if mono else FONT
        r.font.bold = bool(bold or bold_all)
        r.font.color.rgb = _rgb(color)


def _fill_text(tf, text, size, mono=False, color='000000', bold_all=False, align=None):
    first = True
    for para in text.split('\n'):
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        if align:
            p.alignment = align
        _add_runs(p, para, size, mono, color, bold_all)


def _bullet(paragraph):
    pPr = paragraph._p.get_or_add_pPr()
    pPr.set('marL', str(int(0.25 * 914400)))
    pPr.set('indent', str(int(-0.18 * 914400)))
    bu = etree.SubElement(pPr, qn('a:buChar'))
    bu.set('char', '\u2022')


def _rect(slide, x, y, w, h, fill, line=None, line_w=0.75):
    shape = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    shape.shadow.inherit = False
    if fill is None:
        shape.fill.background()
    else:
        shape.fill.solid()
        shape.fill.fore_color.rgb = _rgb(fill)
    if line is None:
        shape.line.fill.background()
    else:
        shape.line.color.rgb = _rgb(line)
        shape.line.width = Pt(line_w)
    return shape


def _cell_border(cell, color='000000', width_pt=0.75):
    tcPr = cell._tc.get_or_add_tcPr()
    for tag in ('a:lnL', 'a:lnR', 'a:lnT', 'a:lnB'):
        ln = etree.SubElement(tcPr, qn(tag))
        ln.set('w', str(int(width_pt * 12700)))
        fill = etree.SubElement(ln, qn('a:solidFill'))
        etree.SubElement(fill, qn('a:srgbClr')).set('val', color)


def _write_block(slide, block, x, y, w, h):
    kind = block[0]
    if kind == 'h':
        _, tf = _textbox(slide, x, y, w, h)
        _fill_text(tf, block[1], block[2], bold_all=True)
    elif kind == 'p':
        _, tf = _textbox(slide, x, y, w, h)
        _fill_text(tf, block[1], block[2])
    elif kind == 'b':
        _, tf = _textbox(slide, x, y, w, h)
        for i, item in enumerate(block[1]):
            p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
            _bullet(p)
            p.space_after = Pt(2)
            _add_runs(p, item, block[2])
    elif kind == 'img':
        iw, ih = img_size(block[1], min(block[2], w))
        slide.shapes.add_picture(os.path.join(IMG, block[1]), Inches(x + (w - iw) / 2), Inches(y), Inches(iw), Inches(ih))
        if block[3]:
            _, tf = _textbox(slide, x, y + ih + 0.01, w, h - ih)
            _fill_text(tf, block[3], 10, color='444444', align=PP_ALIGN.CENTER)
    elif kind == 'code':
        shape = _rect(slide, x, y, w, h, 'F3F3F3', 'B7B7B7')
        tf = shape.text_frame
        tf.word_wrap = False
        tf.margin_left = Inches(0.08)
        tf.margin_top = tf.margin_bottom = Inches(0.05)
        tf.vertical_anchor = MSO_ANCHOR.TOP
        first = True
        for line in block[1].split('\n'):
            p = tf.paragraphs[0] if first else tf.add_paragraph()
            first = False
            p.alignment = PP_ALIGN.LEFT
            comment = line.find('//')
            parts = [(line, False)] if comment < 0 else [(line[:comment], False), (line[comment:], True)]
            for text, is_comment in parts:
                if not text:
                    continue
                r = p.add_run()
                r.text = text
                r.font.size = Pt(block[2])
                r.font.name = MONO
                r.font.bold = False
                r.font.color.rgb = _rgb('38761D' if is_comment else '000000')
    elif kind == 'box':
        shape = _rect(slide, x, y, w, h, block[2], '999999')
        tf = shape.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_right = Inches(0.1)
        tf.margin_top = tf.margin_bottom = Inches(0.06)
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        _fill_text(tf, block[1], block[3], align=PP_ALIGN.LEFT)
    elif kind == 'table':
        rows, fills, size, bold_first = block[1], block[3], block[4], block[5]
        total_row = block[6] if len(block) > 6 else None
        widths, heights = table_layout(block, w)
        shape = slide.shapes.add_table(len(rows), len(rows[0]), Inches(x), Inches(y), Inches(w), Inches(sum(heights)))
        tbl = shape.table
        tblPr = tbl._tbl.tblPr
        for attr in ('firstRow', 'bandRow'):
            tblPr.set(attr, '0')
        style = tblPr.find(qn('a:tableStyleId'))
        if style is not None:
            style.text = '{5940675A-B579-460E-94D1-54222C63F5DA}'  # no style, table grid
        for c, cw in enumerate(widths):
            tbl.columns[c].width = Inches(cw)
        for r, row in enumerate(rows):
            tbl.rows[r].height = Inches(heights[r])
            for c, value in enumerate(row):
                cell = tbl.cell(r, c)
                cell.margin_left = cell.margin_right = Inches(0.06)
                cell.margin_top = cell.margin_bottom = Inches(0.03)
                cell.vertical_anchor = MSO_ANCHOR.MIDDLE
                fill = fills.get((r, c), GRAY if r == 0 else 'FFFFFF')
                cell.fill.solid()
                cell.fill.fore_color.rgb = _rgb(fill)
                _cell_border(cell)
                bold = r == 0 or (bold_first and c == 0) or r == total_row
                tf = cell.text_frame
                tf.word_wrap = True
                _fill_text(tf, str(value), size, bold_all=bold,
                           align=PP_ALIGN.LEFT if (c == 0 or len(str(value)) > 28) else PP_ALIGN.CENTER)
    elif kind == 'side':
        iw = block[2]
        iw, ih = img_size(block[1], iw)
        tw = w - iw - 0.15
        ix = x if block[4] else x + tw + 0.15
        tx = x + iw + 0.15 if block[4] else x
        slide.shapes.add_picture(os.path.join(IMG, block[1]), Inches(ix), Inches(y), Inches(iw), Inches(ih))
        if block[5]:
            _, tf = _textbox(slide, ix, y + ih + 0.01, iw, 0.3)
            _fill_text(tf, block[5], 10, color='444444', align=PP_ALIGN.CENTER)
        yy = y
        for b in block[3]:
            bh = block_h(b, tw)
            _write_block(slide, b, tx, yy, tw, bh)
            yy += bh + GAP


def _set_placeholder(slide, idx, text):
    for ph in slide.placeholders:
        if ph.placeholder_format.idx == idx:
            ph.text_frame.text = text
            return
    raise KeyError(idx)


def write_page(prs, page):
    layout_ = next(l for l in prs.slide_layouts if l.name == 'ONE_COLUMN_TEXT')
    slide = prs.slides.add_slide(layout_)
    _set_placeholder(slide, 0, 'SapphireLib: ' + page.title)
    _set_placeholder(slide, 1, page.author)
    _set_placeholder(slide, 2, page.date)
    _set_placeholder(slide, 3, page.number)
    for ph in list(slide.placeholders):
        if ph.placeholder_format.idx == 4:
            ph._element.getparent().remove(ph._element)

    fill, ink = CATEGORY[page.category]
    bar = _rect(slide, 0.24, 0.26, 8.02, 0.51, fill, '000000', 1)
    tf = bar.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    _fill_text(tf, page.category, 22, color=ink, bold_all=True, align=PP_ALIGN.CENTER)

    placed, end = layout(page)
    for block, x, y, w, h in placed:
        _write_block(slide, block, x, y, w, h)

    # sources: a red box with red text, to be formatted into IEEE citations later
    src_top, src_h = src_box(page)
    src = _rect(slide, LEFT, src_top, 7.05, src_h, 'FFFFFF', 'FF0000', 1.5)
    stf = src.text_frame
    stf.word_wrap = True
    stf.margin_left = stf.margin_right = Inches(0.06)
    stf.margin_top = stf.margin_bottom = Inches(0.03)
    stf.vertical_anchor = MSO_ANCHOR.MIDDLE
    _fill_text(stf, '**Sources:** ' + page.sources, SRC_PT, color='FF0000', align=PP_ALIGN.LEFT)
    _, itf = _textbox(slide, 7.42, SRC_FLOOR - 0.52, 0.8, 0.5)
    _fill_text(itf, 'CP', 24, bold_all=True, align=PP_ALIGN.CENTER)
    return end


def write_divider(prs, line1, line2):
    layout_ = next(l for l in prs.slide_layouts if l.name == 'CUSTOM_2')
    slide = prs.slides.add_slide(layout_)
    tf = slide.shapes.title.text_frame
    tf.text = line1
    tf.add_paragraph().text = line2


# ---------------------------------------------------------------- PNG preview

S = 100  # px per inch


def _draw_text(d, text, x, y, size, width, mono=False, color='#000000', bold_all=False):
    font_r = ImageFont.truetype('C:\\Windows\\Fonts\\' + ('cour.ttf' if mono else 'arial.ttf'), int(size * S / 72))
    font_b = ImageFont.truetype('C:\\Windows\\Fonts\\' + ('courbd.ttf' if mono else 'arialbd.ttf'), int(size * S / 72))
    for i, line in enumerate(wrap(text, size, width, mono)):
        cx = x * S
        for chunk, bold in line:
            f = font_b if (bold or bold_all) else font_r
            d.text((cx, (y + i * line_h(size)) * S), chunk, font=f, fill=color)
            cx += f.getlength(chunk)


def _preview_block(im, d, block, x, y, w, h):
    kind = block[0]
    if kind == 'h':
        _draw_text(d, block[1], x + PAD, y, block[2], w - 2 * PAD, bold_all=True)
    elif kind == 'p':
        _draw_text(d, block[1], x + PAD, y, block[2], w - 2 * PAD)
    elif kind == 'b':
        yy = y
        for item in block[1]:
            d.text(((x + PAD + 0.07) * S, yy * S), '\u2022', font=ImageFont.truetype('C:\\Windows\\Fonts\\arial.ttf', int(block[2] * S / 72)), fill='#000000')
            _draw_text(d, item, x + PAD + 0.25, yy, block[2], w - 2 * PAD - 0.25)
            yy += text_h(item, block[2], w - 2 * PAD - 0.25) + 0.035
    elif kind == 'img':
        iw, ih = img_size(block[1], min(block[2], w))
        with Image.open(os.path.join(IMG, block[1])) as pic:
            pic = pic.convert('RGB').resize((int(iw * S), int(ih * S)))
            im.paste(pic, (int((x + (w - iw) / 2) * S), int(y * S)))
        if block[3]:
            _draw_text(d, block[3], x, y + ih + 0.01, 10, w, color='#444444')
    elif kind == 'code':
        d.rectangle([x * S, y * S, (x + w) * S, (y + h) * S], fill='#F3F3F3', outline='#B7B7B7')
        font = ImageFont.truetype('C:\\Windows\\Fonts\\cour.ttf', int(block[2] * S / 72))
        for i, line in enumerate(block[1].split('\n')):
            d.text(((x + 0.08) * S, (y + 0.05 + i * line_h(block[2])) * S), line, font=font, fill='#000000')
            if font.getlength(line) > (w - 0.12) * S:
                d.rectangle([(x + w) * S - 6, (y + 0.05 + i * line_h(block[2])) * S, (x + w) * S, (y + 0.05 + (i + 1) * line_h(block[2])) * S], fill='#FF0000')
    elif kind == 'box':
        d.rectangle([x * S, y * S, (x + w) * S, (y + h) * S], fill='#' + block[2], outline='#999999')
        _draw_text(d, block[1], x + 0.12, y + 0.08, block[3], w - 0.24)
    elif kind == 'table':
        rows, fills, size, bold_first = block[1], block[3], block[4], block[5]
        total_row = block[6] if len(block) > 6 else None
        widths, heights = table_layout(block, w)
        yy = y
        for r, row in enumerate(rows):
            xx = x
            for c, value in enumerate(row):
                fill = fills.get((r, c), GRAY if r == 0 else 'FFFFFF')
                d.rectangle([xx * S, yy * S, (xx + widths[c]) * S, (yy + heights[r]) * S], fill='#' + fill, outline='#000000')
                bold = r == 0 or (bold_first and c == 0) or r == total_row
                _draw_text(d, str(value), xx + 0.08, yy + 0.05, size, widths[c] - 0.16, bold_all=bold)
                xx += widths[c]
            yy += heights[r]
    elif kind == 'side':
        iw, ih = img_size(block[1], block[2])
        tw = w - iw - 0.15
        ix = x if block[4] else x + tw + 0.15
        tx = x + iw + 0.15 if block[4] else x
        with Image.open(os.path.join(IMG, block[1])) as pic:
            pic = pic.convert('RGB').resize((int(iw * S), int(ih * S)))
            im.paste(pic, (int(ix * S), int(y * S)))
        if block[5]:
            _draw_text(d, block[5], ix, y + ih + 0.01, 10, iw, color='#444444')
        yy = y
        for b in block[3]:
            bh = block_h(b, tw)
            _preview_block(im, d, b, tx, yy, tw, bh)
            yy += bh + GAP


def preview(page, path):
    im = Image.new('RGB', (int(8.5 * S), 11 * S), '#FFFFFF')
    d = ImageDraw.Draw(im)
    fill, ink = CATEGORY[page.category]
    d.rectangle([0.24 * S, 0.26 * S, 8.26 * S, 0.77 * S], fill='#' + fill, outline='#000000')
    f = ImageFont.truetype('C:\\Windows\\Fonts\\arialbd.ttf', int(22 * S / 72))
    tw = f.getlength(page.category)
    d.text((4.25 * S - tw / 2, 0.35 * S), page.category, font=f, fill='#' + ink)
    placed, end = layout(page)
    for block, x, y, w, h in placed:
        _preview_block(im, d, block, x, y, w, h)
    src_top, src_h = src_box(page)
    d.rectangle([LEFT * S, src_top * S, (LEFT + 7.05) * S, (src_top + src_h) * S], outline='#FF0000', width=2)
    _draw_text(d, '**Sources:** ' + page.sources, LEFT + 0.08, src_top + 0.05, SRC_PT, 7.05 - 0.16, color='#FF0000')
    _draw_text(d, 'CP', 7.6, SRC_FLOOR - 0.46, 24, 1, bold_all=True)
    small = ImageFont.truetype('C:\\Windows\\Fonts\\arial.ttf', int(11 * S / 72))
    d.text((0.75 * S, 10.27 * S), 'SapphireLib: ' + page.title, font=small, fill='#000000')
    d.text((0.75 * S, 10.53 * S), page.author, font=small, fill='#000000')
    d.text((4.76 * S, 10.53 * S), page.date, font=small, fill='#000000')
    d.text((7.9 * S, 10.53 * S), page.number, font=small, fill='#000000')
    im.save(path)
    return end


def build(pages, template, out_path, preview_dir, divider=('Section G', 'SapphireLib Development')):
    prs = Presentation(template)
    write_divider(prs, *divider)
    os.makedirs(preview_dir, exist_ok=True)
    report = []
    for page in pages:
        end = write_page(prs, page)
        preview(page, os.path.join(preview_dir, page.number + '.png'))
        report.append((page.number, page.date, page.category, page.title, end, page_bottom(page)))
    prs.save(out_path)
    return report
