"""Strip every slide out of the team notebook, leaving its master, layouts and theme.

    python make_template.py "<notebook.pptx>" template.pptx

The result is what build_section.py adds pages to, so new pages carry the
notebook's own page frame (title, author, date and page-number placeholders).
"""
import sys
from pptx import Presentation

src, dst = sys.argv[1], sys.argv[2]
prs = Presentation(src)
ids = prs.slides._sldIdLst
for sld in list(ids):
    prs.part.drop_rel(sld.rId)
    ids.remove(sld)
prs.save(dst)

prs = Presentation(dst)
print('slides left:', len(prs.slides))
for i, layout in enumerate(prs.slide_layouts):
    if layout.name in ('ONE_COLUMN_TEXT', 'CUSTOM_2'):
        print(i, layout.name)
        for ph in layout.placeholders:
            f = ph.placeholder_format
            print('   idx', f.idx, f.type, ph.name,
                  round(ph.left / 914400, 2), round(ph.top / 914400, 2),
                  round(ph.width / 914400, 2), round(ph.height / 914400, 2))
