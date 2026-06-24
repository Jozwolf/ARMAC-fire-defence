#!/usr/bin/env python3
"""Convert ios_shortcut_instructions.md to Word (.docx) and RTF."""

from pathlib import Path
from docx import Document
from docx.shared import Pt, RGBColor
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
import re

SRC = Path(__file__).parent / "ios_shortcut_instructions.md"
DOCX_OUT = Path(__file__).parent / "iPad_to_DEVONthink_Guide.docx"
RTF_OUT  = Path(__file__).parent / "iPad_to_DEVONthink_Guide.rtf"


# ── Word export ────────────────────────────────────────────────────────────────

def add_hyperlink_run(para, text):
    """Add plain bold run (no real hyperlinks needed here)."""
    run = para.add_run(text)
    run.bold = True

def make_docx(src: Path, out: Path) -> None:
    doc = Document()

    # Default body font
    style = doc.styles["Normal"]
    style.font.name = "Calibri"
    style.font.size = Pt(11)

    lines = src.read_text().splitlines()
    i = 0
    table_buffer: list[list[str]] = []

    def flush_table():
        nonlocal table_buffer
        if not table_buffer:
            return
        # strip separator rows (---|---...)
        rows = [r for r in table_buffer if not re.match(r"^\|[-| :]+\|$", r[0].strip() if r else "")]
        if not rows:
            table_buffer = []
            return
        col_count = max(len(r) for r in rows)
        tbl = doc.add_table(rows=len(rows), cols=col_count)
        tbl.style = "Table Grid"
        for ri, row_cells in enumerate(rows):
            for ci, cell_text in enumerate(row_cells):
                cell = tbl.cell(ri, ci)
                cell.text = cell_text.strip()
                if ri == 0:
                    for run in cell.paragraphs[0].runs:
                        run.bold = True
        doc.add_paragraph()
        table_buffer = []

    in_code_block = False
    code_lines: list[str] = []

    while i < len(lines):
        line = lines[i]

        # Code block
        if line.startswith("```"):
            if not in_code_block:
                in_code_block = True
                code_lines = []
            else:
                in_code_block = False
                flush_table()
                p = doc.add_paragraph()
                p.style = doc.styles["Normal"]
                run = p.add_run("\n".join(code_lines))
                run.font.name = "Courier New"
                run.font.size = Pt(9)
                run.font.color.rgb = RGBColor(0x20, 0x20, 0x20)
            i += 1
            continue

        if in_code_block:
            code_lines.append(line)
            i += 1
            continue

        # Table row
        if line.startswith("|"):
            cells = [c for c in line.split("|")[1:-1]]
            table_buffer.append(cells)
            i += 1
            continue
        else:
            flush_table()

        # Headings
        if line.startswith("# "):
            doc.add_heading(line[2:], level=1)
        elif line.startswith("## "):
            doc.add_heading(line[3:], level=2)
        elif line.startswith("### "):
            doc.add_heading(line[4:], level=3)
        elif line.startswith("---"):
            doc.add_paragraph()
        elif re.match(r"^\d+\.", line):
            p = doc.add_paragraph(style="List Number")
            _add_inline(p, line[line.index(".")+2:])
        elif line.startswith("- "):
            p = doc.add_paragraph(style="List Bullet")
            _add_inline(p, line[2:])
        elif line.strip() == "":
            doc.add_paragraph()
        else:
            p = doc.add_paragraph()
            _add_inline(p, line)

        i += 1

    flush_table()
    doc.save(str(out))
    print(f"Saved Word doc: {out}")


def _add_inline(para, text: str) -> None:
    """Parse **bold** and `code` inline spans and add runs."""
    pattern = re.compile(r"(\*\*(.+?)\*\*|`(.+?)`)")
    pos = 0
    for m in pattern.finditer(text):
        if m.start() > pos:
            para.add_run(text[pos:m.start()])
        if m.group(0).startswith("**"):
            run = para.add_run(m.group(2))
            run.bold = True
        else:
            run = para.add_run(m.group(3))
            run.font.name = "Courier New"
            run.font.size = Pt(10)
        pos = m.end()
    if pos < len(text):
        para.add_run(text[pos:])


# ── RTF export ─────────────────────────────────────────────────────────────────

def md_to_rtf(src: Path, out: Path) -> None:
    lines = src.read_text().splitlines()
    rtf: list[str] = []

    rtf.append(r"{\rtf1\ansi\deff0")
    rtf.append(r"{\fonttbl{\f0 Calibri;}{\f1 Courier New;}}")
    rtf.append(r"{\colortbl ;\red0\green0\blue0;\red50\green50\blue50;}")
    rtf.append(r"\f0\fs22\cf1")

    in_code = False
    in_table = False

    for line in lines:
        if line.startswith("```"):
            in_code = not in_code
            if in_code:
                rtf.append(r"\f1\fs18\cf2")
            else:
                rtf.append(r"\f0\fs22\cf1\par")
            continue

        if in_code:
            rtf.append(_rtf_escape(line) + r"\line")
            continue

        if line.startswith("|"):
            # Render table rows as tab-separated lines
            cells = [c.strip() for c in line.split("|")[1:-1]]
            if re.match(r"^[-| :]+$", line.replace("|", "")):
                continue  # skip separator
            rtf.append(r"\b " + "  |  ".join(_rtf_escape(c) for c in cells) + r"\b0\par")
            continue

        if line.startswith("# "):
            rtf.append(r"\pard\sb240\sa60\b\fs32 " + _rtf_escape(line[2:]) + r"\b0\par")
        elif line.startswith("## "):
            rtf.append(r"\pard\sb200\sa60\b\fs28 " + _rtf_escape(line[3:]) + r"\b0\par")
        elif line.startswith("### "):
            rtf.append(r"\pard\sb160\sa40\b\fs24 " + _rtf_escape(line[4:]) + r"\b0\par")
        elif line.startswith("---"):
            rtf.append(r"\pard\brdrb\brdrs\brdrw10\brdr0\par")
        elif re.match(r"^\d+\.", line):
            content = line[line.index(".")+2:]
            rtf.append(r"\pard\li360\fi-360 " + _rtf_inline(content) + r"\par")
        elif line.startswith("- "):
            rtf.append(r"\pard\li360\fi-360\bullet  " + _rtf_inline(line[2:]) + r"\par")
        elif line.strip() == "":
            rtf.append(r"\par")
        else:
            rtf.append(r"\pard " + _rtf_inline(line) + r"\par")

    rtf.append("}")
    out.write_text("\n".join(rtf))
    print(f"Saved RTF: {out}")


def _rtf_escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace("{", r"\{").replace("}", r"\}")


def _rtf_inline(text: str) -> str:
    result = []
    pattern = re.compile(r"(\*\*(.+?)\*\*|`(.+?)`)")
    pos = 0
    for m in pattern.finditer(text):
        if m.start() > pos:
            result.append(_rtf_escape(text[pos:m.start()]))
        if m.group(0).startswith("**"):
            result.append(r"\b " + _rtf_escape(m.group(2)) + r"\b0")
        else:
            result.append(r"\f1\fs18 " + _rtf_escape(m.group(3)) + r"\f0\fs22")
        pos = m.end()
    if pos < len(text):
        result.append(_rtf_escape(text[pos:]))
    return "".join(result)


if __name__ == "__main__":
    make_docx(SRC, DOCX_OUT)
    md_to_rtf(SRC, RTF_OUT)
    print("Done.")
