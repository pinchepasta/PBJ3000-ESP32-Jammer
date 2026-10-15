#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Render doc/registers/<chip>.svd into doc/undocumented-registers-<chip>.md.

The page keeps its prose; the generator replaces what stands between
<!-- svd:summary --> or <!-- svd:registers --> and the next <!-- svd:end -->.
Conventions inside the SVD descriptions (Markdown text):
- peripheral: first line is the section title, the rest its introduction;
- register: first line is the summary, the rest notes; "Init: <value>" gives the
  value after PHY init for the bit-field table, "Evidence:" lines the sources;
- field: the description, then "Evidence:" lines.
Names render as Espressif's headers spell them: registers PREFIX_NAME_REG,
fields PREFIX_FIELD, the prefix being the peripheral's <prependToName> or its
name and an underscore. Unlisted bits show as "(unknown)", or as "(documented)" where
Espressif documents them: the register description's line "Other bits:
documented" names their ranges ("31:29, 22"), or all of them without any. Peripherals in group ANALOG_I2C are analog I2C blocks: base
address block << 8, one 8-bit register per offset, shown as block:register,
named PERIPHERAL_REGISTER.
--check: exit 1 if a page is not up to date.
"""

import re
import sys
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

DOC = Path(__file__).resolve().parent.parent
ACCESS = {"read-write": "R/W", "read-only": "RO", "write-only": "WO"}


def num(text):
    return int(text.strip(), 0)


def lines(elem, tag):
    text = elem.findtext(tag) or ""
    return textwrap.dedent(text.strip("\n")).strip().splitlines()


def split(desc):
    """Body lines, the Init value, the evidence (each a string), the documented bits' ranges."""
    body, init, evidence, cur, documented = [], None, [], None, None
    for line in desc:
        s = line.strip()
        if s.startswith("Other bits: documented"):
            documented = s[22:].strip()
            cur = None
        elif s.startswith("Init:"):
            init = s[5:].strip()
            cur = None
        elif s.startswith("Evidence:"):
            cur = [s[9:].strip()]
            evidence.append(cur)
        elif cur is not None and s:
            cur.append(s)
        else:
            cur = None
            body.append(line)
    return body, init, [" ".join(e) for e in evidence], documented


def anchor(title):
    s = re.sub(r"[^\w\- ]", "", title.lower())
    return s.replace(" ", "-")


def bits(f):
    if f.find("bitRange") is not None:
        msb, lsb = map(int, f.findtext("bitRange").strip("[]").split(":"))
    elif f.find("lsb") is not None:
        msb, lsb = num(f.findtext("msb")), num(f.findtext("lsb"))
    else:
        lsb = num(f.findtext("bitOffset"))
        msb = lsb + num(f.findtext("bitWidth")) - 1
    return msb, lsb


def label(msb, lsb):
    return str(msb) if msb == lsb else f"{msb}:{lsb}"


def bitset(ranges, size):
    """The bits of ranges like "31:29, 22", all of them for "", none for None."""
    if ranges is None:
        return set()
    if not ranges:
        return set(range(size))
    out = set()
    for part in ranges.split(","):
        msb, _, lsb = part.strip().partition(":")
        out |= set(range(int(lsb or msb), int(msb) + 1))
    return out


class Register:
    def __init__(self, r, periph, analog):
        self.desc, self.init, self.evidence, documented = split(lines(r, "description"))
        pname = periph.findtext("name")
        prefix = periph.findtext("prependToName") or f"{pname}_"
        size = r.findtext("size") or periph.findtext("size") or "32"
        self.size = 8 if analog else num(size)
        self.documented = bitset(documented, self.size)
        access = r.findtext("access") or periph.findtext("access") or "read-write"
        base, offset = num(periph.findtext("baseAddress")), num(r.findtext("addressOffset"))
        dim = num(r.findtext("dim")) if r.find("dim") is not None else 1
        inc = num(r.findtext("dimIncrement")) if dim > 1 else 0
        index = (r.findtext("dimIndex") or ",".join(map(str, range(dim)))).split(",")
        name = r.findtext("name")
        fmt = f"{pname}_{{}}" if analog else f"{prefix}{{}}_REG"
        self.names = [fmt.format(name.replace("%s", i)) for i in index[:dim]]
        addrs = [offset + k * inc for k in range(dim)]
        self.addrs = ([f"0x{base >> 8:02x}:{a}" for a in addrs] if analog else
                      [f"0x{base + a:08x}" for a in addrs])
        self.fields = []
        for f in r.iterfind("fields/field"):
            msb, lsb = bits(f)
            fdesc, _, fev, _ = split(lines(f, "description"))
            fa = ACCESS[f.findtext("access") or access]
            fname = (f"{pname}_" if analog else prefix) + f.findtext("name")
            self.fields.append((msb, lsb, fname, " ".join(fdesc), fa, fev))
        self.fields.sort(key=lambda x: -x[0])
        kinds = {x[4] for x in self.fields} or {ACCESS[access]}
        self.access = kinds.pop() if len(kinds) == 1 else "varies"
        self.title = f"Register {', '.join(self.names)} ({', '.join(self.addrs)})"

    def gaps(self, top, bottom):
        """Columns for the unlisted bits top..bottom, one per run of like bits."""
        cols = []
        for b in range(top, bottom - 1, -1):
            kind = "(documented)" if b in self.documented else "(unknown)"
            if cols and cols[-1][2] == kind:
                cols[-1] = (cols[-1][0], b, kind)
            else:
                cols.append((b, b, kind))
        return cols

    def table(self):
        cols, top = [], self.size - 1
        for msb, lsb, name, *_ in self.fields:
            cols += self.gaps(top, msb + 1)
            cols.append((msb, lsb, name))
            top = lsb - 1
        cols += self.gaps(top, 0)
        rows = [["Bits"] + [label(m, l) for m, l, _ in cols], ["Field"] + [n for *_, n in cols]]
        if self.init:
            value = num(self.init.split()[0].rstrip(",.;"))
            rows.append(["Init"])
            for m, l, _ in cols:
                v = (value >> l) & ((1 << (m - l + 1)) - 1)
                rows[-1].append(str(v) if m == l else f"0x{v:x}")
        out = ["| " + " | ".join(rows[0]) + " |", "|" + "---|" * len(rows[0])]
        return out + ["| " + " | ".join(r) + " |" for r in rows[1:]]

    def markdown(self):
        out = [f"#### {self.title}", "", self.desc[0], ""] + self.table() + [""]
        for _, _, name, desc, access, _ in self.fields:
            out += [f"**{name}** {desc} ({access})", ""]
        notes = "\n".join(self.desc[1:]).strip()
        if self.init and len(self.init.split(maxsplit=1)) > 1:
            notes = (f"Init: {self.init}\n\n" + notes).strip()
        if notes:
            out += [notes, ""]
        evidence = [f"- {e}" for e in self.evidence]
        evidence += [f"- {n}: {e}" for *_, n, _, _, fev in self.fields for e in fev]
        if evidence:
            out += ["Evidence:", ""] + evidence + [""]
        return out


def render(svd):
    root = ET.parse(svd).getroot()
    summary = ["| Name | Description | Address | Access |", "|---|---|---|---|"]
    body = []
    for p in root.iterfind("peripherals/peripheral"):
        analog = (p.findtext("groupName") or "") == "ANALOG_I2C"
        desc = lines(p, "description")
        summary.append(f"| **{desc[0]}** | | | |")
        body += [f"### {desc[0]}", ""]
        intro = "\n".join(desc[1:]).strip()
        if intro:
            body += [intro, ""]
        for r in p.iterfind("registers/register"):
            reg = Register(r, p, analog)
            link = f"[{', '.join(reg.names)}](#{anchor(reg.title)})"
            summary.append(f"| {link} | {reg.desc[0]} | {', '.join(reg.addrs)} | {reg.access} |")
            body += reg.markdown()
    return {"summary": summary, "registers": body}


def update(page, parts):
    text = page.read_text()
    for key, out in parts.items():
        pat = re.compile(rf"(<!-- svd:{key} -->\n).*?(<!-- svd:end -->)", re.S)
        if not pat.search(text):
            sys.exit(f"{page}: no <!-- svd:{key} --> region")
        block = "\n" + "\n".join(out).strip() + "\n\n"
        text = pat.sub(lambda m: m.group(1) + block + m.group(2), text)
    return text


def main():
    check = "--check" in sys.argv[1:]
    stale = []
    for svd in sorted(Path(__file__).parent.glob("*.svd")):
        page = DOC / f"undocumented-registers-{svd.stem}.md"
        text = update(page, render(svd))
        if text != page.read_text():
            stale.append(page.name)
            if not check:
                page.write_text(text)
    if check and stale:
        sys.exit("out of date, run doc/registers/svd2md.py: " + ", ".join(stale))
    return 0


if __name__ == "__main__":
    sys.exit(main())
