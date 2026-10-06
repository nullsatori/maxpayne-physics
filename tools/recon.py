#!/usr/bin/env python3
"""Inventory a game install (Max Payne 1 or 3) for modding work.

Reads the game directory without modifying it and writes two files:
  recon.json  full data (exports, imports, sections, file list)
  recon.md    short summary to paste into chat

Only metadata is collected (names, sizes, hashes, symbol names); no game
code or assets are copied. Requires Python 3.8+, standard library only.

Usage:
  python tools/recon.py "C:\\Program Files (x86)\\Max Payne" --out recon\\mp1
"""

import argparse
import collections
import hashlib
import json
import math
import os
import re
import struct
import sys

PE_EXTENSIONS = {".exe", ".dll", ".asi", ".flt", ".ocx"}

# Names Ultimate ASI Loader (x86) can be installed as. A name only works
# if the game actually loads it, so the report cross-checks exe imports.
ASI_LOADER_NAMES = {
    "dinput.dll", "dinput8.dll", "dsound.dll", "d3d8.dll", "d3d9.dll",
    "d3d10.dll", "d3d11.dll", "d3d12.dll", "ddraw.dll", "version.dll",
    "winmm.dll", "wininet.dll", "winhttp.dll", "xinput1_3.dll",
}

# SafeDisc / SecuROM leave recognizable files and section names.
DRM_FILES = {
    "00000001.tmp": "SafeDisc", "clcd16.dll": "SafeDisc", "clcd32.dll": "SafeDisc",
    "clokspl.exe": "SafeDisc", "dplayerx.dll": "SafeDisc", "drvmgt.dll": "SafeDisc",
    "secdrv.sys": "SafeDisc", "cms16.dll": "SecuROM", "cms_95.dll": "SecuROM",
    "cms_nt.dll": "SecuROM",
}
DRM_SECTION_PATTERNS = [
    (re.compile(r"^\.stxt\d+$"), "SafeDisc"),
    (re.compile(r"^stxt\d+$"), "SafeDisc"),
    (re.compile(r"^\.securom$"), "SecuROM"),
    (re.compile(r"^\.cms_[td]$"), "SecuROM"),
]

# Strings worth locating before reverse engineering character code.
KEYWORDS = [
    "ragdoll", "havok", "physics", "skeleton", "bone", ".kf2", ".ldb",
    "bullettime", "bullet time", "shootdodge", "dodge", "death",
    "animation", "collision", "hitbox", "impulse",
]

MACHINES = {0x14C: "i386", 0x8664: "x86_64", 0xAA64: "arm64", 0x1C4: "armnt"}


class PEError(Exception):
    pass


class PE:
    def __init__(self, data):
        self.data = data
        if data[:2] != b"MZ":
            raise PEError("no MZ header")
        e_lfanew = self.u32(0x3C)
        if data[e_lfanew:e_lfanew + 4] != b"PE\0\0":
            raise PEError("no PE signature")
        coff = e_lfanew + 4
        (self.machine, nsections, self.timestamp, _, _, opt_size,
         self.characteristics) = struct.unpack_from("<HHIIIHH", data, coff)
        opt = coff + 20
        self.magic = self.u16(opt)
        if self.magic == 0x10B:
            self.is64 = False
            self.entry = self.u32(opt + 16)
            self.image_base = self.u32(opt + 28)
            self.image_size = self.u32(opt + 56)
            ndirs = self.u32(opt + 92)
            dirs = opt + 96
        elif self.magic == 0x20B:
            self.is64 = True
            self.entry = self.u32(opt + 16)
            self.image_base = struct.unpack_from("<Q", data, opt + 24)[0]
            self.image_size = self.u32(opt + 56)
            ndirs = self.u32(opt + 108)
            dirs = opt + 112
        else:
            raise PEError("unknown optional header magic 0x%x" % self.magic)
        self.dirs = [struct.unpack_from("<II", data, dirs + 8 * i)
                     for i in range(min(ndirs, 16))]
        self.sections = []
        sec = opt + opt_size
        for i in range(nsections):
            raw = data[sec + 40 * i: sec + 40 * i + 40]
            name = raw[:8].rstrip(b"\0").decode("latin-1")
            vsize, va, rsize, rptr = struct.unpack_from("<IIII", raw, 8)
            chars = struct.unpack_from("<I", raw, 36)[0]
            self.sections.append({
                "name": name, "va": va, "vsize": vsize, "raw_ptr": rptr,
                "raw_size": rsize, "characteristics": chars,
            })

    def u16(self, off):
        return struct.unpack_from("<H", self.data, off)[0]

    def u32(self, off):
        return struct.unpack_from("<I", self.data, off)[0]

    def rva_to_off(self, rva):
        for s in self.sections:
            size = max(s["vsize"], s["raw_size"])
            if s["va"] <= rva < s["va"] + size:
                off = rva - s["va"] + s["raw_ptr"]
                if off >= len(self.data):
                    raise PEError("rva 0x%x outside file" % rva)
                return off
        raise PEError("rva 0x%x not in any section" % rva)

    def cstr(self, rva, limit=512):
        off = self.rva_to_off(rva)
        end = self.data.find(b"\0", off, off + limit)
        if end < 0:
            end = off + limit
        return self.data[off:end].decode("latin-1")

    def exports(self):
        rva, size = self.dirs[0] if self.dirs else (0, 0)
        if not rva:
            return None
        off = self.rva_to_off(rva)
        (_, _, _, _, name_rva, base, nfuncs, nnames, funcs_rva, names_rva,
         ords_rva) = struct.unpack_from("<IIHHIIIIIII", self.data, off)
        funcs = self.rva_to_off(funcs_rva) if nfuncs else 0
        named = {}
        if nnames:
            names = self.rva_to_off(names_rva)
            ords = self.rva_to_off(ords_rva)
            for i in range(nnames):
                idx = self.u16(ords + 2 * i)
                named[idx] = self.cstr(self.u32(names + 4 * i))
        symbols = []
        for idx in range(nfuncs):
            func_rva = self.u32(funcs + 4 * idx)
            if not func_rva:
                continue
            entry = {"ordinal": base + idx, "rva": func_rva, "name": named.get(idx)}
            if rva <= func_rva < rva + size:
                entry["forwarder"] = self.cstr(func_rva)
            symbols.append(entry)
        return {"dll_name": self.cstr(name_rva), "symbols": symbols}

    def imports(self):
        rva = self.dirs[1][0] if len(self.dirs) > 1 else 0
        if not rva:
            return []
        result = []
        off = self.rva_to_off(rva)
        thunk_size = 8 if self.is64 else 4
        ordinal_flag = 1 << (63 if self.is64 else 31)
        while True:
            oft, _, _, name_rva, ft = struct.unpack_from("<IIIII", self.data, off)
            if not (oft or name_rva or ft):
                break
            funcs = []
            thunk = self.rva_to_off(oft or ft)
            while True:
                fmt = "<Q" if self.is64 else "<I"
                value = struct.unpack_from(fmt, self.data, thunk)[0]
                if not value:
                    break
                if value & ordinal_flag:
                    funcs.append("#%d" % (value & 0xFFFF))
                else:
                    funcs.append(self.cstr((value & 0x7FFFFFFF) + 2))
                thunk += thunk_size
            result.append({"dll": self.cstr(name_rva), "functions": funcs})
            off += 20
        return result

    def pdb_path(self):
        rva, size = self.dirs[6] if len(self.dirs) > 6 else (0, 0)
        if not rva:
            return None
        off = self.rva_to_off(rva)
        for i in range(size // 28):
            dtype, dsize, _, dptr = struct.unpack_from("<IIII", self.data, off + 28 * i + 12)
            if dtype != 2 or not dptr:
                continue
            sig = self.data[dptr:dptr + 4]
            start = {b"RSDS": 24, b"NB10": 16}.get(sig)
            if start is None:
                continue
            raw = self.data[dptr + start:dptr + dsize]
            return raw.split(b"\0", 1)[0].decode("latin-1")
        return None

    def file_version(self):
        sig = self.data.find(b"\xbd\x04\xef\xfe")
        if sig < 0:
            return None
        ms, ls = struct.unpack_from("<II", self.data, sig + 8)
        return "%d.%d.%d.%d" % (ms >> 16, ms & 0xFFFF, ls >> 16, ls & 0xFFFF)

    def section_entropy(self, section):
        raw = self.data[section["raw_ptr"]:section["raw_ptr"] + section["raw_size"]]
        if not raw:
            return 0.0
        counts = collections.Counter(raw)
        n = len(raw)
        return -sum(c / n * math.log2(c / n) for c in counts.values())


def msvc_scope(name):
    """Return the class scope of an MSVC-mangled name, e.g. 'X_Character'."""
    if not name or not name.startswith("?"):
        return None
    if name.startswith("??_"):
        rest = name[4:]
        named_member = False
    elif name.startswith("??") and not name.startswith("??$"):
        rest = name[3:]
        named_member = False
    else:
        rest = name[1:]
        named_member = True
    parts = rest.split("@@", 1)[0].split("@")
    if named_member:
        parts = parts[1:]
    parts = [p for p in parts if p]
    if not parts:
        return None
    return "::".join(reversed(parts))


def msvc_member(name):
    if name.startswith("??0"):
        return "<ctor>"
    if name.startswith("??1"):
        return "<dtor>"
    if name.startswith("??_7"):
        return "<vftable>"
    if name.startswith("??_"):
        return "<special %s>" % name[2:4]
    if name.startswith("??"):
        return "<operator %s>" % name[2]
    return name[1:].split("@", 1)[0]


def keyword_strings(data, limit_per_keyword=12):
    found = collections.defaultdict(list)
    for match in re.finditer(rb"[\x20-\x7e]{5,}", data):
        text = match.group().decode("ascii")
        low = text.lower()
        for kw in KEYWORDS:
            if kw in low and len(found[kw]) < limit_per_keyword and text not in found[kw]:
                found[kw].append(text)
    return dict(found)


def analyze_pe(path, data):
    pe = PE(data)
    info = {
        "machine": MACHINES.get(pe.machine, hex(pe.machine)),
        "timestamp": pe.timestamp,
        "image_base": hex(pe.image_base),
        "image_size": pe.image_size,
        "entry_rva": hex(pe.entry),
        "is_dll": bool(pe.characteristics & 0x2000),
        "file_version": pe.file_version(),
        "pdb_path": pe.pdb_path(),
        "sections": [],
        "drm_markers": [],
    }
    for s in pe.sections:
        entropy = round(pe.section_entropy(s), 2)
        info["sections"].append({
            "name": s["name"], "va": hex(s["va"]), "vsize": s["vsize"],
            "raw_size": s["raw_size"], "characteristics": hex(s["characteristics"]),
            "entropy": entropy,
        })
        for pattern, drm in DRM_SECTION_PATTERNS:
            if pattern.match(s["name"].lower()):
                info["drm_markers"].append("%s section %s" % (drm, s["name"]))
    errors = []
    try:
        info["imports"] = pe.imports()
    except (PEError, struct.error) as exc:
        info["imports"] = []
        errors.append("imports: %s" % exc)
    try:
        info["exports"] = pe.exports()
    except (PEError, struct.error) as exc:
        info["exports"] = None
        errors.append("exports: %s" % exc)
    info["keyword_strings"] = keyword_strings(data)
    if errors:
        info["parse_errors"] = errors
    return info


def scan(game_dir):
    report = {"game_dir": game_dir, "files": [], "modules": {}, "drm_files": [],
              "asi_files": [], "errors": []}
    for root, dirs, files in os.walk(game_dir):
        dirs.sort()
        for fname in sorted(files):
            full = os.path.join(root, fname)
            rel = os.path.relpath(full, game_dir)
            try:
                size = os.path.getsize(full)
            except OSError as exc:
                report["errors"].append("%s: %s" % (rel, exc))
                continue
            report["files"].append({"path": rel, "size": size})
            low = fname.lower()
            if low in DRM_FILES:
                report["drm_files"].append("%s (%s)" % (rel, DRM_FILES[low]))
            if low.endswith(".asi"):
                report["asi_files"].append(rel)
            if os.path.splitext(low)[1] not in PE_EXTENSIONS:
                continue
            try:
                with open(full, "rb") as f:
                    data = f.read()
                module = analyze_pe(full, data)
                module["sha256"] = hashlib.sha256(data).hexdigest()
                module["size"] = size
                report["modules"][rel] = module
            except (OSError, PEError, struct.error) as exc:
                report["errors"].append("%s: %s" % (rel, exc))
    return report


def class_index(module):
    exports = module.get("exports") or {}
    classes = collections.defaultdict(list)
    for sym in exports.get("symbols", []):
        scope = msvc_scope(sym.get("name"))
        if scope:
            classes[scope].append(msvc_member(sym["name"]))
    return classes


def summarize(report):
    out = ["# Recon: %s" % os.path.basename(os.path.normpath(report["game_dir"])), "", "Game dir: `%s`" % report["game_dir"], ""]
    total = sum(f["size"] for f in report["files"])
    out.append("Files: %d, total %.1f MB" % (len(report["files"]), total / 1e6))
    exts = collections.Counter(os.path.splitext(f["path"])[1].lower() or "<none>"
                               for f in report["files"])
    out.append("Extensions: " + ", ".join("%s×%d" % kv for kv in exts.most_common(15)))
    out.append("")

    out.append("## Modules")
    out.append("")
    out.append("| file | arch | version | sha256[:12] | exports (C++) | imports | notes |")
    out.append("|---|---|---|---|---|---|---|")
    for rel, m in sorted(report["modules"].items()):
        syms = (m.get("exports") or {}).get("symbols", [])
        cpp = sum(1 for s in syms if (s.get("name") or "").startswith("?"))
        notes = list(m["drm_markers"])
        hot = [s["name"] for s in m["sections"] if s["entropy"] > 7.2 and s["raw_size"] > 4096]
        if hot:
            notes.append("high-entropy: " + ",".join(hot))
        if m.get("pdb_path"):
            notes.append("pdb: " + m["pdb_path"])
        if m.get("parse_errors"):
            notes.append("; ".join(m["parse_errors"]))
        out.append("| %s | %s | %s | %s | %d (%d) | %d dlls | %s |" % (
            rel, m["machine"], m.get("file_version") or "-", m["sha256"][:12],
            len(syms), cpp, len(m.get("imports", [])), "; ".join(notes) or "-"))
    out.append("")

    out.append("## Injection")
    out.append("")
    out.append("ASI files present: %s" % (", ".join(report["asi_files"]) or "none"))
    present = [f["path"] for f in report["files"]
               if os.path.basename(f["path"]).lower() in ASI_LOADER_NAMES]
    out.append("Loader-named DLLs in game dir: %s" % (", ".join(present) or "none"))
    for rel, m in sorted(report["modules"].items()):
        if m["is_dll"]:
            continue
        names = sorted({i["dll"].lower() for i in m.get("imports", [])} & ASI_LOADER_NAMES)
        out.append("`%s` imports loader-compatible: %s" % (rel, ", ".join(names) or "none"))
    out.append("DRM files: %s" % (", ".join(report["drm_files"]) or "none"))
    out.append("")

    out.append("## Exported C++ classes (per module, top 40 by member count)")
    out.append("")
    for rel, m in sorted(report["modules"].items()):
        classes = class_index(m)
        if not classes:
            continue
        ranked = sorted(classes.items(), key=lambda kv: (-len(kv[1]), kv[0]))[:40]
        out.append("**%s** — %d classes: " % (rel, len(classes)) +
                   ", ".join("%s(%d)" % (k, len(v)) for k, v in ranked))
        out.append("")

    out.append("## Keyword strings")
    out.append("")
    for rel, m in sorted(report["modules"].items()):
        hits = m.get("keyword_strings") or {}
        if not hits:
            continue
        out.append("**%s**" % rel)
        for kw, samples in sorted(hits.items()):
            out.append("- %s: %s" % (kw, " | ".join("`%s`" % s[:80] for s in samples[:6])))
        out.append("")

    if report["errors"]:
        out.append("## Errors")
        out.append("")
        out.extend("- " + e for e in report["errors"])
    return "\n".join(out) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("game_dir", nargs="?", default=r"C:\Program Files (x86)\Max Payne")
    parser.add_argument("--out", default=".", help="output directory (not the game dir)")
    args = parser.parse_args(argv)

    if not os.path.isdir(args.game_dir):
        print("game dir not found: %s" % args.game_dir, file=sys.stderr)
        return 2
    report = scan(args.game_dir)
    if not report["modules"]:
        print("no PE modules found under %s" % args.game_dir, file=sys.stderr)
        return 1
    os.makedirs(args.out, exist_ok=True)
    json_path = os.path.join(args.out, "recon.json")
    md_path = os.path.join(args.out, "recon.md")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1)
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(summarize(report))
    print("modules: %d, files: %d" % (len(report["modules"]), len(report["files"])))
    print("wrote %s and %s" % (json_path, md_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
