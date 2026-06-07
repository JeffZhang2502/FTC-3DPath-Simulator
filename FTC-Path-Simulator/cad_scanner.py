#!/usr/bin/env python3
"""
Parasolid .x_t Assembly Scanner for FTC DECODE Field (am-5700).

Extracts all part identifiers from the binary and cross-references
them with known FTC field specifications to generate a structured
inventory of collision bodies.
"""

import re
import struct
import sys
from collections import Counter

FILE = 'DECODE™ presented by RTX Full Field - am-5700_Full.x_t'

# ── known AndyMark part specs (from product pages & game manual §9) ──
KNOWN_PARTS = {
    # FTC standard field infrastructure
    'am-2160b':  {'desc': 'Perimeter Wall Panel',   'w': 48,  'd': 1.25, 'h': 12.125},
    'am-2600b':  {'desc': 'Field Corner Bracket',    'w': 3,   'd': 3,    'h': 12.125},
    'am-2556a':  {'desc': 'Soft Foam Floor Tile',    'w': 24,  'd': 24,   'h': 0.59},
    'am-2589':   {'desc': 'Goal Basket Assembly',    'w': 20,  'd': 20,   'h': 38.75},
    'am-0481b':  {'desc': 'T-Channel Extrusion',     'w': 1,   'd': 1,    'h': 24},
    'am-1705':   {'desc': 'T-Nut Hardware',          'w': 0.5, 'd': 0.5,  'h': 0.5},

    # DECODE-specific field elements
    'am-5700':   {'desc': 'DECODE Full Field Assembly (top-level)', 'w': 144, 'd': 144, 'h': 48},
    'am-5704':   {'desc': 'DECODE Gate Assembly',      'w': 24,  'd': 12,   'h': 18},
    'am-5707':   {'desc': 'DECODE Ramp',               'w': 24,  'd': 12,   'h': 6},
    'am-5715':   {'desc': 'DECODE Obelisk Base',       'w': 11,  'd': 11,   'h': 4},
    'am-5716':   {'desc': 'DECODE Obelisk Top',        'w': 11,  'd': 11,   'h': 19},
    'am-5717':   {'desc': 'DECODE Basket/Goal Frame',  'w': 20,  'd': 20,   'h': 38.75},
    'am-5718':   {'desc': 'DECODE Classifier Box',     'w': 18,  'd': 12,   'h': 12},
    'am-5721':   {'desc': 'DECODE Wall Element',       'w': 24,  'd': 1.5,  'h': 12},
    'am-5728a':  {'desc': 'DECODE Scoring Element',    'w': 3,   'd': 3,    'h': 1.5},
    'am-5730':   {'desc': 'DECODE Alliance Station Pole', 'w': 1.5, 'd': 1.5, 'h': 36},
    'am-5731':   {'desc': 'DECODE Alliance Station Base', 'w': 6, 'd': 6,    'h': 3},
    'am-5735':   {'desc': 'DECODE Goal Assembly',      'w': 20,  'd': 20,   'h': 38.75},
    'am-5469':   {'desc': 'DECODE Structural Bracket', 'w': 3,   'd': 3,    'h': 2},

    # Hardware
    'am-1631':   {'desc': 'M3 Hardware Kit',          'w': 0.5, 'd': 0.5,  'h': 0.5},
    'am-3376a':  {'desc': 'Channel Bracket',           'w': 2,   'd': 2,    'h': 1},
}


def scan_parasolid(path):
    """Extract part names and their byte offsets from a Parasolid .x_t file."""
    with open(path, 'rb') as f:
        data = f.read()

    print(f"File size: {len(data):,} bytes")
    print(f"Parasolid version: {detect_version(data)}")
    print()

    # Find all am-XXXX part numbers with their byte offsets.
    pattern = re.compile(rb'am-\d+[a-z]*')
    parts = [(m.start(), m.group().decode('ascii'))
             for m in pattern.finditer(data)]

    # Deduplicate and count.
    counter = Counter(name for _, name in parts)
    first_occurrence = {}
    for offset, name in parts:
        if name not in first_occurrence:
            first_occurrence[name] = offset

    print(f"{'Part #':<16} {'Count':>6}  {'First Offset':>10}  {'Description'}")
    print("-" * 75)
    for name, count in sorted(counter.items()):
        desc = KNOWN_PARTS.get(name, {}).get('desc', 'UNKNOWN')
        offset = first_occurrence[name]
        print(f"{name:<16} {count:>6}  {offset:>10}  {desc}")

    print()
    print(f"Total unique parts: {len(counter)}")
    print(f"Total instances:    {sum(counter.values())}")

    # Report DECODE-specific parts.
    print()
    print("─" * 60)
    print("DECODE FIELD ELEMENTS (collision-relevant subset):")
    print("─" * 60)
    for name in sorted(counter):
        spec = KNOWN_PARTS.get(name)
        if spec and 'DECODE' in spec.get('desc', '') or name in (
            'am-2160b', 'am-2600b', 'am-2556a', 'am-2589'):
            print(f"  {name}: {spec['desc']}")
            print(f"    BBox: {spec['w']}\" × {spec['d']}\" × {spec['h']}\"")
            if name == 'am-2160b':
                print(f"    Qty wall panels: {counter[name]} (48\" each → "
                      f"{counter[name]*48}\" total perimeter)")
            if name == 'am-2556a':
                print(f"    Qty tiles: {counter[name]} (expected 36 for 6×6 grid)")

    # Check for unknown parts that might be DECODE-specific.
    unknown_decode = [n for n in sorted(counter)
                      if n.startswith('am-57') and n not in KNOWN_PARTS]
    if unknown_decode:
        print()
        print(f"⚠ UNKNOWN DECODE parts (need manual lookup): {unknown_decode}")

    # Scan for potential coordinate data near key part markers.
    print()
    print("─" * 60)
    print("GEOMETRY PROBE (scanning near 'am-5700' for potential coords):")
    print("─" * 60)
    probe_offset = first_occurrence.get('am-5700', 0)
    if probe_offset > 0:
        chunk = data[probe_offset:probe_offset + 200]
        # Look for float-like byte patterns.
        for i in range(0, len(chunk) - 4, 4):
            try:
                f = struct.unpack('<f', chunk[i:i+4])[0]
                if 10 < abs(f) < 200 and f == f:  # not NaN, plausible field coord
                    print(f"  offset +{i:4d}:  float32 = {f:10.3f}")
            except Exception:
                pass


def detect_version(data):
    """Extract Parasolid version string from header."""
    m = re.search(rb'FRU=Parasolid ([\d.]+);', data[:512])
    return m.group(1).decode() if m else "unknown"


if __name__ == '__main__':
    scan_parasolid(FILE)
