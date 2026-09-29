#!/usr/bin/env python3
# Copyright (C) 2023-2026 Ola Gatner
# SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
"""Compare register read-backs between boards: parses /tmp/amycore.regs (written by AmyCore for every register read
reply, e.g. after "amyctl regdump all 21 00 13 4") and prints one row per (module, register) with each board's value.
Rows where the boards differ are marked with '*'.
usage: regcompare.py [file] [--diff]   (--diff: only rows that differ)"""
import sys, struct, collections

fn = next((a for a in sys.argv[1:] if not a.startswith('--')), '/tmp/amycore.regs')
only_diff = '--diff' in sys.argv
vals = collections.defaultdict(dict)                 # (mod, reg) -> {board: value or 'ERR'}
boards = set()
for line in open(fn):
    head, _, hexs = line.partition(':')
    board = int(head.split()[1], 16) // 8            # hw byte = node * 8; AmyCore's jointList index
    b = bytes.fromhex(hexs.replace(' ', ''))
    i = 0
    while i + 4 <= len(b) and b[i] in (0x41, 0x51):
        op, size, reg, mod = b[i:i+4]
        if op == 0x51:                               # read refused (seen for debug variables, group 2)
            vals[(mod, reg)][board] = 'ERR'; i += 4; boards.add(board); continue
        raw = b[i+4:i+4+size]
        v = struct.unpack('<h' if size == 2 else '<i', raw)[0] if size in (2, 4) else raw.hex()
        vals[(mod, reg)][board] = v; boards.add(board)
        i += 4 + size

boards = sorted(boards)
print('mod reg ' + ''.join(f'{"board "+str(b):>13}' for b in boards))
for (mod, reg) in sorted(vals):
    row = vals[(mod, reg)]
    differ = len(set(str(row.get(b)) for b in boards)) > 1
    if only_diff and not differ: continue
    print(f'{mod:02x}  {reg:02x}  ' + ''.join(f'{str(row.get(b, "-")):>13}' for b in boards) + ('  *' if differ else ''))
