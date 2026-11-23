#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2026 QuantaByte Private Limited
# Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
"""Derive an ISP tuning file for a new resolution from an existing one.

The X5 image processor (VeriSilicon ISP) refuses to start unless the tuning
JSON declares the exact sensor resolution: the name "WIDTHxHEIGHT" appears in
the header and in every per-resolution table, and the lens shading grid is
sized in pixels. This script copies a vendor tuning file (for example the
IMX219 one that ships with the board) and rewrites those parts for the
resolution you ask for.

It does not make colour or lens shading correct for the new sensor; with
--flat-lsc the lens shading correction is switched to "no correction", which
is the honest choice when the source file was measured on a different lens.

    make_tuning.py SOURCE.json OUT.json --width 2304 --height 1296 --name imx708
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

LSC_UNITY = 1024  # lens shading gains are 2.10 fixed point


def split(total: int, parts: int) -> list[int]:
    """`parts` integers that sum to `total`, as even as possible."""
    base, extra = divmod(total, parts)
    return [base + (1 if i < extra else 0) for i in range(parts)]


def rename(node, old: str, new: str):
    if isinstance(node, dict):
        return {k: rename(v, old, new) for k, v in node.items()}
    if isinstance(node, list):
        return [rename(v, old, new) for v in node]
    if isinstance(node, str) and old in node:
        return node.replace(old, new)
    return node


def convert(src: dict, width: int, height: int, sensor: str, flat_lsc: bool) -> dict:
    res = src["header"]["resolution"][0]
    old = res["name"]
    new = f"{width}x{height}"
    out = rename(src, old, new)

    head = out["header"]
    head["sensor_name"] = f"{sensor}_{new}"
    head["resolution"][0]["width"] = width
    head["resolution"][0]["height"] = height

    xs = split(width, 32)
    ys = split(height // 2, 16)
    for profile in out["sensor"]["LSC"]:
        profile["LSC_SECT_SIZE_X"] = xs
        profile["LSC_SECT_SIZE_Y"] = ys + [0] * 16
        if flat_lsc:
            for key in ("LSC_SAMPLES_red", "LSC_SAMPLES_greenR",
                        "LSC_SAMPLES_greenB", "LSC_SAMPLES_blue"):
                profile[key] = [[LSC_UNITY] * len(row) for row in profile[key]]

    tuning = out["tuning"]
    lsc = tuning.get("LSC_3")
    if lsc:
        lsc["xSize"] = xs
        lsc["ySize"] = ys
        if flat_lsc:
            lsc["matrix"] = [[LSC_UNITY] * len(m) for m in lsc["matrix"]]

    rect = tuning.get("AES_2", {}).get("rectangle")
    if rect and rect.get("width", 0) >= res["width"] - 2:
        rect["width"] = width
        rect["height"] = height
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("source", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--width", type=int, required=True)
    ap.add_argument("--height", type=int, required=True)
    ap.add_argument("--name", default="imx708", help="sensor name for the header")
    ap.add_argument("--flat-lsc", action="store_true",
                    help="disable lens shading correction (all gains 1.0)")
    args = ap.parse_args()

    src = json.loads(args.source.read_text())
    out = convert(src, args.width, args.height, args.name, args.flat_lsc)
    args.out.write_text(json.dumps(out, indent=1))
    print(f"{args.out}: {args.width}x{args.height} from {args.source.name}"
          f"{' (flat lens shading)' if args.flat_lsc else ''}")


if __name__ == "__main__":
    main()
