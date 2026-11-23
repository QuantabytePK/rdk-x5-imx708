#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2026 QuantaByte Private Limited
# Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
"""Generate driver/inc/imx708_setting.h from the Linux kernel imx708 driver.

The register tables for the IMX708 are not published by Sony; the reference
is the Raspberry Pi kernel driver. This script pulls the tables this port
uses out of that file and writes them in the form the X5 camera framework
wants (flat arrays of address, value pairs).

    curl -LO https://raw.githubusercontent.com/raspberrypi/linux/rpi-6.6.y/drivers/media/i2c/imx708.c
    python3 tools/gen_settings.py imx708.c driver/inc/imx708_setting.h
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

TABLES = [
    ("imx708_common_setting", "mode_common_regs"),
    ("imx708_4608x2592_setting", "mode_4608x2592_regs"),
    ("imx708_2304x1296_setting", "mode_2x2binned_regs"),
    ("imx708_link_450mhz_setting", "link_450Mhz_regs"),
]

HEADER = """// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 QuantaByte Private Limited
 * Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
 *
 * Sony IMX708 (Raspberry Pi Camera Module 3) register settings for the
 * D-Robotics RDK X5 camera framework.
 *
 * Register values are taken from the Linux kernel driver
 * drivers/media/i2c/imx708.c (Copyright (C) 2022-2023, Raspberry Pi Ltd).
 */
#ifndef IMX708_SETTING_H
#define IMX708_SETTING_H
#include <stdint.h>
"""


def table(src: str, name: str) -> list[tuple[str, str]]:
    m = re.search(r"static const struct imx708_reg %s\[\] = \{(.*?)\};" % name, src, re.S)
    if not m:
        raise SystemExit(f"table {name} not found in the kernel source")
    return re.findall(r"\{(0x[0-9A-Fa-f]+),\s*(0x[0-9A-Fa-f]+)\}", m.group(1))


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    src = Path(sys.argv[1]).read_text()
    out = [HEADER]
    for cname, kname in TABLES:
        rows = table(src, kname)
        out.append("static uint32_t %s[] = {" % cname)
        out += ["\t%s, %s," % row for row in rows]
        out.append("};\n")
    out.append("static uint32_t imx708_stream_on_setting[] = {\n\t0x0100, 0x01,\n};\n")
    out.append("static uint32_t imx708_stream_off_setting[] = {\n\t0x0100, 0x00,\n};\n")
    out.append("/* Shield pixel correction gains written when the sensor reports 0x40 at 0x7b10. */")
    out.append("static const uint8_t imx708_pdaf_gains[2][9] = {\n"
               "\t{ 0x4c, 0x4c, 0x4c, 0x46, 0x3e, 0x38, 0x35, 0x35, 0x35 },\n"
               "\t{ 0x35, 0x35, 0x35, 0x38, 0x3e, 0x46, 0x4c, 0x4c, 0x4c },\n};\n")
    out.append("/*\n * Analogue gain table. Index i is a gain of 2^(i/32); the register value is\n"
               " * 1024 - 1024/gain, limited to the sensor maximum of 960 (16x).\n */")
    out.append("static uint32_t imx708_gain_lut[] = {")
    values = [min(960, round(1024 - 1024 / 2 ** (i / 32))) for i in range(129)]
    for i in range(0, 129, 8):
        out.append("\t" + " ".join("0x%03X," % v for v in values[i:i + 8]))
    out.append("};\n\n#endif")
    Path(sys.argv[2]).write_text("\n".join(out) + "\n")
    print("wrote", sys.argv[2])


if __name__ == "__main__":
    main()
