# Where the code comes from

This repository is licensed GPL-2.0-only as a whole (see `LICENSE`).

| Part | Origin |
|---|---|
| `driver/inc/imx708_setting.h` | Register tables taken from the Linux kernel driver `drivers/media/i2c/imx708.c` in the Raspberry Pi kernel tree (GPL-2.0, Copyright (C) 2022-2023 Raspberry Pi Ltd), reformatted by `tools/gen_settings.py`. Sony does not publish these tables. |
| `driver/imx708_utility.c` | Written by QuantaByte for the D-Robotics X5 camera framework. Its structure (the callbacks the framework requires, and their order) follows the vendor's `imx219` sensor driver, which D-Robotics publishes under Apache-2.0. |
| `vp_sensors/` | Written by QuantaByte in the form the vendor sample programs expect. |
| `csi/`, `tools/`, `examples/`, `install.sh` | Written by QuantaByte. |

Not included, and needed to build: the D-Robotics camera headers
([`D-Robotics/x5-libcam-inc`](https://github.com/D-Robotics/x5-libcam-inc),
fetched by `install.sh`) and the camera libraries already installed on the
board. Those remain under their own licences.

The ISP tuning files are not distributed here either: `install.sh` derives
them on your board from the IMX219 tuning files that ship with RDK OS.

## Copyright and required credit

Copyright (C) 2026 QuantaByte Private Limited.
Author: Zohaib (@zohaibcodes9642), founder of QuantaByte, who wrote this
port: the sensor driver, the capture service, the lens control, the tuning
tools and the documentation.

Contact: ceo@quantabyte.org, zohaibmalik964242@gmail.com, +92 334 1056209.

This file, the copyright and author lines at the top of each source file,
and `LICENSE` are the "appropriate copyright notice" that the GPL requires
to be kept intact (section 1) and carried into modified versions (section
2). Any copy, fork, modified version or product that includes this work must
keep them, and must state prominently what it changed and when. Removing
them, or distributing this work or a derivative under other terms, is not
permitted by the licence and ends the right to use it (section 4).
