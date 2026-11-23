# Raspberry Pi Camera Module 3 (Sony IMX708) on the D-Robotics RDK X5

The RDK X5 ships with drivers for a fixed list of camera sensors, and the
IMX708 of the Raspberry Pi Camera Module 3 is not one of them. This
repository adds it: a sensor driver for the X5 camera framework, tuning files
for the image processor, a small capture service that keeps the camera
streaming, and control of the module's autofocus lens.

Written and maintained by **QuantaByte Private Limited**. Author: Zohaib
([@zohaibcodes9642](https://github.com/zohaibcodes9642)), founder of
QuantaByte, who did the port and the bring-up work described below. It is
published so that it saves others the weeks it took.

## What works

| | State |
|---|---|
| 4096 x 2592 at 14 fps (full resolution, centre crop) | works |
| 2304 x 1296 at 30 fps (2x2 binned, whole field of view) | works |
| 4608 x 2592 at 14 fps (full sensor width) | sensor and MIPI work, but the X5 image processor gives no frames above 4096 wide |
| Auto exposure, auto white balance | work (the board's ISP does them) |
| Lens motor (focus position 0 to 1023) | works |
| Autofocus | by the client: step the lens and measure sharpness. An example is included |
| Colour accuracy, lens shading | approximate: the tuning is adapted from the IMX219 files on the board |
| Phase-detect autofocus, HDR mode | not done |

Tested on: RDK X5 (8 GB), RDK OS 3.5.0 (Ubuntu 22.04, kernel 6.1.83),
`hobot-camera` 3.1.1, Camera Module 3 standard lens, first CSI connector.
Other RDK OS versions need the matching vendor header branch (see
"Things learned").

## Install

You need the Camera Module 3, the 22-pin to 15-pin ribbon that Raspberry Pi
sells for the Pi 5 and Zero, and an RDK X5 with internet access for the first
build. Push the ribbon fully home at both ends: a half-seated cable is the
most common reason for no picture (see below).

On the board:

```bash
git clone https://github.com/QuantabytePK/rdk-x5-imx708.git
cd rdk-x5-imx708
./install.sh
```

The installer builds the sensor driver and the capture service with the
board's own compiler (it downloads the vendor camera headers for that),
writes the ISP tuning files, and starts the `imx708-csi` service now and at
every boot. It asks for `sudo` where it installs files.

If another service is using the same CSI connector or the image processor
(a vendor camera sample, a ROS camera node), stop it first: only one
pipeline can own them.

## Check it

```bash
systemctl status imx708-csi                         # should be active
python3 examples/imx708_cam.py --info                    # resolution, frame rate, lens position
python3 examples/imx708_cam.py --save /tmp/shots         # still.jpg and preview.jpg
python3 examples/imx708_cam.py --focus --save /tmp/shots # autofocus first
```

`examples/imx708_cam.py` needs `numpy` and `opencv-python` (or the board's
`python3-opencv`).

## Use it from your program

The service publishes two pictures to shared memory on every frame, with a
sequence lock, so taking a photo is a memory copy: no mode switch and no
settling time.

| File | Contents |
|---|---|
| `/dev/shm/imx708_csi_still` | the full frame, NV12 (4096 x 2592 by default) |
| `/dev/shm/imx708_csi_preview` | the same view scaled down, NV12 (1280 x 810 by default) |
| `/dev/shm/imx708_csi_ctl` | status (size, frame rate, lens position) and lens requests |

The layout is in [`csi/imx708_csi_shm.h`](csi/imx708_csi_shm.h).
[`examples/imx708_cam.py`](examples/imx708_cam.py) is a complete client in about 150
lines of Python: copy it into your project.

```python
from imx708_cam import Camera

cam = Camera()
picture = cam.bgr("still")        # numpy array, 2592 x 4096 x 3
cam.move_lens(600)                # 0 is far, 1023 is near; about 555 is 40 cm
best = cam.autofocus()            # sweep and keep the sharpest position
```

Service options (edit `ExecStart` in `imx708-csi.service`):

```
imx708-csi [--mode full|binned] [--preview WxH] [--focus POS]
  --mode     full: 4096x2592 at 14 fps (default); binned: 2304x1296 at 30 fps
  --preview  preview size, at most 1920x1080 (default: a third of the still)
  --focus    lens position at start, 0 to 1023 (default 555, about 40 cm)
```

## How it fits together

```
IMX708 --MIPI 2 lanes, 900 Mbps--> VIN --raw10--> ISP --NV12--> VSE --+--> still   4096 x 2592
   ^                                (libimx708.so sets exposure/gain) +--> preview 1280 x 810
   |
lens motor (DW9817, I2C 0x0c)  <-- imx708-csi <-- lens requests (shared memory)
```

```
driver/           libimx708.so: the sensor driver (register tables, exposure, gain)
csi/              imx708-csi: the capture service
vp_sensors/       sensor configs for the vendor sample programs (bring-up and debugging)
tools/            gen_settings.py  register tables from the Linux kernel driver
                  make_tuning.py   ISP tuning file for a new resolution
examples/imx708_cam.py  Python client: frames, lens, autofocus
install.sh        builds and installs all of it, on the board
```

## Things learned

These cost hours each. They are written down so they cost nobody else any.

**Match the vendor headers to the board.** The driver is built against
[`D-Robotics/x5-libcam-inc`](https://github.com/D-Robotics/x5-libcam-inc).
The branch must match the installed `hobot-camera` package:
`sensor_tuning_data_t` grew by 256 bytes between RDK OS 3.3.3 and 3.4.x, and
its size is part of the ioctl number, so a mismatched build loads and then
fails. Set `VENDOR_BRANCH=...` when running `install.sh` on an older image.
Quick check: build the vendor `imx219` driver from the same headers and
compare the `movk` constants in `objdump -d libimx219.so` with the installed
library's.

**New sensor libraries need `ldconfig`.** The framework opens `libimx708.so`
by name; `/usr/hobot/lib/sensor` is on the loader path only through the cache.

**No frames and `phy_fatal: 0x100`: suspect a dead data lane.** The MIPI
receiver raised bit 8 (`err_deskew`) continuously and decoded no packets. As
far as we could establish, that bit reports the lanes disagreeing, which is
what a 2-lane link looks like with one lane missing. Here it was the ribbon
not fully seated: I2C worked, the clock lane and lane 1 worked, lane 0 (the
pair at the edge of the connector) did not. Changing settle time, bit rate
or deskew settings does nothing in this state. The test that proves it:
configure the receiver for one lane; if lane 0 were alive it would report ECC
errors on the half-stream, and it reports nothing at all. Reseat the cable.

**The ISP wants its exact resolution in the tuning file.** `cam-service`
logs `Get Resoulution name error` and the pipeline delivers garbage frames if
the tuning JSON does not declare the sensor's `WIDTHxHEIGHT`, by name, in the
header and in every per-resolution table. `tools/make_tuning.py` rewrites a
vendor file for a new size.

**The ISP input is at most 4096 pixels wide.** At 4608 x 2592 the sensor
streams, VIN receives 14 fps with no errors, and the ISP produces nothing and
logs nothing. The driver therefore crops the full-resolution mode on the
sensor (any width below 4608 in the camera config is a centre crop).

**`getframe` hands out the oldest frame, not the newest.** The vendor
`get_isp_data -> g` dump after ten seconds is still frame 0, taken before
auto exposure has done anything (it looks black or pink). Drain the queue.

**A frame's timestamp is not the start of its exposure.** For autofocus you
must judge sharpness on a frame taken after the lens stopped. Frames stamped
up to 83 ms after the stop still showed the old lens position, one at 100 ms
was a mixture, and they were clean from 135 ms on (14 fps). Wait about 1.35
frame times plus 40 ms after the reported stop. With a shorter wait every
lens position is scored on the picture of the one before it, and the focus
lands one step away from the sharpest place, which looks like a lens that
"almost" focuses. `examples/imx708_cam.py` has this in `clean_frame_after()`.

**Starting the camera is slow, so do not stop it.** From service start, the
first frame arrives after 3.1 s and exposure and colour are steady after
5.3 s. Leave the service running and read frames when you need them.

**The camera is powered by its enable pin.** Between runs the pin is low and
the sensor does not answer on I2C. That is not a fault.

## Help wanted

* Colour and lens-shading calibration for the IMX708 (the Raspberry Pi
  libcamera tuning has the numbers; they need translating to the X5 ISP).
* Phase-detect autofocus from the sensor's embedded data, which would bring
  focusing down to a frame or two.
* Reports from other RDK OS versions, the wide and NoIR variants of the
  module, and the second CSI connector.

See [CONTRIBUTING.md](CONTRIBUTING.md). Every change comes in as a pull
request and is reviewed by QuantaByte before it is merged.

## Author and contact

| | |
|---|---|
| Company | QuantaByte Private Limited ([github.com/QuantabytePK](https://github.com/QuantabytePK)) |
| Author | Zohaib ([@zohaibcodes9642](https://github.com/zohaibcodes9642)): founder of QuantaByte, developer of this port |
| Email | ceo@quantabyte.org, zohaibmalik964242@gmail.com |
| Phone | +92 334 1056209 |

For bugs and questions about the code, please open an issue here so that
others can find the answer. For commercial use, support or licensing
questions, write to the addresses above.

## Licence and credit

Copyright (C) 2026 QuantaByte Private Limited. All rights reserved except as
granted by the licence.

The code is released under GPL-2.0-only: see [LICENSE](LICENSE). In plain
words, you may use, study, change and redistribute it, including in
products, on these conditions:

* **Keep the credit.** The copyright notices, the author line and
  [NOTICE.md](NOTICE.md) must stay in every copy and in anything built from
  it. You may not remove them or present this work as your own.
* **Share your changes.** If you distribute a changed version, or a product
  containing it, you must publish the source of your changes under the same
  licence and say what you changed.
* **No other terms.** If you need different terms (for example, to ship it
  inside closed software), ask QuantaByte for a separate licence.

If you use this in a product, a tutorial, a fork or vendor software, credit
it as: "IMX708 support for the RDK X5 by QuantaByte Private Limited
(Zohaib, @zohaibcodes9642), https://github.com/QuantabytePK/rdk-x5-imx708".

The register tables come from the Linux kernel driver (Copyright (C)
2022-2023 Raspberry Pi Ltd); [NOTICE.md](NOTICE.md) lists where each part
comes from.

This is an independent project. It is not affiliated with or endorsed by
D-Robotics, Raspberry Pi Ltd or Sony. No warranty: see the licence.
