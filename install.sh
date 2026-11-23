#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2026 QuantaByte Private Limited
# Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
# Install the Camera Module 3 (IMX708) on an RDK X5. Run on the board, from
# this directory:
#
#   ./install.sh
#
# It builds the sensor driver and the capture service with the board's own
# compiler, derives the ISP tuning files from the IMX219 ones that ship with
# the board, and starts the service at boot.
set -euo pipefail
cd "$(dirname "$0")"

SENSOR_LIB_DIR=/usr/hobot/lib/sensor
# Vendor camera headers. The branch has to match the board's hobot-camera
# package, or the driver and the kernel disagree about the size of the
# tuning structure. "main" matches hobot-camera 3.1.1 (RDK OS 3.4.x and 3.5.0).
VENDOR_REPO=https://github.com/D-Robotics/x5-libcam-inc
VENDOR_BRANCH=${VENDOR_BRANCH:-main}
V=vendor/x5-libcam-inc

if [ ! -d "$V" ]; then
    echo "fetching vendor camera headers ($VENDOR_BRANCH)"
    mkdir -p vendor
    git clone --depth 1 --branch "$VENDOR_BRANCH" "$VENDOR_REPO" "$V"
fi

echo "building the sensor driver"
INC="-Idriver -I$V -I$V/private -I$V/develop -I$V/legacy -I$V/legacy/inc -I$V/legacy/utility -I$V/legacy/utility/sensor -I/usr/include"
gcc -O2 -Wall -Wno-unused-function -Wno-unused-variable -Wno-stringop-truncation \
    -fPIC -shared -fno-strict-aliasing -DCAMERA_FRAMEWORK_HBN $INC \
    driver/imx708_utility.c -L/usr/hobot/lib -lcam -lpthread -lm -lrt -o driver/libimx708.so
sudo install -m 755 driver/libimx708.so "$SENSOR_LIB_DIR/libimx708.so"
sudo ldconfig

echo "writing the ISP tuning files"
mkdir -p tuning
python3 tools/make_tuning.py "$SENSOR_LIB_DIR/imx219_3264x2464_tuning.json" \
    tuning/imx708_4096x2592_tuning.json --width 4096 --height 2592 --flat-lsc
python3 tools/make_tuning.py "$SENSOR_LIB_DIR/imx219_1920x1080_tuning.json" \
    tuning/imx708_2304x1296_tuning.json --width 2304 --height 1296 --flat-lsc
sudo install -m 644 tuning/imx708_*_tuning.json "$SENSOR_LIB_DIR/"

echo "building the capture service"
make -C csi
sudo systemctl stop imx708-csi.service 2>/dev/null || true
sudo make -C csi install

echo "installing the service"
sudo install -m 644 imx708-csi.service /etc/systemd/system/imx708-csi.service
sudo systemctl daemon-reload
sudo systemctl enable --now imx708-csi.service

sleep 4
systemctl is-active imx708-csi.service
sudo journalctl -u imx708-csi.service -n 3 --no-pager || true
echo "done. Try it:  python3 examples/imx708_cam.py --info --save /tmp/shots"
