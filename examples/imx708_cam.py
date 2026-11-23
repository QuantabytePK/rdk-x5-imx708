#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2026 QuantaByte Private Limited
# Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
"""Use the camera from Python: frames from shared memory, and the lens.

The capture service (imx708-csi) keeps two pictures in /dev/shm, always
current: the full-size "still" and a small "preview". Reading one is a memory
copy. This file is a complete client in about 150 lines; copy it into your
project, or run it to check the camera:

    python3 examples/imx708_cam.py --info
    python3 examples/imx708_cam.py --save /tmp/shots      still.jpg and preview.jpg
    python3 examples/imx708_cam.py --lens 600             move the lens (0 to 1023)
    python3 examples/imx708_cam.py --sweep                sharpness across the lens range
    python3 examples/imx708_cam.py --focus                autofocus on the middle of the view

Needs numpy; OpenCV only for --save, --sweep and --focus. The layout mirrored
here is csi/imx708_csi_shm.h.
"""
from __future__ import annotations

import argparse
import mmap
import os
import struct
import time

import numpy as np

CTL, PREVIEW, STILL = "/dev/shm/imx708_csi_ctl", "/dev/shm/imx708_csi_preview", "/dev/shm/imx708_csi_still"
MAGIC, DATA_OFFSET, STREAMING = 0x49534349, 4096, 1
FRAME = struct.Struct("<8I4Q")  # magic version seq width height stride format data_size, frame_id timestamp arrival session
CONTROL = struct.Struct("<2IQ10I3Q2I")
FIELDS = ("magic", "version", "session", "state", "fps", "width", "height", "preview_width", "preview_height",
          "focus_min", "focus_max", "focus_pos", "focus_ack", "focus_settled_ns", "frames", "heartbeat_ns",
          "focus_req", "focus_req_seq")


def _map(path: str, write: bool = False) -> mmap.mmap:
    fd = os.open(path, os.O_RDWR if write else os.O_RDONLY)
    try:
        return mmap.mmap(fd, 0, access=mmap.ACCESS_WRITE if write else mmap.ACCESS_READ)
    finally:
        os.close(fd)


class Camera:
    def __init__(self) -> None:
        self.ctl = _map(CTL, write=True)
        self.maps = {"still": _map(STILL), "preview": _map(PREVIEW)}

    def status(self) -> dict:
        d = dict(zip(FIELDS, CONTROL.unpack_from(self.ctl, 0)))
        if d["magic"] != MAGIC:
            raise RuntimeError("imx708-csi is not running")
        return d

    def frame(self, which: str = "still", after_ns: int = 0, timeout: float = 2.0):
        """(header, NV12 pixels). The copy is kept only if the service did
        not write the frame meanwhile (``seq`` even and unchanged).
        ``after_ns`` waits for a frame stamped later than that moment."""
        buf, end = self.maps[which], time.monotonic() + timeout
        while time.monotonic() < end:
            magic, _, seq, width, height, stride, _, size, frame_id, stamp, arrival, _ = FRAME.unpack_from(buf, 0)
            if magic == MAGIC and seq % 2 == 0 and size and stamp >= after_ns and DATA_OFFSET + size <= len(buf):
                pixels = np.frombuffer(buf, np.uint8, size, DATA_OFFSET).reshape(height * 3 // 2, stride).copy()
                if struct.unpack_from("<I", buf, 8)[0] == seq:
                    return {"width": width, "height": height, "frame_id": frame_id, "timestamp_ns": stamp,
                            "arrival_ns": arrival}, pixels[:, :width]
            time.sleep(0.002)
        raise RuntimeError("no frame: is the camera streaming? (systemctl status imx708-csi)")

    def bgr(self, which: str = "still", after_ns: int = 0) -> np.ndarray:
        import cv2

        return cv2.cvtColor(np.ascontiguousarray(self.frame(which, after_ns)[1]), cv2.COLOR_YUV2BGR_NV12)

    def move_lens(self, pos: int, timeout: float = 1.0) -> int:
        """Ask for a lens position. Returns the time (CLOCK_MONOTONIC, ns)
        at which the service says the lens had stopped."""
        s = self.status()
        pos = int(max(s["focus_min"], min(s["focus_max"], pos)))
        seq = (s["focus_req_seq"] + 1) & 0xFFFFFFFF
        struct.pack_into("<I", self.ctl, 80, pos)   # focus_req
        struct.pack_into("<I", self.ctl, 84, seq)   # focus_req_seq: the service acts when this changes
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.status()
            if s["focus_ack"] == seq:
                return s["focus_settled_ns"]
            time.sleep(0.001)
        raise RuntimeError("the service did not move the lens")

    def clean_frame_after(self, settled_ns: int) -> int:
        """The earliest timestamp of a frame that really shows the lens
        where it now is. A frame is stamped about one frame time after its
        exposure began, so frames stamped just after the lens stopped still
        show the OLD position: at 14 fps we measured old pictures up to 83 ms
        after, a mixture at 100 ms and clean ones from 135 ms. Judge focus on
        an earlier frame and every lens position is scored on the picture of
        the one before it."""
        fps = self.status()["fps"] or 14
        return settled_ns + int(1.35e9 / fps) + 40_000_000

    def sharpness(self, pos: int | None = None) -> float:
        """Mean squared edge strength in the middle of the still, at half
        size, after moving the lens to ``pos`` if given."""
        import cv2

        after = self.clean_frame_after(self.move_lens(pos)) if pos is not None else 0
        h, nv12 = self.frame("still", after)
        y0, y1, x0, x1 = int(h["height"] * 0.3), int(h["height"] * 0.7), int(h["width"] * 0.3), int(h["width"] * 0.7)
        small = cv2.resize(nv12[y0:y1, x0:x1], None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        gx, gy = cv2.Sobel(small, cv2.CV_32F, 1, 0), cv2.Sobel(small, cv2.CV_32F, 0, 1)
        return float(np.mean(gx * gx + gy * gy))

    def autofocus(self, lo: int = 400, hi: int = 840, step: int = 40) -> int:
        """The simplest thing that works: a coarse sweep, then a fine one
        around the best stop. About 4 s; a hill climb from the last position
        is quicker if you focus often."""
        coarse = {p: self.sharpness(p) for p in range(lo, hi + 1, step)}
        best = max(coarse, key=coarse.get)
        fine = {p: self.sharpness(p) for p in range(max(lo, best - step), min(hi, best + step) + 1, step // 4)}
        best = max(fine, key=fine.get)
        self.move_lens(best)
        return best


def main() -> None:
    ap = argparse.ArgumentParser(description="Camera Module 3 on the RDK X5: check and examples")
    ap.add_argument("--info", action="store_true", help="what the service is delivering")
    ap.add_argument("--save", metavar="DIR", help="write still.jpg and preview.jpg here")
    ap.add_argument("--lens", type=int, help="move the lens to this position (0 far to 1023 near)")
    ap.add_argument("--sweep", action="store_true", help="print sharpness across the lens range")
    ap.add_argument("--focus", action="store_true", help="autofocus on the middle of the view")
    args = ap.parse_args()
    cam = Camera()
    s = cam.status()
    if s["state"] != STREAMING:
        raise SystemExit("imx708-csi is running but not streaming yet")
    if args.info or not any((args.save, args.lens is not None, args.sweep, args.focus)):
        a = cam.frame("still")[0]["frame_id"]
        time.sleep(1.0)
        b = cam.frame("still")[0]["frame_id"]
        print(f"still {s['width']}x{s['height']}, preview {s['preview_width']}x{s['preview_height']}, "
              f"{b - a} frames in the last second, lens at {s['focus_pos']} ({s['focus_min']} to {s['focus_max']})")
    if args.lens is not None:
        cam.move_lens(args.lens)
        print("lens at", cam.status()["focus_pos"])
    if args.sweep:
        for pos in range(400, 841, 20):
            v = cam.sharpness(pos)
            print(f"  lens {pos:4d}  sharpness {v:9.1f}  {'#' * int(min(60, v ** 0.5))}")
    if args.focus:
        t = time.monotonic()
        print(f"focused at lens {cam.autofocus()} in {time.monotonic() - t:.1f} s")
    if args.save:
        import cv2

        os.makedirs(args.save, exist_ok=True)
        cv2.imwrite(os.path.join(args.save, "still.jpg"), cam.bgr("still"), [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(os.path.join(args.save, "preview.jpg"), cam.bgr("preview"))
        print("saved still.jpg and preview.jpg in", args.save)


if __name__ == "__main__":
    main()
