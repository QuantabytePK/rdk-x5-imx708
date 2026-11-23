# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2026 QuantaByte Private Limited
# Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
# Build on the RDK X5 itself: the vendor headers and libraries are on the board.
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
INCS    := -I/usr/include -I/usr/hobot/include
LIBS    := -L/usr/hobot/lib -lcam -lvpf -lhbmem -lgdcbin -lcjson -lalog -lpthread -ldl
PREFIX  ?= /usr/local

imx708-csi: imx708_csi.c imx708_csi_shm.h
	$(CC) $(CFLAGS) $(INCS) imx708_csi.c $(LIBS) -o $@

install: imx708-csi
	install -m 755 imx708-csi $(PREFIX)/bin/imx708-csi

clean:
	rm -f imx708-csi

.PHONY: install clean
