// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 QuantaByte Private Limited
 * Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
 *
 * Shared memory layout between the IMX708 capture service (imx708-csi) and its
 * clients. Everything is little endian and lives in /dev/shm.
 *
 *   /dev/shm/imx708_csi_ctl      struct icsi_control
 *   /dev/shm/imx708_csi_preview  struct icsi_frame + NV12 pixels at ICSI_DATA_OFFSET
 *   /dev/shm/imx708_csi_still    struct icsi_frame + NV12 pixels at ICSI_DATA_OFFSET
 *
 * Frames use a sequence lock: the writer makes `seq` odd, copies the pixels,
 * then makes it even. A reader copies what it needs and keeps the copy only if
 * `seq` was even and did not change in between.
 *
 * A Python mirror of this file is examples/imx708_cam.py.
 */
#ifndef IMX708_CSI_SHM_H
#define IMX708_CSI_SHM_H

#include <stdint.h>

#define ICSI_MAGIC		0x49534349u	/* "ICSI" */
#define ICSI_VERSION		1u
#define ICSI_DATA_OFFSET	4096u
#define ICSI_FORMAT_NV12	1u

#define ICSI_CTL_PATH		"/dev/shm/imx708_csi_ctl"
#define ICSI_PREVIEW_PATH	"/dev/shm/imx708_csi_preview"
#define ICSI_STILL_PATH		"/dev/shm/imx708_csi_still"

enum icsi_state {
	ICSI_STARTING = 0,
	ICSI_STREAMING = 1,
	ICSI_STOPPED = 2,
};

/* 64 bytes */
struct icsi_frame {
	uint32_t magic;
	uint32_t version;
	uint32_t seq;		/* odd while the pixels are being written */
	uint32_t width;
	uint32_t height;
	uint32_t stride;	/* bytes per row, luma and interleaved chroma alike */
	uint32_t format;	/* ICSI_FORMAT_NV12 */
	uint32_t data_size;	/* bytes of pixel data: stride * height * 3 / 2 */
	uint64_t frame_id;	/* frames delivered by the service since it started */
	uint64_t timestamp_ns;	/* CLOCK_MONOTONIC stamp from the vendor pipeline. Measured: about
				 * one frame time AFTER the exposure began (see the README on
				 * autofocus timing), not the start of exposure. */
	uint64_t arrival_ns;	/* CLOCK_MONOTONIC when the service received it */
	uint64_t session;	/* service start time; changes when it restarts */
};

/* 104 bytes */
struct icsi_control {
	uint32_t magic;
	uint32_t version;

	/* written by the service */
	uint64_t session;
	uint32_t state;			/* enum icsi_state */
	uint32_t fps;
	uint32_t width;			/* still size */
	uint32_t height;
	uint32_t preview_width;
	uint32_t preview_height;
	uint32_t focus_min;
	uint32_t focus_max;
	uint32_t focus_pos;		/* where the lens is now */
	uint32_t focus_ack;		/* last focus_req_seq that was carried out */
	uint64_t focus_settled_ns;	/* CLOCK_MONOTONIC when the lens stopped moving */
	uint64_t frames;		/* frames delivered so far */
	uint64_t heartbeat_ns;		/* CLOCK_MONOTONIC of the last delivered frame */

	/* written by the client */
	uint32_t focus_req;		/* wanted lens position */
	uint32_t focus_req_seq;		/* increase by one to apply focus_req */

	uint32_t reserved[4];
};

#endif
