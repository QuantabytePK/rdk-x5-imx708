// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 QuantaByte Private Limited
 * Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
 *
 * imx708-csi: capture service for the Raspberry Pi Camera Module 3 (Sony
 * IMX708) on the D-Robotics RDK X5.
 *
 * It runs the vendor pipeline  sensor -> VIN -> ISP -> VSE  and publishes two
 * streams to shared memory (see imx708_csi_shm.h):
 *
 *   still    the full sensor frame, for reading text
 *   preview  the same view scaled down, for the screen and document finding
 *
 * It also owns the autofocus lens motor (DW9817 voice coil at I2C 0x0c):
 * clients ask for a lens position through the control block.
 *
 * The service exits with a non zero status when the camera is missing or the
 * stream stalls, and relies on systemd to start it again.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "hb_camera_interface.h"
#include "hb_camera_data_config.h"
#include "hb_mem_mgr.h"
#include "hbn_api.h"
#include "vin_cfg.h"
#include "isp_cfg.h"
#include "vse_cfg.h"
#include "cam_def.h"

#include "imx708_csi_shm.h"

#ifndef AUTO_ALLOC_ID
#define AUTO_ALLOC_ID		(-1)
#endif
#define RAW10			0x2B

#define IMX708_I2C_ADDR		0x1a
#define IMX708_REG_CHIP_ID	0x0016
#define IMX708_CHIP_ID		0x0708

/* DW9817 lens driver, same protocol as the kernel dw9807-vcm driver. */
#define VCM_I2C_ADDR		0x0c
#define VCM_REG_CTL		0x02
#define VCM_REG_MSB		0x03
#define VCM_REG_STATUS		0x05
#define VCM_MAX_POS		1023
#define VCM_IDLE_POS		512	/* where the lens rests with no current */
#define VCM_RAMP_STEP		16
#define VCM_RAMP_DELAY_US	1000
#define VCM_SETTLE_US		12000

#define VSE_CH_STILL		0	/* up to 4096x3076 */
#define VSE_CH_PREVIEW		1	/* up to 1920x1080 */

#define FRAME_TIMEOUT_MS	2000
#define MAX_FRAME_FAILURES	3

struct mode {
	const char *name;
	uint32_t width, height, fps;
	uint32_t line_length, frame_length, settle;
	const char *tuning;
};

static const struct mode modes[] = {
	/* Full resolution, cropped to the 4096 pixel width the ISP accepts. */
	{ "full", 4096, 2592, 14, 15648, 2717, 20,
	  "/usr/hobot/lib/sensor/imx708_4096x2592_tuning.json" },
	/* 2x2 binned, whole field of view, for a fast preview. */
	{ "binned", 2304, 1296, 30, 7824, 2495, 30,
	  "/usr/hobot/lib/sensor/imx708_2304x1296_tuning.json" },
};

struct port {
	int index;	/* vcon index in the device tree */
	int bus;	/* I2C bus of the camera connector */
	int rx_phy;	/* MIPI receiver the connector is wired to */
	int gpio;	/* camera enable pin */
};

struct shm {
	const char *path;
	uint8_t *map;
	size_t size;
};

static volatile sig_atomic_t running = 1;
static struct shm shm_ctl, shm_preview, shm_still;
static struct icsi_control *ctl;
static uint64_t session;
static int vcm_fd = -1;
static int vcm_pos = VCM_IDLE_POS;
static pthread_mutex_t vcm_lock = PTHREAD_MUTEX_INITIALIZER;

static void logmsg(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
	fflush(stderr);
}

static uint64_t now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void on_signal(int signo)
{
	(void)signo;
	running = 0;
}

/* ---------------------------------------------------------------- shm --- */

static int shm_open_file(struct shm *s, const char *path, size_t size)
{
	int fd = open(path, O_RDWR | O_CREAT, 0666);

	if (fd < 0) {
		logmsg("cannot open %s: %s", path, strerror(errno));
		return -1;
	}
	fchmod(fd, 0666);
	if (ftruncate(fd, (off_t)size) != 0) {
		logmsg("cannot size %s: %s", path, strerror(errno));
		close(fd);
		return -1;
	}
	s->map = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (s->map == MAP_FAILED) {
		logmsg("cannot map %s: %s", path, strerror(errno));
		s->map = NULL;
		return -1;
	}
	s->path = path;
	s->size = size;
	return 0;
}

static size_t frame_file_size(uint32_t width, uint32_t height)
{
	/* Leave room for a stride rounded up to 64 pixels. */
	size_t stride = (width + 63u) & ~63u;

	return ICSI_DATA_OFFSET + stride * height * 3 / 2;
}

static void frame_header_init(struct shm *s)
{
	struct icsi_frame *h = (struct icsi_frame *)s->map;

	memset(h, 0, sizeof(*h));
	h->magic = ICSI_MAGIC;
	h->version = ICSI_VERSION;
	h->format = ICSI_FORMAT_NV12;
	h->session = session;
}

/* Copy one NV12 image into a frame file under the sequence lock. */
static int publish(struct shm *s, const hbn_vnode_image_t *img, uint64_t frame_id,
		   uint64_t arrival)
{
	struct icsi_frame *h = (struct icsi_frame *)s->map;
	const hb_mem_graphic_buf_t *b = &img->buffer;
	uint32_t stride = (uint32_t)b->stride;
	uint32_t height = (uint32_t)b->height;
	size_t luma = (size_t)stride * height;
	size_t chroma = luma / 2;
	uint8_t *dst = s->map + ICSI_DATA_OFFSET;

	if (ICSI_DATA_OFFSET + luma + chroma > s->size ||
	    luma > b->size[0] || chroma > b->size[1]) {
		logmsg("%s: frame %ux%u stride %u does not fit", s->path,
		       (uint32_t)b->width, height, stride);
		return -1;
	}

	hb_mem_invalidate_buf_with_vaddr((uint64_t)b->virt_addr[0], b->size[0]);
	hb_mem_invalidate_buf_with_vaddr((uint64_t)b->virt_addr[1], b->size[1]);

	__atomic_add_fetch(&h->seq, 1, __ATOMIC_SEQ_CST);	/* odd: writing */
	memcpy(dst, b->virt_addr[0], luma);
	memcpy(dst + luma, b->virt_addr[1], chroma);
	h->width = (uint32_t)b->width;
	h->height = height;
	h->stride = stride;
	h->data_size = (uint32_t)(luma + chroma);
	h->frame_id = frame_id;
	h->timestamp_ns = img->info.timestamps;
	h->arrival_ns = arrival;
	__atomic_add_fetch(&h->seq, 1, __ATOMIC_SEQ_CST);	/* even: complete */
	return 0;
}

/* ------------------------------------------------------- device tree --- */

static int read_cells(const char *dir, const char *name, int32_t *out, int max)
{
	char path[256];
	uint8_t raw[64];
	int fd, n, i;

	snprintf(path, sizeof(path), "%s/%s", dir, name);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return 0;
	n = (int)read(fd, raw, sizeof(raw));
	close(fd);
	if (n < 4)
		return 0;
	n /= 4;
	if (n > max)
		n = max;
	for (i = 0; i < n; i++)
		out[i] = (int32_t)((uint32_t)raw[i * 4] << 24 | (uint32_t)raw[i * 4 + 1] << 16 |
				   (uint32_t)raw[i * 4 + 2] << 8 | (uint32_t)raw[i * 4 + 3]);
	return n;
}

static int read_port(int index, struct port *p)
{
	char dir[128], path[192], status[16] = { 0 };
	int32_t cells[8];
	int fd;

	snprintf(dir, sizeof(dir), "/proc/device-tree/soc/cam/vcon@%d", index);
	snprintf(path, sizeof(path), "%s/status", dir);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -1;
	if (read(fd, status, sizeof(status) - 1) < 0)
		status[0] = 0;
	close(fd);
	if (strncmp(status, "okay", 4) != 0)
		return -1;

	p->index = index;
	if (read_cells(dir, "bus", cells, 1) != 1)
		return -1;
	p->bus = cells[0];
	if (read_cells(dir, "rx_phy", cells, 2) != 2)
		return -1;
	p->rx_phy = cells[1];
	if (read_cells(dir, "gpio_oth", cells, 8) < 1 || cells[0] == 0)
		return -1;	/* no enable pin: not a camera connector on this board */
	p->gpio = cells[0];
	return 0;
}

/* --------------------------------------------------------------- gpio --- */

static int write_text(const char *path, const char *text)
{
	int fd = open(path, O_WRONLY);
	ssize_t n;

	if (fd < 0)
		return -1;
	n = write(fd, text, strlen(text));
	close(fd);
	return n < 0 ? -1 : 0;
}

/* Power cycle the camera through its enable pin and leave it switched on. */
static int camera_power_cycle(int gpio)
{
	char num[16], path[96];
	static const char *const levels[] = { "1", "0", "1" };
	size_t i;

	snprintf(num, sizeof(num), "%d", gpio);
	write_text("/sys/class/gpio/export", num);	/* fails if already exported */
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
	if (write_text(path, "out") != 0) {
		logmsg("cannot drive camera enable pin %d: %s", gpio, strerror(errno));
		return -1;
	}
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
	for (i = 0; i < sizeof(levels) / sizeof(levels[0]); i++) {
		write_text(path, levels[i]);
		usleep(30 * 1000);
	}
	write_text("/sys/class/gpio/unexport", num);
	return 0;
}

/* ---------------------------------------------------------------- i2c --- */

static int i2c_read_reg16(int bus, int addr, uint16_t reg, uint8_t *buf, int len)
{
	char path[32];
	uint8_t regbuf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
	struct i2c_msg msgs[2] = {
		{ .addr = (uint16_t)addr, .flags = 0, .len = 2, .buf = regbuf },
		{ .addr = (uint16_t)addr, .flags = I2C_M_RD, .len = (uint16_t)len, .buf = buf },
	};
	struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
	int fd, ret;

	snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
	fd = open(path, O_RDWR);
	if (fd < 0)
		return -1;
	ret = ioctl(fd, I2C_RDWR, &xfer);
	close(fd);
	return ret == 2 ? 0 : -1;
}

static int find_camera(struct port *found)
{
	int index;

	for (index = 0; index < 4; index++) {
		struct port p;
		uint8_t id[2] = { 0, 0 };

		if (read_port(index, &p) != 0)
			continue;
		if (camera_power_cycle(p.gpio) != 0)
			continue;
		if (i2c_read_reg16(p.bus, IMX708_I2C_ADDR, IMX708_REG_CHIP_ID, id, 2) == 0 &&
		    ((id[0] << 8) | id[1]) == IMX708_CHIP_ID) {
			*found = p;
			return 0;
		}
	}
	return -1;
}

/* ---------------------------------------------------------- lens motor --- */

static int vcm_write(const uint8_t *data, size_t len)
{
	return write(vcm_fd, data, len) == (ssize_t)len ? 0 : -1;
}

static int vcm_set_raw(int pos)
{
	uint8_t reg = VCM_REG_STATUS, status = 0;
	uint8_t tx[3] = { VCM_REG_MSB, (uint8_t)((pos >> 8) & 0x03), (uint8_t)(pos & 0xff) };
	int tries;

	/* The driver refuses a new position while it is still busy. */
	for (tries = 0; tries < 10; tries++) {
		if (write(vcm_fd, &reg, 1) != 1 || read(vcm_fd, &status, 1) != 1)
			return -1;
		if (!(status & 0x01))
			break;
		usleep(100);
	}
	return vcm_write(tx, sizeof(tx));
}

/* Walk the lens to `target` in small steps so it does not ring or click. */
static int vcm_move(int target)
{
	int step, pos;

	if (vcm_fd < 0)
		return -1;
	if (target < 0)
		target = 0;
	if (target > VCM_MAX_POS)
		target = VCM_MAX_POS;

	pos = vcm_pos;
	step = target > pos ? VCM_RAMP_STEP : -VCM_RAMP_STEP;
	while (pos != target) {
		pos += step;
		if ((step > 0 && pos > target) || (step < 0 && pos < target))
			pos = target;
		if (vcm_set_raw(pos) != 0) {
			logmsg("lens motor write failed: %s", strerror(errno));
			return -1;
		}
		vcm_pos = pos;
		usleep(VCM_RAMP_DELAY_US);
	}
	return 0;
}

static int vcm_open(int bus, int start_pos)
{
	char path[32];
	static const uint8_t power_on[2] = { VCM_REG_CTL, 0x00 };

	snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
	vcm_fd = open(path, O_RDWR);
	if (vcm_fd < 0) {
		logmsg("lens motor: cannot open %s: %s", path, strerror(errno));
		return -1;
	}
	if (ioctl(vcm_fd, I2C_SLAVE, VCM_I2C_ADDR) != 0 || vcm_write(power_on, 2) != 0) {
		logmsg("lens motor: no answer at 0x%02x on bus %d (fixed focus?)", VCM_I2C_ADDR, bus);
		close(vcm_fd);
		vcm_fd = -1;
		return -1;
	}
	usleep(200);
	vcm_pos = VCM_IDLE_POS;
	return vcm_move(start_pos);
}

static void vcm_close(void)
{
	static const uint8_t power_off[2] = { VCM_REG_CTL, 0x01 };

	if (vcm_fd < 0)
		return;
	vcm_move(VCM_IDLE_POS);
	vcm_write(power_off, 2);
	close(vcm_fd);
	vcm_fd = -1;
}

/* Applies focus requests from the control block. */
static void *focus_thread(void *arg)
{
	(void)arg;
	while (running) {
		uint32_t seq = __atomic_load_n(&ctl->focus_req_seq, __ATOMIC_SEQ_CST);

		if (seq != ctl->focus_ack) {
			uint32_t want = ctl->focus_req;

			pthread_mutex_lock(&vcm_lock);
			vcm_move((int)want);
			usleep(VCM_SETTLE_US);
			ctl->focus_pos = (uint32_t)vcm_pos;
			ctl->focus_settled_ns = now_ns();
			pthread_mutex_unlock(&vcm_lock);
			__atomic_store_n(&ctl->focus_ack, seq, __ATOMIC_SEQ_CST);
		} else {
			usleep(4000);
		}
	}
	return NULL;
}

/* ----------------------------------------------------------- pipeline --- */

struct pipeline {
	camera_handle_t cam;
	hbn_vnode_handle_t vin, isp, vse;
	hbn_vflow_handle_t flow;
	int cam_ok, vin_ok, isp_ok, vse_ok, flow_ok, started;
};

#define CHECK(call, what) do { \
		int32_t _r = (int32_t)(call); \
		if (_r != 0) { \
			logmsg("%s failed (%d)", what, _r); \
			return -1; \
		} \
	} while (0)

static int pipeline_start(struct pipeline *p, const struct mode *m, const struct port *port,
			  uint32_t pv_w, uint32_t pv_h)
{
	static mipi_config_t mipi;
	static camera_config_t cam;
	vin_node_attr_t vin_attr;
	vin_ichn_attr_t vin_in;
	vin_ochn_attr_t vin_out;
	isp_attr_t isp_attr;
	isp_ichn_attr_t isp_in;
	isp_ochn_attr_t isp_out;
	vse_attr_t vse_attr;
	vse_ichn_attr_t vse_in;
	vse_ochn_attr_t vse_out;
	hbn_buf_alloc_attr_t bufs;

	memset(&mipi, 0, sizeof(mipi));
	mipi.rx_enable = 1;
	mipi.rx_attr.phy = 0;
	mipi.rx_attr.lane = 2;
	mipi.rx_attr.datatype = RAW10;
	mipi.rx_attr.fps = (uint16_t)m->fps;
	mipi.rx_attr.mclk = 24;
	mipi.rx_attr.mipiclk = 1800;		/* 2 lanes at 900 Mbps */
	mipi.rx_attr.width = (uint16_t)m->width;
	mipi.rx_attr.height = (uint16_t)m->height;
	mipi.rx_attr.linelenth = (uint16_t)m->line_length;
	mipi.rx_attr.framelenth = (uint16_t)m->frame_length;
	mipi.rx_attr.settle = (uint16_t)m->settle;
	mipi.rx_attr.channel_num = 1;
	mipi.rx_attr.channel_sel[0] = 0;
	mipi.rx_ex_mask = 0x40;
	mipi.rx_attr_ex.stop_check_instart = 1;

	memset(&cam, 0, sizeof(cam));
	snprintf(cam.name, sizeof(cam.name), "imx708");
	cam.addr = IMX708_I2C_ADDR;
	cam.sensor_mode = 1;			/* linear */
	cam.fps = m->fps;
	cam.format = RAW10;
	cam.width = m->width;
	cam.height = m->height;
	cam.gpio_enable_bit = 0x01;
	cam.gpio_level_bit = 0x00;
	cam.mipi_cfg = &mipi;
	snprintf(cam.calib_lname, sizeof(cam.calib_lname), "%s", m->tuning);

	memset(&vin_attr, 0, sizeof(vin_attr));
	vin_attr.cim_attr.mipi_rx = (uint32_t)port->rx_phy;
	vin_attr.cim_attr.vc_index = 0;
	vin_attr.cim_attr.ipi_channel = 1;
	vin_attr.cim_attr.cim_isp_flyby = 0;	/* VIN writes to memory, ISP reads it */
	vin_attr.cim_attr.func.enable_frame_id = 1;
	vin_attr.cim_attr.func.set_init_frame_id = 0;
	vin_attr.cim_attr.func.hdr_mode = NOT_HDR;
	vin_attr.cim_attr.func.time_stamp_en = 0;

	memset(&vin_in, 0, sizeof(vin_in));
	vin_in.width = m->width;
	vin_in.height = m->height;
	vin_in.format = RAW10;

	memset(&vin_out, 0, sizeof(vin_out));
	vin_out.ddr_en = 1;
	vin_out.ochn_attr_type = VIN_BASIC_ATTR;
	vin_out.vin_basic_attr.format = RAW10;
	vin_out.vin_basic_attr.wstride = m->width * 2;	/* raw10 is stored in 16 bits */

	memset(&isp_attr, 0, sizeof(isp_attr));
	isp_attr.input_mode = DDR_MODE;
	isp_attr.sensor_mode = ISP_NORMAL_M;
	isp_attr.crop.x = 0;
	isp_attr.crop.y = 0;
	isp_attr.crop.w = m->width;
	isp_attr.crop.h = m->height;

	memset(&isp_in, 0, sizeof(isp_in));
	isp_in.width = m->width;
	isp_in.height = m->height;
	isp_in.fmt = FRM_FMT_RAW;
	isp_in.bit_width = 10;

	memset(&isp_out, 0, sizeof(isp_out));
	isp_out.ddr_en = 1;
	isp_out.fmt = FRM_FMT_NV12;
	isp_out.bit_width = 8;

	memset(&bufs, 0, sizeof(bufs));
	bufs.buffers_num = 3;
	bufs.is_contig = 1;
	bufs.flags = HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CPU_WRITE_OFTEN |
		     HB_MEM_USAGE_CACHED;

	CHECK(hbn_camera_create(&cam, &p->cam), "camera create");
	p->cam_ok = 1;

	CHECK(hbn_vnode_open(HB_VIN, (uint32_t)port->rx_phy, AUTO_ALLOC_ID, &p->vin), "vin open");
	p->vin_ok = 1;
	CHECK(hbn_vnode_set_attr(p->vin, &vin_attr), "vin attr");
	CHECK(hbn_vnode_set_ichn_attr(p->vin, 0, &vin_in), "vin input");
	CHECK(hbn_vnode_set_ochn_attr(p->vin, 0, &vin_out), "vin output");
	CHECK(hbn_vnode_set_ochn_buf_attr(p->vin, 0, &bufs), "vin buffers");

	CHECK(hbn_vnode_open(HB_ISP, 0, AUTO_ALLOC_ID, &p->isp), "isp open");
	p->isp_ok = 1;
	CHECK(hbn_vnode_set_attr(p->isp, &isp_attr), "isp attr");
	CHECK(hbn_vnode_set_ochn_attr(p->isp, 0, &isp_out), "isp output");
	CHECK(hbn_vnode_set_ichn_attr(p->isp, 0, &isp_in), "isp input");
	CHECK(hbn_vnode_set_ochn_buf_attr(p->isp, 0, &bufs), "isp buffers");

	memset(&vse_attr, 0, sizeof(vse_attr));
	memset(&vse_in, 0, sizeof(vse_in));
	vse_in.width = m->width;
	vse_in.height = m->height;
	vse_in.fmt = FRM_FMT_NV12;
	vse_in.bit_width = 8;

	CHECK(hbn_vnode_open(HB_VSE, 0, AUTO_ALLOC_ID, &p->vse), "vse open");
	p->vse_ok = 1;
	CHECK(hbn_vnode_set_attr(p->vse, &vse_attr), "vse attr");
	CHECK(hbn_vnode_set_ichn_attr(p->vse, 0, &vse_in), "vse input");

	memset(&vse_out, 0, sizeof(vse_out));
	vse_out.chn_en = CAM_TRUE;
	vse_out.roi.x = 0;
	vse_out.roi.y = 0;
	vse_out.roi.w = m->width;
	vse_out.roi.h = m->height;
	vse_out.fmt = FRM_FMT_NV12;
	vse_out.bit_width = 8;

	vse_out.target_w = m->width;
	vse_out.target_h = m->height;
	CHECK(hbn_vnode_set_ochn_attr(p->vse, VSE_CH_STILL, &vse_out), "vse still output");
	CHECK(hbn_vnode_set_ochn_buf_attr(p->vse, VSE_CH_STILL, &bufs), "vse still buffers");

	vse_out.target_w = pv_w;
	vse_out.target_h = pv_h;
	CHECK(hbn_vnode_set_ochn_attr(p->vse, VSE_CH_PREVIEW, &vse_out), "vse preview output");
	CHECK(hbn_vnode_set_ochn_buf_attr(p->vse, VSE_CH_PREVIEW, &bufs), "vse preview buffers");

	CHECK(hbn_vflow_create(&p->flow), "flow create");
	p->flow_ok = 1;
	CHECK(hbn_vflow_add_vnode(p->flow, p->vin), "flow add vin");
	CHECK(hbn_vflow_add_vnode(p->flow, p->isp), "flow add isp");
	CHECK(hbn_vflow_add_vnode(p->flow, p->vse), "flow add vse");
	CHECK(hbn_vflow_bind_vnode(p->flow, p->vin, 0, p->isp, 0), "bind vin to isp");
	CHECK(hbn_vflow_bind_vnode(p->flow, p->isp, 0, p->vse, 0), "bind isp to vse");
	CHECK(hbn_camera_attach_to_vin(p->cam, p->vin), "attach camera");
	CHECK(hbn_vflow_start(p->flow), "flow start");
	p->started = 1;
	return 0;
}

static void pipeline_stop(struct pipeline *p)
{
	if (p->started)
		hbn_vflow_stop(p->flow);
	if (p->vse_ok)
		hbn_vnode_close(p->vse);
	if (p->isp_ok)
		hbn_vnode_close(p->isp);
	if (p->vin_ok)
		hbn_vnode_close(p->vin);
	if (p->cam_ok)
		hbn_camera_destroy(p->cam);
	if (p->flow_ok)
		hbn_vflow_destroy(p->flow);
}

/* --------------------------------------------------------------- main --- */

static void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s [--mode full|binned] [--preview WxH] [--focus POS]\n"
		"  --mode     full: 4096x2592 at 14 fps (default); binned: 2304x1296 at 30 fps\n"
		"  --preview  preview size, at most 1920x1080 (default: a third of the still)\n"
		"  --focus    lens position at start, 0 to 1023 (default 555, about 40 cm)\n",
		argv0);
}

int main(int argc, char **argv)
{
	static const struct option options[] = {
		{ "mode", required_argument, NULL, 'm' },
		{ "preview", required_argument, NULL, 'p' },
		{ "focus", required_argument, NULL, 'f' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	const struct mode *mode = &modes[0];
	struct pipeline pipe;
	struct port port = { 0, 0, 0, 0 };
	pthread_t focus_tid;
	struct sigaction sa;
	uint32_t pv_w = 0, pv_h = 0;
	int start_focus = 555;
	int failures = 0, exit_code = 0, focus_thread_started = 0, c;
	uint64_t frames = 0, last_report, report_frames = 0;
	size_t i;

	while ((c = getopt_long(argc, argv, "m:p:f:h", options, NULL)) != -1) {
		switch (c) {
		case 'm':
			mode = NULL;
			for (i = 0; i < sizeof(modes) / sizeof(modes[0]); i++)
				if (strcmp(optarg, modes[i].name) == 0)
					mode = &modes[i];
			if (!mode) {
				usage(argv[0]);
				return 2;
			}
			break;
		case 'p':
			if (sscanf(optarg, "%ux%u", &pv_w, &pv_h) != 2) {
				usage(argv[0]);
				return 2;
			}
			break;
		case 'f':
			start_focus = atoi(optarg);
			break;
		default:
			usage(argv[0]);
			return c == 'h' ? 0 : 2;
		}
	}
	if (pv_w == 0 || pv_h == 0) {
		/* Same view as the still, small enough for the preview scaler. */
		pv_w = mode->width > 2560 ? (mode->width * 5 / 16) & ~1u : (mode->width / 2) & ~1u;
		pv_h = (uint32_t)((uint64_t)mode->height * pv_w / mode->width) & ~1u;
	}
	if (pv_w > 1920 || pv_h > 1080 || pv_w > mode->width || pv_h > mode->height) {
		logmsg("preview %ux%u is outside what the scaler can do", pv_w, pv_h);
		return 2;
	}

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	session = now_ns();
	if (shm_open_file(&shm_ctl, ICSI_CTL_PATH, 4096) != 0 ||
	    shm_open_file(&shm_preview, ICSI_PREVIEW_PATH, frame_file_size(pv_w, pv_h)) != 0 ||
	    shm_open_file(&shm_still, ICSI_STILL_PATH, frame_file_size(mode->width, mode->height)) != 0)
		return 1;

	ctl = (struct icsi_control *)shm_ctl.map;
	memset(ctl, 0, sizeof(*ctl));
	ctl->magic = ICSI_MAGIC;
	ctl->version = ICSI_VERSION;
	ctl->session = session;
	ctl->state = ICSI_STARTING;
	ctl->fps = mode->fps;
	ctl->width = mode->width;
	ctl->height = mode->height;
	ctl->preview_width = pv_w;
	ctl->preview_height = pv_h;
	ctl->focus_min = 0;
	ctl->focus_max = VCM_MAX_POS;
	frame_header_init(&shm_preview);
	frame_header_init(&shm_still);

	/* No camera is not an error to exit on: wait for one to be plugged in. */
	for (c = 0; running && find_camera(&port) != 0; c++) {
		if (c % 20 == 0)
			logmsg("no IMX708 camera on the CSI connectors, waiting for one");
		ctl->state = ICSI_STOPPED;
		sleep(3);
	}
	if (!running)
		return 0;
	ctl->state = ICSI_STARTING;
	logmsg("IMX708 on connector %d (i2c bus %d, mipi rx %d); mode %s %ux%u at %u fps, preview %ux%u",
	       port.index, port.bus, port.rx_phy, mode->name, mode->width, mode->height,
	       mode->fps, pv_w, pv_h);

	hb_mem_module_open();
	memset(&pipe, 0, sizeof(pipe));
	if (pipeline_start(&pipe, mode, &port, pv_w, pv_h) != 0) {
		exit_code = 4;
		goto out;
	}

	if (vcm_open(port.bus, start_focus) == 0) {
		ctl->focus_pos = (uint32_t)vcm_pos;
		ctl->focus_settled_ns = now_ns();
		if (pthread_create(&focus_tid, NULL, focus_thread, NULL) == 0)
			focus_thread_started = 1;
	}

	last_report = now_ns();
	while (running) {
		hbn_vnode_image_t pv, st;
		int got_pv, got_st;
		uint64_t arrival;

		memset(&pv, 0, sizeof(pv));
		memset(&st, 0, sizeof(st));
		got_pv = hbn_vnode_getframe(pipe.vse, VSE_CH_PREVIEW, FRAME_TIMEOUT_MS, &pv) == 0;
		got_st = hbn_vnode_getframe(pipe.vse, VSE_CH_STILL, FRAME_TIMEOUT_MS, &st) == 0;
		arrival = now_ns();

		if (got_pv && got_st) {
			int bad = publish(&shm_preview, &pv, frames, arrival) != 0 ||
				  publish(&shm_still, &st, frames, arrival) != 0;

			if (frames == 0)
				logmsg("first frame: still %dx%d stride %d, preview %dx%d stride %d, "
				       "sensor time %.3f s, arrival %.3f s",
				       st.buffer.width, st.buffer.height, st.buffer.stride,
				       pv.buffer.width, pv.buffer.height, pv.buffer.stride,
				       (double)st.info.timestamps / 1e9, (double)arrival / 1e9);
			frames++;
			failures = 0;
			ctl->frames = frames;
			ctl->heartbeat_ns = arrival;
			ctl->state = ICSI_STREAMING;
			if (bad) {
				exit_code = 5;
				running = 0;
			}
		} else if (running) {
			failures++;
			logmsg("no frame from the camera (%d of %d)", failures, MAX_FRAME_FAILURES);
		}
		if (got_pv)
			hbn_vnode_releaseframe(pipe.vse, VSE_CH_PREVIEW, &pv);
		if (got_st)
			hbn_vnode_releaseframe(pipe.vse, VSE_CH_STILL, &st);

		if (failures >= MAX_FRAME_FAILURES) {
			logmsg("camera stream stalled, giving up so the service restarts");
			exit_code = 1;
			break;
		}

		if (arrival - last_report >= 60ull * 1000000000ull) {
			logmsg("%.1f fps, %llu frames, lens at %d",
			       (double)(frames - report_frames) * 1e9 / (double)(arrival - last_report),
			       (unsigned long long)frames, vcm_pos);
			last_report = arrival;
			report_frames = frames;
		}
	}

out:
	logmsg("stopping");
	running = 0;
	ctl->state = ICSI_STOPPED;
	alarm(8);	/* do not hang in teardown: let systemd restart us */
	if (focus_thread_started)
		pthread_join(focus_tid, NULL);
	pthread_mutex_lock(&vcm_lock);
	vcm_close();
	pthread_mutex_unlock(&vcm_lock);
	pipeline_stop(&pipe);
	hb_mem_module_close();
	return exit_code;
}
