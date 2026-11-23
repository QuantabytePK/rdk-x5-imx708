// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 QuantaByte Private Limited
 * Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
 *
 * Sony IMX708 (Raspberry Pi Camera Module 3) sensor driver for the
 * D-Robotics RDK X5 camera framework (libcam, HBN).
 *
 * Structure follows the vendor imx219 driver (Apache-2.0, D-Robotics).
 * Register settings come from the Linux kernel imx708 driver.
 *
 * Modes:
 *   2304x1296 raw10, 30 fps, 2 lane  (2x2 binned, full field of view)
 *   4608x2592 raw10, 14 fps, 2 lane  (full resolution)
 *   4096x2592 raw10, 14 fps, 2 lane  (full resolution, centre crop)
 *
 * The X5 image processor produces no frames for a 4608 pixel wide input, so
 * the full resolution mode can be cropped on the sensor: any width below 4608
 * in the camera config is taken from the centre of the pixel array.
 */
#define pr_fmt(fmt)		"[imx708]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/imx708_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define REG_WIDTH			2	/* 16 bit address, 8 bit data */
#define IMX708_REG_ORIENTATION		0x0101
#define IMX708_REG_EXPOSURE_HI		0x0202
#define IMX708_REG_EXPOSURE_LO		0x0203
#define IMX708_REG_AGAIN_HI		0x0204
#define IMX708_REG_AGAIN_LO		0x0205
#define IMX708_REG_FRAME_LENGTH_HI	0x0340
#define IMX708_REG_FRAME_LENGTH_LO	0x0341
#define IMX708_REG_LONG_EXP_SHIFT	0x3100
#define IMX708_REG_X_OUT_SIZE_HI	0x034C
#define IMX708_REG_X_OUT_SIZE_LO	0x034D
#define IMX708_REG_DIG_CROP_X_HI	0x0408
#define IMX708_REG_DIG_CROP_X_LO	0x0409
#define IMX708_REG_DIG_CROP_W_HI	0x040C
#define IMX708_REG_DIG_CROP_W_LO	0x040D
#define IMX708_FULL_WIDTH		4608
#define IMX708_REG_SPC_GAINS_L		0x7b10
#define IMX708_REG_SPC_GAINS_R		0x7c00
#define IMX708_REG_LPF_INTENSITY_EN	0xC428
#define IMX708_REG_LPF_INTENSITY	0xC429
#define IMX708_EXPOSURE_OFFSET		48
#define IMX708_QBC_ADJUST		2

/* Pixel clock and line length per mode, used to turn fps into a frame length. */
#define IMX708_FULL_PIXEL_RATE		595200000ULL
#define IMX708_FULL_LINE_LENGTH		15648
#define IMX708_FULL_FRAME_LENGTH_MIN	2650
#define IMX708_BIN_PIXEL_RATE		585600000ULL
#define IMX708_BIN_LINE_LENGTH		7824
#define IMX708_BIN_FRAME_LENGTH_MIN	1336

/* Per port state shared between init and the exposure callbacks. */
#define IMX708_MAX_PORTS		8
static uint32_t imx708_frame_length[IMX708_MAX_PORTS];
static uint32_t imx708_line_step[IMX708_MAX_PORTS];
static uint32_t imx708_line_min[IMX708_MAX_PORTS];

static int imx708_linear_data_init(sensor_info_t *sensor_info);

static int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if (sensor_info->gpio_num > 0) {
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if (sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
					sensor_info->gpio_level[gpio]);
				if (ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWEROFF_FAIL;
				}
			}
		}
	}
	return ret;
}

static int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if (sensor_info->gpio_num > 0) {
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if (sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
					sensor_info->gpio_level[gpio]);
				usleep(100 * 1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
					1 - sensor_info->gpio_level[gpio]);
				if (ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100 * 1000);
			}
		}
	}
	return ret;
}

static int imx708_write_table(sensor_info_t *sensor_info, uint32_t *table, size_t bytes)
{
	int size = (int)(bytes / sizeof(uint32_t) / 2);

	return vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			REG_WIDTH, size, table);
}

static int imx708_write8(sensor_info_t *sensor_info, uint16_t reg, uint8_t val)
{
	return vin_i2c_write8(sensor_info->bus_num, 16, sensor_info->sensor_addr, reg, val);
}

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int i, val;
	uint32_t fps = sensor_info->fps ? sensor_info->fps : 30;
	uint32_t frame_length, frame_min, line_length;
	uint64_t pixel_rate;
	int full = (sensor_info->resolution == 2592);
	uint32_t port = sensor_info->port % IMX708_MAX_PORTS;
	uint32_t crop_w = 0, crop_x = 0;

	/* A narrower full resolution width is a centre crop, on a 4 pixel grid. */
	if (full && sensor_info->width >= 16 && sensor_info->width < IMX708_FULL_WIDTH) {
		crop_w = sensor_info->width & ~3U;
		crop_x = ((IMX708_FULL_WIDTH - crop_w) / 2) & ~3U;
	}

	if (sensor_info->sensor_mode != NORMAL_M) {
		vin_err("not support mode %d\n", sensor_info->sensor_mode);
		return -RET_ERROR;
	}
	if (!full && sensor_info->resolution != 1296) {
		vin_err("not support resolution %d\n", sensor_info->resolution);
		return -RET_ERROR;
	}

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	vin_info("imx708 linear mode, bus %d addr 0x%x, %dx%d, %d fps\n",
		sensor_info->bus_num, sensor_info->sensor_addr,
		full ? (crop_w ? crop_w : IMX708_FULL_WIDTH) : 2304,
		full ? 2592 : 1296, fps);

	ret = imx708_write_table(sensor_info, imx708_common_setting,
			sizeof(imx708_common_setting));
	if (ret < 0) {
		vin_err("%d : common init %s fail\n", __LINE__, sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}

	/* Shield pixel correction gains, only when the sensor holds the defaults. */
	val = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, IMX708_REG_SPC_GAINS_L);
	if (val == 0x40) {
		for (i = 0; i < 54; i++)
			imx708_write8(sensor_info, IMX708_REG_SPC_GAINS_L + i,
				imx708_pdaf_gains[0][i % 9]);
		for (i = 0; i < 54; i++)
			imx708_write8(sensor_info, IMX708_REG_SPC_GAINS_R + i,
				imx708_pdaf_gains[1][i % 9]);
	}

	if (full)
		ret = imx708_write_table(sensor_info, imx708_4608x2592_setting,
				sizeof(imx708_4608x2592_setting));
	else
		ret = imx708_write_table(sensor_info, imx708_2304x1296_setting,
				sizeof(imx708_2304x1296_setting));
	if (ret < 0) {
		vin_err("%d : mode init %s fail\n", __LINE__, sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}

	if (crop_w) {
		imx708_write8(sensor_info, IMX708_REG_DIG_CROP_X_HI, (crop_x >> 8) & 0xFF);
		imx708_write8(sensor_info, IMX708_REG_DIG_CROP_X_LO, crop_x & 0xFF);
		imx708_write8(sensor_info, IMX708_REG_DIG_CROP_W_HI, (crop_w >> 8) & 0xFF);
		imx708_write8(sensor_info, IMX708_REG_DIG_CROP_W_LO, crop_w & 0xFF);
		imx708_write8(sensor_info, IMX708_REG_X_OUT_SIZE_HI, (crop_w >> 8) & 0xFF);
		imx708_write8(sensor_info, IMX708_REG_X_OUT_SIZE_LO, crop_w & 0xFF);
	}

	ret = imx708_write_table(sensor_info, imx708_link_450mhz_setting,
			sizeof(imx708_link_450mhz_setting));
	if (ret < 0) {
		vin_err("%d : link init %s fail\n", __LINE__, sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}

	/* Quad Bayer re-mosaic correction is only used at full resolution. */
	if (full) {
		imx708_write8(sensor_info, IMX708_REG_LPF_INTENSITY, IMX708_QBC_ADJUST);
		imx708_write8(sensor_info, IMX708_REG_LPF_INTENSITY_EN, 0x00);
	} else {
		imx708_write8(sensor_info, IMX708_REG_LPF_INTENSITY_EN, 0x01);
	}

	/* Frame length for the requested frame rate. */
	pixel_rate = full ? IMX708_FULL_PIXEL_RATE : IMX708_BIN_PIXEL_RATE;
	line_length = full ? IMX708_FULL_LINE_LENGTH : IMX708_BIN_LINE_LENGTH;
	frame_min = full ? IMX708_FULL_FRAME_LENGTH_MIN : IMX708_BIN_FRAME_LENGTH_MIN;
	frame_length = (uint32_t)((pixel_rate + (uint64_t)line_length * fps / 2) /
			((uint64_t)line_length * fps));
	if (frame_length < frame_min)
		frame_length = frame_min;
	if (frame_length > 0xFFFF)
		frame_length = 0xFFFF;
	imx708_write8(sensor_info, IMX708_REG_FRAME_LENGTH_HI, (frame_length >> 8) & 0xFF);
	imx708_write8(sensor_info, IMX708_REG_FRAME_LENGTH_LO, frame_length & 0xFF);
	imx708_write8(sensor_info, IMX708_REG_LONG_EXP_SHIFT, 0x00);
	imx708_write8(sensor_info, IMX708_REG_ORIENTATION, 0x00);

	imx708_frame_length[port] = frame_length;
	imx708_line_step[port] = full ? 1 : 2;
	imx708_line_min[port] = full ? 8 : 4;

	ret = imx708_linear_data_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
		return -HB_CAM_INIT_FAIL;
	}

	vin_info("imx708 config success, frame length %d\n", frame_length);
	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret;

	vin_info("%s start\n", sensor_info->sensor_name);
	ret = imx708_write_table(sensor_info, imx708_stream_on_setting,
			sizeof(imx708_stream_on_setting));
	if (ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}
	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret;

	vin_info("%s stop\n", sensor_info->sensor_name);
	ret = imx708_write_table(sensor_info, imx708_stream_off_setting,
			sizeof(imx708_stream_off_setting));
	if (ret < 0) {
		vin_err("stop %s fail\n", sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}
	return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = sensor_poweroff(sensor_info);

	if (ret < 0)
		vin_err("%d : deinit %s fail\n", __LINE__, sensor_info->sensor_name);
	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode,
		uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
	uint32_t count = sizeof(imx708_gain_lut) / sizeof(uint32_t);
	uint32_t index, reg;

	if (mode != NORMAL_M) {
		vin_err("unsupport mode %d\n", mode);
		return 0;
	}
	index = again[0] >= count ? count - 1 : again[0];
	reg = imx708_gain_lut[index];
	vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
			IMX708_REG_AGAIN_HI, (reg >> 8) & 0x03);
	vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
			IMX708_REG_AGAIN_LO, reg & 0xFF);
	return 0;
}

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode,
		uint32_t *line, uint32_t line_num)
{
	uint32_t port = info->port % IMX708_MAX_PORTS;
	uint32_t frame_length = imx708_frame_length[port];
	uint32_t step = imx708_line_step[port] ? imx708_line_step[port] : 1;
	uint32_t min = imx708_line_min[port] ? imx708_line_min[port] : 8;
	uint32_t sline, max;

	if (mode != NORMAL_M) {
		vin_err("unsupport mode %d\n", mode);
		return 0;
	}
	if (frame_length == 0)
		frame_length = IMX708_FULL_FRAME_LENGTH_MIN;
	max = frame_length - IMX708_EXPOSURE_OFFSET;
	sline = line[0];
	if (sline > max)
		sline = max;
	if (sline < min)
		sline = min;
	sline -= sline % step;

	vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
			IMX708_REG_EXPOSURE_HI, (sline >> 8) & 0xFF);
	vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
			IMX708_REG_EXPOSURE_LO, sline & 0xFF);
	return 0;
}

static int imx708_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	sensor_turning_data_t turning_data;
	uint32_t port = sensor_info->port % IMX708_MAX_PORTS;
	uint32_t vts = imx708_frame_length[port];
	uint32_t fps = sensor_info->fps ? sensor_info->fps : 30;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
			sizeof(turning_data.sensor_name));

	turning_data.sensor_data.active_width = sensor_info->width;
	turning_data.sensor_data.active_height = sensor_info->height;
	turning_data.sensor_data.conversion = 1;
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = vts * fps;
	turning_data.sensor_data.exposure_time_max = vts - IMX708_EXPOSURE_OFFSET;
	turning_data.sensor_data.exposure_time_long_max = vts - IMX708_EXPOSURE_OFFSET;
	turning_data.sensor_data.analog_gain_max =
		sizeof(imx708_gain_lut) / sizeof(uint32_t) - 1;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.exposure_time_min = imx708_line_min[port];

	sensor_data_bayer_fill(&turning_data.sensor_data, 10,
			(uint32_t)BAYER_START_R, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.stream_ctrl.data_length = 1;
	memcpy(turning_data.stream_ctrl.stream_on, imx708_stream_on_setting,
			sizeof(imx708_stream_on_setting));
	memcpy(turning_data.stream_ctrl.stream_off, imx708_stream_off_setting,
			sizeof(imx708_stream_off_setting));

	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx708_gain_lut,
			sizeof(imx708_gain_lut));
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (ret < 0) {
		vin_err("%s sync gain lut ioctl fail %d\n", sensor_info->sensor_name, ret);
		return -RET_ERROR;
	}
	return ret;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(imx708, CAM_MODULE_FLAG_A16D8);
sensor_module_t imx708 = {
	.module = SENSOR_MNAME(imx708),
#else
sensor_module_t imx708 = {
	.module = "imx708",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.aexp_gain_control = sensor_aexp_gain_control,
	.aexp_line_control = sensor_aexp_line_control,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.userspace_control = sensor_userspace_control,
};
