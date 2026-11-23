// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 QuantaByte Private Limited
 * Author: Zohaib (@zohaibcodes9642) <ceo@quantabyte.org>
 *
 * IMX708 (Raspberry Pi Camera Module 3) on the RDK X5: 4096x2592 raw10, 14 fps, 2 lane
 * (full resolution, centre crop: the X5 ISP gives no output at 4608 wide).
 */
#include "vp_sensors.h"

#define SENSOR_WIDTH  4096
#define SENSOR_HEIGHT  2592
#define SENSOE_FPS 14
#define RAW10 0x2B

static mipi_config_t imx708_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 2,
		.datatype = RAW10,
		.fps = SENSOE_FPS,
		.mclk = 24,
		.mipiclk = 1800,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 15648,
		.framelenth = 2717,
		.settle = 20,
		.channel_num = 1,
		.channel_sel = {0},
	},
	.rx_ex_mask = 0x40,
	.rx_attr_ex = {
		.stop_check_instart = 1,
	}
};

static camera_config_t imx708_camera_config = {
	.name = "imx708",
	.addr = 0x1a,
	.sensor_mode = 1,
	.fps = SENSOE_FPS,
	.format = RAW10,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.gpio_enable_bit = 0x01,
	.gpio_level_bit = 0x00,
	.mipi_cfg = &imx708_mipi_config,
	.calib_lname = "/usr/hobot/lib/sensor/imx708_4096x2592_tuning.json",
};

static vin_node_attr_t imx708_vin_node_attr = {
	.cim_attr = {
		.mipi_rx = 2,
		.vc_index = 0,
		.ipi_channel = 1,
		.cim_isp_flyby = 0,
		.func = {
			.enable_frame_id = 1,
			.set_init_frame_id = 0,
			.hdr_mode = NOT_HDR,
			.time_stamp_en = 0,
		},
	},
};

static vin_attr_ex_t vin_attr_ex = {
	.vin_attr_ex_mask = 0x00,
	.mclk_ex_attr = {
		.mclk_freq = 24000000,
	},
};

static vin_ichn_attr_t imx708_vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW10,
};

static vin_ochn_attr_t imx708_vin_ochn_attr = {
	.ddr_en = 1,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = RAW10,
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static isp_attr_t imx708_isp_attr = {
	.input_mode = DDR_MODE,
	.sensor_mode= ISP_NORMAL_M,
	.crop = {
		.x = 0,
		.y = 0,
		.h = SENSOR_HEIGHT,
		.w = SENSOR_WIDTH,
	},
};

static isp_ichn_attr_t imx708_isp_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.fmt = FRM_FMT_RAW,
	.bit_width = 10,
};

static isp_ochn_attr_t imx708_isp_ochn_attr = {
	.ddr_en = 1,
	.fmt = FRM_FMT_NV12,
	.bit_width = 8,
};

vp_sensor_config_t imx708_linear_4096x2592_raw10_14fps_2lane = {
	.chip_id_reg = 0x0016,
	.chip_id = 0x0708,
	.sensor_i2c_addr_list = {0x1A},
	.sensor_name = "imx708-4096x2592-14fps",
	.support_sensor_mode  = {NORMAL_M},
	.config_file = "linear_4096x2592_raw10_14fps_2lane.c",
	.camera_config = &imx708_camera_config,
	.vin_ichn_attr = &imx708_vin_ichn_attr,
	.vin_node_attr = &imx708_vin_node_attr,
	.vin_attr_ex   = &vin_attr_ex,
	.vin_ochn_attr = &imx708_vin_ochn_attr,
	.isp_attr      = &imx708_isp_attr,
	.isp_ichn_attr = &imx708_isp_ichn_attr,
	.isp_ochn_attr = &imx708_isp_ochn_attr,
};
