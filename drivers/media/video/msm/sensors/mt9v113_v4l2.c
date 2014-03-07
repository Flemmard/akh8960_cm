/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_sensor.h"
#define SENSOR_NAME "mt9v113"
#define PLATFORM_DRIVER_NAME "msm_camera_mt9v113"
#define mt9v113_obj mt9v113_##obj

/* Sysctl registers */
#define MT9V113_COMMAND_REGISTER                0x0080
#define MT9V113_COMMAND_REGISTER_APPLY_PATCH    (1 << 0)
#define MT9V113_COMMAND_REGISTER_SET_STATE      (1 << 1)
#define MT9V113_COMMAND_REGISTER_REFRESH        (1 << 2)
#define MT9V113_COMMAND_REGISTER_WAIT_FOR_EVENT (1 << 3)
#define MT9V113_COMMAND_REGISTER_OK             (1 << 15)

DEFINE_MUTEX(mt9v113_mut);
static struct msm_sensor_ctrl_t mt9v113_s_ctrl;

static struct v4l2_subdev_info mt9v113_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};



static struct msm_sensor_output_info_t mt9v113_dimensions[] = {
	{
		.x_output = 0x280,
		.y_output = 0x1E0,
		.line_length_pclk = 0x34A,
		.frame_length_lines = 0x22A,
		.vt_pixel_clk = 28000000,
		.op_pixel_clk = 28000000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x280,
		.y_output = 0x1E0,
		.line_length_pclk = 0x34A,
		.frame_length_lines = 0x22A,
		.vt_pixel_clk = 28000000,
		.op_pixel_clk = 28000000,
		.binning_factor = 1,
	},
};

static struct msm_sensor_output_reg_addr_t mt9v113_reg_addr = {
	.x_output = 0xC868,
	.y_output = 0xC86A,
	.line_length_pclk = 0xC868,
	.frame_length_lines = 0xC86A,
};

static enum msm_camera_vreg_name_t mt9v113_veg_seq[] = {
	CAM_VDIG,
	CAM_VIO,
	CAM_VANA,
	CAM_VAF,
};

static struct msm_sensor_id_info_t mt9v113_id_info = {
	.sensor_id_reg_addr = 0x0,
	.sensor_id = 0x2280,
};

static const struct i2c_device_id mt9v113_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&mt9v113_s_ctrl},
	{ }
};

static struct i2c_driver mt9v113_i2c_driver = {
	.id_table = mt9v113_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client mt9v113_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&mt9v113_i2c_driver);
}

static struct v4l2_subdev_core_ops mt9v113_subdev_core_ops = {
	.s_ctrl = msm_sensor_v4l2_s_ctrl,
	.queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops mt9v113_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops mt9v113_subdev_ops = {
	.core = &mt9v113_subdev_core_ops,
	.video  = &mt9v113_subdev_video_ops,
};

static struct msm_sensor_fn_t mt9v113_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_setting = msm_sensor_setting,
	.sensor_csi_setting = msm_sensor_setting1,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines1,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t mt9v113_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
        //	.start_stream_conf = mt9v113_config_change_settings,
        //	.start_stream_conf_size = ARRAY_SIZE(mt9v113_config_change_settings),
        //	.init_settings = &mt9v113_init_conf[0],
        //	.init_size = ARRAY_SIZE(mt9v113_init_conf),
        //	.mode_settings = &mt9v113_confs[0],
        .output_settings = &mt9v113_dimensions[0],
        //	.num_conf = ARRAY_SIZE(mt9v113_confs),
};

static struct msm_sensor_ctrl_t mt9v113_s_ctrl = {
	.msm_sensor_reg = &mt9v113_regs,
	.sensor_i2c_client = &mt9v113_sensor_i2c_client,
	.sensor_i2c_addr = 0x78,
	.vreg_seq = mt9v113_veg_seq,
	.num_vreg_seq = ARRAY_SIZE(mt9v113_veg_seq),
	.sensor_output_reg_addr = &mt9v113_reg_addr,
	.sensor_id_info = &mt9v113_id_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.min_delay = 30,
	.power_seq_delay = 60,
	.msm_sensor_mutex = &mt9v113_mut,
	.sensor_i2c_driver = &mt9v113_i2c_driver,
	.sensor_v4l2_subdev_info = mt9v113_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(mt9v113_subdev_info),
	.sensor_v4l2_subdev_ops = &mt9v113_subdev_ops,
	.func_tbl = &mt9v113_func_tbl,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Aptina 1.26MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
