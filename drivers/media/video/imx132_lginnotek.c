/**
 * Copyright(c) 2014 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 * Initializing code based on NVIDIA's imx132.c
 *
 * imx132.c - imx132 sensor driver
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **/
#define DEBUG 1

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/v4l2-mediabus.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include <media/soc_camera.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>

#define IMX132_SIZEOF_I2C_BUF 16
struct imx132_reg {
	u16 addr;
	u16 val;
};
struct imx132_info {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler handler;
	
	int    mode;

	struct i2c_client		*client;
	
	struct regulator *imx132_ext_reg1;
	struct regulator *imx132_ext_reg2;

	unsigned			edp_state;
	atomic_t			in_use;
};

#define IMX132_TABLE_WAIT_MS 0
#define IMX132_TABLE_END 1
#define IMX132_WAIT_MS 5
#define IMX132_FUSE_ID_SIZE 8

/* IMX132 default supported geometry */
#define IMX132_WIDTH			1920
#define IMX132_HEIGHT			1080

static struct imx132_reg mode_1920x1080[] = {
	/* Stand by */
	{0x0100, 0x00},
	{0x0101, 0x03},
	{IMX132_TABLE_WAIT_MS, IMX132_WAIT_MS},
	/* global settings */
	{0x3087, 0x53},
	{0x308B, 0x5A},
	{0x3094, 0x11},
	{0x309D, 0xA4},
	{0x30AA, 0x01},
	{0x30C6, 0x00},
	{0x30C7, 0x00},
	{0x3118, 0x2F},
	{0x312A, 0x00},
	{0x312B, 0x0B},
	{0x312C, 0x0B},
	{0x312D, 0x13},
	/* PLL Setting */
	{0x0305, 0x02},
	{0x0307, 0x42},
	{0x30A4, 0x02},
	{0x303C, 0x4B},
	/* Mode Setting */
	{0x0340, 0x04},
	{0x0341, 0x92},
	{0x0342, 0x08},
	{0x0343, 0xC8},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x1C},
	{0x0348, 0x07},
	{0x0349, 0xB7},
	{0x034A, 0x04},
	{0x034B, 0x93},
	{0x034C, 0x07},
	{0x034D, 0xB8},
	{0x034E, 0x04},
	{0x034F, 0x78},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x303D, 0x10},
	{0x303E, 0x5A},
	{0x3040, 0x08},
	{0x3041, 0x97},
	{0x3048, 0x00},
	{0x304C, 0x2F},
	{0x304D, 0x02},
	{0x3064, 0x92},
	{0x306A, 0x10},
	{0x309B, 0x00},
	{0x309E, 0x41},
	{0x30A0, 0x10},
	{0x30A1, 0x0B},
	{0x30B2, 0x00},
	{0x30D5, 0x00},
	{0x30D6, 0x00},
	{0x30D7, 0x00},
	{0x30D8, 0x00},
	{0x30D9, 0x00},
	{0x30DA, 0x00},
	{0x30DB, 0x00},
	{0x30DC, 0x00},
	{0x30DD, 0x00},
	{0x30DE, 0x00},
	{0x3102, 0x0C},
	{0x3103, 0x33},
	{0x3104, 0x18},
	{0x3105, 0x00},
	{0x3106, 0x65},
	{0x3107, 0x00},
	{0x3108, 0x06},
	{0x3109, 0x04},
	{0x310A, 0x04},
	{0x315C, 0x3D},
	{0x315D, 0x3C},
	{0x316E, 0x3E},
	{0x316F, 0x3D},
	{0x3301, 0x01},
	{0x3304, 0x07},
	{0x3305, 0x06},
	{0x3306, 0x19},
	{0x3307, 0x03},
	{0x3308, 0x0F},
	{0x3309, 0x07},
	{0x330A, 0x0C},
	{0x330B, 0x06},
	{0x330C, 0x0B},
	{0x330D, 0x07},
	{0x330E, 0x03},
	{0x3318, 0x61},
	{0x3322, 0x09},
	{0x3342, 0x00},
	{0x3348, 0xE0},
	/* Shutter gain Settings */
	{0x0202, 0x04},
	{0x0203, 0x33},
	/* Streaming */
	{0x0100, 0x01},
	{IMX132_TABLE_WAIT_MS, IMX132_WAIT_MS},
	{IMX132_TABLE_END, 0x00}
};
static struct imx132_reg mode_1976x1200[] = {
	/* Stand by */
	{0x0100, 0x00},
	{0x0101, 0x03},
	{IMX132_TABLE_WAIT_MS, IMX132_WAIT_MS},
	/* global settings */
	{0x3087, 0x53},
	{0x308B, 0x5A},
	{0x3094, 0x11},
	{0x309D, 0xA4},
	{0x30AA, 0x01},
	{0x30C6, 0x00},
	{0x30C7, 0x00},
	{0x3118, 0x2F},
	{0x312A, 0x00},
	{0x312B, 0x0B},
	{0x312C, 0x0B},
	{0x312D, 0x13},
	/* PLL Setting */
	{0x0305, 0x02},
	{0x0307, 0x21},
	{0x30A4, 0x02},
	{0x303C, 0x4B},
	/* Mode Setting */
	{0x0340, 0x04},
	{0x0341, 0xCA},
	{0x0342, 0x08},
	{0x0343, 0xC8},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x07},
	{0x0349, 0xB7},
	{0x034A, 0x04},
	{0x034B, 0xAF},
	{0x034C, 0x07},
	{0x034D, 0xB8},
	{0x034E, 0x04},
	{0x034F, 0xB0},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x303D, 0x10},
	{0x303E, 0x4A},
	{0x3040, 0x08},
	{0x3041, 0x97},
	{0x3048, 0x00},
	{0x304C, 0x2F},
	{0x304D, 0x02},
	{0x3064, 0x92},
	{0x306A, 0x10},
	{0x309B, 0x00},
	{0x309E, 0x41},
	{0x30A0, 0x10},
	{0x30A1, 0x0B},
	{0x30B2, 0x00},
	{0x30D5, 0x00},
	{0x30D6, 0x00},
	{0x30D7, 0x00},
	{0x30D8, 0x00},
	{0x30D9, 0x00},
	{0x30DA, 0x00},
	{0x30DB, 0x00},
	{0x30DC, 0x00},
	{0x30DD, 0x00},
	{0x30DE, 0x00},
	{0x3102, 0x0C},
	{0x3103, 0x33},
	{0x3104, 0x30},
	{0x3105, 0x00},
	{0x3106, 0xCA},
	{0x3107, 0x00},
	{0x3108, 0x06},
	{0x3109, 0x04},
	{0x310A, 0x04},
	{0x315C, 0x3D},
	{0x315D, 0x3C},
	{0x316E, 0x3E},
	{0x316F, 0x3D},
	{0x3301, 0x00},
	{0x3304, 0x07},
	{0x3305, 0x06},
	{0x3306, 0x19},
	{0x3307, 0x03},
	{0x3308, 0x0F},
	{0x3309, 0x07},
	{0x330A, 0x0C},
	{0x330B, 0x06},
	{0x330C, 0x0B},
	{0x330D, 0x07},
	{0x330E, 0x03},
	{0x3318, 0x67},
	{0x3322, 0x09},
	{0x3342, 0x00},
	{0x3348, 0xE0},
	/* Shutter gain Settings */
	{0x0202, 0x04},
	{0x0203, 0x33},
	/* Streaming */
	{0x0100, 0x01},
	{IMX132_TABLE_WAIT_MS, IMX132_WAIT_MS},
	{IMX132_TABLE_END, 0x00}
};
enum {
	IMX132_MODE_1920X1080,
	IMX132_MODE_1976X1200,
};
static struct imx132_reg *mode_table[] = {
	[IMX132_MODE_1920X1080] = mode_1920x1080,
	[IMX132_MODE_1976X1200] = mode_1976x1200,
};
static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static inline void
imx132_get_frame_length_regs(struct imx132_reg *regs, u32 frame_length)
{
	regs->addr = IMX132_FRAME_LEN_LINES_15_8;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = IMX132_FRAME_LEN_LINES_7_0;
	(regs + 1)->val = (frame_length) & 0xff;
}
static inline void
imx132_get_coarse_time_regs(struct imx132_reg *regs, u32 coarse_time)
{
	regs->addr = IMX132_COARSE_INTEGRATION_TIME_15_8;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX132_COARSE_INTEGRATION_TIME_7_0;
	(regs + 1)->val = (coarse_time) & 0xff;
}
static inline void
imx132_get_gain_reg(struct imx132_reg *regs, u16 gain)
{
	regs->addr = IMX132_ANA_GAIN_GLOBAL;
	regs->val = gain;
}
static int
imx132_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];
	if (!client->adapter)
		return -ENODEV;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;
	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;
	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2)
		return -EINVAL;
	*val = data[2];
	return 0;
}
static int
imx132_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	if (!client->adapter)
		return -ENODEV;
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;
	dev_err(&client->dev, "%s:i2c write failed, %x = %x\n",
			__func__, addr, val);
	return err;
}
static int imx132_i2c_wr_blk(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msg;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buf;
	if (i2c_transfer(client->adapter, &msg, 1) != 1)
		return -EIO;
	return 0;
}
static int
imx132_write_table(struct i2c_client *client,
			 const struct imx132_reg table[],
			 const struct imx132_reg override_list[],
			 int num_override_regs)
{
	int err;
	u8 i2c_transfer_buf[IMX132_SIZEOF_I2C_BUF];
	const struct imx132_reg *next;
	const struct imx132_reg *n_next;
	u8 *b_ptr = i2c_transfer_buf;
	u16 buf_count = 0;
	for (next = table; next->addr != IMX132_TABLE_END; next++) {
		if (next->addr == IMX132_TABLE_WAIT_MS) {
			msleep_range(next->val);
			continue;
		}
		if (!buf_count) {
			b_ptr = i2c_transfer_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xFF;
			buf_count = 2;
		}
		*b_ptr++ = next->val;
		buf_count++;
		n_next = next + 1;
		if ((n_next->addr == next->addr + 1) &&
			(n_next->addr != IMX132_TABLE_WAIT_MS) &&
			(buf_count < IMX132_SIZEOF_I2C_BUF) &&
			(n_next->addr != IMX132_TABLE_END))
				continue;
		err = imx132_i2c_wr_blk(client, i2c_transfer_buf, buf_count);
		if (err) {
			pr_err("%s:imx132_write_table:%d", __func__, err);
			return err;
		}
		buf_count = 0;
	}
	return 0;
}
static void imx132_edp_lowest(struct imx132_info *info)
{
	if (!info->edpc)
		return;
	info->edp_state = info->edpc->num_states - 1;
	dev_dbg(&info->i2c_client->dev, "%s %d\n", __func__, info->edp_state);
	if (edp_update_client_request(info->edpc, info->edp_state, NULL)) {
		dev_err(&info->i2c_client->dev, "THIS IS NOT LIKELY HAPPEN!\n");
		dev_err(&info->i2c_client->dev,
			"UNABLE TO SET LOWEST EDP STATE!\n");
	}
}
static void imx132_edp_throttle(unsigned int new_state, void *priv_data);
static void imx132_edp_register(struct imx132_info *info)
{
	struct edp_manager *edp_manager;
	struct edp_client *edpc = &info->pdata->edpc_config;
	int ret;
	info->edpc = NULL;
	if (!edpc->num_states) {
		dev_notice(&info->i2c_client->dev,
			"%s: NO edp states defined.\n", __func__);
		return;
	}
	strncpy(edpc->name, "imx132", EDP_NAME_LEN - 1);
	edpc->name[EDP_NAME_LEN - 1] = 0;
	edpc->private_data = info;
	edpc->throttle = imx132_edp_throttle;
	dev_dbg(&info->i2c_client->dev, "%s: %s, e0 = %d, p %d\n",
		__func__, edpc->name, edpc->e0_index, edpc->priority);
	for (ret = 0; ret < edpc->num_states; ret++)
		dev_dbg(&info->i2c_client->dev, "e%d = %d mA",
			ret - edpc->e0_index, edpc->states[ret]);
	edp_manager = edp_get_manager("battery");
	if (!edp_manager) {
		dev_err(&info->i2c_client->dev,
			"unable to get edp manager: battery\n");
		return;
	}
	ret = edp_register_client(edp_manager, edpc);
	if (ret) {
		dev_err(&info->i2c_client->dev,
			"unable to register edp client\n");
		return;
	}
	info->edpc = edpc;
	/* set to lowest state at init */
	imx132_edp_lowest(info);
}
static int imx132_edp_req(struct imx132_info *info, unsigned new_state)
{
	unsigned approved;
	int ret = 0;
	if (!info->edpc)
		return 0;
	dev_dbg(&info->i2c_client->dev, "%s %d\n", __func__, new_state);
	ret = edp_update_client_request(info->edpc, new_state, &approved);
	if (ret) {
		dev_err(&info->i2c_client->dev, "E state transition failed\n");
		return ret;
	}
	if (approved > new_state) {
		dev_err(&info->i2c_client->dev, "EDP no enough current\n");
		return -ENODEV;
	}
	info->edp_state = approved;
	return 0;
}

static int
imx132_set_mode(struct imx132_info *info, struct imx132_mode *mode)
{
	struct device *dev = &info->i2c_client->dev;
	int sensor_mode;
	int err;
	struct imx132_reg reg_list[5];
	dev_info(dev, "%s: res [%ux%u] framelen %u coarsetime %u gain %u\n",
		__func__, mode->xres, mode->yres,
		mode->frame_length, mode->coarse_time, mode->gain);
	if ((mode->xres == 1920) && (mode->yres == 1080)) {
		sensor_mode = IMX132_MODE_1920X1080;
	} else if ((mode->xres == 1976) && (mode->yres == 1200)) {
		sensor_mode = IMX132_MODE_1976X1200;
	} else {
		dev_err(dev, "%s: invalid resolution to set mode %d %d\n",
			__func__, mode->xres, mode->yres);
		return -EINVAL;
	}
	/* request highest edp state */
	err = imx132_edp_req(info, 0);
	if (err) {
		dev_err(&info->i2c_client->dev,
			"%s: ERROR cannot set edp state! %d\n",	__func__, err);
		return err;
	}
	/*
	 * get a list of override regs for the asking frame length,
	 * coarse integration time, and gain.
	 */
	imx132_get_frame_length_regs(reg_list, mode->frame_length);
	imx132_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	imx132_get_gain_reg(reg_list + 4, mode->gain);
	err = imx132_write_table(info->i2c_client, mode_table[sensor_mode],
			reg_list, 5);
	if (err)
		return err;
	info->mode = sensor_mode;
	dev_info(dev, "[imx132]: stream on.\n");
	return 0;
}
static int
imx132_get_status(struct imx132_info *info, u8 *dev_status)
{
	/* TBD */
	*dev_status = 0;
	return 0;
}
static int
imx132_set_frame_length(struct imx132_info *info,
				u32 frame_length,
				bool group_hold)
{
	struct imx132_reg reg_list[2];
	int i = 0;
	int ret;
	imx132_get_frame_length_regs(reg_list, frame_length);
	if (group_hold) {
		ret = imx132_write_reg(info->i2c_client,
					IMX132_GROUP_PARAM_HOLD, 0x01);
		if (ret)
			return ret;
	}
	for (i = 0; i < NUM_OF_FRAME_LEN_REG; i++) {
		ret = imx132_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
	if (group_hold) {
		ret = imx132_write_reg(info->i2c_client,
					IMX132_GROUP_PARAM_HOLD, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}
static int
imx132_set_coarse_time(struct imx132_info *info,
				u32 coarse_time,
				bool group_hold)
{
	int ret;
	struct imx132_reg reg_list[2];
	int i = 0;
	imx132_get_coarse_time_regs(reg_list, coarse_time);
	if (group_hold) {
		ret = imx132_write_reg(info->i2c_client,
					IMX132_GROUP_PARAM_HOLD,
					0x01);
		if (ret)
			return ret;
	}
	for (i = 0; i < NUM_OF_COARSE_TIME_REG; i++) {
		ret = imx132_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
	if (group_hold) {
		ret = imx132_write_reg(info->i2c_client,
					IMX132_GROUP_PARAM_HOLD, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}
static int
imx132_set_gain(struct imx132_info *info, u16 gain, bool group_hold)
{
	int ret;
	struct imx132_reg reg_list;
	imx132_get_gain_reg(&reg_list, gain);
	if (group_hold) {
		ret = imx132_write_reg(info->i2c_client,
					IMX132_GROUP_PARAM_HOLD, 0x1);
		if (ret)
			return ret;
	}
	ret = imx132_write_reg(info->i2c_client, reg_list.addr, reg_list.val);
	if (ret)
		return ret;
	if (group_hold) {
		ret = imx132_write_reg(info->i2c_client,
					IMX132_GROUP_PARAM_HOLD, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}
static int
imx132_set_group_hold(struct imx132_info *info, struct imx132_ae *ae)
{
	int ret;
	int count = 0;
	bool groupHoldEnabled = false;
	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		groupHoldEnabled = true;
	if (groupHoldEnabled) {
		ret = imx132_write_reg(info->i2c_client,
					IMX132_GROUP_PARAM_HOLD, 0x1);
		if (ret)
			return ret;
	}
	if (ae->gain_enable)
		imx132_set_gain(info, ae->gain, false);
	if (ae->coarse_time_enable)
		imx132_set_coarse_time(info, ae->coarse_time, false);
	if (ae->frame_length_enable)
		imx132_set_frame_length(info, ae->frame_length, false);
	if (groupHoldEnabled) {
		ret = imx132_write_reg(info->i2c_client,
					IMX132_GROUP_PARAM_HOLD, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}
static int imx132_get_fuse_id(struct imx132_info *info)
{
	int ret = 0;
	int i;
	u8 bak = 0;
	if (info->fuse_id.size)
		return 0;
	/*
	 * TBD 1: If the sensor does not have power at this point
	 * Need to supply the power, e.g. by calling power on function
	 */
	ret |= imx132_write_reg(info->i2c_client, 0x34C9, 0x10);
	for (i = 0; i < IMX132_FUSE_ID_SIZE ; i++) {
		ret |= imx132_read_reg(info->i2c_client, 0x3580 + i, &bak);
		info->fuse_id.data[i] = bak;
	}
	if (!ret)
		info->fuse_id.size = i;
	return ret;
}

static long
imx132_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct imx132_info *info = file->private_data;
	struct device *dev = &info->i2c_client->dev;
	switch (cmd) {
	case IMX132_IOCTL_SET_MODE:
	{
		struct imx132_mode mode;
		if (copy_from_user(&mode,
			(const void __user *)arg,
			sizeof(struct imx132_mode))) {
			dev_err(dev, "%s:Failed to get mode from user.\n",
			__func__);
			return -EFAULT;
		}
		return imx132_set_mode(info, &mode);
	}
	case IMX132_IOCTL_SET_FRAME_LENGTH:
		return imx132_set_frame_length(info, (u32)arg, true);
	case IMX132_IOCTL_SET_COARSE_TIME:
		return imx132_set_coarse_time(info, (u32)arg, true);
	case IMX132_IOCTL_SET_GAIN:
		return imx132_set_gain(info, (u16)arg, true);
	case IMX132_IOCTL_GET_STATUS:
	{
		u8 status;
		err = imx132_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 1)) {
			dev_err(dev, "%s:Failed to copy status to user.\n",
			__func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX132_IOCTL_GET_FUSEID:
	{
		err = imx132_get_fuse_id(info);
		if (err) {
			dev_err(dev, "%s:Failed to get fuse id info.\n",
			__func__);
			return err;
		}
		if (copy_to_user((void __user *)arg,
				&info->fuse_id,
				sizeof(struct nvc_fuseid))) {
			dev_info(dev, "%s:Fail copy fuse id to user space\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX132_IOCTL_SET_GROUP_HOLD:
	{
		struct imx132_ae ae;
		if (copy_from_user(&ae, (const void __user *)arg,
				sizeof(struct imx132_ae))) {
			dev_info(dev, "%s:fail group hold\n", __func__);
			return -EFAULT;
		}
		return imx132_set_group_hold(info, &ae);
	}
	default:
		dev_err(dev, "%s:unknown cmd.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int imx132_get_extra_regulators(void)
{
	if (!imx132_ext_reg1) {
		imx132_ext_reg1 = regulator_get(NULL, "imx132_reg1");
		if (WARN_ON(IS_ERR(imx132_ext_reg1))) {
			pr_err("%s: can't get regulator imx132_reg1: %ld\n",
				__func__, PTR_ERR(imx132_ext_reg1));
			imx132_ext_reg1 = NULL;
			return -ENODEV;
		}
	}
	if (!imx132_ext_reg2) {
		imx132_ext_reg2 = regulator_get(NULL, "imx132_reg2");
		if (unlikely(WARN_ON(IS_ERR(imx132_ext_reg2)))) {
			pr_err("%s: can't get regulator imx132_reg2: %ld\n",
				__func__, PTR_ERR(imx132_ext_reg2));
			imx132_ext_reg2 = NULL;
			regulator_put(imx132_ext_reg1);
			return -ENODEV;
		}
	}
	return 0;
}
static void imx132_mclk_disable(struct imx132_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int imx132_mclk_enable(struct imx132_info *info)
{
	int err;
	unsigned long mclk_init_rate = 24000000;
	dev_dbg(&info->i2c_client->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);
	err = clk_set_rate(info->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(info->mclk);
	return err;
}

static int imx132_power_on(struct imx132_info *info)
{
	int err;
	struct imx132_power_rail *pw = &info->power;
	unsigned int cam2_gpio = info->pdata->cam2_gpio;
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;
	if (info->pdata->ext_reg) {
		if (imx132_get_extra_regulators())
			goto imx132_poweron_fail;
		err = regulator_enable(imx132_ext_reg1);
		if (unlikely(err))
			goto imx132_i2c_fail;
		err = regulator_enable(imx132_ext_reg2);
		if (unlikely(err))
			goto imx132_vcm_fail;
	}
	gpio_set_value(cam2_gpio, 0);
	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto imx132_avdd_fail;
	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto imx132_dvdd_fail;
	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto imx132_iovdd_fail;
	usleep_range(1, 2);
	gpio_set_value(cam2_gpio, 1);
	return 0;
imx132_iovdd_fail:
	regulator_disable(pw->dvdd);
imx132_dvdd_fail:
	regulator_disable(pw->avdd);
imx132_avdd_fail:
	if (info->pdata->ext_reg)
		regulator_disable(imx132_ext_reg2);
imx132_vcm_fail:
	if (info->pdata->ext_reg)
		regulator_disable(imx132_ext_reg1);
imx132_i2c_fail:
imx132_poweron_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int imx132_power_off(struct imx132_info *info)
{
	struct imx132_power_rail *pw = &info->power;
	unsigned int cam2_gpio = info->pdata->cam2_gpio;
	if (!info->i2c_client->dev.of_node) {
		if (info->pdata && info->pdata->power_off)
			info->pdata->power_off(&info->power);
		goto imx132_pwroff_end;
	}
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;
	gpio_set_value(cam2_gpio, 0);
	usleep_range(1, 2);
	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);
	if (info->pdata->ext_reg) {
		regulator_disable(imx132_ext_reg1);
		regulator_disable(imx132_ext_reg2);
	}
imx132_pwroff_end:
	imx132_edp_lowest(info);
	return 0;
}

static void imx132_edp_throttle(unsigned int new_state, void *priv_data)
{
	struct imx132_info *info = priv_data;
	imx132_power_off(info);
}


static int imx132_power_put(struct imx132_power_rail *pw)
{
	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);
	if (likely(pw->avdd))
		regulator_put(pw->avdd);
	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);
	if (likely(imx132_ext_reg1))
		regulator_put(imx132_ext_reg1);
	if (likely(imx132_ext_reg2))
		regulator_put(imx132_ext_reg2);
	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;
	imx132_ext_reg1 = NULL;
	imx132_ext_reg2 = NULL;
	return 0;
}
static int imx132_regulator_get(struct imx132_info *info,
	struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;
	reg = regulator_get(&info->i2c_client->dev, vreg_name);
	if (unlikely(IS_ERR(reg))) {
		dev_err(&info->i2c_client->dev, "%s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&info->i2c_client->dev, "%s: %s\n",
			__func__, vreg_name);
	*vreg = reg;
	return err;
}

static int imx132_power_get(struct imx132_info *info)
{
	struct imx132_power_rail *pw = &info->power;
	imx132_regulator_get(info, &pw->dvdd, "vdig"); /* digital 1.2v */
	imx132_regulator_get(info, &pw->avdd, "vana"); /* analog 2.7v */
	imx132_regulator_get(info, &pw->iovdd, "vif"); /* interface 1.8v */
	return 0;
}

static struct imx132_info *to_imx132(const struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx132_info, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct imx132_info, handler)->sd;
}

/* Find a data format by a pixel code in an array */
static const struct imx132_datafmt *imx132_find_datafmt(enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(imx132_colour_fmts); i++)
		if (imx132_colour_fmts[i].code == code)
			return imx132_colour_fmts + i;

	return NULL;
}

static int imx132_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	const struct imx132_datafmt *fmt = imx132_find_datafmt(mf->code);

	dev_dbg(sd->v4l2_dev->dev, "%s(%u)\n", __func__, mf->code);

	if (!fmt) {
		mf->code	= imx132_colour_fmts[0].code;
		mf->colorspace	= imx132_colour_fmts[0].colorspace;
	}

	mf->width	= IMX132_WIDTH;
	mf->height	= IMX132_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int imx132_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx132_info *info = to_imx132(sd);

	dev_dbg(sd->v4l2_dev->dev, "%s(%u)\n", __func__, mf->code);

	/* MIPI CSI could have changed the format, double-check */
	if (!imx132_find_datafmt(mf->code))
		return -EINVAL;

	imx132_try_fmt(sd, mf);

	info->fmt = imx132_find_datafmt(mf->code);

	return 0;
}

static int imx132_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx132_info *priv = to_imx132(client);

	const struct imx132_datafmt *fmt = priv->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= IMX132_WIDTH;
	mf->height	= IMX132_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int imx132_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct v4l2_rect *rect = &a->c;

	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	rect->top	= 0;
	rect->left	= 0;
	rect->width	= IMX132_WIDTH;
	rect->height	= IMX132_HEIGHT;

	return 0;
}

static int imx132_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= IMX132_WIDTH;
	a->bounds.height		= IMX132_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int imx132_set_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	int err = 0;
	struct v4l2_mbus_framefmt *_fmt = &fmt->format;
	struct imx132_info *info = to_imx132(sd);

	pr_debug("%s: %dx%d\n", __func__, _fmt->width, _fmt->height);

	err = imx132_s_fmt(sd, _fmt);

	return err;
}


static int imx132_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

    //int value = ctrl->val;
    int err = 0;

	v4l_info(client,"%s: ctrl->id:%d\n", __func__, ctrl->id);

    switch (ctrl->id) {
        case V4L2_CID_FOCUS_AUTO:
            break;

        case V4L2_CID_DO_WHITE_BALANCE:
            break;

        case V4L2_CID_COLORFX:
            break;

        case V4L2_CID_EXPOSURE:
            break;

        /* custom */
        case V4L2_CID_CAMERA_SCENE_MODE:
            break;

        case V4L2_CID_CAMERA_ANTI_SHAKE:
            break;

        case V4L2_CID_CAMERA_MODE_CHANGE:
            break;

        default:
            pr_err("%s: no such control\n", __func__);
            break;
    }

    return err;
}

static const struct v4l2_ctrl_ops imx132_ctrl_ops = {
	.s_ctrl = imx132_s_ctrl,
};

static const struct v4l2_ctrl_config imx132_custom_ctrls[] = {
    {
        .ops    = &imx132_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SCENE_MODE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "SceneMode",
        .min    = 0,
        .max    = SCENE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &imx132_ctrl_ops,
        .id     = V4L2_CID_CAMERA_ANTI_SHAKE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "AntiShake",
        .min    = 0,
        .max    = ANTI_SHAKE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &imx132_ctrl_ops,
        .id     = V4L2_CID_CAMERA_MODE_CHANGE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "ModeChange",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    },
};

static int imx132_initialize_ctrls(struct imx132_info *info)
{
    v4l2_ctrl_handler_init(&info->handler, NUM_CTRLS);

    /* standard */
    info->focus = v4l2_ctrl_new_std(&info->handler, &imx132_ctrl_ops,
            V4L2_CID_FOCUS_AUTO, 0, 1, 1, 0);
    if (!info->focus) {
        pr_err("%s: failed to create focus ctrl\n", __func__);
        return -1;
    }
    info->wb = v4l2_ctrl_new_std(&info->handler, &imx132_ctrl_ops,
            V4L2_CID_DO_WHITE_BALANCE, WB_AUTO, WB_MAX - 1, 1, WB_AUTO);
    if (!info->wb) {
        pr_err("%s: failed to create wb ctrl\n", __func__);
        return -1;
    }
    info->color_effect = v4l2_ctrl_new_std_menu(&info->handler, &imx132_ctrl_ops,
            V4L2_CID_COLORFX, COLORFX_MAX - 1, 0, COLORFX_NONE);
    if (!info->color_effect) {
        pr_err("%s: failed to create color_effect ctrl\n", __func__);
        return -1;
    }
   
    info->exposure = v4l2_ctrl_new_std_menu(&state->handler, &mt9d111_ctrl_ops,
            V4L2_CID_EXPOSURE, MAX_EXPOSURE, 0, 0);
    if (!info->exposure) {
        pr_err("%s: failed to create exposure ctrl\n", __func__);
        return -1;
    }

    /* custom */
    info->scene_mode = v4l2_ctrl_new_custom(&info->handler, &imx132_custom_ctrls[0], NULL);
    if (!info->scene_mode) {
        pr_err("%s: failed to create scene_mode ctrl\n", __func__);
        return -1;
    }
    info->anti_shake = v4l2_ctrl_new_custom(&info->handler, &imx132_custom_ctrls[1], NULL);
    if (!info->anti_shake) {
        pr_err("%s: failed to create anti_shake ctrl\n", __func__);
        return -1;
    }
    info->mode_change = v4l2_ctrl_new_custom(&info->handler, &imx132_custom_ctrls[2], NULL);
    if (!info->mode_change) {
        pr_err("%s: failed to create mode_change ctrl\n", __func__);
        return -1;
    }

    info->sd.ctrl_handler = &info->handler;
    if (info->handler.error) {
        printk("%s: ctrl handler error(%d)\n", __func__, info->handler.error);
        v4l2_ctrl_handler_free(&info->handler);
        return -1;
    }
    return 0;
}


static int imx132_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if ((unsigned int)index >= ARRAY_SIZE(imx132_colour_fmts))
		return -EINVAL;

	*code = imx132_colour_fmts[index].code;
	return 0;
}

static int imx132_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t val=0;
	uint8_t val1=0;

	imx132_read_reg(client, IMX132_MODEL_ID_H, &val);
	imx132_read_reg(client, IMX132_MODEL_ID_L, &val1);

	v4l_info(client,"################################## \n");
	v4l_info(client,"#  Check for imx132 \n");
	v4l_info(client,"#  Read ID = H:0x%02x, L:0x%02x \n", val, val1);	
	v4l_info(client,"################################## \n");

	if(val != 0x01 || val1 != 0x32 )
	{
		return -1;
	}

	return 0;
}

static void imx132_set_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l_info(client, "%s enter\n", __func__);
#if 0 
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx132_info *info = to_imx132(sd);
	int i2c_id = i2c_adapter_id(client->adapter);
	unsigned int reset_en=0;
	int pwm_ch=0;

	v4l_info(client,"%s() id:%d, on:%d\n", __func__, i2c_id, on);

    switch (i2c_id) {
		case 1:
			pwm_ch = 0;
			reset_en = CFG_IO_CAM0_RESET;
			break;
		case 2:
			pwm_ch = 1;
			reset_en = CFG_IO_CAM1_RESET;
			break;
		default:
			v4l_info(client,"%s() id:%d, unkown id!!!\n", __func__, i2c_id);
			return;
    }

	if (on)
	{
		nxp_soc_gpio_set_io_dir(reset_en, 1);
		nxp_soc_gpio_set_io_func(reset_en, nxp_soc_gpio_get_altnum(reset_en));
		nxp_soc_gpio_set_out_value(reset_en, 0);
		mdelay(10);
		nxp_soc_gpio_set_out_value(reset_en, 1);
		mdelay(1);
		nxp_soc_pwm_set_frequency(pwm_ch, 24000000, 50);
	}
	else 
	{
		nxp_soc_pwm_set_frequency(pwm_ch, 0, 0);
		mdelay(1);
		nxp_soc_gpio_set_out_value(reset_en, 0);
		state->inited = false;
	}
#endif 
	v4l_info(client, "%s exit.\n", __func__);
}

static int imx132_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* MODE_SELECT: stream or standby */
	return reg_write(client, MODE_SELECT, !!enable);
}

static int imx132_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_2_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static const struct v4l2_subdev_pad_ops imx132_pad_ops = {
	.set_fmt  = imx132_set_fmt,
	.set_crop = imx132_set_crop,
	.get_crop = imx132_get_crop,
};

static struct v4l2_subdev_video_ops imx132_subdev_video_ops = {
	.s_stream	= imx132_s_stream,
	//.s_mbus_fmt	= imx132_s_fmt,
	//.g_mbus_fmt	= imx132_g_fmt,
	//.try_mbus_fmt	= imx132_try_fmt,
	//.enum_mbus_fmt	= imx132_enum_fmt,
	//.g_crop		= imx132_g_crop,
	//.cropcap	= imx132_cropcap,
	//.g_mbus_config	= imx132_g_mbus_config,
};

static struct v4l2_subdev_core_ops imx132_subdev_core_ops = {
	.s_power = imx132_s_power,
	.s_ctrl = v4l2_subdev_s_ctrl,
};

static struct v4l2_subdev_ops imx132_subdev_ops = {
	.core	= &imx132_subdev_core_ops,
	.video	= &imx132_subdev_video_ops,
	.pad    = &imx132_pad_ops,
};

/**
 * media_entity_operations
 */
static int _link_setup(struct media_entity *entity,
							const struct media_pad *local,
							const struct media_pad *remote, u32 flags)
{
	/* printk("%s: entered\n", __func__); */
	return 0;
}

static const struct media_entity_operations imx132_media_ops = {
	.link_setup = _link_setup,
};

static int
imx132_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct imx132_info *info;
	const char *mclk_name;
	int err = 0;

	pr_info("[imx132]: probing sensor.\n");
	info = devm_kzalloc(&client->dev,
		sizeof(struct imx132_info), GFP_KERNEL);
	if (!info) {
		pr_err("[imx132]:%s:Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &imx132_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.ops = &imx132_media_ops;
	if (media_entity_init(&sd->entity, 1, &info->pad, 0)) {
		dev_err(&client->dev, "%s: failed to media_entity_init()\n", __func__);
		kfree(info);
		return -ENOENT;
	}

	err = imx132_initialize_ctrls(info);
	if (err < 0) {
		pr_err("%s: failed to initialize controls\n", __func__);
		kfree(info);
		return err;
	}
	
	// Now check IMX132 exists...
	err = imx132_detect(sd);
	if (err) {
		v4l_dbg(1, debug, client,
			"chip found @ 0x%x (%s) is not a imx132 chip.\n",
			client->addr << 1, client->adapter->name);
		kfree(info);
		return -ENODEV;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

/*
	imx132_power_get(info);
	imx132_edp_register(info);
	if (err) {
		imx132_power_put(&info->power);
		pr_err("[imx132]:%s:Unable to register misc device!\n",
		__func__);
	}
*/
	
	return err;
}
static int
imx132_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_entity_cleanup(&sd->entity);
	//imx132_power_put(&info->power);
	kfree(to_imx132(sd));

	return 0;
}
static const struct i2c_device_id imx132_id[] = {
	{ "imx132", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx132_id);
static struct i2c_driver imx132_i2c_driver = {
	.driver = {
		.name = "imx132",
		.owner = THIS_MODULE,
	},
	.probe = imx132_probe,
	.remove = imx132_remove,
	.id_table = imx132_id,
};
static int __init
imx132_init(void)
{
	pr_info("[imx132] sensor driver loading\n");
	return i2c_add_driver(&imx132_i2c_driver);
}
static void __exit
imx132_exit(void)
{
	i2c_del_driver(&imx132_i2c_driver);
}
module_init(imx132_init);
module_exit(imx132_exit);

MODULE_DESCRIPTION("Sony IMX132 Camera driver");
MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_LICENSE("GPL v2");
