/*! linux/drivers/media/video/nexel/ov7675.c
 *
 * OmniVision OV7675 CMOS Image Sensor driver
 * Copyright(c) 2013~2014 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 * ksw:
 * It is just preliminary version with minimal resolution support.
 * It seems their sensor doesn't support other then VGA mode.
 * Register names are very similar with OV7670 but most of 
 *  mode related registers are not working properly.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/videodev2.h>
#include <linux/errno.h>

#include <mach/devices.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>

#include "nx_vip.h"
#include "ov7675.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */
struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

struct ov7675_state {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler handler;
	/* standard */
	struct v4l2_ctrl *focus;
	struct v4l2_ctrl *wb;
	struct v4l2_ctrl *color_effect;
	struct v4l2_ctrl *exposure;
	/* custom */
	struct v4l2_ctrl *scene_mode;
	struct v4l2_ctrl *anti_shake;
	struct v4l2_ctrl *mode_change;

	bool inited;
	int width;
	int height;
	int mode; // PREVIEW or CAPTURE
	int id;

	/* for zoom */
	struct v4l2_rect crop;

	struct regulator *cam_core_18V;
	struct regulator *cam_io_28V;

#ifdef USE_INITIAL_WORKER_THREAD
	struct workqueue_struct *init_wq;
	struct work_struct init_work;
#endif
};

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */
static struct regval_list ov7675_default_regs[] = {
	{ 0x04,0x40 },// CCIR656
	{ 0x11,0x80 },// 30fps(80), 15fps(00)
	{ 0x3a,0x0C },
	{ 0x3D,0xC0 },
	{ 0x12,0x00 },
	{ 0x15,0x40 },
	{ 0xc1,0x7f },
	{ 0x17,0x13 },
	{ 0x18,0x01 },
	{ 0x32,0x3f },
	{ 0x19,0x02 },
	{ 0x1a,0x7a },//7b
	{ 0x03,0x2f },
	{ 0x0c,0x00 },
	{ 0x3e,0x00 },
	{ 0x70,0x3a },
	{ 0x71,0x35 },
	{ 0x72,0x11 },
	{ 0x73,0xf0 },
	{ 0xa2,0x02 },
	{ 0x7a,0x24 },
	{ 0x7b,0x04 },
	{ 0x7c,0x07 },
	{ 0x7d,0x10 },
	{ 0x7e,0x38 },
	{ 0x7f,0x4a },
	{ 0x80,0x5a },
	{ 0x81,0x67 },
	{ 0x82,0x71 },
	{ 0x83,0x7b },
	{ 0x84,0x85 },
	{ 0x85,0x95 },
	{ 0x86,0xa4 },
	{ 0x87,0xbc },
	{ 0x88,0xd2 },
	{ 0x89,0xe5 },
	{ 0x00,0x00 },
	{ 0x0d,0x40 },
	{ 0x14,0x28 },
	{ 0xa5,0x06 },
	{ 0xab,0x07 },
	{ 0x24,0x58 },
	{ 0x25,0x48 },
	{ 0x26,0x93 },
	{ 0x9f,0x78 },
	{ 0xa0,0x68 },
	{ 0xa1,0x03 },
	{ 0xa6,0xD8 },
	{ 0xa7,0xD8 },
	{ 0xa8,0xf0 },
	{ 0xa9,0x90 },
	{ 0xaa,0x14 },
	{ 0x0e,0x61 },
	{ 0x0f,0x4b },
	{ 0x16,0x02 },
	{ 0x1e,0x07 },
	{ 0x21,0x02 },
	{ 0x22,0x91 },
	{ 0x29,0x07 },
	{ 0x33,0x0b },
	{ 0x35,0x0b },
	{ 0x37,0x1d },
	{ 0x38,0x71 },
	{ 0x39,0x2a },
	{ 0x3c,0x78 },
	{ 0x4d,0x40 },
	{ 0x4e,0x20 },
	{ 0x69,0x00 },
	{ 0x6b,0x0a },
	{ 0x74,0x10 },
	{ 0x8d,0x4f },
	{ 0x8e,0x00 },
	{ 0x8f,0x00 },
	{ 0x90,0x00 },
	{ 0x91,0x00 },
	{ 0x96,0x00 },
	{ 0x9a,0x80 },
	{ 0xb0,0x84 },
	{ 0xb1,0x0c },
	{ 0xb2,0x0e },
	{ 0xb3,0x82 },
	{ 0xb8,0x0a },
	{ 0xbb,0xa1 },//blc target
	{ 0x0d,0x60 },
	{ 0x42,0x80 },
	{ 0x43,0x0a },
	{ 0x44,0xf0 },
	{ 0x45,0x34 },
	{ 0x46,0x58 },
	{ 0x47,0x28 },
	{ 0x48,0x3a },
	{ 0x59,0x88 },
	{ 0x5a,0x88 },
	{ 0x5b,0xc2 },
	{ 0x5c,0x60 },
	{ 0x5d,0x58 },
	{ 0x5e,0x18 },
	{ 0x6c,0x0a },
	{ 0x6d,0x55 },
	{ 0x6e,0x11 },
	{ 0x6f,0x9e },
	{ 0x6a,0x40 },
	{ 0x01,0x56 },
	{ 0x02,0x44 },
	{ 0x07,0x00 },//exposure [5:0]
	{ 0x10,0x70 },//exposure [7:0]
	{ 0x04,0x40 },//exposure [1:0]
	{ 0x13,0xe2 },//manual exposure setting
	{ 0x4f,0xa6 },
	{ 0x50,0xb5 },
	{ 0x51,0x0f },
	{ 0x52,0x18 },
	{ 0x53,0x9d },
	{ 0x54,0xb5 },
	{ 0x58,0x1a },
	{ 0x3f,0x02 },
	{ 0x75,0x63 },
	{ 0x76,0xe1 },
	{ 0x4c,0x00 },
	{ 0x77,0x01 },
	{ 0x3D,0xC2 },
	{ 0x4b,0x09 },
	{ 0xc9,0x60 },
	{ 0x41,0x38 },
	{ 0x56,0x40 },
	{ 0x34,0x11 },
	{ 0x3b,0x0a },
	{ 0xa4,0x88 },
	{ 0x96,0x00 },
	{ 0x97,0x30 },
	{ 0x98,0x20 },
	{ 0x99,0x30 },
	{ 0x9a,0x84 },
	{ 0x9b,0x29 },
	{ 0x9c,0x03 },
	{ 0x9d,0x99 },
	{ 0x9e,0x7f },
	{ 0x78,0x04 },
	{ 0x79,0x01 },
	{ 0xc8,0xf0 },
	{ 0x79,0x0f },
	{ 0xc8,0x00 },
	{ 0x79,0x10 },
	{ 0xc8,0x7e },
	{ 0x79,0x0a },
	{ 0xc8,0x80 },
	{ 0x79,0x0b },
	{ 0xc8,0x01 },
	{ 0x79,0x0c },
	{ 0xc8,0x0f },
	{ 0x79,0x0d },
	{ 0xc8,0x20 },
	{ 0x79,0x09 },
	{ 0xc8,0x80 },
	{ 0x79,0x02 },
	{ 0xc8,0xc0 },
	{ 0x79,0x03 },
	{ 0xc8,0x40 },
	{ 0x79,0x05 },
	{ 0xc8,0x30 },
	{ 0x79,0x26 },
	{ 0x62,0x00 },
	{ 0x63,0x00 },
	{ 0x64,0x10 },
	{ 0x65,0x07 },
	{ 0x66,0x05 },
	{ 0x94,0x10 },
	{ 0x95,0x12 },
    { 0xff,0xff }, /* END MARKER */
};

static struct regval_list ov7675_fmt_yuv422[] = {
	{ OV7675_COM7, 0x0 },  /* Selects YUV mode */
	{ OV7675_RGB444, 0 },	/* No RGB444 please */
	{ OV7675_COM1, COM1_CCIR656 },
	{ OV7675_COM15, COM15_R00FF },
	{ OV7675_COM9, 0x48 }, /* 32x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0x80 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0x80 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x22 }, 	/* "matrix coefficient 4" */
	{ 0x53, 0x5e }, 	/* "matrix coefficient 5" */
	{ 0x54, 0x80 }, 	/* "matrix coefficient 6" */
	{ OV7675_COM13, COM13_GAMMA|COM13_UVSAT },
	{ 0xff, 0xff },
};

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct ov7675_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
	int bpp;   /* Bytes per pixel */
} ov7675_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= ov7675_fmt_yuv422,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 2,
	},
/*	{
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.regs		= ov7675_fmt_rgb444,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 2,
	},
	{
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.regs		= ov7675_fmt_rgb565,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.regs 		= ov7675_fmt_raw,
		.cmatrix	= { 0, 0, 0, 0, 0, 0 },
		.bpp		= 1
	}, */
};
#define N_OV7675_FMTS ARRAY_SIZE(ov7675_formats)

static struct ov7675_win_size {
	int	width;
	int	height;
	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
/* h/vref stuff */
} ov7675_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.com7_bit	= COM7_FMT_VGA,
		.hstart		= 158,		/* These values from */
		.hstop		=  14,		/* Omnivision */
		.vstart		=  10,
		.vstop		= 490,
		.regs 		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(ov7675_win_sizes))

static inline struct ov7675_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7675_state, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov7675_state, handler)->sd;
}


#define I2C_FLAG_READ   0x10

static int ov7675_i2c_xfer(struct i2c_client *client, u8 reg, char *buf, int num, int tran_flag)
{
	struct i2c_msg msg[2];
	int ret;
	char xbuf[128];

	if (tran_flag & I2C_FLAG_READ) {
	    msg[0].addr = client->addr;
	    msg[0].len = 1;
	    msg[0].buf = (char *)&reg;
	    msg[0].flags = 0;

	    msg[1].addr = client->addr;
	    msg[1].len = num;
	    msg[1].buf = buf;
	    msg[1].flags = I2C_M_RD;

	    ret = i2c_transfer(client->adapter, msg, 2);
	} else {
	    xbuf[0] = reg;
	    memcpy(&xbuf[1], buf, num);
	    msg[0].addr = client->addr;
	    msg[0].len = 1 + num;
	    msg[0].buf = xbuf;
	    msg[0].flags = 0;

	    ret = i2c_transfer(client->adapter, msg, 1);
	}

	if (ret >= 0)
		return 0;

	return ret;
}

static int ov7675_read_reg8(struct i2c_client *client, u8  reg, u8 *val)
{
	int ret;
	u8 rval;

	reg &= 0xFF;

	ret = ov7675_i2c_xfer(client, reg, (u8 *) &rval, 1, I2C_FLAG_READ);
	if (!ret) {
		*val = rval;
		return 0;
	}
	return ret;
}

static int ov7675_write_reg8(struct i2c_client *client, u8 reg, u8 val)
{
	u8 temp1;

	temp1 = reg & 0xFF;

	return ov7675_i2c_xfer(client, temp1, (u8 *) & val, 1, 0);
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7675_write_reg8_array(struct i2c_client *c, struct regval_list *vals)
{
	while (vals->reg_num != 0xff || vals->value != 0xff)
	{
		int ret = ov7675_write_reg8(c, vals->reg_num, vals->value);

		if (ret < 0)
			return ret;
		vals++;
	}

	return 0;
}

#if 0
/*
 * Store a set of start/stop values into the camera.
 */
static int ov7675_set_hw(struct i2c_client *client, int hstart, int hstop, int vstart, int vstop)
{
	int ret;
	unsigned char v;
	/*
	 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
	 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
	 * a mystery "edge offset" value in the top two bits of href.
	 */
	ret =  ov7675_write_reg8(client, OV7675_HSTART, (hstart >> 3) & 0xff);
	ret += ov7675_write_reg8(client, OV7675_HSTOP, (hstop >> 3) & 0xff);
	ret += ov7675_read_reg8(client, OV7675_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += ov7675_write_reg8(client, OV7675_HREF, v);

	/*
	 * Vertical: similar arrangement, but only 10 bits.
	 */
	ret += ov7675_write_reg8(client, OV7675_VSTART, (vstart >> 2) & 0xff);
	ret += ov7675_write_reg8(client, OV7675_VSTOP, (vstop >> 2) & 0xff);
	ret += ov7675_read_reg8(client, OV7675_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += ov7675_write_reg8(client, OV7675_VREF, v);

	return ret;
}
#endif

/*************************************************************************
* FUNCTION
*	config_OV7675_window
*
* DESCRIPTION
*	This function config the hardware window of OV7675 for getting specified
*  data of that window.
*
* PARAMETERS
*	start_x : start column of the interested window
*  start_y : start row of the interested window
*  width  : column widht of the itnerested window
*  height : row depth of the itnerested window
*
* RETURNS
*	the data that read from OV7675
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ov7675_config_window(struct i2c_client *client, u16 startx, u16 starty, u16 width, u16 height)
{
	u16 endx=(startx+width-1);
	u16 endy=(starty+height-1);
	u8  temp_reg1, temp_reg2;

	ov7675_read_reg8(client, 0x03, &temp_reg1);
	ov7675_read_reg8(client, 0x32, &temp_reg2);

	temp_reg1=temp_reg1&0xF0;
	temp_reg2=temp_reg2&0xC0;

	// Horizontal
	ov7675_write_reg8(client, 0x32,0x80|((endx&0x7)<<3)|(startx&0x7));	// b[5:3]:HREF end low 3bits. b[2:0]:HREF start low 3bits.
	ov7675_write_reg8(client, 0x17,(startx&0x7F8)>>3);			// HREF start high 8bits
	ov7675_write_reg8(client, 0x18,(endx&0x7F8)>>3);			// HREF end high 8bits
	// Vertical
	ov7675_write_reg8(client, 0x03,temp_reg1|((endy&0x3)<<2)|(starty&0x3));	// b[3:2]:VREF end low 2bits. b[1:0]:VREF start low 2bits.
	ov7675_write_reg8(client, 0x19,(starty&0x3FC)>>2);   			// VREF start high 8bits
	ov7675_write_reg8(client, 0x1A,(endy&0x3FC)>>2);		   	// VREF end high 8bits
}	/* config_OV7675_window */


static void ov7675_start(struct i2c_client *client)
{
	u8 val;

	ov7675_write_reg8(client, 0x12,COM7_RESET);
	msleep(10);

	ov7675_write_reg8_array(client, ov7675_default_regs);

	ov7675_read_reg8(client, OV7675_COM1, &val);
}

/*!
 * ov7675 get_i2c function
 *
 * @return  0 if success
 */
static int ov7675_get_i2c(struct i2c_client *client, int addr, int *val)
{
	return ov7675_read_reg8(client, addr, (u8 *)val);
}

/*!
 * ov7675 set_i2c function
 *
 * @return  0 if success
 */
static int ov7675_set_i2c(struct i2c_client *client, int addr, int val)
{
    //printk("ov7675 set i2c!, addr = 0x%x val = 0x%x\n", addr, val);
	return ov7675_write_reg8(client, addr, val);
}

/*
 * Stuff that knows about the sensor.
 */
static int ov7675_reset(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ov7675_write_reg8(client, OV7675_COM7, COM7_RESET);
	msleep(1);
	return 0;
}


static int ov7675_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov7675_write_reg8_array(client, ov7675_default_regs);
}

/*!
 * ov7675 detect
 *
 * @return 0(OK) or -NODEV
 */
static int ov7675_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val;
	int ret;
	
	/* read id register and compare to correct value.. */
	ret = ov7675_read_reg8(client, OV7675_PID, &val);
	if (ret)
		return ret;

	if (val != 0x0076)
		return -ENODEV;
	
	ret = ov7675_read_reg8(client, OV7675_VER, &val);
	if (ret)
		return ret;

	if (val != 0x0073)
		return -ENODEV;
	
	return 0;
}

/*!
 * enum_fmt, try_fmt
 * We only support YUV422, 640X480 only.
 *
 * No other modes are tested. So return ERROR if non supported mode specified.
 *
 */
 
static int ov7675_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	struct ov7675_format_struct *ofmt;

	if (fmt->index >= N_OV7675_FMTS)
		return -EINVAL;

	ofmt = ov7675_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;

	return 0;
}


static int ov7675_try_fmt_internal(struct v4l2_subdev *sd,
		struct v4l2_format *fmt,
		struct ov7675_format_struct **ret_fmt,
		struct ov7675_win_size **ret_wsize)
{
	int index;
	struct ov7675_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	for (index = 0; index < N_OV7675_FMTS; index++)
		if (ov7675_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_OV7675_FMTS) {
		/* default to first format */
		index = 0;
		pix->pixelformat = ov7675_formats[0].pixelformat;
	}
	if (ret_fmt != NULL)
		*ret_fmt = ov7675_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	pix->field = V4L2_FIELD_NONE;
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = ov7675_win_sizes; wsize < ov7675_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (pix->width >= wsize->width && pix->height >= wsize->height)
			break;
	if (wsize >= ov7675_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	pix->width = wsize->width;
	pix->height = wsize->height;
	pix->bytesperline = pix->width*ov7675_formats[index].bpp;
	pix->sizeimage = pix->height*pix->bytesperline;

	return 0;
}

static int ov7675_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	return ov7675_try_fmt_internal(sd, fmt, NULL, NULL);
}

/*
 * Set a format.
 */
static int ov7675_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int ret;
	struct ov7675_format_struct *ovfmt;
	struct ov7675_win_size *wsize;

	ret = ov7675_try_fmt_internal(sd, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov7675_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov7675_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}


/*
 * Code for dealing with controls.
 */
static int ov7675_s_sat(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov7675_g_sat(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int ov7675_s_hue(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int ov7675_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int ov7675_s_brightness(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov7675_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int ov7675_s_contrast(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov7675_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int ov7675_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int ov7675_s_hflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int ov7675_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int ov7675_s_vflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov7675_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	switch (qc->id) {
	case V4L2_CID_BRIGHTNESS:
		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
	case V4L2_CID_CONTRAST:
		return v4l2_ctrl_query_fill(qc, 0, 127, 1, 64);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_SATURATION:
		return v4l2_ctrl_query_fill(qc, 0, 256, 1, 128);
	case V4L2_CID_HUE:
		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	}
	return -EINVAL;
}

static int ov7675_subdev_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov7675_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return ov7675_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return ov7675_g_sat(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return ov7675_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return ov7675_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return ov7675_g_hflip(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int ov7675_subdev_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov7675_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return ov7675_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return ov7675_s_sat(sd, ctrl->value);
	case V4L2_CID_HUE:
		return ov7675_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return ov7675_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return ov7675_s_hflip(sd, ctrl->value);
	}
	return -EINVAL;
}

static int ov7675_s_ctrl(struct v4l2_ctrl *ctrl)
{
	// Do something...
	switch (ctrl->id) {
	default: ;
	}
	return -EINVAL;
}

static int ov7675_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV7670, 0);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov7675_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val = 0;
	int ret;

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ret = ov7675_read_reg8(client, reg->reg & 0xff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov7675_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ov7675_write_reg8(client, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

#if 0
static int ov7675_change_resolution(struct i2c_client *client, int res)
{
	switch (res) {
	case CAM_RES_QSVGA:
		return -1;

	case CAM_RES_VGA:
		// Do nothing
		break;
	

	case CAM_RES_MAX:	/* fall through */
	case CAM_RES_DEFAULT:	/* fall through */
		break;

	case CAM_RES_SVGA:
		return -1;

	default:
		return -1;
	}

	return 0;
}

static int ov7675_change_whitebalance(struct i2c_client *client, enum nx_vip_wb_t type)
{

	return 0;
}
#endif

static int ov7675_set_brightness(struct i2c_client *client, int val)
{
	s8 maxval;

    maxval = (s8)((val <= 50) ? (-254 * val / 100 - 1) : (255 * val / 100 - 128));
    ov7675_write_reg8(client,OV7675_BRIGHT, maxval&0xFF);

	return 0;
}

static int ov7675_set_set_power_save(struct i2c_client *client, int val)
{
	return 0;
}

static int ov7675_set_exposure(struct i2c_client *client, int val)
{
	u8	exph, expm, expl;
	u8	aec;

	ov7675_read_reg8(client,OV7675_COM8, &aec);

	aec |= COM8_AEC;
    ov7675_write_reg8(client,OV7675_COM8, aec);

	ov7675_read_reg8(client,OV7675_AECHH, &exph);
	ov7675_read_reg8(client,OV7675_AECH,  &expm);
	ov7675_read_reg8(client,OV7675_COM1,  &expl);

	exph = (exph & (~0x3F)) | ((val >> 10) & 0x3F);
	expm = (expm & (~0xFF)) | ((val >>  2) & 0xFF);
	expl = (expl & (~0x03)) | ((val >>  0) & 0x03);

	//0x04 COM1 Bit[1:0]: Exposure time, the unit is tRow interval
	//AEC[15:0] = {0x07[5:0], 0x10[7:0], 0x04[1:0]}
	//               6bit       8bit       2bit
	//             0x00       0x40       0x00
	exph = ov7675_write_reg8(client,OV7675_AECHH, exph);
	exph = ov7675_write_reg8(client,OV7675_AECH,  expm);
	exph = ov7675_write_reg8(client,OV7675_COM1,  expl);

	aec &= ~COM8_AEC;
    ov7675_write_reg8(client,OV7675_COM8, aec);

	return 0;
}

static int ov7675_set_fixed_frame(struct i2c_client *client, int val)
{
	ov7675_write_reg8(client,0x2a, 0x00);
	ov7675_write_reg8(client,0x2b, 0x00);
	ov7675_write_reg8(client,0x92, 0x00);
	ov7675_write_reg8(client,0x93, 0x00);
	ov7675_write_reg8(client,0x3b, 0x0a);

	if (val == 30)
	{
		ov7675_write_reg8(client,0x6b, 0x0a);	// PLL control Input clock bypass
		ov7675_write_reg8(client,0x11, 0x80);	// Internal clock pre-scalar(Input clock / 1)--datasheet ? x/1?
	}
	else if (val == 15)
	{
		ov7675_write_reg8(client,0x6b, 0x0a);	// PLL control Input clock bypass
		ov7675_write_reg8(client,0x11, 0x00);	// Internal clock pre-scalar(Input clock / 1)--datasheet ? x/1?
	}
	else if (val == 10)
	{
		ov7675_write_reg8(client,0x6b, 0x5a);	// PLL control [7:6]=0x01: Input clock x4
		ov7675_write_reg8(client,0x11, 0x05);	// Internal clock pre-scalar(Input clock / 6)
	}
	else if (val == 7)
	{
		ov7675_write_reg8(client,0x6b, 0x5a);	// PLL control [7:6]=0x01: Input clock x4
		ov7675_write_reg8(client,0x11, 0x07);	// Internal clock pre-scalar(Input clock / 7)
	}
	else
		return -1;

	return 0;
}

//gain = (0x03[7] + 1) x (0x03[6] + 1) x (0x00[7] + 1) x (0x00[6] + 1) x
//       (0x00[5] + 1) x (0x00[4] + 1) x (0x00[3:0] / 16 + 1)
static int ov7675_set_gain(struct i2c_client *client, int val)
{
	u8	reg_gl = 0;
	u8	reg_gh = 0;
	int	div = 0x40;
	int	gain;
	int i;

	if (val < 0 || val > 64)
		return -EINVAL;

	ov7675_read_reg8(client, OV7675_VREF, &reg_gh);
	ov7675_read_reg8(client, OV7675_GAIN, &reg_gl);

	reg_gh &= 0x3F;
	reg_gl &= 0x00;

	for (i = 0 ; i < 6; i++)
	{
		if (val & div)
			break;

		div >>= 1;
	}

	gain = 0;
	for ( ;i < 6; i++)
	{
		gain = (gain << 1) | 1;
	}

	reg_gh |= ((gain & 0x30) << 7);
	reg_gl |= ((gain & 0x0F) << 4);

	ov7675_write_reg8(client, OV7675_VREF, reg_gh);
	ov7675_write_reg8(client, OV7675_GAIN, reg_gl);

	return 0;
}

#if 0
static int ov7675_command(struct i2c_client *client, u32 cmd, void *arg)
{
	u8 val;

	switch (cmd) {
	case I2C_CAM_INIT:
		ov7675_start(client);
		info("OV7675 : external camera initialized\n");
		break;

	case I2C_CAM_RESOLUTION:
		return ov7675_change_resolution(client, (int) arg);

	case I2C_CAM_WB:
		ov7675_change_whitebalance(client, (enum nx_vip_wb_t) arg);
		break;

	case I2C_CAM_BRIGHTNESS:
		ov7675_set_brightness(client, (int) arg);
		break;

	case I2C_CAM_POWER_SAVE:
		ov7675_set_set_power_save(client, (int) arg);
		break;

	case I2C_CAM_EXPOSURE:
		ov7675_set_exposure(client, (int)arg);
		break;

	case I2C_CAM_FIXED_FRAME:
		ov7675_set_fixed_frame(client, (int)arg);
		break;

	case I2C_CAM_EXPOSURE_AUTO:
		ov7675_read_reg8(client, OV7675_COM8, &val);// Manual Exposure

		val &= ~(COM8_AGC | COM8_AEC);
		if ((int)arg)
			val |= (COM8_AGC | COM8_AEC);

		ov7675_write_reg8(client, OV7675_COM8, val);// Auto Exposure
		break;

	case I2C_CAM_GAIN:
		ov7675_set_gain(client, (int)arg);// Gain
		break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}
#endif

static const struct v4l2_ctrl_ops ov7675_ctrl_ops = {
	.s_ctrl = ov7675_s_ctrl,
};

#define NUM_CTRLS				7

#define V4L2_CID_CAMERA_SCENE_MODE		(V4L2_CTRL_CLASS_CAMERA | 0x1001)
#define V4L2_CID_CAMERA_ANTI_SHAKE		(V4L2_CTRL_CLASS_CAMERA | 0x1002)
#define V4L2_CID_CAMERA_MODE_CHANGE		(V4L2_CTRL_CLASS_CAMERA | 0x1003)

enum {
	WB_AUTO = 0,
	WB_DAYLIGHT,
	WB_CLOUDY,
	WB_FLUORESCENT,
	WB_INCANDESCENT,
	WB_MAX
};

enum {
	COLORFX_NONE = 0,
	COLORFX_SEPIA,
	COLORFX_AQUA,
	COLORFX_MONO,
	COLORFX_NEGATIVE,
	COLORFX_SKETCH,
	COLORFX_MAX
};

enum {
	SCENE_OFF = 0,
	SCENE_PORTRAIT,
	SCENE_LANDSCAPE,
	SCENE_SPORTS,
	SCENE_NIGHTSHOT,
	SCENE_MAX
};

enum {
	ANTI_SHAKE_OFF = 0,
	ANTI_SHAKE_50Hz,
	ANTI_SHAKE_60Hz,
	ANTI_SHAKE_MAX
};

#define MIN_EXPOSURE			-4
#define MAX_EXPOSURE			4

static const struct v4l2_ctrl_config ov7675_custom_ctrls[] = {
    {
        .ops    = &ov7675_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SCENE_MODE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "SceneMode",
        .min    = 0,
        .max    = SCENE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &ov7675_ctrl_ops,
        .id     = V4L2_CID_CAMERA_ANTI_SHAKE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "AntiShake",
        .min    = 0,
        .max    = ANTI_SHAKE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &ov7675_ctrl_ops,
        .id     = V4L2_CID_CAMERA_MODE_CHANGE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "ModeChange",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    },
};

static int ov7675_initialize_ctrls(struct ov7675_state *state)
{
    v4l2_ctrl_handler_init(&state->handler, NUM_CTRLS);

    /* standard */
    state->focus = v4l2_ctrl_new_std(&state->handler, &ov7675_ctrl_ops,
            V4L2_CID_FOCUS_AUTO, 0, 1, 1, 0);
    if (!state->focus) {
        pr_err("%s: failed to create focus ctrl\n", __func__);
        return -1;
    }
    state->wb = v4l2_ctrl_new_std(&state->handler, &ov7675_ctrl_ops,
            V4L2_CID_DO_WHITE_BALANCE, WB_AUTO, WB_MAX - 1, 1, WB_AUTO);
    if (!state->wb) {
        pr_err("%s: failed to create wb ctrl\n", __func__);
        return -1;
    }
    state->color_effect = v4l2_ctrl_new_std_menu(&state->handler, &ov7675_ctrl_ops,
            V4L2_CID_COLORFX, COLORFX_MAX - 1, 0, COLORFX_NONE);
    if (!state->color_effect) {
        pr_err("%s: failed to create color_effect ctrl\n", __func__);
        return -1;
    }
    state->exposure = v4l2_ctrl_new_std_menu(&state->handler, &ov7675_ctrl_ops,
            V4L2_CID_EXPOSURE, MAX_EXPOSURE, 0, 0);
    if (!state->exposure) {
        pr_err("%s: failed to create exposure ctrl\n", __func__);
        return -1;
    }

    /* custom */
    state->scene_mode = v4l2_ctrl_new_custom(&state->handler, &ov7675_custom_ctrls[0], NULL);
    if (!state->scene_mode) {
        pr_err("%s: failed to create scene_mode ctrl\n", __func__);
        return -1;
    }
    state->anti_shake = v4l2_ctrl_new_custom(&state->handler, &ov7675_custom_ctrls[1], NULL);
    if (!state->anti_shake) {
        pr_err("%s: failed to create anti_shake ctrl\n", __func__);
        return -1;
    }
    state->mode_change = v4l2_ctrl_new_custom(&state->handler, &ov7675_custom_ctrls[2], NULL);
    if (!state->mode_change) {
        pr_err("%s: failed to create mode_change ctrl\n", __func__);
        return -1;
    }

    state->sd.ctrl_handler = &state->handler;
    if (state->handler.error) {
        printk("%s: ctrl handler error(%d)\n", __func__, state->handler.error);
        v4l2_ctrl_handler_free(&state->handler);
        return -1;
    }
    return 0;
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov7675_core_ops = {
	.g_chip_ident = ov7675_g_chip_ident,
	.g_ctrl = ov7675_subdev_g_ctrl,
	.s_ctrl = ov7675_subdev_s_ctrl,
	.queryctrl = ov7675_queryctrl,
	.reset = ov7675_reset,
	.init = ov7675_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov7675_g_register,
	.s_register = ov7675_s_register,
#endif
};

static const struct v4l2_subdev_video_ops ov7675_video_ops = {
//	.enum_fmt = ov7675_enum_fmt,
//	.try_fmt = ov7675_try_fmt,
//	.s_fmt = ov7675_s_fmt,
	.s_parm = ov7675_s_parm,
	.g_parm = ov7675_g_parm,
};

static const struct v4l2_subdev_ops ov7675_ops = {
	.core = &ov7675_core_ops,
	.video = &ov7675_video_ops,
};

/* ----------------------------------------------------------------------- */

static int ov7675_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct ov7675_state *info;
	int ret;

	info = kzalloc(sizeof(struct ov7675_state), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &ov7675_ops);

	/* Make sure it's an ov7675 */
	ret = ov7675_detect(sd);
	if (ret) {
		v4l_dbg(1, debug, client,
			"chip found @ 0x%x (%s) is not an ov7675 chip.\n",
			client->addr << 1, client->adapter->name);
		kfree(info);
		return ret;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	//info->fmt = &ov7675_formats[0]

	if (strstr(id->name, "_2"))
		info->id = 1;
	else
		info->id = 0;

	printk("OV7675 name = %s, id=%d\n", id->name, info->id);

	/*info->client = client;
	info->i2c_read = ov7675_get_i2c;
	info->i2c_write = ov7675_set_i2c;

	info->order422 = CAM_ORDER422_8BIT_CBYCRY; //CAM_ORDER422_8BIT_YCBYCR
	info->max_width = 640;
	info->max_height = 480;
	info->cur_width = 640;
	info->cur_height = 480;
	info->def_width = 640;
	info->def_height = 480;
	info->cur_pixformat = V4L2_PIX_FMT_YUYV;
	info->bpp = 2;
	info->cam_id = 118;
	
	info->polarity.vsync = 1; */

	//nx_vip_register_subdev(sd);
	ret = ov7675_initialize_ctrls(info);
	if (ret < 0) {
		pr_err("%s: failed to initialize controls\n", __func__);
		kfree(info);
		return ret;
	}
	ov7675_start(client);
	info->inited = 1;

	return 0;
}


static int ov7675_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

#if defined(CONFIG_CAMERA1_OV7675) || defined(CONFIG_SOC_CAMERA_OV7675)
static const struct i2c_device_id ov7675_id[] = {
	{ "ov7675", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov7675_id);

static struct i2c_driver ov7675_i2c_driver = {
	.driver = {
		.name = "OV7675",
		.owner = THIS_MODULE,
	},
	.id_table     = ov7675_id,
	.probe        = ov7675_probe,
    .remove       = __devexit_p(ov7675_remove),
	//.command      = ov7675_command,
};
#endif

#if defined(CONFIG_CAMERA2_OV7675)
static const struct i2c_device_id ov7675_id2[] = {
	{ "ov7675_2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov7675_id2);

static struct i2c_driver ov7675_i2c_driver2 = {
	.driver = {
		.name = "OV7675_2",
		.owner = THIS_MODULE,
	},
	.id_table     = ov7675_id2,
	.probe        = ov7675_probe,
    .remove       = __devexit_p(ov7675_remove),
	.command      = ov7675_command,
};
#endif

static __init int _ov7675_init(void)
{
	int res;

#if defined(CONFIG_CAMERA1_OV7675) || defined(CONFIG_SOC_CAMERA_OV7675)
	res = i2c_add_driver(&ov7675_i2c_driver);
#endif
#if defined(CONFIG_CAMERA2_OV7675)
	res = i2c_add_driver(&ov7675_i2c_driver2);
#endif
	return res;
}

static __init void _ov7675_exit(void)
{
#if defined(CONFIG_CAMERA1_OV7675)
	i2c_del_driver(&ov7675_i2c_driver);
#endif
#if defined(CONFIG_CAMERA2_OV7675)
	i2c_del_driver(&ov7675_i2c_driver2);
#endif
}

module_init(_ov7675_init)
module_exit(_ov7675_exit)

MODULE_AUTHOR("Seungwoo Kim<ksw@stcube.com>");
MODULE_DESCRIPTION("OV7675 Camera Driver");
MODULE_LICENSE("GPL");

