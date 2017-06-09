/*! linux/drivers/media/video/nexel/ov2643.c
 *
 * OmniVision OV2643 CMOS Image Sensor driver
 * Copyright(c) 2017 I4VINE Inc.,
 * All right reserved by JuYoung Ryu <jyryu@stcube.com>
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
#define	DEBUG	1
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

#include <mach/soc.h>
#include "nx_vip.h"
#include "ov2643.h"
#include "./nexell/capture/cap_i2c.h"
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define   CMATRIX_LEN 6

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

struct ov2643_state {
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
static struct regval_list ov2643_default_regs[] = {
	{ OV2643_AEC2, 0x80 },
	{ OV2643_RED, 0x40 },
	{ OV2643_GREEN, 0x40 },
	{ OV2643_BLUE, 0x40 },
	{ OV2643_PIDH, 0x26 },
	{ OV2643_PIDL, 0x43 },
	{ OV2643_CTRL2, 0x14 },
	{ OV2643_PLL1, 0x9C },
	{ OV2643_PLL2, 0x28 },
	{ OV2643_PLL3, 0x0B },
	{ OV2643_CLK, 0x40 },
	{ OV2643_AUTO1, 0xFD },//FC Manual, FF Auto
	{ OV2643_AUTO2, 0xA7 },
	{ OV2643_AUTO3, 0x42 },
	{ OV2643_AUTO4, 0xA1 },
	{ OV2643_AUTO5, 0x40 },
	{ OV2643_WPT, 0x78 },
	{ OV2643_BPT, 0x68 },
	{ OV2643_VPT, 0xD4 },
	{ OV2643_VSOPT, 0x22 },
	{ OV2643_AECGM, 0x05 },
	{ OV2643_BDST2, 0xB8 },
	{ OV2643_HS, 0x01 },
	{ OV2643_HS2, 0x24 },
	{ OV2643_VS2, 0x0A },
	{ OV2643_HW, 0x64 },
	{ OV2643_HW2, 0x08 },
	{ OV2643_VH, 0x4B },
	{ OV2643_VH2, 0x06 },
	{ OV2643_HVOFF, 0x42 },
	{ OV2643_HTS, 0x07 },
	{ OV2643_HTS2, 0x9E },
	{ OV2643_VTS, 0x04 },
	{ OV2643_VTS2, 0xCE },
	{ OV2643_BLC, 0xAC },
	{ OV2643_OFFS, 0x04 },
	{ OV2643_TARGET, 0x04 },
	{ OV2643_TMC0, 0xE0 },
	{ OV2643_TMC1, 0x33 },
	{ OV2643_TMC2, 0xD0 },
	{ OV2643_TMC6, 0x08 },
	{ OV2643_LENCX, 0x04 },
	{ OV2643_LENCY, 0x02 },
	{ OV2643_REG40, 0xFB },
	{ OV2643_REG41, 0x17 },
	{ OV2643_REG44, 0x10 },
	{ OV2643_REG46, 0x04 },
	{ OV2643_REG47, 0x3F },
	{ OV2643_REG4C, 0x03 },
	{ OV2643_REG4D, 0x30 },
	{ OV2643_REG4E, 0x02 },
	{ OV2643_REG4F, 0x5C },
	{ OV2643_REG52, 0xFF },
	{ OV2643_REG53, 0x03 },
	{ OV2643_REG54, 0x30 },
	{ OV2643_REG55, 0x02 },
	{ OV2643_REG56, 0x5C },
	{ OV2643_REG59, 0xFF },
	{ OV2643_REG5A, 0x03 },
	{ OV2643_REG5B, 0x30 },
	{ OV2643_REG5C, 0x02 },
	{ OV2643_REG5D, 0x5C },
	{ OV2643_REG60, 0xFF },
	{ OV2643_REG61, 0x0C },
	{ OV2643_REG62, 0x06 },
	{ OV2643_REG63, 0x80 },
	{ OV2643_REG64, 0x80 },
	{ OV2643_REG65, 0x05 },
	{ OV2643_REG66, 0x0C },
	{ OV2643_REG67, 0x1C },
	{ OV2643_REG68, 0x2A },
	{ OV2643_REG69, 0x29 },
	{ OV2643_REG6A, 0x45 },
	{ OV2643_REG6B, 0x52 },
	{ OV2643_REG6C, 0x5D },
	{ OV2643_REG6D, 0x68 },
	{ OV2643_REG6E, 0x7F },
	{ OV2643_REG6F, 0x91 },
	{ OV2643_REG70, 0xA5 },
	{ OV2643_REG71, 0xC6 },
	{ OV2643_REG72, 0xDE },
	{ OV2643_REG73, 0xEF },
	{ OV2643_REG74, 0x16 },//blc target
	{ OV2643_REG75, 0x5C },
	{ OV2643_REG77, 0x92 },
	{ OV2643_REG78, 0x21 },
	{ OV2643_REG79, 0xE0 },
	{ OV2643_REG7A, 0x02 },
	{ OV2643_REG7B, 0xFF },
	{ OV2643_REG89, 0xF0 },
	{ OV2643_REG8A, 0xF0 },
	{ OV2643_REG8B, 0xF0 },
	{ OV2643_REG8C, 0x40 },
	{ OV2643_REG8D, 0x40 },
	{ OV2643_REG8E, 0x40 },
	{ OV2643_REG8F, 0x1C },
	{ OV2643_REG9B, 0x55 },
	{ OV2643_REG9C, 0x55 },
	{ OV2643_REG9D, 0x55 },
	{ OV2643_REG9E, 0x55 },
	{ OV2643_REG9F, 0x08 },
	{ OV2643_REGA0, 0x48 },
	{ OV2643_REGA1, 0x18 },
	{ OV2643_REGA2, 0x0E },
	{ OV2643_REGA3, 0x08 },//exposure [5:0]
	{ OV2643_REGA4, 0x48 },//exposure [7:0]
	{ OV2643_REGA5, 0x09 },//exposure [1:0]
	{ OV2643_REGA6, 0x16 },//manual exposure setting
	{ OV2643_REGA7, 0x08 },
	{ OV2643_REGA8, 0x48 },
	{ OV2643_REGA9, 0x04 },
	{ OV2643_REGAA, 0xA6 },
	{ OV2643_REGAB, 0x33 },
	{ OV2643_REGAC, 0x41 },
	{ OV2643_REGAD, 0x0F },
	{ OV2643_REGAE, 0x0B },
	{ OV2643_REGAF, 0x44 },
	{ OV2643_REGB0, 0x50 },
	{ OV2643_REGB1, 0x55 },
	{ OV2643_REGB2, 0x3A },
	{ OV2643_REGB3, 0x1C },
	{ OV2643_REGB4, 0x98 },
	{ OV2643_REGB5, 0x21 },
	{ OV2643_HUE_COS, 0x80 },
	{ OV2643_REGB9, 0x40 },
	{ OV2643_REGBC,0x20 },
	{ OV2643_REGBE, 0x01 },
	{ OV2643_REGBF, 0x10 },
	{ OV2643_SHIFT, 0x90 },
	{ OV2643_SA1TMC0, 0x30 },
	{ OV2643_SA1TMC7, 0x12 },
	{ OV2643_ANCOM1, 0x01 },
	{ OV2643_PWCOM0, 0x07 },
	{ OV2643_PWCOM2, 0x27 },
	{ OV2643_PWCOM4, 0xC4 },
	{ OV2643_GRPC, 0x10 },
	{ OV2643_IO, 0x1F},
	{ OV2643_IO2, 0xFF},
	{ OV2643_DVP_CTRL01, 0x08 },	
	{ OV2643_SYS, 0x1E},  /* Selects YUV mode */
	{ OV2643_CTRL1, 0x2C },	
    { 0xff, 0x00 }, /* END MARKER */
};

static struct regval_list ov2643_fmt_yuv422[] = {
	{ OV2643_SYS, SYS_FORMAT_SEL},  /* Selects YUV mode */
	{ OV2643_CTRL1, CTRL1_CCIR656 },	/* Enable CCIR656 */
	{ 0xff, 0xff },
};

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct ov2643_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
/*	int cmatrix[CMATRIX_LEN];*/
	int nlist;
	int bpp;   /* Bytes per pixel */
} ov2643_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= ov2643_fmt_yuv422,
/*		.cmatrix	= { 128, -128, 0, -34, -94, 128 },*/
		.nlist 		= ARRAY_SIZE(ov2643_fmt_yuv422),
		.bpp		= 2,                              
	},
/*	{
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.regs		= ov2643_fmt_rgb444,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 2,
	},
	{
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.regs		= ov2643_fmt_rgb565,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.regs 		= ov2643_fmt_raw,
		.cmatrix	= { 0, 0, 0, 0, 0, 0 },
		.bpp		= 1
	}, */
};
#define N_OV2643_FMTS ARRAY_SIZE(ov2643_formats)

static struct ov2643_win_size {
	int	width;
	int	height;
//	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
/* h/vref stuff */
} ov2643_win_sizes[] = {
	/* VGA */
	{
		.width		= HD720P_WIDTH,
		.height		= HD720P_HEIGHT,
/*		.com7_bit	= COM7_FMT_VGA,*/
		.hstart		= 158,		/* These values from */
		.hstop		=  14,		/* Omnivision */
		.vstart		=  10,
		.vstop		= 490,
		.regs 		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(ov2643_win_sizes))

static inline struct ov2643_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2643_state, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov2643_state, handler)->sd;
}

int (*vsync_i2c_write)(unsigned  char);

#define I2C_FLAG_READ   0x10
#if defined(CONFIG_NXP_CAPTURE_VSYNC_SEQUENCER)
int ov2643_i2c_write(unsigned char ex_flag)
{
    unsigned char buf[8];
    int val;
    int maxval;
//    static unsigned int ex_flag;	

	if ((ex_flag %2)==1){
		val = 0;
		buf[2] = 0x10;
		buf[6] = 0x10;
	}else{
		val = 100;
		buf[2] = 0x80;
		buf[6] = 0xF0;
	}
//	ex_flag++;
	printk("===================ex_flag %d\n", ex_flag);
//	if((ex_flag %3) == 0){
		maxval = 0x1000 - 0x0100;
		maxval = maxval * val / 100 + 0x0100;	
	//	cap_i2c_write(0x6E,POA030_DIGITALGAIN,1,&buf[0],1);//0x30 ov2643
		buf[0] = maxval & 0xFF;
		buf[1] = ((maxval>>8)&0xFF);
	//	buf[3] = (POA030_EXT_GLBG_L>>8)&0x03;
	//	buf[4] = (POA030_EXT_GLBG_H>>8)&0x03;
	//	buf[5] = (POA030_EXT_INTTIME_H>>8)&0x03;
	//	cap_i2c_write(0x6E,POA030_BANK,1,&buf[3],1);
//		cap_i2c_write(0x6E,POA030_EXT_GLBG_L,1,&buf[0],1);
	//	cap_i2c_write(0x6E,POA030_BANK,1,&buf[4],1);
//		cap_i2c_write(0x6E,POA030_EXT_GLBG_H,1,&buf[1],1);
	//	cap_i2c_write(0x6E,POA030_BANK,1,&buf[5],1);          
	//	cap_i2c_write(0x6E,POA030_EXT_INTTIME_H,1,&buf[2],1);
		cap_i2c_write(0x6E,OV2643_AEC2,1,&buf[2],1);
	//	cap_i2c_write(0x6E,POA030_EXT_INTTIME_L,1,&buf[2],1);
	//	cap_i2c_write(0x6E,POA030_EXPOSURE_H,1,&buf[6],1);
	//	cap_i2c_write(0x6E,POA030_EXPFRMH_H,1,&buf[6],1);
//	}	
	
}
#endif

static int ov2643_i2c_xfer(struct i2c_client *client, u8 reg, char *buf, int num, int tran_flag)
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
	    if (ret < 0) {
		printk("Error %d on register write\n", ret);
		return ret;
	}
	
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

static int ov2643_read_reg8(struct i2c_client *client, u8  reg, u8 *val)
{
	int ret;
	u8 rval;

	reg &= 0xFF;

	ret = ov2643_i2c_xfer(client, reg, (u8 *) &rval, 1, I2C_FLAG_READ);
	if (!ret) {
		*val = rval;
		return 0;
	}
	return ret;
}

static int ov2643_write_reg8(struct i2c_client *client, u8 reg, u8 val)
{
	u8 temp1;

	temp1 = reg & 0xFF;

	return ov2643_i2c_xfer(client, temp1, (u8 *) & val, 1, 0);
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov2643_write_reg8_array(struct i2c_client *c, struct regval_list *vals)
{
	while (vals->reg_num != 0xff || vals->value != 0xff)
	{
		int ret = ov2643_write_reg8(c, vals->reg_num, vals->value);

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
static int ov2643_set_hw(struct i2c_client *client, int hstart, int hstop, int vstart, int vstop)
{
	int ret;
	unsigned char v;
	
	/*
	 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
	 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
	 * a mystery "edge offset" value in the top two bits of href.
	 */
	ret =  ov2643_write_reg8(client, OV2643_HSTART, (hstart >> 3) & 0xff);
	ret += ov2643_write_reg8(client, OV2643_HSTOP, (hstop >> 3) & 0xff);
	ret += ov2643_read_reg8(client, OV2643_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += ov2643_write_reg8(client, OV2643_HREF, v);

	/*
	 * Vertical: similar arrangement, but only 10 bits.
	 */
	ret += ov2643_write_reg8(client, OV2643_VSTART, (vstart >> 2) & 0xff);
	ret += ov2643_write_reg8(client, OV2643_VSTOP, (vstop >> 2) & 0xff);
	ret += ov2643_read_reg8(client, OV2643_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += ov2643_write_reg8(client, OV2643_VREF, v);

	return ret;
}
#endif

#if 0
/*************************************************************************
* FUNCTION
*	config_OV2643_window
*
* DESCRIPTION
*	This function config the hardware window of OV2643 for getting specified
*  data of that window.
*
* PARAMETERS
*	start_x : start column of the interested window
*  start_y : start row of the interested window
*  width  : column widht of the itnerested window
*  height : row depth of the itnerested window
*
* RETURNS
*	the data that read from OV2643
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ov2643_config_window(struct i2c_client *client, u16 startx, u16 starty, u16 width, u16 height)
{
	u16 endx=(startx+width-1);
	u16 endy=(starty+height-1);
	u8  temp_reg1, temp_reg2;

	ov2643_read_reg8(client, 0x03, &temp_reg1);
	ov2643_read_reg8(client, 0x32, &temp_reg2);

	temp_reg1=temp_reg1&0xF0;
	temp_reg2=temp_reg2&0xC0;

	// Horizontal
	ov2643_write_reg8(client, 0x32,0x80|((endx&0x7)<<3)|(startx&0x7));	// b[5:3]:HREF end low 3bits. b[2:0]:HREF start low 3bits.
	ov2643_write_reg8(client, 0x17,(startx&0x7F8)>>3);			// HREF start high 8bits
	ov2643_write_reg8(client, 0x18,(endx&0x7F8)>>3);			// HREF end high 8bits
	// Vertical
	ov2643_write_reg8(client, 0x03,temp_reg1|((endy&0x3)<<2)|(starty&0x3));	// b[3:2]:VREF end low 2bits. b[1:0]:VREF start low 2bits.
	ov2643_write_reg8(client, 0x19,(starty&0x3FC)>>2);   			// VREF start high 8bits
	ov2643_write_reg8(client, 0x1A,(endy&0x3FC)>>2);		   	// VREF end high 8bits
}	/* config_OV2643_window */
#endif

static void ov2643_start(struct i2c_client *client)
{
	u8 val;
	pr_info("%s enter\n", __func__);
	ov2643_write_reg8(client, OV2643_SYS, SYS_SOFT_RESET);
	msleep(10);

	ov2643_write_reg8_array(client, ov2643_default_regs);

	ov2643_read_reg8(client, OV2643_CTRL1, &val);
	pr_info("%s exit\n", __func__);	
}

/*!
 * ov2643 get_i2c function
 *
 * @return  0 if success
 */
static int ov2643_get_i2c(struct i2c_client *client, int addr, int *val)
{
	int ret;
	pr_info("%s enter\n", __func__);
	ov2643_read_reg8(client, addr, (u8 *)val);
	pr_info("%s exit\n", __func__);	
	return ret;
}

/*!
 * ov2643 set_i2c function
 *
 * @return  0 if success
 */
static int ov2643_set_i2c(struct i2c_client *client, int addr, int val)
{
	int ret;
	pr_info("%s enter\n", __func__);	
    //printk("ov2643 set i2c!, addr = 0x%x val = 0x%x\n", addr, val);
    ret = ov2643_write_reg8(client, addr, val);
	pr_info("%s exit\n", __func__);  
	
	return ret;
}

/*
 * Stuff that knows about the sensor.
 */
static int ov2643_reset(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	pr_info("%s enter\n", __func__);	
	ov2643_write_reg8(client, OV2643_SYS, SYS_SOFT_RESET);
	msleep(1);
	pr_info("%s exit\n", __func__);	
	return 0;
}


static int ov2643_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client;
	int ret;
	pr_info("%s enter\n", __func__);
	
	client = v4l2_get_subdevdata(sd);
	ret = ov2643_write_reg8_array(client, ov2643_default_regs);
	
	pr_info("%s exit\n", __func__);
	return ret;
}

/*!
 * ov2643 detect
 *
 * @return 0(OK) or -NODEV
 */
static int ov2643_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val;
	unsigned short id;
	int ret;
	pr_info("%s enter\n", __func__);
	/* read id register and compare to correct value.. */
	ret = ov2643_read_reg8(client, OV2643_PIDH, &val);

	pr_info("OV2643_PIDH 0x%X ret %d\n", val, ret);

	if (ret)
		return ret;
	
	id = (val << 8) & 0xFF00;
	printk("PIDH 0x%X\n", id);
	
	ret = ov2643_read_reg8(client, OV2643_PIDL, &val);
	if (ret)
		return ret;
	
	id = id|(val & 0xFF);
	printk("PID 0x%X\n", id);
	
	

	if (id != 0x2643)
		return -ENODEV;
	pr_info("%s exit\n", __func__);
	return 0;
}

/*!
 * enum_fmt, try_fmt
 * We only support YUV422, 720P only.
 *
 * No other modes are tested. So return ERROR if non supported mode specified.
 *
 */
 
static int ov2643_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	struct ov2643_format_struct *ofmt;
	pr_info("%s enter\n", __func__);
	if (fmt->index >= N_OV2643_FMTS)
		return -EINVAL;

	ofmt = ov2643_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	pr_info("%s exit\n", __func__);
	return 0;
}


static int ov2643_try_fmt_internal(struct v4l2_subdev *sd,
		struct v4l2_format *fmt,
		struct ov2643_format_struct **ret_fmt,
		struct ov2643_win_size **ret_wsize)
{
	int index;
	struct ov2643_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	pr_info("%s enter\n", __func__);
	for (index = 0; index < N_OV2643_FMTS; index++)
		if (ov2643_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_OV2643_FMTS) {
		/* default to first format */
		index = 0;
		pix->pixelformat = ov2643_formats[0].pixelformat;
	}
	if (ret_fmt != NULL)
		*ret_fmt = ov2643_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	pix->field = V4L2_FIELD_NONE;
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = ov2643_win_sizes; wsize < ov2643_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (pix->width >= wsize->width && pix->height >= wsize->height)
			break;
	if (wsize >= ov2643_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	pix->width = wsize->width;
	pix->height = wsize->height;
	pix->bytesperline = pix->width*ov2643_formats[index].bpp;
	pix->sizeimage = pix->height*pix->bytesperline;
	pr_info("%s exit\n", __func__);
	return 0;
}

static int ov2643_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	return ov2643_try_fmt_internal(sd, fmt, NULL, NULL);
}

/*
 * Set a format.
 */
static int ov2643_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int ret;
	struct ov2643_format_struct *ovfmt;	
	struct ov2643_win_size *wsize;
	pr_info("%s enter\n", __func__);
	ret = ov2643_try_fmt_internal(sd, fmt, &ovfmt, &wsize);
	

	if (ret)
		return ret;
	pr_info("%s exit\n", __func__);	
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov2643_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov2643_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}


/*
 * Code for dealing with controls.
 */
static int ov2643_s_sat(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov2643_g_sat(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int ov2643_s_hue(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int ov2643_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int ov2643_s_brightness(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov2643_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int ov2643_s_contrast(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov2643_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int ov2643_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int ov2643_s_hflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int ov2643_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int ov2643_s_vflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int ov2643_queryctrl(struct v4l2_subdev *sd,
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

static int ov2643_subdev_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov2643_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return ov2643_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return ov2643_g_sat(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return ov2643_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return ov2643_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return ov2643_g_hflip(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int ov2643_subdev_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov2643_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return ov2643_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return ov2643_s_sat(sd, ctrl->value);
	case V4L2_CID_HUE:
		return ov2643_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return ov2643_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return ov2643_s_hflip(sd, ctrl->value);
	}
	return -EINVAL;
}

static int ov2643_s_ctrl(struct v4l2_ctrl *ctrl)
{
	// Do something...
	switch (ctrl->id) {
	default: ;
	}
	return -EINVAL;
}

static int ov2643_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client;
	int ret;
	pr_info("%s enter\n", __func__);
	
	client = v4l2_get_subdevdata(sd);
	ret = v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV7670, 0);
	
	pr_info("%s exit\n", __func__);
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2643_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val = 0;
	int ret;

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ret = ov2643_read_reg8(client, reg->reg & 0xff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov2643_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ov2643_write_reg8(client, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

#if 0
static int ov2643_change_resolution(struct i2c_client *client, int res)
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

static int ov2643_change_whitebalance(struct i2c_client *client, enum nx_vip_wb_t type)
{

	return 0;
}
#endif

static int ov2643_set_brightness(struct i2c_client *client, int val)
{
	s8 maxval;

    maxval = (s8)((val <= 50) ? (-254 * val / 100 - 1) : (255 * val / 100 - 128));
    ov2643_write_reg8(client,OV2643_REGBD, maxval&0xFF);

	return 0;
}

static int ov2643_set_set_power_save(struct i2c_client *client, int val)
{
	return 0;
}

static int ov2643_set_exposure(struct i2c_client *client, int val)
{
	u8	exph, expm, expl;
	u8	aec;

	ov2643_read_reg8(client,OV2643_AUTO1, &aec);

	aec |= AUTO1_AEC_EN;
    ov2643_write_reg8(client,OV2643_AUTO1, aec);

	ov2643_read_reg8(client,OV2643_AEC, &exph);
	ov2643_read_reg8(client,OV2643_AEC2,  &expl);


	exph = (exph & (~0xFF)) | ((val >> 8) & 0xFF);
	expl = (expl & (~0xFF)) | ((val >> 0) & 0xFF);

	//0x04 COM1 Bit[1:0]: Exposure time, the unit is tRow interval
	//AEC[15:0] = {0x07[5:0], 0x10[7:0], 0x04[1:0]}
	//               6bit       8bit       2bit
	//             0x00       0x40       0x00
	exph = ov2643_write_reg8(client,OV2643_AEC, exph);
	exph = ov2643_write_reg8(client,OV2643_AEC2,  expl);

	aec &= ~AUTO1_AEC_EN;
    ov2643_write_reg8(client,OV2643_AUTO1, aec);

	return 0;
}
#if 0
static int ov2643_set_fixed_frame(struct i2c_client *client, int val)
{
	ov2643_write_reg8(client,0x2a, 0x00);
	ov2643_write_reg8(client,0x2b, 0x00);
	ov2643_write_reg8(client,0x92, 0x00);
	ov2643_write_reg8(client,0x93, 0x00);
	ov2643_write_reg8(client,0x3b, 0x0a);

	if (val == 30)
	{
		ov2643_write_reg8(client,0x6b, 0x0a);	// PLL control Input clock bypass
		ov2643_write_reg8(client,0x11, 0x80);	// Internal clock pre-scalar(Input clock / 1)--datasheet ? x/1?
	}
	else if (val == 15)
	{
		ov2643_write_reg8(client,0x6b, 0x0a);	// PLL control Input clock bypass
		ov2643_write_reg8(client,0x11, 0x00);	// Internal clock pre-scalar(Input clock / 1)--datasheet ? x/1?
	}
	else if (val == 10)
	{
		ov2643_write_reg8(client,0x6b, 0x5a);	// PLL control [7:6]=0x01: Input clock x4
		ov2643_write_reg8(client,0x11, 0x05);	// Internal clock pre-scalar(Input clock / 6)
	}
	else if (val == 7)
	{
		ov2643_write_reg8(client,0x6b, 0x5a);	// PLL control [7:6]=0x01: Input clock x4
		ov2643_write_reg8(client,0x11, 0x07);	// Internal clock pre-scalar(Input clock / 7)
	}
	else
		return -1;

	return 0;
#endif

//gain = (0x03[7] + 1) x (0x03[6] + 1) x (0x00[7] + 1) x (0x00[6] + 1) x
//       (0x00[5] + 1) x (0x00[4] + 1) x (0x00[3:0] / 16 + 1)
static int ov2643_set_gain(struct i2c_client *client, int val)
{
	u8	reg_gl = 0;
	u8	reg_gh = 0;
	int	div = 0x40;
	int	gain;
	int i;

	if (val < 0 || val > 256)
		return -EINVAL;

	ov2643_read_reg8(client, OV2643_AUTO1, &reg_gh);
	ov2643_read_reg8(client, OV2643_AGC, &reg_gl);

	reg_gh &= AUTO1_AGC_EN;
	reg_gl &= 0x00;

	reg_gl = val & 0xFF ;

	ov2643_write_reg8(client, OV2643_AUTO1, reg_gh);
	ov2643_write_reg8(client, OV2643_AGC, reg_gl);

	return 0;
}

#if 0
static int ov2643_command(struct i2c_client *client, u32 cmd, void *arg)
{
	u8 val;

	switch (cmd) {
	case I2C_CAM_INIT:
		ov2643_start(client);
		info("OV2643 : external camera initialized\n");
		break;

	case I2C_CAM_RESOLUTION:
		return ov2643_change_resolution(client, (int) arg);

	case I2C_CAM_WB:
		ov2643_change_whitebalance(client, (enum nx_vip_wb_t) arg);
		break;

	case I2C_CAM_BRIGHTNESS:
		ov2643_set_brightness(client, (int) arg);
		break;

	case I2C_CAM_POWER_SAVE:
		ov2643_set_set_power_save(client, (int) arg);
		break;

	case I2C_CAM_EXPOSURE:
		ov2643_set_exposure(client, (int)arg);
		break;

/*	case I2C_CAM_FIXED_FRAME:
		ov2643_set_fixed_frame(client, (int)arg);
		break;
*/
	case I2C_CAM_EXPOSURE_AUTO:
		ov2643_read_reg8(client, OV2643_COM8, &val);// Manual Exposure

		val &= ~(COM8_AGC | COM8_AEC);
		if ((int)arg)
			val |= (COM8_AGC | COM8_AEC);

		ov2643_write_reg8(client, OV2643_COM8, val);// Auto Exposure
		break;

	case I2C_CAM_GAIN:
		ov2643_set_gain(client, (int)arg);// Gain
		break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}
#endif

static const struct v4l2_ctrl_ops ov2643_ctrl_ops = {
	.s_ctrl = ov2643_s_ctrl,
};

#define NUM_CTRLS				6

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

static const struct v4l2_ctrl_config ov2643_custom_ctrls[] = {
    {
        .ops    = &ov2643_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SCENE_MODE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "SceneMode",
        .min    = 0,
        .max    = SCENE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &ov2643_ctrl_ops,
        .id     = V4L2_CID_CAMERA_ANTI_SHAKE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "AntiShake",
        .min    = 0,
        .max    = ANTI_SHAKE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &ov2643_ctrl_ops,
        .id     = V4L2_CID_CAMERA_MODE_CHANGE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "ModeChange",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    },
};

static int ov2643_initialize_ctrls(struct ov2643_state *state)
{
	pr_info("%s enter\n", __func__);
    v4l2_ctrl_handler_init(&state->handler, NUM_CTRLS);

    /* standard */
    state->focus = v4l2_ctrl_new_std(&state->handler, &ov2643_ctrl_ops,
            V4L2_CID_FOCUS_AUTO, 0, 1, 1, 0);
    if (!state->focus) {
        pr_err("%s: failed to create focus ctrl\n", __func__);
        return -1;
    }
    state->wb = v4l2_ctrl_new_std(&state->handler, &ov2643_ctrl_ops,
            V4L2_CID_DO_WHITE_BALANCE, WB_AUTO, WB_MAX - 1, 1, WB_AUTO);
    if (!state->wb) {
        pr_err("%s: failed to create wb ctrl\n", __func__);
        return -1;
    }
    state->color_effect = v4l2_ctrl_new_std_menu(&state->handler, &ov2643_ctrl_ops,
            V4L2_CID_COLORFX, COLORFX_MAX - 1, 0, COLORFX_NONE);
    if (!state->color_effect) {
        pr_err("%s: failed to create color_effect ctrl\n", __func__);
        return -1;
    }
/*    state->exposure = v4l2_ctrl_new_std_menu(&state->handler, &ov2643_ctrl_ops,
            V4L2_CID_EXPOSURE, MAX_EXPOSURE, 0, 0);
    if (!state->exposure) {
        pr_err("%s: failed to create exposure ctrl\n", __func__);
        return -1;
    }
*/
    /* custom */
    state->scene_mode = v4l2_ctrl_new_custom(&state->handler, &ov2643_custom_ctrls[0], NULL);
    if (!state->scene_mode) {
        pr_err("%s: failed to create scene_mode ctrl\n", __func__);
        return -1;
    }
    state->anti_shake = v4l2_ctrl_new_custom(&state->handler, &ov2643_custom_ctrls[1], NULL);
    if (!state->anti_shake) {
        pr_err("%s: failed to create anti_shake ctrl\n", __func__);
        return -1;
    }
    state->mode_change = v4l2_ctrl_new_custom(&state->handler, &ov2643_custom_ctrls[2], NULL);
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
    
   	pr_info("%s exit\n", __func__);
    return 0;
}

static int ov2643_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int ov2643_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	int i, err = 0;
	struct v4l2_mbus_framefmt *_fmt = &fmt->format;
	
	printk("%s : %dx%d\n", __func__, _fmt->width, _fmt->height);
	
	return ov2643_s_fmt(sd, _fmt);
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov2643_core_ops = {
	.g_chip_ident = ov2643_g_chip_ident,
	.g_ctrl = ov2643_subdev_g_ctrl,
	.s_ctrl = ov2643_subdev_s_ctrl,
	.queryctrl = ov2643_queryctrl,
	.reset = ov2643_reset,
	.init = ov2643_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov2643_g_register,
	.s_register = ov2643_s_register,
#endif
};

static const struct v4l2_subdev_video_ops ov2643_video_ops = {
//	.enum_fmt = ov2643_enum_fmt,
//	.try_fmt = ov2643_try_fmt,
//	.s_fmt = ov2643_s_fmt,
	.s_parm = ov2643_s_parm,
	.g_parm = ov2643_g_parm,
};

static const struct v4l2_subdev_pad_ops ov2643_pad_ops = {
	.get_fmt = ov2643_get_fmt,
	.set_fmt = ov2643_set_fmt,
};

static const struct v4l2_subdev_ops ov2643_ops = {
	.core = &ov2643_core_ops,
	.video = &ov2643_video_ops,
	.pad = &ov2643_pad_ops,
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

static const struct media_entity_operations ov2643_media_ops = {
	.link_setup = _link_setup,
};

/* ----------------------------------------------------------------------- */

static int ov2643_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct ov2643_state *info;
	int ret;

	pr_info("%s enter\n", __func__);
	
	/*ret = nxp_soc_gpio_get_in_value(PAD_GPIO_A + 27);
	pr_info("PAD_GPIO_A + 27 read 0x%X\n", ret);	
	nxp_soc_gpio_set_out_value(PAD_GPIO_A + 27,0x00);*/
	ret = nxp_soc_gpio_get_in_value(PAD_GPIO_A + 27);	
	pr_info("PAD_GPIO_A + 27 read 0x%X\n", ret);
	info = kzalloc(sizeof(struct ov2643_state), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &ov2643_ops);

	if (strstr(id->name, "ov")){
	/* Make sure it's an ov2643 */
		ret = ov2643_detect(sd);
		pr_info("ov2643_detect ret %d\n", ret);
	}
	if (ret) {
		v4l_dbg(1, debug, client,
			"chip found @ 0x%x (%s) is not an ov2643 chip.\n",
			client->addr << 1, client->adapter->name);
		kfree(info);
		return ret;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	//info->fmt = &ov2643_formats[0]

	if (strstr(id->name, "_2"))
		info->id = 1;
	else
		info->id = 0;

	printk("OV2643 name = %s, id=%d\n", id->name, info->id);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.ops = &ov2643_media_ops;
	if(media_entity_init(&sd->entity, 1, &info->pad, 0)){
		printk("media_entity_init error\n");
		dev_err(&client->dev, "%s:failed to media_entity_init()\n", __func__);
		kfree(info);
		return -ENOENT;
	}
	
	printk("subdev %p\n", sd);
	/*info->client = client;
	info->i2c_read = ov2643_get_i2c;
	info->i2c_write = ov2643_set_i2c;

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
	vsync_i2c_write = NULL;
	ret = ov2643_initialize_ctrls(info);
	if (ret < 0) {
		pr_err("%s: failed to initialize controls\n", __func__);
		kfree(info);
		return ret;
	}
	ov2643_start(client);
	info->inited = 1;
//	vsync_i2c_write = ov2643_i2c_write;
	pr_info("%s exit\n", __func__);
	return 0;
}


static int ov2643_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	pr_info("%s enter\n", __func__);	
	sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	pr_info("%s exit\n", __func__);	
	return 0;
}

#if defined(CONFIG_CAMERA1_OV2643) || defined(CONFIG_SOC_CAMERA_OV2643)
static const struct i2c_device_id ov2643_id[] = {
	{ "ov2643", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2643_id);

static struct i2c_driver ov2643_i2c_driver = {
	.driver = {
		.name = "ov2643",
		.owner = THIS_MODULE,
	},
	.id_table     = ov2643_id,
	.probe        = ov2643_probe,
    .remove       = __devexit_p(ov2643_remove),
	//.command      = ov2643_command,
};
#endif

#if defined(CONFIG_SOC_CAMERA2_OV2643)
static const struct i2c_device_id ov2643_id2[] = {
	{ "ov2643_2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2643_id2);

static struct i2c_driver ov2643_i2c_driver2 = {
	.driver = {
		.name = "ov2643_2",
		.owner = THIS_MODULE,
	},
	.id_table     = ov2643_id2,
	.probe        = ov2643_probe,
    .remove       = __devexit_p(ov2643_remove),
/*	.command      = ov2643_command,*/
};
#endif

static __init int _ov2643_init(void)
{
	int res;
	pr_info("%s enter\n", __func__);
#if defined(CONFIG_CAMERA1_OV2643) || defined(CONFIG_SOC_CAMERA_OV2643)
	res = i2c_add_driver(&ov2643_i2c_driver);
#endif
#if defined(CONFIG_SOC_CAMERA2_OV2643)
	res = i2c_add_driver(&ov2643_i2c_driver2);
#endif
	pr_info("%s exit\n", __func__);
	return res;
}

static __init void _ov2643_exit(void)
{
	pr_info("%s enter\n", __func__);
#if defined(CONFIG_CAMERA1_OV2643) || defined(CONFIG_SOC_CAMERA_OV2643)
	i2c_del_driver(&ov2643_i2c_driver);
#endif
#if defined(CONFIG_SOC_CAMERA2_OV2643)
	i2c_del_driver(&ov2643_i2c_driver2);
#endif
	pr_info("%s exit\n", __func__);
}

module_init(_ov2643_init)
module_exit(_ov2643_exit)

MODULE_AUTHOR("JuYoung Ryu<jyryu@stcube.com>");
MODULE_DESCRIPTION("OV2643 Camera Driver");
MODULE_LICENSE("GPL");

