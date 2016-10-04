/*! linux/drivers/media/video/po2210.c
 *
 * PixelPlus PO2210N CMOS Image Sensor driver
 * Copyright(c) 2015 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#define DEBUG	1
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/io.h>

#include <mach/devices.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include <mach/soc.h>

#include "nx_vip.h"
//#include "nxp-capture.h"
//#include "nxp-vin-clipper.h"
#include "po2210.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

#define FHD_WIDTH	1920
#define FHD_HEIGHT	1080
#define HD_WIDTH	1280
#define HD_HEIGHT	720                        
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
                             
struct regval_list {
	unsigned short int reg_num;
	unsigned short int value;
};

struct po2210n_state {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler handler;
	/* standard */
	struct v4l2_ctrl *exposure_auto;
	struct v4l2_ctrl *wb;
	struct v4l2_ctrl *color_effect;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	/* custom */
	struct v4l2_ctrl *scene_mode;
	struct v4l2_ctrl *anti_shake;
	struct v4l2_ctrl *mode_change;
	struct v4l2_ctrl *camera_select;
	struct v4l2_ctrl *laser_ctrl;

	bool inited;
	int priv_data;
	int width;
	int height;
	int mode; // PREVIEW or CAPTURE
	int id;
	struct mutex power_lock;
	int power_count;

	/* for zoom */
	struct v4l2_rect crop;

	struct regulator *cam_core_18V;
	struct regulator *cam_io_28V;

	/*
	 * req_fmt is the requested format from the application.
	 * set_fmt is the output format of the camera. Finally FIMC
	 * converts the camera output(set_fmt) to the requested format
	 * with hardware scaler.
	 */
	struct v4l2_pix_format req_fmt;
	struct v4l2_pix_format cur_fmt;

#ifdef USE_INITIAL_WORKER_THREAD
	struct workqueue_struct *init_wq;
	struct work_struct init_work;
#endif
};

enum {
	_INT_YUV422_UYVY = 0,
	_INT_YUV422_VYUY,
	_INT_YUV422_YUYV,
	_INT_YUV422_YVYU,
	_INT_RGB444,
	_INT_RGB565,
	_INT_RAW_BAYER
};

struct regval_list po2210n_def_regs[] = {
//	############## Base Settings ################

//W0300
	{0x0029,0x66}, // DEF POWERMODE
	{0x003C,0xC4}, // Internal DVDD OFF, bgrcon_15 = 000b (1.35V)
	{0x0004,0x01}, // chip_mode

	{0x006,0x08}, // framewidth_h
	{0x007,0x97}, // framewidth_l

	{0x0024,0x0A}, // clkdiv1
	{0x0025,0x22}, // clkdiv2

	{0x002F,0x01}, // pad_control7        (01)
	{0x002A,0x43}, // pad_control2        (00)
	{0x002B,0x9C}, // pad_control3        (00)
	{0x002E,0x03}, // pad_control6        (00)
	{0x0030,0xFF}, // pad_control8        (00)
	{0x0031,0xFF}, // pad_control9        (00)

	{0x0087,0x88}, // led_control

	{0x0040,0x0B}, // pll_m_cnt
	{0x0041,0x04}, // pll_r_cnt  //02

	{0x003F,0x50}, // pll_control1
	// delay some time... $0010  
	{0x003F,0x40}, // pll_control1             

//############# Start Settings ################
//W0301
	{0x0116,0x04}, // led_dsel
	{0x01B7,0x40}, // adcoffset

//W0302
	{0x022B,0x14}, // dpc_offset

//#################################### blacksun

//W0301
	{0x011E,0x0E}, // bsmode off
	{0x0126,0x04}, // blacksunth_h

//#################################### recommended by design1 150813
//W0301   # Limiter reference fitting due to gain
	{0x01F6,0x0E}, // bs_ofst0
	{0x01F7,0x14}, // bs_ofst1
	{0x01F8,0x24}, // bs_ofst2
	{0x01F9,0x26}, // bs_ofst3
	{0x01FA,0x26}, // bs_ofst4
	{0x01FB,0x26}, // bs_ofst5
	{0x01FC,0x26}, // bs_ofst6
	{0x01FD,0x26}, // bs_ofst_max
	{0x01FE,0x00}, // bs_ofst_min

//#################################### cds v1.1
//W0300
	{0x0035,0x08}, // pixelbias (01)
	{0x0036,0x04}, // compbias (02)

//W0301
	{0x0119,0xC4}, // ramppclk_sel
	{0x011C,0x11}, // ramp speed X1, adc speed X1

//W0301
	{0x0157,0x08},
	{0x0158,0x7F},
	{0x0159,0x08},
	{0x015A,0x96},
	{0x0153,0x00},
	{0x0154,0x02},
	{0x0155,0x08},
	{0x0156,0x7F},
	{0x0167,0x00},
	{0x0168,0x54},
	{0x0169,0x00},
	{0x016A,0x5E},
	{0x015B,0x00},
	{0x015C,0x00},
	{0x015D,0x08},
	{0x015E,0x7F},
	{0x015F,0x00},
	{0x0160,0x00},
	{0x0161,0x00},
	{0x0162,0x50},
	{0x0199,0x00},
	{0x019A,0x54},
	{0x019B,0x08},
	{0x019C,0x7F},
	{0x016F,0x00},
	{0x0170,0x00},
	{0x0171,0x05},
	{0x0172,0x7A},
	{0x0173,0x00},
	{0x0174,0x00},
	{0x0175,0x05},
	{0x0176,0x78},
	{0x0177,0x08},
	{0x0178,0x95},
	{0x0179,0x08},
	{0x017A,0x96},
	//#WB740
	{0x018F,0x00},
	{0x0190,0x52},
	{0x018B,0x00},
	{0x018C,0x64},
	{0x018D,0x08},
	{0x018E,0x6A},
	{0x0187,0x08},
	{0x0188,0x48},
	{0x0189,0x08},
	{0x018A,0x7C},
	{0x0195,0x08},
	{0x0196,0x80},
	{0x0197,0x08},
	{0x0198,0x8F},
	{0x0191,0x08},
	{0x0192,0x80},
	{0x0193,0x08},
	{0x0194,0x97},
	{0x017F,0x08},
	{0x0180,0x80},
	{0x0181,0x08},
	{0x0182,0x8F},
	{0x0183,0x08},
	{0x0184,0x80},
	{0x0185,0x08},
	{0x0186,0x8F},
	{0x01B9,0x08},
	{0x01BA,0x80},
	{0x01BB,0x08},
	{0x01BC,0x8F},
	{0x01A1,0x0B},
	{0x01A2,0x84},
	{0x0136,0x00},
	{0x0137,0xBE},
	{0x0138,0x08},
	{0x0139,0x4E},
	{0x017B,0x00},
	{0x017C,0x00},
	{0x017D,0x05},
	{0x017E,0x7C},
	{0x013E,0x00},
	{0x013F,0xBE},
	{0x0140,0x08},
	{0x0141,0x4E},

//#################################### ablc
//W0300
	{0x0038,0x90}, // analog_control_02
	{0x003D,0x2F}, // analog_control_07

//W0301 # bank B
	{0x011F,0x51}, // bayer_control_10
	{0x0120,0xA8}, // Median value for filter and Average value for selection
	{0x01A3,0xE0}, // blc_top_th
	{0x01A4,0x70}, // blc_bot_th
	{0x01A5,0x02}, // ablc_update

//W0304
	{0x0406,0xA1}, // auto_control_3[0] : ablc fitting enable

//# fitting x reference
//W0304
	{0x04C7,0x00}, // overOBP_xref0 
	{0x04C8,0x08}, // overOBP_xref1 
	{0x04C9,0x1E}, // overOBP_xref2 
	{0x04CA,0x32}, // overOBP_xref3 
	{0x04CB,0x58}, // overOBP_xref4 
   
//# fitting y reference
//W0303
	{0x03DC,0x00}, // overOBP_yref0 
	{0x03DD,0x16}, // overOBP_yref1 
	{0x03DE,0x1B}, // overOBP_yref2 
	{0x03E0,0x25}, // overOBP_yref3 
	{0x03E1,0x30}, // overOBP_yref4 		

//#################################### intp
//W0303
	{0x0330,0x00},	// intp_w0       (10)
	{0x0331,0xFF},	// intp_x0       (00)
	{0x0332,0x40},	// intp_slope    (40)

//#################################### blf
//W0302
	{0x0205,0xFB},	// [4] edge_blf_mode : 0=new, 1=old	FB

//W0303
	{0x0333,0x00},	// blf_w0_ref0	00
	{0x0334,0x40},	// blf_w0_ref1	00
	{0x0335,0x40},	// blf_w0_ref2	00

	{0x0337,0x20},	// blf_x0	20
	{0x0338,0x40},	// blf_slope	40

	{0x0339,0x7F},	// blf_c0	80	7F
	{0x033A,0x78},	// blf_c1	60	78
	{0x033B,0x63},	// blf_c2	40	63
	{0x033C,0x3F},	// blf_c3	20	2F
	{0x033D,0x2B},	// blf_c4	10	0B
	{0x033E,0x18},	// blf_c5	08	00

//#################################### sc
//W0309
	{0x0904,0x03}, //	acce_ctrl_0 [1]:acce enable, [0]:histogram enable (00)
	{0x096D,0x04}, // ac_ctrl_0 [2]:AE relate mode

	{0x0949,0x30},	// ce_th      (20)
	{0x094A,0x10},	// ce_x0      (40)
	{0x094B,0x40},	// ce_slope   (40)

	{0x09AD,0x08}, //08 # lpf_w1 (08)
	{0x09AE,0x10}, //10 # lpf_w2 (18)
	{0x09AF,0x20}, //20 # lpf_w3 (40)
	{0x09B0,0x10}, //10 # lpf_w4 (18)
	{0x09B1,0x08}, //08 # lpf_w5 (08)

	{0x09B2,0x04}, // ac_offset
	{0x09B3,0x60}, // max_ac_gain0
	{0x09B4,0x60}, // max_ac_gain1
	{0x09B5,0x40}, // max_ac_gain2
      
	{0x09B7,0x40}, // min_ac_gain
	{0x09B8,0x03}, // ac_speed
	{0x09B9,0x02}, // ac_lock
	{0x09BB,0x04}, // ac_frame

	{0x098E,0x00}, // ac_cv_w0 
	{0x098F,0x04}, // ac_cv_w1 
	{0x0990,0x06}, // ac_cv_w2 
	{0x0991,0x06}, // ac_cv_w3 
	{0x0992,0x04}, // ac_cv_w4 
	{0x0993,0x03}, // ac_cv_w5 
	{0x0994,0x01}, // ac_cv_w6 
	{0x0995,0x00}, // ac_cv_w7 

//#################################### ae

//#Flicker canceling mode - manual 60hz
//W0300
	{0x004A,0x08},

//W0300
	{0x0054,0x01},
	{0x0055,0x17},
	{0x0056,0x80},

//#Y target control
//W0304
	{0x043B,0x50},
	{0x043C,0x58},
	{0x043D,0x48},
	{0x043E,0x50},
	{0x043F,0x48},
	{0x0440,0x48},
	{0x0441,0x00},
	{0x0442,0x00},
	{0x0443,0x14},
	{0x0444,0x00},
	{0x0445,0x02},
	{0x0446,0xE8},
	{0x0447,0x00},
	{0x0448,0x45},
	{0x0449,0xE0},
	{0x044A,0x00},
	{0x044B,0x8B},
	{0x044C,0xC0},

//#Auto exposure control
//W0304
	{0x0412,0x04},
	{0x0413,0x5E},
	{0x0414,0x04},
	{0x0415,0x5E},
	{0x0416,0x04},
	{0x0417,0x5E},
	{0x041B,0x00},
	{0x041C,0x8B},
	{0x041D,0xC0},
	{0x041E,0x00},
	{0x041F,0x8B},
	{0x0420,0xC0},

//#Reference Gain Control
//W0304
	{0x04BA,0x10},

//#saturation level th
//W0304
	{0x042C,0x66},

//#Auto exposure option
//W0304
	{0x0455,0x04},
	{0x0456,0x04},
	{0x0457,0x0C},

//#Center window weight
//W0304
	{0x0434,0x20},

//#################################### gm

//#gamma curve fitting
//W0302
	{0x023D,0x00},
	{0x023E,0x27},
	{0x023F,0x36},
	{0x0240,0x40},
	{0x0241,0x49},
	{0x0242,0x58},
	{0x0243,0x64},
	{0x0244,0x78},
	{0x0245,0x89},
	{0x0246,0xA4},
	{0x0247,0xBB},
	{0x0248,0xCF},
	{0x0249,0xE0},
	{0x024A,0xF1},
	{0x024B,0xFF},

//#gamma curve fitting
//W0302
	{0x025B,0x00},
	{0x025C,0x01},
	{0x025D,0x07},
	{0x025E,0x10},
	{0x025F,0x1C},
	{0x0260,0x35},
	{0x0261,0x4C},
	{0x0262,0x70},
	{0x0263,0x89},
	{0x0264,0xAB},
	{0x0265,0xC3},
	{0x0266,0xD5},
	{0x0267,0xE5},
	{0x0268,0xF3},
	{0x0269,0xFF},                                          

//#################################### awb

//# Set AWB Sampling Boundary
//W0304
	{0x045C,0x00},
	{0x045D,0x40},
	{0x045E,0xA0},
	{0x045F,0x01},
	{0x0460,0x02},
	{0x0461,0x50},
	{0x0462,0x02},
	{0x0463,0x00},
	{0x0464,0x04},
	{0x0465,0x6E},
	{0x0466,0x45},

//W0304
	{0x0467,0x27},
	{0x0468,0x4F},
	{0x0469,0x64},
	{0x046A,0xC4},
	{0x046B,0x0A},
	{0x046C,0x46},
	{0x046D,0x32},
	{0x046E,0x78},
	{0x046F,0x37},
	{0x0470,0xAF},
	{0x0471,0x32},
	{0x0472,0x23},
	
	//#AWB option
//W0304
	{0x047E,0x08},
	{0x047F,0x04},

//lens / cs axis
//W0303
	{0x030A,0x3E},
	{0x030B,0x5D},
	{0x030C,0x6C},

//################################### color

//#Color correction
//W0302
	{0x0233,0x32},
	{0x0234,0x87},
	{0x0235,0x8B},
	{0x0236,0x8E},
	{0x0237,0x3D},
	{0x0238,0x8F},
	{0x0239,0x92},
	{0x023A,0x85},
	{0x023B,0x37},
	
//#Color saturation weight
//W0303
	{0x030D,0x20},

//W0302
	{0x020B,0x86}, // user_cs_mode [3]

//W0303
	{0x038A,0xF0},	// dc_y2

//################################ edge

//#Edge control
//W0302
	{0x0205,0xFB},
	{0x0209,0x00},
	{0x020B,0x82},

//#sharpness control
//W0303
	{0x0370,0x40},
	{0x0371,0x40},

//#Edge gamma curve
//W0303
	{0x0348,0x50},
	{0x0349,0x40},
	{0x034A,0x30},
	{0x034B,0x20},
	{0x034C,0x18},
	{0x034D,0x10},
	{0x034E,0x10},

//#edge_gain_lf
//W0303
	{0x0353,0x18},
	{0x0354,0x10},
	{0x0355,0x08},

//#edge_gain_ghf
//W0303
	{0x0357,0x10},
	{0x0358,0x10},
	{0x0359,0x10},

//#edge_gain_ehf
//W0303
	{0x035B,0x10},
	{0x035C,0x10},
	{0x035D,0x10},

//##################################### vec

//W0303
	{0x0393,0x0C}, // vec_en (00)

// Set sample range for Vector Control
//W0303
	{0x0394,0x14},
	{0x0395,0x1B},
	{0x0398,0x1C},
	{0x0399,0x26},
	{0x039C,0x35},
	{0x039D,0x42},
	{0x03A0,0x44},
	{0x03A1,0x62},
	{0x03A4,0x64},
	{0x03A5,0x72},
	{0x03A8,0x76},
	{0x03A9,0x80},

// Set hue & saturation (a)
//W0303
	{0x03AC,0x90},
	{0x03AF,0x04},
	{0x03B2,0x07},
	{0x03B5,0x04},
	{0x03B8,0x82},
	{0x03BB,0x04},
	{0x03BE,0x92},
	{0x03C1,0x04},
	{0x03C4,0x85},
	{0x03C7,0x04},
	{0x03CA,0x86},
	{0x03CD,0x04},

// Set hue & saturation (b)
//W0303
	{0x03AD,0x88},
	{0x03B0,0x04},
	{0x03B3,0x0B},
	{0x03B6,0x00},
	{0x03B9,0x05},
	{0x03BC,0x04},
	{0x03BF,0x8B},
	{0x03C2,0x04},
	{0x03C5,0x85},
	{0x03C8,0x04},
	{0x03CB,0x84},
	{0x03CE,0x04},

//# Set hue & saturation (c)
//W0303
	{0x03AE,0x88},
	{0x03B1,0x04},
	{0x03B4,0x82},
	{0x03B7,0x04},
	{0x03BA,0x01},
	{0x03BD,0x04},
	{0x03C0,0x00},
	{0x03C3,0x06},
	{0x03C6,0x07},
	{0x03C9,0x04},
	{0x03CC,0x00},
	{0x03CF,0x04},

//#################################### dark

//#Darkness X reference
//W0304
	{0x0498,0x00},
	{0x0499,0x04},
	{0x049A,0x00},
	{0x049B,0x10},
	{0x049C,0x00},
	{0x049D,0x20},

//#dark_y_weight
//W0303
	{0x037A,0x40},
	{0x037B,0x40},
	{0x037C,0x80},

//#dark_ccr
//W0303
	{0x037E,0x04},
	{0x037F,0x04},
	{0x0380,0x04},

//#dark_dc
//W0303
	{0x0382,0x00},
	{0x0383,0x0A},
	{0x0384,0x24},

//#ycont_slope2
//W0303
	{0x03E8,0x40},
	{0x03E9,0x40},
	{0x03EA,0x40},

//#dark_edge_gm
//W0303
	{0x034F,0x00},
	{0x0350,0x00},
	{0x0351,0x00},

//#dark_ec_pth
//W0303
	{0x0360,0x04},
	{0x0361,0x04},
	{0x0362,0x04},

//#dark_ec_mth
//W0303
	{0x0364,0x04},
	{0x0365,0x04},
	{0x0366,0x04},

//#dark_dpc_p
//W0303
	{0x031A,0x00},
	{0x031B,0x04},
	{0x031C,0x7F},

//#dark_dpc_n
//W0303
	{0x031E,0x00},
	{0x031F,0x08},
	{0x0320,0x18},

//#ybrightness
//W0302
	{0x0295,0x00},
	{0x0296,0x00},
	{0x0297,0x00},

//#blf_darkness
//W0303
	{0x033F,0x38},
	{0x0340,0x10},
	{0x0341,0x00},

//#dark_ec_pmax
//W0303
	{0x0368,0x40},
	{0x0369,0x10},
	{0x036A,0x08},

//#dark_ec_mmax
//W0303
	{0x036C,0x40},
	{0x036D,0x20},
	{0x036E,0x08},

//#hf_dir_max
//W0303
	{0x0324,0x1C},
	{0x0325,0x1C},
	{0x0326,0x7F},

//#intp_dir_th
//W0303
	{0x0328,0x08},
	{0x0329,0x08},
	{0x032A,0x7F},

	/* Now Set CCIR 656 */
	{PO2210N_SYNC_BLANKSAV_F0, 0xAB}, // 0xAB
	{PO2210N_SYNC_BLANKEAV_F0, 0xB6}, // 0xB6
	{PO2210N_SYNC_ACTIVESAV_F0, 0x80},
	{PO2210N_SYNC_ACTIVEEAV_F0, 0x9D},
};
#define PO2210N_DEF_REG_SIZE	ARRAY_SIZE(po2210n_def_regs)

struct regval_list po2210n_fmt_yuv422_uyvy[] = {
	{PO2210N_FORMAT, PO2210N_FMT_CBYCRY},
	{PO2210N_YCONTRAST, 0x40},
	{PO2210N_YBRIGHTNESS, 0x01},
	//{PO2210N_Y_MAX, 0xFE},
	//{PO2210N_SYNC_CONTROL_0, 0x00},
};

struct regval_list po2210n_fmt_yuv422_vyuy[] = {
	{PO2210N_FORMAT, PO2210N_FMT_CRYCBY},
	{PO2210N_YCONTRAST, 0x40},
	{PO2210N_YBRIGHTNESS, 0x01},
	//{PO2210N_Y_MAX, 0xFE},
	//{PO2210N_SYNC_CONTROL_0, 0x00},
};

struct regval_list po2210n_fmt_yuv422_yuyv[] = {
	{PO2210N_FORMAT, PO2210N_FMT_YCBYCR},
	{PO2210N_YCONTRAST, 0x40},
	{PO2210N_YBRIGHTNESS, 0x01},
	//{PO2210N_Y_MAX, 0xFE},
	//{PO2210N_SYNC_CONTROL_0, 0x00},
};

struct regval_list po2210n_fmt_yuv422_yvyu[] = {
	{PO2210N_FORMAT, PO2210N_FMT_YCRYCB},
	{PO2210N_YCONTRAST, 0x40},
	{PO2210N_YBRIGHTNESS, 0x01},
	//{PO2210N_Y_MAX, 0xFE},
	//{PO2210N_SYNC_CONTROL_0, 0x00},
};

struct regval_list po2210n_fmt_rgb444[] = {
	{PO2210N_FORMAT,PO2210N_FMT_XRGB444},
	{PO2210N_YCONTRAST, 0x40},
	{PO2210N_YBRIGHTNESS, 0x00},
	//{PO2210N_Y_MAX, 0xFE},
	//{PO2210N_SYNC_CONTROL_0, 0x00},
};

struct regval_list po2210n_fmt_rgb565[] = {
	{PO2210N_FORMAT, PO2210N_FMT_RGB565},
	{PO2210N_YCONTRAST, 0x40},
	{PO2210N_YBRIGHTNESS, 0x00},
	//{PO2210N_Y_MAX, 0xFE},
	//{PO2210N_SYNC_CONTROL_0, 0x00},
};


struct regval_list po2210n_fmt_raw[] = {
	{PO2210N_FORMAT, PO2210N_FMT_DPC_BAYER},
	{PO2210N_YCONTRAST, 0x40},
	{PO2210N_YBRIGHTNESS, 0x00},
	//{PO2210N_Y_MAX, 0xFE},
	//{PO2210N_SYNC_CONTROL_0, 0x01},
};

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct po2210n_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int nlist;
	int bpp;   /* Bytes per pixel */
} po2210n_formats[] = {
	[_INT_YUV422_UYVY] = {
		.desc		= "UYVY 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		.regs 		= po2210n_fmt_yuv422_uyvy,
		.nlist		= ARRAY_SIZE(po2210n_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	[_INT_YUV422_VYUY] = {
		.desc		= "VYUY 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_VYUY,
		.regs 		= po2210n_fmt_yuv422_vyuy,
		.nlist		= ARRAY_SIZE(po2210n_fmt_yuv422_vyuy),
		.bpp		= 2,
	},
	[_INT_YUV422_YUYV] = {
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= po2210n_fmt_yuv422_yuyv,
		.nlist		= ARRAY_SIZE(po2210n_fmt_yuv422_yuyv),
		.bpp		= 2,
	},
	[_INT_YUV422_YVYU] = {
		.desc		= "YVYU 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YVYU,
		.regs 		= po2210n_fmt_yuv422_yvyu,
		.nlist		= ARRAY_SIZE(po2210n_fmt_yuv422_yvyu),
		.bpp		= 2,
	},
	[_INT_RGB444] = {
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.regs		= po2210n_fmt_rgb444,
		.nlist		= ARRAY_SIZE(po2210n_fmt_rgb444),
		.bpp		= 2,
	},
	[_INT_RGB565] = {
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.regs		= po2210n_fmt_rgb565,
		.nlist		= ARRAY_SIZE(po2210n_fmt_rgb565),
		.bpp		= 2,
	},
	[_INT_RAW_BAYER] = {
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.regs 		= po2210n_fmt_raw,
		.nlist		= ARRAY_SIZE(po2210n_fmt_raw),		
		.bpp		= 1
	},
};
#define N_PO2210_FMTS ARRAY_SIZE(po2210n_formats)
/*
	FRAMEWIDTH		0x0A4F -> 2639
	FRAMEHEIGHT     0x0464 -> 1124
	WINDOWX1        0x0005 -> 5
	WINDOWY1        0x0005 -> 5
	WINDOWX2        0x0784 -> 1924
	WOMDOWY2        0x043C -> 1084
	VSYNCSTARTROW	0x001A -> 26
	VSYNCSTOPROW	0x0451 -> 1105   1105-26 = 1079
	
	FRAME_RATE = FREQ(PCLK) / (FRAMEHEIGHT+1) * (FRAMEWIDTH +1)
	             74250000 / (2639+1) * (1124+1) = 74250000/2970000 = 25Hz 
	             
	PLL OUT = FREQ(27MHz) * PLL_M_CNT / PLL_R_CNT
		FREQ / PLL_R_CNT > 3.8MHz
		FREQ * PLL_M_CNT / PLL_R_CNT < 300MHz
		27000000 * 11 / 2 = 148.5MHz (= 74.25MHz * 2)
		27000000 * 11 / 4 = 74.25MHz
		
	AE_FWX1		0x0005 -> 5
	AE_FWX2		0x0784 -> 1924
	AE_FWY1		0x0005 -> 5
	AE_FWY2		0x043C -> 1084
	AE_CWX1		0x0285 -> 645
	AE_CWX2		0x0504 -> 1284
	AE_CWY1		0x016D -> 365
	AE_CWY2		0x02D4 -> 724
	AE_XAXIS	0x03C5 -> 965
	AE_YAXIS	0x0221 -> 545
	
	AWB_WX1		0x0005 -> 5
	AWB_WX2		0x0784 -> 1924
	AWB_WY1		0x0005 -> 5
	AWB_WY2		0x043C -> 1084
*/
static struct regval_list fhd_reg_vals[] = {
	{PO2210N_WINDOWX1_H, 0x00},
	{PO2210N_WINDOWX1_L, 0x05},
	{PO2210N_WINDOWY1_H, 0x00},
	{PO2210N_WINDOWY1_L, 0x05},
	{PO2210N_WINDOWX2_H, 0x07},
	{PO2210N_WINDOWX2_L, 0x84},
	{PO2210N_WINDOWY2_H, 0x04},
	{PO2210N_WINDOWY2_L, 0x3C},
//	{PO2210N_SCALE_X,	 0x20},
//	{PO2210N_SCALE_Y,	 0x20},
	{PO2210N_AE_FWX1_H,	 0x00},
	{PO2210N_AE_FWX1_L,	 0x05},
	{PO2210N_AE_FWX2_H,	 0x07},
	{PO2210N_AE_FWX2_L,	 0x84},
	{PO2210N_AE_FWY1_H,	 0x00},
	{PO2210N_AE_FWY1_L,	 0x05},
	{PO2210N_AE_FWY2_H,	 0x04},
	{PO2210N_AE_FWY2_L,	 0x3C},
	{PO2210N_AE_CWX1_H,	 0x02},
	{PO2210N_AE_CWX1_L,	 0x85},
	{PO2210N_AE_CWX2_H,	 0x05},
	{PO2210N_AE_CWX2_L,	 0x04},
	{PO2210N_AE_CWY1_H,	 0x01},
	{PO2210N_AE_CWY1_L,	 0x6D},
	{PO2210N_AE_CWY2_H,	 0x02},
	{PO2210N_AE_CWY2_L,	 0xD4},
	{PO2210N_AE_XAXIS_H,	 0x03},
	{PO2210N_AE_XAXIS_L,	 0xC5},
	{PO2210N_AE_YAXIS_H,	 0x02},
	{PO2210N_AE_YAXIS_L,	 0x21},
	{PO2210N_AWB_WX1_H,	 0x00},
	{PO2210N_AWB_WX1_L,	 0x05},
	{PO2210N_AWB_WX2_H,	 0x07},
	{PO2210N_AWB_WX2_L,	 0x84},
	{PO2210N_AWB_WY1_H,	 0x00},
	{PO2210N_AWB_WY1_L,	 0x05},
	{PO2210N_AWB_WY2_H,	 0x04},
	{PO2210N_AWB_WY2_L,	 0x3C},
};

static struct regval_list hd720p_reg_vals[] = {
	{PO2210N_WINDOWX1_H, 0x00},
	{PO2210N_WINDOWX1_L, 0x05},
	{PO2210N_WINDOWY1_H, 0x00},
	{PO2210N_WINDOWY1_L, 0x05},
	{PO2210N_WINDOWX2_H, 0x07},
	{PO2210N_WINDOWX2_L, 0x84},
	{PO2210N_WINDOWY2_H, 0x04},
	{PO2210N_WINDOWY2_L, 0x3C},
//	{PO2210N_SCALE_X,	 0x20},
//	{PO2210N_SCALE_Y,	 0x20},
	{PO2210N_AE_FWX1_H,	 0x00},
	{PO2210N_AE_FWX1_L,	 0x05},
	{PO2210N_AE_FWX2_H,	 0x07},
	{PO2210N_AE_FWX2_L,	 0x84},
	{PO2210N_AE_FWY1_H,	 0x00},
	{PO2210N_AE_FWY1_L,	 0x05},
	{PO2210N_AE_FWY2_H,	 0x04},
	{PO2210N_AE_FWY2_L,	 0x3C},
	{PO2210N_AE_CWX1_H,	 0x02},
	{PO2210N_AE_CWX1_L,	 0x85},
	{PO2210N_AE_CWX2_H,	 0x05},
	{PO2210N_AE_CWX2_L,	 0x04},
	{PO2210N_AE_CWY1_H,	 0x01},
	{PO2210N_AE_CWY1_L,	 0x6D},
	{PO2210N_AE_CWY2_H,	 0x02},
	{PO2210N_AE_CWY2_L,	 0xD4},
	{PO2210N_AE_XAXIS_H,	 0x03},
	{PO2210N_AE_XAXIS_L,	 0xC5},
	{PO2210N_AE_YAXIS_H,	 0x02},
	{PO2210N_AE_YAXIS_L,	 0x21},
	{PO2210N_AWB_WX1_H,	 0x00},
	{PO2210N_AWB_WX1_L,	 0x05},
	{PO2210N_AWB_WX2_H,	 0x07},
	{PO2210N_AWB_WX2_L,	 0x84},
	{PO2210N_AWB_WY1_H,	 0x00},
	{PO2210N_AWB_WY1_L,	 0x05},
	{PO2210N_AWB_WY2_H,	 0x04},
	{PO2210N_AWB_WY2_L,	 0x3C},
};
/* 
	VGA NEED to set registers like 1280X960,
	then scaled to 1/2(X) 1/2(Y).
	
	74250000Hz / ((5279+1) * (1124 + 1)) = 12.5Hz 

	WINDOWX1        0x0145 -> 325
	WINDOWY1        0x0041 -> 65
	WINDOWX2        0x0644 -> 1604
	WOMDOWY2        0x0400 -> 1024
	
	AE_FWX1		0x0145 -> 325
	AE_FWX2		0x0644 -> 1604
	AE_FWY1		0x0041 -> 65
	AE_FWY2		0x0400 -> 1024
	// 426
	AE_CWX1		0x02EF -> 751 (325 + 426)
	AE_CWX2		0x049A -> 1178 (1604 - 426)
	// 320
	AE_CWY1		0x0181 -> 385 (65 + 320)
	AE_CWY2		0x02C0 -> 704 (1024 - 320)
	AE_XAXIS	0x03C5 -> 965
	AE_YAXIS	0x0221 -> 545
	
	AWB_WX1		0x0145 -> 325
	AWB_WX2		0x0644 -> 1604
	AWB_WY1		0x0041 -> 65
	AWB_WY2		0x0400 -> 1024
   
  */
static struct regval_list vga_reg_vals[] = {
	//{PO2210N_FRAMEWIDTH_H, 0x14},
	//{PO2210N_FRAMEWIDTH_L, 0x9F},
	{PO2210N_WINDOWX1_H, 0x01},
	{PO2210N_WINDOWX1_L, 0x45},
	{PO2210N_WINDOWY1_H, 0x00},
	{PO2210N_WINDOWY1_L, 0x41},
	{PO2210N_WINDOWX2_H, 0x06},
	{PO2210N_WINDOWX2_L, 0x44},
	{PO2210N_WINDOWY2_H, 0x04},
	{PO2210N_WINDOWY2_L, 0x00},
	{PO2210N_SCALE_X,	 0x40},
	{PO2210N_SCALE_Y,	 0x40},
	{PO2210N_AE_FWX1_H,	 0x01},
	{PO2210N_AE_FWX1_L,	 0x45},
	{PO2210N_AE_FWX2_H,	 0x06},
	{PO2210N_AE_FWX2_L,	 0x44},
	{PO2210N_AE_FWY1_H,	 0x00},
	{PO2210N_AE_FWY1_L,	 0x41},
	{PO2210N_AE_FWY2_H,	 0x04},
	{PO2210N_AE_FWY2_L,	 0x00},
	{PO2210N_AE_CWX1_H,	 0x02},
	{PO2210N_AE_CWX1_L,	 0xEF},
	{PO2210N_AE_CWX2_H,	 0x04},
	{PO2210N_AE_CWX2_L,	 0x9A},
	{PO2210N_AE_CWY1_H,	 0x01},
	{PO2210N_AE_CWY1_L,	 0x81},
	{PO2210N_AE_CWY2_H,	 0x02},
	{PO2210N_AE_CWY2_L,	 0xC0},
	{PO2210N_AE_XAXIS_H,	 0x03},
	{PO2210N_AE_XAXIS_L,	 0xC5},
	{PO2210N_AE_YAXIS_H,	 0x02},
	{PO2210N_AE_YAXIS_L,	 0x21},
	{PO2210N_AWB_WX1_H,	 0x01},
	{PO2210N_AWB_WX1_L,	 0x45},
	{PO2210N_AWB_WX2_H,	 0x06},
	{PO2210N_AWB_WX2_L,	 0x44},
	{PO2210N_AWB_WY1_H,	 0x00},
	{PO2210N_AWB_WY1_L,	 0x41},
	{PO2210N_AWB_WY2_H,	 0x04},
	{PO2210N_AWB_WY2_L,	 0x00},
};

static struct regval_list qvga_reg_vals[] = {
	{PO2210N_WINDOWX1_L, 0x05},
	{PO2210N_WINDOWY1_L, 0x05},
	{PO2210N_WINDOWX2_H, 0x07},
	{PO2210N_WINDOWX2_L, 0x84},
	{PO2210N_WINDOWY2_H, 0x04},
	{PO2210N_WINDOWY2_L, 0x3C},
//	{PO2210N_SCALE_X,	 0x20},
//	{PO2210N_SCALE_Y,	 0x20},
	{PO2210N_AE_FWX1_H,	 0x00},
	{PO2210N_AE_FWX1_L,	 0x05},
	{PO2210N_AE_FWX2_H,	 0x07},
	{PO2210N_AE_FWX2_L,	 0x84},
	{PO2210N_AE_FWY1_H,	 0x00},
	{PO2210N_AE_FWY1_L,	 0x05},
	{PO2210N_AE_FWY2_H,	 0x04},
	{PO2210N_AE_FWY2_L,	 0x3C},
	{PO2210N_AE_CWX1_H,	 0x02},
	{PO2210N_AE_CWX1_L,	 0x85},
	{PO2210N_AE_CWX2_H,	 0x05},
	{PO2210N_AE_CWX2_L,	 0x04},
	{PO2210N_AE_CWY1_H,	 0x01},
	{PO2210N_AE_CWY1_L,	 0x6D},
	{PO2210N_AE_CWY2_H,	 0x02},
	{PO2210N_AE_CWY2_L,	 0xD4},
	{PO2210N_AE_XAXIS_H,	 0x03},
	{PO2210N_AE_XAXIS_L,	 0xC5},
	{PO2210N_AE_YAXIS_H,	 0x02},
	{PO2210N_AE_YAXIS_L,	 0x21},
	{PO2210N_AWB_WX1_H,	 0x00},
	{PO2210N_AWB_WX1_L,	 0x05},
	{PO2210N_AWB_WX2_H,	 0x07},
	{PO2210N_AWB_WX2_L,	 0x84},
	{PO2210N_AWB_WY1_H,	 0x00},
	{PO2210N_AWB_WY1_L,	 0x05},
	{PO2210N_AWB_WY2_H,	 0x04},
	{PO2210N_AWB_WY2_L,	 0x3C},
};

static struct po2210n_win_size_struct {
	int	width;
	int	height;
	struct regval_list *regs; /* Regs to tweak */
	int nlist;
/* h/vref stuff */
} po2210n_win_sizes[] = {
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.regs 		= qvga_reg_vals,
		.nlist		= ARRAY_SIZE(qvga_reg_vals),
	},
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.regs 		= vga_reg_vals,
		.nlist		= ARRAY_SIZE(vga_reg_vals),
	},
	/* HD 720p */
	{
		.width		= HD_WIDTH,
		.height		= HD_HEIGHT,
		.regs 		= hd720p_reg_vals,
		.nlist		= ARRAY_SIZE(hd720p_reg_vals),
	},
	/* FHD */
	{
		.width		= FHD_WIDTH,
		.height		= FHD_HEIGHT,
		.regs 		= fhd_reg_vals,
		.nlist		= ARRAY_SIZE(fhd_reg_vals),
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(po2210n_win_sizes))

static inline struct po2210n_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct po2210n_state, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct po2210n_state, handler)->sd;
}


#define I2C_FLAG_READ	0x10

static int po2210n_i2c_xfer(struct i2c_client *client, int reg, char *buf, int num, int tran_flag)
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

static int reg_page_map_set(struct i2c_client *client, const u16 reg)
{
	int ret;
	u8 temp1;
	u8 page;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct po2210n_state *cam = to_state(sd);

	/* cam->priv_data would be PageMap cache value */
	page = (reg >> 8);
	if (page == cam->priv_data)
		return 0;
	if (page > 9)
		return -EINVAL;

	temp1 = PO2210N_BANK;

	ret = po2210n_i2c_xfer(client, temp1, (u8 *) & page, 1, 0);
	if (ret >= 0)
		cam->priv_data = page;
	return ret;
}

static int po2210n_read_reg8(struct i2c_client *client, u16	reg, u8 *val)
{
	int ret;
	u8 rval;

	ret = reg_page_map_set(client, reg);
	if (ret < 0) return ret;

	reg &= 0xFF;

	ret = po2210n_i2c_xfer(client, reg, (u8 *) &rval, 1, I2C_FLAG_READ);
	if (0 == ret) {
		*val = rval;
		return 0;
	}
	return ret;
}

static int po2210n_write_reg8(struct i2c_client *client,u16 reg, u8 val)
{
	u8 temp1;
	int ret;

	temp1 = reg & 0xFF;
	ret = reg_page_map_set(client, reg);
	if (ret < 0) return ret;
	//printk("write reg %x val %x.\n", reg, val);
	return po2210n_i2c_xfer(client, temp1, (u8 *) & val, 1, 0);
}

static int po2210n_write_reg8_array(struct i2c_client *client, struct regval_list *regs, int n)
{
	int i, ret;

	for (i=0; i<n; i++, regs++) {
		ret = po2210n_write_reg8(client, regs->reg_num, regs->value);
		if (ret) break;
	}
	return ret;
}


static int po2210n_start(struct i2c_client *client)
{
	int err;

	err = po2210n_write_reg8_array(client, po2210n_def_regs, PO2210N_DEF_REG_SIZE);
#if 0
	for (i=0; i<256; i++) {
		po2210n_read_reg8(client, i + 0x0000, &val);
		printk("page A: %x => %x\n", i, val);
	}
	for (i=0; i<256; i++) {
		po2210n_read_reg8(client, i + 0x0100, &val);
		printk("page B: %x => %x\n", i, val);
	}
	for (i=0; i<256; i++) {
		po2210n_read_reg8(client, i + 0x0200, &val);
		printk("page C: %x => %x\n", i, val);
	}
	for (i=0; i<256; i++) {
		po2210n_read_reg8(client, i + 0x0300, &val);
		printk("page D: %x => %x\n", i, val);
	}
	for (i=0; i<256; i++) {
		po2210n_read_reg8(client, i + 0x0400, &val);
		printk("page E: %x => %x\n", i, val);
	}
	for (i=0; i<256; i++) {
		po2210n_read_reg8(client, i + 0x0500, &val);
		printk("page F: %x => %x\n", i, val);
	}
	for (i=0; i<256; i++) {
		po2210n_read_reg8(client, i + 0x0700, &val);
		printk("page H: %x => %x\n", i, val);
	}
#endif
	return err;
}

/* Save power, so shutdown the chip. */
static int po2210n_stop(struct i2c_client *client)
{
	/* PixelPlus's recomendation */
	po2210n_write_reg8(client, PO2210N_POWER_CTRLX, PO2210N_PCX_POWERSAVE);
	
	return 0;
}

/*
 * Stuff that knows about the sensor.
 */
static int po2210n_reset(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	po2210n_write_reg8(client, PO2210N_SOFTRESET, 0x01); /* Software reset */
	udelay(1000); /* wait some time */
	po2210n_write_reg8(client, PO2210N_SOFTRESET, 0x00); /* Clear software reset */

	return 0;
}

static int po2210n_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	po2210n_start(client);

	return 0;
}

/*!
 * po2210 detect
 *
 * @return 0(OK) or -NODEV
 */
static int po2210n_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val;

	int ret;

	/* read id register and compare to correct value.. */
	ret = po2210n_read_reg8(client, PO2210N_DEVICEID_H, &val);

	if (ret)
		return ret;

	if (val != 0x0090)
		return -ENODEV;
	
	ret = po2210n_read_reg8(client, PO2210N_DEVICEID_L, &val);
	if (ret)
		return ret;

	if (val != 0x0030)
		return -ENODEV;
	
	return 0;
}

static int po2210n_try_fmt_internal(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt,//linux-3.0
		struct po2210n_win_size_struct **ret_wsize,
		struct po2210n_format_struct **ret_format)
{
	struct po2210n_win_size_struct *wsize;

	pr_debug("%s enter\n", __func__);
		
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0
	
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = po2210n_win_sizes; wsize < po2210n_win_sizes + N_WIN_SIZES; wsize++) {
		pr_debug("width=%d height=%d\n", wsize->width, wsize->height);
		if ((wsize->width >= fmt->width) &&  (wsize->height >= fmt->height))//linux-3.0
			break;
	}
	
	if (wsize >= po2210n_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	else {
		pr_debug("%s exit with %d\n", __func__, -EINVAL);
		return -EINVAL;
	}
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;//linux-3.0
	fmt->height = wsize->height;//linux-3.0
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;//linux-3.0
	//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0
	
	/* Now test requested formats, and check if we could provide. */
	if (ret_format == NULL) {
		pr_debug("%s exit with no ret_format\n", __func__);
		return 0;
	}

	if (fmt->code == V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE) {
		*ret_format = &po2210n_formats[_INT_RGB444];
	} else
	if (fmt->code == V4L2_MBUS_FMT_RGB565_2X8_LE) {
		*ret_format = &po2210n_formats[_INT_RGB565];
	} else
	if (fmt->code == V4L2_MBUS_FMT_UYVY8_2X8) {
		*ret_format = &po2210n_formats[_INT_YUV422_UYVY];
	} else
	if (fmt->code == V4L2_MBUS_FMT_VYUY8_2X8) {
		*ret_format = &po2210n_formats[_INT_YUV422_VYUY];
	} else
	if (fmt->code == V4L2_MBUS_FMT_YUYV8_2X8) {
		*ret_format = &po2210n_formats[_INT_YUV422_YUYV];
	} else
	if (fmt->code == V4L2_MBUS_FMT_YVYU8_2X8) {
		*ret_format = &po2210n_formats[_INT_YUV422_YVYU];
	} else
	if (fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8) { // RAW BAYER...
		*ret_format = &po2210n_formats[_INT_RAW_BAYER];
	} else {
		*ret_format = NULL;
		pr_debug("%s exit with %d(fmt->code = %X)\n", __func__, -EINVAL, fmt->code);
		return -EINVAL;
	}

	return 0;
}

/*
 * Implement G/S_PARM.	There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int po2210n_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}

static int po2210n_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}


/*
 * Code for dealing with controls.
 */
static int po2210n_s_sat(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int po2210n_g_sat(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int po2210n_s_hue(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int po2210n_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

#define MANUAL_EXPMODE_01	PO2210N_EM_MANUAL_EXPOSURE
#define MANUAL_EXPMODE_10	PO2210N_EM_ME_EXT_INTTIME_GBGAIN

#define MANUAL_EXPOSURE_MODE	MANUAL_EXPMODE_01

static int po2210n_s_brightness(struct v4l2_subdev *sd, int value)
{
	unsigned int maxval;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	/* Value would be 0 to 100 */
	/*	AE mode 00 01 10 11 */
#if (MANUAL_EXPOSURE_MODE == MANUAL_EXPMODE_10)
	maxval = 0x1000 - 0x0100;
	maxval = maxval * value / 100 + 0x0100;
	
	po2210n_write_reg8(client, PO2210N_EXT_GLBG_L, maxval&0xFF);
	po2210n_write_reg8(client, PO2210N_EXT_GLBG_H, (maxval>>8)&0xFF);
#elif (MANUAL_EXPOSURE_MODE == MANUAL_EXPMODE_01)
	maxval = 163840 * value / 255; 
	po2210n_write_reg8(client, PO2210N_EXPOSURE_T, (maxval >> 24) & 0xFF );
	po2210n_write_reg8(client, PO2210N_EXPOSURE_H, (maxval >> 16) & 0xFF);
	po2210n_write_reg8(client, PO2210N_EXPOSURE_M, (maxval >> 8) & 0xFF );
	po2210n_write_reg8(client, PO2210N_EXPOSURE_L, maxval & 0xFF);
#else
#error "You must select manual brightness method.\n"
#endif
	return 0;
}

static int po2210n_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int po2210n_s_contrast(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int po2210n_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int po2210n_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int po2210n_s_hflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int po2210n_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int po2210n_s_vflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int po2210n_g_exposure_auto(struct v4l2_subdev *sd, __s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 val;

	po2210n_read_reg8(client, PO2210N_EXPOSURE_MODE, &val);

	*value = (val == 0x00) ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL;

	return 0;
}

static int po2210n_s_exposure_auto(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 val;

	if (value == V4L2_EXPOSURE_AUTO)
		val =  0;	// Auto Exposure
	else
	if (value == V4L2_EXPOSURE_MANUAL)
		val =  MANUAL_EXPOSURE_MODE;	// Manual Exposure
	else
		return -1;

	pr_debug("Exposure mode = %x\n", val);
	po2210n_write_reg8(client, PO2210N_EXPOSURE_MODE, val);

	return 0;
}

static int po2210n_s_camera(struct v4l2_subdev *sd, int value)
{
	struct po2210n_state *info = to_state(sd);

	if (info->id == 0)
		NX_VIP_SetInputPort( 0, (NX_VIP_INPUTPORT) value );

	return 0;
}

static int po2210n_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	switch (qc->id) {
	case V4L2_CID_BRIGHTNESS:
		return v4l2_ctrl_query_fill(qc, 0, 1024, 1, 20);
	case V4L2_CID_CONTRAST:
		return v4l2_ctrl_query_fill(qc, 0, 127, 1, 64);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_SATURATION:
		return v4l2_ctrl_query_fill(qc, 0, 256, 1, 128);
	case V4L2_CID_HUE:
		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	case V4L2_CID_EXPOSURE_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	}
	return -EINVAL;
}

static int po2210n_subdev_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_BRIGHTNESS:
		return po2210n_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return po2210n_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return po2210n_g_sat(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return po2210n_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return po2210n_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return po2210n_g_hflip(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return po2210n_g_exposure_auto(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int po2210n_subdev_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_BRIGHTNESS:
		return po2210n_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return po2210n_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return po2210n_s_sat(sd, ctrl->value);
	case V4L2_CID_HUE:
		return po2210n_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return po2210n_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return po2210n_s_hflip(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return po2210n_s_exposure_auto(sd, ctrl->value);
	}
	return -EINVAL;
}

static int po2210n_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* PO2210 is not registered for V4L2-chip-ident... */
	po2210n_detect(sd);
	
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int po2210n_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val = 0;
	int ret;

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ret = po2210n_read_reg8(client, reg->reg & 0xff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int po2210n_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	po2210n_write_reg8(client, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

#define NUM_CTRLS				10

#define V4L2_CID_CAMERA_SCENE_MODE		(V4L2_CTRL_CLASS_CAMERA | 0x1001)
#define V4L2_CID_CAMERA_ANTI_SHAKE		(V4L2_CTRL_CLASS_CAMERA | 0x1002)
#define V4L2_CID_CAMERA_MODE_CHANGE		(V4L2_CTRL_CLASS_CAMERA | 0x1003)
#define V4L2_CID_CAMERA_SELECT			(V4L2_CTRL_CLASS_CAMERA | 0x1004)
#define V4L2_CID_CAMERA_LASER_CTRL		(V4L2_CTRL_CLASS_CAMERA | 0x1005)

static int po2210n_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev	*sd = ctrl_to_sd(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return po2210n_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return po2210n_s_contrast(sd, ctrl->val);
	case V4L2_CID_SATURATION:
		return po2210n_s_sat(sd, ctrl->val);
	case V4L2_CID_HUE:
		return po2210n_s_hue(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return po2210n_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return po2210n_s_hflip(sd, ctrl->val);
	case V4L2_CID_EXPOSURE_AUTO:
		return po2210n_s_exposure_auto(sd, ctrl->val);
	case V4L2_CID_CAMERA_SELECT:
		return po2210n_s_camera(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops po2210n_ctrl_ops = {
	.s_ctrl = po2210n_s_ctrl,
};

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

static const struct v4l2_ctrl_config po2210n_custom_ctrls[] = {
    {
        .ops    = &po2210n_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SCENE_MODE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "SceneMode",
        .min    = 0,
        .max    = SCENE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &po2210n_ctrl_ops,
        .id     = V4L2_CID_CAMERA_ANTI_SHAKE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "AntiShake",
        .min    = 0,
        .max    = ANTI_SHAKE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &po2210n_ctrl_ops,
        .id     = V4L2_CID_CAMERA_MODE_CHANGE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "ModeChange",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &po2210n_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SELECT,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "CameraSelect",
        .min    = 0,
        .max    = 1,
        .def    = 1,
        .step   = 1,
    }, {
        .ops    = &po2210n_ctrl_ops,
        .id     = V4L2_CID_CAMERA_LASER_CTRL,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "LaserCtrl",
        .min    = 0,
        .max    = 3,
        .def    = 3,
        .step   = 1,
    },
};

static int po2210n_initialize_ctrls(struct po2210n_state *state)
{
    v4l2_ctrl_handler_init(&state->handler, NUM_CTRLS);

    /* standard */
    state->gain = v4l2_ctrl_new_std(&state->handler, &po2210n_ctrl_ops,
            V4L2_CID_GAIN, 0, 4096, 1, 0);
    if (!state->gain) {
        pr_err("%s: failed to create gain ctrl\n", __func__);
        return -1;
    }
    state->exposure_auto = v4l2_ctrl_new_std_menu(&state->handler, &po2210n_ctrl_ops,
            V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
    if (!state->exposure_auto) {
        pr_err("%s: failed to create exposure_auto ctrl\n", __func__);
        return -1;
    }
    state->wb = v4l2_ctrl_new_std(&state->handler, &po2210n_ctrl_ops,
            V4L2_CID_DO_WHITE_BALANCE, WB_AUTO, WB_MAX - 1, 1, WB_AUTO);
    if (!state->wb) {
        pr_err("%s: failed to create wb ctrl\n", __func__);
        return -1;
    }
    state->color_effect = v4l2_ctrl_new_std_menu(&state->handler, &po2210n_ctrl_ops,
            V4L2_CID_COLORFX, COLORFX_MAX - 1, 0, COLORFX_NONE);
    if (!state->color_effect) {
        pr_err("%s: failed to create color_effect ctrl\n", __func__);
        return -1;
    }
    state->exposure = v4l2_ctrl_new_std_menu(&state->handler, &po2210n_ctrl_ops,
            V4L2_CID_EXPOSURE, MAX_EXPOSURE, 0, 0);
    if (!state->exposure) {
        pr_err("%s: failed to create exposure ctrl\n", __func__);
        return -1;
    }

    /* custom */
    state->scene_mode = v4l2_ctrl_new_custom(&state->handler, &po2210n_custom_ctrls[0], NULL);
    if (!state->scene_mode) {
        pr_err("%s: failed to create scene_mode ctrl\n", __func__);
        return -1;
    }
    state->anti_shake = v4l2_ctrl_new_custom(&state->handler, &po2210n_custom_ctrls[1], NULL);
    if (!state->anti_shake) {
        pr_err("%s: failed to create anti_shake ctrl\n", __func__);
        return -1;
    }
    state->mode_change = v4l2_ctrl_new_custom(&state->handler, &po2210n_custom_ctrls[2], NULL);
    if (!state->mode_change) {
        pr_err("%s: failed to create mode_change ctrl\n", __func__);
        return -1;
    }

    state->camera_select = v4l2_ctrl_new_custom(&state->handler, &po2210n_custom_ctrls[3], NULL);
    if (!state->camera_select) {
        pr_err("%s: failed to create camera_select ctrl\n", __func__);
        return -1;
    }

    state->laser_ctrl = v4l2_ctrl_new_custom(&state->handler, &po2210n_custom_ctrls[4], NULL);
    if (!state->laser_ctrl) {
        pr_err("%s: failed to create camera_select ctrl\n", __func__);
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

static int __po2210n_set_power(struct po2210n_state *state, int on)
{
	// Do something to power up
	return 0;
}

static int po2210n_set_power(struct v4l2_subdev *subdev, int on)
{
	struct po2210n_state *po2210n = to_state(subdev);
	int ret = 0;

	pr_debug("%s : on(%d)\n", __func__, on);
	mutex_lock(&po2210n->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (po2210n->power_count == !on) {
		ret = __po2210n_set_power(po2210n, !!on);
		if (ret < 0)
			goto done;
	}

	/* Update the power count. */
	po2210n->power_count += on ? 1 : -1;
	WARN_ON(po2210n->power_count < 0);

done:
	mutex_unlock(&po2210n->power_lock);
	return ret;
}

static int po2210n_set_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	int err = 0;
	struct v4l2_mbus_framefmt *_fmt = &fmt->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct po2210n_state *state = to_state(sd);
	struct po2210n_win_size_struct *wsize;
	struct po2210n_format_struct *format;

	pr_debug("%s: %dx%d\n", __func__, _fmt->width, _fmt->height);

	state->req_fmt.width = _fmt->width;
	state->req_fmt.height = _fmt->height;
	
	pr_debug("%s: %dx%d code = %x\n", __func__, state->req_fmt.width, state->req_fmt.height, _fmt->code);

	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	err = po2210n_try_fmt_internal(sd, _fmt, &wsize, &format);
	if (err < 0)
		return err;

	err = 0;
	if (wsize->regs) {
		pr_debug("final : %dx%d\n", wsize->width, wsize->height);
		err = po2210n_write_reg8_array(client, wsize->regs , wsize->nlist);
		if (err < 0)
			return err;
	}
	if (format != NULL) {
		err = po2210n_write_reg8_array(client, format->regs, format->nlist);
		if (err < 0)
			return err;
	}
	

	return err;
}

static int po2210n_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	pr_info("stream mode = %d\n", enable);

	if (!enable) {
		//po2210n_stop(client);
	} else {
		po2210n_start(client);
	}

	if (unlikely(err < 0)) {
		pr_err("ERR: faild\n");
		return err;
	}

	return 0;
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops po2210n_core_ops = {
	.g_chip_ident = po2210n_g_chip_ident,
	.g_ctrl = po2210n_subdev_g_ctrl,
	.s_ctrl = po2210n_subdev_s_ctrl,
	.queryctrl = po2210n_queryctrl,
	.s_power = po2210n_set_power,
	.reset = po2210n_reset,
	.init = po2210n_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = po2210n_g_register,
	.s_register = po2210n_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops po2210n_pad_ops = {
	.set_fmt  = po2210n_set_fmt,
	//.set_crop = po2210n_set_crop,
	//.get_crop = po2210n_get_crop,
};


static const struct v4l2_subdev_video_ops po2210n_video_ops = {
	.s_parm = po2210n_s_parm,
	.g_parm = po2210n_g_parm,
	.s_stream = po2210n_s_stream,
};

static const struct v4l2_subdev_ops po2210n_ops = {
	.core = &po2210n_core_ops,
	.video = &po2210n_video_ops,
	.pad = &po2210n_pad_ops,
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

static const struct media_entity_operations po2210n_media_ops = {
	.link_setup = _link_setup,
};
/* ----------------------------------------------------------------------- */

static int po2210n_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct po2210n_state *info;
	int ret = 0;

	info = kzalloc(sizeof(struct po2210n_state), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &po2210n_ops);

	/* Make sure it's an po2210 */
	if (strstr(id->name, "_2"))
		ret = po2210n_detect(sd);
	if (ret) {
		printk("po2210n_detect error!");
		v4l_dbg(1, debug, client,
			"chip found @ 0x%x (%s) is not a po2210n chip.\n",
			client->addr << 1, client->adapter->name);
		kfree(info);
		return ret;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	mutex_init(&info->power_lock);
	info->power_count = 0;
	
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.ops = &po2210n_media_ops;
	if (media_entity_init(&sd->entity, 1, &info->pad, 0)) {
		dev_err(&client->dev, "%s: failed to media_entity_init()\n", __func__);
		kfree(info);
		return -ENOENT;
	}

	ret = po2210n_initialize_ctrls(info);
	if (ret < 0) {
		pr_err("%s: failed to initialize controls\n", __func__);
		kfree(info);
		return ret;
	}
	
	info->inited = 1;
	info->priv_data = -1; /* page map value */

	printk("PO2210N is probed and ready to use.\n");

	return 0;
}

static int po2210n_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return	0;
}

static const struct i2c_device_id po2210n_id[] = {
	{ "po2210n", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, po2210n_id);

static struct i2c_driver po2210n_driver = {
	.driver = {
		.name = "po2210n",
		.owner = THIS_MODULE,
	},
	.id_table	  = po2210n_id,
	.probe		  = po2210n_probe,
	.remove		  = __devexit_p(po2210n_remove),
};

static __init int _po2210n_init(void)
{
	int ret;

	ret = i2c_add_driver(&po2210n_driver);

	return ret; 
}

static __init void _po2210n_exit(void)
{
	i2c_del_driver(&po2210n_driver);
}

module_init(_po2210n_init)
module_exit(_po2210n_exit)

MODULE_AUTHOR("Seungwoo Kim<ksw@stcube.com>");
MODULE_DESCRIPTION("PO2210N Camera Driver");
MODULE_LICENSE("GPL");
