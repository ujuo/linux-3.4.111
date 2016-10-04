/*
 * Driver for IMX117 12MPixel High framerate CMOS Image Sensor from Sony
 *
 *  Copyright(c) 2014 STCube co., ltd.
 *  All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 * Sony IMX117 is not i2c interface but SPI like system without read register function.
 * I don't have experiance with this type sensors(ksw).
 * Just try to initialize correctly.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/v4l2-mediabus.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>

#include <media/soc_camera.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>

#include "imx117.h"

/* IMX117 registers */

#define REG_STANDBY				0x0000
#define REG_STANDBY_BIT_STANDBY		(1 << 0)
#define REG_STANDBY_BIT_STBLOGIC	(1 << 1)

#define REG_RESET				0x0001
#define REG_RESET_BIT_DCKRST		(1 << 0)
#define REG_RESET_BIT_CLPSQRST		(1 << 4)

#define REG_BREAK				0x0002
#define REG_BREAK_BIT_SSBRK			(1 << 0)

#define REG_LVDS_STANDBY		0x0003
#define REG_LS_STBLVDS_10CH			(0)
#define REG_LS_STBLVDS_8CH			(1)
#define REG_LS_STBLVDS_6CH			(2)
#define REG_LS_STBLVDS_4CH			(3)
#define REG_LS_STBLVDS_2CH			(4)
#define REG_LS_STBLVDS_1CH			(5)
#define REG_LS_STBLVDS_ALL			(0x0F)
#define REG_LS_CHSEL_10CH			((0) << 4)
#define REG_LS_CHSEL_8CH			((1) << 4)
#define REG_LS_CHSEL_6CH			((2) << 4)
#define REG_LS_CHSEL_4CH			((3) << 4)
#define REG_LS_CHSEL_2CH			((4) << 4)
#define REG_LS_CHSEL_1CH			((5) << 4)

#define REG_MDSEL1				0x0004
#define REG_MDSEL2				0x0005
#define REG_MDSEL3				0x0006
#define REG_MDSEL4				0x0007

#define REG_GLB_SHUTTER			0x0008
#define REG_GLB_SHT_SMD				(1 << 0)
#define REG_GLB_SHT_GSVR ( x )		((x) << 4)

#define REG_PGC_LOW				0x0009
#define REG_PGC_HIGH			0x000A

#define REG_SHR_LOW				0x000B
#define REG_SHR_HIGH			0x000C

#define REG_SVR_LOW				0x000D
#define REG_SVR_HIGH			0x000E

#define REG_SPL_LOW				0x000F
#define REG_SPL_HIGH			0x0010

#define REG_DGAIN				0x0011
#define REG_DGAIN_0dB				(0)
#define REG_DGAIN_6dB				(1)
#define REG_DGAIN_12dB				(2)
#define REG_DGAIN_18dB				(3)

#define REG_MDVREV				0x001A
#define REG_MDVREV_BIT_VERT_NORMAL	(0)
#define REG_MDVREV_BIT_VERT_INVERT	(1)

#define REG_MDSEL13				0x0026
#define REG_MDSEL14				0x0027
#define REG_MDSEL15				0x0028


#define REG_MDSEL5_LOW			0x007E
#define REG_MDSEL5_HIGH			0x007F

#define REG_BLKLEVEL			0x0045

#define REG_MDPLS01				0x0080
#define REG_MDPLS02				0x0081
#define REG_MDPLS03				0x0082
#define REG_MDPLS04				0x0083
#define REG_MDPLS05				0x0084
#define REG_MDPLS06				0x0085
#define REG_MDPLS07				0x0086
#define REG_MDPLS08				0x0087
#define REG_MDPLS09				0x0095
#define REG_MDPLS10				0x0096
#define REG_MDPLS11				0x0097
#define REG_MDPLS12				0x0098
#define REG_MDPLS13				0x0099
#define REG_MDPLS14				0x009A
#define REG_MDPLS15				0x009B
#define REG_MDPLS16				0x009C

#define REG_MDSEL6				0x00B6
#define REG_MDSEL7				0x00B7
#define REG_MDSEL8				0x00B8
#define REG_MDSEL9				0x00B9
#define REG_MDSEL10				0x00BA
#define REG_MDSEL11				0x00BB

#define REG_MDPLS17				0x00BC
#define REG_MDPLS18				0x00BD
#define REG_MDPLS19				0x00BE
#define REG_MDPLS20				0x00BF
#define REG_MDPLS21				0x00C0
#define REG_MDPLS22				0x00C1
#define REG_MDPLS23				0x00C2
#define REG_MDPLS24				0x00C3
#define REG_MDPLS25				0x00C4
#define REG_MDPLS26				0x00C5
#define REG_MDPLS27				0x00C6
#define REG_MDPLS28				0x00C7
#define REG_MDPLS29				0x00C8
#define REG_MDPLS30				0x00C9
#define REG_MDPLS31				0x00CA
#define REG_MDPLS32				0x00CB
#define REG_MDPLS33				0x00CC

#define REG_MDSEL12				0x00CE

#define REG_PLSTMG11_LOW		0x0222
#define REG_PLSTMG11_HIGH		0x0223

#define REG_APGC01_LOW			0x0352
#define REG_APGC01_HIGH			0x0353

#define REG_APGC02_LOW			0x0356
#define REG_APGC02_HIGH			0x0357

#define REG_PLSTMG00			0x0358

#define REG_PLSTMG01			0x0528
#define REG_PLSTMG13			0x0529

#define REG_PLSTMG02			0x052A

#define REG_PLSTMG14			0x052B
#define REG_PLSTMG15			0x0534
#define REG_PLSTMG03			0x057E
#define REG_PLSTMG04			0x057F
#define REG_PLSTMG05			0x0580
#define REG_PLSTMG06			0x0581

#define REG_PLSTMG07_LOW		0x0585
#define REG_PLSTMG07_HIGH		0x0586

#define REG_PLSTMG12			0x0617

#define REG_PLSTMG08			0x065C

#define REG_PLSTMG09_LOW		0x0700
#define REG_PLSTMG09_HIGH		0x0701

/* 
  According to IMX117 manual, and 
  document IMX117_Register Setting_20131217,
  Power On Sequence is like Following:
  
 1. Standby Cancel        : 0x0000               = 0x06
                            0x0001               = 0x00
 2. Break & Clamp reset   : 0x0002               = 0x00
 3. Stanby/SubLVDS chan   : 0x0003               = 0x00
 4. Mode select           : 0x0004               = 0x00
                            0x0005               = 0x07
                            0x0006               = 0x00
                            0x0007               = 0x00
 5. Global reset          : 0x0008               = 0x00
 6. Analog gain           : 0x0009               = 0x00
                            0x000A               = 0x00
 7. Electronic shutter    : 0x000B               = 0x00
                            0x000C               = 0x00
                            0x000D               = 0x00
                            0x000E               = 0x00
                            0x000F               = 0x00
                            0x0010               = 0x00
 8. Digital gain            0x0011               = 0x00
 9. Mirror                : 0x001A               = 0x60
 10. Pulse Timing Control : 0x0026(REG_MDSEL13)  = 0x74
 11. Black Level          : 0x0045(REG_BLKLEVEL) = 0x74
 12. Pulse Timing Control : 0x002C(Unknown)      = 0x74
 13. Black Level(Offset)  : 0x0045(REG_BLKLEVEL) = 0x32
 14. Pulse Timing Control : 0x0083               = 0x00
                            0x0084               = 0x00
                            0x0085               = 0x00
                            0x0086               = 0x00
                            0x0087               = 0x00
                            0x0095               = 0x00
                            0x0096               = 0x00
                            0x0097               = 0x00
                            0x0098               = 0x00
                            0x0099               = 0x00
                            0x009A               = 0x00
                            0x009B               = 0x00
                            
 15. Boot init setting    : 0x009C               = 0x00
                            0x00B6               = 0x47
                            0x00B7               = 0x00
                            0x00B8               = 0x00
                            0x00B9               = 0x00
                            0x00BA               = 0x00
                            0x00BB               = 0x00
                            0x00BC               = 0x00
                            0x00BD               = 0x00
                            0x00BE               = 0x00
                            0x00BF               = 0x00
                            0x00C0               = 0x00
                            0x00C1               = 0x00
                            
 16. SPL Init             : 0x00C2               = 0x00
                            0x00C3               = 0x00
 17. Standby mode cancel  : 0x00C4               = 0x00
 
 18. Mode setting         : 0x00C5               = 0x00 
     Set Readout mode 2?    0x00C6               = 0x00
     Set mode 4A            0x00C7               = 0x00
                            0x00C8               = 0x00
                            0x00C9               = 0x00
                            0x00CA               = 0x00
                            0x00CB               = 0x00
                            0x00CC               = 0x00
                            0x00CE               = 0x0E
                            0x0222               = 0x31
                            0x0223               = 0x01
                            0x0352               = 0x3D
                            0x0353               = 0x00
                            0x0356               = 0x3C
                            0x0357               = 0x00
                            0x0358               = 0x01
                            0x0528               = 0x0E
                            0x0529               = 0x0E
                            0x052A               = 0x0E
                            0x052B               = 0x0E
                            0x057E               = 0x00
                            0x057F               = 0x10
                            0x0580               = 0x0D
                            0x0581               = 0x0D
                            0x0585               = 0x00
                            0x0586               = 0x07
                            0x0617               = 0x10
                            0x065C               = 0x05
                            0x0700               = 0x19
                            0x0701               = 0x19
                            
 19. wait 1ms or more
 
 20. Standby mode cancel  : 0x0000               = 0x04

 21. wait next frame (16ms or more)

 22. Standby mode cancel  : 0x0001               = 0x00
 
*/       

/*
  Now I only support Readout mode 2,
  And this is the way they say I should do :

  Readout mode 2
  
  1. Standby              : 0x0003               = 0x33
  2. Mode select          : 0x0004               = 0x11
                            0x0005               = 0x0D
                            0x0006               = 0x00
                            0x0007               = 0x00
  3. Global reset         : 0x0008               = 0x00
  4. Mirror               : 0x001A               = 0x00
  5. Mode select          : 0x0026               = 0x74
                            0x0045               = 0x74
                            0x002C               = 0x74
                            0x027E               = 0x20
                            0x027F               = 0x01
  6. Mode select          : 0x00B6               = 0x67
                            0x00B7               = 0x00
                            0x00B8               = 0x00
                            0x00B9               = 0x00
                            0x00BA               = 0x00
                            0x00BB               = 0x00
  7. Mode select          : 0x00CE               = 0x0E

*/                                       
 
/* IMX117 supported geometry */
#define IMX117_WIDTH			2048
#define IMX117_HEIGHT			1152

#define CHIP_ID					0x81
                            
struct sony_registers {
	unsigned short int reg;
	unsigned char val;
};

struct sony_registers power_on_seq[] = {
	{ 0x0000, 0x06 },  
	{ 0x0001, 0x00 },
	{ 0x0002, 0x00 },
	{ 0x0003, 0x00 },
	{ 0x0004, 0x00 },
	{ 0x0005, 0x07 },
	{ 0x0006, 0x00 },
	{ 0x0007, 0x00 },
	{ 0x0008, 0x00 },
	{ 0x0009, 0x00 },
	{ 0x000A, 0x00 },
	{ 0x000B, 0x07 },
	{ 0x000C, 0x00 },
	{ 0x000D, 0x00 },
	{ 0x000E, 0x00 },
	{ 0x000F, 0x00 },
	{ 0x0010, 0x00 },
	{ 0x0011, 0x00 },
	{ 0x001A, 0x00 },
	{ 0x0026, 0x74 },
	{ 0x0027, 0x74 }, // 0x0045
	{ 0x0028, 0x74 }, // 0x002C
	{ 0x0045, 0x32 },
	{ 0x007E, 0x20 },
	{ 0x007F, 0x01 },
	{ 0x0280, 0x00 },
	{ 0x0081, 0x00 },
	{ 0x0082, 0x00 },
	{ 0x0083, 0x00 },
	{ 0x0084, 0x00 },
	{ 0x0085, 0x00 },
	{ 0x0086, 0x00 },
	{ 0x0087, 0x00 },
	{ 0x0095, 0x00 },
	{ 0x0096, 0x00 },
	{ 0x0097, 0x00 },
	{ 0x0098, 0x00 },
	{ 0x0099, 0x00 },
	{ 0x009A, 0x00 },
	{ 0x009B, 0x00 },
	{ 0x009C, 0x00 },
	{ 0x00B6, 0x47 },
	{ 0x00B7, 0x00 },
	{ 0x00B8, 0x00 },
	{ 0x00B9, 0x00 },
	{ 0x00BA, 0x00 },
	{ 0x00BB, 0x00 },
	{ 0x00BC, 0x00 },
	{ 0x00BD, 0x00 },
	{ 0x00BE, 0x00 },
	{ 0x00BF, 0x00 },
	{ 0x00C0, 0x00 },
	{ 0x00C1, 0x00 },
	{ 0x00C2, 0x00 },
	{ 0x00C3, 0x00 },
	{ 0x00C4, 0x00 },
	{ 0x00C5, 0x00 },
	{ 0x00C6, 0x00 },
	{ 0x00C7, 0x00 },
	{ 0x00C8, 0x00 },
	{ 0x00C9, 0x00 },
	{ 0x00CA, 0x00 },
	{ 0x00CB, 0x00 },
	{ 0x00CC, 0x00 },
	{ 0x00CE, 0x0E },
	{ 0x0222, 0x31 },
	{ 0x0223, 0x01 },
	{ 0x0352, 0x3D },
	{ 0x0353, 0x00 },
	
	{ 0x0356, 0x3C },
	{ 0x0357, 0x00 },
	{ 0x0358, 0x01 },
	{ 0x0528, 0x0E },
	{ 0x0529, 0x0E },
	{ 0x052A, 0x0E },
	{ 0x052B, 0x0E },
	{ 0x0534, 0x10 },
	{ 0x057E, 0x00 },
	{ 0x057F, 0x10 },
	{ 0x0580, 0x0D },
	{ 0x0581, 0x0D },
	{ 0x0585, 0x00 },
	{ 0x0586, 0x07 },
	{ 0x0617, 0x10 },
	{ 0x065C, 0x05 },
	{ 0x0700, 0x19 },
	{ 0x0701, 0x19 },
	{ 0xFFFF, 0x01 }, // wait 1ms
// Test pattern generation
	//{0x003B, 0x11}, // Test mode setting
	//{0x003C, 0x0A}, // Vertical color bar
	{ 0x003B, 0x00 }, // Test mode setting
	{ 0x003C, 0x00 }, // Vertical color bar

	{ 0x0000, 0x04 },
	{ 0xFFFE, 30   }, // wait 30ms (1 frame)
	{ 0x0001, 0x00 }
};

struct sony_registers readout_mode0[] = {
	{0x0003, 0x00},
	{0x0004	,0x00},
	{0x0005	,0x07},
	{0x0006	,0x00},
	{0x0007	,0x00},
	{0x0008	,0x00},
	{0x0012, 0x01}, // This is 144Mhz stuff
	{0x001A	,0x00},
	{0x0026	,0x74},
	{0x0027	,0x74},
	{0x0028	,0x74},
	{0x007E	,0x20},
	{0x007F	,0x01},
	{0x00B6	,0x47},
	{0x00B7	,0x00},
	{0x00B8	,0x00},
	{0x00B9	,0x00},
	{0X00BA	,0x00},
	{0x00BB	,0x00},
	{0x00CE	,0x0E},	
};

struct sony_registers readout_mode2[] = {
	{0x0003, 0x33},
	{0x0004, 0x89},
	{0x0005, 0x4D},             
	{0x0006, 0x00},
	{0x0007, 0x00},
	{0x0008, 0x00},
	{0x0012, 0x01}, // This is 144Mhz stuff
	{0x001A, 0x00},
	{0x0026, 0x74}, 
	{0x0027, 0x74}, // MDSEL14 0x45
	{0x0028, 0x74}, // MDSLE15 0x2C
	{0x007E, 0x20},
	{0x007F, 0x01},
	{0x00B6, 0x67},
	{0x00B7, 0x00},
	{0x00B8, 0x00},
	{0x00B9, 0x00},
	{0x00BA, 0x00},
	{0x00BB, 0x00},
	{0x00CE, 0x0E},
	//{0x0000, 0x05}, // Standby mode for test
	{0x000B, 0x0A}, // SHR : Shutter Speed to 1024
	{0x000C, 0x00},

	/*{0x0009, 0x00},
	{0x000A, 0x06},
	{REG_APGC01_LOW, 0x28},
	{REG_APGC01_HIGH, 0},
	{REG_APGC02_LOW, 0x27},
	{REG_APGC02_HIGH, 0x0},*/
	{0x0011, 0x00},
	
};


unsigned char bit_reverse_table[256] = {
	0x00,0x80,0x40,0xC0,0x20,0xA0,0x60,0xE0,
	0x10,0x90,0x50,0xD0,0x30,0xB0,0x70,0xF0,
	0x08,0x88,0x48,0xC8,0x28,0xA8,0x68,0xE8,
	0x18,0x98,0x58,0xD8,0x38,0xB8,0x78,0xF8,
	0x04,0x84,0x44,0xC4,0x24,0xA4,0x64,0xE4,
	0x14,0x94,0x54,0xD4,0x34,0xB4,0x74,0xF4,
	0x0C,0x8C,0x4C,0xCC,0x2C,0xAC,0x6C,0xEC,
	0x1C,0x9C,0x5C,0xDC,0x3C,0xBC,0x7C,0xFC,
	0x02,0x82,0x42,0xC2,0x22,0xA2,0x62,0xE2,
	0x12,0x92,0x52,0xD2,0x32,0xB2,0x72,0xF2,
	0x0A,0x8A,0x4A,0xCA,0x2A,0xAA,0x6A,0xEA,
	0x1A,0x9A,0x5A,0xDA,0x3A,0xBA,0x7A,0xFA,
	0x06,0x86,0x46,0xC6,0x26,0xA6,0x66,0xE6,
	0x16,0x96,0x56,0xD6,0x36,0xB6,0x76,0xF6,
	0x0E,0x8E,0x4E,0xCE,0x2E,0xAE,0x6E,0xEE,
	0x1E,0x9E,0x5E,0xDE,0x3E,0xBE,0x7E,0xFE,
	0x01,0x81,0x41,0xC1,0x21,0xA1,0x61,0xE1,
	0x11,0x91,0x51,0xD1,0x31,0xB1,0x71,0xF1,
	0x09,0x89,0x49,0xC9,0x29,0xA9,0x69,0xE9,
	0x19,0x99,0x59,0xD9,0x39,0xB9,0x79,0xF9,
	0x05,0x85,0x45,0xC5,0x25,0xA5,0x65,0xE5,
	0x15,0x95,0x55,0xD5,0x35,0xB5,0x75,0xF5,
	0x0D,0x8D,0x4D,0xCD,0x2D,0xAD,0x6D,0xED,
	0x1D,0x9D,0x5D,0xDD,0x3D,0xBD,0x7D,0xFD,
	0x03,0x83,0x43,0xC3,0x23,0xA3,0x63,0xE3,
	0x13,0x93,0x53,0xD3,0x33,0xB3,0x73,0xF3,
	0x0B,0x8B,0x4B,0xCB,0x2B,0xAB,0x6B,0xEB,
	0x1B,0x9B,0x5B,0xDB,0x3B,0xBB,0x7B,0xFB,
	0x07,0x87,0x47,0xC7,0x27,0xA7,0x67,0xE7,
	0x17,0x97,0x57,0xD7,0x37,0xB7,0x77,0xF7,
	0x0F,0x8F,0x4F,0xCF,0x2F,0xAF,0x6F,0xEF,
	0x1F,0x9F,0x5F,0xDF,0x3F,0xBF,0x7F,0xFF
};

/* IMX117 has only one fixed colorspace per pixelcode */
struct imx117_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct imx117_info {
	struct v4l2_subdev		subdev;
	struct media_pad pad;
	struct spi_device *spi;
	int    cur_readout_mode;
	const struct imx117_datafmt	*fmt;
	
	struct v4l2_ctrl_handler handler;
	/* standard */
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *exposure;
	/* custom */
	struct v4l2_ctrl *test_mode;
};

static const struct imx117_datafmt imx117_colour_fmts[] = {
	{V4L2_MBUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB},
};

static struct imx117_info *to_imx117_info(const struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx117_info, subdev);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct imx117_info, handler)->subdev;
}


/* Find a data format by a pixel code in an array */
static const struct imx117_datafmt *imx117_find_datafmt(enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(imx117_colour_fmts); i++)
		if (imx117_colour_fmts[i].code == code)
			return imx117_colour_fmts + i;

	return NULL;
}

/* Only write function exists for imx117 */
static int reg_write(struct spi_device *spi, const u16 addr, const u8 data)
{
	unsigned char txbuf[8];

	txbuf[0] = bit_reverse_table[CHIP_ID];
	txbuf[1] = bit_reverse_table[(addr >> 8) & 0xFF];
	txbuf[2] = bit_reverse_table[addr & 0xFF];
	txbuf[3] = bit_reverse_table[data];
	
	spi_write(spi, txbuf, 4);
	
	return	0; 
}    

static int write_regs_table(struct spi_device *spi, struct sony_registers *regs, int num)
{
	int i;
	struct sony_registers *p;
	
	p = regs;           
	for (i = 0; i< num; i++, p++) {
		if (p->reg >= 0xFFFE) {
			if (p->reg == 0xFFFF) {
				msleep(p->val);
			} else {
				msleep(p->val); // Do wait frame, instead.
			}
		} else {
			reg_write(spi, p->reg, p->val); 
		}	
	}
	return 0;
}

static int imx117_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	const struct imx117_datafmt *fmt = imx117_find_datafmt(mf->code);

	dev_dbg(sd->v4l2_dev->dev, "%s(%u)\n", __func__, mf->code);

	if (!fmt) {
		mf->code	= imx117_colour_fmts[0].code;
		mf->colorspace	= imx117_colour_fmts[0].colorspace;
	}

	mf->width	= IMX117_WIDTH;
	mf->height	= IMX117_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int imx117_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct imx117_info *priv = to_imx117_info(sd);
//?	struct spi_device *spi = priv->spi;

	dev_dbg(sd->v4l2_dev->dev, "%s(%u)\n", __func__, mf->code);

	/* MIPI CSI could have changed the format, double-check */
	if (!imx117_find_datafmt(mf->code))
		return -EINVAL;

	imx117_try_fmt(sd, mf);

	priv->fmt = imx117_find_datafmt(mf->code);

	return 0;
}

static int imx117_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct imx117_info *priv = to_imx117_info(sd);

	const struct imx117_datafmt *fmt = priv->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= IMX117_WIDTH;
	mf->height	= IMX117_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int imx117_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct v4l2_rect *rect = &a->c;

	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	rect->top	= 0;
	rect->left	= 0;
	rect->width	= IMX117_WIDTH;
	rect->height	= IMX117_HEIGHT;

	return 0;
}

static int imx117_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= IMX117_WIDTH;
	a->bounds.height		= IMX117_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int imx117_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if ((unsigned int)index >= ARRAY_SIZE(imx117_colour_fmts))
		return -EINVAL;

	*code = imx117_colour_fmts[index].code;
	return 0;
}

static int imx117_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx117_info *priv = to_imx117_info(sd);
	struct spi_device *spi = priv->spi;
	
	if (enable) {
		msleep(20); // May delay some ms...
		write_regs_table(spi, power_on_seq, sizeof(power_on_seq) / sizeof(struct sony_registers));
		// Now set mode 0
		if (priv->cur_readout_mode == 0) {
			write_regs_table(spi, readout_mode0, sizeof(readout_mode0) / sizeof(struct sony_registers));
		} else
		if (priv->cur_readout_mode == 2) { 
		// Now set mode 2
			write_regs_table(spi, readout_mode2, sizeof(readout_mode2) / sizeof(struct sony_registers));
		}
	} else {
		reg_write(spi, REG_STANDBY, 0x06);
	}
	return 0;
}

static int imx117_set_power(struct v4l2_subdev *subdev, int on)
{
	struct imx117_info *priv = to_imx117_info(subdev);
	struct spi_device *spi = priv->spi;

	pr_debug("%s: on(%d)\n", __func__, on);

	if (on) {
		// DO nothing...
		
	} else {
		reg_write(spi, REG_STANDBY, 0x07);
	}
	return 0;
}

#if 0
static int __g_volatile_ctrl(struct imx117_info *priv, struct v4l2_ctrl *ctrl)
{
	if (!priv->power)
		return 0;
	
	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:    
		break;
	
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

static int imx117_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct imx117_info *priv = to_imx117_info(sd);
	int ret;
	
	v4l2_dbg(1, debug, sd, "g_ctrl: %s\n", ctrl->name);
	
	mutex_lock(&priv->lock);
	ret = __g_volatile_ctrl(priv, ctrl);
	mutex_unlock(&priv->lock);
	return ret;
}
#endif

static int imx117_set_exposure(struct imx117_info *priv, int val)
{
	struct spi_device *spi = priv->spi;
	int shr, svr=0, spl=0;
	
	if (priv->cur_readout_mode == 0) {
		// exposure time = 8 to ((SVR + 1) * NumberOf XHS pulses per frame -4)
		shr = 3000 - val;
		if (shr < 8)
			shr = 8;
	} else
	if (priv->cur_readout_mode == 2) {
		// exposure time = 10 to ((SVR + 1) * NumberOf XHS pulses per frame -4)
		shr = 2166 - val;
		if (shr < 10)
			shr = 10;
	} else {
		return -1;
	}
	reg_write(spi, REG_SHR_LOW, shr & 0xFF);
	reg_write(spi, REG_SHR_HIGH, (shr >> 8) & 0xFF);
	
	reg_write(spi, REG_SVR_LOW, svr & 0xFF);
	reg_write(spi, REG_SVR_HIGH, (svr >> 8) & 0xFF);
	
	reg_write(spi, REG_SPL_LOW, spl & 0xFF);
	reg_write(spi, REG_SPL_HIGH, (spl >> 8) & 0xFF);
	
	return 0;	
}

static int get_apgcval(int rmode, int val, int *apgc01, int *apgc02)
{
		if (rmode != 0) {
			if (val <= 0x146) {
				*apgc01 = 0x1F;
			} else
			if (val <= 0x400) {
				*apgc01 = 0x21;
			} else
			if (val <= 0x52C) {
				*apgc01 = 0x26;
			} else
			if (val <= 0x696) {
				*apgc01 = 0x28;
			} else
			if (val <= 0x74B) {
				*apgc01 = 0x2C;
			} else {				
				*apgc01 = 0x30;
			}
		} else {
			*apgc01 = 0x3D;
		}
		*apgc02 = *apgc01 - 1;
		
		return 0;
}

static int imx117_set_gain(struct imx117_info *priv, int val)
{
	struct spi_device *spi = priv->spi;
	int ret = 0;
	int p01val, p02val;

	//
	// Gain [dB] = -20log((2048-PGC)/2048)
	//
	if (val > 1957) {
		int val2;
		// Then set digital gain...
		val2 = val / 2048;
		val -= val2 * 2048;
		// Set digital gain
		if (val2 > 3) {
			val2 = REG_DGAIN_18dB;
		} else
		if (val2 > 1) {
			val2 = REG_DGAIN_12dB;
		} else {
			val2 = REG_DGAIN_6dB;
		}
		reg_write(spi, REG_DGAIN, val2);
	}
	// Set analog gain.
	// 1. PGC setting.
	reg_write(spi, REG_PGC_LOW, val & 0xFF);
	reg_write(spi, REG_PGC_HIGH, (val >> 8) & 0xFF);
	
	ret = get_apgcval(priv->cur_readout_mode, val, &p01val, &p02val);
	// 2. APGC01 and APGC02
	reg_write(spi, REG_APGC01_LOW , p01val & 0xFF);
	reg_write(spi, REG_APGC01_HIGH, (p01val >> 8) & 0xFF);
	reg_write(spi, REG_APGC02_LOW , p02val & 0xFF);
	reg_write(spi, REG_APGC02_HIGH, (p02val >> 8) & 0xFF);

	return ret;
}

static int imx117_set_testmode(struct imx117_info *priv, int val)
{
	struct spi_device *spi = priv->spi;
	int ret = 0;

	switch (val) {
	case 0:
		reg_write(spi, 0x003B, 0x00); // Set normal mode
		reg_write(spi, 0x003C, 0x00); // Set normal mode
		break;
	case 1:
		reg_write(spi, 0x003B, 0x11);
		reg_write(spi, 0x003C, 0x0A); // Vertical BAR
		break;
	case 2:
		reg_write(spi, 0x003B, 0x11);
		reg_write(spi, 0x003C, 0x0B); // Horizontal BAR
		break;
	default:
		// Do nothing fo invalid value.
		ret = -1;
		break;
	}
	return ret;
}

#define NUM_CTRLS	3

static int imx117_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct imx117_info *priv = to_imx117_info(sd);

    int value = ctrl->val;
    int err = 0;
    
    switch (ctrl->id) {
        case V4L2_CID_EXPOSURE_ABSOLUTE:
            err = imx117_set_exposure(priv, value);
            break;

        case V4L2_CID_GAIN:
    		err = imx117_set_gain(priv, value); 	
        	break;

        case V4L2_CID_CAMERA_TEST_MODE:
        	err = imx117_set_testmode(priv, value);
        	break;

        default:
            pr_err("%s: no such control\n", __func__);
            err = -1;
            break;
    }

    return err;
}

static const struct v4l2_ctrl_ops imx117_ctrl_ops = {
	//.g_volatile_ctrl = imx117_g_volatile_ctrl,
	.s_ctrl = imx117_s_ctrl,
};

static const struct v4l2_ctrl_config imx117_custom_ctrls[] = {
    {
        .ops    = &imx117_ctrl_ops,
        .id     = V4L2_CID_CAMERA_TEST_MODE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "TestMode",
        .min    = 0,
        .max    = 2,
        .def    = 0,
        .step   = 1,
    },
};

static int imx117_initialize_ctrls(struct imx117_info *priv)
{
    v4l2_ctrl_handler_init(&priv->handler, NUM_CTRLS);

    /* standard */
    priv->gain = v4l2_ctrl_new_std(&priv->handler, &imx117_ctrl_ops,
            V4L2_CID_GAIN, 0, 20000, 1, 200);
    if (!priv->gain) {
        pr_err("%s: failed to create gain ctrl\n", __func__);
        return -1;
    }
    priv->exposure = v4l2_ctrl_new_std(&priv->handler, &imx117_ctrl_ops,
            V4L2_CID_EXPOSURE_ABSOLUTE, 10, 4000, 1, 10);
    if (!priv->exposure) {
        pr_err("%s: failed to create exposure ctrl\n", __func__);
        return -1;
    }

    /* custom */
    priv->test_mode = v4l2_ctrl_new_custom(&priv->handler, &imx117_custom_ctrls[0], NULL);
    if (!priv->test_mode) {
        pr_err("%s: failed to create test_mode ctrl\n", __func__);
        return -1;
    }

    priv->subdev.ctrl_handler = &priv->handler;
    if (priv->handler.error) {
        pr_err("%s: ctrl handler error(%d)\n", __func__, priv->handler.error);
        v4l2_ctrl_handler_free(&priv->handler);
        return -1;
    }
    return 0;
}

static int imx117_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_2_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static struct v4l2_subdev_video_ops imx117_subdev_video_ops = {
	.s_stream	= imx117_s_stream,
	.s_mbus_fmt	= imx117_s_fmt,
	.g_mbus_fmt	= imx117_g_fmt,
	.try_mbus_fmt	= imx117_try_fmt,
	.enum_mbus_fmt	= imx117_enum_fmt,
	.g_crop		= imx117_g_crop,
	.cropcap	= imx117_cropcap,
	.g_mbus_config	= imx117_g_mbus_config,
};

static struct v4l2_subdev_core_ops imx117_subdev_core_ops = {
	.s_power	= imx117_set_power,
	.s_ctrl     = v4l2_subdev_s_ctrl,
};

static struct v4l2_subdev_ops imx117_subdev_ops = {
	.core	= &imx117_subdev_core_ops,
	.video	= &imx117_subdev_video_ops,
};

static int imx117_video_probe(struct spi_device *spi)
{
	// We don't have method yet probe imx117
	// Just do initialize Power On Sequence.
	write_regs_table(spi, power_on_seq, sizeof(power_on_seq) / sizeof(struct sony_registers));
                           
	return 0;
}

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

static const struct media_entity_operations imx117_media_ops = {
	.link_setup = _link_setup,
};

static int imx117_probe(struct spi_device *spi)
{
	struct imx117_info *priv;
	struct v4l2_subdev *sd;
	int ret;

	printk("imx117 probe\n");
	priv = kzalloc(sizeof(struct imx117_info), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_spi_subdev_init(&priv->subdev, spi, &imx117_subdev_ops);
	sd = &priv->subdev;
	priv->spi = spi;

	priv->fmt	= &imx117_colour_fmts[0];
	priv->cur_readout_mode = 2; /* start with readout mode 2 */

	ret = imx117_video_probe(spi);
	if (ret < 0) {
		kfree(priv);
		return ret;
	}
	
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.ops = &imx117_media_ops;
	if (media_entity_init(&sd->entity, 1, &priv->pad, 0)) {
		dev_err(&spi->dev, "%s: failed to media_entity_init()\n", __func__);
		kfree(priv);
		return -ENOENT;
	}
	
	ret = imx117_initialize_ctrls(priv);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed to initialize controls\n", __func__);
		kfree(priv);
	}

	return ret;
}

static int imx117_remove(struct spi_device *spi)
{
	struct v4l2_subdev *sd = spi_get_drvdata(spi);
	struct imx117_info *priv = to_imx117_info(sd);

	kfree(priv);

	return 0;
}

static struct spi_driver imx117_spi_driver = {
	.driver = {
		.name	= "imx117",
		.owner	= THIS_MODULE,
	},
	.probe	= imx117_probe,
	.remove	= __devexit_p(imx117_remove),
};

module_spi_driver(imx117_spi_driver);

MODULE_DESCRIPTION("Sony IMX117 Camera driver");
MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_LICENSE("GPL v2");

