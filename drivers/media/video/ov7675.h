/*!
 *    Copyright (c) 2014 STcube Inc.
 *    All right reserved. by Seungwoo Kim.<ksw@stcube.com>
 *
 *    Omnivision OV7675 sensor is supported.(Prelimary)
 *    
*/
#ifndef __OV7675_H__
#define __OV7675_H__

/*!
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/*! Registers */
#define OV7675_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define OV7675_BLUE	0x01	/* Blue gain */
#define OV7675_RED		0x02	/* Red gain */
#define OV7675_VREF	0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define OV7675_COM1	0x04	/* Control 1 */
#define  COM1_CCIR656	  0x40  /* CCIR656 enable */
#define OV7675_BAVE	0x05	/* U/B Average level */
#define OV7675_GbAVE	0x06	/* Y/Gb Average level */
#define OV7675_AECHH	0x07	/* AEC MS 5 bits */
#define OV7675_RAVE	0x08	/* V/R Average level */
#define OV7675_COM2	0x09	/* Control 2 */
#define  COM2_SSLEEP	  0x10	/* Soft sleep mode */
#define OV7675_PID		0x0a	/* Product ID MSB */
#define OV7675_VER		0x0b	/* Product ID LSB */
#define OV7675_COM3	0x0c	/* Control 3 */
#define  COM3_SWAP	  0x40	  /* Byte swap */
#define  COM3_SCALEEN	  0x08	  /* Enable scaling */
#define  COM3_DCWEN	  0x04	  /* Enable downsamp/crop/window */
#define OV7675_COM4	0x0d	/* Control 4 */
#define OV7675_COM5	0x0e	/* All "reserved" */
#define OV7675_COM6	0x0f	/* Control 6 */
#define OV7675_AECH	0x10	/* More bits of AEC value */
#define OV7675_CLKRC	0x11	/* Clocl control */
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define OV7675_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x38
#define   COM7_FMT_VGA	  0x00
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* bits 0 and 2 - RGB format */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define OV7675_COM8	0x13	/* Control 8 */
#define   COM8_FASTAEC	  0x80	  /* Enable fast AGC/AEC */
#define   COM8_AECSTEP	  0x40	  /* Unlimited AEC step size */
#define   COM8_BFILT	  0x20	  /* Band filter enable */
#define   COM8_AGC	  0x04	  /* Auto gain enable */
#define   COM8_AWB	  0x02	  /* White balance enable */
#define   COM8_AEC	  0x01	  /* Auto exposure enable */
#define OV7675_COM9	0x14	/* Control 9  - gain ceiling */
#define OV7675_COM10	0x15	/* Control 10 */
#define   COM10_HSYNC	  0x40	  /* HSYNC instead of HREF */
#define   COM10_PCLK_HB	  0x20	  /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08	  /* Reverse HREF */
#define   COM10_VS_LEAD	  0x04	  /* VSYNC on clock leading edge */
#define   COM10_VS_NEG	  0x02	  /* VSYNC negative */
#define   COM10_HS_NEG	  0x01	  /* HSYNC negative */
#define OV7675_HSTART	0x17	/* Horiz start high bits */
#define OV7675_HSTOP	0x18	/* Horiz stop high bits */
#define OV7675_VSTART	0x19	/* Vert start high bits */
#define OV7675_VSTOP	0x1a	/* Vert stop high bits */
#define OV7675_PSHFT	0x1b	/* Pixel delay after HREF */
#define OV7675_MIDH	0x1c	/* Manuf. ID high */
#define OV7675_MIDL	0x1d	/* Manuf. ID low */
#define OV7675_MVFP	0x1e	/* Mirror / vflip */
#define   MVFP_MIRROR	  0x20	  /* Mirror image */
#define   MVFP_FLIP	  0x10	  /* Vertical flip */

#define OV7675_AEW		0x24	/* AGC upper limit */
#define OV7675_AEB		0x25	/* AGC lower limit */
#define OV7675_VPT		0x26	/* AGC/AEC fast mode op region */
#define OV7675_EXHCH	0x2a
#define OV7675_EXHCL	0x2b
#define OV7675_ADVFL	0x2d
#define OV7675_ADVFH	0x2e
#define OV7675_HSYST	0x30	/* HSYNC rising edge delay */
#define OV7675_HSYEN	0x31	/* HSYNC falling edge delay */
#define OV7675_HREF	0x32	/* HREF pieces */
#define OV7675_TSLB	0x3a	/* lots of stuff */
#define   TSLB_YLAST	  0x04	  /* UYVY or VYUY - see com13 */
#define OV7675_COM11	0x3b	/* Control 11 */
#define   COM11_NIGHT	  0x80	  /* NIght mode enable */
#define   COM11_NMFR	  0x60	  /* Two bit NM frame rate */
#define   COM11_HZAUTO	  0x10	  /* Auto detect 50/60 Hz */
#define	  COM11_50HZ	  0x08	  /* Manual 50Hz select */
#define   COM11_EXP	  0x02
#define OV7675_COM12	0x3c	/* Control 12 */
#define   COM12_HREF	  0x80	  /* HREF always */
#define OV7675_COM13	0x3d	/* Control 13 */
#define   COM13_GAMMA	  0x80	  /* Gamma enable */
#define	  COM13_UVSAT	  0x40	  /* UV saturation auto adjustment */
#define   COM13_UVSWAP	  0x01	  /* V before U - w/TSLB */
#define OV7675_COM14	0x3e	/* Control 14 */
#define   COM14_DCWEN	  0x10	  /* DCW/PCLK-scale enable */
#define OV7675_EDGE	0x3f	/* Edge enhancement factor */
#define OV7675_COM15	0x40	/* Control 15 */
#define   COM15_R10F0	  0x00	  /* Data range 10 to F0 */
#define	  COM15_R01FE	  0x80	  /*            01 to FE */
#define   COM15_R00FF	  0xc0	  /*            00 to FF */
#define   COM15_RGB565	  0x10	  /* RGB565 output */
#define   COM15_RGB555	  0x30	  /* RGB555 output */
#define OV7675_COM16	0x41	/* Control 16 */
#define   COM16_AWBGAIN   0x08	  /* AWB gain enable */
#define OV7675_COM17	0x42	/* Control 17 */
#define   COM17_AECWIN	  0xc0	  /* AEC window - must match COM4 */
#define   COM17_CBAR	  0x08	  /* DSP Color bar */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	OV7675_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define OV7675_CMATRIX_SIGN 0x58


#define OV7675_BRIGHT	0x55	/* Brightness */
#define OV7675_CONTRAS	0x56	/* Contrast control */

#define OV7675_MANU	0x67	/* Fix gain control */
#define OV7675_MANV	0x68	/* Fix gain control */

#define OV7675_GFIX	0x69	/* Fix gain control */

#define OV7675_GREEN	0x6a	/* Green gain */

#define OV7675_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */

#define OV7675_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */

#define OV7675_DM_LNL	0x91
#define OV7675_DM_LNH	0x92

#define OV7675_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define OV7675_HAECC2	0xa0	/* Hist AEC/AGC control 2 */

#define OV7675_BD50MAX	0xa5	/* 50hz banding step limit */
#define OV7675_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define OV7675_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define OV7675_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define OV7675_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define OV7675_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define OV7675_BD60MAX	0xab	/* 60hz banding step limit */
#define OV7675_REGCA	0xca

#define OV7675_CLOCK_MIN	1500000
#define OV7675_CLOCK_12MHZ	12000000
#define OV7675_CLOCK_24MHZ	24000000
#define OV7675_CLOCK_MAX	27000000

#define OV7675_I2C_ADDR        (0x42 >> 1)

#endif
