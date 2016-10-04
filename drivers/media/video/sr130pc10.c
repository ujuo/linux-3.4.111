/*
 * Driver for SR130PC10 from Samsung Electronics
 * Adapted for Nexell's NXP4330q device.
 *
 * Copyright(c) 2015 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 *  1.3Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/sr130pc10_platform.h>

#include "sr130pc10.h"

#define DELAY_SEQ               0xFF

#define INIT_NUM_OF_REGS                (sizeof(front_init_regs) / sizeof(regs_short_t))
#define INIT_VT_NUM_OF_REGS             (sizeof(front_init_vt_regs) / sizeof(regs_short_t))
#define PREVIEW_CAMERA_NUM_OF_REGS      (sizeof(front_preview_camera_regs) / sizeof(regs_short_t))
#define SNAPSHOT_NORMAL_NUM_OF_REGS     (sizeof(front_snapshot_normal_regs) / sizeof(regs_short_t))

#define WB_AUTO_NUM_OF_REGS	            (sizeof(front_wb_auto_regs) / sizeof(regs_short_t))
#define WB_SUNNY_NUM_OF_REGS	        (sizeof(front_wb_sunny_regs) / sizeof(regs_short_t))
#define WB_CLOUDY_NUM_OF_REGS	        (sizeof(front_wb_cloudy_regs) / sizeof(regs_short_t))
#define WB_TUNSTEN_NUM_OF_REGS	        (sizeof(front_wb_tungsten_regs) / sizeof(regs_short_t))
#define WB_FLUORESCENT_NUM_OF_REGS	    (sizeof(front_wb_fluorescent_regs) / sizeof(regs_short_t))

#define EFFECT_NORMAL_NUM_OF_REGS	    (sizeof(front_effect_normal_regs) / sizeof(regs_short_t))
#define EFFECT_NEGATIVE_NUM_OF_REGS	    (sizeof(front_effect_negative_regs) / sizeof(regs_short_t))
#define EFFECT_SEPIA_NUM_OF_REGS	    (sizeof(front_effect_sepia_regs) / sizeof(regs_short_t))
#define EFFECT_MONO_NUM_OF_REGS         (sizeof(front_effect_mono_regs) / sizeof(regs_short_t))

#define EV_M4_NUM_OF_REGS	            (sizeof(front_ev_minus_4_regs) / sizeof(regs_short_t))
#define EV_M3_NUM_OF_REGS	            (sizeof(front_ev_minus_3_regs) / sizeof(regs_short_t))
#define EV_M2_NUM_OF_REGS	            (sizeof(front_ev_minus_2_regs) / sizeof(regs_short_t))
#define EV_M1_NUM_OF_REGS	            (sizeof(front_ev_minus_1_regs) / sizeof(regs_short_t))
#define EV_DEFAULT_NUM_OF_REGS	        (sizeof(front_ev_default_regs) / sizeof(regs_short_t))
#define EV_P1_NUM_OF_REGS	            (sizeof(front_ev_plus_1_regs) / sizeof(regs_short_t))
#define EV_P2_NUM_OF_REGS	            (sizeof(front_ev_plus_2_regs) / sizeof(regs_short_t))
#define EV_P3_NUM_OF_REGS	            (sizeof(front_ev_plus_3_regs) / sizeof(regs_short_t))
#define EV_P4_NUM_OF_REGS	            (sizeof(front_ev_plus_4_regs) / sizeof(regs_short_t))

#define EV_VT_M4_NUM_OF_REGS	        (sizeof(front_ev_vt_minus_4_regs) / sizeof(regs_short_t))
#define EV_VT_M3_NUM_OF_REGS	        (sizeof(front_ev_vt_minus_3_regs) / sizeof(regs_short_t))
#define EV_VT_M2_NUM_OF_REGS	        (sizeof(front_ev_vt_minus_2_regs) / sizeof(regs_short_t))
#define EV_VT_M1_NUM_OF_REGS	        (sizeof(front_ev_vt_minus_1_regs) / sizeof(regs_short_t))
#define EV_VT_DEFAULT_NUM_OF_REGS	    (sizeof(front_ev_vt_default_regs) / sizeof(regs_short_t))
#define EV_VT_P1_NUM_OF_REGS	        (sizeof(front_ev_vt_plus_1_regs) / sizeof(regs_short_t))
#define EV_VT_P2_NUM_OF_REGS	        (sizeof(front_ev_vt_plus_2_regs) / sizeof(regs_short_t))
#define EV_VT_P3_NUM_OF_REGS	        (sizeof(front_ev_vt_plus_3_regs) / sizeof(regs_short_t))
#define EV_VT_P4_NUM_OF_REGS	        (sizeof(front_ev_vt_plus_4_regs) / sizeof(regs_short_t))

#define FPS_AUTO_NUM_OF_REGS	        (sizeof(front_fps_auto_regs) / sizeof(regs_short_t))
#define FPS_7_NUM_OF_REGS	            (sizeof(front_fps_7_regs) / sizeof(regs_short_t))
#define FPS_10_NUM_OF_REGS	            (sizeof(front_fps_10_regs) / sizeof(regs_short_t))
#define FPS_15_NUM_OF_REGS	            (sizeof(front_fps_15_regs) / sizeof(regs_short_t))

#define FPS_VT_AUTO_NUM_OF_REGS	        (sizeof(front_fps_vt_auto_regs) / sizeof(regs_short_t))
#define FPS_VT_7_NUM_OF_REGS	        (sizeof(front_fps_vt_7_regs) / sizeof(regs_short_t))
#define FPS_VT_10_NUM_OF_REGS	        (sizeof(front_fps_vt_10_regs) / sizeof(regs_short_t))
#define FPS_VT_15_NUM_OF_REGS	        (sizeof(front_fps_vt_15_regs) / sizeof(regs_short_t))

#define PATTERN_ON_NUM_OF_REGS	        (sizeof(front_pattern_on_regs) / sizeof(regs_short_t))
#define PATTERN_OFF_NUM_OF_REGS	        (sizeof(front_pattern_off_regs) / sizeof(regs_short_t))

static int sr130pc10_i2c_read_byte(struct i2c_client *client,
							unsigned char subaddr,
							unsigned char *data);

static int sr130pc10_i2c_write_byte(struct i2c_client *client,
							unsigned char subaddr,
							unsigned char data);

static int sr130pc10_i2c_set_data_burst(struct i2c_client *client,
								regs_short_t reg_buffer[],
								int num_of_regs);

static int sr130pc10_i2c_set_config_register(struct i2c_client *client,
									regs_short_t reg_buffer[],
									int num_of_regs,
									char *name);

#ifdef CONFIG_LOAD_FILE
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#define MAX_REG_TABLE_LEN 3500
#define MAX_ONE_LINE_LEN 500

typedef struct
{
    char name[100];
    char *location_ptr;
} reg_hash_t;

static char *front_regs_buf_ptr = NULL;
static char *front_curr_pos_ptr = NULL;
static char front_current_line[MAX_ONE_LINE_LEN];
static regs_short_t front_reg_table[MAX_REG_TABLE_LEN];
static int front_reg_num_of_element = 0;

static reg_hash_t front_reg_hash_table[] =
{
	{"front_init_regs",                   NULL},
	{"front_init_vt_regs",              NULL},
	{"front_preview_camera_regs",         NULL},
	{"front_snapshot_normal_regs",        NULL},
	{"front_ev_minus_4_regs",             NULL},
	{"front_ev_minus_3_regs",             NULL},
	{"front_ev_minus_2_regs",             NULL},
	{"front_ev_minus_1_regs",             NULL},
	{"front_ev_default_regs",             NULL},
	{"front_ev_plus_1_regs",              NULL},
	{"front_ev_plus_2_regs",              NULL},
	{"front_ev_plus_3_regs",              NULL},
	{"front_ev_plus_4_regs",              NULL},
	{"front_ev_vt_minus_4_regs",          NULL},
	{"front_ev_vt_minus_3_regs",          NULL},
	{"front_ev_vt_minus_2_regs",          NULL},
	{"front_ev_vt_minus_1_regs",          NULL},
	{"front_ev_vt_default_regs",          NULL},
	{"front_ev_vt_plus_1_regs",           NULL},
	{"front_ev_vt_plus_2_regs",           NULL},
	{"front_ev_vt_plus_3_regs",           NULL},
	{"front_ev_vt_plus_4_regs",           NULL},
	{"front_wb_auto_regs",                NULL},
	{"front_wb_sunny_regs",               NULL},
	{"front_wb_cloudy_regs",              NULL},
	{"front_wb_tungsten_regs",            NULL},
	{"front_wb_fluorescent_regs",         NULL},
	{"front_effect_normal_regs",          NULL},
	{"front_effect_negative_regs",        NULL},
	{"front_effect_sepia_regs",           NULL},
	{"front_effect_mono_regs",            NULL},
	{"front_fps_auto_regs",               NULL},
	{"front_fps_7_regs",                  NULL},
	{"front_fps_10_regs",                 NULL},
	{"front_fps_15_regs",                 NULL},
	{"front_fps_vt_auto_regs",            NULL},
	{"front_fps_vt_7_regs",               NULL},
	{"front_fps_vt_10_regs",              NULL},
	{"front_fps_vt_15_regs",              NULL},
	{"front_pattern_on_regs",             NULL},
	{"front_pattern_off_regs",            NULL},
};

static bool sr130pc10_regs_get_line(char *line_buf)
{
	int i;
	char *r_n_ptr = NULL;

	memset(line_buf, 0, MAX_ONE_LINE_LEN);

	r_n_ptr = strstr(front_curr_pos_ptr, "\n");

	if(r_n_ptr ) {
		for(i = 0; i < MAX_ONE_LINE_LEN; i++) {
			if(front_curr_pos_ptr + i == r_n_ptr) {
				front_curr_pos_ptr = r_n_ptr + 1;
				break;
			}
			line_buf[i] = front_curr_pos_ptr[i];
		}
		line_buf[i] = '\0';

		return true;
	} else {
		if(strlen(front_curr_pos_ptr) > 0) {
			strcpy(line_buf, front_curr_pos_ptr);
			return true;
		} else {
			return false;
		}
	}
}

static bool sr130pc10_regs_trim(char *line_buf)
{
	int left_index;
	int buff_len;
	int i;

	buff_len = strlen(line_buf);
	left_index  = -1;

	if(buff_len == 0)
		return false;

	/* Find the first letter that is not a white space from left side */
	for(i = 0; i < buff_len; i++) {
		if((line_buf[i] != ' ') && (line_buf[i] != '\t') && (line_buf[i] != '\n') && (line_buf[i] != '\r'))	{
			left_index = i;
			break;
		}
	}

	if(left_index == -1) {
		return false;
	}

	if((line_buf[left_index] == '\0') || ((line_buf[left_index] == '/') && (line_buf[left_index + 1] == '/')))	{
		return false;
	}

	if(left_index != 0)	{
		strcpy(line_buf, line_buf + left_index);
	}

	return true;
}

static int sr130pc10_regs_parse_table(void)
{
#if 0 /* Parsing a register format : {0x0000, 0x0000}, */
	char reg_buf[7], data_buf[7];
	int reg_index = 0;
	reg_buf[6] = '\0';
	data_buf[6] = '\0';
    while(sr130pc10_regs_get_line(front_current_line))
    {
        if(sr130pc10_regs_trim(front_current_line) == false)
        {
            continue;
        }
        /* Check End line of a table.*/
        if((front_current_line[0] == '}') && (front_current_line[1] == ';'))
        {
            break;
        }
        /* Parsing a register format : {0x0000, 0x0000},*/
        if((front_current_line[0] == '{') && (front_current_line[1] == '0') && (front_current_line[15] == '}'))
        {
            memcpy(reg_buf, (const void *)&front_current_line[1], 6);
            memcpy(data_buf, (const void *)&front_current_line[9], 6);
            front_reg_table[reg_index].subaddr = (unsigned short)simple_strtoul(reg_buf, NULL, 16);
            front_reg_table[reg_index].value = (unsigned int)simple_strtoul(data_buf, NULL, 16);
            reg_index++;
        }
    }
#else /* Parsing a register format : {0x00, 0x00}, */

	char reg_buf[5], data_buf[5];
	int reg_index = 0;

	reg_buf[4] = '\0';
	data_buf[4] = '\0';

	while(sr130pc10_regs_get_line(front_current_line)) {
		if(sr130pc10_regs_trim(front_current_line) == false)
			continue;

	/* Check End line of a table.*/
	if((front_current_line[0] == '}') && (front_current_line[1] == ';'))
		break;

	/* Parsing a register format : {0x00, 0x00},*/
	if((front_current_line[0] == '{') && (front_current_line[1] == '0') && (front_current_line[11] == '}'))	{
		memcpy(reg_buf, (const void *)&front_current_line[1], 4);
		memcpy(data_buf, (const void *)&front_current_line[7], 4);

		front_reg_table[reg_index].subaddr = (unsigned short)simple_strtoul(reg_buf, NULL, 16);
		front_reg_table[reg_index].value = (unsigned int)simple_strtoul(data_buf, NULL, 16);

	reg_index++;
	}
	}
#endif

    return reg_index;
}

static int sr130pc10_regs_table_write(struct i2c_client *client, char *name)
{
	bool bFound_table = false;
	int i, err = 0;

	front_reg_num_of_element = 0;

	for(i = 0; i < sizeof(front_reg_hash_table)/sizeof(reg_hash_t); i++) {
		if(strcmp(name, front_reg_hash_table[i].name) == 0) {
			bFound_table = true;

			front_curr_pos_ptr = front_reg_hash_table[i].location_ptr;
			break;
		}
	}

	if(bFound_table) {
		front_reg_num_of_element = sr130pc10_regs_parse_table();
	} else {
		cam_err("%s doesn't exist\n", name);
		return -EIO;
	}

	err = sr130pc10_i2c_set_data_burst(client, front_reg_table, front_reg_num_of_element);
	if(err < 0) {
		cam_err(" ERROR! sr130pc10_i2c_set_data_burst failed\n");
		return -EIO;
	}

	return err;
}

int sr130pc10_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret, i, retry_cnt;
	mm_segment_t fs = get_fs();
	char *location_ptr = NULL;
	bool bFound_name;

	cam_dbg("E\n");

	set_fs(get_ds());

	filp = filp_open("/mnt/sdcard/external_sd/sr130pc10_setfile.h", O_RDONLY, 0);
	if(IS_ERR(filp)) {
		cam_err("file open error\n");
		return -EIO;
	}
	l = filp->f_path.dentry->d_inode->i_size;
	cam_info("file size = %ld\n",l);

	//msleep(50);
	cam_dbg("Start vmalloc\n");
	for(retry_cnt = 5; retry_cnt > 0; retry_cnt--) {
		dp = kmalloc(l, GFP_KERNEL);
		if(dp != NULL)
			break;

		msleep(50);
	}
	if(dp == NULL) {
		cam_err("Out of Memory\n");
		filp_close(filp, current->files);
		return -ENOMEM;
	}
	cam_dbg("End vmalloc\n");

	pos = 0;
	memset(dp, 0, l);
	cam_dbg("Start vfs_read\n");
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if(ret != l) {
		cam_err("Failed to read file\n");
		vfree(dp);
		filp_close(filp, current->files);
		return -EINVAL;
	}
	cam_dbg("End vfs_read\n");

	filp_close(filp, current->files);

	set_fs(fs);

	front_regs_buf_ptr = dp;

	*((front_regs_buf_ptr + l) - 1) = '\0';

	/* Make hash table to enhance speed.*/
	front_curr_pos_ptr = front_regs_buf_ptr;
	location_ptr = front_curr_pos_ptr;

	for(i = 0; i < sizeof(front_reg_hash_table)/sizeof(reg_hash_t); i++) {
		front_reg_hash_table[i].location_ptr = NULL;
		bFound_name = false;

		while(sr130pc10_regs_get_line(front_current_line)) {
			if(strstr(front_current_line, front_reg_hash_table[i].name) != NULL) {
				bFound_name = true;
				front_reg_hash_table[i].location_ptr = location_ptr;
				break;
			}
			location_ptr = front_curr_pos_ptr;
		}

		if(bFound_name == false) {
			if(i == 0)	{
				cam_err(" ERROR! Couldn't find the reg name in hash table\n");
				return -EIO;
			} else {
				front_curr_pos_ptr = front_reg_hash_table[i-1].location_ptr;
			}
			location_ptr = front_curr_pos_ptr;

			cam_err(" ERROR! Couldn't find the reg name in hash table\n");
		}
	}

	cam_dbg("X\n");

	return 0;
}

void sr130pc10_regs_table_exit(void)
{
	cam_dbg(" start\n");

	if(front_regs_buf_ptr) {
		vfree(front_regs_buf_ptr);
		front_regs_buf_ptr = NULL;
	}

	cam_dbg(" done\n");
}
#endif

//SXGA: 1280*960
#define SXGA_WIDTH		1280
#define SXGA_HEIGHT		1024
#define P960_WIDTH		1280
#define P960_HEIGHT		960
#define P720_WIDTH 		1280
#define P720_HEIGHT		720
#define VGA_WIDTH		640
#define VGA_HEIGHT		480
#define QVGA_WIDTH		320
#define QVGA_HEIGHT		240

regs_short_t sr130pc10_sxga_regs[] = {
{0x03, 0x00},
{0x01, 0xf9},  //sleep
};

regs_short_t sr130pc10_p960_regs[] = {
{0x03, 0x00},
{0x01, 0xf9},  //sleep
};

regs_short_t sr130pc10_p720_regs[] = {
{0x03, 0x00},
{0x01, 0xf9},  //sleep
};

regs_short_t sr130pc10_vga_regs[] = {
{0x03, 0x00},
{0x01, 0xf9},  //sleep
};

regs_short_t sr130pc10_qvga_regs[] = {
{0x03, 0x00},
{0x01, 0xf9},  //sleep
};

static struct sr130pc10_win_size {
	int	width;
	int	height;
	regs_short_t *regs; /* Regs to tweak */
	int regs_size;
	int (*set_size) (struct v4l2_subdev *sd);
/* h/vref stuff */
} sr130pc10_win_sizes[] = {
	/* SXGA (1280X1024) */
	{
		.width			= SXGA_WIDTH,
		.height 		= SXGA_HEIGHT,
		.regs			= sr130pc10_sxga_regs,
		.regs_size	= ARRAY_SIZE(sr130pc10_sxga_regs),
		.set_size		= NULL,
	},
	/* 1280X960 */
	{
		.width			= P960_WIDTH,
		.height 		= P960_HEIGHT,
		.regs			= sr130pc10_p960_regs,
		.regs_size	= ARRAY_SIZE(sr130pc10_p960_regs),
		.set_size		= NULL,
	},
	/* 720p (1280X720) */
	{
		.width			= P720_WIDTH,
		.height			= P720_HEIGHT,
		.regs 			= sr130pc10_p720_regs,
		.regs_size	= ARRAY_SIZE(sr130pc10_p720_regs),
		.set_size		= NULL,
	},
	/* VGA (640X480) */
	{
		.width			= VGA_WIDTH,
		.height			= VGA_HEIGHT,
		.regs				= sr130pc10_vga_regs,
		.regs_size	= ARRAY_SIZE(sr130pc10_vga_regs),
		.set_size		= NULL,
	},
	/* QVGA (320X240) */
	{
		.width			= QVGA_WIDTH,
		.height			= QVGA_HEIGHT,
		.regs			= sr130pc10_qvga_regs,
		.regs_size	= ARRAY_SIZE(sr130pc10_qvga_regs),
		.set_size		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sr130pc10_win_sizes))

static int sr130pc10_i2c_read_byte(struct i2c_client *client,
                                     unsigned char subaddr,
                                     unsigned char *data)
{
	unsigned char buf[2] = {0,};
	struct i2c_msg msg = {client->addr, 0, 1, buf};
	int err = 0;

	if(!client->adapter) {
		cam_err("ERROR! can't search i2c client adapter\n");
		return -EIO;
	}

	buf[0] = (unsigned char)subaddr;

	err = i2c_transfer(client->adapter, &msg, 1);
	if(err < 0) {
		cam_err(" ERROR! %d register read failed\n",subaddr);
		return -EIO;
	}

	msg.flags = I2C_M_RD;
	msg.len = 1;

	err = i2c_transfer(client->adapter, &msg, 1);
	if(err < 0) {
		cam_err(" ERROR! %d register read failed\n",subaddr);
		return -EIO;
	}

	*data = (unsigned short)buf[0];

	return 0;
}

static int sr130pc10_i2c_write_byte(struct i2c_client *client,
                                     unsigned char subaddr,
                                     unsigned char data)
{
	unsigned char buf[2] = {0,};
	struct i2c_msg msg = {client->addr, 0, 2, buf};
	int err = 0;

	if(!client->adapter) {
		cam_err(" ERROR! can't search i2c client adapter\n");
		return -EIO;
	}

	buf[0] = subaddr & 0xFF;
	buf[1] = data & 0xFF;

	err = i2c_transfer(client->adapter, &msg, 1);

	return (err == 1)? 0 : -EIO;
}

static int sr130pc10_i2c_set_data_burst(struct i2c_client *client,
                                         regs_short_t reg_buffer[],
          				                 int num_of_regs)
{
	unsigned short subaddr, data_value;
	int i, err = 0;

	for(i = 0; i < num_of_regs; i++) {
		subaddr = reg_buffer[i].subaddr;
		data_value = reg_buffer[i].value;

		switch(subaddr) {
		case DELAY_SEQ:
			cam_dbg("delay = %d\n",data_value*10);
			msleep(data_value * 10);
			break;
		default:
			err = sr130pc10_i2c_write_byte(client, subaddr, data_value);
			if(err < 0) {
				cam_err("i2c write fail\n");
				return -EIO;
			}
			break;
		}
	}

	return 0;
}

static int sr130pc10_i2c_set_config_register(struct i2c_client *client,
                                         regs_short_t reg_buffer[],
          				                 int num_of_regs,
          				                 char *name)
{
	int err = 0;

	cam_err("sr130pc10_i2c_set_config_register E: %s, %d\n",  name, err);

#ifdef CONFIG_LOAD_FILE
	err = sr130pc10_regs_table_write(client, name);
#else
	err = sr130pc10_i2c_set_data_burst(client, reg_buffer, num_of_regs);
#endif

	cam_err("sr130pc10_i2c_set_config_register X: %s, %d\n",  name, err);

	return err;
}

/*!
 * sr130pc10 detect
 *
 * @return 0(OK) or -NODEV
 */
static int sr130pc10_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val;

	int ret;

	/* read id register and compare to correct value.. */
	sr130pc10_i2c_write_byte(client, 0x03, 0x00); // page 0
	ret = sr130pc10_i2c_read_byte(client, 0x04, &val); // device id register.
	printk("detect 0x0004 : %X\n", val);
	if (ret)
		return ret;

	if (val != 0x94)
		return -ENODEV;
	
	return 0;
}

static int sr130pc10_get_iso_speed_rate(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char read_value;
	int gain;
	int isospeedrating = 0;

	sr130pc10_i2c_write_byte(client, 0x03, 0x20);
	sr130pc10_i2c_read_byte(client, 0xb0, &read_value);
	gain = ((int)read_value * 100  / 32) + 50;

	if (read_value < 125)
		isospeedrating = 50;
	else if (read_value < 175)
		isospeedrating = 100;
	else if (read_value < 250)
		isospeedrating = 200;
	else if (read_value < 375)
		isospeedrating = 400;
	else if (read_value < 550)
		isospeedrating = 800;
	else
		isospeedrating = 1600;

	cam_dbg("get iso = %d\n", isospeedrating);
	return isospeedrating;
}

static int sr130pc10_get_shutterspeed(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char read_value;
	int cintr;
	int ShutterSpeed = 0;

	sr130pc10_i2c_write_byte(client, 0x03, 0x20);
	sr130pc10_i2c_read_byte(client, 0x80, &read_value);
	cintr = (int)read_value << 19;
	sr130pc10_i2c_read_byte(client, 0x81, &read_value);
	cintr += (int)read_value << 11;
	sr130pc10_i2c_read_byte(client, 0x82, &read_value);
	cintr += (int)read_value << 3;

	ShutterSpeed =  cintr / 24000;

	cam_dbg("get shutterspeed = %d\n", ShutterSpeed);
	return ShutterSpeed;
}

static int sr130pc10_get_exif(struct v4l2_subdev *sd)
{
	struct sr130pc10_state *state = to_state(sd);

	state->exif.shutter_speed = 100;
	state->exif.iso = 0;

	/* Get shutter speed */
	state->exif.shutter_speed = sr130pc10_get_shutterspeed(sd);

	/* Get ISO */
	state->exif.iso = sr130pc10_get_iso_speed_rate(sd);

	cam_dbg("Shutter speed=%d, ISO=%d\n",state->exif.shutter_speed, state->exif.iso);
	return 0;
}

static int sr130pc10_check_dataline(struct v4l2_subdev *sd, s32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int err = 0;

	cam_info("DTP %s\n", val ? "ON" : "OFF");

	if (val) {
		cam_dbg("load sr130pc10_pattern_on\n");
		err = sr130pc10_i2c_set_config_register(client,
										  front_pattern_on_regs,
										  PATTERN_ON_NUM_OF_REGS,
										  "front_pattern_on_regs");
	} else {
		cam_dbg("load sr130pc10_pattern_off\n");
		err = sr130pc10_i2c_set_config_register(client,
										  front_pattern_off_regs,
										  PATTERN_OFF_NUM_OF_REGS,
										  "front_pattern_off_regs");
	}
	if (unlikely(err)) {
		cam_err("fail to DTP setting\n");
		return err;
	}

	return 0;
}

static int sr130pc10_set_preview_start(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc10_state *state = to_state(sd);
	int err = -EINVAL;

	cam_info("reset preview\n");

	cam_dbg("load sr130pc10_preview\n");
	err = sr130pc10_i2c_set_config_register(client,
									front_preview_camera_regs,
									PREVIEW_CAMERA_NUM_OF_REGS,
									"front_preview_camera_regs");
	if (state->check_dataline)
		err = sr130pc10_check_dataline(sd, 1);
	if (unlikely(err)) {
		cam_err("fail to make preview\n");
		return err;
	}

	if (state->ctl_preview_onoff->val == SENSOR_PREVIEW) {
		cam_info("load recording setting\n");
		err = sr130pc10_i2c_set_config_register(client,
										  front_fps_15_regs,
										  FPS_15_NUM_OF_REGS,
										  "front_fps_15_regs");
	}
	if (unlikely(err)) {
		cam_err("failed to recording setting\n");
		return err;
	}

	return 0;
}

static int sr130pc10_set_preview_stop(struct v4l2_subdev *sd)
{
	int err = 0;
	cam_info("do nothing.\n");

	return err;
}

static int sr130pc10_set_capture_start(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;

	/* set initial regster value */
	cam_dbg("load sr130pc10_capture\n");
	err = sr130pc10_i2c_set_config_register(client,
									front_snapshot_normal_regs,
									SNAPSHOT_NORMAL_NUM_OF_REGS,
									"front_snapshot_normal_regs");
	if (unlikely(err)) {
		cam_err("failed to make capture\n");
		return err;
	}
	sr130pc10_get_exif(sd);
	cam_info("Capture ConfigSync\n");
	return err;
}

/*
static int sr130pc10_set_sensor_mode(struct v4l2_subdev *sd,
					int value)
{
	struct sr130pc10_state *state = to_state(sd);

	// 1 for preview, 0 for capture
	if ((value != 0) && (value != 1)) {
		cam_err("ERR: Not support.(%d)\n", value);
		return -EINVAL;
	}

	if ((state->cur_fmt.width > 640) && (value == 1)) {
		cam_err("ERR: Not supported in High-res mode.(%d)\n", value);
		return -EINVAL;
	}
	// Register setting ? 
	if (value != state->sensor_mode) {
		// Now Preview -> Capture or Capture ->Preview setting
		if (value == 0) {
			// Preview to Capture
		} else {
			// Capture to Preview
		}
	}

	return 0;
}
*/
static int sr130pc10_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	cam_dbg("E\n");
	return 0;
}

static int sr130pc10_enum_framesizes(struct v4l2_subdev *sd, \
					struct v4l2_frmsizeenum *fsize)
{
	struct sr130pc10_state *state = to_state(sd);

	cam_dbg("E\n");

	/*
	 * Return the actual output settings programmed to the camera
	 */
	

	cam_info("width - %d , height - %d\n",
		fsize->discrete.width, fsize->discrete.height);

	return 0;
}

static int sr130pc10_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	cam_dbg("E\n");

	return err;
}

static int sr130pc10_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	struct sr130pc10_state *state = to_state(sd);
	u32 *width = NULL, *height = NULL;

	cam_dbg("E\n");
	/*
	 * Do nothing for a while.(ksw)
	 */
	return 0;
}

static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt,//linux-3.0
		struct sr130pc10_win_size **ret_wsize)
{
	struct sr130pc10_win_size *wsize;

	pr_debug("%s enter\n", __func__);
		
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0
	
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sr130pc10_win_sizes; wsize < sr130pc10_win_sizes + N_WIN_SIZES; wsize++) {
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)//linux-3.0
			break;
	}
	
	if (wsize >= sr130pc10_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;//linux-3.0
	fmt->height = wsize->height;//linux-3.0
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;//linux-3.0
	//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0
	
	return 0;
}

static int sr130pc10_set_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	int err = 0;
	struct v4l2_mbus_framefmt *_fmt = &fmt->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc10_state *state = to_state(sd);
	struct sr130pc10_win_size *wsize;

	pr_debug("%s: %dx%d\n", __func__, _fmt->width, _fmt->height);

	state->req_fmt.width = _fmt->width;
	state->req_fmt.height = _fmt->height;
	
	printk("%s: %dx%d\n", __func__, state->req_fmt.width, state->req_fmt.height);
	
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	err = sensor_try_fmt_internal(sd, _fmt, &wsize);
	if (err)
		return err;
	
	err = 0;
	if (wsize->regs)
	{
		err = sr130pc10_i2c_set_data_burst(client, wsize->regs , wsize->regs_size);
		if (err < 0)
			return err;
	}

	return err;
}

static int sr130pc10_set_frame_rate(struct v4l2_subdev *sd, u32 fps)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	cam_info("frame rate %d\n\n", fps);

	switch (fps) {
	case 7:
		cam_dbg("load front_fps_vt_7_regs\n");
		err = sr130pc10_i2c_set_config_register(client,
										  front_fps_vt_7_regs,
										  FPS_VT_7_NUM_OF_REGS,
										  "front_fps_vt_7_regs");
		break;
	case 10:
		cam_dbg("load front_fps_vt_10_regs\n");
		err = sr130pc10_i2c_set_config_register(client,
										  front_fps_vt_10_regs,
										  FPS_VT_10_NUM_OF_REGS,
										  "front_fps_vt_10_regs");
		break;
	case 15:
		cam_dbg("load front_fps_vt_15_regs\n");
		err = sr130pc10_i2c_set_config_register(client,
										  front_fps_vt_15_regs,
										  FPS_VT_15_NUM_OF_REGS,
										  "front_fps_vt_15_regs");
		break;
	default:
		err = sr130pc10_i2c_set_config_register(client,
										  front_fps_auto_regs,
										  FPS_AUTO_NUM_OF_REGS,
										  "front_fps_auto_regs");
		break;
	}
	if (unlikely(err < 0)) {
		cam_err("i2c_write for set framerate\n");
		return -EIO;
	}

	return err;
}

static int sr130pc10_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	int err = 0;

	cam_dbg("E\n");

	return err;
}

static int sr130pc10_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	int err = 0;
	u32 fps = 0;
	struct sr130pc10_state *state = to_state(sd);

	cam_dbg("E\n");

	fps = parms->parm.capture.timeperframe.denominator /
			parms->parm.capture.timeperframe.numerator;

	if (fps != state->set_fps) {
		if (fps < 0 && fps > 15) {
			cam_err("invalid frame rate %d\n", fps);
			fps = 15;
		}
		state->req_fps = fps;

		if (state->initialized) {
			err = sr130pc10_set_frame_rate(sd, state->req_fps);
			if (err >= 0)
				state->set_fps = state->req_fps;
		}

	}

	return err;
}

static int sr130pc10_control_stream(struct v4l2_subdev *sd, stream_cmd_t cmd)
{
	int err = 0;

	switch (cmd) {
	case STREAM_START:
		cam_warn("WARN: do nothing\n");
		break;

	case STREAM_STOP:
		cam_dbg("stream stop!!!\n");
		break;
	default:
		cam_err("ERR: Invalid cmd\n");
		break;
	}

	if (unlikely(err))
		cam_err("failed to stream start(stop)\n");

	return err;
}

static int sr130pc10_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc10_state *state = to_state(sd);
	int err = -EINVAL;

	cam_dbg("%s: enter\n", __func__);

	/* set initial regster value */
#ifdef CONFIG_LOAD_FILE
	err = sr130pc10_regs_table_init();
	if(err < 0)
	{
		cam_err("sr130pc10_regs_table_init failed\n");
		return -ENOIOCTLCMD;
	}
#endif

	{
		cam_info("load camera common setting\n");
		err = sr130pc10_i2c_set_config_register(client,
										  front_init_regs,
										  INIT_NUM_OF_REGS,
										  "front_init_regs");
	}

	if (unlikely(err)) {
		cam_err("failed to init\n");
		return err;
	}

	/* We stop stream-output from sensor when starting camera. */
	err = sr130pc10_control_stream(sd, STREAM_STOP);
	if (unlikely(err < 0))
		return err;
	msleep(150);

	if (state->req_fps != state->set_fps) {
		err = sr130pc10_set_frame_rate(sd, state->req_fps);
		if (unlikely(err < 0))
			return err;
		else
			state->set_fps = state->req_fps;
	}

	state->initialized = 1;

	return 0;
}

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize
 * every single opening time therefor,
 * it is not necessary to be initialized on probe time.
 * except for version checking
 * NOTE: version checking is optional
 */
static int sr130pc10_s_config(struct v4l2_subdev *sd,
		int irq, void *platform_data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc10_state *state = to_state(sd);
	struct sr130pc10_platform_data *pdata;

	cam_dbg("%s: enter", __func__);

	state->initialized = 0;
	state->req_fps = state->set_fps = 8;

	pdata = client->dev.platform_data;

	if (!pdata) {
		cam_err("no platform data\n");
		return -ENODEV;
	}

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */

	state->req_fmt.width = VGA_WIDTH;
	state->req_fmt.height = VGA_HEIGHT;
	if (!pdata->pixelformat)
		state->req_fmt.pixelformat = DEFAULT_FMT;
	else
		state->req_fmt.pixelformat = pdata->pixelformat;

	return 0;
}

static int sr130pc10_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sr130pc10_state *state = to_state(sd);
	/* struct i2c_client *client = v4l2_get_subdevdata(sd); */
	int err = 0;

	cam_info("stream mode = %d\n", enable);

	if (!enable) {
		if (state->ctl_check_dataline->val)
			err = sr130pc10_check_dataline(sd, 0);
		else
			err = sr130pc10_control_stream(sd, STREAM_STOP);
	} else {
		if (state->initialized == 0) {
			sr130pc10_init(sd, 0);
		}
		if (state->ctl_preview_onoff->val == 0)
			err = sr130pc10_set_capture_start(sd);
		else
			err = sr130pc10_set_preview_start(sd);
	}

	if (unlikely(err < 0)) {
		cam_err("ERR: faild\n");
		return err;
	}

	return 0;
}

static int sr130pc10_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct sr130pc10_state *state = to_state(sd);
	int err = 0;

	cam_dbg("ctrl->id : %d\n", ctrl->id - V4L2_CID_PRIVATE_BASE);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_EXIF_TV:
		ctrl->value = state->exif.shutter_speed;
		break;
	case V4L2_CID_CAMERA_EXIF_ISO:
		ctrl->value = state->exif.iso;
		break;
	default:
		cam_err("no such control id %d\n",
				ctrl->id - V4L2_CID_PRIVATE_BASE);
		break;
	}

	return err;
}

static int sr130pc10_set_brightness(struct v4l2_subdev *sd, int value) {
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc10_state *state = to_state(sd);
	int err = -EINVAL;

	cam_dbg("E\n");

	if (state->check_dataline)
		return 0;

	switch (value) {
	case EV_MINUS_4:
		cam_dbg("load sr130pc10_bright_m4\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_minus_4_regs,
										EV_M4_NUM_OF_REGS,
										"front_ev_minus_4_regs");
		break;
	case EV_MINUS_3:
		cam_dbg("load sr130pc10_bright_m3\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_minus_3_regs,
										EV_M3_NUM_OF_REGS,
										"front_ev_minus_3_regs");
		break;
	case EV_MINUS_2:
		cam_dbg("load sr130pc10_bright_m2\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_minus_2_regs,
										EV_M2_NUM_OF_REGS,
										"front_ev_minus_2_regs");
		break;
	case EV_MINUS_1:
		cam_dbg("load sr130pc10_bright_m1\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_minus_1_regs,
										EV_M1_NUM_OF_REGS,
										"front_ev_minus_1_regs");
		break;
	case EV_DEFAULT:
		cam_dbg("load sr130pc10_bright_default\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_default_regs,
										EV_DEFAULT_NUM_OF_REGS,
										"front_ev_default_regs");
		break;
	case EV_PLUS_1:
		cam_dbg("load sr130pc10_bright_p1\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_plus_1_regs,
										EV_P1_NUM_OF_REGS,
										"front_ev_plus_1_regs");
		break;
	case EV_PLUS_2:
		cam_dbg("load sr130pc10_bright_p2\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_plus_2_regs,
										EV_P2_NUM_OF_REGS,
										"front_ev_plus_2_regs");
		break;
	case EV_PLUS_3:
		cam_dbg("load sr130pc10_bright_p3\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_plus_3_regs,
										EV_P3_NUM_OF_REGS,
										"front_ev_plus_3_regs");
		break;
	case EV_PLUS_4:
		cam_dbg("load sr130pc10_bright_p4\n");
		err = sr130pc10_i2c_set_config_register(client,
										front_ev_plus_4_regs,
										EV_P4_NUM_OF_REGS,
										"front_ev_plus_4_regs");
		break;
	default:
		cam_err("ERR: invalid brightness(%d)\n", value);
		return err;
		break;
	}

	if (unlikely(err < 0)) {
		cam_err("ERR: i2c_write for set brightness\n");
		return -EIO;
	}

	return 0;
}

static int sr130pc10_check_dataline_stop(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc10_state *state = to_state(sd);
	int err = -EINVAL;

	extern int sr130pc10_power_reset(void);

	cam_warn("Warning: do nothing!!\n");
	return err;

	//sr130pc10_write(client, 0xFCFCD000);
	//sr130pc10_write(client, 0x0028D000);
	//sr130pc10_write(client, 0x002A3100);
	//sr130pc10_write(client, 0x0F120000);

	//	err =  sr130pc10_write_regs(sd, sr130pc10_pattern_off,	sizeof(sr130pc10_pattern_off) / sizeof(sr130pc10_pattern_off[0]));
	cam_dbg("sensor reset\n");
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined(CONFIG_TARGET_LOCALE_EUR) || defined(CONFIG_TARGET_LOCALE_HKTW) || defined(CONFIG_TARGET_LOCALE_HKTW_FET) || defined(CONFIG_TARGET_LOCALE_USAGSM)
        // dont't know where this code came from - comment out for compile error
        // sr130pc10_power_reset();
 #endif
	cam_dbg("load camera init setting\n");
	err = sr130pc10_i2c_set_config_register(client,
								   front_init_regs,
								   INIT_NUM_OF_REGS,
								   "front_init_regs");
	state->check_dataline = 0;
	/* mdelay(100); */
	return err;
}

static int sr130pc10_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sr130pc10_state *state = to_state(sd);
	int err = 0;
	
	v4l_info(client,"%s: ctrl->id:%d\n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->val) {
		} else {
		}
		break;
	case V4L2_CID_CAM_PREVIEW_ONOFF:
		if (ctrl->val)
			err = sr130pc10_set_preview_start(sd);
		else
			err = sr130pc10_set_preview_stop(sd);
		cam_dbg("V4L2_CID_CAM_PREVIEW_ONOFF [%d]\n", ctrl->value);
		break;

	case V4L2_CID_CAM_CAPTURE:
		err = sr130pc10_set_capture_start(sd);
		cam_dbg("V4L2_CID_CAM_CAPTURE\n");
		break;

	case V4L2_CID_CAM_BRIGHTNESS:
		err = sr130pc10_set_brightness(sd, ctrl->val);
		cam_dbg("V4L2_CID_CAM_BRIGHTNESS [%d]\n", ctrl->val);
		break;

	//case V4L2_CID_CAMERA_VT_MODE:
	//	state->vt_mode = ctrl->val;
	//	break;

	case V4L2_CID_CAM_CHECK_DATALINE:
		state->ctl_check_dataline = ctrl->val;
		cam_dbg("check_dataline = %d\n", state->ctl_check_dataline);
		err = 0;
		break;

	//case V4L2_CID_CAMERA_SENSOR_MODE:
	//	err = sr130pc10_set_sensor_mode(sd, ctrl->val);
	//	cam_dbg("sensor_mode = %d\n", ctrl->val);
	//	break;

	case V4L2_CID_CAM_CHECK_DATALINE_STOP:
		err = sr130pc10_check_dataline_stop(sd);
		break;

	case V4L2_CID_CAM_FRAME_RATE:
		cam_dbg("do nothing\n");
		break;

	default:
		cam_err("ERR(ENOIOCTLCMD)\n");
		/* no errors return.*/
		err = 0;
	}

	cam_dbg("X\n");
	return err;
}

static const struct v4l2_ctrl_ops sr130pc10_ctrl_ops = {
	.s_ctrl = sr130pc10_s_ctrl,
};

static const struct v4l2_ctrl_config sr130pc10_custom_ctrls[] = {
    {
        .ops    = &sr130pc10_ctrl_ops,
        .id     = V4L2_CID_CAM_PREVIEW_ONOFF,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "PreviewOnOff",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &sr130pc10_ctrl_ops,
        .id     = V4L2_CID_CAM_CAPTURE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "CaptureStarat",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    },{
        .ops    = &sr130pc10_ctrl_ops,
        .id     = V4L2_CID_CAM_CHECK_DATALINE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "CheckDataline",
        .min    = 0,
        .max    = 1024,
        .def    = 0,
        .step   = 1,
    },{
        .ops    = &sr130pc10_ctrl_ops,
        .id     = V4L2_CID_CAM_CHECK_DATALINE_STOP,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "CheckDatalineStop",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    },{
        .ops    = &sr130pc10_ctrl_ops,
        .id     = V4L2_CID_CAM_FRAME_RATE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "Framerate",
        .min    = 0,
        .max    = 15,
        .def    = 0,
        .step   = 1,
    },{
        .ops    = &sr130pc10_ctrl_ops,
        .id     = V4L2_CID_CAM_BRIGHTNESS,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "Brightness",
        .min    = -4,
        .max    = 4,
        .def    = 0,
        .step   = 1,
    },
};

#define NUM_CTRLS				7

static int sr130pc10_initialize_ctrls(struct sr130pc10_state *state)
{
    v4l2_ctrl_handler_init(&state->handler, NUM_CTRLS);

    state->ctl_exposure_auto = v4l2_ctrl_new_std_menu(&state->handler,  &sr130pc10_ctrl_ops,
    		V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
    if (!state->ctl_exposure_auto) {
        pr_err("%s: failed to create exposure_auto ctrl\n", __func__);
        return -1;
    }
    /* custom */
    state->ctl_preview_onoff = v4l2_ctrl_new_custom(&state->handler, &sr130pc10_custom_ctrls[0], NULL);
    if (!state->ctl_preview_onoff) {
    	pr_err("%s: failed to create preview onoff ctrl\n", __func__);
        return -1;
    }
    	
	state->ctl_capture = v4l2_ctrl_new_custom(&state->handler, &sr130pc10_custom_ctrls[1], NULL);
	if (!state->ctl_capture) {
    	pr_err("%s: failed to create capture ctrl\n", __func__);
        return -1;
    }

	state->ctl_check_dataline = v4l2_ctrl_new_custom(&state->handler, &sr130pc10_custom_ctrls[2], NULL);
	if (!state->ctl_check_dataline) {
    	pr_err("%s: failed to create check dataline ctrl\n", __func__);
        return -1;
    }

	state->ctl_check_dataline_stop = v4l2_ctrl_new_custom(&state->handler, &sr130pc10_custom_ctrls[3], NULL);
	if (!state->ctl_check_dataline_stop) {
    	pr_err("%s: failed to create dataline stop ctrl\n", __func__);
        return -1;
    }
	state->ctl_frame_rate = v4l2_ctrl_new_custom(&state->handler, &sr130pc10_custom_ctrls[4], NULL);
	if (!state->ctl_frame_rate) {
    	pr_err("%s: failed to create frame rate ctrl\n", __func__);
        return -1;
    }

    state->ctl_brightness = v4l2_ctrl_new_custom(&state->handler, &sr130pc10_custom_ctrls[5], NULL);
    if (!state->ctl_brightness) {
        pr_err("%s: failed to create brightness ctrl\n", __func__);
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

static int sr130pc10_s_power(struct v4l2_subdev *sd, int on)
{
	// Do nothing now...
	cam_dbg("%s: enter\n", __func__);
	return 0;
}

static const struct v4l2_subdev_core_ops sr130pc10_core_ops = {
	.s_ctrl = v4l2_subdev_s_ctrl,
	.s_power = sr130pc10_s_power,
	//.init = sr130pc10_init,		/* initializing API */
};

static const struct v4l2_subdev_video_ops sr130pc10_video_ops = {
	.s_stream = sr130pc10_s_stream,
};

static const struct v4l2_subdev_pad_ops sr130pc10_pad_ops = {
	.set_fmt  = sr130pc10_set_fmt,
	//.set_crop = sr130pc10_set_crop,
	//.get_crop = sr130pc10_get_crop,
};

static const struct v4l2_subdev_ops sr130pc10_ops = {
	.core = &sr130pc10_core_ops,
	.video = &sr130pc10_video_ops,
	.pad = &sr130pc10_pad_ops,
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

static const struct media_entity_operations sr130_media_ops = {
	.link_setup = _link_setup,
};

/*
 * sr130pc10_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int sr130pc10_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sr130pc10_state *state = NULL;
	struct v4l2_subdev *sd = NULL;
	int ret;

	state = kzalloc(sizeof(struct sr130pc10_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, SR130PC10_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &sr130pc10_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.ops = &sr130_media_ops;
	if (media_entity_init(&sd->entity, 1, &state->pad, 0)) {
		dev_err(&client->dev, "%s: failed to media_entity_init()\n", __func__);
		kfree(state);
		return -ENOENT;
	}

	ret = sr130pc10_initialize_ctrls(state);
	if (ret < 0) {
		pr_err("%s: failed to initialize controls\n", __func__);
		kfree(state);
		return ret;
	}
	if (sr130pc10_detect(sd) != 0) {
		kfree(state);
		return -ENODEV;	
	}

	dev_info(&client->dev, "sr130pc10 probed.\n");

	return 0;
}

static int sr130pc10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sr130pc10_state *state = to_state(sd);

	cam_dbg("E\n");

	state->initialized = 0;

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_entity_cleanup(&sd->entity);
	kfree(state);
	return 0;
}

static const struct i2c_device_id sr130pc10_id[] = {
	{ SR130PC10_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sr130pc10_id);

static struct i2c_driver sr130pc10_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name = SR130PC10_DRIVER_NAME,
  },
  .probe = sr130pc10_probe,
  .remove = sr130pc10_remove,
  .id_table = sr130pc10_id,
};

module_i2c_driver(sr130pc10_driver);

MODULE_DESCRIPTION("SR130PC10 ISP driver");
MODULE_AUTHOR("DongSeong Lim<dongseong.lim@samsung.com>");
MODULE_LICENSE("GPL");