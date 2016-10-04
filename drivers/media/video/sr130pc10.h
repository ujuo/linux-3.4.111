/*
 * Driver Register File for SR130PC10(1.3MP Camera) from SILICONFILE
 *
 * Copyright (C) 2011,
 * DongSeong Lim<dongseong.lim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __SR130PC10_H
#define __SR130PC10_H

#include <linux/types.h>
#include <linux/regulator/machine.h>

#define SR130PC10_DRIVER_NAME	"SR130PC10"

typedef enum {
	STREAM_STOP,
	STREAM_START,
} stream_cmd_t;

struct sr130pc10_framesize {
	u32 width;
	u32 height;
};

struct sr130pc10_exif {
	u32 shutter_speed;
	u16 iso;
};

enum sr130pc10_sensor_mode {
        SENSOR_CAPTURE,
        SENSOR_PREVIEW,
};

/*
 * Driver information
 */
struct sr130pc10_state {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler handler;
	/* standard */
	struct v4l2_ctrl *ctl_exposure_auto;
	
	/* custom */
	struct v4l2_ctrl *ctl_preview_onoff;
	struct v4l2_ctrl *ctl_capture;
	//struct v4l2_ctrl *ctl_vt_mode;
	struct v4l2_ctrl *ctl_check_dataline;
	//struct v4l2_ctrl *ctl_sensor_mode;
	struct v4l2_ctrl *ctl_check_dataline_stop;
	struct v4l2_ctrl *ctl_frame_rate;
	struct v4l2_ctrl *ctl_brightness;

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
	//struct sr130pc10_framesize preview_frmsizes;
	//struct sr130pc10_framesize capture_frmsizes;
	struct sr130pc10_exif exif;

	//enum sr130pc10_sensor_mode sensor_mode;
	//s32 vt_mode;
	s32 check_dataline;
	u32 req_fps;
	u32 set_fps;
	u32 initialized;
};

static inline struct sr130pc10_state *to_state(struct v4l2_subdev *sd) {
	return container_of(sd, struct sr130pc10_state, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct sr130pc10_state, handler)->sd;
}

/* #define CONFIG_CAM_DEBUG */
#define cam_warn(fmt, ...)	\
	do { \
		printk(KERN_WARNING "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define cam_err(fmt, ...)	\
	do { \
		printk(KERN_ERR "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define cam_info(fmt, ...)	\
	do { \
		printk(KERN_INFO "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#ifdef CONFIG_CAM_DEBUG
#define cam_dbg(fmt, ...)	\
	do { \
		printk(KERN_DEBUG "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)
#else
#define cam_dbg(fmt, ...)
#endif /* CONFIG_CAM_DEBUG */

enum v4l2_ev_mode {
	EV_MINUS_4	= -4,
	EV_MINUS_3	= -3,
	EV_MINUS_2	= -2,
	EV_MINUS_1	= -1,
	EV_DEFAULT	= 0,
	EV_PLUS_1	= 1,
	EV_PLUS_2	= 2,
	EV_PLUS_3	= 3,
	EV_PLUS_4	= 4,
	EV_MAX,
};

#define V4L2_CID_CAM_CAPTURE					(V4L2_CTRL_CLASS_CAMERA | 0x1001)
#define V4L2_CID_CAM_PREVIEW_ONOFF				(V4L2_CTRL_CLASS_CAMERA | 0x1002)
#define V4L2_CID_CAM_CHECK_DATALINE				(V4L2_CTRL_CLASS_CAMERA | 0x1003)
#define V4L2_CID_CAM_CHECK_DATALINE_STOP		(V4L2_CTRL_CLASS_CAMERA | 0x1004)
#define V4L2_CID_CAM_FRAME_RATE					(V4L2_CTRL_CLASS_CAMERA | 0x1005)
#define V4L2_CID_CAM_BRIGHTNESS					(V4L2_CTRL_CLASS_CAMERA | 0x1006)
#define V4L2_CID_CAMERA_EXIF_TV					(V4L2_CTRL_CLASS_CAMERA | 0x1007)
#define V4L2_CID_CAMERA_EXIF_ISO				(V4L2_CTRL_CLASS_CAMERA | 0x1008)

/************ driver feature ************/
#define SR130PC10_USLEEP
/* #define CONFIG_LOAD_FILE */

#include  "sr130pc10_setfile.h"

#endif /* __SR130PC10_H */
