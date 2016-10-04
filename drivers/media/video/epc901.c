/*! linux/drivers/media/video/epc901.c
 *
 * Espros EPC901 Line Scan Sensor driver
 * Copyright(c) 2015 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

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

#include "nx_vip.h"
#include "epc901.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct regval_list {
	unsigned short int reg_num;
	unsigned short int value;
};

struct epc901_state {
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

#ifdef USE_INITIAL_WORKER_THREAD
	struct workqueue_struct *init_wq;
	struct work_struct init_work;
#endif
};

static inline struct epc901_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct epc901_state, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct epc901_state, handler)->sd;
}


#define I2C_FLAG_READ	0x10

static int epc901_i2c_xfer(struct i2c_client *client, int reg, char *buf, int num, int tran_flag)
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

static int epc901_read_reg8(struct i2c_client *client, u16	reg, u8 *val)
{
	int ret;
	u8 rval;

	ret = epc901_i2c_xfer(client, reg, (u8 *) &rval, 1, I2C_FLAG_READ);
	if (0 == ret) {
		*val = rval;
		return 0;
	}
	return ret;
}

static int epc901_write_reg8(struct i2c_client *client,u16 reg, u8 val)
{
	return epc901_i2c_xfer(client, reg, (u8 *) & val, 1, 0);
}

static int epc901_write_reg8_array(struct i2c_client *client, struct regval_list *regs, int n)
{
	int i, ret;

	for (i=0; i<n; i++, regs++) {
		ret = epc901_write_reg8(client, regs->reg_num, regs->value);
		if (ret) break;
	}
	return ret;
}

/*
 * Stuff that knows about the sensor.
 */
static int epc901_reset(struct v4l2_subdev *sd, u32 val)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	/* software reset ? hmm, do nothing for now. */

	return 0;
}

static int epc901_init(struct v4l2_subdev *sd, u32 val)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	/* Currently do nothing. */

	return 0;
}

/*!
 * epc901 detect
 *
 * @return 0(OK) or -NODEV
 */
static int epc901_detect(struct v4l2_subdev *sd)
{
	/* Now do nothing, but later we would change this. */

	return 0;
}
 
static int epc901_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	// do nothing.

	return 0;
}

/*
 * Implement G/S_PARM.	There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int epc901_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}

static int epc901_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}


/*
 * Code for dealing with controls.
 */
static int epc901_s_sat(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int epc901_g_sat(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int epc901_s_hue(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int epc901_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int epc901_s_brightness(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int epc901_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int epc901_s_contrast(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int epc901_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int epc901_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int epc901_s_hflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int epc901_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int epc901_s_vflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int epc901_queryctrl(struct v4l2_subdev *sd,
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

static int epc901_subdev_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return epc901_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return epc901_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return epc901_g_sat(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return epc901_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return epc901_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return epc901_g_hflip(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int epc901_subdev_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return epc901_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return epc901_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return epc901_s_sat(sd, ctrl->value);
	case V4L2_CID_HUE:
		return epc901_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return epc901_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return epc901_s_hflip(sd, ctrl->value);
	}
	return -EINVAL;
}

static int epc901_s_ctrl(struct v4l2_ctrl *ctrl)
{
	// Do something...
	switch (ctrl->id) {
	default: ;
	}
	return -EINVAL;
}

static int epc901_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* EPC901 is not registered for V4L2-chip-ident... */
	epc901_detect(sd);
	
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int epc901_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val = 0;
	int ret;

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ret = epc901_read_reg8(client, reg->reg & 0xff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int epc901_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	epc901_write_reg8(client, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

static const struct v4l2_ctrl_ops epc901_ctrl_ops = {
	.s_ctrl = epc901_s_ctrl,
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

static const struct v4l2_ctrl_config epc901_custom_ctrls[] = {
    {
        .ops    = &epc901_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SCENE_MODE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "SceneMode",
        .min    = 0,
        .max    = SCENE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &epc901_ctrl_ops,
        .id     = V4L2_CID_CAMERA_ANTI_SHAKE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "AntiShake",
        .min    = 0,
        .max    = ANTI_SHAKE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &epc901_ctrl_ops,
        .id     = V4L2_CID_CAMERA_MODE_CHANGE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "ModeChange",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    },
};

static int epc901_initialize_ctrls(struct epc901_state *state)
{
    v4l2_ctrl_handler_init(&state->handler, NUM_CTRLS);

    /* standard */
    state->focus = v4l2_ctrl_new_std(&state->handler, &epc901_ctrl_ops,
            V4L2_CID_FOCUS_AUTO, 0, 1, 1, 0);
    if (!state->focus) {
        pr_err("%s: failed to create focus ctrl\n", __func__);
        return -1;
    }
    state->wb = v4l2_ctrl_new_std(&state->handler, &epc901_ctrl_ops,
            V4L2_CID_DO_WHITE_BALANCE, WB_AUTO, WB_MAX - 1, 1, WB_AUTO);
    if (!state->wb) {
        pr_err("%s: failed to create wb ctrl\n", __func__);
        return -1;
    }
    state->color_effect = v4l2_ctrl_new_std_menu(&state->handler, &epc901_ctrl_ops,
            V4L2_CID_COLORFX, COLORFX_MAX - 1, 0, COLORFX_NONE);
    if (!state->color_effect) {
        pr_err("%s: failed to create color_effect ctrl\n", __func__);
        return -1;
    }
    state->exposure = v4l2_ctrl_new_std_menu(&state->handler, &epc901_ctrl_ops,
            V4L2_CID_EXPOSURE, MAX_EXPOSURE, 0, 0);
    if (!state->exposure) {
        pr_err("%s: failed to create exposure ctrl\n", __func__);
        return -1;
    }

    /* custom */
    state->scene_mode = v4l2_ctrl_new_custom(&state->handler, &epc901_custom_ctrls[0], NULL);
    if (!state->scene_mode) {
        pr_err("%s: failed to create scene_mode ctrl\n", __func__);
        return -1;
    }
    state->anti_shake = v4l2_ctrl_new_custom(&state->handler, &epc901_custom_ctrls[1], NULL);
    if (!state->anti_shake) {
        pr_err("%s: failed to create anti_shake ctrl\n", __func__);
        return -1;
    }
    state->mode_change = v4l2_ctrl_new_custom(&state->handler, &epc901_custom_ctrls[2], NULL);
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

static int __epc901_set_power(struct epc901_state *state, int on)
{
	// Do something to power up
	return 0;
}

static int epc901_set_power(struct v4l2_subdev *subdev, int on)
{
	struct epc901_state *epc901 = to_state(subdev);
	int ret = 0;

	printk("%s : on(%d)\n", __func__, on);
	mutex_lock(&epc901->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (epc901->power_count == !on) {
		ret = __epc901_set_power(epc901, !!on);
		if (ret < 0)
			goto done;
	}

	/* Update the power count. */
	epc901->power_count += on ? 1 : -1;
	WARN_ON(epc901->power_count < 0);

done:
	mutex_unlock(&epc901->power_lock);
	return ret;
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops epc901_core_ops = {
	.g_chip_ident = epc901_g_chip_ident,
	.g_ctrl = epc901_subdev_g_ctrl,
	.s_ctrl = epc901_subdev_s_ctrl,
	.queryctrl = epc901_queryctrl,
	.s_power = epc901_set_power,
	.reset = epc901_reset,
	.init = epc901_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = epc901_g_register,
	.s_register = epc901_s_register,
#endif
};

static const struct v4l2_subdev_video_ops epc901_video_ops = {
//	.enum_fmt = epc901_enum_fmt,
//	.try_fmt = epc901_try_fmt,
//	.s_fmt = epc901_s_fmt,
	.s_parm = epc901_s_parm,
	.g_parm = epc901_g_parm,
};

static const struct v4l2_subdev_ops epc901_ops = {
	.core = &epc901_core_ops,
	.video = &epc901_video_ops,
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

static const struct media_entity_operations epc901_media_ops = {
	.link_setup = _link_setup,
};
/* ----------------------------------------------------------------------- */

static int epc901_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct epc901_state *info;
	int ret;

	info = kzalloc(sizeof(struct epc901_state), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &epc901_ops);

	mutex_init(&info->power_lock);
	info->power_count = 0;
	
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.ops = &epc901_media_ops;
	if (media_entity_init(&sd->entity, 1, &info->pad, 0)) {
		dev_err(&client->dev, "%s: failed to media_entity_init()\n", __func__);
		kfree(info);
		return -ENOENT;
	}

	printk("EPC901 name = %s, id=%d\n", id->name, info->id);
	
	ret = epc901_initialize_ctrls(info);
	if (ret < 0) {
		pr_err("%s: failed to initialize controls\n", __func__);
		kfree(info);
		return ret;
	}
	
	info->inited = 1;
	info->priv_data = -1; /* page map value */

	return 0;
}

static int epc901_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return	0;
}

#if defined(CONFIG_CAMERA1_EPC901) || defined(CONFIG_SOC_CAMERA_EPC901)
static const struct i2c_device_id epc901_id[] = {
	{ "epc901", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, epc901_id);

static struct i2c_driver epc901_driver = {
	.driver = {
		.name = "epc901",
		.owner = THIS_MODULE,
	},
	.id_table	  = epc901_id,
	.probe		  = epc901_probe,
	.remove		  = __devexit_p(epc901_remove),
};
#endif

static __init int _epc901_init(void)
{
	int ret;

#if defined(CONFIG_CAMERA1_EPC901) || defined(CONFIG_SOC_CAMERA_EPC901)
	ret = i2c_add_driver(&epc901_driver);
#endif

	return ret; 
}

static __init void _epc901_exit(void)
{
#if defined(CONFIG_CAMERA1_EPC901) || defined(CONFIG_SOC_CAMERA_EPC901)
	i2c_del_driver(&epc901_driver);
#endif

}

module_init(_epc901_init)
module_exit(_epc901_exit)

MODULE_AUTHOR("Seungwoo Kim<ksw@stcube.com>");
MODULE_DESCRIPTION("EPC901 Camera Driver");
MODULE_LICENSE("GPL");

