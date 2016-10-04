/*
	Copyright(c) 2016 STcube Inc.,
	All right reserved by Seungwoo Kim <ksw@stcube.com>
	
  ALL OF SUNXI specific V4L2 constants are defined here.
  
  
 */
#ifndef __SUNXI_V4L2_TYPE_H__
#define __SUNXI_V4L2_TYPE_H__

#include <linux/videodev2.h>

#define V4L2_MODE_VIDEO                         0x0002  /*  Added by raymonxiu For video capture */
#define V4L2_MODE_IMAGE                         0x0003  /*  Added by raymonxiu For image capture */
#define V4L2_MODE_PREVIEW                       0x0004  /*  Added by raymonxiu For preview capture */

/* Add thumb picture mirror and flip by raymonxiu */
#define V4L2_CID_HFLIP_THUMB            (V4L2_CID_PRIVATE_BASE+0)
#define V4L2_CID_VFLIP_THUMB            (V4L2_CID_PRIVATE_BASE+1)
/* Add autofocus windows number by raymonxiu */
#define V4L2_CID_AUTO_FOCUS_WIN_NUM             (V4L2_CID_PRIVATE_BASE+2)
/* Add camera autofocus ctrl by raymonxiu */
#define V4L2_CID_AUTO_FOCUS_INIT                (V4L2_CID_PRIVATE_BASE+3)
#define V4L2_CID_AUTO_FOCUS_RELEASE     (V4L2_CID_PRIVATE_BASE+4)

/* Add auto exposure windows number by raymonxiu */
#define V4L2_CID_AUTO_EXPOSURE_WIN_NUM          (V4L2_CID_PRIVATE_BASE+5)
/* Add gsensor rotation degree by raymonxiu */
#define V4L2_CID_GSENSOR_ROTATION               (V4L2_CID_PRIVATE_BASE+6)
#define V4L2_CID_FRAME_RATE             (V4L2_CID_PRIVATE_BASE+7)
/* Add take picture by yang */
#define V4L2_CID_TAKE_PICTURE           (V4L2_CID_PRIVATE_BASE+8)
enum v4l2_take_picture {
        V4L2_TAKE_PICTURE_STOP  = 0,
        V4L2_TAKE_PICTURE_NORM  = 1,
        V4L2_TAKE_PICTURE_FAST  = 2,
        V4L2_TAKE_PICTURE_FLASH = 3,
        V4L2_TAKE_PICTURE_HDR   = 4,
};

#define V4L2_CID_HDR                   (V4L2_CID_PRIVATE_BASE+9)
typedef union
{
        unsigned int dwval;
        struct
        {
                unsigned int af_sharp             : 16 ;
                unsigned int hdr_cnt                    :  4 ;
                unsigned int flash_ok             : 1 ;
                unsigned int capture_ok        :  1 ;
                unsigned int fast_capture_ok        :  1 ;
                unsigned int res0        :  9 ;
        } bits;
} IMAGE_FLAG_t;

#define V4L2_GAIN_SHIFT                         0
#define V4L2_SHARP_LEVEL_SHIFT          8
#define V4L2_SHARP_MIN_SHIFT            20
#define V4L2_NDF_SHIFT                          26

/* Add horizontal visual angle by yang */
#define  V4L2_CID_HOR_VISUAL_ANGLE      (V4L2_CID_PRIVATE_BASE+10)
/* Add vertical  visual angle by yang */
#define  V4L2_CID_VER_VISUAL_ANGLE       (V4L2_CID_PRIVATE_BASE+11)
/* Add focus length by yang */
#define  V4L2_CID_FOCUS_LENGTH               (V4L2_CID_PRIVATE_BASE+12)
/* Add red gain by yang */
#define  V4L2_CID_R_GAIN                                        (V4L2_CID_PRIVATE_BASE+13)
/* Add green gain by yang */
#define  V4L2_CID_G_GAIN                            (V4L2_CID_PRIVATE_BASE+14)
/* Add blue gain by yang */
#define  V4L2_CID_B_GAIN                           (V4L2_CID_PRIVATE_BASE+15)
/* Add sensor type by yang */
#define V4L2_CID_SENSOR_TYPE         (V4L2_CID_PRIVATE_BASE+16)
enum v4l2_sensor_type
{
        V4L2_SENSOR_TYPE_YUV            = 0,
        V4L2_SENSOR_TYPE_RAW            = 1,
};

#define V4L2_CID_AUTO_EXPOSURE_BIAS             (V4L2_CID_CAMERA_CLASS_BASE+19)

#define V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE    (V4L2_CID_CAMERA_CLASS_BASE+20)
enum v4l2_auto_n_preset_white_balance {
        V4L2_WHITE_BALANCE_MANUAL               = 0,
        V4L2_WHITE_BALANCE_AUTO                 = 1,
        V4L2_WHITE_BALANCE_INCANDESCENT         = 2,
        V4L2_WHITE_BALANCE_FLUORESCENT          = 3,
        V4L2_WHITE_BALANCE_FLUORESCENT_H        = 4,
        V4L2_WHITE_BALANCE_HORIZON              = 5,
        V4L2_WHITE_BALANCE_DAYLIGHT             = 6,
        V4L2_WHITE_BALANCE_FLASH                = 7,
        V4L2_WHITE_BALANCE_CLOUDY               = 8,
        V4L2_WHITE_BALANCE_SHADE                = 9,
};
#define V4L2_CID_WIDE_DYNAMIC_RANGE             (V4L2_CID_CAMERA_CLASS_BASE+21)
#define V4L2_CID_IMAGE_STABILIZATION            (V4L2_CID_CAMERA_CLASS_BASE+22)

#define V4L2_CID_ISO_SENSITIVITY                (V4L2_CID_CAMERA_CLASS_BASE+23)

#define V4L2_CID_ISO_SENSITIVITY_AUTO           (V4L2_CID_CAMERA_CLASS_BASE+24)
enum v4l2_iso_sensitivity_auto_type {
        V4L2_ISO_SENSITIVITY_MANUAL             = 0,
        V4L2_ISO_SENSITIVITY_AUTO               = 1,
};

#define V4L2_CID_EXPOSURE_METERING              (V4L2_CID_CAMERA_CLASS_BASE+25)
enum v4l2_exposure_metering {
        V4L2_EXPOSURE_METERING_AVERAGE          = 0,
        V4L2_EXPOSURE_METERING_CENTER_WEIGHTED  = 1,
        V4L2_EXPOSURE_METERING_SPOT             = 2,
};

#define V4L2_CID_SCENE_MODE                     (V4L2_CID_CAMERA_CLASS_BASE+26)
enum v4l2_scene_mode {
        V4L2_SCENE_MODE_NONE                    = 0,
        V4L2_SCENE_MODE_BACKLIGHT               = 1,
        V4L2_SCENE_MODE_BEACH_SNOW              = 2,
        V4L2_SCENE_MODE_CANDLE_LIGHT            = 3,
        V4L2_SCENE_MODE_DAWN_DUSK               = 4,
        V4L2_SCENE_MODE_FALL_COLORS             = 5,
        V4L2_SCENE_MODE_FIREWORKS               = 6,
        V4L2_SCENE_MODE_LANDSCAPE               = 7,
        V4L2_SCENE_MODE_NIGHT                   = 8,
        V4L2_SCENE_MODE_PARTY_INDOOR            = 9,
        V4L2_SCENE_MODE_PORTRAIT                = 10,
        V4L2_SCENE_MODE_SPORTS                  = 11,
        V4L2_SCENE_MODE_SUNSET                  = 12,
        V4L2_SCENE_MODE_TEXT                    = 13,
};

#define V4L2_CID_3A_LOCK                        (V4L2_CID_CAMERA_CLASS_BASE+27)
#define V4L2_LOCK_EXPOSURE                      (1 << 0)
#define V4L2_LOCK_WHITE_BALANCE                 (1 << 1)
#define V4L2_LOCK_FOCUS                         (1 << 2)

#define V4L2_CID_AUTO_FOCUS_START               (V4L2_CID_CAMERA_CLASS_BASE+28)
#define V4L2_CID_AUTO_FOCUS_STOP                (V4L2_CID_CAMERA_CLASS_BASE+29)

#define V4L2_CID_AUTO_FOCUS_STATUS              (V4L2_CID_CAMERA_CLASS_BASE+30)
#define V4L2_AUTO_FOCUS_STATUS_IDLE             (0 << 0)
#define V4L2_AUTO_FOCUS_STATUS_BUSY             (1 << 0)
#define V4L2_AUTO_FOCUS_STATUS_REACHED          (1 << 1)
#define V4L2_AUTO_FOCUS_STATUS_FAILED           (1 << 2)

#define V4L2_CID_AUTO_FOCUS_RANGE               (V4L2_CID_CAMERA_CLASS_BASE+31)
enum v4l2_auto_focus_range {
        V4L2_AUTO_FOCUS_RANGE_AUTO              = 0,
        V4L2_AUTO_FOCUS_RANGE_NORMAL            = 1,
        V4L2_AUTO_FOCUS_RANGE_MACRO             = 2,
        V4L2_AUTO_FOCUS_RANGE_INFINITY          = 3,
};

#define V4L2_FLASH_LED_MODE_AUTO	(V4L2_FLASH_LED_MODE_TORCH + 1)   //add by raymonxiu
#define V4L2_FLASH_LED_MODE_RED_EYE	(V4L2_FLASH_LED_MODE_AUTO + 1)    //add by raymonxiu

/* Add windows coordinates struct by raymonxiu */
struct v4l2_win_coordinate {
        __s32                   x1;
        __s32                   y1;
        __s32                   x2;
        __s32                   y2;
};

#define MAX_EXP_FRAMES     5
#define MAX_ISP_BUF_SIZE   0x2100

struct isp_hdr_setting_t {
  __s32 hdr_en;
  __s32 hdr_mode;
  __s32 frames_count;
  __s32 total_frames;
  __s32 values[MAX_EXP_FRAMES];
};

struct isp_exif_attribute {
        struct v4l2_fract exposure_time;
        struct v4l2_fract shutter_speed;
        __u32 fnumber;
        __u32 focal_length;
        __s32 exposure_bias;
        __u32 iso_speed;
        __u32 flash_fire;
        __u32 brightness;
        __s32 reserved[16];
};

struct isp_stat_buf {
        void __user *buf;
        __u32 buf_size;
};

//add by yang
#define VIDIOC_ISP_AE_STAT_REQ \
        _IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct isp_stat_buf)
//add by yang
#define VIDIOC_ISP_HIST_STAT_REQ \
        _IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct isp_stat_buf)
//add by yang
#define VIDIOC_ISP_AF_STAT_REQ \
        _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct isp_stat_buf)
//add by yang
#define VIDIOC_ISP_EXIF_REQ \
        _IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct isp_exif_attribute)
//add by yang
#define VIDIOC_ISP_GAMMA_REQ \
        _IOWR('V', BASE_VIDIOC_PRIVATE + 5, struct isp_stat_buf)

#endif