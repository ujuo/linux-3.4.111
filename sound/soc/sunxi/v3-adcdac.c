/*
 * V3 SOC CPU DAI driver
 *
 * Copyright (C) 2018 I4VINE Co., Ltd.
 * All right reserved by JuYoung Ryu <jyryu@i4vine.com>
 *
 * sound\soc\sunxi\audiocodec\v3-adcdac.c
 * (C) Copyright 2010-2016
 * Reuuimlla Technology Co., Ltd. <www.reuuimllatech.com>
 * huangxin <huangxin@Reuuimllatech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */
 
//#define DEBUG	
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <mach/hardware.h>
//#include "sunxi_codecdma.h"
#include "v3-audio.h"

#ifdef DEBUG
#define DBG_ENTER()	printk("%s: %s enter\n", __FILE__,__func__)
#define DBG_EXIT()	printk("%s: %s exit\n", __FILE__,__func__)
#else
#define DBG_ENTER()	do{}while(0)
#define DBG_EXIT()	do{}while(0)
#endif

#define SUNXI_PCM_RATES (SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT)

static struct snd_soc_dai_ops v3_adcdac_dai_ops = {
	.startup = v3_codec_startup,
/*	.shutdown = v3_codec_dev_shutdown,
	.prepare  =	v3_codec_prepare,
	.trigger 	= v3_codec_trigger,
	.hw_params = v3_codec_hw_params,
	.digital_mute = v3_codec_mute,
	.set_sysclk = v3_codec_set_dai_sysclk,
	.set_clkdiv = v3_codec_set_dai_clkdiv,
	.set_fmt = v3_codec_set_dai_fmt,*/
};

static struct snd_soc_dai_driver v3_adcdac_dai = {
	.playback 	= {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SUNXI_PCM_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture 	= {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SUNXI_PCM_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops		= &v3_adcdac_dai_ops,
#if defined CONFIG_ARCH_SUN8IW5 || defined CONFIG_ARCH_SUN8IW9
	.ops 		= &sunxi_i2s_dai_ops,
#endif
};

static int __init v3_adcdac_dev_probe(struct platform_device *pdev)
{
	int err = -1;
	DBG_ENTER();
#ifdef CONFIG_ARCH_SUN8IW9
	/*global enable*/
	codec_wr_control(SUNXI_DA_CTL, 0x1, GEN, 0x1);
//	codec_wr_control(SUNXI_DA_CTL, 0x1, LOOP, 0x1);  //for loopback test
#endif
	err = snd_soc_register_dai(&pdev->dev, &v3_adcdac_dai);	
	if (err) {
		dev_err(&pdev->dev, "Failed to register DAI\n");
		return err;
	}
	DBG_EXIT();
	return 0;
}

static int __exit v3_adcdac_dev_remove(struct platform_device *pdev)
{
	DBG_ENTER();	
	snd_soc_unregister_dai(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	DBG_EXIT();
	return 0;
}


/*data relating*/
static struct platform_device v3_adcdac_device = {
	.name = "v3-adcdac",
	.id = -1,
};

/*method relating*/
static struct platform_driver v3_adcdac_driver = {
	.probe = v3_adcdac_dev_probe,
	.remove = __exit_p(v3_adcdac_dev_remove),	
	.driver = {
		.name = "v3-adcdac",
		.owner = THIS_MODULE,
	},	
};

static int __init v3_adcdac_init(void)
{
	int err = 0;
	DBG_ENTER();
	if((err = platform_device_register(&v3_adcdac_device)) < 0)
		return err;

	if ((err = platform_driver_register(&v3_adcdac_driver)) < 0)
		return err;	
	DBG_EXIT();
	return 0;
}
module_init(v3_adcdac_init);

static void __exit v3_adcdac_exit(void)
{
	DBG_ENTER();	
	platform_driver_unregister(&v3_adcdac_driver);
	DBG_EXIT();	
}
module_exit(v3_adcdac_exit);

/* Module information */
MODULE_AUTHOR("JuYoung Ryu");
MODULE_DESCRIPTION("V3 codec SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:v3-adcdac");


