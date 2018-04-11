/*
 * SoC audio for v3 i4vine board 
 *
 * Copyright (C) 2018 I4VINE Co., Ltd.
 * All right reserved by JuYoung Ryu <jyryu@i4vine.com>
 
 * Copyright (c) 2010 Alexander Sverdlin <subaparts@yandex.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
//#define DEBUG	1
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/mach-types.h>
//#include <mach/corgi.h>
//#include <mach/audio.h>

#ifdef DEBUG
#define DBG_ENTER()	printk("%s: %s enter\n", __FILE__,__func__)
#define DBG_EXIT()	printk("%s: %s exit\n", __FILE__,__func__)
#define DBG_MSG(...)	printk
#else
#define DBG_ENTER()	do{}while(0)
#define DBG_EXIT()	do{}while(0)
#define DBG_MSG(...) 	do{}while(0)
#endif

static struct snd_soc_dai_link v3_audio_dai = {
	.name = "v3-audio",
	.stream_name = "V3-codec",
	.platform_name = "v3-pcm-audio",
	.codec_name = "v3-codec",	
	.cpu_dai_name = "v3-adcdac",	
	.codec_dai_name = "v3-codec-recplay",
};


static struct snd_soc_card snd_soc_v3_audio = {
	.name = "v3-audio",
	.owner = THIS_MODULE,
	.dai_link = &v3_audio_dai,
	.num_links = 1,
};
	

static int __devinit v3_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_v3_audio;
	int ret;
	DBG_ENTER();
	DBG_MSG("dev name %s \n", pdev->dev.name);
	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if(ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);
	DBG_EXIT();
	return ret;
}


static int __exit v3_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	DBG_ENTER();
	snd_soc_unregister_card(card);
	DBG_EXIT();
	return 0;
}

static struct platform_device v3_audio_device = {
	.name = "v3-audio",
	.id = PLATFORM_DEVID_NONE,
};

static struct platform_driver v3_audio_driver = {
	.driver		= {
		.name	= "v3-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= v3_audio_probe,
	.remove		= __exit_p(v3_audio_remove),
};

static int __init v3_audio_init(void)
{
	int err = 0;
	DBG_ENTER();
	if((err == platform_device_register(&v3_audio_device)) < 0)
		return err;
	
	if((err == platform_driver_register(&v3_audio_driver)) < 0)
		return err;
	
	DBG_EXIT();
	return 0;
}
module_init(v3_audio_init);

static void __exit v3_audio_exit(void)
{
	DBG_ENTER();
	platform_driver_unregister(&v3_audio_driver);
	DBG_EXIT();
}
module_exit(v3_audio_exit);

MODULE_AUTHOR("JuYoung Ryu");
MODULE_DESCRIPTION("ALSA SoC v3-audio");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:v3-audio");





