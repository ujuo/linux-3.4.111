/*
 * V3 ALSA PCM interface
 *
 * Copyright (C) 2018 I4VINE Co., Ltd.
 * All right reserved by JuYoung Ryu <jyryu@i4vine.com>
 *
 * Copyright (C) 2006 Lennert Buytenhek <buytenh@wantstofly.org>
 * Copyright (C) 2006 Applied Data Systems
 *
 * Rewritten for the SoC audio subsystem (Based on PXA2xx code):
 *   Copyright (c) 2008 Ryan Mallon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
//#define DEBUG	1
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/sunxi-dma.h>
//#include "sunxi_codecdma.h"
#include "v3-audio.h"

#ifdef DEBUG
#define DBG_ENTER()	printk("%s: %s enter\n", __FILE__,__func__)
#define DBG_EXIT()	printk("%s: %s exit\n", __FILE__,__func__)
#define DBG_MSG(...)	printk
#else
#define DBG_ENTER()	do{}while(0)
#define DBG_EXIT()	do{}while(0)
#define DBG_MSG(...) 	do{}while(0)
#endif

struct v3_dma_data{
	const char		*name;	
	int				port;
	enum 			dma_transfer_direction	direction;

};

static const struct snd_pcm_hardware v3_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
				      SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
				      SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates			= SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= 1024*1024,    /* value must be (2^n)Kbyte size */
	.period_bytes_min	= 256,
	.period_bytes_max	= 1024*128,
	.periods_min		= 2,
	.periods_max		= 8,
	.fifo_size		= 128,
};

static inline enum dma_transfer_direction v3_dma_chan_direction(struct dma_chan *chan)
{
	DBG_ENTER();
	DBG_MSG("chan->chan_id %d DMA_MEM_TO_DEV %d DMA_DEV_TO_MEM %d\n", chan->chan_id,DMA_MEM_TO_DEV,DMA_DEV_TO_MEM);
	return (chan->chan_id % 2 == 0) ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;
}
	
static bool v3_pcm_dma_filter(struct dma_chan *chan, void *filter_param)
{
	struct v3_dma_data *data = filter_param;
	DBG_ENTER();
	if(data->direction == v3_dma_chan_direction(chan)){
		chan->private = data;
		DBG_EXIT();		
		return true;
	}
	DBG_EXIT();	
	return false;
}

static int v3_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct v3_pcm_dma_params *dma_params;
	struct v3_dma_data *dma_data;

	int ret;
	DBG_ENTER();
	snd_soc_set_runtime_hwparams(substream, &v3_pcm_hardware);
	dma_data = kmalloc(sizeof(*dma_data), GFP_KERNEL);
	DBG_MSG("dma_data 0x%X\n", dma_data);
	if(!dma_data)
		return -ENOMEM;
	
	dma_params = snd_soc_dai_get_dma_data(cpu_dai, substream);
//	dma_data->port = dma_params->dma_port;
	dma_data->name = dma_params->name;
	dma_data->name = "sunxi_dmac";
	dma_data->direction = snd_pcm_substream_to_dma_direction(substream);
	
//	ret = snd_dmaengine_pcm_open(substream, sunxi_dma_filter_fn, dma_data);
	ret = snd_dmaengine_pcm_open(substream, v3_pcm_dma_filter, dma_data);	
	DBG_MSG("ret 0x%X\n", ret);
	if(ret){
		kfree(dma_data);
		return ret;
	}
	snd_dmaengine_pcm_set_data(substream,dma_data);
	DBG_EXIT();	
	return 0;
}

static int v3_pcm_close(struct snd_pcm_substream *substream)
{
	struct dma_data *dma_data = snd_dmaengine_pcm_get_data(substream);
	DBG_ENTER();
	snd_dmaengine_pcm_close(substream);
	kfree(dma_data);
	DBG_EXIT();
	return 0;
}

static int v3_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	struct v3_pcm_dma_params *dmap;
	struct dma_slave_config slave_config;	
	int ret;
	DBG_ENTER();

	dmap = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	ret = snd_hwparams_to_dma_slave_config(substream, params,&slave_config);
	if (ret) {
		dev_err(dev, "hw params config failed with err %d\n", ret);
		return ret;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (SNDRV_PCM_FORMAT_S16_LE == params_format(params)) {
			slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
			slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		} else {
			slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
			slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		}
		slave_config.dst_addr = dmap->dma_addr;
		slave_config.src_maxburst = 4;
		slave_config.dst_maxburst = 4;
		slave_config.slave_id = sunxi_slave_id(DRQDST_AUDIO_CODEC, DRQSRC_SDRAM);
	} else {
		slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		slave_config.src_addr = dmap->dma_addr;
		slave_config.src_maxburst = 4;
		slave_config.dst_maxburst = 4;
		slave_config.slave_id = sunxi_slave_id(DRQDST_SDRAM, DRQSRC_AUDIO_CODEC);
	}

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret < 0) {
		dev_err(dev, "dma slave config failed with err %d\n", ret);
		return ret;
	}
	
#if defined USED_SRAM_ADDR
/*for a23*/
#if defined CONFIG_ARCH_SUN8IW3
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		substream->dma_buffer.addr = 0x00002000;
		substream->dma_buffer.area = 0xf0002000;
		memset(0xf0002000, 0, 0x4000-0x00002000);
	}
#endif
#endif
/*for a33*/
#if defined CONFIG_ARCH_SUN8IW5
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		substream->dma_buffer.addr = 0x00002400;//0x00002400;
		substream->dma_buffer.area = (unsigned char *)0xf0002400;//0xf0002400;
		memset((void *)0xf0002400, 0, 0x6400-0x00002400);
	}
#endif	
	
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	DBG_EXIT();	
	return 0;
}

static int v3_pcm_hw_free(struct snd_pcm_substream *substream)
{
	DBG_ENTER();
	snd_pcm_set_runtime_buffer(substream, NULL);
	DBG_EXIT();	
	return 0;
}

static int v3_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	DBG_ENTER();
	return dma_mmap_writecombine(substream->pcm->card->dev, vma, 
		runtime->dma_area,runtime->dma_addr,runtime->dma_bytes);
}

static int v3_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct dma_chan		*chan = snd_dmaengine_pcm_get_chan(substream);	
	struct dma_async_tx_descriptor *desc;
	enum dma_transfer_direction direction;
	struct v3_dma_data	*dma_data;
	struct snd_pcm_runtime *runtime = substream->runtime;

	
	DBG_ENTER();
	DBG_MSG("dma_chan->device 0x%x\n", chan->device);
	DBG_MSG("runtime->buffer_size %d runtime->period_size %d \n",runtime->buffer_size,runtime->period_size);

	DBG_EXIT();
	return 0;
	
}

static int v3_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	DBG_ENTER();
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			return snd_dmaengine_pcm_trigger(substream, SNDRV_PCM_TRIGGER_START);
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			return snd_dmaengine_pcm_trigger(substream, SNDRV_PCM_TRIGGER_STOP);
		}
	} else {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			return snd_dmaengine_pcm_trigger(substream, SNDRV_PCM_TRIGGER_START);
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			return snd_dmaengine_pcm_trigger(substream, SNDRV_PCM_TRIGGER_STOP);
		}
	}
	DBG_EXIT();
	return 0;
}
	
static struct snd_pcm_ops v3_pcm_ops = {
	.open = v3_pcm_open,
	.close = v3_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = v3_pcm_hw_params,
	.hw_free = v3_pcm_hw_free,
	/*.prepare = v3_pcm_prepare,*/
	.trigger = v3_pcm_trigger,
	.pointer = snd_dmaengine_pcm_pointer,
/*	.trigger = snd_dmaengine_pcm_trigger,
	.pointer = snd_dmaengine_pcm_pointer,*/
	.mmap = v3_pcm_mmap,
};

static int v3_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	DBG_ENTER();
	size_t size = v3_pcm_hardware.buffer_bytes_max;
	
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size, &buf->addr, GFP_KERNEL);
	buf->bytes = size;
	DBG_EXIT();	
	return (buf->area == NULL)? -ENOMEM : 0;
}

static void v3_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;
	DBG_ENTER();
	for(stream = 0; stream < 2; stream++){
		substream = pcm->streams[stream].substream;
		if(!substream)
			continue;
		
		buf = &substream->dma_buffer;
		if(!buf->area)
			continue;
		dma_free_writecombine(pcm->card->dev, buf->bytes, buf->area, buf->addr);
		
		buf->area = NULL;
	}
	DBG_EXIT();	
}

static u64 v3_pcm_dmamask = DMA_BIT_MASK(32);



static int v3_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;
	DBG_ENTER();
	if(!card->dev->dma_mask)
		card->dev->dma_mask = &v3_pcm_dmamask;
	if(!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
	if(pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream){
		ret = v3_pcm_preallocate_dma_buffer(pcm,SNDRV_PCM_STREAM_PLAYBACK);
		if(ret)
			return ret;
	}
	DBG_EXIT();	
	return 0;
}

static struct snd_soc_platform_driver v3_soc_platform = {
	.ops = &v3_pcm_ops,
	.pcm_new = v3_pcm_new,
	.pcm_free = v3_pcm_free_dma_buffers,
};

static int __init v3_soc_platform_probe(struct platform_device *pdev)
{	
	DBG_ENTER();
	return snd_soc_register_platform(&pdev->dev, &v3_soc_platform);
}

static int __exit v3_soc_platform_remove(struct platform_device *pdev)
{
	DBG_ENTER();
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_device v3_pcm_device = {
	.name = "v3-pcm-audio",
	.id = -1,
};

static struct platform_driver v3_pcm_driver = {
	.probe = v3_soc_platform_probe,
	.remove = __exit_p(v3_soc_platform_remove),	
	.driver = {
			.name = "v3-pcm-audio",
			.owner = THIS_MODULE,
	},
};

static int __init v3_pcm_audio_init(void)
{
	int err = 0;
	DBG_ENTER();
	if((err == platform_device_register(&v3_pcm_device)) < 0)
		return err;
	
	if((err == platform_driver_register(&v3_pcm_driver)) < 0)
		return err;
	
	DBG_EXIT();
	return 0;
}
module_init(v3_pcm_audio_init);

static void __exit v3_pcm_audio_exit(void)
{
	DBG_ENTER();
	return platform_driver_unregister(&v3_pcm_driver);
}
module_exit(v3_pcm_audio_exit);

MODULE_AUTHOR("JuYoung Ryu");
MODULE_DESCRIPTION("V3 ALSA PCM interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:v3-pcm-audio");