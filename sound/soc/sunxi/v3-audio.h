#ifndef _V3_AUDIO_H
#define _V3_AUDIO_H

#include <linux/types.h>
#include <linux/dmaengine.h>




struct v3_pcm_dma_params {
	const char 	*name;		
	dma_addr_t 		dma_addr;
/*	int 			dma_port;	*/
};




int v3_codec_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai);
void v3_codec_dev_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai);
int v3_codec_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai);
int v3_codec_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai);
int v3_codec_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, 
	struct snd_soc_dai *dai);
int v3_codec_mute(struct snd_soc_dai *dai, int mute);
int v3_codec_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id, unsigned int freq, int dir);
int v3_codec_set_dai_clkdiv(struct snd_soc_dai *codec_dai, int div_id, int div);
int v3_codec_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);



                                                                        




















#endif