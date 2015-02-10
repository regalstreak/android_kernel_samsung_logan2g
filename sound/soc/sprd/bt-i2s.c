/*
 * sound/soc/sprd/bt-i2s.c
 *
 * Copyright (C) 2013 SpreadTrum Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "[audio:bti2s] " fmt

#include <sound/soc.h>
#include "dai/i2s/i2s.h"

#ifdef CONFIG_SPRD_AUDIO_DEBUG
#define bt_i2s_dbg pr_debug
#else
#define bt_i2s_dbg (...)
#endif

static struct snd_soc_dai_link bt_i2s_dai[] = {
	{
	 .name = "bt-i2s",
	 .stream_name = "i2s",

	 .codec_name = "null-codec",
	 .platform_name = "sprd-pcm-audio",
	 .cpu_dai_name = "i2s.0",
	 .codec_dai_name = "null-codec-dai",
	 },
};

static struct i2s_config bt_i2s_config = {
        .fs = 8000,
        .slave_timeout = 0xF11,
        .bus_type = I2S_BUS,
        .byte_per_chan = I2S_BPCH_16,
        .mode = I2S_MASTER,
        .lsb = I2S_MSB,
        .rtx_mode = I2S_RTX_MODE,
        .sync_mode = I2S_LRCK,
        .lrck_inv = I2S_L_LEFT,
        .clk_inv = I2S_CLK_R,
        .i2s_bus_mode = I2S_COMPATIBLE,
        .tx_watermark = 8,
        .rx_watermark = 24,
};

#if 0
static struct i2s_config bt_i2s_config = {
	.fs = 8000,
	.slave_timeout = 0xF11,
	.bus_type = PCM_BUS,
	.byte_per_chan = I2S_BPCH_16,
	.mode = I2S_MASTER,
	.lsb = I2S_LSB,
	.rtx_mode = I2S_RTX_MODE,
	.sync_mode = I2S_LRCK,
	.lrck_inv = I2S_L_LEFT,
	.clk_inv = I2S_CLK_N,
	.pcm_bus_mode = I2S_SHORT_FRAME,
	.pcm_slot = 0x1,
	.pcm_cycle = 1,
	.tx_watermark = 8,
	.rx_watermark = 24,
};
#endif
struct i2s_private bt_i2s_priv = { 0 };

static int bt_i2s_late_probe(struct snd_soc_card *card)
{
	int i;
	bt_i2s_dbg("Entering %s\n", __func__);
	bt_i2s_priv.config = &bt_i2s_config;
	for (i = 0; i < card->num_links; i++) {
		struct snd_soc_dai *cpu_dai = card->rtd[i].cpu_dai;
		cpu_dai->ac97_pdata = &bt_i2s_priv;
	}
	return 0;
}

static struct snd_soc_card bt_i2s_card = {
	.name = "bt-i2s",
	.dai_link = bt_i2s_dai,
	.num_links = ARRAY_SIZE(bt_i2s_dai),
	.owner = THIS_MODULE,
	.late_probe = bt_i2s_late_probe,
};

static struct platform_device *bt_i2s_snd_device;

static int __init bt_i2s_modinit(void)
{
	int ret;

	bt_i2s_dbg("Entering %s\n", __func__);

	bt_i2s_snd_device = platform_device_alloc("soc-audio", 1);
	if (!bt_i2s_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bt_i2s_snd_device, &bt_i2s_card);
	ret = platform_device_add(bt_i2s_snd_device);

	if (ret)
		platform_device_put(bt_i2s_snd_device);

	return ret;
}

static void __exit bt_i2s_modexit(void)
{
	platform_device_unregister(bt_i2s_snd_device);
}

module_init(bt_i2s_modinit);
module_exit(bt_i2s_modexit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum IIS+BT");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("machine:i2s+bt");
