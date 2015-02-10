/*
 * sound/soc/sprd/dai/sprd-pcm.h
 *
 * SpreadTrum DMA for the pcm stream.
 *
 * Copyright (C) 2012 SpreadTrum Ltd.
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
#ifndef __SPRD_PCM_H
#define __SPRD_PCM_H

#include <mach/dma.h>

#define VBC_FIFO_FRAME_NUM	160
#define VBC_BUFFER_BYTES_MAX	(128 * 1024)

#define I2S_BUFFER_BYTES_MAX	(64 * 1024)

#define AUDIO_BUFFER_BYTES_MAX	(VBC_BUFFER_BYTES_MAX + I2S_BUFFER_BYTES_MAX)

struct sprd_pcm_dma_params {
	char *name;		/* stream identifier */
	int channels[2];	/* channel id */
	int workmode;		/* dma work type */
	int irq_type;		/* dma interrupt type */
	struct sprd_dma_channel_desc desc;	/* dma description struct */
	u32 dev_paddr[2];	/* device physical address for DMA */
};

#endif /* __SPRD_PCM_H */
