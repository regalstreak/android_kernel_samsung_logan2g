/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#ifndef _MEMFREQ_ONDEMAND_H
#define _MEMFREQ_ONDEMAND_H

#include <linux/list.h>

enum {
	MEMFREQ_ONDEMAND_LEVEL = 0,
};

struct memfreq_dbs {
	struct list_head link;
	int level;
	unsigned int (*memfreq_demand)(struct memfreq_dbs *h);
};

void register_memfreq_ondemand(struct memfreq_dbs *handler);
void unregister_memfreq_ondemand(struct memfreq_dbs *handler);

#endif
