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

#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/page.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <linux/memblock.h>

static int __init __pmem_reserve_memblock(void)
{
	if (memblock_is_region_reserved(SPRD_PMEM_BASE, SPRD_IO_MEM_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_PMEM_BASE, SPRD_IO_MEM_SIZE))
		return -ENOMEM;
	return 0;
}
static int __init __emc_training_reserve_memblock(void)
{
	printk("__emc_training_reserve_memblock %x, %x", SPRD_EMC_TRAINING_MEM_BASE, SPRD_EMC_TRAINING_MEM_SIZE);
	if (memblock_is_region_reserved(SPRD_EMC_TRAINING_MEM_BASE, SPRD_EMC_TRAINING_MEM_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_EMC_TRAINING_MEM_BASE, SPRD_EMC_TRAINING_MEM_SIZE))
		return -ENOMEM;
	return 0;
}
#ifdef CONFIG_ANDROID_RAM_CONSOLE
int __init __ramconsole_reserve_memblock(void)
{
	if (memblock_is_region_reserved(SPRD_RAM_CONSOLE_START, SPRD_RAM_CONSOLE_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_RAM_CONSOLE_START, SPRD_RAM_CONSOLE_SIZE))
		return -ENOMEM;
	return 0;
}
#endif

void __init sc8825_reserve(void)
{
	int ret = __pmem_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for pmem. errno=%d\n", ret);

	ret = __emc_training_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for emc training mem. errno=%d\n", ret);

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	if (ret = __ramconsole_reserve_memblock())
		pr_err("Fail to reserve mem for ram_console. errno=%d\n", ret);
#endif
}
