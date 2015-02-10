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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/bug.h>

#ifdef CONFIG_DEBUG_FS
extern void cp_abort(void);
static int debugfs_make_kernel_panic(void *data, u64 val)
{
#if defined(CONFIG_ARCH_SC8810) || defined(CONFIG_ARCH_SC8825)
	if (val > 0) cp_abort();
#endif
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(do_panic_fops, NULL, debugfs_make_kernel_panic,"%llu\n");

void create_sys_debugfs(void)
{
	struct dentry *root = NULL;

	root = debugfs_create_dir("system", NULL);
	if (IS_ERR(root) || !root) {
		return;
	}

	if (!debugfs_create_file("dopanic", S_IWUSR, root, NULL, &do_panic_fops)) {
		goto err_create;
	}

	return;

err_create:
	debugfs_remove_recursive(root);
	pr_err("failed to create debugfs: do-panic\n");
}

#endif

void sys_debug_init(void)
{
#ifdef CONFIG_DEBUG_FS
	create_sys_debugfs();
#endif
}


