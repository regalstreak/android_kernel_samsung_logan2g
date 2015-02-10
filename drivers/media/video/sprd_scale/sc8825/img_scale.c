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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include "img_scale.h"
#include <linux/kthread.h>

#define PARAM_SIZE              128
#define SCALE_USER_MAX          4
#define INVALID_USER_ID         PID_MAX_DEFAULT

struct scale_user {
	pid_t pid;
	struct semaphore sem_done;
};

static struct mutex scale_param_cfg_mutex;
static struct mutex scale_dev_open_mutex;
static atomic_t scale_users = ATOMIC_INIT(0);
static struct proc_dir_entry *img_scale_proc_file;
static struct scale_frame frm_rtn;
static struct scale_user *g_scale_user = NULL;
static pid_t cur_task_pid;

static struct scale_user *scale_get_user(pid_t user_pid)
{
	struct scale_user *ret_user = NULL;
	int i = 0;

	for (i = 0; i < SCALE_USER_MAX; i ++) {
		if ((g_scale_user + i)->pid == user_pid) {
			ret_user = g_scale_user + i;
			break;
		}
	}

	if (ret_user == NULL) {
		for (i = 0; i < SCALE_USER_MAX; i ++) {
			if ((g_scale_user + i)->pid == INVALID_USER_ID) {
				ret_user = g_scale_user + i;
				ret_user->pid = user_pid;
				break;
			}
		}
	}

	return ret_user;
}

static void scale_done(struct scale_frame* frame, void* u_data)
{
	struct scale_user *p_user = NULL;
	p_user = scale_get_user(cur_task_pid);
	printk("sc done.\n");
	(void)u_data;
	memcpy(&frm_rtn, frame, sizeof(struct scale_frame));
	frm_rtn.type = 0;
	up(&p_user->sem_done);
}

static int img_scale_open(struct inode *node, struct file *pf)
{
	int ret = 0;
	struct scale_user *p_user = NULL;

	mutex_lock(&scale_dev_open_mutex);

	SCALE_TRACE("img_scale_open \n");

	p_user = scale_get_user(current->pid);
	if (NULL == p_user) {
		printk("img_scale_open user cnt full  pid:%d. \n",current->pid);
		return -1;
	}
	pf->private_data = p_user;

	if (1 == atomic_inc_return(&scale_users)) {
		ret = scale_module_en();
		if (unlikely(ret)) {
			printk("Failed to enable scale module \n");
			ret = -EIO;
			goto faile;
		}

		ret = scale_reg_isr(SCALE_TX_DONE, scale_done, NULL);
		if (unlikely(ret)) {
			printk("Failed to register ISR \n");
			ret = -EACCES;
			goto reg_faile;
		} else {
			goto exit;
		}
	}
	else{
		goto exit;
	}
reg_faile:
	scale_module_dis();
faile:
	atomic_dec(&scale_users);
	p_user->pid = INVALID_USER_ID;
	pf->private_data = NULL;
exit:
	mutex_unlock(&scale_dev_open_mutex);

	SCALE_TRACE("img_scale_open %d \n", ret);

	return ret;

}

ssize_t img_scale_write(struct file *file, const char __user * u_data, size_t cnt, loff_t *cnt_ret)
{
	(void)file; (void)u_data; (void)cnt_ret;
	printk("scale write %d, \n", cnt);
	frm_rtn.type = 0xFF;
	up(&(((struct scale_user *)(file->private_data))->sem_done));

	return 1;
}

ssize_t img_scale_read(struct file *file, char __user *u_data, size_t cnt, loff_t *cnt_ret)
{
	uint32_t                 rt_word[2];

	if (cnt < sizeof(uint32_t)) {
		printk("img_scale_read , wrong size of u_data %d \n", cnt);
		return -1;
	}

	rt_word[0] = SCALE_LINE_BUF_LENGTH;
	rt_word[1] = SCALE_SC_COEFF_MAX;
	SCALE_TRACE("img_scale_read line threshold %d %d, sc factor \n", rt_word[0], rt_word[1]);
	(void)file; (void)cnt; (void)cnt_ret;
	return copy_to_user(u_data, (void*)rt_word, (uint32_t)(2*sizeof(uint32_t)));
}

static int img_scale_release(struct inode *node, struct file *file)
{
	((struct scale_user *)(file->private_data))->pid = INVALID_USER_ID;
	if (0 == atomic_dec_return(&scale_users)) {
		scale_reg_isr(SCALE_TX_DONE, NULL, NULL);
		scale_module_dis();
	}

	SCALE_TRACE("img_scale_release \n");
	return 0;
}

static long img_scale_ioctl(struct file *file,
				unsigned int cmd,
				unsigned long arg)
{
	int                      ret = 0;
	uint8_t                  param[PARAM_SIZE];
	uint32_t                 param_size;
	void                     *data = param;

	param_size = _IOC_SIZE(cmd);
	printk("img_scale_ioctl, io number 0x%x, param_size %d \n",
		_IOC_NR(cmd),
		param_size);

	if (param_size) {
		if (copy_from_user(data, (void*)arg, param_size)) {
			printk("img_scale_ioctl, failed to copy_from_user \n");
			ret = -EFAULT;
			goto exit;
		}
	}

	if (SCALE_IO_IS_DONE == cmd) {
		ret = down_timeout(&(((struct scale_user *)(file->private_data))->sem_done), msecs_to_jiffies(100));
		if (ret) {
			printk("img_scale_ioctl, failed to down, 0x%x \n", ret);
			ret = -ERESTARTSYS;
			goto exit;
		} else {
			if (frm_rtn.type) {
				SCALE_TRACE("abnormal scale done \n");
				ret = -1;
				goto exit;
			}
			if (copy_to_user((void*)arg, &frm_rtn, sizeof(struct scale_frame))) {
				printk("img_scale_ioctl, failed to copy_to_user \n");
				ret = -EFAULT;
				goto exit;
			}
		}
	} else {
		if (cur_task_pid == INVALID_USER_ID)
		{
			mutex_lock(&scale_param_cfg_mutex);
			cur_task_pid = ((struct scale_user *)(file->private_data))->pid;
		}else if (cur_task_pid != ((struct scale_user *)(file->private_data))->pid){
			mutex_lock(&scale_param_cfg_mutex);
		}

		ret = scale_cfg(_IOC_NR(cmd), data);

		if (SCALE_IO_STOP == cmd) {
			cur_task_pid = INVALID_USER_ID;
			mutex_unlock(&scale_param_cfg_mutex);
		}

	}

exit:
	if (ret) {
		SCALE_TRACE("img_scale_ioctl, error code 0x%x \n", ret);
	}
	return ret;

}

static struct file_operations img_scale_fops = {
	.owner          = THIS_MODULE,
	.open           = img_scale_open,
	.write          = img_scale_write,
	.read           = img_scale_read,
	.unlocked_ioctl = img_scale_ioctl,
	.release        = img_scale_release
};

static struct miscdevice img_scale_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sprd_scale",
	.fops = &img_scale_fops
};

static int  img_scale_proc_read(char           *page,
			char  	       **start,
			off_t          off,
			int            count,
			int            *eof,
			void           *data)
{
	int                      len = 0, ret;
	uint32_t*                reg_buf;
	uint32_t                 reg_buf_len = 0x400;
	uint32_t                 print_len = 0, print_cnt = 0;
	
	(void)start; (void)off; (void)count; (void)eof;
	
	reg_buf = (uint32_t*)kmalloc(reg_buf_len, GFP_KERNEL);
	ret = scale_read_registers(reg_buf, &reg_buf_len);
	if (ret)
		return len;
	
	len += sprintf(page + len, "********************************************* \n");
	len += sprintf(page + len, "scale registers \n");
	print_cnt = 0;
	while (print_len < reg_buf_len) {
		len += sprintf(page + len, "offset 0x%x : 0x%x, 0x%x, 0x%x, 0x%x \n",
			print_len,
			reg_buf[print_cnt], 
			reg_buf[print_cnt+1],
			reg_buf[print_cnt+2],
			reg_buf[print_cnt+3]);
		print_cnt += 4;
		print_len += 16;
	}
	len += sprintf(page + len, "********************************************* \n");
	len += sprintf(page + len, "The end of DCAM device \n");
	msleep(10);
	kfree(reg_buf);
	
	return len;
}

int img_scale_probe(struct platform_device *pdev)
{
	int                      ret = 0;
	int                      i = 0;
	struct scale_user *p_user = NULL;
	
	SCALE_TRACE("scale_probe called \n");

	ret = misc_register(&img_scale_dev);
	if (ret) {
		SCALE_TRACE("cannot register miscdev (%d)\n", ret);
		goto exit;
	}

	img_scale_proc_file = create_proc_read_entry("driver/scale",
						0444,
						NULL,
						img_scale_proc_read,
						NULL);
	if (unlikely(NULL == img_scale_proc_file)) {
		printk("Can't create an entry for scale in /proc \n");
		ret = ENOMEM;
		goto exit;
	}
	
	/* initialize locks */
	mutex_init(&scale_param_cfg_mutex);
	mutex_init(&scale_dev_open_mutex);

	g_scale_user = kzalloc(SCALE_USER_MAX * sizeof(struct scale_user), GFP_KERNEL);
	if (NULL == g_scale_user) {
		printk("scale_user, no mem");
		return -1;
	}
	p_user = g_scale_user;
	for (i =  0; i < SCALE_USER_MAX; i++) {
		p_user->pid = INVALID_USER_ID;
		sema_init(&p_user->sem_done, 0);
		p_user++;
	}
	cur_task_pid = INVALID_USER_ID;
	
exit:	
	return ret;
}

static int img_scale_remove(struct platform_device *dev)
{
	SCALE_TRACE( "scale_remove called !\n");

	if (g_scale_user) {
		kfree(g_scale_user);
	}
	
	if (img_scale_proc_file) {
		remove_proc_entry("driver/scale", NULL);
	}

	misc_deregister(&img_scale_dev);
	return 0;
}

static struct platform_driver img_scale_driver =
{
	.probe = img_scale_probe,
	.remove = img_scale_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_scale"
	}
};

int __init img_scale_init(void)
{
	if (platform_driver_register(&img_scale_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}
	return 0;
}

void img_scale_exit(void)
{
	platform_driver_unregister(&img_scale_driver);
}

module_init(img_scale_init);
module_exit(img_scale_exit);

MODULE_DESCRIPTION("Image Scale Driver");
MODULE_LICENSE("GPL");

