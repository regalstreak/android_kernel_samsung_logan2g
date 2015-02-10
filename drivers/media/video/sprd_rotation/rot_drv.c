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
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <linux/file.h>
#include <mach/dma.h>
#include <linux/sched.h>
#include <video/sprd_rot_k.h>
#include "rot_reg.h"
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include <mach/sci.h>

#define RTT_PRINT pr_debug
//#define RTT_PRINT printk

//#define ROTATION_DEBUG		0

#define BOOLEAN 				char
#define ROT_TRUE 			1
#define ROT_FALSE 			0
#define DISABLE_AHB_SLEEP 	0
#define ENABLE_AHB_SLEEP 	1
#define ROT_TIMEOUT         100/*ms*/

#define REG_RD(a)						__raw_readl(a)
#define REG_WR(a,v)					__raw_writel(v,a)
#define REG_AWR(a,v)					__raw_writel((__raw_readl(a) & v), a)
#define REG_OWR(a,v)                           	 __raw_writel((__raw_readl(a) | v), a)
#define REG_XWR(a,v)                                 	 __raw_writel((__raw_readl(a) ^ v), a)
#define REG_MWR(a,m,v)                                 \
	do {                                           \
		uint32_t _tmp = __raw_readl(a);        \
		_tmp &= ~(m);                          \
		__raw_writel(_tmp | ((m) & (v)), (a)); \
	}while(0)


typedef struct _dma_rot_tag {
	ROT_SIZE_T 				img_size;
	ROT_DATA_FORMAT_E 		format;
	ROT_ANGLE_E 				angle;
	ROT_ADDR_T 				src_addr;
	ROT_ADDR_T 				dst_addr;
	uint32_t 					s_addr;
	uint32_t 					d_addr;
	ROT_PIXEL_FORMAT_E 		pixel_format;
	ROT_UV_MODE_E 			uv_mode;
	BOOLEAN 				is_end;
	int 						ch_id;
} ROT_DMA_CFG_T, *ROT_DMA_CFG_T_PTR;

static ROT_DMA_CFG_T 	s_rotation_cfg;
static 	int s_ch_id = -1;
static 	int s_virtual_ch_id = -1;

#define ALGIN_FOUR      0x03
#define DECLARE_ROTATION_PARAM_ENTRY(s) 		ROT_DMA_CFG_T *s=&s_rotation_cfg
#define ROTATION_MINOR MISC_DYNAMIC_MINOR
#define ROT_USER_MAX    4
#define INVALID_USER_ID PID_MAX_DEFAULT
static wait_queue_head_t wait_queue;
static wait_queue_head_t thread_queue;
static int condition;
static struct semaphore g_sem_dev_open;
static struct semaphore g_sem_rot;
static struct semaphore g_sem_copy;
static struct semaphore g_sem_physical_ch;
static struct semaphore g_sem_virtual_ch;
static int g_copy_done;
static int g_thread_run;
static int g_is_rot_timeout;

struct task_struct *g_rot_task;

struct rot_context {
	atomic_t start_flag;
	struct timer_list  rot_timer;
};

struct rot_user {
	pid_t pid;
	uint32_t is_exit_force;
	uint32_t is_rot_enable;
	struct semaphore sem_done;
};

static pid_t cur_task_pid;
static struct rot_context rot_conext;
static struct rot_context *rot_cnt = &rot_conext;
static struct rot_user      *g_rot_user = NULL;

static int rot_start_timer(struct timer_list *rot_timer, uint32_t time_val);
static void rot_stop_timer(struct timer_list *rot_timer);
static int rot_k_check_param(ROT_CFG_T * param_ptr)
{
	if (NULL == param_ptr) {
		RTT_PRINT("Rotation: the param ptr is null.\n");
		return -1;
	}

	if (    (param_ptr->src_addr.y_addr & ALGIN_FOUR)
	    || (param_ptr->src_addr.u_addr & ALGIN_FOUR)
	    || (param_ptr->src_addr.v_addr & ALGIN_FOUR)
	    || (param_ptr->dst_addr.y_addr & ALGIN_FOUR)
	    || (param_ptr->dst_addr.u_addr & ALGIN_FOUR)
	    || (param_ptr->dst_addr.v_addr & ALGIN_FOUR)) {
		RTT_PRINT("Rotation: the addr not algin.\n");
		return -1;
	}

	if (ROT_RGB565 < param_ptr->format) {
		RTT_PRINT("Rotation: data for err : %d.\n", param_ptr->format);
		return -1;
	}
	return 0;
}

static ROT_PIXEL_FORMAT_E rot_k_get_pixel_format(void)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	switch (s->format) {
	case ROT_YUV422:
	case ROT_YUV420:
	case ROT_YUV400:
		s->pixel_format = ROT_ONE_BYTE;
		break;
	case ROT_RGB565:
		s->pixel_format = ROT_TWO_BYTES;
		break;
	case ROT_RGB888:
	case ROT_RGB666:
		s->pixel_format = ROT_FOUR_BYTES;
		break;
	default:
		break;
	}
	return s->pixel_format;
}

static BOOLEAN rot_k_is_end(void)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	switch (s->format) {
	case ROT_YUV422:
	case ROT_YUV420:
		s->is_end = ROT_FALSE;
		break;
	case ROT_YUV400:
	case ROT_RGB888:
	case ROT_RGB565:
	case ROT_RGB666:
		s->is_end = ROT_TRUE;
		break;
	default:
		break;
	}
	return s->is_end;
}

static int rot_k_set_y_param(ROT_CFG_T * param_ptr)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	memcpy((void *)&(s->img_size), (void *)&(param_ptr->img_size),
	       sizeof(ROT_SIZE_T));
	memcpy((void *)&(s->src_addr), (void *)&(param_ptr->src_addr),
	       sizeof(ROT_ADDR_T));
	memcpy((void *)&(s->dst_addr), (void *)&(param_ptr->dst_addr),
	       sizeof(ROT_ADDR_T));

	s->s_addr = param_ptr->src_addr.y_addr;
	s->d_addr = param_ptr->dst_addr.y_addr;
	s->format = param_ptr->format;
	s->angle = param_ptr->angle;

	s->pixel_format = rot_k_get_pixel_format();
	s->is_end = rot_k_is_end();
	s->uv_mode = ROT_NORMAL;
	return 0;
}

static void rot_k_cfg(void)
{
	// rot eb
	sci_glb_set(AHB_GLOBAL_REG_CTL0, BIT(14));	//ROTATION_DRV_ONE

	// rot soft reset
	sci_glb_set(AHB_GLOBAL_REG_SOFTRST, BIT(10));
	sci_glb_clr(AHB_GLOBAL_REG_SOFTRST, BIT(10));
}

static void rot_k_disable(void)
{
	// rot eb
	sci_glb_clr(AHB_GLOBAL_REG_CTL0, BIT(14));	//ROTATION_DRV_ONE
}

static void rot_k_software_reset(void)
{
	// rot soft reset
	sci_glb_set(AHB_GLOBAL_REG_SOFTRST, BIT(10));
	sci_glb_clr(AHB_GLOBAL_REG_SOFTRST, BIT(10));
}

static void rot_k_set_src_addr(uint32_t src_addr)
{
	REG_WR(REG_ROTATION_SRC_ADDR, src_addr);
	return;
}

static void rot_k_set_dst_addr(uint32_t dst_addr)
{
	REG_WR(REG_ROTATION_DST_ADDR, dst_addr);
	return;
}

static void rot_k_set_img_size(ROT_SIZE_T * size)
{
	REG_AWR(REG_ROTATION_IMG_SIZE, 0xFF000000);
	REG_OWR(REG_ROTATION_IMG_SIZE,
	      (size->h & 0xFFF) | ((size->w & 0xFFF) << 12));
	REG_WR(REG_ROTATION_ORIGWIDTH, size->w & 0xFFF);
	return;
}

static void rot_k_set_pixel_mode(ROT_PIXEL_FORMAT_E pixel_format)
{
	REG_AWR(REG_ROTATION_IMG_SIZE, ~(0x3 << 24));
	REG_OWR(REG_ROTATION_IMG_SIZE, pixel_format << 24);
	return;
}

static void rot_k_set_dir(ROT_ANGLE_E angle)
{
	REG_AWR(REG_ROTATION_CTRL, ~(0x3 << 1));
	REG_OWR(REG_ROTATION_CTRL, (angle & 0x3) << 1);
	return;
}

static void rot_k_set_UV_mode(ROT_UV_MODE_E uv_mode)
{
	REG_AWR(REG_ROTATION_CTRL, (~BIT(0)));
	REG_OWR(REG_ROTATION_CTRL, (uv_mode & 0x1));
	return;
}

static void rot_k_enable(void)
{
	REG_OWR(REG_ROTATION_CTRL, BIT(3));
	return;
}

#ifdef ROTATION_DEBUG		//for debug
static void rot_k_get_reg(void)
{
	uint32_t i, value;
	RTT_PRINT
	    ("###############rot_k_get_reg##########################\n");
	for (i = 0; i < 12; i++) {
		value = REG_RD(REG_ROTATION_SRC_ADDR + i * 4);
		RTT_PRINT("ROT reg:0x%x, 0x%x.\n",
			  REG_ROTATION_SRC_ADDR + i * 4, value);
	}
	RTT_PRINT("###############get_DMA_reg##########################\n");
	for (i = 0; i < 49; i++) {
		value = REG_RD(SPRD_DMA_BASE + i * 4);
		RTT_PRINT("DMA reg:0x%x, 0x%x.\n", SPRD_DMA_BASE + i * 4,
			  value);
	}
	for (i = 0; i < 8; i++) {
		value = REG_RD(SPRD_DMA_BASE + 0x6A0 + i * 4);
		RTT_PRINT("DMA chn 21 reg:0x%x, 0x%x.\n",
			  SPRD_DMA_BASE + 0x6A0 + i * 4, value);
	}
}
#endif
static void rot_k_done(void)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	rot_k_software_reset();
	rot_k_set_src_addr(s->s_addr);
	rot_k_set_dst_addr(s->d_addr);
	rot_k_set_img_size(&(s->img_size));
	rot_k_set_pixel_mode(s->pixel_format);
	rot_k_set_dir(s->angle);
	rot_k_set_UV_mode(s->uv_mode);
	rot_k_enable();
	RTT_PRINT("ok to rotation_done.\n");
#ifdef ROTATION_DEBUG
	rot_k_get_reg();
#endif
}

static int rot_k_set_UV_param(void)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	s->s_addr = s->src_addr.u_addr;
	s->d_addr = s->dst_addr.u_addr;
	s->img_size.w >>= 0x01;
	s->pixel_format = ROT_TWO_BYTES;
	if ((ROT_YUV422 == s->format)
	    && ((ROT_90 == s->angle)
		|| (ROT_270 == s->angle))) {
		s->uv_mode = ROT_UV422;
		s->img_size.h >>= 0x01;
	} else if (ROT_YUV420 == s->format) {
		s->img_size.h >>= 0x01;
	}
	return 0;
}

struct rot_user *rot_get_user(pid_t user_pid)
{
	struct rot_user *ret_user = NULL;
	int                      i;

	for (i = 0; i < ROT_USER_MAX; i ++) {
		if ((g_rot_user + i)->pid == user_pid) {
			ret_user = g_rot_user + i;
			break;
		}
	}

	if (ret_user == NULL) {
		for (i = 0; i < ROT_USER_MAX; i ++) {
			if ((g_rot_user + i)->pid == INVALID_USER_ID) {
				ret_user = g_rot_user + i;
				ret_user->pid = user_pid;
				break;
			}
		}
	}

	return ret_user;
}

static void rot_k_dma_irq(int dma_ch, void *dev_id)
{
	RTT_PRINT("%s, come\n", __func__ );
	g_is_rot_timeout = 0;
	condition = 1;
	wake_up(&wait_queue);
	RTT_PRINT("rotation_dma_irq X .\n");
}

int rot_k_dma_start(void)
{
	int ch_id = -1;
	struct sprd_dma_channel_desc dma_desc;
	DECLARE_ROTATION_PARAM_ENTRY(s);
	RTT_PRINT("rotation_dma_start E .\n");
	ch_id = sprd_dma_request(DMA_ROT, rot_k_dma_irq, &dma_desc);
	if (ch_id < 0) {
		RTT_PRINT("fail to sprd_request_dma.\n");
		return -EFAULT;
	}
	g_is_rot_timeout = 1;
	s->ch_id = ch_id;
	condition = 0;
	memset(&dma_desc, 0, sizeof(struct sprd_dma_channel_desc));
	dma_desc.llist_ptr = (dma_addr_t) 0x20800420;
	dma_desc.cfg_req_mode_sel = DMA_REQMODE_LIST;
	sprd_dma_channel_config(ch_id, DMA_LINKLIST, &dma_desc);
	sprd_dma_set_irq_type(ch_id, LINKLIST_DONE, 1);
	sprd_dma_start(ch_id);
	RTT_PRINT("rotation_dma_start X .\n");
	return 0;
}

int rot_k_dma_wait_stop(void)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);
	RTT_PRINT("rotation_dma_wait_stop E .\n");
	wait_event(wait_queue, condition);
	sprd_dma_stop(s->ch_id);
	sprd_dma_free(s->ch_id);
	RTT_PRINT("ok to rotation_dma_wait_stop.\n");
	if (g_is_rot_timeout) {
		return -1;
	} else {
		return 0;
	}
}

static int rot_k_thread(void *data_ptr)
{
	int ret = 0;
	struct rot_user *p_user = NULL;
	DECLARE_ROTATION_PARAM_ENTRY(s);

	while(1)
	{
		wait_event(thread_queue,  g_thread_run || kthread_should_stop());

		if (kthread_should_stop()){
			RTT_PRINT("rot_k_thread should stopped \n");
			break;
		}

		RTT_PRINT("rot_k_thread start \n");

		rot_k_cfg();
		ret = rot_k_dma_start();
		RTT_PRINT("rot_k_thread y start \n");

		if(0 == ret){
			rot_k_done();

			if (ROT_FALSE == s->is_end) {
				ret = rot_k_dma_wait_stop();
				if (ret) {
					printk("rot_k_thread y wait error \n");
					goto exit;
				}
				
				RTT_PRINT("rot_k_thread y done, uv start \n");
				
				ret = rot_k_dma_start();
				if (ret) {
					printk("rot_k_thread uv start error \n");
					goto exit;
				}
				rot_k_set_UV_param();
				rot_k_done();
				s->is_end = ROT_TRUE;
			}
			ret = rot_k_dma_wait_stop();
			if (ret) {
				printk("rot_k_thread  wait error \n");
				goto exit;
			}
			RTT_PRINT("rot_k_thread  done \n");
		}
		exit:
		g_thread_run = 0;
		atomic_set(&rot_cnt->start_flag, 0);
		rot_stop_timer(&rot_cnt->rot_timer);
		p_user = rot_get_user(cur_task_pid);
		up(&p_user->sem_done);
	}
	
	return ret;
}
int rot_k_start(void)
{
	int ret = 0;
	g_thread_run = 1;

	atomic_set(&rot_cnt->start_flag, 1);
	rot_start_timer(&rot_cnt->rot_timer,ROT_TIMEOUT);
	wake_up(&thread_queue);
	return ret;
}

static int rot_start_timer(struct timer_list *rot_timer, uint32_t time_val)
{
	int ret;
	ret = mod_timer(rot_timer, jiffies + msecs_to_jiffies(time_val));
	if (ret)
		printk("rot:Error in mod_timer\n");
	return 0;
}

static void rot_stop_timer(struct timer_list *rot_timer)
{
	del_timer_sync(rot_timer);
}

static void rot_timer_callback(unsigned long data)
{
	struct rot_user *p_user = NULL;
	if (1 == atomic_read(&rot_cnt->start_flag)) {
		printk("rot timeout.\n");
		condition = 1;
		wake_up(&wait_queue);
	}
}

static int rot_init_timer(struct timer_list *rot_timer)
{
	RTT_PRINT("Timer module installing\n");
	setup_timer(rot_timer, rot_timer_callback, 0);
	RTT_PRINT("Timer module installing e\n");
	return 0;
}

int rot_k_open(struct inode *node, struct file *file)
{
	struct rot_user *p_user = NULL;
	int ret = 0;

	down(&g_sem_dev_open);
	p_user = rot_get_user(current->pid);
	if (NULL == p_user) {
		printk("rot_k_open user cnt full  pid:%d. \n",current->pid);
		up(&g_sem_dev_open);
		return -1;
	}
	file->private_data = p_user;
	up(&g_sem_dev_open);

	return ret;
}

ssize_t rot_k_write(struct file *file, const char __user * u_data, size_t cnt, loff_t *cnt_ret)
{
	(void)file; (void)u_data; (void)cnt_ret;
	((struct rot_user *)(file->private_data))->is_exit_force = 1;
	up(&(((struct rot_user *)(file->private_data))->sem_done));

	return 1;
}


int rot_k_release(struct inode *node, struct file *file)
{
	((struct rot_user *)(file->private_data))->pid = INVALID_USER_ID;

	down(&g_sem_physical_ch);
	if (s_ch_id >= 0) {
		sprd_dma_free(s_ch_id);
		s_ch_id = -1;
	}
	up(&g_sem_physical_ch);

	down(&g_sem_virtual_ch);
	if (s_virtual_ch_id >= 0) {
		sprd_dma_free(s_virtual_ch_id);
		s_virtual_ch_id = -1;
	}
	up(&g_sem_virtual_ch);
	
	return 0;
}

static void rot_k_dma_copy_irq(int dma_ch, void *dev_id)
{
	RTT_PRINT("%s, come\n", __func__ );
	g_copy_done = 1;
	wake_up_interruptible(&wait_queue);
	RTT_PRINT("rotation_dma_irq X .\n");
}

static int rot_k_start_copy_data(ROT_CFG_T * param_ptr)
{
	struct sprd_dma_channel_desc dma_desc;

	uint32_t src_addr = param_ptr->src_addr.y_addr;
	uint32_t dst_addr = param_ptr->dst_addr.y_addr;
	uint32_t block_len;
	uint32_t total_len;
	int32_t ret = 0;

	RTT_PRINT("rotation_start_copy_data,w=%d,h=%d s!\n",param_ptr->img_size.w,param_ptr->img_size.h);
	if (ROT_YUV420 == param_ptr->format) {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 3 / 2;
	} else if (ROT_RGB888 == param_ptr->format || ROT_RGB666 == param_ptr->format) {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 4;
	} else {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 2;
	}
	total_len = block_len;

	down(&g_sem_physical_ch);
	if(s_ch_id < 0) {
		while (1) {
			s_ch_id =
			    sprd_dma_request(DMA_UID_SOFTWARE, rot_k_dma_copy_irq,
					     &dma_desc);
			if (s_ch_id < 0) {
				RTT_PRINT
				    ("SCALE: convert endian request dma fail.ret : %d.\n",
				     ret);
				msleep(5);
			} else {
				RTT_PRINT
				    ("SCALE: convert endian request dma OK. ch_id:%d,total_len=0x%x.\n",
				     s_ch_id, total_len);
				break;
			}
		}
	}

	g_copy_done = 0;
	memset(&dma_desc, 0, sizeof(struct sprd_dma_channel_desc));
	dma_desc.src_burst_mode = SRC_BURST_MODE_8;
	dma_desc.dst_burst_mode = SRC_BURST_MODE_8;
	dma_desc.cfg_src_data_width = DMA_SDATA_WIDTH32;
	dma_desc.cfg_dst_data_width = DMA_DDATA_WIDTH32;
	dma_desc.cfg_req_mode_sel = DMA_REQMODE_TRANS;
	dma_desc.total_len = total_len;
	dma_desc.cfg_blk_len = block_len;
	dma_desc.src_addr = src_addr;
	dma_desc.dst_addr = dst_addr;
	dma_desc.cfg_swt_mode_sel = 7 << 16;
	dma_desc.src_elem_postm = 0x0004;
	dma_desc.dst_elem_postm = 0x0004;
	sprd_dma_channel_config(s_ch_id, DMA_NORMAL, &dma_desc);
	sprd_dma_set_irq_type(s_ch_id, TRANSACTION_DONE, 1);
	RTT_PRINT("rotation_start_copy_data E!\n");
	sprd_dma_channel_start(s_ch_id);
	if (!wait_event_interruptible_timeout(wait_queue, g_copy_done,msecs_to_jiffies(30))) {

		printk("dma timeout. rot_k_start_copy_data  \n");
	}
	sprd_dma_channel_stop(s_ch_id);

	up(&g_sem_physical_ch);

	return ret;
}

static uint32_t user_va2pa(struct mm_struct *mm, uint32_t addr)
{
        pgd_t *pgd = pgd_offset(mm, addr);
        uint32_t pa = 0;

        if (!pgd_none(*pgd)) {
                pud_t *pud = pud_offset(pgd, addr);
                if (!pud_none(*pud)) {
                        pmd_t *pmd = pmd_offset(pud, addr);
                        if (!pmd_none(*pmd)) {
                                pte_t *ptep, pte;

                                ptep = pte_offset_map(pmd, addr);
                                pte = *ptep;
                                if (pte_present(pte))
                                        pa = pte_val(pte) & PAGE_MASK;
                                pte_unmap(ptep);
                        }
                }
        }

        return pa;
}

static int rot_k_start_copy_data_to_virtual(ROT_CFG_T * param_ptr)
{
	struct sprd_dma_channel_desc dma_desc;
	uint32_t dma_src_phy = param_ptr->src_addr.y_addr;
	uint32_t dst_vir_addr = param_ptr->dst_addr.y_addr;
	uint32_t dma_dst_phy;
	uint32_t block_len;
	uint32_t total_len;
	int32_t ret = 0;
	int i;
	uint32_t list_size;
	uint32_t list_copy_size = 4096;
	struct sprd_dma_linklist_desc *dma_cfg;
	dma_addr_t dma_cfg_phy;
	struct timeval time1, time2;

	if (ROT_YUV420 == param_ptr->format) {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 3 / 2;
	} else if (ROT_RGB888 == param_ptr->format || ROT_RGB666 == param_ptr->format) {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 4;
	} else {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 2;
	}

	total_len = block_len;

	if(0 != dst_vir_addr%list_copy_size){
		printk("rot_k_start_copy_data_to_virtual: dst_vir_addr = %x not 4K bytes align, error \n", dst_vir_addr);
		return -ENOMEM;
	}

	list_size = (total_len + list_copy_size -1)/list_copy_size;

	RTT_PRINT("rot_k_start_copy_data_to_virtual: dst_vir_addr = %x, list_copy_size=%x, list_size=%x,  \n", dst_vir_addr, list_copy_size, list_size);

	down(&g_sem_virtual_ch);
	if (s_virtual_ch_id < 0) {
		while (1) {
			s_virtual_ch_id = sprd_dma_request(DMA_UID_SOFTWARE, rot_k_dma_copy_irq, &dma_desc);
			if (s_virtual_ch_id < 0) {
				printk("rot_k_start_copy_data_to_virtual: convert endian request dma fail.ret : %d.\n", ret);
				msleep(5);
			} else {
				RTT_PRINT("rot_k_start_copy_data_to_virtual: convert endian request dma OK. ch_id:%d,total_len=0x%x.\n",
				     s_virtual_ch_id, total_len);
				break;
			}
		}
	}

	memset(&dma_desc, 0, sizeof(struct sprd_dma_channel_desc));

	dma_cfg = (struct sprd_dma_linklist_desc *)dma_alloc_writecombine(NULL,
										sizeof(*dma_cfg) * list_size,
										&dma_cfg_phy,
										GFP_KERNEL);
	if (!dma_cfg) {
		printk("rot_k_start_copy_data_to_virtual allocate failed, size=%d \n", sizeof(*dma_cfg) * list_size);
		up(&g_sem_virtual_ch);
		return -ENOMEM;
	}

	memset(dma_cfg, 0x0, sizeof(*dma_cfg) * list_size);

	do_gettimeofday(&time1);
	RTT_PRINT("pid = %d = 0x%x \n", current->pid, current->pid);
	for (i = 0; i < list_size; i++) {
		dma_dst_phy = user_va2pa(current->mm, dst_vir_addr+i*list_copy_size);
		//sprd_dma_default_linklist_setting(dma_cfg + i);
		dma_cfg[i].cfg = DMA_LIT_ENDIAN | DMA_SDATA_WIDTH32 | DMA_DDATA_WIDTH32 | DMA_REQMODE_LIST;
		dma_cfg[i].elem_postm = 0x4 << 16 | 0x4;
		dma_cfg[i].src_blk_postm = SRC_BURST_MODE_8;
		dma_cfg[i].dst_blk_postm = SRC_BURST_MODE_8;

		dma_cfg[i].llist_ptr = (u32) ((char *)dma_cfg_phy + sizeof(*dma_cfg) * (i + 1));
		dma_cfg[i].src_addr = dma_src_phy + i * list_copy_size;
		dma_cfg[i].dst_addr = dma_dst_phy;
		dma_cfg[i].total_len = (block_len > list_copy_size) ? list_copy_size : block_len;
		/* block length */
		dma_cfg[i].cfg |= list_copy_size & CFG_BLK_LEN_MASK;
		block_len -= dma_cfg[i].total_len;
	}
	do_gettimeofday(&time2);
	RTT_PRINT("rot_k_start_copy_data_to_virtual: virtual/physical convert time=%d \n",((time2.tv_sec-time1.tv_sec)*1000*1000+(time2.tv_usec-time1.tv_usec)));

	dma_cfg[list_size - 1].cfg |= DMA_LLEND;

	sprd_dma_linklist_config(s_virtual_ch_id, dma_cfg_phy);

	sprd_dma_set_irq_type(s_virtual_ch_id, LINKLIST_DONE, 1);

	g_copy_done = 0;

	sprd_dma_channel_start(s_virtual_ch_id);

	if (!wait_event_interruptible_timeout(wait_queue, g_copy_done,msecs_to_jiffies(30))) {
		printk("dma timeout.  rot_k_start_copy_data_to_virtual\n");
		sprd_dma_dump_regs();
	}

	sprd_dma_channel_stop(s_virtual_ch_id);

	dma_free_writecombine(NULL, sizeof(*dma_cfg) * list_size, dma_cfg, dma_cfg_phy);

	up(&g_sem_virtual_ch);

	RTT_PRINT("rot_k_start_copy_data_to_virtual done \n");

	return ret;
}

static int rot_k_start_copy_data_from_virtual(ROT_CFG_T * param_ptr)
{
	struct sprd_dma_channel_desc dma_desc;
	uint32_t dma_src_phy;
	uint32_t src_vir_addr = param_ptr->src_addr.y_addr;
	uint32_t dma_dst_phy = param_ptr->dst_addr.y_addr;
	uint32_t block_len;
	uint32_t total_len;
	int32_t ret = 0;
	int i;
	uint32_t list_size;
	uint32_t list_copy_size = 4096;
	struct sprd_dma_linklist_desc *dma_cfg;
	dma_addr_t dma_cfg_phy;
	struct timeval time1, time2;

	if (ROT_YUV420 == param_ptr->format) {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 3 / 2;
	} else if (ROT_RGB888 == param_ptr->format || ROT_RGB666 == param_ptr->format) {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 4;
	} else {
		block_len = param_ptr->img_size.w * param_ptr->img_size.h * 2;
	}

	total_len = block_len;

	if(0 != src_vir_addr%list_copy_size){
		printk("rot_k_start_copy_data_from_virtual: src_vir_addr = %x not 4K bytes align, error \n", src_vir_addr);
		return -ENOMEM;
	}

	list_size = (total_len + list_copy_size -1)/list_copy_size;

	RTT_PRINT("rot_k_start_copy_data_from_virtual: src_vir_addr = %x, list_copy_size=%x, list_size=%x,  \n", src_vir_addr, list_copy_size, list_size);

	down(&g_sem_virtual_ch);
	if (s_virtual_ch_id < 0) {
		while (1) {
			s_virtual_ch_id = sprd_dma_request(DMA_UID_SOFTWARE, rot_k_dma_copy_irq, &dma_desc);
			if (s_virtual_ch_id < 0) {
				printk("rot_k_start_copy_data_from_virtual: convert endian request dma fail.ret : %d.\n", ret);
				msleep(5);
			} else {
				RTT_PRINT("rot_k_start_copy_data_from_virtual: convert endian request dma OK. ch_id:%d,total_len=0x%x.\n",
				     s_virtual_ch_id, total_len);
				break;
			}
		}
	}

	memset(&dma_desc, 0, sizeof(struct sprd_dma_channel_desc));

	dma_cfg = (struct sprd_dma_linklist_desc *)dma_alloc_writecombine(NULL,
										sizeof(*dma_cfg) * list_size,
										&dma_cfg_phy,
										GFP_KERNEL);
	if (!dma_cfg) {
		printk("rot_k_start_copy_data_to_virtual allocate failed, size=%d \n", sizeof(*dma_cfg) * list_size);
		up(&g_sem_virtual_ch);
		return -ENOMEM;
	}

	memset(dma_cfg, 0x0, sizeof(*dma_cfg) * list_size);

	do_gettimeofday(&time1);
	RTT_PRINT("pid = %d = 0x%x \n", current->pid, current->pid);
	for (i = 0; i < list_size; i++) {
		dma_src_phy = user_va2pa(current->mm, src_vir_addr+i*list_copy_size);
		//sprd_dma_default_linklist_setting(dma_cfg + i);
		dma_cfg[i].cfg = DMA_LIT_ENDIAN | DMA_SDATA_WIDTH32 | DMA_DDATA_WIDTH32 | DMA_REQMODE_LIST;
		dma_cfg[i].elem_postm = 0x4 << 16 | 0x4;
		dma_cfg[i].src_blk_postm = SRC_BURST_MODE_8;
		dma_cfg[i].dst_blk_postm = SRC_BURST_MODE_8;

		dma_cfg[i].llist_ptr = (u32) ((char *)dma_cfg_phy + sizeof(*dma_cfg) * (i + 1));
		dma_cfg[i].src_addr = dma_src_phy;
		dma_cfg[i].dst_addr = dma_dst_phy + i * list_copy_size;
		dma_cfg[i].total_len = (block_len > list_copy_size) ? list_copy_size : block_len;
		/* block length */
		dma_cfg[i].cfg |= list_copy_size & CFG_BLK_LEN_MASK;
		block_len -= dma_cfg[i].total_len;
	}
	do_gettimeofday(&time2);
	RTT_PRINT("rot_k_start_copy_data_to_virtual: virtual/physical convert time=%d \n",((time2.tv_sec-time1.tv_sec)*1000*1000+(time2.tv_usec-time1.tv_usec)));

	dma_cfg[list_size - 1].cfg |= DMA_LLEND;

	sprd_dma_linklist_config(s_virtual_ch_id, dma_cfg_phy);

	sprd_dma_set_irq_type(s_virtual_ch_id, LINKLIST_DONE, 1);

	g_copy_done = 0;

	sprd_dma_channel_start(s_virtual_ch_id);

	if (!wait_event_interruptible_timeout(wait_queue, g_copy_done,msecs_to_jiffies(30))) {
		printk("dma timeout. rot_k_start_copy_data_from_virtual  \n");
	}

	sprd_dma_channel_stop(s_virtual_ch_id);

	dma_free_writecombine(NULL, sizeof(*dma_cfg) * list_size, dma_cfg, dma_cfg_phy);

	up(&g_sem_virtual_ch);

	RTT_PRINT("rot_k_start_copy_data_to_virtual done \n");

	return ret;
}

static int rot_k_io_cfg(ROT_CFG_T * param_ptr)
{
	int ret = 0;
	ROT_CFG_T *p = param_ptr;

	RTT_PRINT("rot_k_io_cfg start \n");
	RTT_PRINT("w=%d, h=%d \n", p->img_size.w, p->img_size.h);
	RTT_PRINT("format=%d, angle=%d \n", p->format, p->angle);
	RTT_PRINT("s.y=%x, s.u=%x, s.v=%x \n", p->src_addr.y_addr, p->src_addr.u_addr, p->src_addr.v_addr);
	RTT_PRINT("d.y=%x, d.u=%x, d.v=%x \n", p->dst_addr.y_addr, p->dst_addr.u_addr, p->dst_addr.v_addr);
	
	ret = rot_k_check_param(param_ptr);

	if(0 == ret)
		ret = rot_k_set_y_param(param_ptr);

	return ret;
}

static long rot_k_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	RTT_PRINT("rot_k_ioctl, 0x%x \n", cmd);

	switch (cmd) {
	case ROT_IO_CFG:
		down(&g_sem_rot);
		{
			int ret = 0;
			ROT_CFG_T params;

			cur_task_pid =  ((struct rot_user *)(file->private_data))->pid;
			((struct rot_user *)(file->private_data))->is_rot_enable = 1;

			ret = copy_from_user(&params, (ROT_CFG_T *) arg, sizeof(ROT_CFG_T));
			if (0 == ret){
				ret = rot_k_io_cfg(&params);
			}

			if(ret) {
				printk("rot_k_ioctl  1 fail.\n");
				up(&g_sem_rot);
			}

			RTT_PRINT("rot_k_ioctl, ROT_IO_CFG, %d \n", ret);
			return ret;
		}
	
	case ROT_IO_START:
		{
			int ret = 0;

			if (rot_k_start()) {
				ret = -EFAULT;
				up(&g_sem_rot);
			}

			RTT_PRINT("rot_k_ioctl, ROT_IO_START, %d \n", ret);
			return ret;
		}

	case ROT_IO_IS_DONE:
		{
			down(&(((struct rot_user *)(file->private_data))->sem_done));
			{
				int ret = 0;
				if (((struct rot_user *)(file->private_data))->is_exit_force) {
					((struct rot_user *)(file->private_data))->is_exit_force = 0;
					ret = -1;
				}

				if(((struct rot_user *)(file->private_data))->is_rot_enable) {
					((struct rot_user *)(file->private_data))->is_rot_enable = 0;
					if (g_is_rot_timeout)
						ret = -1;
					up(&g_sem_rot);
				}

				return ret;
			}
		}

	case ROT_IO_DATA_COPY:
		down(&g_sem_copy);
		{
			int ret = 0;
			ROT_CFG_T params;

			ret = copy_from_user(&params, (ROT_CFG_T *) arg, sizeof(ROT_CFG_T));
			if (0 == ret){
				if (rot_k_start_copy_data(&params)) {
					ret = -EFAULT;
				}
			}
			up(&g_sem_copy);
			return ret;
		}

	case ROT_IO_DATA_COPY_TO_VIRTUAL:
		down(&g_sem_copy);
		{
			int ret = 0;
			ROT_CFG_T params;

			ret = copy_from_user(&params, (ROT_CFG_T *) arg, sizeof(ROT_CFG_T));
			if (0 == ret){
				if (rot_k_start_copy_data_to_virtual(&params)) {
					ret = -EFAULT;
				}
			}
			up(&g_sem_copy);
			return ret;
		}

	case ROT_IO_DATA_COPY_FROM_VIRTUAL:
		down(&g_sem_copy);
		{
			int ret = 0;
			ROT_CFG_T params;

			ret = copy_from_user(&params, (ROT_CFG_T *) arg, sizeof(ROT_CFG_T));
			if (0 == ret){
				if (rot_k_start_copy_data_from_virtual(&params)) {
					ret = -EFAULT;
				}
			}
			up(&g_sem_copy);
			return ret;
		}

	default:
		return 0;
	}

}

static struct file_operations rotation_fops = {
	.owner = THIS_MODULE,
	.open = rot_k_open,
	.write = rot_k_write,
	.unlocked_ioctl = rot_k_ioctl,
	.release = rot_k_release,
};

static struct miscdevice rotation_dev = {
	.minor = ROTATION_MINOR,
	.name = "sprd_rotation",
	.fops = &rotation_fops,
};

int rot_k_probe(struct platform_device *pdev)
{
	int ret, i;
	struct rot_user *p_user;
	printk(KERN_ALERT "rot_k_probe called\n");

	ret = misc_register(&rotation_dev);
	if (ret) {
		printk(KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
		       ROTATION_MINOR, ret);
		return ret;
	}

	g_rot_user = kzalloc(ROT_USER_MAX * sizeof(struct rot_user), GFP_KERNEL);
	if (NULL == g_rot_user) {
		printk("rot_user, no mem");
		return -1;
	}

	init_waitqueue_head(&wait_queue);
	init_waitqueue_head(&thread_queue);
	rot_init_timer(&rot_cnt->rot_timer);
	p_user = g_rot_user;
	for (i =  0; i < ROT_USER_MAX; i++) {
		p_user->pid = INVALID_USER_ID;
		p_user->is_exit_force = 0;
		p_user->is_rot_enable = 0;
		sema_init(&p_user->sem_done, 0);
		p_user ++;
	}

	g_rot_task = kthread_create(rot_k_thread, NULL, "rot thread");
	if (g_rot_task == 0) {
		printk("rot: create thread error \n");
		kfree(g_rot_user);
		g_rot_user = NULL;
		return -1;
	}else{
		wake_up_process(g_rot_task);
	}

	printk(KERN_ALERT " rot_k_probe Success\n");
	return 0;
}

static int rot_k_remove(struct platform_device *dev)
{
	printk(KERN_INFO "rot_k_remove called !\n");
	if (g_rot_user) {
		kfree(g_rot_user);
	}
	misc_deregister(&rotation_dev);
	printk(KERN_INFO "rot_k_remove Success !\n");
	return 0;
}

static struct platform_driver rotation_driver = {
	.probe = rot_k_probe,
	.remove = rot_k_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sprd_rotation",
		   },
};

int __init rot_k_init(void)
{
	printk(KERN_INFO "rot_k_init called !\n");
	if (platform_driver_register(&rotation_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}
	sema_init(&g_sem_dev_open, 1);
	sema_init(&g_sem_rot, 1);
	sema_init(&g_sem_copy, 1);
	sema_init(&g_sem_physical_ch, 1);
	sema_init(&g_sem_virtual_ch, 1);
	return 0;
}

void rot_k_exit(void)
{
	printk(KERN_INFO "rot_k_exit called !\n");
	platform_driver_unregister(&rotation_driver);
}

module_init(rot_k_init);
module_exit(rot_k_exit);

MODULE_DESCRIPTION("rotation Driver");
MODULE_LICENSE("GPL");
