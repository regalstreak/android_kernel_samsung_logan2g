#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <mach/regs_glb.h>
#include <mach/regs_ahb.h>
#include <asm/irqflags.h>
//#include "clock_common.h"
//#include "clock_sc8810.h"
#include <mach/adi.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <mach/pm_debug.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/regs_ana_glb.h>
#include <asm/hardware/gic.h>
#include <mach/emc_repower.h>
#include <mach/sci.h>
#include <mach/regs_emc.h>
#include <linux/earlysuspend.h>
#include <mach/emc_change_freq_data.h>
//static struct kobject *emc_freq_kobj;
static u32 last_freq_set = -1;
static DECLARE_COMPLETION(my_completion);
static DEFINE_MUTEX(memfreq_mutex);
static int emc_write_data_to_cp(void);
static struct wake_lock emc_freq_wakelock;
extern volatile unsigned int memfreq_bypass;
extern volatile unsigned int memfreq_curr;

#define DMC_CLK_400M		0
#define DMC_CLK_200M		1
#define DMC_CLK_300M		0
#define DMC_EARLY_SUSPEND	3
#define DMC_EARLY_RESUME	4
#define DMC_CONNECT_USB		0

extern int memfreq_usb_state;

void wait_emc_freq_complete(u32 mask)
{
	u32 value;
	u32 times;
	printk("wait_emc_freq_complete\n");
	value = sci_glb_read(REG_AHB_ARM_CLK, -1UL);
	times = 0;
	/*judge the if emc freq has changed*/
	while((value & (1 << 8)) != mask) {
		value = sci_glb_read(REG_AHB_ARM_CLK, -1UL);
		mdelay(5);
		if(times >= 10) {
			break;
		}
		times ++;
	}
	printk("wait_emc_freq_complete armclk= %x, dpll = %x\n", sci_glb_read(REG_AHB_ARM_CLK, -1UL), sci_glb_read(REG_GLB_D_PLL_CTL, -1UL));
}

int check_early_memfreq;
static void emc_earlysuspend_early_suspend(struct early_suspend *h)
{
	if(memfreq_usb_state == 0)
	{
	last_freq_set = DMC_EARLY_SUSPEND;
	printk("****DDRDFS**** emc_earlysuspend_early_suspend\n");
	emc_write_data_to_cp();
	wait_emc_freq_complete(1 << 8);
	//complete(&my_completion);
		check_early_memfreq = 1;
}
	else
		printk("****DDRDFS**** emc_earlysuspend_early_suspend(USB)\n");
}
static void emc_earlysuspend_late_resume(struct early_suspend *h)
{
	if(memfreq_usb_state == 0 || check_early_memfreq == 1)
	{
	last_freq_set = DMC_EARLY_RESUME;
	printk("****DDRDFS**** emc_earlysuspend_late_resume\n");
	emc_write_data_to_cp();
	wait_emc_freq_complete(0);
	//complete(&my_completion);
		check_early_memfreq = 0;
}
	else
		printk("****DDRDFS**** emc_earlysuspend_late_resume(USB)\n");

}
static struct early_suspend emc_early_suspend_desc = {
	.level = 0, //EARLY_SUSPEND_LEVEL_DISABLE_FB + 100,
	.suspend = emc_earlysuspend_early_suspend,
	.resume = emc_earlysuspend_late_resume,
};
#define emc_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0620,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}
static ssize_t emc_freq_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	if(!memfreq_curr){
		return sprintf(buf,"400mhz\n");
	}
	return sprintf(buf,"%umhz\n",memfreq_curr);
}
static ssize_t emc_freq_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
#ifdef CONFIG_NKERNEL
#ifndef CONFIG_MACH_SP6825GA
	u32 freq;
	switch(buf[0]) {
	case '0': /*400MHz*/
		freq = DMC_CLK_400M;
		printk("****DDRDFS**** emc_freq_store 0\n");
		break;
	case '1' : /*200MHz*/
		freq = DMC_CLK_200M;
		printk("****DDRDFS**** emc_freq_store 1\n");
		break;
#if 0
	case '2' : /*300MHz*/
		freq = DMC_CLK_300M;
		break;
#endif
	default:
		freq = -1;
		break;
	}
	if(freq != -1) {
		printk("emc_freq_store %x\n", last_freq_set);
		last_freq_set = freq;
		complete(&my_completion);
	}
#endif
#endif
	return 1;
}

static ssize_t emc_freq_bypass_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
    if(memfreq_bypass)
		return sprintf(buf,"%d:means off\n",memfreq_bypass);
	else
		return sprintf(buf,"%d:means on\n",memfreq_bypass);
}


static ssize_t emc_freq_bypass_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	switch(buf[0]) {
	case '0':
		printk("****DDRDFS**** emc_freq_bypass_store 0\n");
		mutex_lock(&memfreq_mutex);
		memfreq_bypass = 0;
		mutex_unlock(&memfreq_mutex);
		break;
	case '1' :
		printk("****DDRDFS**** emc_freq_bypass_store 1\n");
		mutex_lock(&memfreq_mutex);
		memfreq_bypass = 1;
		mutex_unlock(&memfreq_mutex);
		break;
	default:
		break;
	}
	return 1;
}

u32 emc_freq_get(void)
{
	u32 freq = 0;
	u32 n = 0;
	u32 div = 0;
	n = sci_glb_read(REG_GLB_D_PLL_CTL, -1);
	n &= 0x7ff;
	div = sci_glb_read(REG_AHB_ARM_CLK, -1);
	div &= 0xf00;
	div >>= 8;
	freq = (n * 4) / (div + 1);
	return freq;
}
void emc_freq_set(u32 freq)
{
	switch(freq) {
	case 200:
		freq = DMC_CLK_200M;
		break;
	case 300:
		freq = DMC_CLK_300M;
		break;
	case 400:
		freq = DMC_CLK_400M;
		break;
	default:
		return;
	}

	last_freq_set = freq;
	complete(&my_completion);
}
emc_attr(emc_freq);
emc_attr(emc_freq_bypass);
static struct attribute * g[] = {
	&emc_freq_attr.attr,
	&emc_freq_bypass_attr.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = g,
};
#define EMC_FREQ_MINOR	8
int ex_open_k (unsigned int minor);
int ex_release_k (unsigned int minor);
ssize_t ex_read_k (unsigned int minor, char * buf, size_t count);
ssize_t ex_write_k (unsigned int minor, const char * buf,size_t count);
static int emc_write_data_to_cp(void)
{
	int size;
	u8 data;
	#define OFF 0
	#define ON 1
	static int pipe_state = OFF;
	data = last_freq_set;
	wake_lock(&emc_freq_wakelock);
	for(;;) {
		if(pipe_state != ON) {
			int ret;
			ret = ex_open_k (EMC_FREQ_MINOR);
			printk("emc_vbpipe_thread ret = %x\n", ret);
			if(ret < 0) {
				/* we won't be able to work in this case */
				return ret;
			} else {
				pipe_state = ON;
			}
		}
		size = ex_write_k(EMC_FREQ_MINOR,&data, 1);

		printk("****DDRDFS**** emc_write_data_to_cp  last_freq_set = %x\n", last_freq_set);
		printk("emc_vbpipe_thread size = %x freq %x\n", size, last_freq_set);
		if(size < 0) {
			/* something wrong, write again */
			if ( size == -EPIPE ) {
				/* peer side is down, we need to reopen the pipe */
				printk("emc_vbpipe [%d] is down, reopen it.\n", EMC_FREQ_MINOR);
				ex_release_k(EMC_FREQ_MINOR);
				pipe_state = OFF;
			} else {
				/* something wrong, write again */
				continue;
			}
			continue;
		} else if ( size == 0) {
			/* peer side is down, we need to reopen the pipe */
			printk("emc_vbpipe [%d] is down, reopen it.\n", EMC_FREQ_MINOR);
			ex_release_k(EMC_FREQ_MINOR);
			pipe_state = OFF;
		} else {
			break;
		}
		msleep(100);
	}
	wake_unlock(&emc_freq_wakelock);
	return 0;
}
static int emc_vbpipe_thread(void * context)
{
	while(1){
		wait_for_completion(&my_completion);
		printk("emc_vbpipe_thread event on %x\n", last_freq_set);
		if(last_freq_set != -1) {
			emc_write_data_to_cp();
		}
	}
	return 0;
}
static void vbpipe_open(void)
{
	struct task_struct * task;
	task = kthread_create(emc_vbpipe_thread, NULL, "emc_vbpipe");
	if (task == 0) {
		printk("Can't crate emc_vbpipe thread!\n");
	}
	else {
		wake_up_process(task);
	}
	wake_lock_init(&emc_freq_wakelock, WAKE_LOCK_SUSPEND, "emc_freq_wakelock");
}
struct kobject *emc_kobj;
void cp_code_init(void)
{
	u32 cp_iram_addr;
	cp_iram_addr = (u32)ioremap(0x30000, 0x1000);
	if(cp_iram_addr == 0) {
		printk("cp_code_init remap cp iram erro\n");
	}
	memcpy((void *)cp_iram_addr, cp_code_data, sizeof(cp_code_data));
	iounmap((void *)cp_iram_addr);
}
static int __init emc_early_suspend_init(void)
{
#ifdef CONFIG_NKERNEL
#ifndef CONFIG_MACH_SP6825GA
	/*set cp ahb clk div to 2*/
	sci_glb_set(REG_AHB_CP_AHB_ARM_CLK, 1 << 4);
	/*set cp arm clk to 330MHz*/
	sci_glb_clr(REG_AHB_CP_AHB_ARM_CLK, 3 << 23);
	register_early_suspend(&emc_early_suspend_desc);
	cp_code_init();
	vbpipe_open();
#endif
#endif
	emc_kobj = kobject_create_and_add("emc", NULL);
	if (!emc_kobj)
		return -ENOMEM;
	return sysfs_create_group(emc_kobj, &attr_group);
}
static void  __exit emc_early_suspend_exit(void)
{
#ifdef CONFIG_NKERNEL
#ifndef CONFIG_MACH_SP6825GA
	unregister_early_suspend(&emc_early_suspend_desc);
#endif
#endif
}

module_init(emc_early_suspend_init);
module_exit(emc_early_suspend_exit);
