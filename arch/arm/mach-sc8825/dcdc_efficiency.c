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
#include <mach/adi.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/regs_ana_glb.h>
#include <asm/hardware/gic.h>
#include <mach/emc_repower.h>
#include <mach/sci.h>
#include <mach/regs_emc.h>
#include <linux/earlysuspend.h>


static DEFINE_MUTEX(dcdc_efficiency_mutex);
static u32 dcdc_efficiency_level = 3;
#define dcdc_efficiency_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0620,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}
static ssize_t dcdc_efficiency_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	return 1;
}
static u32 get_dcdc_efficiency(void)
{
	u32 level;
	u32 reg;
	mutex_lock(&dcdc_efficiency_mutex);
	reg = sci_adi_read(ANA_REG_GLB_DCDC_CTRL1);
	level = reg & BITS_DCDC_PDRSLOW(0xf);
	level >>= 0x8;
	mutex_unlock(&dcdc_efficiency_mutex);
	return level;
}
static void do_dcdc_change_efficiency(u32 level)
{
	u32 set_bits;
	u32 clr_bits;
	printk("do_dcdc_change_efficiency %x\n", level);
	mutex_lock(&dcdc_efficiency_mutex);
	if(level != dcdc_efficiency_level) {
		set_bits = (level << 8) & (0xf << 8);
		set_bits |= ((~level) << 12) & (0xf << 12);
		clr_bits = (~(level << 8)) & (0xf << 8);
		clr_bits |= (level << 12) & (0xf << 12);	
		printk("set_bits %x, clr_bits %x", set_bits, clr_bits);

		sci_adi_write(ANA_REG_GLB_DCDC_CTRL1, set_bits, clr_bits);
		sci_adi_write(ANA_REG_GLB_DCDCARM_CTRL1, set_bits, clr_bits);
		sci_adi_write(ANA_REG_GLB_DCDCMEM_CTRL1, set_bits, clr_bits);
		sci_adi_write(ANA_REG_GLB_DCDCLDO_CTRL1, set_bits, clr_bits);
		printk("ANA_REG_GLB_DCDC_CTRL1 %x\n", sci_adi_read(ANA_REG_GLB_DCDC_CTRL1));
		printk("ANA_REG_GLB_DCDCARM_CTRL1 %x\n", sci_adi_read(ANA_REG_GLB_DCDCARM_CTRL1));
		printk("ANA_REG_GLB_DCDCMEM_CTRL1 %x\n", sci_adi_read(ANA_REG_GLB_DCDCMEM_CTRL1));
		printk("ANA_REG_GLB_DCDCLDO_CTRL1 %x\n", sci_adi_read(ANA_REG_GLB_DCDCLDO_CTRL1));
		dcdc_efficiency_level = level;
	}
	mutex_unlock(&dcdc_efficiency_mutex);
}
static ssize_t dcdc_efficiency_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	u32 level = 0;
	switch(buf[0]) {
	case '0': /*DCDC level 0*/
		level = 0;
		break;
	case '1' : /*DCDC level 1*/
		level = 1;
		break;
	case '2' : /*DCDC level 2*/
		level = 2;
		break;
	case '3': /*DCDCD level 3*/
		level = 3;
		break;
	default: /*DCDC level 3*/
		return 1; //do nothing
	}
	do_dcdc_change_efficiency(level);
	return 1;
}
dcdc_efficiency_attr(dcdc_efficiency);
static struct attribute * g[] = {
	&dcdc_efficiency_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};
struct kobject *dcdc_efficiency_kobj;
static int __init dcdc_efficiency_init(void)
{
	printk("dcdc_efficiency_init = %x", dcdc_efficiency_level);
	dcdc_efficiency_kobj = kobject_create_and_add("dcdc_efficiency", NULL);
	if (!dcdc_efficiency_kobj)
		return -ENOMEM;
	return sysfs_create_group(dcdc_efficiency_kobj, &attr_group);
}
static void  __exit dcdc_efficiency_exit(void)
{
}
module_init(dcdc_efficiency_init);
module_exit(dcdc_efficiency_exit);
