 /*****************************************************************************
 *
 * Title: Linux Device Driver for Proximity Sensor GP2AP002S00F
 * COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2015 ALL RIGHTS RESERVED
 *
 *****************************************************************************/
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <mach/hardware.h>
#include <asm/gpio.h>  
#include <linux/pm.h>
 
#include <linux/gp2a_dev.h> 
#include <linux/gp2a_prox.h>
#include <linux/regulator/consumer.h>
#include <mach/pinmap.h>
#include <linux/uaccess.h>

#define GP2A_DEBUG 1
#define error(fmt, arg...) printk("--------" fmt "\n", ##arg)
#if GP2A_DEBUG
#define PROXDBG(fmt, args...) printk(KERN_INFO fmt, ## args)
#define debug(fmt, arg...) printk("--------" fmt "\n", ##arg)
#else
#define PROXDBG(fmt, args...)
#define debug(fmt,arg...)
#endif

#define PROX_NONDETECT	0x40
#define PROX_DETECT		0x20


#ifdef GP2AP002X_PROXIMITY_OFFSET
#define PROX_NONDETECT_MODE1	0x43
#define PROX_DETECT_MODE1		0x28
#define PROX_NONDETECT_MODE2	0x48
#define PROX_DETECT_MODE2		0x42
#define OFFSET_FILE_PATH	"/efs/sensor/prox_cal"
#endif

#define CHIP_DEV_NAME	"GP2AP002"
#define CHIP_DEV_VENDOR	"SHARP"

static int nondetect;
static int detect;

//static struct regulator *prox_regulator=NULL;
extern struct class *sensors_class;
struct device *proximity_dev;
//static bool prox_power_mode = false;
static short prox_value_cnt = 0;
static struct file_operations gp2a_prox_fops = {
	.owner  	= THIS_MODULE,
	.open   	= gp2a_prox_open,
	.release 	= gp2a_prox_release,    
	.unlocked_ioctl 	= gp2a_prox_ioctl,
};

static struct miscdevice gp2a_prox_misc_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "proximity",
    .fops   = &gp2a_prox_fops,
};

static struct regulator *prox_led;
static void prox_ctrl_regulator(int on_off)
{
	if(on_off)
	{
		//LDO Power On=============
		if (!prox_led) {
			prox_led = regulator_get(NULL, "vddcammot");
			if (IS_ERR(prox_led)) {
				pr_err("[GP2A] %s regulator get error!\n", __func__);
				prox_led = NULL;
				return;
			}
		}
		regulator_set_voltage(prox_led, 3300000, 3300000);
		regulator_enable(prox_led);
		msleep(2);
		printk("[GP2A] : %s ON \n",__func__);       
	}
	else
	{
		regulator_disable(prox_led);        
		printk("[GP2A] : %s OFF \n",__func__);                            
	}
}

static int gp2a_prox_open(struct inode *ip, struct file *fp)
{
	debug("%s called",__func__);
	return nonseekable_open(ip, fp);	
}

static int gp2a_prox_release(struct inode *ip, struct file *fp)
{	
	debug("%s called",__func__);
	return 0;
}

static long gp2a_prox_ioctl(struct file *filp, unsigned int ioctl_cmd,  unsigned long arg)
{	
	int ret = 0;

	if( _IOC_TYPE(ioctl_cmd) != PROX_IOC_MAGIC )
	{
		error("Wrong _IOC_TYPE 0x%x",ioctl_cmd);
		return -ENOTTY;
	}
	if( _IOC_NR(ioctl_cmd) > PROX_IOC_NR_MAX )
	{
		error("Wrong _IOC_NR 0x%x",ioctl_cmd);	
		return -ENOTTY;
	}
	switch (ioctl_cmd)
	{
	case PROX_IOC_NORMAL_MODE:
	{
		printk("[GP2A] PROX_IOC_NORMAL_MODE called\n");
		if(0==gp2a_data->val_state)
		{
			if( (ret = gp2a_prox_mode(1)) < 0 )        
				error("PROX_IOC_NORMAL_MODE failed"); 
		}
            else
			debug("Proximity Sensor is already Normal Mode");
		break;
	}
	case PROX_IOC_SHUTDOWN_MODE:			
	{
		printk("[GP2A] PROX_IOC_SHUTDOWN_MODE called\n");				
		if(1==gp2a_data->val_state)
		{
			if( (ret = gp2a_prox_mode(0)) < 0 )        
				error("PROX_IOC_SHUTDOWN_MODE failed"); 
		}
		else
			debug("Proximity Sensor is already set in Shutdown mode");
		break;
	}
	default:
		error("Unknown IOCTL command");
		ret = -ENOTTY;
		break;
	}
	return ret;
}

#if USE_INTERRUPT
/*Only One Read Only register, so word address need not be specified (from Data Sheet)*/
static int gp2a_i2c_read(u8 reg, u8 *value)
{
	int ret =0;
	int count=0;
	u8 buf[3];
	struct i2c_msg msg[1];

	buf[0] = reg;
	
	/*first byte read(buf[0]) is dummy read*/
	msg[0].addr = gp2a_data->gp2a_prox_i2c_client->addr;
	msg[0].flags = I2C_M_RD;	
	msg[0].len = 2;
	msg[0].buf = buf;
	count = i2c_transfer(gp2a_data->gp2a_prox_i2c_client->adapter, msg, 1);
	
	if(count < 0)
	{
		debug("%s %d i2c transfer error\n", __func__, __LINE__);
		ret =-1;
	}
	else
	{
		*value = buf[0] << 8 | buf[1];
		debug("value=%d", *value);
	}
	
	return ret;	
}
#endif

static int gp2a_i2c_write( u8 reg, u8 *value )
{
	int ret =0;
	int count=0;
	struct i2c_msg msg[1];
	u8 data[2];

	if( (gp2a_data->gp2a_prox_i2c_client == NULL) || (!gp2a_data->gp2a_prox_i2c_client->adapter) )
		return -ENODEV;

	data[0] = reg;
	data[1] = *value;

	msg[0].addr = gp2a_data->gp2a_prox_i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf 	= data;
	count = i2c_transfer(gp2a_data->gp2a_prox_i2c_client->adapter,msg,1);

	if(count < 0)
		ret =-1;
	
	return ret;
}
/* 
* Operation Mode B
* Initiate operation
*/
static int gp2a_prox_reset(void)
{
	u8 reg_value;
	int ret=0;
	debug("[GP2A] %s called\n",__func__); 
	/* 
	    Procedure 1: After Power Supply is turned on, setup the following register
	    01H GAIN - 0x08
	    02H HYS - 0x40
	    03H CYCLE - 0x04
	    04H OPMOD - 0x03
	    Procedure 2: Permit host's interrupt input
	    Procedure 3 : VOUT terminal changes from "H" to "L"
	    Procedure 4 : Forbit host's interrupt input
	    Procedure 5 : Read VO value through I2C bus interface, VO=0 : no detection, VO=1 : detection
	    Procedure 6 : Write HYS register through I2C bus interface
	    Procedure 7 : Set 06H CON register as follows, forcing VOUT terminal to go H.
	    06H CON - 0x18
	    Procedure 8 : Permit host's interrupt input
	    Prodecure 9 : Set 06H CON register as follows, enabling VOUT terminal in normal operation
	    06H CON - 0x00
	    Prodecure 10 : Repeat procedures from 3 to 9
	*/
	reg_value = 0x18;
	if((ret=gp2a_i2c_write(GP2A_REG_CON/*0x06*/,&reg_value))<0)
		error("gp2a_i2c_write 4 failed\n");	
    
	reg_value = 0x08;
	if((ret=gp2a_i2c_write(GP2A_REG_GAIN/*0x01*/,&reg_value))<0)
		error("gp2a_i2c_write 1 failed\n");

#if defined(CONFIG_SENSORS_TOTORO) //mode B2 : Internal receiver sensitivity is now 2 times higher than B1
	reg_value = 0x2F;//B15 //B2: 0x20;
#else
	reg_value = PROX_NONDETECT;
#endif
	if((ret=gp2a_i2c_write(GP2A_REG_HYS/*0x02*/,&reg_value))<0)
		error("gp2a_i2c_write 2 failed\n");

	reg_value = 0x04;
	if((ret=gp2a_i2c_write(GP2A_REG_CYCLE/*0x03*/,&reg_value))<0)
		error("gp2a_i2c_write 3 failed\n");
	
	return ret;
}

/* 
 * Setting GP2AP002S00F proximity sensor operation mode, 
 * enable=1-->Normal Operation Mode
 * enable=0-->Shutdown Mode 
 */
static int gp2a_prox_mode(int enable)
{	
	u8 reg_value;
	int ret=0;
	debug("%s called\n",__func__); 
    
	if(1==enable)
	{
/*
* To go back operation
        Procedure 1 : Set 06H CON register as following, forcing VOUT terminal to go H
        06H CON - 0x18        
        Procedure 2 : Set 02H HYS register as follows, preparing VO reset to 0
        02H HYS - 0x40
        Procedure 3 : Release from shutdown
        04H OPMOD - 0x03
        Procedure 4 : Permit host's interrupt input
        Prodecure 5 : Set 06H CON register as follows, enabling VOUT terminal in normal operation
        06H CON - 0x00
*/
		//prox_ctrl_regulator(1);   //not used

		//err = set_irq_wake(gp2a_data->irq, 1);
		//if (err)
			//error("set_irq_wake[1] failed");

		reg_value = 0x18;
		if((ret=gp2a_i2c_write(GP2A_REG_CON,&reg_value))<0)
			error("gp2a_i2c_write 1 failed");
		
#if defined(CONFIG_SENSORS_TOTORO) //mode B2 : Internal receiver sensitivity is now 2 times higher than B1		
		reg_value = 0x2F;//B15 //B2: 0x20;
#else
		reg_value = 0x40;
#endif
		if((ret=gp2a_i2c_write(GP2A_REG_HYS,&reg_value))<0)
			error("gp2a_i2c_write 2 failed");
		
		reg_value = 0x03;
		if((ret=gp2a_i2c_write(GP2A_REG_OPMOD,&reg_value))<0)
			error("gp2a_i2c_write 3 failed");

		enable_irq(gp2a_data->irq);
		
		reg_value = 0x00;
		if((ret=gp2a_i2c_write(GP2A_REG_CON,&reg_value))<0)
			error("gp2a_i2c_write 4 failed");
		
		gp2a_data->val_state=1;
	}
	else 
	{
/*
* Software Shutdown
        04H OPMOD - 0x02
*/
		//err = set_irq_wake(gp2a_data->irq, 0);
		//if (err)
			//error("set_irq_wake[0] failed");

		disable_irq_nosync(gp2a_data->irq);

		reg_value = 0x02;
		if((ret=gp2a_i2c_write(GP2A_REG_OPMOD,&reg_value))<0)
			error("gp2a_i2c_write 3 failed");
		
		gp2a_data->val_state=0;
		prox_value_cnt = 0;
#ifdef AK8973_MAGNETIC_SENSORS		
		proximity_value = 0;
#endif

		//prox_ctrl_regulator(0);   //not used
	}   
	
	return ret;
}

/* 
 * PS_OUT =0, when object is near
 * PS_OUT =1, when object is far
 */
/*
 * get_gp2a_proximity_value() is called by magnetic sensor driver(ak8973)
 * for reading proximity value.
 */
#ifdef AK8973_MAGNETIC_SENSORS
int gp2a_get_proximity_value(void)
{
	PROXDBG("[GP2A] gp2a_get_proximity_value called : %d\n",proximity_value); 

	return ((proximity_value == 1)? 0:1);

}
EXPORT_SYMBOL(gp2a_get_proximity_value);
#endif

#if 0
static void gp2a_chip_init(void)
{
	debug("%s called",__func__); 	
}
#endif

#if USE_INTERRUPT
static void gp2a_prox_work_func(struct work_struct *work)
{
	unsigned char value;
	unsigned char int_val = GP2A_REG_PROX;
	unsigned char vout = 0;
        int ret=0;

	/* Read VO & INT Clear */	
	debug("[PROXIMITY] %s : \n",__func__);
    
	if(INT_CLEAR)
	{
		//int_val = GP2A_REG_PROX | (1 <<7);
	}
	
	if((ret=gp2a_i2c_read((u8)(int_val), &value))<0)
	{
		error("gp2a_i2c_read  failed\n");            
		gp2a_prox_reset();
            
		if(gp2a_data->val_state == 1)
			gp2a_prox_mode(1);
		else
			gp2a_prox_mode(0);
            
		return;
	}
    
	vout = value & 0x01;
	printk(KERN_INFO "[GP2A] vout = %d \n",vout);

#ifdef AK8973_MAGNETIC_SENSORS
	/* Report proximity information */
	proximity_value = vout;
    
#ifdef TIMEOUT_WAKELOCK	 //not used
	if(proximity_value ==0)
	{
		struct timespec ts;
        
		timeB = ktime_get();
		
		timeSub = ktime_sub(timeB,timeA);
		debug("[PROXIMITY] timeSub sec = %d, timeSub nsec = %d \n",timeSub.tv.sec,timeSub.tv.nsec);
		
		//if (timeSub.tv.sec>=3 )
		ts = ktime_to_timespec(timeSub);		
		if (ts.tv_sec >=3 )
		{
			wake_lock_timeout(&prx_wake_lock,HZ/2);
			debug("[PROXIMITY] wake_lock_timeout : HZ/2 \n");
		}
		else
			error("[PROXIMITY] wake_lock is already set \n");
	}
#endif
#endif

	input_report_abs(gp2a_data->prox_input_dev, ABS_DISTANCE,((vout == 1)? 0:1));
	input_sync(gp2a_data->prox_input_dev);
	mdelay(1);

	/* Write HYS Register */
#if defined(CONFIG_SENSORS_TOTORO) //mode B2 : Internal receiver sensitivity is now 2 times higher than B1		    
	if(!vout)
		value = 0x2F;//B15 //B2:0x20;
	else
		value = 0x0F;//B15 //B2:0x00;
#else
	if(!vout)
		value = PROX_NONDETECT;
	else
		value = PROX_DETECT;
#endif    	
	gp2a_i2c_write((u8)(GP2A_REG_HYS),&value);

	/* Forcing vout terminal to go high */
	value = 0x18;
	gp2a_i2c_write((u8)(GP2A_REG_CON),&value);

	/* enable INT */
	enable_irq(gp2a_data->irq);
	printk(KERN_INFO "[GP2A] enable_irq IRQ_NO:%d\n",gp2a_data->irq);

	/* enabling VOUT terminal in nomal operation */
	value = 0x00;
	gp2a_i2c_write((u8)(GP2A_REG_CON),&value);
	
}

/*
 * Operating the device in Normal Output Mode(not operating in Interrupt Mode), but treating 
 * ps_out as interrupt pin by the Application Processor.When sensor detects that the object is 
 * near,ps_out changes from 1->0, which is treated as edge-falling interrupt, then it 
 * reports to the input event only once.If,reporting to the input events should be done as long 
 * as the object is near, treat ps_out as low-level interrupt.
 */
static irqreturn_t gp2a_irq_handler( int irq, void *unused )
{
  	printk(KERN_INFO "[GP2A] gp2a_irq_handler called\n");
	if(gp2a_data->irq !=-1)
	{
		disable_irq_nosync(gp2a_data->irq);
		printk(KERN_INFO "[GP2A] disable_irq : IRQ_NO:%d\n",gp2a_data->irq);
		queue_work(gp2a_prox_wq, &gp2a_data->work_prox);
	}
	else
	{
		error("PROX_IRQ not handled");
		return IRQ_NONE;
	}
	debug("PROX_IRQ handled");
	return IRQ_HANDLED;
}
#endif

static ssize_t adc_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a_data->val_state);
}

static ssize_t state_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a_data->val_state);
}

static ssize_t name_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_NAME);
}

static ssize_t vendor_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_VENDOR);
}

#ifdef GP2AP002X_PROXIMITY_OFFSET
int gp2a_cal_mode_read_file(char *mode)
{
	int err = 0;
	mm_segment_t old_fs;
	struct file *cal_mode_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_mode_filp = filp_open(OFFSET_FILE_PATH,O_RDONLY, 0666);
	if (IS_ERR(cal_mode_filp)) {
		err = PTR_ERR(cal_mode_filp);
		if (err != -ENOENT)
			pr_err("%s: Can't open cal_mode file\n", __func__);
		set_fs(old_fs);
		return err;
	}
	err = cal_mode_filp->f_op->read(cal_mode_filp,	(char *)&mode,sizeof(u8), &cal_mode_filp->f_pos);

	if (err != sizeof(u8)) {
		pr_err("%s: Can't read the cal_mode from file\n",__func__);
		filp_close(cal_mode_filp, current->files);
		set_fs(old_fs);
		return -EIO;
	}

	filp_close(cal_mode_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int gp2a_cal_mode_save_file(char mode)
{
	struct file *cal_mode_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_mode_filp = filp_open(OFFSET_FILE_PATH, O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_mode_filp)) {
		pr_err("%s: Can't open cal_mode file\n",__func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_mode_filp);
		pr_err("%s: err = %d\n",__func__, err);
		return err;
	}

	err = cal_mode_filp->f_op->write(cal_mode_filp, (char *)&mode, sizeof(u8), &cal_mode_filp->f_pos);
	if (err != sizeof(u8)) 
	{
		pr_err("%s: Can't read the cal_mode from file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_mode_filp, current->files);
	set_fs(old_fs);

	return err;
}

static ssize_t prox_cal_read(struct device *dev,struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a_data->cal_mode);
}

static ssize_t prox_cal_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 value;
	int err;

	if (sysfs_streq(buf, "1")) 
	{
		gp2a_data->cal_mode = 1;
		nondetect = PROX_NONDETECT_MODE1;
		detect = PROX_DETECT_MODE1;
	} 
	else if (sysfs_streq(buf, "2")) 
	{
		gp2a_data->cal_mode = 2;
		nondetect = PROX_NONDETECT_MODE2;
		detect = PROX_DETECT_MODE2;
	} 
	else if (sysfs_streq(buf, "0")) 
	{
		gp2a_data->cal_mode = 0;
		nondetect = PROX_NONDETECT;
		detect = PROX_DETECT;
	} 
	else 
	{
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

#if defined(CONFIG_MACH_GARDA) || defined(CONFIG_MACH_KYLEVE) || defined(CONFIG_MACH_LOGAN)
	value = 0x08;
#else
	value = 0x00;
#endif
	gp2a_i2c_write(GP2A_REG_GAIN, &value);
	value = nondetect;
	gp2a_i2c_write(GP2A_REG_HYS, &value);
	value = 0x04;
	gp2a_i2c_write(GP2A_REG_CYCLE, &value);
	value = 0x03;
	gp2a_i2c_write(GP2A_REG_OPMOD, &value);
	value = 0x00;
	gp2a_i2c_write(GP2A_REG_CON, &value);

	err = gp2a_cal_mode_save_file(gp2a_data->cal_mode);

	if (err < 0) {
		pr_err("%s: prox_cal_write() failed\n", __func__);
		return err;
	}

	return size;
}
#endif

static DEVICE_ATTR(adc, 0440, adc_read, NULL);
static DEVICE_ATTR(state, 0440, state_read, NULL);
static DEVICE_ATTR(name, 0440, name_read, NULL);
static DEVICE_ATTR(vendor, 0440, vendor_read, NULL);
#ifdef GP2AP002X_PROXIMITY_OFFSET
static DEVICE_ATTR(prox_cal, 0664, prox_cal_read, prox_cal_write);
#endif

static struct attribute *proxi_attrs[] = {
	&dev_attr_adc.attr,
	&dev_attr_state.attr,
	&dev_attr_name.attr,
	&dev_attr_vendor.attr,
#ifdef GP2AP002X_PROXIMITY_OFFSET
	&dev_attr_prox_cal.attr,
#endif
	NULL,
};

static struct attribute_group prox_sensor_attributes_group = {
	.attrs = proxi_attrs
};

static ssize_t proximity_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gp2a_data->val_state);
}

static ssize_t proximity_enable_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t size)
{
	bool new_value;
	int err;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else 
	{
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	printk(KERN_INFO "[GP2A] proximity_enable_store : new_value=%d\n", new_value);

	mutex_lock(&gp2a_data->power_lock);
    
	if (new_value )
	{  
		prox_ctrl_regulator(1);
#ifdef GP2AP002X_PROXIMITY_OFFSET
		err = gp2a_cal_mode_read_file(&gp2a_data->cal_mode);
		if (err < 0 && err != -ENOENT)
			pr_err("%s: cal_mode file read fail\n", __func__);
		pr_info("%s: mode = %02x\n", __func__, gp2a_data->cal_mode);
		if (gp2a_data->cal_mode == 2) 
		{
			nondetect = PROX_NONDETECT_MODE2;
			detect = PROX_DETECT_MODE2;
		} 
		else if (gp2a_data->cal_mode == 1) 
		{
			nondetect = PROX_NONDETECT_MODE1;
			detect = PROX_DETECT_MODE1;
		} 
		else 
		{
			nondetect = PROX_NONDETECT;
			detect = PROX_DETECT;
		}
#endif        
		gp2a_data->val_state = 1;
		input_report_abs(gp2a_data->prox_input_dev, ABS_DISTANCE, gp2a_data->val_state);
		input_sync(gp2a_data->prox_input_dev);
        
		gp2a_prox_mode(1);
        
	}
	else if (!new_value ) 
	{
		gp2a_prox_mode(0);
    		prox_ctrl_regulator(0);		
	}

	mutex_unlock(&gp2a_data->power_lock);
	return size;
}

static struct device_attribute dev_attr_prox_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       proximity_enable_show, proximity_enable_store);

static struct attribute *gp2a_prox_attributes[] = {
	&dev_attr_prox_enable.attr,
	NULL
};

static struct attribute_group gp2a_prox_attr_group = {
	.attrs = gp2a_prox_attributes,
};
#define __raw_readl(a) (*(volatile unsigned int *)(a))
#define __raw_writel(v,a) (*(volatile unsigned int *)(a) = (v))
static int gp2a_prox_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret =0;
	u8 reg_value;
	struct gp2a_prox_platform_data *platform_data;
	unsigned int val;
	printk(KERN_INFO "[GP2A] %s start \n", __func__);	
	
	nondetect = PROX_NONDETECT;
	detect = PROX_DETECT;
	/* Allocate driver_data */
	gp2a_data = kzalloc(sizeof(struct gp2a_prox_data),GFP_KERNEL);
	if(!gp2a_data)
	{
		error("kzalloc:allocating driver_data error");
		return -ENOMEM;		
	} 
	
	val = __raw_readl( CTL_PIN_BASE + REG_PIN_CTRL4  );
	printk("val2 = %x\n",val);
	val = val | 1 << 24;
	printk("val = %x\n",val);
	val = val | 1 << 25;
	printk("val = %x\n",val);
	__raw_writel( val, CTL_PIN_BASE + REG_PIN_CTRL4);	
	val = __raw_readl( CTL_PIN_BASE + REG_PIN_CTRL4 );
	printk("last val = %x\n",val);
    	platform_data = client->dev.platform_data;
	gp2a_data->gp2a_prox_i2c_client = client;
	i2c_set_clientdata(client, gp2a_data);

	/*misc device registration*/
	if( (ret = misc_register(&gp2a_prox_misc_device)) < 0 )
	{
		error("gp2a_prox driver misc_register failed");
		goto FREE_GP2A_DATA;
	}
	
	gp2a_data->irq_gpio = platform_data->irq_gpio;		
	/*Initialisation of GPIO_PS_OUT of proximity sensor*/
	if (gpio_request(gp2a_data->irq_gpio, "Proximity Out")) {
		printk(KERN_ERR "Proximity Request GPIO_%d failed!\n", gp2a_data->irq_gpio);
	}
	
	gpio_direction_input(gp2a_data->irq_gpio);

	mutex_init(&gp2a_data->power_lock);
	
	/*Input Device Settings*/
	gp2a_data->prox_input_dev = input_allocate_device();
	if (!gp2a_data->prox_input_dev) 
	{
		error("Not enough memory for gp2a_data->prox_input_dev");
		ret = -ENOMEM;
		goto MISC_DREG;
	}
	gp2a_data->prox_input_dev->name = "proximity_sensor";
	set_bit(EV_SYN,gp2a_data->prox_input_dev->evbit);
	set_bit(EV_ABS,gp2a_data->prox_input_dev->evbit);	

	input_set_capability(gp2a_data->prox_input_dev, EV_ABS, ABS_DISTANCE);    
	input_set_abs_params(gp2a_data->prox_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	ret = input_register_device(gp2a_data->prox_input_dev);
	if (ret) 
	{
		error("Failed to register input device");
		input_free_device(gp2a_data->prox_input_dev);
		goto MISC_DREG;
	}
	debug("Input device settings complete");
    
	/*create sysfs attributes*/
	ret = sysfs_create_group(&gp2a_data->prox_input_dev->dev.kobj, &gp2a_prox_attr_group);
	if (ret)
	{
		error("Failed to create sysfs attributes");
		goto MISC_DREG;
	}
    
#if USE_INTERRUPT	
	/* Workqueue Settings */
	gp2a_prox_wq = create_singlethread_workqueue("gp2a_prox_wq");
	if (!gp2a_prox_wq)
	{
		error("Not enough memory for gp2a_prox_wq");
		ret = -ENOMEM;
		goto INPUT_DEV_DREG;
	}	     
	INIT_WORK(&gp2a_data->work_prox, gp2a_prox_work_func);
	debug("Workqueue settings complete");	

	gp2a_data->irq = gpio_to_irq(gp2a_data->irq_gpio);
			
	//gp2a_data->irq = platform_data->irq;    

	irq_set_irq_type(gp2a_data->irq, IRQ_TYPE_EDGE_FALLING);	
	if( (ret = request_irq(gp2a_data->irq, gp2a_irq_handler,IRQF_DISABLED | IRQF_NO_SUSPEND , "proximity_int", NULL )) )
	{
		error("GP2A request_irq failed IRQ_NO:%d", gp2a_data->irq);
		goto DESTROY_WORK_QUEUE;
	} 
	else
		debug("GP2A request_irq success IRQ_NO:%d", gp2a_data->irq);
	
#endif

#if 0 //not used
	/* wake lock init */
	wake_lock_init(&prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");

	timeA = ktime_set(0,0);
	timeB = ktime_set(0,0);
#endif	

	/*Device Initialisation with recommended register values from datasheet*/
	
	reg_value = 0x18;
	if((ret=gp2a_i2c_write(GP2A_REG_CON,&reg_value))<0)
		error("gp2a_i2c_write 1 failed");	
		
	reg_value = 0x08;
	if((ret=gp2a_i2c_write(GP2A_REG_GAIN,&reg_value))<0)
		error("gp2a_i2c_write 2 failed");
	
#if defined(CONFIG_SENSORS_TOTORO) //mode B2 : Internal receiver sensitivity is now 2 times higher than B1		
	reg_value = 0x2F;//B15 //B2: 0x20;
#else
	reg_value = PROX_NONDETECT;
#endif
	if((ret=gp2a_i2c_write(GP2A_REG_HYS,&reg_value))<0)
		error("gp2a_i2c_write 3 failed");
	
	reg_value = 0x04;
	if((ret=gp2a_i2c_write(GP2A_REG_CYCLE,&reg_value))<0)
		error("gp2a_i2c_write 4 failed");
	
	/*Pulling the GPIO_PS_OUT Pin High*/
	//gpio_set_value(gp2a_data->irq_gpio, 1);
	printk(KERN_INFO "[GP2A] gpio_get_value of GPIO_PS_OUT is %d\n",gpio_get_value(gp2a_data->irq_gpio));

	/*Setting the device into shutdown mode*/
	gp2a_prox_mode(0);

#if 1 //not yet ready
	proximity_dev = device_create(sensors_class, NULL, 0, gp2a_data->prox_input_dev, "proximity_sensor");

	if (IS_ERR(proximity_dev))
	{
		dev_err(&client->dev, "Failed to create device for the sysfs\n");
		goto SENSORS_REGISTER;
	}
	ret = sysfs_create_group(&proximity_dev->kobj, &prox_sensor_attributes_group);


#endif	
	/* set initial proximity value as 1 */
	input_report_abs(gp2a_data->prox_input_dev, ABS_DISTANCE, 1);
	input_sync(gp2a_data->prox_input_dev);

	printk(KERN_INFO "[GP2A] %s end\n", __func__);	
	return ret;
    
SENSORS_REGISTER:
	free_irq(gp2a_data->irq, gp2a_data);
	gpio_free(gp2a_data->gp2a_prox_i2c_client->irq);	
DESTROY_WORK_QUEUE:
	destroy_workqueue(gp2a_prox_wq);
INPUT_DEV_DREG:
	input_unregister_device(gp2a_data->prox_input_dev);	
MISC_DREG:
	misc_deregister(&gp2a_prox_misc_device);
FREE_GP2A_DATA:
	kfree(gp2a_data);
	return ret;
}	

static int __devexit gp2a_prox_remove(struct i2c_client *client)
{	
  	debug("%s called",__func__); 
	gp2a_prox_mode(0);
	gp2a_data->gp2a_prox_i2c_client = NULL;
	free_irq(gp2a_data->irq,NULL);
	sysfs_remove_group(&client->dev.kobj, &gp2a_prox_attr_group);
	destroy_workqueue(gp2a_prox_wq);
	input_unregister_device(gp2a_data->prox_input_dev);	
	misc_deregister(&gp2a_prox_misc_device);
	mutex_destroy(&gp2a_data->power_lock);
	kfree(gp2a_data);
	return 0;
}


#ifdef CONFIG_PM
static int gp2a_prox_suspend(struct device *dev)
{   	   
	debug("%s called",__func__); 

	//gp2a_prox_mode(0);
	//disable_irq(gp2a_data->irq);

	return 0;
}
static int gp2a_prox_resume(struct device *dev)
{  	   
	debug("%s called",__func__); 
	
	//gp2a_prox_mode(1);
	//enable_irq(gp2a_data->irq);

	return 0;
}
#else
#define gp2a_prox_suspend NULL
#define gp2a_prox_resume NULL
#endif

static const struct i2c_device_id gp2a_prox_id[] = {
	{"gp2a_prox", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, gp2a_prox_id);

static const struct dev_pm_ops gp2a_prox_pm_ops = {
	.suspend = gp2a_prox_suspend,
	.resume = gp2a_prox_resume,
};

static struct i2c_driver gp2a_prox_i2c_driver = {
	.driver = {
                .name = "gp2a_prox",
                .owner = THIS_MODULE,
                .pm = &gp2a_prox_pm_ops,
        },
	.probe 		= gp2a_prox_probe,
	.remove 	= __exit_p(gp2a_prox_remove),
	.id_table 	= gp2a_prox_id,
};

static int __init gp2a_prox_init(void)
{
	debug("%s called",__func__); 
	
//	gp2a_chip_init();
	
	return i2c_add_driver(&gp2a_prox_i2c_driver);

}
static void __exit gp2a_prox_exit(void)
{	
	debug("%s called",__func__);

/*	if (prox_regulator) 
	{
       	 regulator_put(prox_regulator);
		 prox_regulator = NULL;
    }
*/	
	i2c_del_driver(&gp2a_prox_i2c_driver);	
}

module_init(gp2a_prox_init);
module_exit(gp2a_prox_exit);

MODULE_DESCRIPTION("Proximity Sensor driver for GP2AP002S00F");
MODULE_AUTHOR("SHARP Electronic");
MODULE_LICENSE("GPL"); 
