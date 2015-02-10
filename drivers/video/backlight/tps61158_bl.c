/*
by jbass.choi@samsung.com 15/JUL/2013
 */

/*******************************************************************************
by jbass.choi@samsung.com 15/JUL/2013
*******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/tps61158_bl.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

static DEFINE_SPINLOCK(bl_ctrl_lock);
#define BACKLIGHT_DEBUG 0
#if BACKLIGHT_DEBUG
#define BLDBG(fmt, args...) printk(fmt, ## args)
#else
#define BLDBG(fmt, args...)
#endif

#define BACKLIGHT_DEV_NAME	"panel"

int is_poweron = 1;
int current_brightness;
int wakeup_brightness;

#define GPIO_BL_CTRL	136

#define MAX_BRIGHTNESS	255
//#define MIN_BRIGHTNESS	20 //temporary disabled by jbass.choi@samsung.com
#define DEFAULT_BRIGHTNESS 122
//#define DEFAULT_PULSE 20 //temporary disabled by jbass.choi@samsung.com
//#define DIMMING_VALUE	31 //temporary disabled by jbass.choi@samsung.com
#define MAX_BRIGHTNESS_IN_BLU	32
#define TPS61158_DATA_BITS 8
#define TPS61158_ADDRESS_BITS 8
#define TPS61158_TIME_T_EOS 300
#define TPS61158_TIME_ES_DET_DELAY 150
#define TPS61158_TIME_ES_DET_TIME 500
#define TPS61158_TIME_T_START 10
#define TPS61158_TIME_DATA_DELAY_SHORT 180
#define TPS61158_TIME_DATA_DELAY_LONG 360


//static int lcd_brightness = DEFAULT_PULSE;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend_BL;
#endif



struct tps61158_data {
	int bl_value;
	unsigned int bl_level;
	unsigned char bl_send_address; /* Always setted to 0x1A */
	unsigned char bl_send_data;
};

struct tps61158_data tps61158_table[] = {
	/* Brightness tuning available by adjusting "bl_value" of table */ 
	/*                        D0 D1 D2 D3  D4 A0 A1 RFA */
	{   0,  0, 0x1A, 0x00}, /* 0  0  0  0   0  0  0  0 */
	{  15,  1, 0x1A, 0x80}, /* 1  0  0  0   0  0  0  0 */
	{  23,  2, 0x1A, 0x40}, /* 0  1  0  0   0  0  0  0 */
	{  31,  3, 0x1A, 0xC0}, /* 1  1  0  0   0  0  0  0 */
	{  39,  4, 0x1A, 0x20}, /* 0  0  1  0   0  0  0  0 */
	{  47,  5, 0x1A, 0xA0}, /* 1  0  1  0   0  0  0  0 */
	{  55,  6, 0x1A, 0x60}, /* 0  1  1  0   0  0  0  0 */
	{  63,  7, 0x1A, 0xE0}, /* 1  1  1  0   0  0  0  0 */
	{  71,  8, 0x1A, 0x10}, /* 0  0  0  1   0  0  0  0 */
	{  79,  9, 0x1A, 0x90}, /* 1  0  0  1   0  0  0  0 */
	{  87, 10, 0x1A, 0x50}, /* 0  1  0  1   0  0  0  0 */
	{  95, 11, 0x1A, 0xD0}, /* 1  1  0  1   0  0  0  0 */
	{ 103, 12, 0x1A, 0x30}, /* 0  0  1  1   0  0  0  0 */
	{ 111, 13, 0x1A, 0xB0}, /* 1  0  1  1   0  0  0  0 */
	{ 119, 14, 0x1A, 0x70}, /* 0  1  1  1   0  0  0  0 */
	{ 127, 15, 0x1A, 0xF0}, /* 1  1  1  1   0  0  0  0 */
	{ 135, 16, 0x1A, 0x08}, /* 0  0  0  0   1  0  0  0 */
	{ 143, 17, 0x1A, 0x88}, /* 1  0  0  0   1  0  0  0 */
	{ 151, 18, 0x1A, 0x48}, /* 0  1  0  0   1  0  0  0 */
	{ 159, 19, 0x1A, 0xC8}, /* 1  1  0  0   1  0  0  0 */
	{ 167, 20, 0x1A, 0x28}, /* 0  0  1  0   1  0  0  0 */
	{ 175, 21, 0x1A, 0xA8}, /* 1  0  1  0   1  0  0  0 */
	{ 183, 22, 0x1A, 0x68}, /* 0  1  1  0   1  0  0  0 */
	{ 191, 23, 0x1A, 0xE8}, /* 1  1  1  0   1  0  0  0 */
	{ 199, 24, 0x1A, 0x18}, /* 0  0  0  1   1  0  0  0 */
	{ 207, 25, 0x1A, 0x98}, /* 1  0  0  1   1  0  0  0 */
	{ 215, 26, 0x1A, 0x58}, /* 0  1  0  1   1  0  0  0 */
	{ 223, 27, 0x1A, 0xD8}, /* 1  1  0  1   1  0  0  0 */
	{ 231, 28, 0x1A, 0x38}, /* 0  0  1  1   1  0  0  0 */
	{ 239, 29, 0x1A, 0xB8}, /* 1  0  1  1   1  0  0  0 */
	{ 247, 30, 0x1A, 0x78}, /* 0  1  1  1   1  0  0  0 */
	{ 255, 31, 0x1A, 0xF8}, /* 1  1  1  1   1  0  0  0 */
};

static int tps61158_send_bit (bool bit)
{
	if (bit) { /* Send bit 1 */
		gpio_set_value(GPIO_BL_CTRL, 0);
		udelay(TPS61158_TIME_DATA_DELAY_SHORT);
		gpio_set_value(GPIO_BL_CTRL, 1);
		udelay(TPS61158_TIME_DATA_DELAY_LONG);
		return 0;
	} else { /* Send bit 0 */
		gpio_set_value(GPIO_BL_CTRL, 0);
		udelay(TPS61158_TIME_DATA_DELAY_LONG);
		gpio_set_value(GPIO_BL_CTRL, 1);
		udelay(TPS61158_TIME_DATA_DELAY_SHORT);
		return 0;
	}
}

static int tps61158_send_tstart_signal (void)
{
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(TPS61158_TIME_T_EOS);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(TPS61158_TIME_ES_DET_DELAY);
	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(TPS61158_TIME_ES_DET_TIME);
	gpio_set_value(GPIO_BL_CTRL, 1);
	mdelay(3);
	udelay(500); /* ES window time */
	return 0;
}

static int tps61158_send_brightness_signal (unsigned int brightness_index)
{
	int i;

	printk("[%s]\n", __func__);
	printk("brightness =%d\n", tps61158_table[brightness_index].bl_value);
	printk("bl_level =%d\n", tps61158_table[brightness_index].bl_level);

	tps61158_send_tstart_signal();
	for (i = 0; i < TPS61158_ADDRESS_BITS ; i++) /* Send Address */
		tps61158_send_bit(tps61158_table[brightness_index].bl_send_address & (0x1 << i));

	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(TPS61158_TIME_T_EOS);
	gpio_set_value(GPIO_BL_CTRL, 1);
	udelay(TPS61158_TIME_T_START);

	for (i = 0; i < TPS61158_DATA_BITS ; i++) /* Send DATA */
		tps61158_send_bit(tps61158_table[brightness_index].bl_send_data & (0x1 << i));

	gpio_set_value(GPIO_BL_CTRL, 0);
	udelay(TPS61158_TIME_T_EOS);
	gpio_set_value(GPIO_BL_CTRL, 1); /* BL brightness enable */ 
	return 0;
}

#if defined(CONFIG_FB_SC8825) && defined(CONFIG_FB_LCD_NT35510_MIPI)
extern bool is_first_frame_done;
#endif

#ifdef CONFIG_EARLYSUSPEND
static void tps61158_backlight_early_suspend(struct early_suspend *h)
{
	is_poweron = 0;
	tps_backlight_set_brightness(0);
	return;
}

static void tps61158_backlight_late_resume(struct early_suspend *h)
{
	int i = 0;

#if defined(CONFIG_FB_SC8825) && defined(CONFIG_FB_LCD_NT35510_MIPI)
	while (i++ < 10 && !is_first_frame_done) {
		printk(KERN_INFO "[Backlight] wait first vsync_done, sleep 200msec\n", __func__);
		msleep(200);
	}
#endif
	tps_backlight_set_brightness(wakeup_brightness);
	is_poweron = 1;
}
#endif
/* Temporary supported by jbass.choi@samsung.com */
void ktd_backlight_set_brightness(int level)
{
	tps_backlight_set_brightness(level);
	return;
}

static int tps_backlight_set_brightness(int level)
{
	int tune_level = 0;
	int i;

	spin_lock(&bl_ctrl_lock);

	current_brightness = level;
	if (level > 0) {
		for (i = 0; i < MAX_BRIGHTNESS_IN_BLU; i++) {
			if (level >= tps61158_table[i].bl_value)
				tune_level = tps61158_table[i].bl_level;
			else
				break;
		}
	} else {
		gpio_set_value(GPIO_BL_CTRL, 0);
		mdelay(3);
		printk ("level = %d, bl_level= %d\n", level, tune_level);
		spin_unlock(&bl_ctrl_lock);
		return;
	}

	printk ("[%s]level = %d, bl_level= %d\n", __func__, level, tune_level);
	tps61158_send_brightness_signal(tune_level);

	spin_unlock(&bl_ctrl_lock);
	return 0;
}

static int tps_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	wakeup_brightness = brightness;
	if (is_poweron == 1)
		tps_backlight_set_brightness(brightness);
	else
		BLDBG("[BACKLIGHT] warning : ignore set brightness\n");

	return 0;
}

static int tps_backlight_get_brightness(struct backlight_device *bl)
{
	BLDBG("[BACKLIGHT] tps_backlight_get_brightness\n");

	return bl->props.brightness;
}

static const struct backlight_ops tps_backlight_ops = {
	.update_status	= tps_backlight_update_status,
	.get_brightness	= tps_backlight_get_brightness,
};

static int tps_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bl;
	struct backlight_properties props;

	printk("[BACKLIGHT] tps61158_backlight_probe\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = MAX_BRIGHTNESS;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register(BACKLIGHT_DEV_NAME, &pdev->dev, NULL,
					&tps_backlight_ops, &props);
	if (IS_ERR(bl))
	{
		dev_err(&pdev->dev, "failed to register backlight\n");
		return PTR_ERR(bl);
	}

	bl->props.max_brightness = MAX_BRIGHTNESS;
	bl->props.brightness = DEFAULT_BRIGHTNESS;

	platform_set_drvdata(pdev, bl);

	if(gpio_request(GPIO_BL_CTRL,"BL_CTRL"))
		printk(KERN_ERR "Request GPIO failed,""gpio: %d \n", GPIO_BL_CTRL);
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend_BL.suspend = tps61158_backlight_early_suspend;
	early_suspend_BL.resume  = tps61158_backlight_late_resume;
	early_suspend_BL.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&early_suspend_BL);
#endif
	tps_backlight_update_status(bl);

	return 0;
out:
	return 1;
}

static int tps_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	backlight_device_unregister(bl);
	gpio_direction_output(GPIO_BL_CTRL, 0);
	mdelay(3);

	return 0;
}
/* Temporary supported by jbass.choi@samsung.com */
void ktd_backlight_shutdown(struct platform_device *pdev)
{
	printk("[%s] has called in tps61158 driver!!\n",__FUNCTION__);
	gpio_direction_output(GPIO_BL_CTRL, 0);
	mdelay(5);
}

static void tps_backlight_shutdown(struct platform_device *pdev)
{
	gpio_direction_output(GPIO_BL_CTRL, 0);
	mdelay(5);
}

static struct platform_driver tps_backlight_driver = {
	.driver		= {
		.name	= BACKLIGHT_DEV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= tps_backlight_probe,
	.remove		= tps_backlight_remove,
	.shutdown       = tps_backlight_shutdown,
};

static int __init tps_backlight_init(void)
{
	return platform_driver_register(&tps_backlight_driver);
}
module_init(tps_backlight_init);

static void __exit tps_backlight_exit(void)
{
	platform_driver_unregister(&tps_backlight_driver);
}
module_exit(tps_backlight_exit);

MODULE_DESCRIPTION("TPS61158 based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps-backlight");

