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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <mach/gpio.h>
#include <linux/headset_sec.h>
#include <mach/board.h>
#include <mach/adc.h>

#ifndef HEADSET_DETECT_GPIO
#define HEADSET_DETECT_GPIO 165
#endif
#ifndef HEADSET_BUTTON_GPIO
#define HEADSET_BUTTON_GPIO 51 // 164 -> 51(NFD[13]) from REV0.1
#endif

#ifndef HEADSET_DETECT_GPIO_DEBOUNCE_SW
#define HEADSET_DETECT_GPIO_DEBOUNCE_SW 200
#endif
#ifndef HEADSET_BUTTON_GPIO_DEBOUNCE_SW
#define HEADSET_BUTTON_GPIO_DEBOUNCE_SW 100
#endif

 /* ffkaka */
#define HEADSET_MICBIAS_GPIO	50
#define HEADSET_COM_DETECT_GPIO	136
#define HEADSET_ADC_3POLE_MAX    0x33f 
#define HEADSET_ADC_4POLE_MAX    0x3ff 

#define HEADSET_DET_RETRY_CNT	4
#define HEADSET_ADC_EN_GPIO 49

/*+ BUTTON STATUS +*/
#define RELEASE 0
#define PUSH 1

///////////////////////////////////////////////////////////////////
//jinwon.baek 120808 for earloopback switch checking 
int ret_estate = KEY_RESERVED;
int det_cnt =0;

unsigned int code; 
/*- BUTTON STATUS -*/

static struct _headset headset = {
	.sdev = {
		.name = "h2w",
	},
	.detect = {
		.desc = "headset detect",
		.active_low = 1,
		.gpio = HEADSET_DETECT_GPIO,
		.debounce = 0,
		.debounce_sw = HEADSET_DETECT_GPIO_DEBOUNCE_SW,
		.irq_enabled = 1,
#ifdef CONFIG_MACH_VASTOI
		.adc_channel = ADC_CHANNEL_7,
#else
		.adc_channel = ADC_CHANNEL_TEMP,
#endif
	},
	.button = {
		.desc = "headset button",
		.active_low = 1,
		.gpio = HEADSET_BUTTON_GPIO,
		.debounce = 0,
		.debounce_sw = HEADSET_BUTTON_GPIO_DEBOUNCE_SW,
		.irq_enabled = 1,
#ifdef CONFIG_MACH_VASTOI
		.adc_channel = ADC_CHANNEL_7,
#else
		.adc_channel = ADC_CHANNEL_TEMP,
#endif
	},
};

#ifndef headset_gpio_init
#define headset_gpio_init(gpio, desc) \
	do { \
		gpio_request(gpio, desc); \
		gpio_direction_input(gpio); \
	} while (0)
#endif

#ifndef headset_gpio_free
#define headset_gpio_free(gpio) \
	gpio_free(gpio)
#endif

#ifndef headset_gpio2irq_free
#define headset_gpio2irq_free(irq, args) { }
#endif

#ifndef headset_gpio2irq
#define headset_gpio2irq(gpio) \
	gpio_to_irq(gpio)
#endif

#ifndef headset_gpio_set_irq_type
#define headset_gpio_set_irq_type(irq, type) \
	irq_set_irq_type(irq, type)
#endif

#ifndef headset_gpio_get_value
#define headset_gpio_get_value(gpio) \
	gpio_get_value(gpio)
#endif

#ifndef headset_gpio_debounce
#define headset_gpio_debounce(gpio, ms) \
	gpio_set_debounce(gpio, ms)
#endif

#define HEADSET_DEBOUNCE_ROUND_UP(dw) \
	dw = (((dw ? dw : 1) + HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD - 1) / \
		HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD) * HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD;

static struct _headset_keycap headset_key_capability[20] = {
	{ EV_KEY, KEY_MEDIA },
	{ EV_KEY, KEY_VOLUMEUP },
	{ EV_KEY, KEY_VOLUMEDOWN },
};

static unsigned int (*headset_get_button_code_board_method)(int v);
static unsigned int (*headset_map_code2push_code_board_method)(unsigned int code, int push_type);
static __devinit int headset_button_probe(struct platform_device *pdev)
{
	struct _headset_button *headset_button = platform_get_drvdata(pdev);
	headset_get_button_code_board_method = headset_button->headset_get_button_code_board_method;
	headset_map_code2push_code_board_method = headset_button->headset_map_code2push_code_board_method;
	memcpy(headset_key_capability, headset_button->cap, sizeof headset_button->cap);
	return 0;
}

static struct platform_driver headset_button_driver = {
	.driver = {
		.name = "headset-button",
		.owner = THIS_MODULE,
	},
	.probe = headset_button_probe,
};

/* Determine 3pole or 4pole is detected - ffkaka */
static void headset_hs_detect_mode(struct _headset_gpio *hgp)
{
	int adc_val = sci_adc_get_value(hgp->adc_channel, ADC_SCALE_3V);
	pr_info("[headset] Detection ADC = 0x%0x !!!!\n",adc_val);

	if(adc_val < HEADSET_ADC_3POLE_MAX){
		hgp->parent->headphone = 1;
	}
	else if(adc_val < HEADSET_ADC_4POLE_MAX){
		hgp->parent->headphone = 0;
	}
	else{
		hgp->parent->headphone = 0;
	}
}

/* add for control of external mic bias - ffkaka */
static void headset_mic_bias_control(int mode)
{

	int gpio_value;
	
	gpio_value=headset_gpio_get_value(HEADSET_MICBIAS_GPIO);

	//pr_info("[headset] before gpio_value=0x%08x \n",gpio_value);

	if(mode){
		if(gpio_value == 0x00){
			gpio_direction_output(HEADSET_MICBIAS_GPIO,1);
			pr_info("[headset] Enable Headset MIC BIAS!!!\n");
		}
		else
			pr_info("[headset] MIC BIAS was already enabled!!!\n");
	}
	else{
		if(gpio_value == 0x01){
			gpio_direction_output(HEADSET_MICBIAS_GPIO,0);
			pr_info("[headset] Disable Headset MIC BIAS!!!\n");
		}
		else
			pr_info("[headset] MIC BIAS was already disabled!!!\n");
	}
	
}

static unsigned int headset_get_button_code(int v)
{
	unsigned int code;

	pr_info("headset_get_button_code headset_get_button_code_board_method [%d] \n",headset_get_button_code_board_method);
											
	if (headset_get_button_code_board_method != NULL)
		code = headset_get_button_code_board_method(v);
	else
		code = KEY_MEDIA;
	return code;
}

static unsigned int headset_map_code2key_type(unsigned int code)
{
	unsigned int key_type = EV_KEY;
	int i;

	pr_info("headset_map_code2key_type [%d] \n",code);
	
	for(i = 0; headset_key_capability[i].key != KEY_RESERVED &&
		headset_key_capability[i].key != code && i < ARRY_SIZE(headset_key_capability); i++);
		if (i < ARRY_SIZE(headset_key_capability) && headset_key_capability[i].key == code)
			key_type = headset_key_capability[i].type;
	else
		pr_err("headset not find code [0x%x]'s maping type\n", code);
	return key_type;
}

static void headset_gpio_irq_enable(int enable, struct _headset_gpio *hgp);
#define HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD	50 /* 10 */



static ssize_t estate_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", ret_estate);	
}
static DEVICE_ATTR(estate, S_IRUGO, estate_show, NULL);

/* adc checking sys fs - ffkaka */
static ssize_t headset_adc_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int adc_value;

	headset_mic_bias_control(1);
	adc_value = sci_adc_get_value(headset.detect.adc_channel,ADC_SCALE_3V);
	
	return sprintf(buf,"ADC Value = 0x%x\n",adc_value);
}
static DEVICE_ATTR(hs_adc, S_IRUGO, headset_adc_show, NULL);

/* gnd detect sys fs - ffkaka */
static ssize_t headset_gnd_detect_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"GND DET= %d\n",headset_gpio_get_value(HEADSET_COM_DETECT_GPIO));
}
static DEVICE_ATTR(gnd_detect, S_IRUGO, headset_gnd_detect_show, NULL);

/* work function thread - ffkaka */
static void headset_button_gpio_work(struct work_struct *work)
{
	struct _headset_gpio *hgp = container_of(work, struct _headset_gpio, gpio_work);
	unsigned int key_type  = KEY_RESERVED;

	ret_estate = hgp->active;//jinwon.baek 120808 for earloopback switch checking

	if(hgp->parent->detect.active == 0) return;

	if (hgp->active) {
		code = headset_get_button_code(0);
		pr_info("[headset] button push active=[%d] \n",hgp->active);
	} else {
		pr_info("[headset] button release active=[%d] \n",hgp->active);	
	}

	key_type = headset_map_code2key_type(code);
	switch (key_type) {
		case EV_KEY:
			if(ret_estate == PUSH)
			{
				input_event(hgp->parent->input, key_type, code, 1);
				input_sync(hgp->parent->input);
			}else{
				input_event(hgp->parent->input, key_type, code, 0);
				input_sync(hgp->parent->input);
			}
			break;
		default:
			pr_err("headset not support key type [%d]\n", key_type);
	}
	
}
static void headset_detect_gpio_work(struct work_struct *work)
{
	struct _headset_gpio *hgp = container_of(work, struct _headset_gpio, gpio_work);
	int com_detect = headset_gpio_get_value(HEADSET_COM_DETECT_GPIO);
	
	if (hgp->active) {
		/* Active=1 plug in case - ffkaka */
		
		if(com_detect == 1){
			pr_info("[headset] Fake hs detection interrupt!!! com_detect[%d]\n",com_detect);
			det_cnt=0;
			headset_gpio_set_irq_type(hgp->irq, hgp->irq_type_active);
			return;
		}

		if(det_cnt < HEADSET_DET_RETRY_CNT){
			det_cnt++;
			mod_timer(&hgp->gpio_timer,jiffies + msecs_to_jiffies(50));
			return;
		}
		
		headset_hs_detect_mode(hgp);
		if (hgp->parent->headphone) {
			switch_set_state(&hgp->parent->sdev, BIT_HEADSET_NO_MIC);
			/* add for control of external mic bias REV0.1 - ffkaka */
			headset_mic_bias_control(0);
			pr_info("[headset] headphone plug in\n");
		} else {
			switch_set_state(&hgp->parent->sdev, BIT_HEADSET_MIC);
			pr_info("[headset] headset plug in\n");
			headset_gpio_set_irq_type(hgp->parent->button.irq, hgp->parent->button.irq_type_active);
			headset_gpio_irq_enable(1, &hgp->parent->button);
		}
	} else {
	/* Active=0 unplug case - ffkaka */
		headset_gpio_irq_enable(0, &hgp->parent->button);
		/* add for control of external mic bias REV0.1 - ffkaka */
		headset_mic_bias_control(0);
		if (hgp->parent->headphone){
			hgp->parent->headphone = 0;
			pr_info("[headset] headphone unplugged\n");
		}
		else
			pr_info("[headset] headset unplugged\n");
		switch_set_state(&hgp->parent->sdev, BIT_HEADSET_OUT);
	}

	det_cnt=0;

}


static void headset_detect_timer(unsigned long data)
{
	struct _headset_gpio *hgp = (struct _headset_gpio *)data;

	schedule_work(&hgp->gpio_work);
}

static void headset_button_timer(unsigned long data)
{
	struct _headset_gpio *hgp = (struct _headset_gpio *)data;

	schedule_work(&hgp->gpio_work);
}

static irqreturn_t headset_detect_irq_handler(int irq, void *dev)
{
	struct _headset_gpio *hgp = dev;
	int det_status=0;

	wake_lock_timeout(&hgp->gpio_wakelock,HZ*4);
	del_timer(&hgp->gpio_timer);
	det_status = headset_gpio_get_value(hgp->gpio);

	pr_debug("%s : %s %s\n", __func__, hgp->desc, hgp->active ? "active" : "inactive");

	if(det_status != 1){
		hgp->active = 1;
		headset_gpio_set_irq_type(hgp->irq, hgp->irq_type_inactive);
		headset_mic_bias_control(1);
		mod_timer(&hgp->gpio_timer, jiffies + msecs_to_jiffies(50));
	}
	else {
		hgp->active = 0;
		headset_gpio_set_irq_type(hgp->irq, hgp->irq_type_active);
		mod_timer(&hgp->gpio_timer,jiffies + msecs_to_jiffies(50));
	}

	return IRQ_HANDLED;
}

static irqreturn_t headset_button_irq_handler(int irq, void *dev)
{

	struct _headset_gpio *hgp = dev;
	int det_status=0;

	wake_lock_timeout(&hgp->gpio_wakelock,HZ*4);
	del_timer(&hgp->gpio_timer);
	det_status = headset_gpio_get_value(hgp->gpio);

	pr_debug("%s : %s %s\n", __func__, hgp->desc, hgp->active ? "active" : "inactive");

	if(det_status != 1){
		hgp->active = 1;
		headset_gpio_set_irq_type(hgp->irq, hgp->irq_type_inactive);
		mod_timer(&hgp->gpio_timer, jiffies + msecs_to_jiffies(100));
	}
	else {
		hgp->active = 0;
		headset_gpio_set_irq_type(hgp->irq, hgp->irq_type_active);
		mod_timer(&hgp->gpio_timer,jiffies + msecs_to_jiffies(100));
	}

	return IRQ_HANDLED;
}
/* ffkaka */

static void headset_gpio_irq_enable(int enable, struct _headset_gpio *hgp)
{
	int action = 0;
	if (enable) {
		if (!hgp->irq_enabled) {
			del_timer(&hgp->gpio_timer);
			hgp->irq_enabled = 1;
			action = 1;
			enable_irq(hgp->irq);
		}
	} else {
		if (hgp->irq_enabled) {
			disable_irq(hgp->irq);
			del_timer(&hgp->gpio_timer);
			hgp->irq_enabled = 0;
			action = 0;
		}
	}
	pr_info("%s [ irq=%d ] --- %saction %s\n", __func__, hgp->irq, action ? "do " : "no ", hgp->desc);
}

static int __init headset_init(void)
{
	int ret, i;
	struct _headset *ht = &headset;
	ret = switch_dev_register(&ht->sdev);
	if (ret < 0) {
		pr_err("switch_dev_register failed!\n");
		return ret;
	}
	platform_driver_register(&headset_button_driver); 
	ht->input = input_allocate_device();
	if (ht->input == NULL) {
		pr_err("switch_dev_register failed!\n");
		goto _switch_dev_register;
	}
	//jinwon.baek 120808 for earloopback switch checking 
	ret = device_create_file((&ht->sdev)->dev, &dev_attr_estate);
	if (ret < 0) {
		pr_err("eswitch_dev_register failed!\n");
		return ret;
	}
	/* Add for manual adc checking - ffkaka */
	ret = device_create_file((&ht->sdev)->dev, &dev_attr_hs_adc);
	if (ret < 0) {
		pr_err("hs_adc_dev_register failed!\n");
		return ret;
	}
	/* gnd detection checking - ffkaka */
	ret = device_create_file((&ht->sdev)->dev, &dev_attr_gnd_detect);
	if (ret < 0) {
		pr_err("gnd_detect_dev_register failed!\n");
		return ret;
	}	

	ht->input->name = "headset-keyboard";
	ht->input->id.bustype = BUS_HOST;
	ht->input->id.vendor = 0x0001;
	ht->input->id.product = 0x0001;
	ht->input->id.version = 0x0100;

	for(i = 0; headset_key_capability[i].key != KEY_RESERVED; i++) {
		__set_bit(headset_key_capability[i].type, ht->input->evbit);
		input_set_capability(ht->input, headset_key_capability[i].type, headset_key_capability[i].key);
	}

	if (input_register_device(ht->input))
		goto _switch_dev_register;

	headset_gpio_init(ht->detect.gpio, ht->detect.desc);
	headset_gpio_init(ht->button.gpio, ht->button.desc);

	/* control of external mic bias - ffkaka */
	ret = gpio_request(HEADSET_MICBIAS_GPIO, "hs_mic_control");
	if (ret)
	{
		pr_info("[headset] hs mic control gpio get fail!!!\n");
		goto _gpio_request;
	}
	gpio_direction_output(HEADSET_MICBIAS_GPIO, 0);	

	/* com open detect - ffkaka */
	ret = gpio_request(HEADSET_COM_DETECT_GPIO,"hs_com_detect");
	if(ret)
	{
		pr_info("[headset] hs_com_detect gpio get fail!!!\n");
		goto _gpio_request;	
	}
	gpio_direction_input(HEADSET_COM_DETECT_GPIO);

	/* adc path enable/disable control(REV0.3) - ffkaka */
	ret = gpio_request(HEADSET_ADC_EN_GPIO,"hs_adc_enable");
	if(ret)
	{
		pr_info("[headset] hs_adc_enable gpio get fail!!!\n");
		goto _gpio_request;	
	}
	gpio_direction_input(HEADSET_ADC_EN_GPIO);
	
	/* ffkaka */

	headset_gpio_debounce(ht->detect.gpio, ht->detect.debounce * 1000);
	headset_gpio_debounce(ht->button.gpio, ht->button.debounce * 1000);

	HEADSET_DEBOUNCE_ROUND_UP(ht->button.debounce_sw);
	ht->button.parent = ht;
	ht->button.irq = headset_gpio2irq(ht->button.gpio);
	INIT_WORK(&ht->button.gpio_work,headset_button_gpio_work);
	wake_lock_init(&ht->button.gpio_wakelock,WAKE_LOCK_SUSPEND,"hs_button_wakelock");
	init_timer(&ht->button.gpio_timer);
	ht->button.gpio_timer.function = headset_button_timer;
	ht->button.gpio_timer.data = (unsigned long)&ht->button;	
	ht->button.irq_type_active = ht->button.active_low ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
	ht->button.irq_type_inactive = ht->button.active_low ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
	ret = request_irq(ht->button.irq, headset_button_irq_handler,
					ht->button.irq_type_active, ht->button.desc, &ht->button);
	if (ret) {
		pr_err("request_irq gpio %d's irq failed!\n", ht->button.gpio);
		goto _gpio_request;
	}
	headset_gpio_irq_enable(0, &ht->button);

	HEADSET_DEBOUNCE_ROUND_UP(ht->detect.debounce_sw);
	ht->detect.parent = ht;
	ht->detect.irq = headset_gpio2irq(ht->detect.gpio);
	INIT_WORK(&ht->detect.gpio_work,headset_detect_gpio_work);
	wake_lock_init(&ht->detect.gpio_wakelock,WAKE_LOCK_SUSPEND,"hs_detect_wakelock");	
	init_timer(&ht->detect.gpio_timer);
	ht->detect.gpio_timer.function = headset_detect_timer;
	ht->detect.gpio_timer.data = (unsigned long)&ht->detect;
	ht->detect.irq_type_active = ht->detect.active_low ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
	ht->detect.irq_type_inactive = ht->detect.active_low ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
	ret = request_irq(ht->detect.irq, headset_detect_irq_handler,
					ht->detect.irq_type_active, ht->detect.desc, &ht->detect);
	if (ret) {
		pr_err("request_irq gpio %d's irq failed!\n", ht->detect.gpio);
		goto _headset_button_gpio_irq_handler;
	}
	return 0;
_headset_button_gpio_irq_handler:
	free_irq(ht->button.irq, &ht->button);
	headset_gpio2irq_free(ht->button.irq, &ht->button);
_gpio_request:
	headset_gpio_free(ht->detect.gpio);
	headset_gpio_free(ht->button.gpio);
	headset_gpio_free(HEADSET_MICBIAS_GPIO); /* add for control of external mic bias - ffkaka */
	headset_gpio_free(HEADSET_COM_DETECT_GPIO); /* com open detect - ffkaka */
	input_free_device(ht->input);
_switch_dev_register:
	platform_driver_unregister(&headset_button_driver);
	switch_dev_unregister(&ht->sdev);
	return ret;
}
module_init(headset_init);

static void __exit headset_exit(void)
{
	struct _headset *ht = &headset;
	device_remove_file((&ht->sdev)->dev, &dev_attr_estate);
	device_remove_file((&ht->sdev)->dev, &dev_attr_hs_adc);
	headset_gpio_irq_enable(0, &ht->button);
	headset_gpio_irq_enable(0, &ht->detect);
	free_irq(ht->detect.irq, &ht->detect);
	headset_gpio2irq_free(ht->detect.irq, &ht->detect);
	free_irq(ht->button.irq, &ht->button);
	headset_gpio2irq_free(ht->button.irq, &ht->button);
	headset_gpio_free(ht->detect.gpio);
	headset_gpio_free(ht->button.gpio);
	headset_gpio_free(HEADSET_MICBIAS_GPIO); /* add for control of external mic bias - ffkaka */
	headset_gpio_free(HEADSET_COM_DETECT_GPIO); /* com open detect - ffkaka */	
	input_free_device(ht->input);
	platform_driver_unregister(&headset_button_driver);
	switch_dev_unregister(&ht->sdev);
}
module_exit(headset_exit);

MODULE_DESCRIPTION("sprd-headset detect driver for sec");
MODULE_AUTHOR("wonjun.ka@samsung.com");
MODULE_LICENSE("GPL");
