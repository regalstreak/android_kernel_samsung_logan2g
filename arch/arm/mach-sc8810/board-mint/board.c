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
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/spi/spi.h>
#include <mach/globalregs.h>
#include <mach/board.h>
#include <mach/regulator.h>
#include <linux/regulator/consumer.h>
#include <sound/audio_pa.h>
#include "../devices.h"
#include <linux/ktd253b_bl.h>
#include <mach/gpio.h>
#include <mach/serial_sprd.h>
#include <mach/sys_debug.h>
#include <linux/spi/mxd_cmmb_026x.h>
#include <gps/gpsctl.h>
#include <mach/adc.h>
#include <linux/headset.h>
#include <linux/err.h>
#include <linux/input.h>
#if defined(CONFIG_SPA)
#include <linux/power/spa.h>
#endif

#if defined(CONFIG_MUSB_TSU8111)
#include <linux/tsu8111.h>
#endif

extern void __init sc8810_reserve(void);
extern void __init sc8810_map_io(void);
extern void __init sc8810_init_irq(void);
extern void __init sc8810_timer_init(void);
extern void __init regulator_add_devices(void);
extern void __init sc8810_clock_init(void);
#ifdef CONFIG_ANDROID_RAM_CONSOLE
extern int __init sprd_ramconsole_init(void);
#endif
static struct platform_device rfkill_device;
static struct platform_device brcm_bluesleep_device;

static unsigned int sd_detect_gpio = GPIO_SDIO_DETECT;

/* Control ldo for maxscend cmmb chip according to HW design */
static struct regulator *cmmb_regulator_1v8 = NULL;

static struct platform_gpsctl_data pdata_gpsctl = {
	.reset_pin = GPIO_GPS_RESET,
	.onoff_pin = GPIO_GPS_ONOFF,
	.clk_type  = "clk_aux0",
};

static struct platform_device  gpsctl_dev = {
	.name               = "gpsctl",
	.dev.platform_data  = &pdata_gpsctl,
};

#if defined(CONFIG_SPA)
static struct spa_platform_data spa_info = {
	.use_fuelgauge = 0,
	.battery_capacity = 1200,
	.VF_low	= 100,
	.VF_high = 600,
};
static struct platform_device Sec_BattMonitor = {
	.name		= "Sec_BattMonitor",
	.id		= -1,
	.dev		= {
		.platform_data = &spa_info,
	},
};
#endif

static struct platform_device *devices[] __initdata = {
	&sprd_serial_device0,
	&sprd_serial_device1,
	&sprd_serial_device2,
	&sprd_device_rtc,
	&sprd_nand_device,
	&sprd_lcd_device0,
	&sprd_backlight_device,
	&sprd_battery_device,
	&sprd_i2c_device0,
	&sprd_i2c_device1,
	&sprd_i2c_device2,
	&sprd_i2c_device3,
	&sprd_spi0_device,
	&sprd_spi1_device,
	&sprd_keypad_device,
	&sprd_audio_platform_vbc_pcm_device,
	&sprd_audio_cpu_dai_vaudio_device,
	&sprd_audio_cpu_dai_vbc_device,
	&sprd_audio_codec_dolphin_device,
#ifdef CONFIG_ANDROID_PMEM
	&sprd_pmem_device,
	&sprd_pmem_adsp_device,
#endif
#ifdef CONFIG_ION
	&sprd_ion_dev,
#endif
	&sprd_sdio1_device,
	&sprd_sdio0_device,
	&sprd_vsp_device,
	&sprd_dcam_device,
	&sprd_scale_device,
	&sprd_rotation_device,
	&rfkill_device,
	&brcm_bluesleep_device,
	&gpsctl_dev,
};

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= GPIO_BT2AP_WAKE,
		.end	= GPIO_BT2AP_WAKE,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= GPIO_AP2BT_WAKE,
		.end	= GPIO_AP2BT_WAKE,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device brcm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

static struct resource rfkill_resources[] = {
	{
		.name   = "bt_power",
		.start  = GPIO_BT_POWER,
		.end    = GPIO_BT_POWER,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bt_reset",
		.start  = GPIO_BT_RESET,
		.end    = GPIO_BT_RESET,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device rfkill_device = {
	.name = "rfkill",
	.id = -1,
	.num_resources	= ARRAY_SIZE(rfkill_resources),
	.resource	= rfkill_resources,
};

static struct platform_ktd253b_backlight_data ktd253b_data = {
	.max_brightness = 255,
	.dft_brightness = 50,
	.ctrl_pin       = GPIO_BK,
};

static struct sys_timer sc8810_timer = {
	.init = sc8810_timer_init,
};

static int calibration_mode = false;
static int __init calibration_start(char *str)
{
	if(str)
		pr_info("modem calibartion:%s\n", str);
	calibration_mode = true;
	return 1;
}
__setup("calibration=", calibration_start);

int in_calibration(void)
{
	return (calibration_mode == true);
}

EXPORT_SYMBOL(in_calibration);

#ifdef CONFIG_RMI4_I2C
#include <linux/interrupt.h>
#include <linux/rmi.h>
#include <mach/gpio.h>


#define SYNA_TM2303 0x00000800

#define SYNA_BOARDS SYNA_TM2303 /* (SYNA_TM1333 | SYNA_TM1414) */
#define SYNA_BOARD_PRESENT(board_mask) (SYNA_BOARDS & board_mask)

struct syna_gpio_data {
	u16 gpio_number;
	char* gpio_name;
};

#define TOUCH_ON 1
#define TOUCH_OFF 0
//#define TOUCH_POWER_GPIO 43 //PSJ

static struct regulator *tsp_regulator_3_3=NULL;
static struct regulator *tsp_regulator_1_8=NULL;


static int s2200_ts_power(int on_off)
{
	int retval,ret;


#if 1
	if (on_off == TOUCH_ON) {

		if(tsp_regulator_3_3 == NULL) {
			tsp_regulator_3_3 = regulator_get(NULL, REGU_NAME_TP); /*touch power*/
			if(IS_ERR(tsp_regulator_3_3)){
				printk("[TSP] can not get TSP VDD 3.3V\n");
				tsp_regulator_3_3 = NULL;
				return -1;
			}

			ret = regulator_set_voltage(tsp_regulator_3_3,3300000,3300000);
			printk("[TSP] %s --> regulator_set_voltage ret = %d \n",__func__, ret);
		}

		if(tsp_regulator_1_8 == NULL) {
			tsp_regulator_1_8 = regulator_get(NULL, REGU_NAME_TP1); /*touch power*/
			if(IS_ERR(tsp_regulator_1_8)){
				printk("[TSP] can not get TSP VDD 1.8V\n");
				tsp_regulator_1_8 = NULL;
				return -1;
			}

			ret = regulator_set_voltage(tsp_regulator_1_8,1800000,1800000);
			printk("[TSP] %s --> regulator_set_voltage ret = %d \n",__func__, ret);
		}

		ret = regulator_enable(tsp_regulator_1_8);
		printk("[TSP] --> 1.8v regulator_enable ret = %d \n", ret);

		msleep(5);


		ret = regulator_enable(tsp_regulator_3_3);
		printk("[TSP] --> 3.3v regulator_enable ret = %d \n", ret);



	}
	else {

		ret = regulator_disable(tsp_regulator_3_3);
		printk("[TSP] --> 3.3v regulator_disable ret = %d \n", ret);

		ret = regulator_disable(tsp_regulator_1_8);
		printk("[TSP] --> 1.8v regulator_disable ret = %d \n", ret);

	}
	msleep(210);

#else
	pr_info("%s: TS power change to %d.\n", __func__, on_off);
	retval = gpio_request(TOUCH_POWER_GPIO,"Touch_en");
	if (retval) {
		pr_err("%s: Failed to acquire power GPIO, code = %d.\n",
			 __func__, retval);
		return retval;
	}

	if (on_off == TOUCH_ON) {
		retval = gpio_direction_output(TOUCH_POWER_GPIO,1);
		if (retval) {
			pr_err("%s: Failed to set power GPIO to 1, code = %d.\n",
				__func__, retval);
			return retval;
	}
		gpio_set_value(TOUCH_POWER_GPIO,1);
	} else {
		retval = gpio_direction_output(TOUCH_POWER_GPIO,0);
		if (retval) {
			pr_err("%s: Failed to set power GPIO to 0, code = %d.\n",
				__func__, retval);
			return retval;
		}
		gpio_set_value(TOUCH_POWER_GPIO,0);
	}

	gpio_free(TOUCH_POWER_GPIO);
	msleep(200);

#endif

	return 0;
}

static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval=0;
	struct syna_gpio_data *data = gpio_data;

	pr_info("%s: RMI4 gpio configuration set to %d.\n", __func__,
		configure);

	if (configure) {
		retval = gpio_request(data->gpio_number, "rmi4_attn");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			return retval;
		}

		//omap_mux_init_signal(data->gpio_name, OMAP_PIN_INPUT_PULLUP);
		retval = gpio_direction_input(data->gpio_number);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			gpio_free(data->gpio_number);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		       __func__, data->gpio_number);
	}

	return s2200_ts_power(configure);
}
#endif


#if defined(CONFIG_RMI4_I2C)

/* I2C_GPIO emulation ++*/

#define TSP_SDA 59
#define TSP_SCL 18

static struct i2c_gpio_platform_data touch_i2c_gpio_data = {
        .sda_pin    = TSP_SDA,
        .scl_pin    = TSP_SCL,
        .udelay  = 3,  //// brian :3
        .timeout = 100,
};

static struct platform_device touch_i2c_gpio_device = {
        .name       = "i2c-gpio",
        .id     = 4,
        .dev        = {
            .platform_data  = &touch_i2c_gpio_data,
        },
};

static struct platform_device *gpio_i2c_devices[] __initdata = {
	&touch_i2c_gpio_device,
};

/* I2C_GPIO emulation --*/



//#if SYNA_BOARD_PRESENT(SYNA_TM2303)
	/* tm2303 has four buttons.
	 */

#define AXIS_ALIGNMENT { }

#define TM2303_ADDR 0x20
#define TM2303_ATTN 60 /*tsp int*/
static unsigned char tm2303_f1a_button_codes[] = {KEY_MENU, KEY_BACK};

static int tm2303_post_suspend(void *pm_data) {
	pr_info("%s: RMI4 callback.\n", __func__);
	return s2200_ts_power(TOUCH_OFF);
}

static int tm2303_pre_resume(void *pm_data) {
	pr_info("%s: RMI4 callback.\n", __func__);
	return s2200_ts_power(TOUCH_ON);
}

static struct rmi_f1a_button_map tm2303_f1a_button_map = {
	.nbuttons = ARRAY_SIZE(tm2303_f1a_button_codes),
	.map = tm2303_f1a_button_codes,
};

static struct syna_gpio_data tm2303_gpiodata = {
	.gpio_number = TM2303_ATTN,
	.gpio_name = "simrst3.gpio_60",
};

static struct rmi_device_platform_data tm2303_platformdata = {
	.driver_name = "rmi_generic",
	.attn_gpio = TM2303_ATTN,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.reset_delay_ms = 250,
	.gpio_data = &tm2303_gpiodata,
	.gpio_config = synaptics_touchpad_gpio_setup,
	.axis_align = AXIS_ALIGNMENT,
	.f1a_button_map = &tm2303_f1a_button_map,
	.post_suspend = tm2303_post_suspend,
	.pre_resume = tm2303_pre_resume,
	.f11_type_b = true,
};

static struct i2c_board_info __initdata synaptics_i2c_devices[] = {
	{
         I2C_BOARD_INFO("rmi_i2c", TM2303_ADDR),
        .platform_data = &tm2303_platformdata,
	},
};

//#endif /* TM2303 */
#endif /* RMI4_I2C */


static void __init sprd_add_otg_device(void)
{
	/*
	 * if in calibrtaion mode, we do nothing, modem will handle everything
	 */
	if (calibration_mode)
		return;
	platform_device_register(&sprd_otg_device);
}

static struct serial_data plat_data0 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 48000000,
};
static struct serial_data plat_data1 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 26000000,
};
static struct serial_data plat_data2 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 26000000,
};

/* config TSP I2C2 SDA/SCL to SIM2 pads */
static void sprd8810_i2c2sel_config(void)
{
	sprd_greg_set_bits(REG_TYPE_GLOBAL, PINCTRL_I2C2_SEL, GR_PIN_CTL);
}


static struct i2c_board_info i2c0_boardinfo[] = {

#if defined  (CONFIG_SENSORS_K3DH)
	{
		I2C_BOARD_INFO("k3dh", 0x19),
	},
#endif

#if defined  (CONFIG_SENSORS_HSCD)
	{
		I2C_BOARD_INFO("hscd_i2c", 0x0c),
	},
 #endif

#if defined  (CONFIG_SENSORS_GP2A)
	{
		I2C_BOARD_INFO("gp2a_prox", 0x44),
	},
#endif

};

#if defined(CONFIG_MUSB_TSU8111)
#define MUSB_INT_GPIO_PIN (136)

static struct  tsu8111_platform_data tsu8111_platform_pdata =
{
    .intb_gpio = MUSB_INT_GPIO_PIN,
};

static struct i2c_board_info __initdata i2c3_boardinfo[] = {
	 {
		 I2C_BOARD_INFO("tsu8111", 0x25),
         .platform_data = &tsu8111_platform_pdata,
//		 .irq = gpio_to_irq(MUSB_INT_GPIO_PIN),
	 },
};
#endif
static struct i2c_board_info i2c1_boardinfo[] = {
	{I2C_BOARD_INFO("sensor_main",0x3C),},
	{I2C_BOARD_INFO("sensor_sub",0x21),},
};


static int sc8810_add_i2c_devices(void)
{
	sprd8810_i2c2sel_config();

#if 0
	gpio_request(TSP_INT_GPIO_PIN, "tsp_int");
	gpio_direction_input(TSP_INT_GPIO_PIN);

	i2c2_boardinfo[0].irq = gpio_to_irq(TSP_INT_GPIO_PIN);
#endif

	i2c_register_board_info(1, i2c1_boardinfo, ARRAY_SIZE(i2c1_boardinfo));
	i2c_register_board_info(0, i2c0_boardinfo, ARRAY_SIZE(i2c0_boardinfo));
#if defined(CONFIG_MUSB_TSU8111)	
	i2c_register_board_info(3, i2c3_boardinfo,ARRAY_SIZE(i2c3_boardinfo));
#endif
#if defined(CONFIG_RMI4_I2C)
	i2c_register_board_info(0x4, synaptics_i2c_devices,
				ARRAY_SIZE(synaptics_i2c_devices)); //PSJ
#endif

	return 0;
}

struct platform_device audio_pa_amplifier_device = {
	.name = "speaker-pa",
	.id = -1,
};

static int audio_pa_amplifier_speaker(u32 cmd, void *data)
{
	int ret = 0;
	if (cmd < 0) {
		/* get speaker amplifier status : enabled or disabled */
		ret = 0;
	} else {
		/* set speaker amplifier */
	}
	return ret;
}

static int audio_pa_amplifier_headset_init(void)
{
	if (gpio_request(HEADSET_PA_CTL_GPIO, "headset outside pa")) {
		pr_err("failed to alloc gpio %d\n", HEADSET_PA_CTL_GPIO);
		return -1;
	}
	gpio_direction_output(HEADSET_PA_CTL_GPIO, 0);
	return 0;
}

static int audio_pa_amplifier_headset(u32 cmd, void *data)
{
	gpio_direction_output(HEADSET_PA_CTL_GPIO, cmd);
	return 0;
}

static _audio_pa_control audio_pa_control = {
	.speaker = {
		.init = NULL,
		.control = NULL,
	},
	.earpiece = {
		.init = NULL,
		.control = NULL,
	},
	.headset = {
		.init = audio_pa_amplifier_headset_init,
		.control = audio_pa_amplifier_headset,
	},
};

static unsigned int headset_get_button_code_board_method(int v)
{
	static struct headset_adc_range {
		int min;
		int max;
		int code;
	} adc_range[] = {
		{ 0x0005, 0x0050, KEY_MEDIA},
		{ 0x0050, 0x0090, KEY_VOLUMEUP },
		{ 0x0090, 0x0150, KEY_VOLUMEDOWN },
	};
	int adc_value = sci_adc_get_value(ADC_CHANNEL_TEMP, false);
	int i;
	for (i = 0; i < ARRY_SIZE(adc_range); i++)
		if (adc_value >= adc_range[i].min && adc_value < adc_range[i].max)
			return adc_range[i].code;
	return KEY_RESERVED;
}

static unsigned int headset_map_code2push_code_board_method(unsigned int code, int push_type)
{
	switch (push_type) {
	case HEADSET_BUTTON_DOWN_SHORT:
		break;
	case HEADSET_BUTTON_DOWN_LONG:
		code = KEY_RESERVED;
		break;
	}
	return code;
}

static struct platform_device headset_get_button_code_board_method_device = {
	.name = "headset-button",
	.id = -1,
};

static struct _headset_button headset_button = {
	.cap = {
		{ EV_KEY, KEY_MEDIA },
		{ EV_KEY, KEY_VOLUMEUP },
		{ EV_KEY, KEY_VOLUMEDOWN },
		{ EV_KEY, KEY_RESERVED },
	},
	.headset_get_button_code_board_method = headset_get_button_code_board_method,
	.headset_map_code2push_code_board_method = headset_map_code2push_code_board_method,
};


static void mxd_cmmb_poweron()
{
       gpio_direction_output(GPIO_CMMB_EN, 0);
       msleep(3);
       gpio_direction_output(GPIO_CMMB_EN, 1);
}
static void mxd_cmmb_poweroff()
{
       gpio_direction_output(GPIO_CMMB_EN, 0);
}
static int mxd_cmmb_init()
{
	int ret=0;
	ret = gpio_request(GPIO_CMMB_EN,   "MXD_CMMB_EN");
	if (ret)
	{
		pr_debug("mxd spi req gpio en err!\n");
		goto err_gpio_init;
	}
        gpio_direction_output(GPIO_CMMB_EN, 0);
	gpio_set_value(GPIO_CMMB_EN, 0);
	return 0;
err_gpio_init:
        gpio_free(GPIO_CMMB_EN);
	return ret;
}



static struct mxd_cmmb_026x_platform_data mxd_plat_data = {
	.poweron  = mxd_cmmb_poweron,
	.poweroff = mxd_cmmb_poweroff,
	.init     = mxd_cmmb_init,
};

static int spi_cs_gpio_map[][2] = {
    {SPI0_WIFI_CS_GPIO,  0},
    {SPI1_CMMB_CS_GPIO,  0},
} ;


static struct spi_board_info spi_boardinfo[] = {
	{
		.modalias = "wlan_spi",
		.bus_num = 0,
		.chip_select = 0,
		.max_speed_hz = 48 * 1000 * 1000,
		.mode = SPI_CPOL | SPI_CPHA,
	},
	{
		.modalias = "cmmb-dev",
		.bus_num = 1,
		.chip_select = 0,
		.max_speed_hz = 10 * 1000 * 1000,
		.mode = SPI_CPOL | SPI_CPHA,
		.platform_data = &mxd_plat_data,
	},
};


static void sprd_spi_init(void)
{
	int busnum, cs, gpio;
	int i;

	struct spi_board_info *info = spi_boardinfo;

	for (i = 0; i < ARRAY_SIZE(spi_boardinfo); i++) {
		busnum = info[i].bus_num;
		cs = info[i].chip_select;
		gpio   = spi_cs_gpio_map[busnum][cs];

		info[i].controller_data = (void *)gpio;
	}

        spi_register_board_info(info, ARRAY_SIZE(spi_boardinfo));
}

static int sc8810_add_misc_devices(void)
{
	if (audio_pa_control.speaker.control || audio_pa_control.earpiece.control || \
		audio_pa_control.headset.control) {
		platform_set_drvdata(&audio_pa_amplifier_device, &audio_pa_control);
		if (platform_device_register(&audio_pa_amplifier_device))
			pr_err("faile to install audio_pa_amplifier_device\n");
	}
#if 1
	platform_set_drvdata(&headset_get_button_code_board_method_device, &headset_button);
	if (platform_device_register(&headset_get_button_code_board_method_device))
		pr_err("faile to install headset_get_button_code_board_method_device\n");
#endif
	return 0;
}

static void __init sc8810_init_machine(void)
{
	int clk;
	regulator_add_devices();
	sprd_add_otg_device();
	platform_device_add_data(&sprd_sdio0_device, &sd_detect_gpio, sizeof(sd_detect_gpio));
	platform_device_add_data(&sprd_backlight_device,&ktd253b_data,sizeof(ktd253b_data));
	platform_device_add_data(&sprd_serial_device0,(const void*)&plat_data0,sizeof(plat_data0));
	platform_device_add_data(&sprd_serial_device1,(const void*)&plat_data1,sizeof(plat_data1));
	platform_device_add_data(&sprd_serial_device2,(const void*)&plat_data2,sizeof(plat_data2));

	platform_add_devices(devices, ARRAY_SIZE(devices));
	platform_add_devices(gpio_i2c_devices, ARRAY_SIZE(gpio_i2c_devices));

	sc8810_add_i2c_devices();
	sc8810_add_misc_devices();
	sprd_spi_init();
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	sprd_ramconsole_init();
#endif
	sys_debug_init();
}

static void __init sc8810_fixup(struct machine_desc *desc, struct tag *tag,
		char **cmdline, struct meminfo *mi)
{
	struct tag *t = tag;

#ifdef CONFIG_NKERNEL
	/* manipulate cmdline if not in calibration mode, for mfserial */
	for (; t->hdr.size; t = (struct tag*)((__u32*)(t) + (t)->hdr.size)) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			char *p, *c;
			c = (char*)t->u.cmdline.cmdline;
			if(strstr(c, "calibration=") == NULL) {
				p = strstr(c, "console=");
				/* break it, if exists */
				if (p)
					*p = 'O';
				/* add our kernel parameters */
				strcat(c, "console=ttyS1,115200n8 loglevel=8");
			}
			break;
		}
	}
#endif
}

static void __init sc8810_init_early(void)
{
	/* earlier init request than irq and timer */
	sc8810_clock_init();

}

MACHINE_START(SC8810OPENPHONE, "SP8810")
	.reserve	= sc8810_reserve,
	.map_io		= sc8810_map_io,
	.init_irq	= sc8810_init_irq,
	.timer		= &sc8810_timer,
	.init_machine	= sc8810_init_machine,
	.fixup		= sc8810_fixup,
	.init_early	= sc8810_init_early,
MACHINE_END
