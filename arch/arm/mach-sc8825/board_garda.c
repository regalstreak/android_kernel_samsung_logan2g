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

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/localtimer.h>

#include <mach/hardware.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <mach/globalregs.h>
#include <mach/board.h>
#include <mach/serial_sprd.h>
#include <mach/adi.h>
#include <mach/adc.h>
#include "devices.h"
#include <linux/ktd253b_bl.h>
#include <sound/audio_pa.h>
#include <linux/headset.h>
#include <gps/gpsctl.h>
#include <mach/sys_debug.h>
#include <linux/i2c-gpio.h>
#include <mach/gpio.h>
#include <linux/gpio_keys.h>

#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/regs_glb.h>
#include <mach/regs_ahb.h>

#ifdef CONFIG_INPUT_YAS_SENSORS
#include <linux/yas.h>
#endif

#ifdef CONFIG_SENSORS_K3DH
#include <linux/k3dh.h>
#endif

#if defined(CONFIG_SS6000_CHARGER)
#include <linux/ss6000_charger.h>
#endif
#if defined(CONFIG_SPA)
#include <linux/power/spa.h>
#endif 
#if defined(CONFIG_STC3115_FUELGAUGE)
#include <linux/stc3115_battery.h>
#endif

#if defined(CONFIG_SENSORS_GP2A)
#include <linux/gp2a_dev.h>
#endif
extern void __init sc8825_reserve(void);
extern void __init sci_map_io(void);
extern void __init sc8825_init_irq(void);
extern void __init sc8825_timer_init(void);
extern int __init sc8825_clock_init(void);
extern int __init sc8825_regulator_init(void);
extern int __init sci_clock_init(void);
#ifdef CONFIG_ANDROID_RAM_CONSOLE
extern int __init sprd_ramconsole_init(void);
#endif
extern int sci_adc_get_value(unsigned chan, int scale);

static struct platform_device rfkill_device;
static struct platform_device brcm_bluesleep_device;
static struct platform_device gpio_button_device;

static struct platform_gpsctl_data pdata_gpsctl = {
	.reset_pin = GPIO_GPS_RESET,
	.onoff_pin = GPIO_GPS_ONOFF,
	.clk_type = "clk_aux0",
	.pwr_type = "vdd18",
};

static struct platform_device gpsctl_dev = {
	.name = "gpsctl",
	.dev.platform_data = &pdata_gpsctl,
};

#if defined(CONFIG_SS6000_CHARGER)
static struct ss6000_platform_data ss6000_info = {
	.en_set = CHARGERIC_CHG_EN_GPIO,
	.pgb = CHARGERIC_TA_INT_GPIO,
	.chgsb = CHARGERIC_CHG_INT_GPIO,
}; 
static struct platform_device ss6000_charger = {
	.name		= "ss6000_charger",
	.id		= -1, 
	.dev		= {
		.platform_data = &ss6000_info,
	},		
};
#endif

#if defined(CONFIG_SPA)
static struct spa_platform_data spa_info = {
	.use_fuelgauge = 1,
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

#if defined(CONFIG_STC3115_FUELGAUGE)
static struct stc311x_platform_data stc3115_data = {
                .battery_online = NULL,
                .charger_online = NULL, 		// used in stc311x_get_status()
                .charger_enable = NULL,		// used in stc311x_get_status()
                .power_supply_register = NULL,
                .power_supply_unregister = NULL,
	
		.Vmode= 0,       /*REG_MODE, BIT_VMODE 1=Voltage mode, 0=mixed mode */
  		.Alm_SOC = 10,      /* SOC alm level %*/
  		.Alm_Vbat = 3600,   /* Vbat alm level mV*/
  		.CC_cnf = 290,      /* nominal CC_cnf, coming from battery characterisation*/
  		.VM_cnf = 308,      /* nominal VM cnf , coming from battery characterisation*/
  		.Cnom = 1500,       /* nominal capacity in mAh, coming from battery characterisation*/
  		.Rsense = 10,       /* sense resistor mOhms*/
  		.RelaxCurrent = 100, /* current for relaxation in mA (< C/20) */
  		.Adaptive = 1,     /* 1=Adaptive mode enabled, 0=Adaptive mode disabled */

		/* Elentec Co Ltd Battery pack - 80 means 8% */
		.CapDerating[6] = 80,   /* capacity derating in 0.1%, for temp = -20°C */
  		.CapDerating[5] = 50,   /* capacity derating in 0.1%, for temp = -10°C */
		.CapDerating[4] = 20,    /* capacity derating in 0.1%, for temp = 0°C */
		.CapDerating[3] = 10,  /* capacity derating in 0.1%, for temp = 10°C */
		.CapDerating[2] = 0,  /* capacity derating in 0.1%, for temp = 25°C */
		.CapDerating[1] = -10,  /* capacity derating in 0.1%, for temp = 40°C */
		.CapDerating[0] = -10,  /* capacity derating in 0.1%, for temp = 60°C */

  		.OCVOffset[15] = -110,    /* OCV curve adjustment */
		.OCVOffset[14] = -10,   /* OCV curve adjustment */
		.OCVOffset[13] = -6,    /* OCV curve adjustment */
		.OCVOffset[12] = -6,    /* OCV curve adjustment */
		.OCVOffset[11] = 0,    /* OCV curve adjustment */
		.OCVOffset[10] = -6,    /* OCV curve adjustment */
		.OCVOffset[9] = 2,     /* OCV curve adjustment */
		.OCVOffset[8] = -9,      /* OCV curve adjustment */
		.OCVOffset[7] = -1,      /* OCV curve adjustment */
		.OCVOffset[6] = 11,    /* OCV curve adjustment */
		.OCVOffset[5] = 9,    /* OCV curve adjustment */
		.OCVOffset[4] = 16,     /* OCV curve adjustment */
		.OCVOffset[3] = 37,    /* OCV curve adjustment */
		.OCVOffset[2] = 26,     /* OCV curve adjustment */
		.OCVOffset[1] = -44,    /* OCV curve adjustment */
		.OCVOffset[0] = -66,     /* OCV curve adjustment */
		
		.OCVOffset2[15] = -58,    /* OCV curve adjustment */
		.OCVOffset2[14] = -86,   /* OCV curve adjustment */
		.OCVOffset2[13] = -53,    /* OCV curve adjustment */
		.OCVOffset2[12] = -36,    /* OCV curve adjustment */
		.OCVOffset2[11] = -38,    /* OCV curve adjustment */
		.OCVOffset2[10] = -57,    /* OCV curve adjustment */
		.OCVOffset2[9] = -28,     /* OCV curve adjustment */
		.OCVOffset2[8] = -24,      /* OCV curve adjustment */
		.OCVOffset2[7] = -16,      /* OCV curve adjustment */
		.OCVOffset2[6] = -20,    /* OCV curve adjustment */
		.OCVOffset2[5] = -26,    /* OCV curve adjustment */
		.OCVOffset2[4] = -49,     /* OCV curve adjustment */
		.OCVOffset2[3] = -81,    /* OCV curve adjustment */
		.OCVOffset2[2] = -112,     /* OCV curve adjustment */
		.OCVOffset2[1] = -104,    /* OCV curve adjustment */
		.OCVOffset2[0] = 0,     /* OCV curve adjustment */

			/*if the application temperature data is preferred than the STC3115 temperature*/
  		.ExternalTemperature = NULL, /*External temperature fonction, return C*/
  		.ForceExternalTemperature = 0, /* 1=External temperature, 0=STC3115 temperature */
		
};
#endif

static struct platform_device *devices[] __initdata = {
	&sprd_serial_device0,
	&sprd_serial_device1,
	&sprd_serial_device2,
	&sprd_device_rtc,
	&sprd_nand_device,
	&sprd_lcd_device0,
	&ktd253_backlight_device,
	&sprd_i2c_device0,
	&sprd_i2c_device1,
	&sprd_i2c_device2,
	&sprd_i2c_device3,
	&sprd_spi0_device,
	&sprd_spi1_device,
	&sprd_spi2_device,
	&sprd_keypad_device,
	&sprd_audio_platform_vbc_pcm_device,
	&sprd_audio_cpu_dai_vaudio_device,
	&sprd_audio_cpu_dai_vbc_device,
	&sprd_audio_codec_sprd_codec_device,
#if defined(CONFIG_SPA)
	&Sec_BattMonitor,
#endif
	&sprd_battery_device,
#ifdef CONFIG_ANDROID_PMEM
	&sprd_pmem_device,
	&sprd_pmem_adsp_device,
#endif
#ifdef CONFIG_ION
	&sprd_ion_dev,
#endif
	&sprd_emmc_device,
	&sprd_sdio0_device,
	&sprd_sdio1_device,
	//&sprd_sdio2_device,
	&sprd_vsp_device,
	&sprd_dcam_device,
	&sprd_scale_device,
	&sprd_rotation_device,
	&sprd_sensor_device,
	&sprd_ahb_bm0_device,
	&sprd_ahb_bm1_device,
	&sprd_ahb_bm2_device,
	&sprd_ahb_bm3_device,
	&sprd_ahb_bm4_device,
	&sprd_axi_bm0_device,
	&sprd_axi_bm1_device,
	&sprd_axi_bm2_device,
	&rfkill_device,
	&brcm_bluesleep_device,
	&gpsctl_dev,
	&gpio_button_device,
#if defined(CONFIG_SPRD_PEER_STATE)
	&sprd_peer_state_device,
#endif
#if defined(CONFIG_SS6000_CHARGER)
	&ss6000_charger,
#endif
};

/* BT suspend/resume */
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

/* RFKILL */
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

static struct sys_timer sc8825_timer = {
	.init = sc8825_timer_init,
};

bool calibration_mode = false;
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

static void __init sprd_add_otg_device(void)
{
	/*
	 * if in calibrtaion mode, we do nothing, modem will handle everything
	 */
	if (calibration_mode)
		return;
	platform_device_register(&sprd_otg_device);
}

#if defined  (CONFIG_SENSORS_GP2A)
#define PROXI_INT_GPIO_PIN      (213)
//#define PROXI_POWER_GPIO_PIN      (92)
static struct gp2a_prox_platform_data gp2a_prox_platform_data = {
	.irq_gpio = PROXI_INT_GPIO_PIN,
//	.power = PROXI_POWER_GPIO_PIN,
};
#endif

static struct i2c_board_info i2c3_boardinfo[] = {
	
#if defined(CONFIG_STC3115_FUELGAUGE)
	{
		I2C_BOARD_INFO("stc3115", 0x70),
		.platform_data	= &stc3115_data,
	},
#endif
};
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

#if (defined(CONFIG_SENSORS_K3DH) && defined(CONFIG_MACH_GARDA))
#define GPIO_ACC_INT (48)		/* KEYOUT2 =,GPIO48 */
static struct k3dh_platform_data k3dh_platform_data = {
	.gpio_acc_int = GPIO_ACC_INT,
};
#endif

static struct i2c_board_info i2c2_boardinfo[] = {
	//	I2C_BOARD_INFO(FT5206_TS_DEVICE, FT5206_TS_ADDR),
	//	.platform_data = &ft5x0x_ts_info,
#ifdef CONFIG_INPUT_YAS_SENSORS
	#ifdef CONFIG_YAS_ACC_DRIVER_LIS3DH
	{
		I2C_BOARD_INFO("accelerometer", 0x19),
	},
	#elif CONFIG_YAS_ACC_DRIVER_BMA222E
	{
		I2C_BOARD_INFO("accelerometer", 0x18),
	},
	#endif
	{
		I2C_BOARD_INFO("geomagnetic", 0x2e),
	},
#endif
#ifdef CONFIG_SENSORS_K3DH
	{
		I2C_BOARD_INFO("k3dh", 0x19),                  
		#if defined (CONFIG_MACH_GARDA)
		.platform_data = &k3dh_platform_data, /*GPIO48*/
		#endif
	},
#endif
#if defined  (CONFIG_SENSORS_GP2A)
	{
		I2C_BOARD_INFO("gp2a_prox", 0x44),
		.platform_data = &gp2a_prox_platform_data,    
 	},
#endif
};

static struct i2c_board_info i2c1_boardinfo[] = {
	{I2C_BOARD_INFO("sensor_main",0x20),},
	{I2C_BOARD_INFO("sensor_sub",0x21),},
};

static struct i2c_board_info i2c0_boardinfo[] = {
	{I2C_BOARD_INFO("zinitix_isp", 0x50),},
	{I2C_BOARD_INFO("Zinitix_tsp", 0x20),},
};

/* config I2C2 SDA/SCL to SIM2 pads */
/*sprd8810_i2c2sel_config unused*/
#if 0
static void sprd8810_i2c2sel_config(void)
{
	sprd_greg_set_bits(REG_TYPE_GLOBAL, PINCTRL_I2C2_SEL, GR_PIN_CTL);
}
#endif
static int sc8810_add_i2c_devices(void)
{
//	sprd8810_i2c2sel_config();
	i2c_register_board_info(3, i2c3_boardinfo, ARRAY_SIZE(i2c3_boardinfo));
	i2c_register_board_info(2, i2c2_boardinfo, ARRAY_SIZE(i2c2_boardinfo));
	i2c_register_board_info(1, i2c1_boardinfo, ARRAY_SIZE(i2c1_boardinfo));
	i2c_register_board_info(0, i2c0_boardinfo, ARRAY_SIZE(i2c0_boardinfo));
	return 0;
}

struct platform_device audio_pa_amplifier_device = {
	.name = "speaker-pa",
	.id = -1,
};

static int audio_pa_amplifier_headset_init(void)
{
	if (gpio_request(HEADSET_PA_CTL_GPIO, "headset outside pa")) {
		pr_err("failed to alloc gpio %d\n", HEADSET_PA_CTL_GPIO);
		return -1;
	}
	gpio_direction_output(HEADSET_PA_CTL_GPIO, 0);
	return 0;
}

static int audio_pa_amplifier_headset(int cmd, void *data)
{
	if (cmd < 0)
		return gpio_get_value(HEADSET_PA_CTL_GPIO);
	else{
		pr_info("[audio:headset] Real headset PA %s \n", cmd? "Turn On" : "Turn Off");
		gpio_direction_output(HEADSET_PA_CTL_GPIO, cmd);
	}
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
	int i;
	static struct headset_adc_range {
		int min;
		int max;
		int code;
	} adc_range[] = {
		{ 0x0000, 0x00FF, KEY_MEDIA},
		{ 0x0100, 0x01df, KEY_VOLUMEUP},
		{ 0x01e0, 0x03c0, KEY_VOLUMEDOWN },
	};
	int adc_value = sci_adc_get_value(ADC_CHANNEL_1, true);
	pr_info("[headset] ********* adc value7 SCALE 0~3V : 0x%x ******* \n", adc_value);	
	
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
	},
	.headset_get_button_code_board_method = headset_get_button_code_board_method,
	.headset_map_code2push_code_board_method = headset_map_code2push_code_board_method,
};

static int spi_cs_gpio_map[][2] = {
    {SPI0_CMMB_CS_GPIO,  0},
    {SPI0_CMMB_CS_GPIO,  0},
    {SPI0_CMMB_CS_GPIO,  0},
} ;

static struct spi_board_info spi_boardinfo[] = {
	{
	.modalias = "spidev",
	.bus_num = 0,
	.chip_select = 0,
	.max_speed_hz = 1000 * 1000,
	.mode = SPI_CPOL | SPI_CPHA,
	},
	{
	.modalias = "spidev",
	.bus_num = 1,
	.chip_select = 0,
	.max_speed_hz = 1000 * 1000,
	.mode = SPI_CPOL | SPI_CPHA,
	},
	{
	.modalias = "spidev",
	.bus_num = 2,
	.chip_select = 0,
	.max_speed_hz = 1000 * 1000,
	.mode = SPI_CPOL | SPI_CPHA,
	}
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
	if (audio_pa_control.speaker.control
	    || audio_pa_control.earpiece.control
	    || audio_pa_control.headset.control) {
		platform_set_drvdata(&audio_pa_amplifier_device,
				     &audio_pa_control);
		if (platform_device_register(&audio_pa_amplifier_device))
			pr_err("faile to install audio_pa_amplifier_device\n");
	}
	platform_set_drvdata(&headset_get_button_code_board_method_device, &headset_button);
	if (platform_device_register(&headset_get_button_code_board_method_device))
		pr_err("faile to install headset_get_button_code_board_method_device\n");
	return 0;
}

const char * sc8825_regulator_map[] = {
	/*supply source, consumer0, consumer1, ..., NULL */
	"vdd28", "iic_vdd", "ctp_vdd", NULL,
	"vddsd0", "tflash_vcc", NULL,
	"vddsim0", "nfc_vcc", NULL,
	"vddsim1", "lcd_vcc", NULL,
	NULL,
};

int __init sc8825_regulator_init(void)
{
	static struct platform_device sc8825_regulator_device = {
		.name 	= "sprd-regulator",
		.id	= -1,
		.dev = {.platform_data = sc8825_regulator_map},
	};
	return platform_device_register(&sc8825_regulator_device);
}

int __init sc8825_clock_init(void)
{
	pr_info("ahb ctl0 %08x, ctl2 %08x glb gen0 %08x gen1 %08x clk_en %08x\n",
		sci_glb_raw_read(REG_AHB_AHB_CTL0),
		sci_glb_raw_read(REG_AHB_AHB_CTL2),
		sci_glb_raw_read(REG_GLB_GEN0),
		sci_glb_raw_read(REG_GLB_GEN1),
		sci_glb_raw_read(REG_GLB_CLK_EN));
	/* FIXME: Force disable all unused clocks */
	sci_glb_clr(REG_AHB_AHB_CTL0,
		BIT_AXIBUSMON2_EB	|
		BIT_AXIBUSMON1_EB	|
		BIT_AXIBUSMON0_EB	|
//		BIT_EMC_EB       	|
//		BIT_AHB_ARCH_EB  	|
//		BIT_SPINLOCK_EB  	|
		BIT_SDIO2_EB     	|
		BIT_EMMC_EB      	|
//		BIT_DISPC_EB     	|
		BIT_G3D_EB       	|
		BIT_SDIO1_EB     	|
		BIT_DRM_EB       	|
		BIT_BUSMON4_EB   	|
		BIT_BUSMON3_EB   	|
		BIT_BUSMON2_EB   	|
		BIT_ROT_EB       	|
		BIT_VSP_EB       	|
		BIT_ISP_EB       	|
		BIT_BUSMON1_EB   	|
		BIT_DCAM_MIPI_EB 	|
		BIT_CCIR_EB      	|
		BIT_NFC_EB       	|
		BIT_BUSMON0_EB   	|
//		BIT_DMA_EB       	|
//		BIT_USBD_EB      	|
		BIT_SDIO0_EB     	|
//		BIT_LCDC_EB      	|
		BIT_CCIR_IN_EB   	|
		BIT_DCAM_EB      	|
		0);
	sci_glb_clr(REG_AHB_AHB_CTL2,
//		BIT_DISPMTX_CLK_EN	|
		BIT_MMMTX_CLK_EN    |
//		BIT_DISPC_CORE_CLK_EN|
//		BIT_LCDC_CORE_CLK_EN|
		BIT_ISP_CORE_CLK_EN |
		BIT_VSP_CORE_CLK_EN |
		BIT_DCAM_CORE_CLK_EN|
		0);
	sci_glb_clr(REG_AHB_AHB_CTL3,
//		BIT_CLK_ULPI_EN		|
//		BIT_CLK_USB_REF_EN	|
		0);
	sci_glb_clr(REG_GLB_GEN0,
		BIT_IC3_EB          |
		BIT_IC2_EB          |
		BIT_IC1_EB          |
//		BIT_RTC_TMR_EB      |
//		BIT_RTC_SYST0_EB    |
		BIT_RTC_KPD_EB      |
		BIT_IIS1_EB         |
//		BIT_RTC_EIC_EB      |
		BIT_UART2_EB        |
//		BIT_UART1_EB        |
		BIT_UART0_EB        |
//		BIT_SYST0_EB        |
		BIT_SPI1_EB         |
		BIT_SPI0_EB         |
//		BIT_SIM1_EB         |
//		BIT_EPT_EB          |
		BIT_CCIR_MCLK_EN    |
//		BIT_PINREG_EB       |
		BIT_IIS0_EB         |
//		BIT_MCU_DSP_RST		|
//		BIT_EIC_EB     		|
		BIT_KPD_EB     		|
		BIT_EFUSE_EB   		|
//		BIT_ADI_EB     		|
//		BIT_GPIO_EB    		|
		BIT_I2C0_EB    		|
//		BIT_SIM0_EB    		|
//		BIT_TMR_EB     		|
		BIT_SPI2_EB    		|
		BIT_UART3_EB   		|
		0);
	sci_glb_clr(REG_AHB_CA5_CFG,
//		BIT_CA5_CLK_DBG_EN	|
		0);
	sci_glb_clr(REG_GLB_GEN1,
		BIT_AUDIF_AUTO_EN	|
		BIT_VBC_EN			|
		BIT_AUD_TOP_EB		|
		BIT_AUD_IF_EB		|
		BIT_CLK_AUX1_EN		|
		BIT_CLK_AUX0_EN		|
		0);
	sci_glb_clr(REG_GLB_CLK_EN,
		BIT_PWM3_EB			|
//		BIT_PWM2_EB			|
		BIT_PWM1_EB			|
//		BIT_PWM0_EB			|
		0);

	sci_glb_clr(REG_GLB_PCTRL,
	//		BIT_MCU_MPLL_EN 	|
	//		BIT_MCU_TDPLL_EN	|
	//		BIT_MCU_DPLL_EN 	|
			BIT_MCU_GPLL_EN);	/* clk_gpu */

	sci_glb_set(REG_GLB_TD_PLL_CTL,
	//		BIT_TDPLL_DIV2OUT_FORCE_PD	|	/* clk_384m */
	//		BIT_TDPLL_DIV3OUT_FORCE_PD	|	/* clk_256m */
	//		BIT_TDPLL_DIV4OUT_FORCE_PD	|	/* clk_192m */
	//		BIT_TDPLL_DIV5OUT_FORCE_PD	|	/* clk_153p6m */
			0);

	printk("sc8825_clock_init ok\n");
	return 0;
}

static struct gpio_keys_button gpio_buttons[] = {
	{
		.gpio		= GPIO_HOME_KEY,
		.code		= KEY_HOME,
		.desc		= "Home Key",
		.type   		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 1,
		.debounce_interval		= 1,
	},
};

static struct gpio_keys_platform_data gpio_button_data = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device gpio_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &gpio_button_data,
	}
};

static void __init sc8825_init_machine(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	sprd_ramconsole_init();
#endif
	sci_adc_init((void __iomem *)ADC_BASE);
	sc8825_regulator_init();
	sprd_add_otg_device();
	platform_device_add_data(&sprd_serial_device0,(const void*)&plat_data0,sizeof(plat_data0));
	platform_device_add_data(&sprd_serial_device1,(const void*)&plat_data1,sizeof(plat_data1));
	platform_device_add_data(&sprd_serial_device2,(const void*)&plat_data2,sizeof(plat_data2));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sc8810_add_i2c_devices();
	sc8810_add_misc_devices();
	sprd_spi_init();
	sys_debug_init();
}

extern void sc8825_enable_timer_early(void);
static void __init sc8825_init_early(void)
{
	/* earlier init request than irq and timer */
	sc8825_clock_init();
	sc8825_enable_timer_early();
	sci_adi_init();
}

/*
 * Setup the memory banks.
 */
 
static void __init sc8825_fixup(struct machine_desc *desc,
	struct tag *tags, char **cmdline, struct meminfo *mi)
{
}

MACHINE_START(SC8825OPENPHONE, "sc8825")	
	.reserve	= sc8825_reserve,
	.map_io		= sci_map_io,
	.fixup		= sc8825_fixup,
	.init_early	= sc8825_init_early,
	.init_irq	= sc8825_init_irq,
	.timer		= &sc8825_timer,
	.init_machine	= sc8825_init_machine,
MACHINE_END



