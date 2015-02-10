/* drivers/video/sc8825/lcd_nt35516_mipi.c
 *
 * Support for nt35516 mipi LCD device
 *
 * Copyright (C) 2010 Spreadtrum
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
#include <linux/delay.h>
#include "sprdfb_panel.h"
#include <linux/ktd253b_bl.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/err.h>
#include "sprdfb_dispc_reg.h"
#include <linux/workqueue.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <linux/timer.h>
#include <linux/semaphore.h>
//#define LCD_Delay(ms)  uDelay(ms*1000)

#define MAX_DATA   56

typedef struct LCM_Init_Code_tag {
	unsigned int tag;
	unsigned char data[MAX_DATA];
}LCM_Init_Code;

typedef struct LCM_force_cmd_code_tag{
	unsigned int datatype;
	LCM_Init_Code real_cmd_code;
}LCM_Force_Cmd_Code;


#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

static LCM_Init_Code init_data[] = {

	//{LCM_SEND(1), {0x11}}, // sleep out
	//{LCM_SLEEP(120),},

	/* Power setting sequence */

	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x01}},
	{LCM_SEND(6), {4, 0, 0xB0,0x09,0x09,0x09}},
	{LCM_SEND(6), {4, 0, 0xB6,0x34,0x34,0x34}},
	{LCM_SEND(6), {4, 0, 0xB1,0x09,0x09,0x09}},
	{LCM_SEND(6), {4, 0, 0xB7,0x24,0x24,0x24}},
	{LCM_SEND(6), {4, 0, 0xB3,0x05,0x05,0x05}},
	{LCM_SEND(6), {4, 0, 0xB9,0x24,0x24,0x24}},
	{LCM_SEND(2), {0xBF,0x00}},
	{LCM_SEND(6), {4, 0, 0xB5,0x0B,0x0B,0x0B}},
	{LCM_SEND(6), {4, 0, 0xBA,0x24,0x24,0x24}},
	{LCM_SEND(6), {4, 0, 0xBC,0x00,0xA3,0x00}},
	{LCM_SEND(6), {4, 0, 0xBD,0x00,0xA3,0x00}},

	{LCM_SLEEP(10),},

	/* Display parameter setting */

	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{LCM_SEND(5), {3, 0, 0xB1,0x6C,0x04}},
	{LCM_SEND(2), {0x36,0x02}},
	{LCM_SEND(2), {0xB6,0x0A}},
	{LCM_SEND(5), {3, 0, 0xB7,0x00,0x00}},
	{LCM_SEND(7), {5, 0, 0xB8,0x01,0x05,0x05,0x05}},
	{LCM_SEND(2), {0xBA,0x01}},
	{LCM_SEND(8), {6, 0, 0xBD,0x01,0x84,0x07,0x32,0x00}},
	{LCM_SEND(8), {6, 0, 0xBE,0x01,0x84,0x07,0x31,0x00}},
	{LCM_SEND(8), {6, 0, 0xBF,0x01,0x84,0x07,0x31,0x00}},
	{LCM_SEND(2), {0x35,0x00}},// Tearing Effect On
	{LCM_SEND(6), {4, 0, 0xCC,0x03,0x00,0x00}},

	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x01}},

	//=> Chage to Page #1
	//{LCM_SEND(2), {0x3A, 0x77}},

	/* Gamma setting */
	/* Gamma set 1 */
	{LCM_SEND(55), {53, 0, 0xD1,
			       0x00,0x01,0x00,0x43,0x00,0x5B,0x00,0x73,
			       0x00,0x8D,0x00,0xB7,0x00,0xD9,0x01,0x11,
			       0x01,0x3F,0x01,0x84,0x01,0xBA,0x02,0x0C,
			       0x02,0x6E,0x02,0x64,0x02,0x8A,0x02,0xC7,
			       0x02,0xEA,0x03,0x1A,0x03,0x40,0x03,0x6B,
			       0x03,0x81,0x03,0xA1,0x03,0xB0,0x03,0xD0,
			       0x03,0xE0,0x03,0xFE}},
	{LCM_SEND(55), {53, 0, 0xD2,
			       0x00,0x01,0x00,0x43,0x00,0x5B,0x00,0x73,
			       0x00,0x8D,0x00,0xB7,0x00,0xD9,0x01,0x11,
			       0x01,0x3F,0x01,0x84,0x01,0xBA,0x02,0x0C,
			       0x02,0x6E,0x02,0x64,0x02,0x8A,0x02,0xC7,
			       0x02,0xEA,0x03,0x1A,0x03,0x40,0x03,0x6B,
			       0x03,0x81,0x03,0xA1,0x03,0xB0,0x03,0xD0,
			       0x03,0xE0,0x03,0xFE}},
	{LCM_SEND(55), {53, 0, 0xD3,
			       0x00,0x01,0x00,0x43,0x00,0x5B,0x00,0x73,
			       0x00,0x8D,0x00,0xB7,0x00,0xD9,0x01,0x11,
			       0x01,0x3F,0x01,0x84,0x01,0xBA,0x02,0x0C,
			       0x02,0x6E,0x02,0x64,0x02,0x8A,0x02,0xC7,
			       0x02,0xEA,0x03,0x1A,0x03,0x40,0x03,0x6B,
			       0x03,0x81,0x03,0xA1,0x03,0xB0,0x03,0xD0,
			       0x03,0xE0,0x03,0xFE}},
	/* Gamma set 2 */
	{LCM_SEND(55), {53, 0, 0xD4,
			       0x00,0x01,0x00,0x43,0x00,0x5B,0x00,0x73,
			       0x00,0x8D,0x00,0xB7,0x00,0xD9,0x01,0x11,
			       0x01,0x3F,0x01,0x84,0x01,0xBA,0x02,0x0C,
			       0x02,0x2E,0x02,0x3C,0x02,0x8A,0x02,0xC7,
			       0x02,0xEA,0x03,0x1A,0x03,0x40,0x03,0x6B,
			       0x03,0x81,0x03,0xA1,0x03,0xB0,0x03,0xD0,
			       0x03,0xE0,0x03,0xFE}},
	{LCM_SEND(55), {53, 0, 0xD5,
			       0x00,0x01,0x00,0x43,0x00,0x5B,0x00,0x73,
			       0x00,0x8D,0x00,0xB7,0x00,0xD9,0x01,0x11,
			       0x01,0x3F,0x01,0x84,0x01,0xBA,0x02,0x0C,
			       0x02,0x2E,0x02,0x3C,0x02,0x8A,0x02,0xC7,
			       0x02,0xEA,0x03,0x1A,0x03,0x40,0x03,0x6B,
			       0x03,0x81,0x03,0xA1,0x03,0xB0,0x03,0xD0,
			       0x03,0xE0,0x03,0xFE}},
	{LCM_SEND(55), {53, 0, 0xD6,
			       0x00,0x01,0x00,0x43,0x00,0x5B,0x00,0x73,
			       0x00,0x8D,0x00,0xB7,0x00,0xD9,0x01,0x11,
			       0x01,0x3F,0x01,0x84,0x01,0xBA,0x02,0x0C,
			       0x02,0x2E,0x02,0x3C,0x02,0x8A,0x02,0xC7,
			       0x02,0xEA,0x03,0x1A,0x03,0x40,0x03,0x6B,
			       0x03,0x81,0x03,0xA1,0x03,0xB0,0x03,0xD0,
			       0x03,0xE0,0x03,0xFE}},
	/* Display on */
	{LCM_SEND(1), {0x11}}, // sleep out
	{LCM_SLEEP(120),},

	{LCM_SEND(1), {0x29}}, // display on
	{LCM_SLEEP(10),},
	//{LCM_SEND(1), {0x2c}}, // normal on
};

static LCM_Init_Code sleep_in[] =  {
	{LCM_SEND(1), {0x28}},
	{LCM_SLEEP(10)},
	{LCM_SEND(1), {0x10}},
	{LCM_SLEEP(120)},
	{LCM_SEND(2), {0x4f, 0x01}},
};

static LCM_Init_Code sleep_out[] =  {
	{LCM_SEND(1), {0x11}},
	{LCM_SLEEP(120)},
	{LCM_SEND(1), {0x29}},
	{LCM_SLEEP(20)},
};

static LCM_Force_Cmd_Code rd_prep_code_1[]={
	{0x37, {LCM_SEND(2), {0x1, 0}}},
};

static LCM_Force_Cmd_Code rd_prep_code_3[]={
	{0x37, {LCM_SEND(2), {0x3, 0}}},
};

static int32_t nt35510_mipi_init(struct panel_spec *self)
{
	int32_t i;
	LCM_Init_Code *init = init_data;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	pr_debug("nt35510_mipi_init\n");

	mipi_set_cmd_mode();

	for(i = 0; i < ARRAY_SIZE(init_data); i++){
		tag = (init->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(init->data, (init->tag & LCM_TAG_MASK));
			udelay(90);
		}else if(tag & LCM_TAG_SLEEP){
			udelay((init->tag & LCM_TAG_MASK) * 1000);
		}
		init++;
	}
	return 0;
}

static uint32_t nt35510_readid(struct panel_spec *self)
{
	/*Jessica TODO: need read id*/
	return 0x10;
}

static int32_t nt35510_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk(KERN_DEBUG "nt35510_enter_sleep, is_sleep = %d\n", is_sleep);

	if(is_sleep){
		sleep_in_out = sleep_in;
		size = ARRAY_SIZE(sleep_in);
	}else{
		sleep_in_out = sleep_out;
		size = ARRAY_SIZE(sleep_out);
	}

	for(i = 0; i <size ; i++){
		tag = (sleep_in_out->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(sleep_in_out->data, (sleep_in_out->tag & LCM_TAG_MASK));
		}else if(tag & LCM_TAG_SLEEP){
			msleep((sleep_in_out->tag & LCM_TAG_MASK));
		}
		sleep_in_out++;
	}
	return 0;
}

static uint32_t nt35510_readpowermode(struct panel_spec *self)
{
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code_1;
	uint8_t read_data[1] = {0};
	int32_t read_rtn = 0;
	unsigned int tag = 0;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;

	pr_debug("lcd_nt35510_mipi read power mode!\n");
	mipi_set_cmd_mode();

	for(j = 0; j < 4; j++){
		rd_prepare = rd_prep_code_1;
		for(i = 0; i < ARRAY_SIZE(rd_prep_code_1); i++){
			tag = (rd_prepare->real_cmd_code.tag >> 24);
			if(tag & LCM_TAG_SEND){
				mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data, (rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			}else if(tag & LCM_TAG_SLEEP){
				udelay((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK) * 1000);
			}
			rd_prepare++;
		}

		read_rtn = mipi_force_read(0x0A, 1,(uint8_t *)read_data);
		printk("lcd_nt35510 mipi read power mode 0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);

		if((0x9c == read_data[0])  && (0 == read_rtn)){
			pr_debug("lcd_nt35510_mipi read power mode success!\n");
			return 0x9c;
		}
	}

	return 0x0;
}

static int32_t nt35510_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

	/*mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;*/
	/*mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;*/

	pr_debug("nt35510_check_esd!\n");


	/*mipi_set_lp_mode();*/
	power_mode = nt35510_readpowermode(self);
	/*mipi_set_hs_mode();*/

	if (power_mode == 0x9c) {
		pr_debug("nt35510_check_esd OK!\n");
		return 1;
	} else {
		printk("nt35510_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}

static struct panel_operations lcd_nt35510_mipi_operations = {
	.panel_init = nt35510_mipi_init,
	.panel_readid = nt35510_readid,
	.panel_enter_sleep = nt35510_enter_sleep,
	.panel_esd_check = nt35510_check_esd,
};

static struct timing_rgb lcd_nt35510_mipi_timing = {
	.hfp = 20,  /* unit: pixel */
	.hbp = 20,
	.hsync = 4,
	.vfp = 10, /*unit: line*/
	.vbp = 10,
	.vsync = 6,
};

static struct info_mipi lcd_nt35510_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_CMD,//SPRDFB_MIPI_MODE_VIDEO,//SPRDFB_MIPI_MODE_CMD,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 2,
	.phy_feq = 500*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_nt35510_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_nt35510_mipi_spec = {
	.width = 480,
	.height = 800,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.info = {
		.mipi = &lcd_nt35510_mipi_info
	},
	.ops = &lcd_nt35510_mipi_operations,
};


struct panel_cfg lcd_nt35510_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x55,
	.lcd_name = "lcd_nt35510_mipi",
	.panel = &lcd_nt35510_mipi_spec,
};

ssize_t lcd_panelName_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "NT35510");
}

ssize_t lcd_MTP_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	((unsigned char*)buf)[0] = 0x55;
	((unsigned char*)buf)[1] = 0xBC;
	((unsigned char*)buf)[2] = 0xC0;    

	printk("ldi mtpdata: %x %x %x\n", buf[0], buf[1], buf[2]);

	return 3;
}

ssize_t lcd_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "INH_%x%x%x",0x55, 0xBC, 0xC0);
}


static DEVICE_ATTR(lcd_MTP, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, lcd_MTP_show, NULL);
static DEVICE_ATTR(lcd_panelName, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, lcd_panelName_show, NULL);
static DEVICE_ATTR(lcd_type, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, lcd_type_show, NULL);

static int __init lcd_nt35510_mipi_init(void)
{
	struct device *dev_t;
	struct class *lcd_class;

	lcd_class = class_create(THIS_MODULE, "lcd_status");

	if (IS_ERR(lcd_class)) {
		printk("Failed to create lcd_class!\n");
		return PTR_ERR( lcd_class );
	}

	dev_t = device_create( lcd_class, NULL, 0, "%s", "lcd_status");

	if (device_create_file(dev_t, &dev_attr_lcd_panelName) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_lcd_panelName.attr.name);
	if (device_create_file(dev_t, &dev_attr_lcd_MTP) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_lcd_MTP.attr.name);
	if (device_create_file(dev_t, &dev_attr_lcd_type) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_lcd_type.attr.name);    

	return sprdfb_panel_register(&lcd_nt35510_mipi);
}

subsys_initcall(lcd_nt35510_mipi_init);
