/*
 * Copyright (C) 2012 by Ming.Jiang@qca.qualcomm.com
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
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/random.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/wlan_plat.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <linux/regulator/consumer.h>
#include <mach/regulator.h>

extern void sdhci_bus_scan(void);

#define CUSTOMER_MAC_FILE 	"/data/wifimac.txt"
#define IFHWADDRLEN             6

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS        1
#define HIF_DMA_BUFFER_SIZE                     (32 * 1024)
#define GET_INODE_FROM_FILEP(filp) \
			((filp)->f_path.dentry->d_inode)

typedef struct mem_prealloc {
        void *mem_ptr;
        unsigned long size;
} mem_prealloc_t;

static mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
		{ NULL, (HIF_DMA_BUFFER_SIZE) },
};

static int wifi_power_on;

static int wlan_readwrite_file(const char *filename,
			   char *rbuf, const char *wbuf, size_t length)
{
	int ret = 0;
	struct file *filp = (struct file *)-ENOENT;
	mm_segment_t oldfs;
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	do {
		int mode = (wbuf) ? O_RDWR : O_RDONLY;
		filp = filp_open(filename, mode, S_IRUSR);

		if (IS_ERR(filp) || !filp->f_op) {
			ret = -ENOENT;
			break;
		}

		if (length == 0) {
			/* Read the length of the file only */
			struct inode    *inode;

			inode = GET_INODE_FROM_FILEP(filp);
			if (!inode) {
				printk(KERN_ERR "%s: Error 2\n", __func__);
				ret = -ENOENT;
				break;
			}
			ret = i_size_read(inode->i_mapping->host);
			break;
		}

		if (wbuf) {
			ret = filp->f_op->write(filp, wbuf, length, &filp->f_pos);
			if (ret < 0) {
				printk(KERN_ERR
				       "%s: Error 3\n", __func__);
				break;
			}
		} else {
			ret = filp->f_op->read(filp, rbuf, length, &filp->f_pos);
			if (ret < 0) {
				printk(KERN_ERR
				       "%s: Error 4\n", __func__);
				break;
			}
		}
	} while (0);

	if (!IS_ERR(filp))
		filp_close(filp, NULL);

	set_fs(oldfs);
	pr_info("%s ret = %d\n", __func__, ret);
	return ret;
}

static void wlan_create_random_mac(unsigned char *ptr_mac)
{
        uint rand_mac;

        srandom32((uint)jiffies);
        rand_mac = random32();
        ptr_mac[0] = 0x00;
        ptr_mac[1] = 0x03;
        ptr_mac[2] = 0x7f;
        ptr_mac[3] = (unsigned char)rand_mac;
        ptr_mac[4] = (unsigned char)(rand_mac >> 8);
        ptr_mac[5] = (unsigned char)(rand_mac >> 16);

        pr_info("wlan use random MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                ptr_mac[0], ptr_mac[1], ptr_mac[2],
                ptr_mac[3], ptr_mac[4], ptr_mac[5]);
}

int __init wlan_init_mem(void)
{
	int i;
        for(i = 0; i < PREALLOC_WLAN_NUMBER_OF_SECTIONS; i++) {
        	wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
                                                        GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL) {
			pr_info("%s failed!!!\n", __func__);
			return -ENOMEM;
		}
        }

	return 0;
}

/*
 * if called 3.3v 1.8v will be powered on when boot
 * and will never be powered off. If AR6005 did not 
 * have current leakage with platform Use this 
 * method since it is a better solution for power
 */
int __init wlan_init_ldo(void)
{
	int err;
	struct regulator *regulator_wifi_bt = NULL;

	if (GPIO_WIFI_POWERON > 0) {
		gpio_request(GPIO_WIFI_POWERON, "wifi_pwron");
		gpio_direction_output(GPIO_WIFI_POWERON, 1);
	}

	regulator_wifi_bt = regulator_get(NULL, "vddsd1");
	if (IS_ERR(regulator_wifi_bt)){
		pr_err("ATH(adapter): Can't get regulator for VDD_SDIO\n");
		return -1;
	}

	err = regulator_set_voltage(regulator_wifi_bt, 1800000, 1800000);
	if (err){
		pr_err("ATH(adapter): Can't set regulator to valtage 1.8V\n");
		return -1;
	}

	regulator_enable(regulator_wifi_bt);

	regulator_wifi_bt = regulator_get(NULL, "vddcmmb1p8");
	if (IS_ERR(regulator_wifi_bt)){
		pr_err("ATH(adapter): Can't get regulator for VDD_SDIO\n");
		return -1;
	}

	err = regulator_set_voltage(regulator_wifi_bt, 1800000, 1800000);
	if (err){
		pr_err("ATH(adapter): Can't set regulator to valtage 1.8V\n");
		return -1;
	}

	regulator_enable(regulator_wifi_bt);

	gpio_request(GPIO_WIFI_RESET , "wifi_rst");
	gpio_request(GPIO_WIFI_IRQ , "wifi_irq");
	gpio_direction_input(GPIO_WIFI_IRQ);

	wifi_power_on = 1;
	return 0;
}

int wlan_device_power(int on)
{
	if (wifi_power_on) {
		pr_info("wlan 3.3v 1.8v already on\n");
		return -1;
	}

	if (on) {
		pr_info("wlan 3.3v 1.8v on\n");
	} else {
		pr_info("wlan 3.3v 1.8v off\n");
	}

	return 0;
}

static unsigned char wlan_mac_addr[IFHWADDRLEN] = { 0xFF,0x22,0x33,0x44,0x55,0x66 };

static unsigned char char2bin( char m)
{
        if (( m >= 'a') && (m <= 'f')) {
                return m - 'a' + 10;
        }
        if (( m >= 'A') && (m <= 'F')) {
                return m - 'A' + 10;
        }
        if (( m >= '0') && (m <= '9')) {
                return m - '0';
        }
	return 0; /*silence the compiler*/ 
}

static int wlan_device_get_mac_addr(unsigned char *buf)
{
	int rc = 0;
	char macaddr[20];
        
	if (wlan_mac_addr[0] != 0xFF)
		goto MAC_COPY;

        rc = wlan_readwrite_file(CUSTOMER_MAC_FILE, macaddr, NULL, 17);
	if (rc >= 0) {
	        wlan_mac_addr[0] = (unsigned char)((char2bin(macaddr[0]) << 4) | char2bin(macaddr[1]));
        	wlan_mac_addr[1] = (unsigned char)((char2bin(macaddr[3]) << 4) | char2bin(macaddr[4]));
	        wlan_mac_addr[2] = (unsigned char)((char2bin(macaddr[6]) << 4) | char2bin(macaddr[7]));
        	wlan_mac_addr[3] = (unsigned char)((char2bin(macaddr[9]) << 4) | char2bin(macaddr[10]));
	        wlan_mac_addr[4] = (unsigned char)((char2bin(macaddr[12]) << 4) | char2bin(macaddr[13]));
        	wlan_mac_addr[5] = (unsigned char)((char2bin(macaddr[15]) << 4) | char2bin(macaddr[16]));
	} else {
	    wlan_create_random_mac(wlan_mac_addr);
	}
MAC_COPY:
        memcpy(buf, wlan_mac_addr, IFHWADDRLEN);
        pr_info("%s wlan wifi mac: %x:%x:%x:%x:%x:%x\n", __func__, buf[0], buf[1], buf[2], 
								buf[3], buf[4], buf[5]);

	return 0;
}

int wlan_device_toggle_power_gpio(int on)
{
	if (on) {
		gpio_direction_output(GPIO_WIFI_RESET , 0);
		msleep(100);
		gpio_direction_output(GPIO_WIFI_RESET , 1);
		msleep(200);
		pr_info("========wlan back to live=========\n");
	} else {
		gpio_direction_output(GPIO_WIFI_RESET , 0);
		pr_info("========wlan power cut=========\n");
	}
        return 0;
}

int wlan_device_set_carddetect(int val)
{
	pr_info("wlan sdio card detect\n");
	sdhci_bus_scan(); 
	mdelay(50);
	return 0;
}

static void *wlan_device_mem_prealloc(int section, unsigned long size)
{
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
        if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

static struct wifi_platform_data wlan_device_control = {
        .set_power      = wlan_device_power,
        .set_reset      = wlan_device_toggle_power_gpio,
        .set_carddetect = wlan_device_set_carddetect,
        .mem_prealloc   = wlan_device_mem_prealloc,
        .get_mac_addr   = wlan_device_get_mac_addr,
};

static struct platform_device ath_wlan_device = {
	.name   = "wlan_ar6000",
        .id     = 1,
        .dev    = {
                .platform_data = &wlan_device_control,
        },
};

static int __init wlan_device_init(void)
{
        int ret;

        wlan_init_mem();
	/* if AR6005 has power leakage with SPRD Comment it
         * power solution will degrade to all on all off mode
         * automatically
         */
	wlan_init_ldo();
        ret = platform_device_register(&ath_wlan_device);

        return ret;
}

late_initcall(wlan_device_init);
