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

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <mach/sci.h>
#include <mach/regulator.h>
#include <mach/hardware.h>
#include <mach/regs_ahb.h>
#include <mach/regs_glb.h>
#include <mach/board.h>
#include <linux/interrupt.h>

#include "sc8825.h"
#ifdef CONFIG_ARCH_SC8825
#include "sdhci.h"
#endif

#define DRIVER_NAME "sprd-sdhci"

/* regulator use uv to set voltage */
#define     SDIO_VDD_VOLT_1V8       1800000
#define     SDIO_VDD_VOLT_3V0       3000000
#define     SDIO_VDD_VOLT_3V3       3300000

/* MMC_RESTORE_REGS depends on devices of 3rd parties, sdio registers
 *will be cleared after suspend. host wakeup only supported in sdio host1
 */
#define MMC_RESTORE_REGS

/* MMC_HOST_WAKEUP_SUPPORTED should be defined if sdio devices use data1
 *to wake the system up
 */

#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
#include <mach/pinmap.h>
static unsigned int sdio_wakeup_irq;
#define HOST_WAKEUP_GPIO     22
#endif

#ifdef CONFIG_MMC_BUS_SCAN
static struct sdhci_host *sdhci_host_g = NULL;
#endif

extern void mmc_power_off(struct mmc_host* mmc);
extern void mmc_power_up(struct mmc_host* mmc);

static void *sdhci_get_platdata(struct sdhci_host *host)
{
	return ((struct sprd_host_data *)sdhci_priv(host))->platdata;
}

/* we don't need to restore sdio0, because of CONFIG_MMC_BLOCK_DEFERED_RESUME=y */
#ifdef MMC_RESTORE_REGS
static void sdhci_save_regs(struct sdhci_host *host)
{
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);
	if (!host_pdata->regs.is_valid) return;
	host_pdata->regs.addr = sdhci_readl(host, SDHCI_DMA_ADDRESS);
	host_pdata->regs.blk_size = sdhci_readw(host, SDHCI_BLOCK_SIZE);
	host_pdata->regs.blk_cnt = sdhci_readw(host, SDHCI_BLOCK_COUNT);
	host_pdata->regs.arg = sdhci_readl(host, SDHCI_ARGUMENT);
	host_pdata->regs.tran_mode = sdhci_readw(host, SDHCI_TRANSFER_MODE);
	host_pdata->regs.ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	host_pdata->regs.power = sdhci_readb(host, SDHCI_POWER_CONTROL);
	host_pdata->regs.clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
}

static void sdhci_restore_regs(struct sdhci_host *host)
{
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);
	if (!host_pdata->regs.is_valid) return;
	sdhci_writel(host, host_pdata->regs.addr, SDHCI_DMA_ADDRESS);
	sdhci_writew(host, host_pdata->regs.blk_size, SDHCI_BLOCK_SIZE);
	sdhci_writew(host, host_pdata->regs.blk_cnt, SDHCI_BLOCK_COUNT);
	sdhci_writel(host, host_pdata->regs.arg, SDHCI_ARGUMENT);
	sdhci_writew(host, host_pdata->regs.tran_mode, SDHCI_TRANSFER_MODE);
	sdhci_writeb(host, host_pdata->regs.ctrl, SDHCI_HOST_CONTROL);
	sdhci_writeb(host, host_pdata->regs.power, SDHCI_POWER_CONTROL);
	sdhci_writew(host, host_pdata->regs.clk, SDHCI_CLOCK_CONTROL);
}
#ifdef CONFIG_MMC_DEBUG
static void sdhci_dump_saved_regs(struct sdhci_host *host)
{
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);
	if (!host_pdata->regs.is_valid) return;
	printk("%s, host_addr:0x%x\n", host->hw_name, host_pdata->regs.addr);
	printk("%s, host_blk_size:0x%x\n", host->hw_name, host_pdata->regs.blk_size);
	printk("%s, host_blk_cnt:0x%x\n", host->hw_name, host_pdata->regs.blk_cnt);
	printk("%s, host_arg:0x%x\n", host->hw_name, host_pdata->regs.arg);
	printk("%s, host_tran_mode:0x%x\n", host->hw_name, host_pdata->regs.tran_mode);
	printk("%s, host_ctrl:0x%x\n", host->hw_name, host_pdata->regs.ctrl);
	printk("%s, host_power:0x%x\n", host->hw_name, host_pdata->regs.power);
	printk("%s, host_clk:0x%x\n", host->hw_name, host_pdata->regs.clk);
}
#endif
#endif

#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
static irqreturn_t sdhci_wakeup_irq_handler(int irq, void *dev)
{
	printk("sdhci_wakeup_irq_handler\n");
	/* Disable interrupt before calling handler */
	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

void sdhci_set_data1_to_gpio(struct sdhci_host *host)
{
	unsigned int val;
	/* configurate sdio1 data1 to gpio when system in deep sleep */
	val = BITS_PIN_DS(1) | BITS_PIN_AF(3)  |
		BIT_PIN_WPU  | BIT_PIN_SLP_WPU |
		BIT_PIN_SLP_IE ;
	__raw_writel( val, CTL_PIN_BASE + REG_PIN_SD2_D1 );

	printk("%s, PIN_SD2_D1_REG:0x%x\n", __func__, __raw_readl(REG_PIN_SD2_D1));
	printk("sdhci_set_data1_to_gpio done\n");
}

void sdhci_set_gpio_to_data1(struct sdhci_host *host)
{
	unsigned int val;
	/* configurate sdio1 gpio to data1 when system wakeup */
	val = __raw_readl( CTL_PIN_BASE + REG_PIN_SD2_D1 );
	val = BITS_PIN_DS(1) | BITS_PIN_AF(0)  |
		BIT_PIN_WPU  | BIT_PIN_SLP_NUL |
		BIT_PIN_SLP_Z ;
	__raw_writel( val, CTL_PIN_BASE + REG_PIN_SD2_D1 );

	printk("%s, REG_PIN_SD2_D1:0x%x\n", __func__, __raw_readl(REG_PIN_SD2_D1));
	printk("sdhci_set_gpio_to_data1 done\n");
}


static void  sdhci_host_wakeup_set( struct sdhci_host *host )
{
	unsigned int val;
	int ret;


	if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
		sdhci_set_data1_to_gpio(host);
		gpio_request(HOST_WAKEUP_GPIO, "host_wakeup_irq");
		sdio_wakeup_irq = gpio_to_irq(HOST_WAKEUP_GPIO);
		gpio_direction_input(HOST_WAKEUP_GPIO);
		ret = request_threaded_irq(sdio_wakeup_irq, sdhci_wakeup_irq_handler, NULL,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "host_wakeup_irq", host);
		if(ret){
			printk(KERN_ERR "%s, request threaded irq error:%d\n",
				mmc_hostname(host->mmc), ret);
			return;
		}
		enable_irq_wake(sdio_wakeup_irq);
	}
	return;
}

static void  sdhci_host_wakeup_clear(struct sdhci_host *host)
{
       if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
		disable_irq_wake(sdio_wakeup_irq);
		free_irq(sdio_wakeup_irq, host);
		gpio_free(HOST_WAKEUP_GPIO);
		sdhci_set_gpio_to_data1(host);
        }
	return;
}
#endif

#ifdef CONFIG_MMC_BUS_SCAN
/*
 *  force card detection
 *  some sdio devices can use this API for detection
 */
void sdhci_bus_scan(void)
{
#ifdef CONFIG_MMC_BUS_SCAN
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		printk("%s, entry\n", __func__);
		if (sdhci_host_g->ops->set_clock) {
			sdhci_host_g->ops->set_clock(sdhci_host_g, 1);
		}

		//sdhci_reinit(sdhci_host_g);
		mmc_detect_change(sdhci_host_g->mmc, 0);
	}
#endif
	return;
}
EXPORT_SYMBOL_GPL(sdhci_bus_scan);

unsigned int sdhci_wifi_detect_isbusy(void) {
    unsigned int busy = 0;
#ifdef CONFIG_MMC_BUS_SCAN
    if(sdhci_host_g && sdhci_host_g->mmc) {
        busy = work_busy(&sdhci_host_g->mmc->detect.work);
    }
#endif
    return busy;
}
EXPORT_SYMBOL_GPL(sdhci_wifi_detect_isbusy);

#endif

/*
 *   set indicator indicates that whether any devices attach on sdio bus.
 *   NOTE: devices must already attached on bus before calling this function.
 *   @ on: 0---deattach devices
 *         1---attach devices on bus
 *   @ return: 0---set indicator ok
 *             -1---no devices on sdio bus
 */
int sdhci_device_attach(int on)
{
	struct mmc_host *mmc = NULL;
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		mmc = sdhci_host_g->mmc;
		if(mmc->card){
			unsigned long flags;
			sdhci_host_g->dev_attached = on;
			if(!on){
				spin_lock_irqsave(&mmc->lock, flags);
				mmc->rescan_disable = 1;
				spin_unlock_irqrestore(&mmc->lock, flags);
				if (cancel_delayed_work_sync(&mmc->detect))
				    wake_unlock(&mmc->detect_wake_lock);
			}else{
				spin_lock_irqsave(&mmc->lock, flags);
				mmc->rescan_disable = 0;
				spin_unlock_irqrestore(&mmc->lock, flags);
			}
		}else{
			/* no devices */
			sdhci_host_g->dev_attached = 0;
			return -1;
		}
		return 0;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sdhci_device_attach);

/*
 *   Slave start sdhci_bus_scan Ops then check SDIO card attach Bus status
 *
 *   @ return:  true--- SDIO device attach ready
 *              false---SDIO device attach not ready
 */
int sdhci_device_attached()
{
	struct mmc_host *mmc = NULL;
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		mmc = sdhci_host_g->mmc;
		if(mmc->card){
			return true;
		}else{
			return false;
		}
	}
	return false;
}
EXPORT_SYMBOL_GPL(sdhci_device_attached);
/**
 * sdhci_sprd_get_max_clk - callback to get maximum clock frequency.
 * @host: The SDHCI host instance.
 *
 * Callback to return the maximum clock rate acheivable by the controller.
*/
static unsigned int sdhci_sprd_get_max_clk(struct sdhci_host *host)
{
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);

	return host_pdata->max_clock;
}

/**
 * sdhci_sprd_set_base_clock - set base clock for SDIO module
 * @host:  sdio host to be set.
 * @clock: The clock rate being requested.
*/
static void sdhci_sprd_set_base_clock(struct sdhci_host *host)
{
	struct clk *clk_parent;
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);

	/* Select the clk source of SDIO, default is 96MHz */
	host->clk = clk_get(NULL, host_pdata->clk_name);
	clk_parent = clk_get(NULL, host_pdata->clk_parent);
	BUG_ON(IS_ERR(host->clk) || IS_ERR(clk_parent));
	clk_set_parent(host->clk, clk_parent);

	/* if parent clock from mpll, will set parent clock div*/
	if (0 == strcmp(host_pdata->clk_parent, "clk_sdio_src")) {
		// 1.get cur mpll clock. defalut is 1GHZ.
		// 2.set div, parent clock is set 90MHZ.
		clk_set_rate(clk_parent, 90000000);
	}

	pr_debug("after set sd clk, CLK_GEN5:0x%x\n", sci_glb_raw_read(REG_GLB_CLK_GEN5));
	return;
}

/**
 * sdhci_sprd_enable_clock - enable or disable sdio base clock
 * @host:  sdio host to be set.
 * @clock: The clock enable(clock>0) or disable(clock==0).
 *
*/
static void sdhci_sprd_enable_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sprd_host_data *host_data= sdhci_priv(host);
	if(clock == 0){
		if (host_data->clk_enable) {
			clk_disable(host->clk);
			host_data->clk_enable = 0;
		}
	}else{
		if (0 == host_data->clk_enable) {
			clk_enable(host->clk);
			host_data->clk_enable = 1;
		}
	}
	pr_debug("clock:%d, host->clock:%d, AHB_CTL1:0x%x\n", clock,host->clock,
			sci_glb_raw_read(REG_AHB_AHB_CTL1));
	return;
}

/*
*   The vdd_sdio is supplied by external LDO, power bit in register xxx is useless
*/
static void sdhci_sprd_set_power(struct sdhci_host *host, unsigned int power)
{
    int ret = 0;
    unsigned int volt_level = 0;
#ifdef CONFIG_ARCH_SC8825
    unsigned int volt_ext_level = SDIO_VDD_VOLT_3V0;
#else
    unsigned int volt_ext_level = SDIO_VDD_VOLT_3V0;
#endif
    struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);

    if(host->vmmc == NULL)
        return;

    switch(power){
        case SDHCI_POWER_180:
            volt_level = SDIO_VDD_VOLT_1V8;
            break;
        case SDHCI_POWER_300:
            volt_level = SDIO_VDD_VOLT_3V0;
            break;
        case SDHCI_POWER_330:
            volt_level = SDIO_VDD_VOLT_3V3;
            break;
        default:
            ;
    }

    printk("%s, power:%d, set regulator voltage:%d\n", mmc_hostname(host->mmc), power, volt_level);

    if(volt_level == 0) {
       // if (regulator_is_enabled(host->vmmc)) {
            ret = regulator_force_disable(host->vmmc);
            if(ret) {
                printk(KERN_ERR "%s, disable regulator error:%d\n", mmc_hostname(host->mmc), ret);
                return;
         //   }
        }
        if (host->vmmc_ext != NULL) {
            //if (regulator_is_enabled(host->vmmc_ext)) {
                ret = regulator_force_disable(host->vmmc_ext);
                if(ret) {
                    printk(KERN_ERR "%s, disable regulator ext error:%d\n", mmc_hostname(host->mmc), ret);
                    return;
                }
          //  }
        }
    } else {
        ret = regulator_set_voltage(host->vmmc, volt_level, volt_level);
        if(ret) {
            printk(KERN_ERR "%s, set vmmc voltage error:%d\n", mmc_hostname(host->mmc), ret);
            return;
        }
        if (host->vmmc_ext != NULL) {
            ret = regulator_set_voltage(host->vmmc_ext, volt_ext_level, volt_ext_level);
            if(ret) {
                printk(KERN_ERR "%s, set vmmc ext voltage error:%d\n", mmc_hostname(host->mmc), ret);
                return;
            }
        }
        if(!regulator_is_enabled(host->vmmc)) {
            ret = regulator_enable(host->vmmc);
            if(ret) {
                printk(KERN_ERR "%s, enabel regulator error:%d\n", mmc_hostname(host->mmc), ret);
                return;
            }
        }
        if (host->vmmc_ext != NULL) {
            if(!regulator_is_enabled(host->vmmc_ext)) {
                ret = regulator_enable(host->vmmc_ext);
                if(ret) {
                    printk(KERN_ERR "%s, enabel regulator ext error:%d\n", mmc_hostname(host->mmc), ret);
                    return;
                }
            }
        }
    }
}

static struct sdhci_ops sdhci_sprd_ops = {
	.get_max_clock		= sdhci_sprd_get_max_clk,
	.set_clock		= sdhci_sprd_enable_clock,
	.set_power		= sdhci_sprd_set_power,
	.save_regs		= sdhci_save_regs,
	.restore_regs	= sdhci_restore_regs,
};

#define SPL_EMMC_DATA_ADDR			0x5C80
#define EMMC_PRIV_DATA				0x11
#define EMMC_MAGIC_SDR50			0xEAC0AD50
#define EMMC_MAGIC_DDR50			0xEAC1DD50

typedef struct {
	unsigned int tag;
	unsigned int len;
	unsigned int data[];
} spl_priv_data;

typedef struct {
	unsigned int flag;
	unsigned int para1;
	unsigned int para2;
	unsigned int para3;
	unsigned int para4;
	unsigned int check_sum;
} emmc_priv_data;

static void emmc_get_spl_data(struct sdhci_host* host)
{
	struct sprd_host_data *host_data= sdhci_priv(host);
	emmc_priv_data *emmc_data;
	spl_priv_data *spl_data;
	unsigned char sdr50_flag, ddr50_flag, i;

	sdr50_flag = 0;
	ddr50_flag = 0;

	spl_data = (spl_priv_data *)(SPRD_IRAM_BASE + SPL_EMMC_DATA_ADDR - SPRD_IRAM_PHYS);
	if (spl_data->tag == EMMC_PRIV_DATA) {
		/*only sdr50 and ddr50 parameters*/
		emmc_data = (emmc_priv_data *)(spl_data->data);
		for ( i = 0; i < 2; i++) {
			if (emmc_data->flag == EMMC_MAGIC_SDR50) {
				unsigned int check_sum = 0;
				check_sum ^= emmc_data->flag;
				check_sum ^= emmc_data->para1;
				check_sum ^= emmc_data->para2;
				check_sum ^= emmc_data->para3;
				check_sum ^= emmc_data->para4;
				if (check_sum == emmc_data->check_sum) {
					host_data->sdr50_clk_pin = emmc_data->para1;
					host_data->sdr50_data_pin = emmc_data->para2;
					host_data->sdr50_write_delay = emmc_data->para3;
					host_data->sdr50_read_pos_delay = emmc_data->para4;
					sdr50_flag = 1;
					pr_debug("emmc get SDR50 para: 0x%x, 0x%x, 0x%x, 0x%x\n\r",
							emmc_data->para1, emmc_data->para2,
							emmc_data->para3,emmc_data->para4);
				}
			}
			else if (emmc_data->flag == EMMC_MAGIC_DDR50) {
				unsigned int check_sum = 0;
				check_sum ^= emmc_data->flag;
				check_sum ^= emmc_data->para1;
				check_sum ^= emmc_data->para2;
				check_sum ^= emmc_data->para3;
				check_sum ^= emmc_data->para4;
				if (check_sum == emmc_data->check_sum) {
					host_data->ddr50_clk_pin = emmc_data->para1;
					host_data->ddr50_write_delay = emmc_data->para2;
					host_data->ddr50_read_pos_delay = emmc_data->para3;
					host_data->ddr50_read_neg_delay = emmc_data->para4;
					ddr50_flag = 1;
					pr_debug("emmc get DDR50 para: 0x%x, 0x%x, 0x%x, 0x%x\n\r",
							emmc_data->para1, emmc_data->para2,
							emmc_data->para3,emmc_data->para4);
				}
			}
			emmc_data ++;
		}
	}
	/* if not get parameters , used default parameters.*/
	if (sdr50_flag == 0) {
		host_data->sdr50_clk_pin = 0;
		host_data->sdr50_data_pin = 1;
		host_data->sdr50_write_delay = 0x20;
		host_data->sdr50_read_pos_delay = 0x08;
	}
	if (ddr50_flag == 0) {
		host_data->ddr50_clk_pin = 0;
		host_data->ddr50_write_delay = 0x18;
		host_data->ddr50_read_pos_delay = 0x07;
		host_data->ddr50_read_neg_delay = 0x05;
	}
}

static void sdhci_module_init(struct sdhci_host* host)
{
	struct sprd_host_platdata *host_pdata;
	host_pdata = sdhci_get_platdata(host);
	/* Enable SDIO Module */
	sci_glb_set(REG_AHB_AHB_CTL0, host_pdata->enb_bit);
	/* Reset SDIO Module */
	sci_glb_set(REG_AHB_SOFT_RST, host_pdata->rst_bit);
	sci_glb_clr(REG_AHB_SOFT_RST, host_pdata->rst_bit);
	sdhci_sprd_set_base_clock(host);
	host->ops->set_clock(host, true);
	if ( !strcmp(host->hw_name, "sprd-emmc") ) {
		emmc_get_spl_data(host);
		/* add alter pin driver strength*/
	}
}

extern struct class *sec_class;
static struct device *sd_detection_cmd_dev;
int gpio_detect = 777;

static ssize_t sd_detection_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int	detect;
	if(gpio_detect != 777)
	{
		detect = gpio_get_value(gpio_detect);

		/* File Location : /sys/class/sec/sdcard/status */
		pr_info("%s : detect = %d.\n", __func__,  detect);
		detect = !detect;
		if (detect)
		{
			pr_info("sdhci: card inserted.\n");
			return sprintf(buf, "Insert\n");
		}
		else
		{
			pr_info("sdhci: card removed.\n");
			return sprintf(buf, "Remove\n");
		}
	}
	else
	{
		pr_info("sdhci: card removed.\n");
		return sprintf(buf, "Remove\n");
	}
}

static DEVICE_ATTR(status, 0444, sd_detection_cmd_show, NULL);

static int __devinit sdhci_sprd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct resource *res;
	int ret, irq;
#ifdef CONFIG_MMC_CARD_HOTPLUG
	int sd_detect_gpio;
	int detect_irq;
#endif
	struct sprd_host_data *host_data;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	host = sdhci_alloc_host(dev, sizeof(struct sprd_host_data));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	host_data = sdhci_priv(host);
	host_data->platdata = dev_get_platdata(dev);
	host_data->clk_enable = 0;
	BUG_ON(NULL == host_data->platdata);
	printk("sdio probe %s, vdd %s (%d), clk %s parent %s\n",
		host_data->platdata->hw_name,
		host_data->platdata->vdd_name,	host_data->platdata->volt_level,
		host_data->platdata->clk_name, host_data->platdata->clk_parent);

	platform_set_drvdata(pdev, host);
	host->ioaddr = (void __iomem *)res->start;
	pr_debug("sdio: host->ioaddr:0x%x\n", (u32)host->ioaddr);
	host->hw_name = (host_data->platdata->hw_name)?
		host_data->platdata->hw_name:pdev->name;
	host->ops = &sdhci_sprd_ops;
	/*
	 *   tiger don't have timeout value and cann't find card
	 *insert, write protection...
	 *   too sad
	 */
	host->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |\
		SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |\
		SDHCI_QUIRK_BROKEN_CARD_DETECTION|\
		SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
	host->irq = irq;
#ifdef CONFIG_MMC_CARD_HOTPLUG
	sd_detect_gpio = host_data->platdata->detect_gpio;
	if(sd_detect_gpio > 0){
		pr_info("%s, sd_detect_gpio:%d\n", __func__, sd_detect_gpio);

		if (0 == pdev->id){
			ret = gpio_request(sd_detect_gpio, "sdio0_detect");
		}else{
			ret = gpio_request(sd_detect_gpio, "sdio1_detect");
		}

		if (ret) {
			dev_err(dev, "cannot request gpio\n");
			return -1;
		}

		ret = gpio_direction_input(sd_detect_gpio);
		if (ret) {
			dev_err(dev, "gpio can not change to input\n");
			return -1;
		}

		detect_irq = gpio_to_irq(sd_detect_gpio);
		if (detect_irq < 0){
			dev_err(dev, "cannot alloc detect irq\n");
			return -1;
		}
		if (SDC_SLAVE_SD == pdev->id){
			gpio_detect = sd_detect_gpio;
		}
		host_data->detect_irq = detect_irq;
	}else{
		printk("%s, sd_detect_gpio == 0 \n", __func__ );
	}
#endif
	if (sd_detection_cmd_dev == NULL)
	{
		sd_detection_cmd_dev = device_create(sec_class, NULL, 0, NULL, "sdcard");

		if (IS_ERR(sd_detection_cmd_dev))
			pr_err("Fail to create sysfs dev\n");

		if (device_create_file(sd_detection_cmd_dev,&dev_attr_status) < 0)
			pr_err("Fail to create sysfs file\n");
	}

	host->vmmc = regulator_get(NULL, host_data->platdata->vdd_name);
	BUG_ON(IS_ERR(host->vmmc));
	regulator_enable(host->vmmc);
	if(host_data->platdata->vdd_ext_name) {
	    host->vmmc_ext = regulator_get(NULL, host_data->platdata->vdd_ext_name);
	    BUG_ON(IS_ERR(host->vmmc_ext));
	    regulator_enable(host->vmmc_ext);
	}

	host->clk = NULL;
	sdhci_module_init(host);

	host->mmc->caps |= MMC_CAP_HW_RESET;
    switch(pdev->id) {
    case SDC_SLAVE_CP:
        host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY | MMC_PM_WAKE_SDIO_IRQ;
        host->mmc->caps |= MMC_CAP_NONREMOVABLE;
        break;
    case SDC_SLAVE_WIFI:
        /* MMC_PM_WAKE_SDIO_IRQ does not suitable for bcm wifi suspend */
        host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY | MMC_PM_KEEP_POWER | MMC_PM_DISABLE_TIMEOUT_IRQ;
        host->mmc->pm_caps |= MMC_PM_KEEP_POWER;
        //host->mmc->caps |= MMC_CAP_NONREMOVABLE;
#ifdef CONFIG_PM_RUNTIME
        host->mmc->caps |= MMC_CAP_POWER_OFF_CARD;
#endif
        break;
    case SDC_SLAVE_EMMC:
#ifdef CONFIG_ARCH_SC8825
        host->caps = sdhci_readl(host, SDHCI_CAPABILITIES) & (~(SDHCI_CAN_VDD_330 | SDHCI_CAN_VDD_300));
        host->quirks |= SDHCI_QUIRK_MISSING_CAPS;
#endif
        host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY | MMC_PM_KEEP_POWER;
        host->mmc->caps |= MMC_CAP_NONREMOVABLE | MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR;
        break;
    case SDC_SLAVE_SD:
        host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
        break;
    default:
        break;
    }

#ifdef CONFIG_PM_RUNTIME
        switch(pdev->id) {
        case SDC_SLAVE_CP:
            break;
        case SDC_SLAVE_WIFI:
            pm_runtime_set_active(&pdev->dev);
            pm_runtime_set_autosuspend_delay(&pdev->dev, 500);
            pm_runtime_use_autosuspend(&pdev->dev);
            pm_runtime_enable(&pdev->dev);
            pm_runtime_no_callbacks(mmc_classdev(host->mmc));
            pm_suspend_ignore_children(mmc_classdev(host->mmc), true);
            pm_runtime_set_active(mmc_classdev(host->mmc));
            pm_runtime_enable(mmc_classdev(host->mmc));
            break;
        case SDC_SLAVE_EMMC:
        case SDC_SLAVE_SD:
            pm_suspend_ignore_children(&pdev->dev, true);
            pm_runtime_set_active(&pdev->dev);
            pm_runtime_set_autosuspend_delay(&pdev->dev, 500);
            pm_runtime_use_autosuspend(&pdev->dev);
            pm_runtime_enable(&pdev->dev);
        default:
            break;
        }
#endif

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}

#ifdef CONFIG_MMC_BUS_SCAN
	if (pdev->id == 1)
		sdhci_host_g = host;
#endif
	return 0;

err_add_host:
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&(pdev)->dev);
	pm_runtime_set_suspended(&(pdev)->dev);
#endif
	if (host_data->clk_enable) {
		clk_disable(host->clk);
		host_data->clk_enable = 0;
	}
	sdhci_free_host(host);
	return ret;
}

/*
 * TODO: acommplish the funcion.
 * SDIO driver is a build-in module
 */
static int __devexit sdhci_sprd_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sprd_host_data *host_data = sdhci_priv(host);

#ifdef CONFIG_PM_RUNTIME
	if (pm_runtime_suspended(&(pdev)->dev))
		pm_runtime_resume(&(pdev)->dev);
#endif
	sdhci_remove_host(host, 1);
	sdhci_free_host(host);

	if (host_data->clk_enable) {
		clk_disable(host->clk);
		host_data->clk_enable = 0;
	}

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&(pdev)->dev);
	pm_runtime_set_suspended(&(pdev)->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int sprd_mmc_host_runtime_suspend(struct device *dev) {
    int rc = -EBUSY;
    unsigned long flags;
    struct platform_device *pdev = container_of(dev, struct platform_device, dev);
    struct sdhci_host *host = platform_get_drvdata(pdev);
    struct mmc_host *mmc = host->mmc;
    if(dev->driver != NULL) {
            sdhci_runtime_suspend_host(host);
            spin_lock_irqsave(&host->lock, flags);
            if(host->ops->set_clock)
                host->ops->set_clock(host, 0);
            spin_unlock_irqrestore(&host->lock, flags);
            rc = 0;
    }
    return rc;
}

static int sprd_mmc_host_runtime_resume(struct device *dev) {
    unsigned long flags;
    struct platform_device *pdev = container_of(dev, struct platform_device, dev);
    struct sdhci_host *host = platform_get_drvdata(pdev);
    struct mmc_host *mmc = host->mmc;
    if(dev->driver != NULL) {
        if(host->ops->set_clock) {
            spin_lock_irqsave(&host->lock, flags);
            host->ops->set_clock(host, 1);
            spin_unlock_irqrestore(&host->lock, flags);
            mdelay(10);
        }
        sdhci_runtime_resume_host(host);
    }
    return 0;
}

static int sprd_mmc_host_runtime_idle(struct device *dev) {
    return 0;
}
#endif

static int sdhci_pm_suspend(struct device *dev) {
    int retval = 0;
    struct platform_device *pdev = container_of(dev, struct platform_device, dev);
    struct sdhci_host *host = platform_get_drvdata(pdev);
    struct mmc_host *mmc = host->mmc;
#ifdef CONFIG_PM_RUNTIME
    if(pm_runtime_enabled(dev))
        retval = pm_runtime_get_sync(dev);
#endif
    if(retval >= 0) {
            retval = sdhci_suspend_host(host, PMSG_SUSPEND);
            if(!retval) {
                unsigned long flags;
#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
                if( !strcmp(host->hw_name, "Spread SDIO host1") ) {
                    sdhci_host_wakeup_set(host);
                }
#endif
                spin_lock_irqsave(&host->lock, flags);
                if(host->ops->set_clock)
                    host->ops->set_clock(host, 0);
                spin_unlock_irqrestore(&host->lock, flags);
            } else {
#ifdef CONFIG_PM_RUNTIME
                if(pm_runtime_enabled(dev))
                    pm_runtime_put_autosuspend(dev);
#endif
            }
    }
    return retval;
}

static int sdhci_pm_resume(struct device *dev) {
    int retval = 0;
    unsigned long flags;
    struct platform_device *pdev = container_of(dev, struct platform_device, dev);
    struct sdhci_host *host = platform_get_drvdata(pdev);
    struct mmc_host *mmc = host->mmc;
    spin_lock_irqsave(&host->lock, flags);
    if(host->ops->set_clock)
        host->ops->set_clock(host, 1);
    spin_unlock_irqrestore(&host->lock, flags);
#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
        if(!strcmp(host->hw_name, "Spread SDIO host1")) {
            sdhci_host_wakeup_clear(host);
        }
#endif
    retval = sdhci_resume_host(host);
#ifdef CONFIG_PM_RUNTIME
    if(pm_runtime_enabled(dev))
        pm_runtime_put_autosuspend(dev);
#endif
    return retval;
}

static const struct dev_pm_ops sdhci_sprd_dev_pm_ops = {
	.suspend	 	= sdhci_pm_suspend,
	.resume		= sdhci_pm_resume,
	SET_RUNTIME_PM_OPS(sprd_mmc_host_runtime_suspend, sprd_mmc_host_runtime_resume, sprd_mmc_host_runtime_idle)
};

#define SDHCI_SPRD_DEV_PM_OPS_PTR	(&sdhci_sprd_dev_pm_ops)
#else
#define SDHCI_SPRD_DEV_PM_OPS_PTR	NULL
#endif

static struct platform_driver sdhci_sprd_driver = {
	.probe		= sdhci_sprd_probe,
	.driver		= {
		.owner	= THIS_MODULE,
		.pm 		=  SDHCI_SPRD_DEV_PM_OPS_PTR,
		.name	= DRIVER_NAME,
	},
};

static int __init sdhci_sprd_init(void)
{
	return platform_driver_register(&sdhci_sprd_driver);
}

static void __exit sdhci_sprd_exit(void)
{
	platform_driver_unregister(&sdhci_sprd_driver);
}

module_init(sdhci_sprd_init);
module_exit(sdhci_sprd_exit);

MODULE_DESCRIPTION("Spredtrum SDHCI glue");
MODULE_AUTHOR("spreadtrum.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sprd-sdhci");
