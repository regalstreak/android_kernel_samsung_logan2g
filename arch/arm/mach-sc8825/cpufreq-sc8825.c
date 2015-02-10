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

#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/bitops.h>
#include <mach/hardware.h>
#include <mach/regulator.h>
#include <mach/regs_ana_glb.h>
#include <mach/regs_ahb.h>
#include <mach/regs_glb.h>
#include <mach/adi.h>
#include <mach/sci.h>
#include <linux/earlysuspend.h>

//#define CONFIG_CPU_FREQ_MPLL
#define CONFIG_CPU_FREQ_EARLY_SUSPEND

//#define CONFIG_CPU_FREQ_DEBUG

#define MCU_CLK_PARENT_MASK 	(BIT(24)|BIT(23))
#define MCU_CLK_PARENT_26M  	(BIT(24)|BIT(23))
#define MCU_CLK_PARENT_256M 	(BIT(24))
#define MCU_CLK_PARENT_384M 	(BIT(23))
#define MCU_CLK_PARENT_MPLL 	(0)
#define MHz                     (1000000)
#define GR_MPLL_REFIN_2M        (2*MHz)
#define GR_MPLL_REFIN_4M        (4*MHz)
#define GR_MPLL_REFIN_13M       (13*MHz)
#define GR_MPLL_REFIN_SHIFT     (16)
#define GR_MPLL_REFIN_MASK      (0x3)
#define GR_MPLL_N_MASK          (0x7ff)
#define GR_MPLL_MN		(REG_GLB_M_PLL_CTL0)
#define GR_GEN1			(REG_GLB_GEN1)

#define DCDCARM_CTL_MASK	(BIT(2)|BIT(1)|BIT(0))
#define DCDCARM_CTL_RST_MASK	(BIT(6)|BIT(5)|BIT(4))
#define MCU_VDD_1V1		(BIT(6)|BIT(5)|BIT(4))
#define MCU_VDD_700m		(BIT(6)|BIT(5)|BIT(0))
#define MCU_VDD_800m		(BIT(6)|BIT(4)|BIT(1))
#define MCU_VDD_900m		(BIT(6)|BIT(1)|BIT(0))
#define MCU_VDD_1V0		(BIT(5)|BIT(4)|BIT(2))
#define MCU_VDD_650m		(BIT(5)|BIT(2)|BIT(0))
#define MCU_VDD_1V2		(BIT(4)|BIT(2)|BIT(1))
#define MCU_VDD_1V3		(BIT(2)|BIT(1)|BIT(0))
#define VDDARM_DEF		(1300)
#define VDDARM_CAL_MASK 	(0x1f)
#define VDDARM_CAL_STEPS 	(3)

#define WAIT_US			200
#define DELTA 			msecs_to_jiffies(500)
#define FREQ_TABLE_ENTRY	(3)
#define DELAY_TIME		(40*HZ)

DECLARE_PER_CPU(struct cpufreq_policy *, cpufreq_cpu_data);

static DEFINE_SPINLOCK(cpufreq_spinlock);
static unsigned long jiffies_delay;
static int sprd_cpufreq_set_rate(struct cpufreq_policy *policy, int index);
static unsigned int get_mcu_clk_freq(u32 cpu);

/*
 *   Cpu freqency is not be scaled yet, because of reasons of stablily.
 *   But we still define CONFIG_CPU_FREQ for some APKs, they will
 *display BogoMIPS instead of the real cpu frequency if CONFIG_CPU_FREQ
 *is not be defined
 */
#if defined(CONFIG_CPU_FREQ_EARLY_SUSPEND)
static int cpufreq_bypass = 1;
#else
static int cpufreq_bypass = 0;
#endif
static int sysfs_cpufreq_bypass = 0;
static int sysfs_cpufreq_max = 0;

/*
 *  sc8825+ indicator
 */
static int sc8825_plus = 0;

struct sprd_dvfs_table {
	unsigned long  clk_mcu_mhz;
	unsigned long  vdd_mcu_mv;
};

static struct sprd_dvfs_table sc8825g_dvfs_table[] = {
	[0] = { 1000000 , 1200000 }, /* 1000,000KHz,  1200mv */
	[1] = {  500000 , 1100000 }, /* 500,000KHz,  1100mv */
};

static struct sprd_dvfs_table sc8825g_plus_dvfs_table[] = {
	[0] = { 1200000 , 1300000 }, /* 1200,000KHz,  1300mv */
	[1] = {  600000 , 1100000 }, /* 600,000KHz,  1100mv */
};

static struct cpufreq_frequency_table sc8825g_freq_table[FREQ_TABLE_ENTRY];

enum scalable_cpus {
	CPU0 = 0,
	CPU1 = 1,
};

struct scalable {
	int 				cpu;
	struct clk			*clk;
	struct regulator		*vdd;
	struct cpufreq_frequency_table	*freq_tbl;
	struct sprd_dvfs_table		*dvfs_tbl;
};

struct scalable scalable_sc8825[] = {
	[CPU0] = {
		.cpu		= CPU0,
		.freq_tbl	= sc8825g_freq_table,
		.dvfs_tbl	= sc8825g_dvfs_table,
	},
	[CPU1] = {
		.cpu            = CPU1,
		.freq_tbl       = sc8825g_freq_table,
		.dvfs_tbl       = sc8825g_dvfs_table,
	},

};

struct sprd_dvfs_table current_cfg = {
	.clk_mcu_mhz = 0,
	.vdd_mcu_mv = 0,
};
#if 0
static unsigned int last_time[NR_CPUS];
#endif

struct clock_state {
	struct sprd_dvfs_table  current_para;
	struct mutex			lock;
}drv_state;

#ifdef CONFIG_SMP
struct cpufreq_work_struct {
	struct work_struct work;
	struct cpufreq_policy *policy;
	struct completion complete;
	unsigned int index;
	int frequency;
	int status;
};

static DEFINE_PER_CPU(struct cpufreq_work_struct, cpufreq_work);
static struct workqueue_struct *sprd_cpufreq_wq;
#endif

struct cpufreq_suspend_t {
	struct mutex suspend_mutex;
	int device_suspended;
};

static DEFINE_PER_CPU(struct cpufreq_suspend_t, cpufreq_suspend);


#ifdef CONFIG_SMP
static void set_cpu_work(struct work_struct *work)
{
	struct cpufreq_work_struct *cpu_work =
		container_of(work, struct cpufreq_work_struct, work);

	cpu_work->status  = sprd_cpufreq_set_rate(cpu_work->policy, cpu_work->index);
	complete(&cpu_work->complete);
}
#endif

/*@return: Hz*/
static unsigned long cpu_clk_get_rate(int cpu){
	struct clk *mcu_clk = NULL;
	unsigned long clk_rate = 0;

	mcu_clk = scalable_sc8825[cpu].clk;
	clk_rate = get_mcu_clk_freq(cpu);

	if(clk_rate < 0){
		pr_err("!!!%s cpu%u frequency is %lu\n", __func__, cpu, clk_rate);
	}
	return clk_rate;
}

static unsigned long get_max_dvfs_vdd(void)
{
	return scalable_sc8825[0].dvfs_tbl[0].vdd_mcu_mv/1000;
}

#if 0
static int set_mcu_vdd(int cpu, unsigned long vdd_mcu_uv){

	int ret = 0;
	struct regulator *vdd = scalable_sc8825[cpu].vdd;
	if(vdd)
		ret = regulator_set_voltage(vdd, vdd_mcu_uv, vdd_mcu_uv);
	else
		pr_err("!!!! %s,  no vdd !!!!\n", __func__);
	if(ret){
		pr_err("!!!! %s, set voltage error:%d !!!!\n", __func__, ret);
		return ret;
	}
    return ret;

}
#endif


/*
 * step = (100/32)mv
 */
static void set_dcdcarm_cal(unsigned long cal_step)
{
	u32 vddarm_cal;
	vddarm_cal = cal_step & VDDARM_CAL_MASK;
	sci_adi_write(ANA_REG_GLB_DCDCARM_CTRL_CAL, vddarm_cal, 0xffff);
}

static void set_dcdcarm(unsigned long vddarm)
{
	u32 val;
	val = sci_adi_read(ANA_REG_GLB_DCDCARM_CTRL0);
#ifdef CONFIG_CPU_FREQ_DEBUG
	printk("***** before, ANA_REG_GLB_DCDCARM_CTRL0:0x%x ****\n", val );
#endif
	val &= ~DCDCARM_CTL_MASK;
	val &= ~DCDCARM_CTL_RST_MASK;

	switch(vddarm){
	case 1100:
		val |= MCU_VDD_1V1;
		break;
	case 700:
		val |= MCU_VDD_700m;
		break;
	case 800:
		val |= MCU_VDD_800m;
		break;
	case 900:
		val |= MCU_VDD_900m;
		break;
	case 1000:
		val |= MCU_VDD_1V0;
		break;
	case 650:
		val |= MCU_VDD_650m;
		break;
	case 1200:
		val |= MCU_VDD_1V2;
		break;
	case 1300:
		val |= MCU_VDD_1V3;
		break;
	default:
		pr_err("!!! no such voltage:%lu supported !!!\n", vddarm);
	}
	sci_adi_write(ANA_REG_GLB_DCDCARM_CTRL0, val, 0xffff);
}

static int set_mcu_vdd(int cpu, unsigned int vdd_mcu_uv)
{
	u32 vdd_mcu_mv;
	vdd_mcu_mv = vdd_mcu_uv / 1000;
	pr_debug("***** cpu:%d, vdd_mcu_uv:%u, vdd_mcu_mv:%u****\n", cpu, vdd_mcu_uv, vdd_mcu_mv);
	switch(cpu){
	case CPU0:
	case CPU1:
		break;
	default:
		pr_err("!!! no more cups, cpu:%d !!!\n", cpu);
		return -1;
	}

	if(vdd_mcu_mv >= get_max_dvfs_vdd()){
		set_dcdcarm(vdd_mcu_mv);
		set_dcdcarm_cal(0);
	}else if(vdd_mcu_mv < get_max_dvfs_vdd()){
		set_dcdcarm_cal(VDDARM_CAL_STEPS);
		set_dcdcarm(vdd_mcu_mv);
	}

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk("**** after, cpu:%d, ANA_REG_GLB_DCDCARM_CTRL0:0x%x ****\n",
			cpu, sci_adi_read(ANA_REG_GLB_DCDCARM_CTRL0) );
	printk("**** after, cpu:%d, ANA_REG_GLB_DCDCARM_CTRL_CAL:0x%x ****\n",
			cpu, sci_adi_read(ANA_REG_GLB_DCDCARM_CTRL_CAL) );
#endif
	return 0;
}


#ifdef CONFIG_CPU_FREQ_MPLL
static void set_mcu_clk_parent(u32 cpu, u32 parent)
{
	u32 val;

	switch(cpu){
	case CPU0:
	case CPU1:
		val = sci_glb_read(REG_AHB_ARM_CLK, -1UL);
		val &= ~MCU_CLK_PARENT_MASK;
		break;
	default :
		pr_err("!!! no more cups !!!\n");
	}

	switch (parent) {
	case MCU_CLK_PARENT_26M:
		val |= MCU_CLK_PARENT_26M;
		break;
	case MCU_CLK_PARENT_256M:
		val |= MCU_CLK_PARENT_256M;
		break;
	case MCU_CLK_PARENT_384M:
		val |= MCU_CLK_PARENT_384M;
		break;
	case MCU_CLK_PARENT_MPLL:
		val |= MCU_CLK_PARENT_MPLL;
		break;
	default:
		pr_err("!!! set mcu clk parent error, no such parent !!!\n");
	}
	sci_glb_write(REG_AHB_ARM_CLK, val, -1UL);
}

static void sc8825_wait_mpll(void)
{
	udelay(WAIT_US);
}
#endif

static void set_mcu_clk_freq_div(u32 cpu, u32 mcu_freq)
{
	u32 val, arm_clk_div, gr_gen1, freq_mpll;
	pr_debug("***** %s, mcu_freq:%d ******\n", __func__, mcu_freq);
	freq_mpll = get_mcu_clk_freq(0);
	arm_clk_div = freq_mpll / mcu_freq;
	arm_clk_div -= 1;
#ifdef CONFIG_CPU_FREQ_DEBUG
	printk("before, AHB_ARM_CLK:%08x \n", __raw_readl(REG_AHB_ARM_CLK) );
#endif

	gr_gen1 =  __raw_readl(GR_GEN1);
	gr_gen1 |= BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);

	val = __raw_readl(REG_AHB_ARM_CLK);
	val &= 0xfffffff8;
	val |= arm_clk_div;
	__raw_writel(val, REG_AHB_ARM_CLK);

	gr_gen1 &= ~BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk("after, AHB_ARM_CLK:%08x, val:%x, div = %d\n",
			__raw_readl(REG_AHB_ARM_CLK), val, arm_clk_div);
#endif
}

static void set_mcu_clk_freq(u32 cpu, u32 mcu_freq)
{
	pr_debug("***** %s, mcu_freq:%d ******\n", __func__, mcu_freq);
#ifdef CONFIG_CPU_FREQ_MPLL
	/* change MPLL */
	u32 mpll_refin, mpll_n, mpll_cfg, rate;
	rate = mcu_freq / MHz;
	mpll_cfg = __raw_readl(GR_MPLL_MN);
	mpll_refin = (mpll_cfg>>GR_MPLL_REFIN_SHIFT) & GR_MPLL_REFIN_MASK;
	switch(mpll_refin){
		case 0:
			mpll_refin = GR_MPLL_REFIN_2M;
			break;
		case 1:
		case 2:
			mpll_refin = GR_MPLL_REFIN_4M;
			break;
		case 3:
			mpll_refin = GR_MPLL_REFIN_13M;
			break;
		default:
			printk("%s ERROR mpll_refin:%d\n", __func__, mpll_refin);
	}
	mpll_refin /= MHz;
	mpll_n = rate / mpll_refin;
	mpll_cfg &= ~GR_MPLL_N_MASK;
	mpll_cfg |= mpll_n;

	u32 gr_gen1 = __raw_readl(GR_GEN1);
	gr_gen1 |= BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);
	printk("before, mpll_cfg:0x%x, mpll_n:%u, mpll_refin:%u\n", __raw_readl(GR_MPLL_MN), mpll_n, mpll_refin);
	__raw_writel(mpll_cfg, GR_MPLL_MN);
	gr_gen1 &= ~BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);

	sc8825_wait_mpll();
	printk("after, mpll_cfg:0x%x, mpll_n:%u, mpll_refin:%u\n", __raw_readl(GR_MPLL_MN), mpll_n, mpll_refin);
#else
	set_mcu_clk_freq_div(cpu, mcu_freq);
#endif
}

static unsigned int get_mcu_clk_freq(u32 cpu)
{
	u32 mpll_refin, mpll_n, mpll_cfg, rate;
	switch(cpu){
	case CPU0:
	case CPU1:
		mpll_cfg = __raw_readl(GR_MPLL_MN);
		break;
	default :
		pr_err("!!! no more cups !!!\n");
		return -1;
	}


	mpll_refin = (mpll_cfg>>GR_MPLL_REFIN_SHIFT) & GR_MPLL_REFIN_MASK;
	switch(mpll_refin){
		case 0:
			mpll_refin = GR_MPLL_REFIN_2M;
			break;
		case 1:
		case 2:
			mpll_refin = GR_MPLL_REFIN_4M;
			break;
		case 3:
			mpll_refin = GR_MPLL_REFIN_13M;
			break;
		default:
			pr_err("%s ERROR mpll_refin:%d\n", __func__, mpll_refin);
	}
	mpll_n = mpll_cfg & GR_MPLL_N_MASK;
	rate = mpll_refin * mpll_n;
	return rate;
}

static int set_mcu_freq(int cpu, unsigned long mcu_freq_khz){
	int ret = 0;
	unsigned long freq_mcu_hz = mcu_freq_khz * 1000;
#if 0
	struct clk *clk = scalable_sc8825[cpu].clk;
	set_mcu_clk_parent(cpu, MCU_CLK_PARENT_384M);
	ret = clk_set_rate(clk, freq_mcu_hz);
	set_mcu_clk_parent(cpu, MCU_CLK_PARENT_MPLL);
	if(ret){
		pr_err("!!!! %s, clk_set_rate:%d !!!!\n", __func__, ret);
	}
#endif
	set_mcu_clk_freq(cpu, freq_mcu_hz);
	return ret;
}

static int cpu_set_freq_vdd(struct cpufreq_policy *policy, unsigned long mcu_clk,
				unsigned long mcu_vdd)
{
	unsigned int ret;
	unsigned long cpu = policy->cpu;
	unsigned long cur_clk = current_cfg.clk_mcu_mhz;
	unsigned int i =0;
	struct cpufreq_policy *policy_update;

	if(mcu_clk > cur_clk){
		ret = set_mcu_vdd(cpu, mcu_vdd);
		mdelay(1);
		ret = set_mcu_freq(cpu, mcu_clk);
	}else if(mcu_clk < cur_clk){
		ret = set_mcu_freq(cpu, mcu_clk);
		mdelay(1);
		ret = set_mcu_vdd(cpu, mcu_vdd);
	}else
	{
		return -1;
	}

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk("****DVFS**** cpu_set_freq_vdd cpu=%lu,old=%lu,new=%lu\n",cpu,current_cfg.clk_mcu_mhz,mcu_clk);
#endif

	current_cfg.clk_mcu_mhz = mcu_clk;
	current_cfg.vdd_mcu_mv  = mcu_vdd;

	for_each_possible_cpu(i) {
		policy_update = per_cpu(cpufreq_cpu_data, i);
		policy_update->cur=current_cfg.clk_mcu_mhz;
		per_cpu(cpufreq_cpu_data, i) = policy_update;
	}

	pr_debug("*** %s, cpu:%lu, cur_clk:%lu, ", __func__, cpu, cur_clk );
	pr_debug("current_cfg.clk_mcu_mhz:%lu, current_cfg.vdd_mcu_mv:%lu ****\n",
				 current_cfg.clk_mcu_mhz, current_cfg.vdd_mcu_mv);
	return ret;
}

static int cpu0_target_freq, cpu0_target_vdd;
static int cpu1_target_freq, cpu1_target_vdd;

extern int memfreq_usb_state;

static int sprd_cpufreq_set_rate(struct cpufreq_policy *policy, int index)
{
	int ret = 0;
	/*   enable scaling after the whole system boot completion
	 */
	unsigned long flags;

#if defined(CONFIG_CPU_FREQ_DEBUG)
	printk("****DVFS**** sprd_cpufreq_set_rate cpufreq_bypass=%d,sysfs_cpufreq_bypass=%d\n",
		cpufreq_bypass,sysfs_cpufreq_bypass);
#endif

	if(time_before(jiffies, jiffies_delay) || cpufreq_bypass){
		return ret;
	}

	/*   fixed cpu frequency in some case, like mp4,mp3
	 */
	if(sysfs_cpufreq_bypass){
		return ret;
	}

	/* when connecting with USB, keep max cpufreq */
	if(memfreq_usb_state == 1)
		index = 0;

	struct cpufreq_freqs freqs;
	struct sprd_dvfs_table *dvfs_tbl = scalable_sc8825[policy->cpu].dvfs_tbl;
	unsigned int new_freq = dvfs_tbl[index].clk_mcu_mhz;
	unsigned int new_vdd = dvfs_tbl[index].vdd_mcu_mv;

	if(new_freq == current_cfg.clk_mcu_mhz){
		return 0;
	}

	/*
	*  2 cores are always in the same voltage, at the same frequency. But,
	* cpu load is calculated individual in each cores, So we remeber the
	* original target frequency and voltage of core0, and use the higher one
	*/
	spin_lock_irqsave(&cpufreq_spinlock, flags);
	if(policy->cpu == 0){
		cpu0_target_freq = new_freq;
		cpu0_target_vdd = new_vdd;
	}else if(policy->cpu == 1){
		cpu1_target_freq = new_freq;
		cpu1_target_vdd = new_vdd;
	}
	pr_debug("*** cpu0_target_freq:%u, cpu1_target_freq:%u ***\n", cpu0_target_freq, cpu1_target_freq);
	new_freq = cpu1_target_freq > cpu0_target_freq ? cpu1_target_freq : cpu0_target_freq;
	new_vdd  = cpu1_target_vdd > cpu0_target_vdd ? cpu1_target_vdd : cpu0_target_vdd;
	spin_unlock_irqrestore(&cpufreq_spinlock, flags);

	freqs.old = current_cfg.clk_mcu_mhz;
	freqs.new = new_freq;
	freqs.cpu = policy->cpu;

	if(freqs.new == freqs.old){
		return 0;
	}
#if 0
	if(time_before(jiffies, last_time[policy->cpu]+DELTA)    &&
		new_freq < current_cfg.clk_mcu_mhz  ){
		pr_info("%s, set rate in 1s(this_time:%u, last_time:%u), skip\n",
					__func__, jiffies, last_time[policy->cpu] );
		return ret;
	}
#endif
	pr_debug(" %s, old_freq:%lu KHz, old_vdd:%lu uv  \n", __func__,
			current_cfg.clk_mcu_mhz, current_cfg.vdd_mcu_mv);
	pr_debug("%s, new_freq:%u KHz, new_vdd:%u uv \n", __func__,
					new_freq, new_vdd);
#if 0
	/*
	* TODO: if the lowest frequency is below 400000KHz, disable core 1
	*/
	if(new_freq<=500000  && cpu_online(1))
		cpu_down(1);
	else
		cpu_up(1);
#endif
	policy->cur = current_cfg.clk_mcu_mhz;
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	spin_lock_irqsave(&cpufreq_spinlock, flags);
	ret = cpu_set_freq_vdd(policy, new_freq, new_vdd);
	spin_unlock_irqrestore(&cpufreq_spinlock, flags);
	if (!ret){
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#if 0
	if(policy->cur == policy->max){
		last_time[policy->cpu] = jiffies;
	}
#endif
	return ret;

}

static int sc8825_is_plus(int cpu){
	int cpu_clk = 0;
	int sc8825_is_plus = 0;

	cpu_clk = cpu_clk_get_rate(cpu);
	/*
	 * sc8825+ runs at 1.2GHz
	 */
	if(cpu_clk == 1200000000)
		sc8825_is_plus =  1;

	return sc8825_is_plus;
}

static int sc8825_cpufreq_table_init(int cpu){
	int cnt;

	scalable_sc8825[cpu].cpu = cpu;
	scalable_sc8825[cpu].clk = clk_get(NULL, "clk_mcu");
	scalable_sc8825[cpu].vdd = regulator_get(NULL, "vddarm");
	if( scalable_sc8825[cpu].clk == NULL ||
		scalable_sc8825[cpu].vdd == NULL ){
		printk("%s, cpu:%d, clk:%p, vdd:%p\n", __func__, cpu,
				scalable_sc8825[cpu].clk, scalable_sc8825[cpu].vdd);
		return -1;
	}

	if (sc8825g_freq_table == NULL) {
		printk(" cpufreq: No frequency information for this CPU\n");
		return -1;
	}

	sc8825_plus = sc8825_is_plus(cpu);
	for (cnt = 0; cnt < FREQ_TABLE_ENTRY-1; cnt++) {
		sc8825g_freq_table[cnt].index = cnt;
		if(!sc8825_plus)
			sc8825g_freq_table[cnt].frequency = sc8825g_dvfs_table[cnt].clk_mcu_mhz;
		else
			sc8825g_freq_table[cnt].frequency = sc8825g_plus_dvfs_table[cnt].clk_mcu_mhz;
	}
	sc8825g_freq_table[cnt].index = cnt;
	sc8825g_freq_table[cnt].frequency = CPUFREQ_TABLE_END;

	scalable_sc8825[cpu].freq_tbl = sc8825g_freq_table;
	if(sc8825_plus)
		scalable_sc8825[cpu].dvfs_tbl = sc8825g_plus_dvfs_table;


	for (cnt = 0; cnt < FREQ_TABLE_ENTRY; cnt++) {
		pr_debug("%s, scalable_sc8825[cpu].freq_tbl[%d].index:%d\n", __func__, cnt,
				scalable_sc8825[cpu].freq_tbl[cnt].index);
		pr_debug("%s, scalable_sc8825[cpu].freq_tbl[%d].frequency:%d\n", __func__, cnt,
				scalable_sc8825[cpu].freq_tbl[cnt].frequency);
	}
	return 0;
}

static int sprd_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, scalable_sc8825[policy->cpu].freq_tbl);
}

/*@return: KHz*/
static unsigned int sprd_cpufreq_get_speed(unsigned int cpu)
{
#if 0
#ifdef CONFIG_CPU_FREQ_MPLL
	return cpu_clk_get_rate(cpu) / 1000;
#else
	if(current_cfg.clk_mcu_mhz!=0){
		return current_cfg.clk_mcu_mhz/1000;
	}
	return scalable_sc8825[0].dvfs_tbl[0].clk_mcu_mhz/1000;
#endif
#else
	return current_cfg.clk_mcu_mhz;
#endif
}

static unsigned int sprd_cpufreq_driver_getavg(struct cpufreq_policy *policy, unsigned int cpu)
{
	return current_cfg.clk_mcu_mhz;
}

static int sprd_cpufreq_set_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	pr_debug("***** %s, cpu:%d, target_freq:%d, relation:%d ***\n",
				__func__, policy->cpu, target_freq, relation);
	int ret;
	int index;
	struct cpufreq_frequency_table *table;
#ifdef CONFIG_SMP
	struct cpufreq_work_struct *cpu_work = NULL;
	cpumask_var_t mask;

	if (!alloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	ret = -EFAULT;
	if (!cpu_active(policy->cpu)) {
		pr_info("cpufreq: cpu %d is not active.\n", policy->cpu);
		return -ENODEV;
	}
#endif
	mutex_lock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);

	if (per_cpu(cpufreq_suspend, policy->cpu).device_suspended) {
		printk("cpufreq: cpu%d scheduling frequency change "
				"in suspend.\n", policy->cpu);
		ret = -EFAULT;
		goto done;
	}

	table = cpufreq_frequency_get_table(policy->cpu);

	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
				&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto done;
	}

	pr_debug("CPU[%d] target %d relation %d (%d-%d) selected %d\n",
			policy->cpu, target_freq, relation,
			policy->min, policy->max, table[index].frequency);

#ifdef CONFIG_SMP
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	cpu_work->policy = policy;
	cpu_work->frequency = table[index].frequency;
	cpu_work->status = -ENODEV;
	cpu_work->index = index;

	cpumask_clear(mask);
	cpumask_set_cpu(policy->cpu, mask);
	if (cpumask_equal(mask, &current->cpus_allowed)) {
		ret = sprd_cpufreq_set_rate(cpu_work->policy, cpu_work->index);
		goto done;
	} else {
		cancel_work_sync(&cpu_work->work);
		INIT_COMPLETION(cpu_work->complete);
		queue_work_on(policy->cpu, sprd_cpufreq_wq, &cpu_work->work);
		wait_for_completion(&cpu_work->complete);
	}

	free_cpumask_var(mask);
	ret = cpu_work->status;
#else
	ret = sprd_cpufreq_set_rate(policy, index);
#endif

done:

	mutex_unlock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);
	return ret;

}

static int sprd_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;
#ifdef CONFIG_SMP
	struct cpufreq_work_struct *cpu_work = NULL;
#endif

	ret = sc8825_cpufreq_table_init(policy->cpu);
	if(ret){
		return -ENODEV;
	}

	policy->cur = cpu_clk_get_rate(policy->cpu) / 1000; /* current cpu frequency : KHz*/
	policy->cpuinfo.transition_latency = 1 * 1000 * 1000;//why this value??
	current_cfg.clk_mcu_mhz = policy->cur;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(scalable_sc8825[policy->cpu].freq_tbl, policy->cpu);
#endif

	ret = cpufreq_frequency_table_cpuinfo(policy, scalable_sc8825[policy->cpu].freq_tbl);
	if (ret != 0) {
		pr_err("cpufreq: Failed to configure frequency table: %d\n", ret);
	}

#ifdef CONFIG_SMP
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	INIT_WORK(&cpu_work->work, set_cpu_work);
	init_completion(&cpu_work->complete);
#endif

	return ret;
}

static struct freq_attr *sprd_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver sprd_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags      	= CPUFREQ_STICKY,
	.verify		= sprd_cpufreq_verify_speed,
	.target		= sprd_cpufreq_set_target,
	.get		= sprd_cpufreq_get_speed,
	.init		= sprd_cpufreq_driver_init,
	.getavg     = sprd_cpufreq_driver_getavg,
	.name		= "sprd_cpufreq",
	.attr		= sprd_cpufreq_attr,
};

static int sprd_cpufreq_suspend(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		mutex_lock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		per_cpu(cpufreq_suspend, cpu).device_suspended = 1;
		mutex_unlock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
	}

	return NOTIFY_DONE;
}

static int sprd_cpufreq_resume(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

	return NOTIFY_DONE;
}

static int sprd_cpufreq_pm_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	switch (event) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		return sprd_cpufreq_resume();
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		return sprd_cpufreq_suspend();
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block sprd_cpufreq_pm_notifier = {
	.notifier_call = sprd_cpufreq_pm_event,
};

static int __init sprd_cpufreq_register(void){
	int cpu;
	jiffies_delay = jiffies + DELAY_TIME;

	for_each_possible_cpu(cpu) {
		mutex_init(&(per_cpu(cpufreq_suspend, cpu).suspend_mutex));
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

#ifdef CONFIG_SMP
	sprd_cpufreq_wq = create_workqueue("sprd-cpufreq");
#endif

	register_pm_notifier(&sprd_cpufreq_pm_notifier);
	return cpufreq_register_driver(&sprd_cpufreq_driver);
}

late_initcall(sprd_cpufreq_register);

/*cpufreq sysfs */
static DEFINE_MUTEX(cpufreq_mutex);
static unsigned int cpufreq_choose = 0;
struct kobject *cpufreq_kobj;
unsigned int cpufreq_store_freq_old;

#define cpufreq_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0620,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

static int cpufreq_set_freq_vdd(unsigned long mcu_clk,
				unsigned long mcu_vdd)
{
	int ret;
	int i;
	struct cpufreq_policy *policy_update;
	unsigned long cur_clk = current_cfg.clk_mcu_mhz;

	if(mcu_clk == cur_clk){
		ret = 1;
		pr_debug("return 1 \n");
		return ret;
	}

	/*Only cpu0*/
	if(mcu_clk > cur_clk){
		ret = set_mcu_vdd(0, mcu_vdd);
		mdelay(1);
		ret = set_mcu_freq(0, mcu_clk);
	}else if(mcu_clk < cur_clk){
		ret = set_mcu_freq(0, mcu_clk);
		mdelay(1);
		ret = set_mcu_vdd(0, mcu_vdd);
	}
	
	printk("****DVFS**** cpufreq_set_freq_vdd cpu=0,old=%lu,new=%lu\n",current_cfg.clk_mcu_mhz,mcu_clk);

	current_cfg.clk_mcu_mhz = mcu_clk;
	current_cfg.vdd_mcu_mv  = mcu_vdd;
	
	for_each_possible_cpu(i){
		policy_update = per_cpu(cpufreq_cpu_data, i);
		policy_update->cur=current_cfg.clk_mcu_mhz;
		per_cpu(cpufreq_cpu_data, i) = policy_update;
	}

	return ret;
}

static ssize_t cpufreq_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	/*Only cpu0*/
	struct sprd_dvfs_table *dvfs_tbl = scalable_sc8825[0].dvfs_tbl;
	unsigned long cpu_freq ;
	unsigned long cpu_vdd ;

	mutex_lock(&cpufreq_mutex);
	cpu_freq = dvfs_tbl[cpufreq_choose].clk_mcu_mhz/1000;
	cpu_vdd = dvfs_tbl[cpufreq_choose].vdd_mcu_mv/1000;
	mutex_unlock(&cpufreq_mutex);

    if(cpu_freq==current_cfg.clk_mcu_mhz/1000)
		return sprintf(buf,"%lumhz,%lumv\n",cpu_freq,cpu_vdd);
	else
		return sprintf(buf,"%lumhz,%lumv\n",current_cfg.clk_mcu_mhz/1000,current_cfg.vdd_mcu_mv/1000);
}

static ssize_t cpufreq_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	/*Only cpu0*/
	struct sprd_dvfs_table *dvfs_tbl = scalable_sc8825[0].dvfs_tbl;
	unsigned int ret;
	struct cpufreq_freqs freqs;

	freqs.old = current_cfg.clk_mcu_mhz;
	freqs.cpu = 0;
	cpufreq_store_freq_old = freqs.old;

	switch(buf[0]) {
	case '0':
		printk("****DVFS**** cpufreq_store 0\n");
		sysfs_cpufreq_bypass = 0;
		cpufreq_choose = 0;
		sysfs_cpufreq_max = 1;
		freqs.new = dvfs_tbl[cpufreq_choose].clk_mcu_mhz;
		if(freqs.new==freqs.old){
			return 1;
		}
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		mutex_lock(&cpufreq_mutex);
		ret = cpufreq_set_freq_vdd(freqs.new,dvfs_tbl[cpufreq_choose].vdd_mcu_mv);
		mutex_unlock(&cpufreq_mutex);
		if(ret>0){
			pr_debug("cpufreq not need to set 0\n");
		}else{
			freqs.old = cpufreq_store_freq_old;
			cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
		}
		break;
	case '1' :
		printk("****DVFS**** cpufreq_store 1\n");
		sysfs_cpufreq_bypass = 1;
		sysfs_cpufreq_max = 0;
		cpufreq_choose = 1;
		freqs.new = dvfs_tbl[cpufreq_choose].clk_mcu_mhz;
		if(freqs.new==freqs.old){
			return 1;
		}
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		mutex_lock(&cpufreq_mutex);
		ret = cpufreq_set_freq_vdd(freqs.new,dvfs_tbl[cpufreq_choose].vdd_mcu_mv);
		mutex_unlock(&cpufreq_mutex);
		if(ret>0){
			pr_debug("cpufreq not need to set 1\n");
		}else{
			freqs.old = cpufreq_store_freq_old;
			cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
		}
		break;
	default:
		break;
	}
	return 1;
}

static ssize_t cpufreq_bypass_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
    if(cpufreq_bypass)
		return sprintf(buf,"%d:means off\n",cpufreq_bypass);
	else
		return sprintf(buf,"%d:means on\n",cpufreq_bypass);
}


static ssize_t cpufreq_bypass_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	switch(buf[0]) {
	case '0':
		printk("****DVFS**** cpufreq_bypass_store 0\n");
		mutex_lock(&cpufreq_mutex);
		cpufreq_bypass = 0;
		mutex_unlock(&cpufreq_mutex);
		break;
	case '1' :
		printk("****DVFS**** cpufreq_bypass_store 1\n");
		mutex_lock(&cpufreq_mutex);
		cpufreq_bypass = 1;
		mutex_unlock(&cpufreq_mutex);
		break;
	default:
		break;
	}
	return 1;
}


cpufreq_attr(cpufreq);
cpufreq_attr(cpufreq_bypass);
static struct attribute * cpufreq_g[] = {
	&cpufreq_attr.attr,
#ifndef CONFIG_CPU_FREQ_EARLY_SUSPEND
	&cpufreq_bypass_attr.attr,
#endif
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = cpufreq_g,
};

static int __init cpufreq_sysfs_init(void)
{
	cpufreq_kobj = kobject_create_and_add("cpu", NULL);
	if (!cpufreq_kobj)
		return -ENOMEM;

	return sysfs_create_group(cpufreq_kobj, &attr_group);
}

static void  __exit cpufreq_sysfs_exit(void)
{
	sysfs_remove_group(cpufreq_kobj, &attr_group);
}

module_init(cpufreq_sysfs_init);
module_exit(cpufreq_sysfs_exit);


#ifdef CONFIG_CPU_FREQ_EARLY_SUSPEND
/*cpufreq earlysuspend*/
static void cpufreq_earlysuspend_early_suspend(struct early_suspend *h)
{
	printk("****DVFS**** cpufreq_earlysuspend_early_suspend cpufreq_bypass = 0\n");
	cpufreq_bypass = 0;
}

static void cpufreq_earlysuspend_late_resume(struct early_suspend *h)
{
	struct sprd_dvfs_table *dvfs_tbl = scalable_sc8825[0].dvfs_tbl;
	unsigned int ret;
	struct cpufreq_freqs freqs;
	unsigned int cpufreq_freq_old;

	printk("****DVFS**** cpufreq_earlysuspend_late_resume cpufreq_bypass = 1\n");
	cpufreq_bypass = 1;

	freqs.old = current_cfg.clk_mcu_mhz;
	freqs.cpu = 0;
	cpufreq_freq_old = freqs.old;

	freqs.new = dvfs_tbl[0].clk_mcu_mhz;
	if(freqs.new==freqs.old){
		return 1;
	}
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	mutex_lock(&cpufreq_mutex);
	ret = cpufreq_set_freq_vdd(freqs.new,dvfs_tbl[0].vdd_mcu_mv);
	mutex_unlock(&cpufreq_mutex);
	if(ret>0){
		pr_debug("earlysuspend cpufreq not need to set 0\n");
	}else{
		freqs.old = cpufreq_freq_old;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
}

static struct early_suspend cpufreq_early_suspend_desc = {
	.level = 0,
	.suspend = cpufreq_earlysuspend_early_suspend,
	.resume = cpufreq_earlysuspend_late_resume,
};

static int __init cpufreq_earlysuspend_init(void)
{
	register_early_suspend(&cpufreq_early_suspend_desc);
}

static void  __exit cpufreq_earlysuspend_exit(void)
{
	unregister_early_suspend(&cpufreq_early_suspend_desc);
}

module_init(cpufreq_earlysuspend_init);
module_exit(cpufreq_earlysuspend_exit);
#endif
