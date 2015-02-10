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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <mach/memfreq_ondemand.h>
#include <mach/hardware.h>
#include <mach/regs_ana_glb.h>
#include <mach/globalregs.h>
#include <mach/regs_glb.h>
#include <mach/regs_ahb.h>
#include <mach/sci.h>
#include <linux/earlysuspend.h>

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */
#ifdef DEBUG
#define memfreq_debug(f, a...)					\
	do {							\
		printk (KERN_DEBUG "(%s, %d): %s: ",		\
				__FILE__, __LINE__, __func__);	\
		printk (f, ## a);				\
	} while (0)
#else
#define memfreq_debug(f, a...)	/**/
#endif

//#define DELAY (5*HZ)
#define DELAY (msecs_to_jiffies(500))
#define FIRST_DELAY	(120*HZ)
#define CYCLE DELAY
#define MEMFREQ_200MHz (200)
#define MEMFREQ_300MHz (300)
#define MEMFREQ_400MHz (400)
#define MEMFREQ_MALI_MEM_LOAD_THRESHOLD (100)

static DEFINE_MUTEX(memfreq_dbs_lock);
static DEFINE_SPINLOCK(memfreq_lock);
static LIST_HEAD(memfreq_dbs_handlers);
struct delayed_work mem_work;
volatile unsigned int memfreq_curr;
volatile unsigned int memfreq_bypass;
bool memfreq_increase_cpu_frequency=0;

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct delayed_work work;
	int cpu;
};

/*TODO: od_cpu_dbs_info is not initialized */
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);
static inline void dbs_timer_init(void);
static inline void dbs_timer_exit(void);
extern u32 emc_freq_get(void);
extern void emc_freq_set(u32);

extern int memfreq_usb_state;

void register_memfreq_ondemand(struct memfreq_dbs *handler)
{
	struct list_head *pos;

	mutex_lock(&memfreq_dbs_lock);
	list_for_each(pos, &memfreq_dbs_handlers) {
		struct memfreq_dbs *e;
		e = list_entry(pos, struct memfreq_dbs, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	mutex_unlock(&memfreq_dbs_lock);
}
EXPORT_SYMBOL(register_memfreq_ondemand);

void unregister_memfreq_ondemand(struct memfreq_dbs *handler)
{
	mutex_lock(&memfreq_dbs_lock);
	list_del(&handler->link);
	mutex_unlock(&memfreq_dbs_lock);
}
EXPORT_SYMBOL(unregister_memfreq_ondemand);

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
			kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_wall_time, busy_time);
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/*
* get cpu load via idle time
*/
static unsigned int dbs_check_cpu(void)
{
	unsigned int j;
	unsigned int max_load;

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate, we look for a the lowest
	 * frequency which can sustain the load while keeping idle time over
	 * 30%. If such a frequency exist, we try to decrease to this frequency.
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of current frequency
	 */

	/* Get Absolute Load - in terms of freq */
	max_load = 0;

	for_each_online_cpu(j){
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		unsigned int load;
		int freq_avg;

		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
				j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
				j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int) cputime64_sub(cur_iowait_time,
				j_dbs_info->prev_cpu_iowait);
		j_dbs_info->prev_cpu_iowait = cur_iowait_time;

#if 0
		if (dbs_tuners_ins.ignore_nice) {
			cputime64_t cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = cputime64_sub(kstat_cpu(j).cpustat.nice,
					j_dbs_info->prev_cpu_nice);
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
				cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}
		/*
		 * For the purpose of ondemand, waiting for disk IO is an
		 * indication that you're performance critical, and not that
		 * the system is actually idle. So subtract the iowait time
		 * from the cpu idle time.
		 */
		if (dbs_tuners_ins.io_is_busy && idle_time >= iowait_time)
			idle_time -= iowait_time;
#endif
		if (unlikely(!wall_time || wall_time < idle_time)){
			memfreq_debug(" *** wall_time:%u, idle_time:%u, continue ****\n",
						wall_time, idle_time);
			continue;
		}

		load = 100 * (wall_time - idle_time) / wall_time;
		memfreq_debug(" cpu%u, wall_time:%u, idle_time:%u \n", j, wall_time, idle_time);
		memfreq_debug(" cpu%u, load:%u \n", j, load);
		max_load = max_load>load ? max_load : load;
	}

	memfreq_curr = emc_freq_get();
	memfreq_debug(" max_load:%u, memfreq_curr:%u \n", max_load, memfreq_curr);
/* block due to too many log */
#if 0
	printk(" %s: max_load:%u, memfreq_curr:%u \n", __func__, max_load, memfreq_curr);
#endif
	return ((max_load*memfreq_curr)/100);
}

static unsigned int dbs_check_mem(void)
{
	unsigned int mem_load;
	unsigned int mem_mst_cnt;
	unsigned int mem_dsp_cnt;
	mem_load = 0;
/*
* TODO: get mem load via bus monitors

#ifndef CONFIG_NKERNEL
	unsigned int mem_cp_cnt;
#endif
	mem_load = ((mem_mst_cnt+mem_dsp_cnt)/CYCLE);
#ifndef CONFIG_NKERNEL
	mem_load = ((mem_mst_cnt+mem_dsp_cnt+cp_cnt)/CYCLE);
#endif
	mem_load = (mem_load/memfreq_curr)*100;
	
	memfreq_debug("*** mem_mst_cnt:%u, mem_dsp_cnt:%u, mem_load:%u ***\n",
				mem_mst_cnt, mem_dsp_cnt, mem_load);
	return mem_load;
*/
	return 0;

}

static unsigned int dbs_ask_mem(void)
{
	struct memfreq_dbs *pos;
	unsigned int demand;

	demand = 0;
	mutex_lock(&memfreq_dbs_lock);
	list_for_each_entry(pos, &memfreq_dbs_handlers, link) {
		if (pos->memfreq_demand != NULL) {
			//printk("%s: calling %pf\n", __func__, pos->memfreq_demand);
			demand += pos->memfreq_demand(pos);
			//printk("%s: calling %pf done, demand:%u \n",
			//			__func__, pos->memfreq_demand, demand);
		}
	}	
	mutex_unlock(&memfreq_dbs_lock);
	return (demand >> 23);
	/*
	 * TODO: return demand;
	*/
}

static unsigned int memfreq_get_demand(void)
{
	unsigned int cpu_load;
	unsigned int mem_load;
	unsigned int memfreq_target;

	cpu_load = dbs_check_cpu( );
	mem_load = dbs_check_mem( );
	mem_load += dbs_ask_mem( );

//	printk("****DDRDFS**** memfreq_get_demand cpu_load=%u, mem_load=%u\n",cpu_load,mem_load);
	if (mem_load > MEMFREQ_MALI_MEM_LOAD_THRESHOLD) {
		memfreq_increase_cpu_frequency = 1;
	} else {
		memfreq_increase_cpu_frequency = 0;
	}

	memfreq_target = (cpu_load+mem_load);
	return memfreq_target;
}

static void memfreq_set_freq(unsigned int target)
{
	unsigned int memfreq_target;

	if(target < 150){
		/* set ddr 200MHz */
		memfreq_target = MEMFREQ_200MHz;
	}else if(target < 250){
		/* set ddr 300MHz */
		memfreq_target = MEMFREQ_300MHz;
	}else{
		/* set ddr 400MHz */
		memfreq_target = MEMFREQ_400MHz;
	}

	if(memfreq_target == MEMFREQ_300MHz)
		memfreq_target = MEMFREQ_400MHz;

	if(memfreq_usb_state == 1)
		memfreq_target = MEMFREQ_400MHz;

	if(memfreq_target == memfreq_curr)
		return;

	printk("****DDRDFS**** %s: memfreq_target:%u, ", __func__, memfreq_target);
	emc_freq_set(memfreq_target);
	memfreq_curr = emc_freq_get();
	memfreq_debug(" ** after set, mem freq: %u ** \n", memfreq_curr );
	printk(" set frequency done, mem freq: %u \n", memfreq_curr );
}

static unsigned int mm_is_on(void)
{
	unsigned int is_on;
	unsigned int val;

	is_on = 0;
	val = sci_glb_read(REG_AHB_AHB_CTL2, -1UL);
	is_on = val & AHB_CTL2_MMMTX_CLK_EN;
	/* TODO: what about dispc & lcdc */
	return is_on;
}

static void do_dbs_timer(struct work_struct *work)
{
	unsigned int mem_target;

	/*   mali, vsp, isp and dcam access memory via axi bus.
	 * and there is no bus monitor in channel5 (dispc, lcdc)
	 */
	memfreq_curr = emc_freq_get();
//	printk("****DDRDFS**** do_dbs_timer memfreq_bypass=%u,mm_is_on()=%u,memfreq_curr=%u\n",
//			memfreq_bypass,mm_is_on(),memfreq_curr);

	spin_lock(&memfreq_lock);
		mem_target = memfreq_get_demand();
	if(!memfreq_bypass){
		if(!mm_is_on()){
		memfreq_set_freq(mem_target);
	}
	}
	spin_unlock(&memfreq_lock);
	schedule_delayed_work_on(0, &mem_work, DELAY);
}

static inline void dbs_timer_init(void)
{
	INIT_DELAYED_WORK_DEFERRABLE(&mem_work, do_dbs_timer);
	schedule_delayed_work_on(0, &mem_work, FIRST_DELAY);
}

static inline void dbs_timer_exit(void)
{
	cancel_delayed_work_sync(&mem_work);
}

/*
*   NOTE: there is no profermance counter in axi bus monitors.
* so we only initialize ahb bus monitors
*/
static inline void bm_reset(void)
{
	/* masters */
	/* dsp */
#ifndef CONFIG_NKERNEL
	/* TODO: cp */
#endif
}

static unsigned int memfreq_get_freq(void)
{
	unsigned int curr;
	curr = emc_freq_get( );
	return curr;
}

static int memfreq_suspend(void)
{
	memfreq_debug("\n");
	//spin_lock(&memfreq_lock);
	//memfreq_bypass = 1;
	//memfreq_increase_cpu_frequency = 0;
	/* bm_stop(); */
	//spin_unlock(&memfreq_lock);
	return NOTIFY_DONE;
}

static int memfreq_resume(void)
{
	memfreq_debug("\n");
	//spin_lock(&memfreq_lock);
	/* bm_start(); */
	//memfreq_bypass = 0;
	//spin_unlock(&memfreq_lock);
	return NOTIFY_DONE;
}

static int memfreq_pm_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	switch (event) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		return memfreq_resume();
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		return memfreq_suspend();
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block memfreq_pm_notifier = {
	.notifier_call = memfreq_pm_event,
};

static void memfreq_earlysuspend_early_suspend(struct early_suspend *h)
{
	printk("****DDRDFS**** memfreq_earlysuspend_early_suspend \n");
	spin_lock(&memfreq_lock);
	memfreq_bypass = 1;
	spin_unlock(&memfreq_lock);
}

static void memfreq_earlysuspend_late_resume(struct early_suspend *h)
{
	printk("****DDRDFS**** memfreq_earlysuspend_late_resume \n");
	spin_lock(&memfreq_lock);
	memfreq_bypass = 0;
	spin_unlock(&memfreq_lock);
}

static struct early_suspend memfreq_early_suspend_desc = {
	.level = 0,
	.suspend = memfreq_earlysuspend_early_suspend,
	.resume = memfreq_earlysuspend_late_resume,
};

static int __init memfreq_gov_dbs_init(void)
{
	register_pm_notifier(&memfreq_pm_notifier);
	memfreq_curr = memfreq_get_freq( );
	memfreq_bypass = 0;
	bm_reset();
	dbs_timer_init( );
	register_early_suspend(&memfreq_early_suspend_desc);
	return 0;
}

static void __exit memfreq_gov_dbs_exit(void)
{
	dbs_timer_exit( );
	unregister_pm_notifier(&memfreq_pm_notifier);
	unregister_early_suspend(&memfreq_early_suspend_desc);
}

module_init(memfreq_gov_dbs_init);
module_exit(memfreq_gov_dbs_exit);
