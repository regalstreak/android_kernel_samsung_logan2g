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

#include<linux/cpufreq.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>

#define DEFAULT_CPU 0


#define cpufreq_attr_rw(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define cpufreq_attr_ro(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0444,			\
	},					\
	.show	= _name##_show,			\
}

static ssize_t cpufreq_table_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	unsigned int i = 0;
	ssize_t count = 0;
	struct cpufreq_frequency_table * table = cpufreq_frequency_get_table(DEFAULT_CPU);
	if(table == NULL)
		return 0;

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;
		count += sprintf(&buf[count], "%d ", table[i].frequency);
	}
	count += sprintf(&buf[count], "\n");

	return count;
}

cpufreq_attr_ro(cpufreq_table);

#define CPU_FREQ_MAX "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_max_freq"
#define CPU_FREQ_MIN "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_min_freq"

static void write_file(char * name, char * buf)
{
	mm_segment_t old_fs;
	struct file *file = NULL;
	if(file == NULL)
		file = filp_open(name, O_RDWR, 0644);
	if (IS_ERR(file)) {
		printk("error occured while opening file %s, exiting...\n", name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	file->f_op->write(file, (char *)buf, strlen(buf)+1, &file->f_pos);
	set_fs(old_fs);
	filp_close(file, NULL);  
	file = NULL;
	return ;
}

#define CPUFREQ_MIN	0x1
#define CPUFREQ_MAX	0x2
static int cpufreq_type = 0;
static long cpufreq_max =-1;
static long cpufreq_min = -1;

static void cpufreq_set_policy(void)
{
	int cpu;
	char name[256], buf[128];
	struct cpufreq_policy policy;
	for_each_cpu(cpu, cpu_online_mask){
		cpufreq_get_policy(& policy, cpu);
		if(CPUFREQ_MAX & cpufreq_type)
			policy.max = cpufreq_max;
		else
			policy.max = policy.cpuinfo.max_freq;
		sprintf(name, CPU_FREQ_MAX, cpu);
		sprintf(buf, "%d", policy.max);
		write_file(name, buf);
		
		if(CPUFREQ_MIN & cpufreq_type)
			policy.min = cpufreq_min;
		else
			policy.min = policy.cpuinfo.min_freq;
		sprintf(name, CPU_FREQ_MIN, cpu);
		sprintf(buf, "%d", policy.min);
		printk("write file = %s buf = %s", name, buf);
		write_file(name, buf);
	}
}

void update_policy_user(struct cpufreq_policy * policy)
{
	unsigned int max, min;
	if(CPUFREQ_MAX & cpufreq_type)
		max = cpufreq_max;
	else
		max = policy->cpuinfo.max_freq;

	if(CPUFREQ_MIN & cpufreq_type)
		min = cpufreq_min;
	else
		min = policy->cpuinfo.min_freq;
	if(min > policy->min)
		policy->min = min;
	if(max < policy->max)
		policy->max = max;
}

EXPORT_SYMBOL(update_policy_user);

static ssize_t cpufreq_max_limit_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	ssize_t count = 0;
	if(!(cpufreq_type & CPUFREQ_MIN) ||cpufreq_max == -1 )
		count += sprintf(&buf[count], "%d \n", -1);
	else
		count += sprintf(&buf[count], "%ld \n", cpufreq_max);
	return count;
}

static ssize_t cpufreq_max_limit_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t n)
{
	int freq = -1, freq_type = 0;
	unsigned int  max, min;
	struct cpufreq_policy policy;
	if(!cpufreq_get_policy(& policy, DEFAULT_CPU)){
		printk("can not get cpufreq_policy\n");
		return -1;
	}
	min = policy.cpuinfo.min_freq;
	max = policy.cpuinfo.max_freq;
	
	if( sscanf(buf, "%d", &freq) > 0){
		if(freq == -1){
			freq_type &= ~CPUFREQ_MAX;
			max = -1;
		}else if(min > freq){
			printk("min = %u max=%u freq=%u\n", min, max, freq);
			return -2;
		}else{
			max = freq;
			freq_type |= CPUFREQ_MAX;
		}
	}else{
		printk("not a number\n");
		return -3;
	}
	if((freq_type & CPUFREQ_MAX) != (cpufreq_type & CPUFREQ_MAX)  
		|| ((freq_type & CPUFREQ_MAX ) && max != cpufreq_max)){
		cpufreq_type &= ~CPUFREQ_MAX;
		cpufreq_type |= freq_type;
		cpufreq_max = max;
		cpufreq_set_policy();
	}
	cpufreq_max = max;
	return n;
}

cpufreq_attr_rw(cpufreq_max_limit);


static ssize_t cpufreq_min_limit_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	ssize_t count = 0;
	if(!(cpufreq_type & CPUFREQ_MIN) ||cpufreq_min == -1 )
		count += sprintf(&buf[count], "%d \n", -1);
	else
		count += sprintf(&buf[count], "%ld \n", cpufreq_min);
	return count;
}

static ssize_t cpufreq_min_limit_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	int freq = -1, freq_type = 0;
	unsigned int  max, min;
	struct cpufreq_policy policy;
	if(cpufreq_get_policy(& policy, DEFAULT_CPU)){
		printk("can not get cpufreq_policy\n");
		return -1;
	}
	min = policy.cpuinfo.min_freq;
	max = policy.cpuinfo.max_freq;
	
	if( sscanf(buf, "%d", &freq) > 0){
		if(freq == -1){
			freq_type &= ~CPUFREQ_MIN;
			max = -1;
		}else if(max < freq){
			printk("min = %u max=%u freq=%u\n", min, max, freq);
			return -2;
		}else{
			min = freq;
			freq_type |= CPUFREQ_MIN;
		}
	}else{
		printk("not a number\n");
		return -3;
	}
	printk("freq_type = %d, cpufreq_type=%d, min=%d, cpufreq_min=%ld\n", freq_type, cpufreq_type, min, cpufreq_min);
	if((freq_type & CPUFREQ_MIN) != (cpufreq_type & CPUFREQ_MIN)  
		|| ((freq_type & CPUFREQ_MIN ) && min != cpufreq_min)){
		cpufreq_type &= ~CPUFREQ_MIN;
		cpufreq_type |= freq_type;
		cpufreq_min = min;
		cpufreq_set_policy();
	}
	cpufreq_min = min;
	return n;
}

cpufreq_attr_rw(cpufreq_min_limit);


static struct attribute * g[] = {
	&cpufreq_table_attr.attr,
	& cpufreq_min_limit_attr.attr,
	& cpufreq_max_limit_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

extern struct kobject *power_kobj;

static int __init cpufreq_user_init(void)
{
	if(!power_kobj)
		power_kobj = kobject_create_and_add("power", NULL);
	if (!power_kobj)
		return -ENOMEM;
	return sysfs_create_group(power_kobj, &attr_group);
}

late_initcall(cpufreq_user_init);



