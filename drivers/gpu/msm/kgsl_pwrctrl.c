/* Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/export.h>
#include <linux/interrupt.h>
#include <asm/page.h>
#include <linux/pm_runtime.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <linux/ktime.h>
#include <linux/delay.h>

#include "kgsl.h"
#include "kgsl_pwrscale.h"
#include "kgsl_device.h"
#include "kgsl_trace.h"
#include "kgsl_sharedmem.h"

#define KGSL_PWRFLAGS_POWER_ON 0
#define KGSL_PWRFLAGS_CLK_ON   1
#define KGSL_PWRFLAGS_AXI_ON   2
#define KGSL_PWRFLAGS_IRQ_ON   3

#define UPDATE_BUSY_VAL		1000000
#define UPDATE_BUSY		50

/*
 * Expected delay for post-interrupt processing on A3xx.
 * The delay may be longer, gradually increase the delay
 * to compensate.  If the GPU isn't done by max delay,
 * it's working on something other than just the final
 * command sequence so stop waiting for it to be idle.
 */
#define INIT_UDELAY		200
#define MAX_UDELAY		2000

#ifdef CONFIG_CPU_FREQ_GOV_KRAKEN/*
 *  drivers/cpufreq/cpufreq_kraken.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *	      (C)  2013 flar2 <asegaert@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
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
#include <linux/kthread.h>
#include <linux/slab.h>

#include <mach/kgsl.h>
static int orig_up_threshold = 90;
static int g_count = 0;

#define DEF_SAMPLING_RATE			(30000)
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(90)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define MIN_FREQUENCY_DOWN_DIFFERENTIAL		(1)
#define UI_DYNAMIC_SAMPLING_RATE		(15000)
#define DBS_SWITCH_MODE_TIMEOUT			(1000)
#define INPUT_EVENT_MIN_TIMEOUT 		(0)
#define INPUT_EVENT_MAX_TIMEOUT 		(3000)
#define INPUT_EVENT_TIMEOUT			(500)
#define MIN_SAMPLING_RATE_RATIO			(2)

static unsigned int min_sampling_rate;
static unsigned int skip_kraken = 0;

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

static void do_dbs_timer(struct work_struct *work);
static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_KRAKEN
static
#endif
struct cpufreq_governor cpufreq_gov_kraken = {
       .name                   = "kraken",
       .governor               = cpufreq_governor_dbs,
       .max_transition_latency = TRANSITION_LATENCY_LIMIT,
       .owner                  = THIS_MODULE,
};

enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;
	unsigned int rate_mult;
	unsigned int prev_load;
	unsigned int max_load;
	int input_event_freq;
	int cpu;
	unsigned int sample_type:1;
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info);
static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info);

static unsigned int dbs_enable;	

static DEFINE_PER_CPU(struct task_struct *, up_task);
static spinlock_t input_boost_lock;
static bool input_event_boost = false;
static unsigned long input_event_boost_expired = 0;

#define TABLE_SIZE			5
#define MAX(x,y)			(x > y ? x : y)
#define MIN(x,y)			(x < y ? x : y)
#define FREQ_NEED_BURST(x)		(x < 600000 ? 1 : 0)

static	struct cpufreq_frequency_table *tbl = NULL;
static unsigned int *tblmap[TABLE_SIZE] __read_mostly;
static unsigned int tbl_select[4];
static unsigned int up_threshold_level[2] __read_mostly = {95, 85};
static int input_event_counter = 0;
struct timer_list freq_mode_timer;

static inline void switch_turbo_mode(unsigned);
static inline void switch_normal_mode(void);

static DEFINE_MUTEX(dbs_mutex);

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int up_threshold_multi_core;
	unsigned int down_differential;
	unsigned int down_differential_multi_core;
	unsigned int optimal_freq;
	unsigned int up_threshold_any_cpu_load;
	unsigned int sync_freq;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	unsigned int io_is_busy;
	unsigned int two_phase_freq;
	unsigned int origin_sampling_rate;
	unsigned int ui_sampling_rate;
	unsigned int input_event_timeout;
	int gboost;
} dbs_tuners_ins = {
	.up_threshold_multi_core = DEF_FREQUENCY_UP_THRESHOLD,
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.down_differential_multi_core = MICRO_FREQUENCY_DOWN_DIFFERENTIAL,
	.up_threshold_any_cpu_load = DEF_FREQUENCY_UP_THRESHOLD,
	.ignore_nice = 0,
	.sync_freq = 0,
	.optimal_freq = 0,
	.io_is_busy = 1,
	.two_phase_freq = 0,
	.ui_sampling_rate = UI_DYNAMIC_SAMPLING_RATE,
	.input_event_timeout = INPUT_EVENT_TIMEOUT,
	.gboost = 1,
};

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_min);

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(up_threshold, up_threshold);
show_one(up_threshold_multi_core, up_threshold_multi_core);
show_one(down_differential, down_differential);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);
show_one(optimal_freq, optimal_freq);
show_one(up_threshold_any_cpu_load, up_threshold_any_cpu_load);
show_one(sync_freq, sync_freq);
show_one(gboost, gboost);

static void update_sampling_rate(unsigned int new_rate)
{
	int cpu;

	dbs_tuners_ins.sampling_rate = new_rate
				     = max(new_rate, min_sampling_rate);

	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct cpu_dbs_info_s *dbs_info;
		unsigned long next_sampling, appointed_at;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		dbs_info = &per_cpu(od_cpu_dbs_info, policy->cpu);
		cpufreq_cpu_put(policy);

		mutex_lock(&dbs_info->timer_mutex);

		if (!delayed_work_pending(&dbs_info->work)) {
			mutex_unlock(&dbs_info->timer_mutex);
			continue;
		}

		next_sampling  = jiffies + usecs_to_jiffies(new_rate);
		appointed_at = dbs_info->work.timer.expires;

		if (time_before(next_sampling, appointed_at)) {

			mutex_unlock(&dbs_info->timer_mutex);
			cancel_delayed_work_sync(&dbs_info->work);
			mutex_lock(&dbs_info->timer_mutex);

			schedule_delayed_work_on(dbs_info->cpu, &dbs_info->work,
						 usecs_to_jiffies(new_rate));

		}
		mutex_unlock(&dbs_info->timer_mutex);
	}
}

show_one(input_event_timeout, input_event_timeout);

static ssize_t store_input_event_timeout(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
	return -EINVAL;

	input = max(input, (unsigned int)INPUT_EVENT_MIN_TIMEOUT);
	dbs_tuners_ins.input_event_timeout = min(input, (unsigned int)INPUT_EVENT_MAX_TIMEOUT);

	return count;
}

static int two_phase_freq_array[NR_CPUS] = {[0 ... NR_CPUS-1] = 1728000} ;

static ssize_t show_two_phase_freq
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int i = 0 ;
	int shift = 0 ;
	char *buf_pos = buf;
	for ( i = 0 ; i < NR_CPUS; i++) {
		shift = sprintf(buf_pos,"%d,",two_phase_freq_array[i]);
		buf_pos += shift;
	}
	*(buf_pos-1) = '\0';
	return strlen(buf);
}

static ssize_t store_two_phase_freq(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	int ret = 0;
	if (NR_CPUS == 1)
		ret = sscanf(buf,"%u",&two_phase_freq_array[0]);
	else if (NR_CPUS == 2)
		ret = sscanf(buf,"%u,%u",&two_phase_freq_array[0],
				&two_phase_freq_array[1]);
	else if (NR_CPUS == 4)
		ret = sscanf(buf, "%u,%u,%u,%u", &two_phase_freq_array[0],
				&two_phase_freq_array[1],
				&two_phase_freq_array[2],
				&two_phase_freq_array[3]);
	if (ret < NR_CPUS)
		return -EINVAL;

	return count;
}

static int input_event_min_freq_array[NR_CPUS] = {1728000, 1267200, 1267200, 1267200} ;

static ssize_t show_input_event_min_freq
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int i = 0 ;
	int shift = 0 ;
	char *buf_pos = buf;
	for ( i = 0 ; i < NR_CPUS; i++) {
		shift = sprintf(buf_pos,"%d,",input_event_min_freq_array[i]);
		buf_pos += shift;
	}
	*(buf_pos-1) = '\0';
	return strlen(buf);
}

static ssize_t store_input_event_min_freq(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	int ret = 0;
	if (NR_CPUS == 1)
		ret = sscanf(buf,"%u",&input_event_min_freq_array[0]);
	else if (NR_CPUS == 2)
		ret = sscanf(buf,"%u,%u",&input_event_min_freq_array[0],
				&input_event_min_freq_array[1]);
	else if (NR_CPUS == 4)
		ret = sscanf(buf, "%u,%u,%u,%u", &input_event_min_freq_array[0],
				&input_event_min_freq_array[1],
				&input_event_min_freq_array[2],
				&input_event_min_freq_array[3]);
	if (ret < NR_CPUS)
		return -EINVAL;

	return count;
}

show_one(ui_sampling_rate, ui_sampling_rate);

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	if (input == dbs_tuners_ins.origin_sampling_rate)
		return count;
	update_sampling_rate(input);
	dbs_tuners_ins.origin_sampling_rate = dbs_tuners_ins.sampling_rate;
	return count;
}

static ssize_t store_ui_sampling_rate(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.ui_sampling_rate = max(input, min_sampling_rate);

	return count;
}

static ssize_t store_sync_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.sync_freq = input;
	return count;
}

static ssize_t store_optimal_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.optimal_freq = input;
	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold = input;
	orig_up_threshold = dbs_tuners_ins.up_threshold;
	return count;
}

static ssize_t store_up_threshold_multi_core(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold_multi_core = input;
	return count;
}

static ssize_t store_up_threshold_any_cpu_load(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold_any_cpu_load = input;
	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input >= dbs_tuners_ins.up_threshold ||
			input < MIN_FREQUENCY_DOWN_DIFFERENTIAL) {
		return -EINVAL;
	}

	dbs_tuners_ins.down_differential = input;

	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_factor = input;

	
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) { 
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
                                                &dbs_info->prev_cpu_wall, dbs_tuners_ins.io_is_busy);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

static ssize_t store_gboost(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if(ret != 1)
		return -EINVAL;
	dbs_tuners_ins.gboost = (input > 0 ? input : 0);
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(up_threshold);
define_one_global_rw(down_differential);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(up_threshold_multi_core);
define_one_global_rw(optimal_freq);
define_one_global_rw(up_threshold_any_cpu_load);
define_one_global_rw(sync_freq);
define_one_global_rw(two_phase_freq);
define_one_global_rw(input_event_min_freq);
define_one_global_rw(ui_sampling_rate);
define_one_global_rw(input_event_timeout);
define_one_global_rw(gboost);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_differential.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&up_threshold_multi_core.attr,
	&optimal_freq.attr,
	&up_threshold_any_cpu_load.attr,
	&sync_freq.attr,
	&two_phase_freq.attr,
	&input_event_min_freq.attr,
	&ui_sampling_rate.attr,
	&input_event_timeout.attr,
	&gboost.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "kraken",
};


static inline void switch_turbo_mode(unsigned timeout)
{
	if (timeout > 0)
		mod_timer(&freq_mode_timer, jiffies + msecs_to_jiffies(timeout));
	tbl_select[0] = 2;
	tbl_select[1] = 3;
	tbl_select[2] = 4;
	tbl_select[3] = 4;
}

static inline void switch_normal_mode(void)
{
	if (input_event_counter > 0)
		return;
	tbl_select[0] = 0;
	tbl_select[1] = 1;
	tbl_select[2] = 2;
	tbl_select[3] = 4;
}

static void switch_mode_timer(unsigned long data)
{
	switch_normal_mode();
}

static void dbs_init_freq_map_table(struct cpufreq_policy *policy)
{
	unsigned int min_diff, top1, top2;
	int cnt, i, j;

	tbl = cpufreq_frequency_get_table(0);
	min_diff = policy->cpuinfo.max_freq;

	
	for (cnt = 0; (tbl[cnt].frequency != CPUFREQ_TABLE_END); cnt++) {
		if (cnt > 0)
			min_diff = MIN(tbl[cnt].frequency - tbl[cnt-1].frequency, min_diff);
	}
	
	top1 = (policy->cpuinfo.max_freq + policy->cpuinfo.min_freq) / 2;
	top2 = (policy->cpuinfo.max_freq + top1) / 2;

	for (i = 0; i < TABLE_SIZE; i++) {
		
		tblmap[i] = kmalloc(sizeof(unsigned int) * cnt, GFP_KERNEL);
		BUG_ON(!tblmap[i]);
		
		for (j = 0; j < cnt; j++)
			tblmap[i][j] = tbl[j].frequency;
	}

	for (j = 0; j < cnt; j++) {
		
		if (tbl[j].frequency < top1) {
			tblmap[0][j] += MAX((top1 - tbl[j].frequency)/3, min_diff);
		}
		
		if (tbl[j].frequency < top2) {
			tblmap[1][j] += MAX((top2 - tbl[j].frequency)/3, min_diff);
			tblmap[2][j] += MAX(((top2 - tbl[j].frequency)*2)/5, min_diff);
			tblmap[3][j] += MAX((top2 - tbl[j].frequency)/2, min_diff);
		}
		else {
			tblmap[3][j] += MAX((policy->cpuinfo.max_freq - tbl[j].frequency)/3, min_diff);
		}
		
		tblmap[4][j] += MAX((policy->cpuinfo.max_freq - tbl[j].frequency)/2, min_diff);
	}

	switch_normal_mode();
	
	init_timer(&freq_mode_timer);
	freq_mode_timer.function = switch_mode_timer;
	freq_mode_timer.data = 0;

#if 0
	
	for (i = 0; i < TABLE_SIZE; i++) {
		pr_info("Table %d shows:\n", i+1);
		for (j = 0; j < cnt; j++) {
			pr_info("%02d: %8u\n", j, tblmap[i][j]);
		}
	}
#endif
}

static void dbs_deinit_freq_map_table(void)
{
	int i;

	if (!tbl)
		return;

	tbl = NULL;

	for (i = 0; i < TABLE_SIZE; i++)
		kfree(tblmap[i]);

	del_timer(&freq_mode_timer);
}

static inline int get_cpu_freq_index(unsigned int freq)
{
	static int saved_index = 0;
	int index;

	if (!tbl) {
		pr_warn("tbl is NULL, use previous value %d\n", saved_index);
		return saved_index;
	}

	for (index = 0; (tbl[index].frequency != CPUFREQ_TABLE_END); index++) {
		if (tbl[index].frequency >= freq) {
			saved_index = index;
			break;
		}
	}

	return index;
}

static void dbs_freq_increase(struct cpufreq_policy *p, unsigned load, unsigned int freq)
{
	if (p->cur == p->max) {
		return;
	}

	__cpufreq_driver_target(p, freq, (freq < p->max) ?
			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);
}

int set_two_phase_freq(int cpufreq)
{
	int i  = 0;
	for ( i = 0 ; i < NR_CPUS; i++)
		two_phase_freq_array[i] = cpufreq;
	return 0;
}

void set_two_phase_freq_by_cpu ( int cpu_nr, int cpufreq){
	two_phase_freq_array[cpu_nr-1] = cpufreq;
}

int input_event_boosted(void)
{
	unsigned long flags;

	spin_lock_irqsave(&input_boost_lock, flags);
	if (input_event_boost) {
		if (time_before(jiffies, input_event_boost_expired)) {
			spin_unlock_irqrestore(&input_boost_lock, flags);
			return 1;
		}
		input_event_boost = false;
		dbs_tuners_ins.sampling_rate = dbs_tuners_ins.origin_sampling_rate;
	}
	spin_unlock_irqrestore(&input_boost_lock, flags);

	return 0;
}

static void boost_min_freq(int min_freq)
{
	int i;
	struct cpu_dbs_info_s *dbs_info;

	for_each_online_cpu(i) {
		dbs_info = &per_cpu(od_cpu_dbs_info, i);
		
		if (dbs_info->cur_policy
			&& dbs_info->cur_policy->cur < min_freq) {
			dbs_info->input_event_freq = min_freq;
			wake_up_process(per_cpu(up_task, i));
		}
	}
}

static unsigned int get_cpu_current_load(unsigned int j, unsigned int *record)
{
	unsigned int cur_load = 0;
	struct cpu_dbs_info_s *j_dbs_info;
	cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
	unsigned int idle_time, wall_time, iowait_time;

	j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

	if (record)
		*record = j_dbs_info->prev_load;

        cur_idle_time = get_cpu_idle_time(j, &cur_wall_time, dbs_tuners_ins.io_is_busy);
	cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

	wall_time = (unsigned int)
		(cur_wall_time - j_dbs_info->prev_cpu_wall);
	j_dbs_info->prev_cpu_wall = cur_wall_time;

	idle_time = (unsigned int)
		(cur_idle_time - j_dbs_info->prev_cpu_idle);
	j_dbs_info->prev_cpu_idle = cur_idle_time;

	iowait_time = (unsigned int)
		(cur_iowait_time - j_dbs_info->prev_cpu_iowait);
	j_dbs_info->prev_cpu_iowait = cur_iowait_time;

	if (dbs_tuners_ins.ignore_nice) {
		u64 cur_nice;
		unsigned long cur_nice_jiffies;

		cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
				 j_dbs_info->prev_cpu_nice;
		cur_nice_jiffies = (unsigned long)
				cputime64_to_jiffies64(cur_nice);

		j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		idle_time += jiffies_to_usecs(cur_nice_jiffies);
	}

	if (dbs_tuners_ins.io_is_busy && idle_time >= iowait_time)
		idle_time -= iowait_time;

	if (unlikely(!wall_time || wall_time < idle_time))
		return j_dbs_info->prev_load;

	cur_load = 100 * (wall_time - idle_time) / wall_time;
	j_dbs_info->max_load  = max(cur_load, j_dbs_info->prev_load);
	j_dbs_info->prev_load = cur_load;

	return cur_load;
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int max_load_freq;
	
	unsigned int cur_load = 0;

	unsigned int max_load_other_cpu = 0;
	struct cpufreq_policy *policy;
	unsigned int j, prev_load = 0, freq_next;

	static unsigned int phase = 0;
	static unsigned int counter = 0;
	unsigned int nr_cpus;

	this_dbs_info->freq_lo = 0;
	policy = this_dbs_info->cur_policy;

	max_load_freq = 0;

	for_each_cpu(j, policy->cpus) {
		cur_load = get_cpu_current_load(j, &prev_load);
					
		if (cur_load > max_load_freq)
			max_load_freq = cur_load * policy->cur;
	}

	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *j_dbs_info;
		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

		if (j == policy->cpu)
			continue;

		if (max_load_other_cpu < j_dbs_info->max_load)
			max_load_other_cpu = j_dbs_info->max_load;

		if ((j_dbs_info->cur_policy != NULL)
			&& (j_dbs_info->cur_policy->cur ==
					j_dbs_info->cur_policy->max)) {

			if (policy->cur >= dbs_tuners_ins.optimal_freq)
				max_load_other_cpu =
				dbs_tuners_ins.up_threshold_any_cpu_load;
		}
	}

	//gboost
	if (g_count > 30) {

		if (max_load_freq > dbs_tuners_ins.up_threshold * policy->cur) {
		
			if (counter < 5) {
				counter++;
				if (counter > 2) {
					phase = 1;
				}
			}

			nr_cpus = num_online_cpus();
			dbs_tuners_ins.two_phase_freq = two_phase_freq_array[nr_cpus-1];
			if (dbs_tuners_ins.two_phase_freq < policy->cur)
				phase=1;
			if (dbs_tuners_ins.two_phase_freq != 0 && phase == 0) {
			
				dbs_freq_increase(policy, cur_load, dbs_tuners_ins.two_phase_freq);
			} else {
			
				if (policy->cur < policy->max)
					this_dbs_info->rate_mult =
						dbs_tuners_ins.sampling_down_factor;
				dbs_freq_increase(policy, cur_load, policy->max);
			}
			return;
		}

	} else {

		if (max_load_freq > up_threshold_level[1] * policy->cur) {
			unsigned int avg_load = (prev_load + cur_load) >> 1;
			int index = get_cpu_freq_index(policy->cur);
	
			if (FREQ_NEED_BURST(policy->cur) && cur_load > up_threshold_level[0]) {
				freq_next = tblmap[tbl_select[3]][index];
			}
			
			else if (avg_load > up_threshold_level[0]) {
				freq_next = tblmap[tbl_select[3]][index];
			}
			
			else if (avg_load <= up_threshold_level[1]) {
				freq_next = tblmap[tbl_select[0]][index];
			}
		
			else {
			
				if (cur_load > up_threshold_level[0]) {
					freq_next = tblmap[tbl_select[2]][index];
				}
			
				else {
					freq_next = tblmap[tbl_select[1]][index];
				}
			}
			dbs_freq_increase(policy, cur_load, freq_next);
			if (policy->cur == policy->max)
				this_dbs_info->rate_mult = dbs_tuners_ins.sampling_down_factor;

			return;
		}
	}

	if (dbs_tuners_ins.gboost) {
		if (counter > 0) {
			counter--;
			if (counter == 0) {			
				phase = 0;
			}
		}
	}

	if (dbs_tuners_ins.gboost) {

		if (g_count < 100 && graphics_boost < 5) {
			++g_count;
		} else if (g_count > 1) {
			--g_count;
			--g_count;
		}

		if (graphics_boost < 4 && g_count > 80) {
			dbs_tuners_ins.up_threshold = 60 + (graphics_boost * 10);
		} else {
			dbs_tuners_ins.up_threshold = orig_up_threshold;
		}

		if (g_count > 80)
			boost_min_freq(1267200);
	}
	//end

	if (num_online_cpus() > 1) {
		if (max_load_other_cpu >
				dbs_tuners_ins.up_threshold_any_cpu_load) {
			if (policy->cur < dbs_tuners_ins.sync_freq)
				dbs_freq_increase(policy, cur_load,
						dbs_tuners_ins.sync_freq);
			return;
		}

		if (max_load_freq > dbs_tuners_ins.up_threshold_multi_core *
								policy->cur) {
			if (policy->cur < dbs_tuners_ins.optimal_freq)
				dbs_freq_increase(policy, cur_load,
						dbs_tuners_ins.optimal_freq);
			return;
		}
	}

	if (input_event_boosted())
	{
		return;
	}
	
	if (policy->cur == policy->min){
		return;
	}

	if (max_load_freq <
	    (dbs_tuners_ins.up_threshold - dbs_tuners_ins.down_differential) *
	     policy->cur) {
		freq_next = max_load_freq /
				(dbs_tuners_ins.up_threshold -
				 dbs_tuners_ins.down_differential);
		
		this_dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (num_online_cpus() > 1) {
			if (max_load_other_cpu >
			(dbs_tuners_ins.up_threshold_multi_core -
			dbs_tuners_ins.down_differential) &&
			freq_next < dbs_tuners_ins.sync_freq)
				freq_next = dbs_tuners_ins.sync_freq;

			if (dbs_tuners_ins.optimal_freq > policy->min && max_load_freq >
				 (dbs_tuners_ins.up_threshold_multi_core -
				  dbs_tuners_ins.down_differential_multi_core) *
				  policy->cur)
				freq_next = dbs_tuners_ins.optimal_freq;
		}

		__cpufreq_driver_target(policy, freq_next,
			CPUFREQ_RELATION_L);
	}
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;
	int sample_type = dbs_info->sample_type;
	int delay = msecs_to_jiffies(50);

	mutex_lock(&dbs_info->timer_mutex);

	if (skip_kraken)
		goto sched_wait;
	
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	if (sample_type == DBS_NORMAL_SAMPLE) {
		dbs_check_cpu(dbs_info);
		if (dbs_info->freq_lo) {
			
			dbs_info->sample_type = DBS_SUB_SAMPLE;
			delay = dbs_info->freq_hi_jiffies;
		} else {
			delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate
				* dbs_info->rate_mult);

			if (num_online_cpus() > 1)
				delay -= jiffies % delay;
		}
	} else {
		if (input_event_boosted())
			goto sched_wait;

		__cpufreq_driver_target(dbs_info->cur_policy,
			dbs_info->freq_lo, CPUFREQ_RELATION_H);
		delay = dbs_info->freq_lo_jiffies;
	}

sched_wait:
	schedule_delayed_work_on(cpu, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	schedule_delayed_work_on(dbs_info->cpu, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
}


static void dbs_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	unsigned long flags;
	int input_event_min_freq;

	if (dbs_tuners_ins.input_event_timeout == 0)
		return;

	if (type == EV_ABS && code == ABS_MT_TRACKING_ID) {
		if (value != -1) {		

			input_event_min_freq = input_event_min_freq_array[num_online_cpus() - 1];

			input_event_counter++;
			switch_turbo_mode(0);
			
			spin_lock_irqsave(&input_boost_lock, flags);
			input_event_boost = true;
			input_event_boost_expired = jiffies + usecs_to_jiffies(dbs_tuners_ins.input_event_timeout * 1000);
			dbs_tuners_ins.sampling_rate = dbs_tuners_ins.ui_sampling_rate;
			spin_unlock_irqrestore(&input_boost_lock, flags);

			boost_min_freq(input_event_min_freq);
		}
		else {		
			if (likely(input_event_counter > 0))
				input_event_counter--;
			else
				pr_debug("dbs_input_event: Touch isn't paired!\n");

			
			switch_turbo_mode(DBS_SWITCH_MODE_TIMEOUT);
		}
	}
}

static int dbs_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dbs_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler dbs_input_handler = {
	.event		= dbs_input_event,
	.connect	= dbs_input_connect,
	.disconnect	= dbs_input_disconnect,
	.name		= "cpufreq_kraken",
	.id_table	= dbs_ids,
};


void set_input_event_min_freq_by_cpu ( int cpu_nr, int cpufreq){
	input_event_min_freq_array[cpu_nr-1] = cpufreq;
}
static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		dbs_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall, dbs_tuners_ins.io_is_busy);
			if (dbs_tuners_ins.ignore_nice)
				j_dbs_info->prev_cpu_nice =
						kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		}
		this_dbs_info->cpu = cpu;
		this_dbs_info->rate_mult = 1;
		if (dbs_enable == 1) {
			unsigned int latency;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;
			
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			dbs_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * LATENCY_MULTIPLIER);
			if (dbs_tuners_ins.sampling_rate < DEF_SAMPLING_RATE)
				dbs_tuners_ins.sampling_rate = DEF_SAMPLING_RATE;
			dbs_tuners_ins.origin_sampling_rate = dbs_tuners_ins.sampling_rate;

			if (dbs_tuners_ins.optimal_freq == 0)
				dbs_tuners_ins.optimal_freq = policy->min;

			if (dbs_tuners_ins.sync_freq == 0)
				dbs_tuners_ins.sync_freq = policy->min;

			dbs_init_freq_map_table(policy);
		}
		if (!cpu)
			rc = input_register_handler(&dbs_input_handler);
		mutex_unlock(&dbs_mutex);

		mutex_init(&this_dbs_info->timer_mutex);

		dbs_timer_init(this_dbs_info);
		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		dbs_enable--;
		this_dbs_info->cur_policy = NULL;
		if (!cpu)
			input_unregister_handler(&dbs_input_handler);
		mutex_unlock(&dbs_mutex);
		if (!dbs_enable) {
			dbs_deinit_freq_map_table();
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);
		}
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if(this_dbs_info->cur_policy){
			if (policy->max < this_dbs_info->cur_policy->cur)
				__cpufreq_driver_target(this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
			else if (policy->min > this_dbs_info->cur_policy->cur)
				__cpufreq_driver_target(this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		}
		mutex_unlock(&this_dbs_info->timer_mutex);
		break;
	}
	return 0;
}

static int cpufreq_gov_dbs_up_task(void *data)
{
	struct cpufreq_policy *policy;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int cpu = smp_processor_id();

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if (kthread_should_stop())
			break;

		set_current_state(TASK_RUNNING);

		get_online_cpus();

		if (lock_policy_rwsem_write(cpu) < 0)
			goto bail_acq_sema_failed;

		this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
		policy = this_dbs_info->cur_policy;
		if (!policy) {
			
			goto bail_incorrect_governor;
		}

		mutex_lock(&this_dbs_info->timer_mutex);

		dbs_freq_increase(policy, this_dbs_info->prev_load, this_dbs_info->input_event_freq);
		this_dbs_info->prev_cpu_idle = get_cpu_idle_time(cpu, &this_dbs_info->prev_cpu_wall, dbs_tuners_ins.io_is_busy);

		mutex_unlock(&this_dbs_info->timer_mutex);

bail_incorrect_governor:
		unlock_policy_rwsem_write(cpu);

bail_acq_sema_failed:
		put_online_cpus();

		dbs_tuners_ins.sampling_rate = dbs_tuners_ins.ui_sampling_rate;
	}

	return 0;
}

static int __init cpufreq_gov_dbs_init(void)
{
	u64 idle_time;
	unsigned int i;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
	struct task_struct *pthread;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();
	if (idle_time != -1ULL) {
		
		dbs_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		dbs_tuners_ins.down_differential =
					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		
		min_sampling_rate =
			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
	}

	spin_lock_init(&input_boost_lock);

	for_each_possible_cpu(i) {
		pthread = kthread_create_on_node(cpufreq_gov_dbs_up_task,
								NULL, cpu_to_node(i),
								"kdbs_up/%d", i);
		if (likely(!IS_ERR(pthread))) {
			kthread_bind(pthread, i);
			sched_setscheduler_nocheck(pthread, SCHED_FIFO, &param);
			get_task_struct(pthread);
			per_cpu(up_task, i) = pthread;
		}
	}
	return cpufreq_register_governor(&cpufreq_gov_kraken);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	unsigned int i;

	cpufreq_unregister_governor(&cpufreq_gov_kraken);
	for_each_possible_cpu(i) {
		struct cpu_dbs_info_s *this_dbs_info =
			&per_cpu(od_cpu_dbs_info, i);
		mutex_destroy(&this_dbs_info->timer_mutex);
		if (per_cpu(up_task, i)) {
			kthread_stop(per_cpu(up_task, i));
			put_task_struct(per_cpu(up_task, i));
		}
	}
}

MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_AUTHOR("flar2 <asegaert@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_kraken' - multiphase dynamic cpufreq governor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_KRAKEN
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);

int graphics_boost = 6;
#endif

struct clk_pair {
	const char *name;
	uint map;
};

static struct clk_pair clks[KGSL_MAX_CLKS] = {
	{
		.name = "src_clk",
		.map = KGSL_CLK_SRC,
	},
	{
		.name = "core_clk",
		.map = KGSL_CLK_CORE,
	},
	{
		.name = "iface_clk",
		.map = KGSL_CLK_IFACE,
	},
	{
		.name = "mem_clk",
		.map = KGSL_CLK_MEM,
	},
	{
		.name = "mem_iface_clk",
		.map = KGSL_CLK_MEM_IFACE,
	},
	{
		.name = "alt_mem_iface_clk",
		.map = KGSL_CLK_ALT_MEM_IFACE,
	},
};

static void kgsl_pwrctrl_clk(struct kgsl_device *device, int state,
					int requested_state);
static void kgsl_pwrctrl_axi(struct kgsl_device *device, int state);
static void kgsl_pwrctrl_pwrrail(struct kgsl_device *device, int state);

/* Update the elapsed time at a particular clock level
 * if the device is active(on_time = true).Otherwise
 * store it as sleep time.
 */
static void update_clk_statistics(struct kgsl_device *device,
				bool on_time)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_clk_stats *clkstats = &pwr->clk_stats;
	ktime_t elapsed;
	int elapsed_us;
	if (clkstats->start.tv64 == 0)
		clkstats->start = ktime_get();
	clkstats->stop = ktime_get();
	elapsed = ktime_sub(clkstats->stop, clkstats->start);
	elapsed_us = ktime_to_us(elapsed);
	clkstats->elapsed += elapsed_us;
	if (on_time)
		clkstats->clock_time[pwr->active_pwrlevel] += elapsed_us;
	else
		clkstats->clock_time[pwr->num_pwrlevels - 1] += elapsed_us;
	clkstats->start = ktime_get();
}

/*
 * Given a requested power level do bounds checking on the constraints and
 * return the nearest possible level
 */

static inline unsigned int _adjust_pwrlevel(struct kgsl_pwrctrl *pwr, int level)
{
	unsigned int max_pwrlevel = max_t(unsigned int, pwr->thermal_pwrlevel,
		pwr->max_pwrlevel);
	unsigned int min_pwrlevel = max_t(unsigned int, pwr->thermal_pwrlevel,
		pwr->min_pwrlevel);

	if (level < max_pwrlevel)
		return max_pwrlevel;
	if (level > min_pwrlevel)
		return min_pwrlevel;

	return level;
}

void kgsl_pwrctrl_buslevel_update(struct kgsl_device *device,
			bool on)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int cur = pwr->pwrlevels[pwr->active_pwrlevel].bus_freq;
	int buslevel = 0;
	if (!pwr->pcl)
		return;
	/* the bus should be ON to update the active frequency */
	if (on && !(test_bit(KGSL_PWRFLAGS_AXI_ON, &pwr->power_flags)))
		return;
	/*
	 * If the bus should remain on calculate our request and submit it,
	 * otherwise request bus level 0, off.
	 */
	if (on) {
		buslevel = min_t(int, pwr->pwrlevels[0].bus_freq,
				cur + pwr->bus_mod);
		buslevel = max_t(int, buslevel, 1);
	}
	msm_bus_scale_client_update_request(pwr->pcl, buslevel);
	trace_kgsl_buslevel(device, pwr->active_pwrlevel, buslevel);
}
EXPORT_SYMBOL(kgsl_pwrctrl_buslevel_update);

void kgsl_pwrctrl_pwrlevel_change(struct kgsl_device *device,
				unsigned int new_level)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_pwrlevel *pwrlevel;

	/* Adjust the power level to the current constraints */
	new_level = _adjust_pwrlevel(pwr, new_level);

	if (new_level == pwr->active_pwrlevel)
		return;

	update_clk_statistics(device, true);

	/*
	 * Set the active powerlevel first in case the clocks are off - if we
	 * don't do this then the pwrlevel change won't take effect when the
	 * clocks come back
	 */

	pwr->active_pwrlevel = new_level;
	pwr->bus_mod = 0;
	pwrlevel = &pwr->pwrlevels[pwr->active_pwrlevel];

	kgsl_pwrctrl_buslevel_update(device, true);

	clk_set_rate(pwr->grp_clks[0], pwr->pwrlevels[new_level].gpu_freq);


	trace_kgsl_pwrlevel(device, pwr->active_pwrlevel, pwrlevel->gpu_freq);

#ifdef CONFIG_CPU_FREQ_GOV_KRAKEN
        graphics_boost = pwr->active_pwrlevel;
#endif
}

EXPORT_SYMBOL(kgsl_pwrctrl_pwrlevel_change);

static ssize_t kgsl_pwrctrl_thermal_pwrlevel_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	int ret;
	unsigned int level = 0;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &level);

	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	if (level > pwr->num_pwrlevels - 2)
		level = pwr->num_pwrlevels - 2;

	pwr->thermal_pwrlevel = level;

	/*
	 * If there is no power policy set the clock to the requested thermal
	 * level - if thermal now happens to be higher than max, then that will
	 * be limited by the pwrlevel change function.  Otherwise if there is
	 * a policy only change the active clock if it is higher then the new
	 * thermal level
	 */
	if (pwr->thermal_pwrlevel > pwr->active_pwrlevel)
		kgsl_pwrctrl_pwrlevel_change(device, pwr->thermal_pwrlevel);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return count;
}

static ssize_t kgsl_pwrctrl_thermal_pwrlevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%d\n", pwr->thermal_pwrlevel);
}

static ssize_t kgsl_pwrctrl_max_pwrlevel_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	int ret;
	unsigned int max_level, level = 0;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &level);
	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	/* You can't set a maximum power level lower than the minimum */
	if (level > pwr->min_pwrlevel)
		level = pwr->min_pwrlevel;

	pwr->max_pwrlevel = level;


	max_level = max_t(unsigned int, pwr->thermal_pwrlevel,
		pwr->max_pwrlevel);

	/*
	 * If there is no policy then move to max by default.  Otherwise only
	 * move max if the current level happens to be higher then the new max
	 */
	if (max_level > pwr->active_pwrlevel)
		kgsl_pwrctrl_pwrlevel_change(device, max_level);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return count;
}

static ssize_t kgsl_pwrctrl_max_pwrlevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%u\n", pwr->max_pwrlevel);
}

static ssize_t kgsl_pwrctrl_min_pwrlevel_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	int ret;
	unsigned int min_level, level = 0;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &level);
	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	if (level > pwr->num_pwrlevels - 2)
		level = pwr->num_pwrlevels - 2;

	/* You can't set a minimum power level lower than the maximum */
	if (level < pwr->max_pwrlevel)
		level = pwr->max_pwrlevel;

	pwr->min_pwrlevel = level;

	min_level = max_t(unsigned int, pwr->thermal_pwrlevel,
		pwr->min_pwrlevel);

	/* Only move the power level higher if minimum is higher then the
	 * current level
	 */

	if (min_level < pwr->active_pwrlevel)
		kgsl_pwrctrl_pwrlevel_change(device, min_level);

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return count;
}

static ssize_t kgsl_pwrctrl_min_pwrlevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%u\n", pwr->min_pwrlevel);
}

static ssize_t kgsl_pwrctrl_active_pwrlevel_show(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%d\n", pwr->active_pwrlevel);
}


static ssize_t kgsl_pwrctrl_num_pwrlevels_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%d\n", pwr->num_pwrlevels - 1);
}

/* Given a GPU clock value, return the lowest matching powerlevel */

static int _get_nearest_pwrlevel(struct kgsl_pwrctrl *pwr, unsigned int clock)
{
	int i;

	for (i = pwr->num_pwrlevels - 1; i >= 0; i--) {
		if (abs(pwr->pwrlevels[i].gpu_freq - clock) < 5000000)
			return i;
	}

	return -ERANGE;
}

static ssize_t kgsl_pwrctrl_max_gpuclk_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	unsigned int val = 0;
	int level, ret;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	level = _get_nearest_pwrlevel(pwr, val);
	if (level < 0)
		goto done;

	pwr->thermal_pwrlevel = (unsigned int) level;

	/*
	 * if the thermal limit is lower than the current setting,
	 * move the speed down immediately
	 */

	if (pwr->thermal_pwrlevel > pwr->active_pwrlevel)
		kgsl_pwrctrl_pwrlevel_change(device, pwr->thermal_pwrlevel);

done:
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	return count;
}

static ssize_t kgsl_pwrctrl_max_gpuclk_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	return snprintf(buf, PAGE_SIZE, "%d\n",
			pwr->pwrlevels[pwr->thermal_pwrlevel].gpu_freq);
}

static ssize_t kgsl_pwrctrl_gpuclk_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	unsigned int val = 0;
	int ret, level;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	level = _get_nearest_pwrlevel(pwr, val);
	if (level >= 0)
		kgsl_pwrctrl_pwrlevel_change(device, (unsigned int) level);

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	return count;
}

static ssize_t kgsl_pwrctrl_gpuclk_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	unsigned long freq;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;

	if (device->state == KGSL_STATE_SLUMBER)
		freq = pwr->pwrlevels[pwr->num_pwrlevels - 1].gpu_freq;
	else
		freq = kgsl_pwrctrl_active_freq(pwr);

	return snprintf(buf, PAGE_SIZE, "%lu\n", freq);
}

static ssize_t kgsl_pwrctrl_idle_timer_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	/*
	 * We don't quite accept a maximum of 0xFFFFFFFF due to internal jiffy
	 * math, so make sure the value falls within the largest offset we can
	 * deal with
	 */

	if (val > jiffies_to_usecs(MAX_JIFFY_OFFSET))
		return -EINVAL;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	/* Let the timeout be requested in jiffies */
	device->pwrctrl.interval_timeout = msecs_to_jiffies(val);

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return count;
}

static ssize_t kgsl_pwrctrl_idle_timer_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	/* Show the idle_timeout in msec */
	return snprintf(buf, PAGE_SIZE, "%d\n",
		jiffies_to_msecs(device->pwrctrl.interval_timeout));
}

static ssize_t kgsl_pwrctrl_pmqos_active_latency_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	device->pwrctrl.pm_qos_active_latency = val;
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return count;
}

static ssize_t kgsl_pwrctrl_pmqos_active_latency_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		device->pwrctrl.pm_qos_active_latency);
}


static ssize_t kgsl_pwrctrl_pmqos_wakeup_latency_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	device->pwrctrl.pm_qos_wakeup_latency = val;
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return count;
}

static ssize_t kgsl_pwrctrl_pmqos_wakeup_latency_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		device->pwrctrl.pm_qos_wakeup_latency);
}

static ssize_t kgsl_pwrctrl_gpubusy_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_clk_stats *clkstats;

	if (device == NULL)
		return 0;
	clkstats = &device->pwrctrl.clk_stats;
	ret = snprintf(buf, PAGE_SIZE, "%7d %7d\n",
			clkstats->on_time_old, clkstats->elapsed_old);
	if (!test_bit(KGSL_PWRFLAGS_AXI_ON, &device->pwrctrl.power_flags)) {
		clkstats->on_time_old = 0;
		clkstats->elapsed_old = 0;
	}
	return ret;
}

static ssize_t kgsl_pwrctrl_gputop_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_clk_stats *clkstats;
	int i = 0;
	char *ptr = buf;

	if (device == NULL)
		return 0;
	clkstats = &device->pwrctrl.clk_stats;
	ret = snprintf(buf, PAGE_SIZE, "%7d %7d ", clkstats->on_time_old,
					clkstats->elapsed_old);
	for (i = 0, ptr += ret; i < device->pwrctrl.num_pwrlevels;
							i++, ptr += ret)
		ret = snprintf(ptr, PAGE_SIZE, "%7d ",
						clkstats->old_clock_time[i]);

	if (!test_bit(KGSL_PWRFLAGS_AXI_ON, &device->pwrctrl.power_flags)) {
		clkstats->on_time_old = 0;
		clkstats->elapsed_old = 0;
		for (i = 0; i < KGSL_MAX_PWRLEVELS ; i++)
			clkstats->old_clock_time[i] = 0;
	}
	return (unsigned int) (ptr - buf);
}

static ssize_t kgsl_pwrctrl_gpu_available_frequencies_show(
					struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	int index, num_chars = 0;

	if (device == NULL)
		return 0;
	pwr = &device->pwrctrl;
	for (index = 0; index < pwr->num_pwrlevels - 1; index++)
		num_chars += snprintf(buf + num_chars, PAGE_SIZE, "%d ",
		pwr->pwrlevels[index].gpu_freq);
	buf[num_chars++] = '\n';
	return num_chars;
}

static ssize_t kgsl_pwrctrl_reset_count_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n", device->reset_counter);
}

static void __force_on(struct kgsl_device *device, int flag, int on)
{
	if (on) {
		switch (flag) {
		case KGSL_PWRFLAGS_CLK_ON:
			kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_ON,
				KGSL_STATE_ACTIVE);
			break;
		case KGSL_PWRFLAGS_AXI_ON:
			kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_ON);
			break;
		case KGSL_PWRFLAGS_POWER_ON:
			kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_ON);
			break;
		}
		set_bit(flag, &device->pwrctrl.ctrl_flags);
	} else {
		clear_bit(flag, &device->pwrctrl.ctrl_flags);
	}
}

static ssize_t __force_on_show(struct device *dev,
					struct device_attribute *attr,
					char *buf, int flag)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		test_bit(flag, &device->pwrctrl.ctrl_flags));
}

static ssize_t __force_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count,
					int flag)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	__force_on(device, flag, val);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return count;
}

static ssize_t kgsl_pwrctrl_force_clk_on_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return __force_on_show(dev, attr, buf, KGSL_PWRFLAGS_CLK_ON);
}

static ssize_t kgsl_pwrctrl_force_clk_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return __force_on_store(dev, attr, buf, count, KGSL_PWRFLAGS_CLK_ON);
}

static ssize_t kgsl_pwrctrl_force_bus_on_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return __force_on_show(dev, attr, buf, KGSL_PWRFLAGS_AXI_ON);
}

static ssize_t kgsl_pwrctrl_force_bus_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return __force_on_store(dev, attr, buf, count, KGSL_PWRFLAGS_AXI_ON);
}

static ssize_t kgsl_pwrctrl_force_rail_on_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return __force_on_show(dev, attr, buf, KGSL_PWRFLAGS_POWER_ON);
}

static ssize_t kgsl_pwrctrl_force_rail_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return __force_on_store(dev, attr, buf, count, KGSL_PWRFLAGS_POWER_ON);
}

static ssize_t kgsl_pwrctrl_bus_split_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		device->pwrctrl.bus_control);
}

static ssize_t kgsl_pwrctrl_bus_split_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val = 0;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	int ret;

	if (device == NULL)
		return 0;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	device->pwrctrl.bus_control = val ? true : false;
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return count;
}

static ssize_t kgsl_pwrctrl_default_pwrlevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	if (device == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		device->pwrctrl.default_pwrlevel);
}

static ssize_t kgsl_pwrctrl_default_pwrlevel_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr;
	struct kgsl_pwrscale *pwrscale;
	int ret;
	unsigned int level = 0;

	if (device == NULL)
		return 0;

	pwr = &device->pwrctrl;
	pwrscale = &device->pwrscale;

	ret = kgsl_sysfs_store(buf, &level);
	if (ret)
		return ret;

	if (level > pwr->num_pwrlevels - 2)
		goto done;

	mutex_lock(&device->mutex);
	pwr->default_pwrlevel = level;
	pwrscale->profile.initial_freq
			= pwr->pwrlevels[level].gpu_freq;

	mutex_unlock(&device->mutex);
done:
	return count;
}

static DEVICE_ATTR(gpuclk, 0644, kgsl_pwrctrl_gpuclk_show,
	kgsl_pwrctrl_gpuclk_store);
static DEVICE_ATTR(max_gpuclk, 0644, kgsl_pwrctrl_max_gpuclk_show,
	kgsl_pwrctrl_max_gpuclk_store);
static DEVICE_ATTR(idle_timer, 0644, kgsl_pwrctrl_idle_timer_show,
	kgsl_pwrctrl_idle_timer_store);
static DEVICE_ATTR(gpubusy, 0444, kgsl_pwrctrl_gpubusy_show,
	NULL);
static DEVICE_ATTR(gputop, 0444, kgsl_pwrctrl_gputop_show,
	NULL);
static DEVICE_ATTR(gpu_available_frequencies, 0444,
	kgsl_pwrctrl_gpu_available_frequencies_show,
	NULL);
static DEVICE_ATTR(max_pwrlevel, 0644,
	kgsl_pwrctrl_max_pwrlevel_show,
	kgsl_pwrctrl_max_pwrlevel_store);
static DEVICE_ATTR(min_pwrlevel, 0644,
	kgsl_pwrctrl_min_pwrlevel_show,
	kgsl_pwrctrl_min_pwrlevel_store);
static DEVICE_ATTR(active_pwrlevel, 0444,
	kgsl_pwrctrl_active_pwrlevel_show,
	NULL);
static DEVICE_ATTR(thermal_pwrlevel, 0644,
	kgsl_pwrctrl_thermal_pwrlevel_show,
	kgsl_pwrctrl_thermal_pwrlevel_store);
static DEVICE_ATTR(num_pwrlevels, 0444,
	kgsl_pwrctrl_num_pwrlevels_show,
	NULL);
static DEVICE_ATTR(pmqos_active_latency, 0644,
	kgsl_pwrctrl_pmqos_active_latency_show,
	kgsl_pwrctrl_pmqos_active_latency_store);
static DEVICE_ATTR(pmqos_wakeup_latency, 0644,
	kgsl_pwrctrl_pmqos_wakeup_latency_show,
	kgsl_pwrctrl_pmqos_wakeup_latency_store);
static DEVICE_ATTR(reset_count, 0444,
	kgsl_pwrctrl_reset_count_show,
	NULL);
static DEVICE_ATTR(force_clk_on, 0644,
	kgsl_pwrctrl_force_clk_on_show,
	kgsl_pwrctrl_force_clk_on_store);
static DEVICE_ATTR(force_bus_on, 0644,
	kgsl_pwrctrl_force_bus_on_show,
	kgsl_pwrctrl_force_bus_on_store);
static DEVICE_ATTR(force_rail_on, 0644,
	kgsl_pwrctrl_force_rail_on_show,
	kgsl_pwrctrl_force_rail_on_store);
static DEVICE_ATTR(bus_split, 0644,
	kgsl_pwrctrl_bus_split_show,
	kgsl_pwrctrl_bus_split_store);
static DEVICE_ATTR(default_pwrlevel, 0644,
	kgsl_pwrctrl_default_pwrlevel_show,
	kgsl_pwrctrl_default_pwrlevel_store);

static const struct device_attribute *pwrctrl_attr_list[] = {
	&dev_attr_gpuclk,
	&dev_attr_max_gpuclk,
	&dev_attr_idle_timer,
	&dev_attr_gpubusy,
	&dev_attr_gputop,
	&dev_attr_gpu_available_frequencies,
	&dev_attr_max_pwrlevel,
	&dev_attr_min_pwrlevel,
	&dev_attr_active_pwrlevel,
	&dev_attr_thermal_pwrlevel,
	&dev_attr_num_pwrlevels,
	&dev_attr_pmqos_active_latency,
	&dev_attr_pmqos_wakeup_latency,
	&dev_attr_reset_count,
	&dev_attr_force_clk_on,
	&dev_attr_force_bus_on,
	&dev_attr_force_rail_on,
	&dev_attr_bus_split,
	&dev_attr_default_pwrlevel,
	NULL
};

int kgsl_pwrctrl_init_sysfs(struct kgsl_device *device)
{
	return kgsl_create_device_sysfs_files(device->dev, pwrctrl_attr_list);
}

void kgsl_pwrctrl_uninit_sysfs(struct kgsl_device *device)
{
	kgsl_remove_device_sysfs_files(device->dev, pwrctrl_attr_list);
}

static void update_statistics(struct kgsl_device *device)
{
	struct kgsl_clk_stats *clkstats = &device->pwrctrl.clk_stats;
	unsigned int on_time = 0;
	int i;
	int num_pwrlevels = device->pwrctrl.num_pwrlevels - 1;
	/*PER CLK TIME*/
	for (i = 0; i < num_pwrlevels; i++) {
		clkstats->old_clock_time[i] = clkstats->clock_time[i];
		on_time += clkstats->clock_time[i];
		clkstats->clock_time[i] = 0;
	}
	clkstats->old_clock_time[num_pwrlevels] =
				clkstats->clock_time[num_pwrlevels];
	clkstats->clock_time[num_pwrlevels] = 0;
	clkstats->on_time_old = on_time;
	clkstats->elapsed_old = clkstats->elapsed;
	clkstats->elapsed = 0;

	trace_kgsl_gpubusy(device, clkstats->on_time_old,
		clkstats->elapsed_old);
}

/* Track the amount of time the gpu is on vs the total system time. *
 * Regularly update the percentage of busy time displayed by sysfs. */
static void kgsl_pwrctrl_busy_time(struct kgsl_device *device, bool on_time)
{
	struct kgsl_clk_stats *clkstats = &device->pwrctrl.clk_stats;
	update_clk_statistics(device, on_time);
	/* Update the output regularly and reset the counters. */
	if ((clkstats->elapsed > UPDATE_BUSY_VAL) ||
		!test_bit(KGSL_PWRFLAGS_AXI_ON, &device->pwrctrl.power_flags)) {
		update_statistics(device);
	}
}

void kgsl_pwrctrl_clk(struct kgsl_device *device, int state,
					  int requested_state)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int i = 0;

	if (test_bit(KGSL_PWRFLAGS_CLK_ON, &pwr->ctrl_flags))
		return;

	if (state == KGSL_PWRFLAGS_OFF) {
		if (test_and_clear_bit(KGSL_PWRFLAGS_CLK_ON,
			&pwr->power_flags)) {
			trace_kgsl_clk(device, state);
			for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
				if (pwr->grp_clks[i])
					clk_disable(pwr->grp_clks[i]);
			/* High latency clock maintenance. */
			if ((pwr->pwrlevels[0].gpu_freq > 0) &&
				(requested_state != KGSL_STATE_NAP)) {
				for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
					if (pwr->grp_clks[i])
						clk_unprepare(pwr->grp_clks[i]);
				clk_set_rate(pwr->grp_clks[0],
					pwr->pwrlevels[pwr->num_pwrlevels - 1].
					gpu_freq);
			}
			kgsl_pwrctrl_busy_time(device, true);
		} else if (requested_state == KGSL_STATE_SLEEP) {
			/* High latency clock maintenance. */
			for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
				if (pwr->grp_clks[i])
					clk_unprepare(pwr->grp_clks[i]);
			if ((pwr->pwrlevels[0].gpu_freq > 0))
				clk_set_rate(pwr->grp_clks[0],
					pwr->pwrlevels[pwr->num_pwrlevels - 1].
					gpu_freq);
		}
	} else if (state == KGSL_PWRFLAGS_ON) {
		if (!test_and_set_bit(KGSL_PWRFLAGS_CLK_ON,
			&pwr->power_flags)) {
			trace_kgsl_clk(device, state);
			/* High latency clock maintenance. */
			if (device->state != KGSL_STATE_NAP) {
				if (pwr->pwrlevels[0].gpu_freq > 0)
					clk_set_rate(pwr->grp_clks[0],
						pwr->pwrlevels
						[pwr->active_pwrlevel].
						gpu_freq);
				for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
					if (pwr->grp_clks[i])
						clk_prepare(pwr->grp_clks[i]);
			}
			/* as last step, enable grp_clk
			   this is to let GPU interrupt to come */
			for (i = KGSL_MAX_CLKS - 1; i > 0; i--)
				if (pwr->grp_clks[i])
					clk_enable(pwr->grp_clks[i]);
			kgsl_pwrctrl_busy_time(device, false);
		}
	}
}

static void kgsl_pwrctrl_axi(struct kgsl_device *device, int state)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if (test_bit(KGSL_PWRFLAGS_AXI_ON, &pwr->ctrl_flags))
		return;

	if (state == KGSL_PWRFLAGS_OFF) {
		if (test_and_clear_bit(KGSL_PWRFLAGS_AXI_ON,
			&pwr->power_flags)) {
			trace_kgsl_bus(device, state);
			kgsl_pwrctrl_buslevel_update(device, false);
		}
	} else if (state == KGSL_PWRFLAGS_ON) {
		if (!test_and_set_bit(KGSL_PWRFLAGS_AXI_ON,
			&pwr->power_flags)) {
			trace_kgsl_bus(device, state);
			kgsl_pwrctrl_buslevel_update(device, true);
		}
	}
}

static void kgsl_pwrctrl_pwrrail(struct kgsl_device *device, int state)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if (test_bit(KGSL_PWRFLAGS_POWER_ON, &pwr->ctrl_flags))
		return;

	if (state == KGSL_PWRFLAGS_OFF) {
		if (test_and_clear_bit(KGSL_PWRFLAGS_POWER_ON,
			&pwr->power_flags)) {
			trace_kgsl_rail(device, state);
			if (pwr->gpu_cx)
				regulator_disable(pwr->gpu_cx);
			if (pwr->gpu_reg)
				regulator_disable(pwr->gpu_reg);
		}
	} else if (state == KGSL_PWRFLAGS_ON) {
		if (!test_and_set_bit(KGSL_PWRFLAGS_POWER_ON,
			&pwr->power_flags)) {
			trace_kgsl_rail(device, state);
			if (pwr->gpu_reg) {
				int status = regulator_enable(pwr->gpu_reg);
				if (status)
					KGSL_DRV_ERR(device,
							"core regulator_enable "
							"failed: %d\n",
							status);
			}
			if (pwr->gpu_cx) {
				int status = regulator_enable(pwr->gpu_cx);
				if (status)
					KGSL_DRV_ERR(device,
							"cx regulator_enable "
							"failed: %d\n",
							status);
			}
		}
	}
}

void kgsl_pwrctrl_irq(struct kgsl_device *device, int state)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if (state == KGSL_PWRFLAGS_ON) {
		if (!test_and_set_bit(KGSL_PWRFLAGS_IRQ_ON,
			&pwr->power_flags)) {
			trace_kgsl_irq(device, state);
			enable_irq(pwr->interrupt_num);
		}
	} else if (state == KGSL_PWRFLAGS_OFF) {
		if (test_and_clear_bit(KGSL_PWRFLAGS_IRQ_ON,
			&pwr->power_flags)) {
			trace_kgsl_irq(device, state);
			if (in_interrupt())
				disable_irq_nosync(pwr->interrupt_num);
			else
				disable_irq(pwr->interrupt_num);
		}
	}
}
EXPORT_SYMBOL(kgsl_pwrctrl_irq);

int kgsl_pwrctrl_init(struct kgsl_device *device)
{
	int i, k, m, n = 0, result = 0;
	unsigned int freq_i;
	struct clk *clk;
	struct platform_device *pdev =
		container_of(device->parentdev, struct platform_device, dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_device_platform_data *pdata = pdev->dev.platform_data;

	/*acquire clocks */
	for (i = 0; i < KGSL_MAX_CLKS; i++) {
		if (pdata->clk_map & clks[i].map) {
			clk = clk_get(&pdev->dev, clks[i].name);
			if (IS_ERR(clk))
				goto clk_err;
			pwr->grp_clks[i] = clk;
		}
	}
	/* Make sure we have a source clk for freq setting */
	if (pwr->grp_clks[0] == NULL)
		pwr->grp_clks[0] = pwr->grp_clks[1];

	if (pdata->num_levels > KGSL_MAX_PWRLEVELS ||
	    pdata->num_levels < 1) {
		KGSL_PWR_ERR(device, "invalid power level count: %d\n",
					 pdata->num_levels);
		result = -EINVAL;
		goto done;
	}
	pwr->num_pwrlevels = pdata->num_levels;

	/* Initialize the user and thermal clock constraints */

	pwr->max_pwrlevel = 0;
	pwr->min_pwrlevel = pdata->num_levels - 2;
	pwr->thermal_pwrlevel = 0;

	pwr->active_pwrlevel = pdata->init_level;
	pwr->default_pwrlevel = pdata->init_level;
	pwr->init_pwrlevel = pdata->init_level;
	pwr->wakeup_maxpwrlevel = 0;
	for (i = 0; i < pdata->num_levels; i++) {
		pwr->pwrlevels[i].gpu_freq =
		(pdata->pwrlevel[i].gpu_freq > 0) ?
		clk_round_rate(pwr->grp_clks[0],
					   pdata->pwrlevel[i].
					   gpu_freq) : 0;
		pwr->pwrlevels[i].bus_freq =
			pdata->pwrlevel[i].bus_freq;
		pwr->pwrlevels[i].io_fraction =
			pdata->pwrlevel[i].io_fraction;
	}
	/* Do not set_rate for targets in sync with AXI */
	if (pwr->pwrlevels[0].gpu_freq > 0)
		clk_set_rate(pwr->grp_clks[0], pwr->
				pwrlevels[pwr->num_pwrlevels - 1].gpu_freq);

	pwr->gpu_reg = regulator_get(&pdev->dev, "vdd");
	if (IS_ERR(pwr->gpu_reg))
		pwr->gpu_reg = NULL;

	if (pwr->gpu_reg) {
		pwr->gpu_cx = regulator_get(&pdev->dev, "vddcx");
		if (IS_ERR(pwr->gpu_cx))
			pwr->gpu_cx = NULL;
	} else
		pwr->gpu_cx = NULL;

	pwr->power_flags = 0;

	pwr->interval_timeout = msecs_to_jiffies(pdata->idle_timeout);
	pwr->strtstp_sleepwake = pdata->strtstp_sleepwake;

	pwr->pm_qos_active_latency = pdata->pm_qos_active_latency;
	pwr->pm_qos_wakeup_latency = pdata->pm_qos_wakeup_latency;

	pm_runtime_enable(device->parentdev);

	if (pdata->bus_scale_table == NULL)
		return result;

	pwr->pcl = msm_bus_scale_register_client(pdata->
						bus_scale_table);
	if (!pwr->pcl) {
		KGSL_PWR_ERR(device,
				"msm_bus_scale_register_client failed: "
				"id %d table %p", device->id,
				pdata->bus_scale_table);
		result = -EINVAL;
		goto done;
	}

	/* Set if independent bus BW voting is supported */
	pwr->bus_control = pdata->bus_control;

	/*
	 * Set the range permitted for BIMC votes per-GPU frequency.
	 * For the moment assume the BIMC votes are listed in order
	 * per GPU frequency.  If this is no longer needed in the bus
	 * table the min/max values can be explicitly set in the dtsi
	 * file.
	 */
	freq_i = pwr->min_pwrlevel;
	pwr->pwrlevels[freq_i].bus_min = 1;

	/*
	 * Pull the BW vote out of the bus table.  They will be used to
	 * calculate the ratio between the votes.
	 */
	for (i = 0; i < pdata->bus_scale_table->num_usecases; i++) {
		struct msm_bus_paths *usecase =
				&pdata->bus_scale_table->usecase[i];
		struct msm_bus_vectors *vector = &usecase->vectors[0];
		if (vector->dst == MSM_BUS_SLAVE_EBI_CH0 &&
				vector->ib != 0) {
			for (k = 0; k < n; k++)
				if (vector->ib == pwr->bus_ib[k]) {
					static uint64_t last_ib = 0xFFFFFFFF;
					if (vector->ib <= last_ib) {
						pwr->pwrlevels[freq_i--].
							bus_max = i - 1;
						pwr->pwrlevels[freq_i].
							bus_min = i;
					}
					last_ib = vector->ib;
					break;
				}
			/* if this is a new ib value, save it */
			if (k == n) {
				pwr->bus_ib[k] = vector->ib;
				n++;
				/* find which pwrlevels use this ib */
				for (m = 0; m < pwr->num_pwrlevels - 1; m++) {
					if (pdata->bus_scale_table->
						usecase[pwr->pwrlevels[m].
						bus_freq].vectors[0].ib
						== vector->ib)
						pwr->bus_index[m] = k;
				}
			}
		}
	}
	pwr->pwrlevels[freq_i].bus_max = i - 1;

	return result;

clk_err:
	result = PTR_ERR(clk);
	KGSL_PWR_ERR(device, "clk_get(%s) failed: %d\n",
				 clks[i].name, result);

done:
	return result;
}

void kgsl_pwrctrl_close(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int i;

	KGSL_PWR_INFO(device, "close device %d\n", device->id);

	pm_runtime_disable(device->parentdev);

	if (pwr->pcl)
		msm_bus_scale_unregister_client(pwr->pcl);

	pwr->pcl = 0;

	if (pwr->gpu_reg) {
		regulator_put(pwr->gpu_reg);
		pwr->gpu_reg = NULL;
	}

	if (pwr->gpu_cx) {
		regulator_put(pwr->gpu_cx);
		pwr->gpu_cx = NULL;
	}

	for (i = 1; i < KGSL_MAX_CLKS; i++)
		if (pwr->grp_clks[i]) {
			clk_put(pwr->grp_clks[i]);
			pwr->grp_clks[i] = NULL;
		}

	pwr->grp_clks[0] = NULL;
	pwr->power_flags = 0;
}

/**
 * kgsl_idle_check() - Work function for GPU interrupts and idle timeouts.
 * @device: The device
 *
 * This function is called for work that is queued by the interrupt
 * handler or the idle timer. It attempts to transition to a clocks
 * off state if the active_cnt is 0 and the hardware is idle.
 */
void kgsl_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work, struct kgsl_device,
							idle_check_ws);
	WARN_ON(device == NULL);
	if (device == NULL)
		return;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	if (device->state == KGSL_STATE_ACTIVE
		   || device->state ==  KGSL_STATE_NAP) {

		if (!atomic_read(&device->active_cnt))
			kgsl_pwrctrl_sleep(device);

		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		if (device->state == KGSL_STATE_ACTIVE) {
			mod_timer(&device->idle_timer,
					jiffies +
					device->pwrctrl.interval_timeout);
			/*
			 * If the GPU has been too busy to sleep, make sure
			 * that is acurately reflected in the % busy numbers.
			 */
			device->pwrctrl.clk_stats.no_nap_cnt++;
			if (device->pwrctrl.clk_stats.no_nap_cnt >
							 UPDATE_BUSY) {
				kgsl_pwrctrl_busy_time(device, true);
				device->pwrctrl.clk_stats.no_nap_cnt = 0;
			}
		}
	}

	kgsl_pwrscale_update(device);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
}
EXPORT_SYMBOL(kgsl_idle_check);

void kgsl_timer(unsigned long data)
{
	struct kgsl_device *device = (struct kgsl_device *) data;

	KGSL_PWR_INFO(device, "idle timer expired device %d\n", device->id);
	if (device->requested_state != KGSL_STATE_SUSPEND) {
		if (device->pwrctrl.strtstp_sleepwake)
			kgsl_pwrctrl_request_state(device, KGSL_STATE_SLUMBER);
		else
			kgsl_pwrctrl_request_state(device, KGSL_STATE_SLEEP);
		/* Have work run in a non-interrupt context. */
		queue_work(device->work_queue, &device->idle_check_ws);
	}
}

bool kgsl_pwrctrl_isenabled(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return ((test_bit(KGSL_PWRFLAGS_CLK_ON, &pwr->power_flags) != 0) &&
		(test_bit(KGSL_PWRFLAGS_AXI_ON, &pwr->power_flags) != 0));
}

/**
 * kgsl_pre_hwaccess - Enforce preconditions for touching registers
 * @device: The device
 *
 * This function ensures that the correct lock is held and that the GPU
 * clock is on immediately before a register is read or written. Note
 * that this function does not check active_cnt because the registers
 * must be accessed during device start and stop, when the active_cnt
 * may legitimately be 0.
 */
void kgsl_pre_hwaccess(struct kgsl_device *device)
{
	/* In order to touch a register you must hold the device mutex...*/
	BUG_ON(!mutex_is_locked(&device->mutex));
	/* and have the clock on! */
	BUG_ON(!kgsl_pwrctrl_isenabled(device));
}
EXPORT_SYMBOL(kgsl_pre_hwaccess);

static int
_nap(struct kgsl_device *device)
{
	switch (device->state) {
	case KGSL_STATE_ACTIVE:
		if (!device->ftbl->isidle(device)) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
			return -EBUSY;
		}

		/*
		 * Read HW busy counters before going to NAP state.
		 * The data might be used by power scale governors
		 * independently of the HW activity. For example
		 * the simple-on-demand governor will get the latest
		 * busy_time data even if the gpu isn't active.
		*/
		kgsl_pwrscale_update_stats(device);

		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
		kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_OFF, KGSL_STATE_NAP);
		kgsl_pwrctrl_set_state(device, KGSL_STATE_NAP);
	case KGSL_STATE_NAP:
	case KGSL_STATE_SLEEP:
	case KGSL_STATE_SLUMBER:
		break;
	default:
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		break;
	}
	return 0;
}

static void
_sleep_accounting(struct kgsl_device *device)
{
	kgsl_pwrctrl_busy_time(device, false);
	device->pwrctrl.clk_stats.start = ktime_set(0, 0);

	kgsl_pwrscale_sleep(device);
}

static int
_sleep(struct kgsl_device *device)
{
	switch (device->state) {
	case KGSL_STATE_ACTIVE:
		if (!device->ftbl->isidle(device)) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
			return -EBUSY;
		}
		/* fall through */
	case KGSL_STATE_NAP:
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
		kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_OFF);
		_sleep_accounting(device);
		kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_OFF, KGSL_STATE_SLEEP);
		kgsl_pwrctrl_set_state(device, KGSL_STATE_SLEEP);
		pm_qos_update_request(&device->pwrctrl.pm_qos_req_dma,
					PM_QOS_DEFAULT_VALUE);
		break;
	case KGSL_STATE_SLEEP:
	case KGSL_STATE_SLUMBER:
		break;
	default:
		KGSL_PWR_WARN(device, "unhandled state %s\n",
				kgsl_pwrstate_to_str(device->state));
		break;
	}

	return 0;
}

static int
_slumber(struct kgsl_device *device)
{
	switch (device->state) {
	case KGSL_STATE_ACTIVE:
		if (!device->ftbl->isidle(device)) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
			return -EBUSY;
		}
		/* fall through */
	case KGSL_STATE_NAP:
	case KGSL_STATE_SLEEP:
		del_timer_sync(&device->idle_timer);
		/* make sure power is on to stop the device*/
		kgsl_pwrctrl_enable(device);
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_ON);
		device->ftbl->suspend_context(device);
		device->ftbl->stop(device);
		_sleep_accounting(device);
		kgsl_pwrctrl_set_state(device, KGSL_STATE_SLUMBER);
		pm_qos_update_request(&device->pwrctrl.pm_qos_req_dma,
						PM_QOS_DEFAULT_VALUE);
		break;
	case KGSL_STATE_SLUMBER:
		break;
	default:
		KGSL_PWR_WARN(device, "unhandled state %s\n",
				kgsl_pwrstate_to_str(device->state));
		break;
	}
	return 0;
}

/******************************************************************/
/* Caller must hold the device mutex. */
int kgsl_pwrctrl_sleep(struct kgsl_device *device)
{
	int status = 0;
	KGSL_PWR_INFO(device, "sleep device %d\n", device->id);

	/* Work through the legal state transitions */
	switch (device->requested_state) {
	case KGSL_STATE_NAP:
		status = _nap(device);
		break;
	case KGSL_STATE_SLEEP:
		status = _sleep(device);
		break;
	case KGSL_STATE_SLUMBER:
		status = _slumber(device);
		break;
	default:
		KGSL_PWR_INFO(device, "bad state request 0x%x\n",
				device->requested_state);
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		status = -EINVAL;
		break;
	}
	return status;
}
EXPORT_SYMBOL(kgsl_pwrctrl_sleep);

/**
 * kgsl_pwrctrl_wake() - Power up the GPU from a slumber/sleep state
 * @device - Pointer to the kgsl_device struct
 * @priority - Boolean flag to indicate that the GPU start should be run in the
 * higher priority thread
 *
 * Resume the GPU from a lower power state to ACTIVE.  The caller to this
 * fucntion must host the kgsl_device mutex.
 */
int kgsl_pwrctrl_wake(struct kgsl_device *device, int priority)
{
	int status = 0;
	unsigned int context_id;
	unsigned int state = device->state;
	unsigned int ts_processed = 0xdeaddead;
	struct kgsl_context *context;

	kgsl_pwrctrl_request_state(device, KGSL_STATE_ACTIVE);
	switch (device->state) {
	case KGSL_STATE_SLUMBER:
		status = device->ftbl->start(device, priority);

		if (status) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
			KGSL_DRV_ERR(device, "start failed %d\n", status);
			break;
		}
		/* fall through */
	case KGSL_STATE_SLEEP:
		kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_ON);
		kgsl_pwrscale_wake(device);
		kgsl_sharedmem_readl(&device->memstore,
			(unsigned int *) &context_id,
			KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
				current_context));
		context = kgsl_context_get(device, context_id);
		if (context)
			kgsl_readtimestamp(device, context,
				KGSL_TIMESTAMP_RETIRED,
				&ts_processed);
		KGSL_PWR_INFO(device, "Wake from %s state. CTXT: %d RTRD TS: %08X\n",
			kgsl_pwrstate_to_str(state),
			context ? context->id : -1, ts_processed);
		kgsl_context_put(context);
		/* fall through */
	case KGSL_STATE_NAP:
		/* Turn on the core clocks */
		kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_ON, KGSL_STATE_ACTIVE);
		/* Enable state before turning on irq */
		kgsl_pwrctrl_set_state(device, KGSL_STATE_ACTIVE);
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_ON);
		mod_timer(&device->idle_timer, jiffies +
				device->pwrctrl.interval_timeout);
	case KGSL_STATE_ACTIVE:
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		break;
	default:
		KGSL_PWR_WARN(device, "unhandled state %s\n",
				kgsl_pwrstate_to_str(device->state));
		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
		status = -EINVAL;
		break;
	}
	return status;
}
EXPORT_SYMBOL(kgsl_pwrctrl_wake);

void kgsl_pwrctrl_enable(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int level;

	if (pwr->wakeup_maxpwrlevel) {
		level = pwr->max_pwrlevel;
		pwr->wakeup_maxpwrlevel = 0;
	} else
		level = pwr->default_pwrlevel;

	pwr->bus_mod = 0;

	if (pwr->constraint.type == KGSL_CONSTRAINT_NONE)
		kgsl_pwrctrl_pwrlevel_change(device, level);

	/* Order pwrrail/clk sequence based upon platform */
	kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_ON);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_ON, KGSL_STATE_ACTIVE);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_ON);
}
EXPORT_SYMBOL(kgsl_pwrctrl_enable);

void kgsl_pwrctrl_disable(struct kgsl_device *device)
{
	/* Order pwrrail/clk sequence based upon platform */
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_OFF);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_OFF, KGSL_STATE_SLEEP);
	kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_OFF);
}
EXPORT_SYMBOL(kgsl_pwrctrl_disable);

void kgsl_pwrctrl_set_state(struct kgsl_device *device, unsigned int state)
{
	trace_kgsl_pwr_set_state(device, state);
	device->state = state;
	device->requested_state = KGSL_STATE_NONE;
}
EXPORT_SYMBOL(kgsl_pwrctrl_set_state);

void kgsl_pwrctrl_request_state(struct kgsl_device *device, unsigned int state)
{
	if (state != KGSL_STATE_NONE && state != device->requested_state)
		trace_kgsl_pwr_request_state(device, state);
	device->requested_state = state;
}
EXPORT_SYMBOL(kgsl_pwrctrl_request_state);

const char *kgsl_pwrstate_to_str(unsigned int state)
{
	switch (state) {
	case KGSL_STATE_NONE:
		return "NONE";
	case KGSL_STATE_INIT:
		return "INIT";
	case KGSL_STATE_ACTIVE:
		return "ACTIVE";
	case KGSL_STATE_NAP:
		return "NAP";
	case KGSL_STATE_SLEEP:
		return "SLEEP";
	case KGSL_STATE_SUSPEND:
		return "SUSPEND";
	case KGSL_STATE_SLUMBER:
		return "SLUMBER";
	default:
		break;
	}
	return "UNKNOWN";
}
EXPORT_SYMBOL(kgsl_pwrstate_to_str);


/**
 * kgsl_active_count_get() - Increase the device active count
 * @device: Pointer to a KGSL device
 *
 * Increase the active count for the KGSL device and turn on
 * clocks if this is the first reference. Code paths that need
 * to touch the hardware or wait for the hardware to complete
 * an operation must hold an active count reference until they
 * are finished. An error code will be returned if waking the
 * device fails. The device mutex must be held while *calling
 * this function.
 */
int kgsl_active_count_get(struct kgsl_device *device)
{
	int ret = 0;
	BUG_ON(!mutex_is_locked(&device->mutex));

	if ((atomic_read(&device->active_cnt) == 0) &&
		(device->state != KGSL_STATE_ACTIVE)) {
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
		wait_for_completion(&device->hwaccess_gate);
		kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

		ret = kgsl_pwrctrl_wake(device, 1);
	}
	if (ret == 0)
		atomic_inc(&device->active_cnt);
	trace_kgsl_active_count(device,
		(unsigned long) __builtin_return_address(0));
	return ret;
}
EXPORT_SYMBOL(kgsl_active_count_get);

/**
 * kgsl_active_count_put() - Decrease the device active count
 * @device: Pointer to a KGSL device
 *
 * Decrease the active count for the KGSL device and turn off
 * clocks if there are no remaining references. This function will
 * transition the device to NAP if there are no other pending state
 * changes. It also completes the suspend gate.  The device mutex must
 * be held while calling this function.
 */
void kgsl_active_count_put(struct kgsl_device *device)
{
	BUG_ON(!mutex_is_locked(&device->mutex));
	BUG_ON(atomic_read(&device->active_cnt) == 0);

	if (atomic_dec_and_test(&device->active_cnt)) {
		if (device->state == KGSL_STATE_ACTIVE &&
			device->requested_state == KGSL_STATE_NONE) {
			kgsl_pwrctrl_request_state(device, KGSL_STATE_NAP);
			queue_work(device->work_queue, &device->idle_check_ws);
		}

		mod_timer(&device->idle_timer,
			jiffies + device->pwrctrl.interval_timeout);
	}

	trace_kgsl_active_count(device,
		(unsigned long) __builtin_return_address(0));

	wake_up(&device->active_cnt_wq);
}
EXPORT_SYMBOL(kgsl_active_count_put);

static int _check_active_count(struct kgsl_device *device, int count)
{
	/* Return 0 if the active count is greater than the desired value */
	return atomic_read(&device->active_cnt) > count ? 0 : 1;
}

/**
 * kgsl_active_count_wait() - Wait for activity to finish.
 * @device: Pointer to a KGSL device
 * @count: Active count value to wait for
 *
 * Block until the active_cnt value hits the desired value
 */
int kgsl_active_count_wait(struct kgsl_device *device, int count)
{
	int result = 0;
	long wait_jiffies = HZ;

	BUG_ON(!mutex_is_locked(&device->mutex));

	while (atomic_read(&device->active_cnt) > count) {
		long ret;
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
		ret = wait_event_timeout(device->active_cnt_wq,
			_check_active_count(device, count), msecs_to_jiffies(1000));
		kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
		result = ret == 0 ? -ETIMEDOUT : 0;
		if (!result)
			wait_jiffies = ret;
		else
			break;
	}

	return result;
}
EXPORT_SYMBOL(kgsl_active_count_wait);
