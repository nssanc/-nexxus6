/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2015 Francisco Franco
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/hrtimer.h>

unsigned int TEMP_THRESHOLD = 70;
unsigned int FREQ_HELL = 960000;
unsigned int FREQ_VERY_HOT = 1267200;
unsigned int FREQ_HOT = 1728000;
unsigned int FREQ_WARM = 2265600;


/* temp_threshold */
static int set_temp_threshold(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 40 || i > 90)
		return -EINVAL;
	
	TEMP_THRESHOLD = i;
	printk("MSM_THERMAL: Temperatrue threshold set to: %x", TEMP_THRESHOLD);
	ret = param_set_uint(val, kp);

	return ret;
}

static struct kernel_param_ops temp_threshold_ops = {
	.set = set_temp_threshold,
	.get = param_get_uint,
};

module_param_cb(TEMP_THRESHOLD, &temp_threshold_ops, &TEMP_THRESHOLD, 0644);

/* FREQ_HELL */
static int set_freq_hell(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;
	
	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	// need to figure out how to verify that the value being set is a valid cpu freq. 
	// not sure how to instantitate the cpufreqtable though to execute this call.
	//if (!cpufreq_verify_within_limits(0, i, i))
	//	return -EINVAL;
	
	FREQ_HELL = i;
	printk("MSM_THERMAL: Freq_Hell limit set to: %x", FREQ_HELL);
	ret = param_set_uint(val, kp);

	return ret;
}

static struct kernel_param_ops freq_hell_ops = {
	.set = set_freq_hell,
	.get = param_get_uint,
};

module_param_cb(FREQ_HELL, &freq_hell_ops, &FREQ_HELL, 0644);

/* FREQ_VERY_HOT */
static int set_freq_very_hot(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;
	
	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	// need to figure out how to verify that the value being set is a valid cpu freq. 
	// not sure how to instantitate the cpufreqtable though to execute this call.
	//if (!cpufreq_verify_within_limits(0, i, i))
	//	return -EINVAL;
	
	FREQ_VERY_HOT = i;
	printk("MSM_THERMAL: Freq_Very_Hot limit set to: %x", FREQ_VERY_HOT);
	ret = param_set_uint(val, kp);

	return ret;
}

static struct kernel_param_ops freq_very_hot_ops = {
	.set = set_freq_very_hot,
	.get = param_get_uint,
};

module_param_cb(FREQ_VERY_HOT, &freq_very_hot_ops, &FREQ_VERY_HOT, 0644);

/* FREQ_HOT */
static int set_freq_hot(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;
	
	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	// need to figure out how to verify that the value being set is a valid cpu freq. 
	// not sure how to instantitate the cpufreqtable though to execute this call.
	//if (!cpufreq_verify_within_limits(0, i, i))
	//	return -EINVAL;
	
	FREQ_HOT = i;
	printk("MSM_THERMAL: Freq_Hot limit set to: %x", FREQ_HOT);
	ret = param_set_uint(val, kp);

	return ret;
}

static struct kernel_param_ops freq_hot_ops = {
	.set = set_freq_hot,
	.get = param_get_uint,
};

module_param_cb(FREQ_HOT, &freq_hot_ops, &FREQ_HOT, 0644);

/* FREQ_WARM */
static int set_freq_warm(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;
	
	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	// need to figure out how to verify that the value being set is a valid cpu freq. 
	// not sure how to instantitate the cpufreqtable though to execute this call.
	//if (!cpufreq_verify_within_limits(0, i, i))
	//	return -EINVAL;
	
	FREQ_WARM = i;
	printk("MSM_THERMAL: Freq_Very_Hot limit set to: %x", FREQ_WARM);
	ret = param_set_uint(val, kp);

	return ret;
}

static struct kernel_param_ops freq_warm_ops = {
	.set = set_freq_warm,
	.get = param_get_uint,
};

module_param_cb(FREQ_WARM, &freq_warm_ops, &FREQ_WARM, 0644);

static struct thermal_info {
	uint32_t cpuinfo_max_freq;
	uint32_t limited_max_freq;
	unsigned int safe_diff;
	bool throttling;
	bool pending_change;
	const int min_interval_us;
	u64 limit_cpu_time;
} info = {
	.cpuinfo_max_freq = LONG_MAX,
	.limited_max_freq = LONG_MAX,
	.safe_diff = 5,
	.throttling = false,
	.pending_change = false,
	/* 1 second */
	.min_interval_us = 1000000,
};

enum threshold_levels {
	LEVEL_HELL		= 1 << 4,
	LEVEL_VERY_HOT	= 1 << 3,
	LEVEL_HOT		= 1 << 2,
};

static struct msm_thermal_data msm_thermal_info;

static struct delayed_work check_temp_work;

static int msm_thermal_cpufreq_callback(struct notifier_block *nfb,
		unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST && !info.pending_change)
		return 0;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
		info.limited_max_freq);

	return 0;
}

static struct notifier_block msm_thermal_cpufreq_notifier = {
	.notifier_call = msm_thermal_cpufreq_callback,
};

static void limit_cpu_freqs(uint32_t max_freq)
{
	unsigned int cpu;

	if (info.limited_max_freq == max_freq)
		return;

	info.limited_max_freq = max_freq;
	info.pending_change = true;
	info.limit_cpu_time = ktime_to_us(ktime_get());

	get_online_cpus();
	for_each_online_cpu(cpu) {
		cpufreq_update_policy(cpu);
		pr_info("%s: Setting cpu%d max frequency to %d\n",
				KBUILD_MODNAME, cpu, info.limited_max_freq);
	}
	put_online_cpus();

	info.pending_change = false;
}

static void check_temp(struct work_struct *work)
{
	struct tsens_device tsens_dev;
	uint32_t freq = 0;
	long temp = 0;
	u64 now;

	tsens_dev.sensor_num = msm_thermal_info.sensor_id;
	tsens_get_temp(&tsens_dev, &temp);

	if (info.throttling)
	{
		if (temp < (TEMP_THRESHOLD - info.safe_diff))
		{
			now = ktime_to_us(ktime_get());

			if (now < (info.limit_cpu_time + info.min_interval_us))
				goto reschedule;

			limit_cpu_freqs(info.cpuinfo_max_freq);
			info.throttling = false;
			goto reschedule;
		}
	}

	if (temp >= TEMP_THRESHOLD + LEVEL_HELL)
		freq = FREQ_HELL;
	else if (temp >= TEMP_THRESHOLD + LEVEL_VERY_HOT)
		freq = FREQ_VERY_HOT;
	else if (temp >= TEMP_THRESHOLD + LEVEL_HOT)
		freq = FREQ_HOT;
	else if (temp > TEMP_THRESHOLD)
		freq = FREQ_WARM;

	if (freq)
	{
		limit_cpu_freqs(freq);

		if (!info.throttling)
			info.throttling = true;
	}

reschedule:
	schedule_delayed_work_on(0, &check_temp_work, msecs_to_jiffies(250));
}

static int msm_thermal_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *node = pdev->dev.of_node;
	struct msm_thermal_data data;

	memset(&data, 0, sizeof(struct msm_thermal_data));

	ret = of_property_read_u32(node, "qcom,sensor-id", &data.sensor_id);
	if (ret)
		return ret;

	WARN_ON(data.sensor_id >= TSENS_MAX_SENSORS);

        memcpy(&msm_thermal_info, &data, sizeof(struct msm_thermal_data));

        INIT_DELAYED_WORK(&check_temp_work, check_temp);
        schedule_delayed_work_on(0, &check_temp_work, 10 * HZ);

	cpufreq_register_notifier(&msm_thermal_cpufreq_notifier,
			CPUFREQ_POLICY_NOTIFIER);

	return ret;
}

static int msm_thermal_dev_remove(struct platform_device *pdev)
{
	cpufreq_unregister_notifier(&msm_thermal_cpufreq_notifier,
                        CPUFREQ_POLICY_NOTIFIER);
	return 0;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,msm-thermal"},
	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.remove = msm_thermal_dev_remove,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
};

int __init  msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}

void __exit msm_thermal_device_exit(void)
{
	platform_driver_unregister(&msm_thermal_device_driver);
}

late_initcall(msm_thermal_device_init);
module_exit(msm_thermal_device_exit);
