/*
 * Copyright 2011 Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/platform_device.h>
#include <plat/omap_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <plat/scm.h>
#include <linux/mfd/omap4_scm.h>
#include <linux/thermal_freq.h>
#include <linux/thermal_framework.h>
#include <linux/thermal.h>

static unsigned long maximum_freq;

static int cd_get_max_state(struct thermal_cooling_device *cdev,
	unsigned long *state)
{
	*state = 1;
	return 0;
}

static int cd_get_cur_state(struct thermal_cooling_device *cdev,
	unsigned long *state)
{
	struct thermal_freq *therm = cdev->devdata;

	*state = therm->state;
	return 0;
}

static int cd_set_cur_state(struct thermal_cooling_device *cdev,
							   unsigned long state)
{
	struct thermal_freq *therm = cdev->devdata;
	struct cpufreq_policy policy;
	int trip, i;

	if (!state && therm->state) {
		if (cpufreq_get_policy(&policy, 0))
			return -EINVAL;
		maximum_freq = policy.cpuinfo.max_freq;
		therm->current_trip = -1;
		therm->state = 0;
		therm->tdev->polling_delay = therm->idle_polling_delay;
		cpufreq_update_policy(0);

		return 0;
	}

	trip = -1;
	therm->state = state;
	for (i = 0; i < therm->trip_count; i++) {
		if (therm->current_temp < therm->trip_table[i].temp)
			break;
		trip = i;
	}

	if (i == therm->current_trip)
		return 0;

	if (cpufreq_get_policy(&policy, 0))
		return -EINVAL;

	if (i < 0) {
		therm->tdev->polling_delay = therm->idle_polling_delay;
		maximum_freq = policy.cpuinfo.max_freq;
	} else {
		therm->tdev->polling_delay =
					 therm->trip_table[i].polling_interval;
		maximum_freq = therm->trip_table[i].freq;
	}
	cpufreq_update_policy(0);
	therm->current_trip = i;

	return 0;
}

static struct thermal_cooling_device_ops thc_ops = {
	.get_max_state = cd_get_max_state,
	.get_cur_state = cd_get_cur_state,
	.set_cur_state = cd_set_cur_state,
};

static int tz_get_mode(struct thermal_zone_device *tz_dev,
						enum thermal_device_mode *mode)
{
	struct thermal_freq *therm = tz_dev->devdata;

	*mode = therm->mode;
	return 0;
}

static int tz_set_mode(struct thermal_zone_device *tz_dev,
						 enum thermal_device_mode mode)
{
	struct thermal_freq *therm = tz_dev->devdata;

	therm->mode = mode;
	return 0;
}

static int tz_get_temp(struct thermal_zone_device *tz_dev, unsigned long *t)
{
	struct thermal_freq *therm = tz_dev->devdata;
	int temp;

	/*
	 * The "0" is copied from the reference code but should probably be
	 * A device nunmber.
	 */
	temp = therm->get_temp(therm->privdata);
	/*
	 * The thermal_zone code assumes temperatures
	 * less than 1C "don't make sense".
	 */
	if (temp < 1000)
		temp = 1000;
	*t = temp;
	therm->current_temp = temp;
	return 0;
}

static int tz_get_trip_type(struct thermal_zone_device *tz_dev, int count,
					  enum thermal_trip_type *tz_trip_type)
{
	struct thermal_freq *therm = tz_dev->devdata;

	/* The last trip entry will be used as the critical temperature */

	if (count < 0 || count >= therm->trip_count)
		return -EINVAL;
	if (count == (therm->trip_count-1))
		*tz_trip_type = THERMAL_TRIP_CRITICAL;
	else
		*tz_trip_type = THERMAL_TRIP_ACTIVE;

	return 0;
}

static int tz_get_trip_temp(struct thermal_zone_device *tz_dev, int count,
							      unsigned long *t)
{
	struct thermal_freq *therm = tz_dev->devdata;

	if (count < 0 || count >= therm->trip_count)
		return -EINVAL;
	*t = therm->trip_table[count].temp;
	return 0;
}

static int tz_get_crit_temp(struct thermal_zone_device *tz_dev,
							   unsigned long *temp)
{
	struct thermal_freq *therm = tz_dev->devdata;

	if (!therm->trip_table || therm->trip_count <= 0)
		return -EINVAL;

	/*
 	 * The last entry in the trip table will always
 	 * be used as the critical temp
 	 */
	*temp = therm->trip_table[therm->trip_count - 1].temp;

	return 0;
}

static int tz_bind(struct thermal_zone_device *tz_dev,
	struct thermal_cooling_device *cdev)
{
	if (!cdev->ops)
		return 0;
	if (cdev->ops->get_max_state != cd_get_max_state)
		return 0;

	if (thermal_zone_bind_cooling_device(tz_dev, 0, cdev))
		return -EINVAL;

	return 0;
}

static struct thermal_zone_device_ops thz_ops = {
	.get_temp = tz_get_temp,
	.get_mode = tz_get_mode,
	.set_mode = tz_set_mode,
	.get_trip_type = tz_get_trip_type,
	.get_trip_temp = tz_get_trip_temp,
	.get_crit_temp = tz_get_crit_temp,
	.bind = tz_bind,
};

static int thermal_notify(struct notifier_block *block, unsigned long event,
	void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST)
		return 0;

	if (maximum_freq)
		cpufreq_verify_within_limits(policy, 0, maximum_freq);

	return 0;
}

static struct notifier_block thermal_cpufreq_notifier_block = {
	.notifier_call = thermal_notify,
};

struct thermal_freq *thermal_freq_register(char *domain,
	int (*get_temp)(void *), void *sensordata,
	struct thermal_freq_table *trip_table, int table_size,
	int polling_interval)
{
	struct thermal_freq *therm;

	if (table_size <= 0)
		return NULL;
	therm = kzalloc(sizeof(struct thermal_freq), GFP_KERNEL);
	therm->get_temp = get_temp;
	therm->privdata = sensordata;
	therm->trip_table = trip_table;
	therm->trip_count = table_size;
	therm->state = -1;
	therm->idle_polling_delay = polling_interval;
	therm->cdev = thermal_cooling_device_register(domain, therm,
		&thc_ops);
	therm->tdev = thermal_zone_device_register(domain, table_size, therm,
		&thz_ops, 1, 1, polling_interval, polling_interval);
	cpufreq_register_notifier(&thermal_cpufreq_notifier_block,
		CPUFREQ_POLICY_NOTIFIER);
	return therm;
}

void thermal_freq_register_unregister(struct thermal_freq *therm)
{
	cpufreq_unregister_notifier(&thermal_cpufreq_notifier_block,
		CPUFREQ_POLICY_NOTIFIER);
	thermal_zone_device_unregister(therm->tdev);
	thermal_cooling_device_unregister(therm->cdev);
	kzfree(therm);
}
