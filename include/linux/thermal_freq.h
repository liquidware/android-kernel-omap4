/*
 * Copyright 2011 Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#ifndef __THERMAL_FREQ_H__
#define __THERMAL_FREQ_H__

#include <linux/mfd/omap4_scm.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>

struct thermal_freq_table {
	unsigned long temp;
	unsigned long freq;
	unsigned long polling_interval;
};

struct thermal_freq {
	int (*get_temp)(void *privdata);
	void *privdata;
	struct thermal_zone_device *tdev;
	int idle_polling_delay;
	enum thermal_device_mode mode;
	int trip_count;
	struct thermal_freq_table *trip_table;
	struct thermal_cooling_device *cdev;
	int current_trip;
	unsigned long state;
	unsigned long current_temp;
};

extern struct thermal_freq *thermal_freq_register(char *domain,
	int (*get_temp)(void *), void *privdata,
	 struct thermal_freq_table *trip_table,
	int table_size, int polling_interval);
void thermal_freq_unregister(struct thermal_freq *therm);

#endif
