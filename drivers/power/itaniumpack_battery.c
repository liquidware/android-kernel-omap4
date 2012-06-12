/*
 * Liquidware ItaniumPack battery driver
 *
 * Copyright (C) 2011 Chris Ladden <chris.ladden@liquidware.com>
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/slab.h>

#undef DEBUG

/* Battery update rate */
#define ITANIUMPACK_READ_DELAY		250
#define ITANIUMPACK_READ_DELAY_ERR	5000 /* read delay if error is detected */

/* Registers */
#define BQ20Z80_REG_MANUFACTURER_ACCESS	0x00
	#define BQ20Z80_REG_MAN_DEV_TYPE 0x01
	#define BQ20Z80_REG_MAN_FIRM_VER 0x02
	#define BQ20Z80_REG_MAN_STATUS	 0x06

#define BQ20Z80_REG_TEMP			0x08
#define BQ20Z80_REG_VOLT			0x09
#define BQ20Z80_REG_CURR			0x0a
#define BQ20Z80_REG_CURR_AVG		0x0b
#define BQ20Z80_REG_RSOC			0x0d
#define BQ20Z80_REG_TTE_AVG			0x12
#define BQ20Z80_REG_TTF_AVG			0x13
#define BQ20Z80_REG_REQ_CHG_CURRENT 0x14
#define BQ20Z80_REG_REQ_CHG_VOLTAGE 0x15
#define BQ20Z80_REG_BATT_STATUS 	0x16
#define BQ20Z80_REG_CYCLE_COUNT 	0x17
#define BQ20Z80_REG_SAFETY_STATUS   0x51
#define BQ20Z80_REG_CHARGING_STATUS 0x55

/* BQ20Z80_REG_BATT_STATUS value bits */
#define BQ20Z80_BIT_DSG		0x0040
#define BQ20Z80_BIT_FC		0x0020
#define BQ20Z80_BIT_FD		0x0010

/* BQ20Z80_REG_MANUFACTURER_ACCESS defines */
#define BQ20Z80_REG_MANUFACTURER_ACCESS_STATUS		0x0006

/* BQ20Z80_REG_MANUFACTURER_STATUS value mask*/
#define BQ20Z80_MANUFACTURER_STATUS_STATE	0x0F00
#define BQ20Z80_STATE_UNSUSPEC	0x09
#define BQ20Z80_STATE_OVERHEAT	0x0b
#define BQ20Z80_STATE_BAT_FAIL	0x0c

#define TIME_UNIT_CONVERSION		60
#define TEMP_KELVIN_TO_CELSIUS		2731

#define CHARGER_ADDR	0x09
#define CHARGER_REG_DEVICE_ID	0xFF
#define CHARGER_REG_INPUT_CURRENT	0x3F
/* Change the input current limit
   ONLY if you know what you're doing */
#define CHARGER_INPUT_CURRENT_LIMIT	2048

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(u8 reg, int *value,
		struct bq27x00_device_info *di);
};

struct bq27x00_device_info {
	struct device 		*dev;
	struct delayed_work	 work;
	int			id;
	int			temp_C;
	int			voltage_uV;
	int			current_uA;
	int			charge_rsoc;
	int			tte_avg;
	int			ttf_avg;
	int			ac_online;
	int			status;
	int			health;
	int			ac_online_time;
	int			ac_current_limit;
	int			req_charge_current;
	int			req_charge_voltage;
	int			battery_status;
	int			cycle_count;
	int			charging_status;
	int			man_status;
	int			man_dev_type;
	int			man_firm_ver;
	int			current_now;
	int			safety_status;
	struct bq27x00_access_methods	*bus;
	struct power_supply	bat;
	struct power_supply	mains;
	struct power_supply	usb;
	int 		work_delay;
	struct i2c_client	*client;
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
};

static int bq20z80_read_charger(u8 reg, int *value,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg_data[2];
	struct i2c_msg *msg;
	unsigned char tx_data[2];
	unsigned char rx_data[3];
	int err;
	u8 pec;

	if (!client->adapter)
		return -ENODEV;

	msg = &msg_data[0];
	msg->addr = CHARGER_ADDR;
	msg->buf = tx_data;
	msg->flags = 0;
	msg->len = 1;
	tx_data[0] = reg;

	msg = &msg_data[1];
	msg->addr = CHARGER_ADDR;
	msg->buf = rx_data;
	msg->flags = I2C_M_RD;
	msg->len = 3;

	/* Transfer */
	err = i2c_transfer(client->adapter, msg_data, 2);
	if (err < 0) {
		//Silently fail
		return err;
	}

	/* Parse the value and PEC */
	*value = (s16)((s8)rx_data[1] << 8) + (s8)rx_data[0];
	pec = rx_data[2];
#ifdef DEBUG
	printk(KERN_INFO "bq20z80_read_charger: I2C transfer success, data [%X][%X][%X]\n",
		   rx_data[0],
		   rx_data[1],
		   rx_data[2]);
	printk(KERN_INFO "bq20z80_read_charger: reg=%X, value=%d, pec=%X\n",
			reg,
			*value,
			pec);
#endif
	return 0;
}

static int bq20z80_write_charger(u8 reg, uint16_t value,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg_data[1];
	struct i2c_msg *msg;
	unsigned char tx_data[3];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg = &msg_data[0];
	msg->addr = CHARGER_ADDR;
	msg->buf = tx_data;
	msg->flags = 0;
	msg->len = 3;
	tx_data[0] = reg;
	tx_data[1] = (value & 0xFF);
	//tx_data[1] = 0x00;
	tx_data[2] = ((value >> 8) & 0xFF);
	//tx_data[2] = 0x04;

	/* Transfer */
	err = i2c_transfer(client->adapter, msg_data, 1);
	if (err < 0) {
		//loudly fail
		printk(KERN_ERR "bq20z80_write_charger: i2c err=%d\n", err);
		return err;
	}

	return 0;
}

static int bq20z80_write(u8 reg, uint16_t value,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg_data[1];
	struct i2c_msg *msg;
	unsigned char tx_data[3];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg = &msg_data[0];
	msg->addr = client->addr;
	msg->buf = tx_data;
	msg->flags = 0;
	msg->len = 3;
	tx_data[0] = reg;
	tx_data[1] = (value & 0xFF);
	tx_data[2] = ((value >> 8) & 0xFF);

	/* Transfer */
	err = i2c_transfer(client->adapter, msg_data, 1);
	if (err < 0) {
		//loudly fail
		printk(KERN_ERR "bq20z80_write: i2c err=%d\n", err);
		return err;
	}

	return 0;
}

static int bq20z80_read(u8 reg, int *value, bool isSignedInt,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg_data[2];
	struct i2c_msg *msg;
	unsigned char tx_data[2];
	unsigned char rx_data[3];
	int err;
	u8 pec;

	if (!client->adapter)
		return -ENODEV;

	msg = &msg_data[0];
	msg->addr = client->addr;
	msg->buf = tx_data;
	msg->flags = 0;
	msg->len = 1;
	tx_data[0] = reg;

	msg = &msg_data[1];
	msg->addr = client->addr;
	msg->buf = rx_data;
	msg->flags = I2C_M_RD;
	msg->len = 3;

	/* Transfer */
	err = i2c_transfer(client->adapter, msg_data, 2);
	if (err < 0) {
		printk(KERN_ERR "bq20x80_read: Error: I2C transfer err=%d\n", err);
		return err;
	}

	/* Parse the value and PEC */
	if (!isSignedInt) {
		*value = get_unaligned_le16(rx_data);
	} else {
		s16 vals16;
		vals16 = (signed)get_unaligned_le16(rx_data);
		*value = vals16; //(s16)((s8)rx_data[1] << 8) + (s8)rx_data[0];
	}


	pec = rx_data[2];
#ifdef DEBUG
	printk(KERN_ERR "bq20x80_read: I2C transfer success, data [%X][%X][%X]\n",
		   rx_data[0],
		   rx_data[1],
		   rx_data[2]);
	printk(KERN_ERR "bq20x80_read: reg=%X, value=%d, pec=%X\n",
			reg,
			*value,
			pec);
#endif
	return 0;
}

/*
 * Return the battery temperature in Celsius degrees
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di)
{
	int ret;
	int temp = 0;
	unsigned int temp_kelvin;
	int temp_c;

	ret = bq20z80_read(BQ20Z80_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	temp_kelvin = (unsigned int) temp;

    /* bq20z80 provides battery temperature in 0.1K
	 * so convert it to 0.1°C
	 */
	temp_c = (int)temp_kelvin - TEMP_KELVIN_TO_CELSIUS;

#ifdef DEBUG
	printk(KERN_ERR "bq27x00_battery_temperature: temp_kelvin=%d temp_c=%d, temp_raw=%d\n",
		   temp_kelvin, temp_c, temp);
#endif
	return temp_c;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq20z80_read(BQ20Z80_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return (unsigned int)volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di)
{
    int ret;
    int curr = 0;

    ret = bq20z80_read(BQ20Z80_REG_CURR, &curr, 1, di);
    if (ret) {
        dev_err(di->dev, "error reading current\n");
        return 0;
    }

    return curr;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_rsoc(struct bq27x00_device_info *di)
{
	int ret;
	int rsoc = 0;

	ret = bq20z80_read(BQ20Z80_REG_RSOC, &rsoc, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	return rsoc;
}

/*
 * Return the battery average time to empty
 * Or < 0 if something fails.
 */
static int bq27x00_battery_time_to_empty_avg(struct bq27x00_device_info *di)
{
	int ret;
	int tval;
	int tte_avg = 0;

	ret = bq20z80_read(BQ20Z80_REG_TTE_AVG, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading average time to empty\n");
		return ret;
	}

	if (tval == 65535)
		return -ENODATA;

	/* bq20z80 provides time to empty and time to full in minutes.
	 * Convert to seconds
	 */
	tte_avg = tval * TIME_UNIT_CONVERSION;
	return tte_avg;
}

/*
 * Return the battery average time to full
 * Or < 0 if something fails.
 */
static int bq27x00_battery_time_to_full_avg(struct bq27x00_device_info *di)
{
	int ret;
	int tval;
	int ttf_avg = 0;

	ret = bq20z80_read(BQ20Z80_REG_TTF_AVG, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading average time to full\n");
		return ret;
	}

	if (tval == 65535)
		return -ENODATA;

	/* bq20z80 provides time to empty and time to full in minutes.
	 * Convert to seconds
	 */
	ttf_avg = tval * TIME_UNIT_CONVERSION;
	return ttf_avg;
}

/*
 * Return the AC charger online
 * Or 0 if ac offline.
 */
static int bq27x00_battery_ac_online(struct bq27x00_device_info *di)
{
	int ret;
	int deviceid = 0;
	int ac_online = 0;
	int ic = 0;

	ret = bq20z80_read_charger(CHARGER_REG_DEVICE_ID, &deviceid, di);
	if (ret) {
		/* Try again */
		if (bq20z80_read_charger(CHARGER_REG_DEVICE_ID, &deviceid, di)) {
			/* ac is offline */
			di->ac_online_time = 0;
			return 0;
		}
	}

	/* Write the input current */
	ret = bq20z80_write_charger(CHARGER_REG_INPUT_CURRENT,
								CHARGER_INPUT_CURRENT_LIMIT, di);
	if (ret) {
		/* Try again */
		ret = bq20z80_write_charger(CHARGER_REG_INPUT_CURRENT,
									CHARGER_INPUT_CURRENT_LIMIT, di);
		if (ret) {
			printk(KERN_ERR "bq27x00_battery: cannot write to charger.\n");
			//return 0;
		}
	}

	/* Read the input current */
	ret = bq20z80_read_charger(CHARGER_REG_INPUT_CURRENT, &ic, di);
	if (ret) {
		/* Try again */
		ret = bq20z80_read_charger(CHARGER_REG_INPUT_CURRENT, &ic, di);
		if (ret) {
			printk(KERN_ERR "bq27x00_battery: cannot read charger current.\n");
			//return 0;
		}
	}

	di->ac_current_limit = ic;

	if (ic != CHARGER_INPUT_CURRENT_LIMIT) {
		printk(KERN_ERR "bq27x00_battery: charger current does not match!\n");
	}

	ac_online = (((unsigned int)deviceid == 0x0007));

	if (ac_online) {
		di->ac_online_time += di->work_delay;
	}

	return ac_online;
}

/*
 * Return the Battery Status
 * Or < 0 if something fails.
 */
static int bq27x00_battery_status(struct bq27x00_device_info *di)
{
	int status = 0;

	//TODO: Fix the status read
	if (di->ac_online)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	return status;
}

/*
 * Return a battery register value
 * Or < 0 if something fails.
 */
static int bq27x00_battery_reg_value(struct bq27x00_device_info *di, u8 reg, bool isSignedInt16)
{
	int ret;
	int val = 0;

	ret = bq20z80_read(reg, &val, isSignedInt16, di);
	if (ret) {
		dev_err(di->dev, "error reading register=%d\n", reg);
		return ret;
	}

	return val;
}

/*
 * Return a special manufacturer access register value
 * Or < 0 if something fails.
 */
static int bq27x00_battery_man_access_reg_value(struct bq27x00_device_info *di, u8 reg, bool isSignedInt16)
{
	int ret;
	int val = 0;

	/* Write to Manufacturer Access */
	ret = bq20z80_write(BQ20Z80_REG_MANUFACTURER_ACCESS,
			reg, di);
	if (ret) {
		dev_err(di->dev, "error writing ManufacturerAccess\n");
		return ret;
	}

	/* Read back the register */
	ret = bq20z80_read(BQ20Z80_REG_MANUFACTURER_ACCESS, &val, isSignedInt16, di);
	if (ret) {
		dev_err(di->dev, "error reading register=%d\n", reg);
		return ret;
	}

	return val;
}

/*
 * Return the Battery Health
 * Or < 0 if something fails.
 */
static int bq27x00_battery_health(struct bq27x00_device_info *di)
{
#if 0
	int ret;
	int value = 0;
	int state = 0;
	int health = 0;

	/* Write to Manufacturer Access */
	ret = bq20z80_write(BQ20Z80_REG_MANUFACTURER_ACCESS,
			BQ20Z80_REG_MANUFACTURER_ACCESS_STATUS, di);
	if (ret) {
		dev_err(di->dev, "error writing ManufacturerAccess\n");
		return ret;
	}

	/* Read ManufacturerStatus */
	ret = bq20z80_read(BQ20Z80_REG_MAN_ACCESS, &value, di);
	if (ret) {
		dev_err(di->dev, "error reading ManufacturerStatus\n");
		return ret;
	}
	value = (unsigned int)value;

	/* Mask off the state bits
	 * and then take the upper byte */
	state = (value & BQ20Z80_MANUFACTURER_STATUS_STATE) >> 8;

	if (state == BQ20Z80_STATE_UNSUSPEC)
		health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	else if (state == BQ20Z80_STATE_OVERHEAT)
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (state == BQ20Z80_STATE_BAT_FAIL)
		health = POWER_SUPPLY_HEALTH_DEAD;
	else
		health = POWER_SUPPLY_HEALTH_GOOD;

	return health;
#endif
	return POWER_SUPPLY_HEALTH_GOOD;
}

#define to_bq27x00_device_info_batt(x) container_of((x), \
				struct bq27x00_device_info, bat);
#define to_bq27x00_device_info_usb(x) container_of((x), \
				struct bq27x00_device_info, usb);
#define to_bq27x00_device_info_mains(x) container_of((x), \
				struct bq27x00_device_info, mains);

static int bq27x00_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info_mains(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->ac_online; //bq27x00_battery_ac_online(di);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* AC Power */

static enum power_supply_property bq27x00_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *bq27x00_ac_supplied_to[] = {
	"main-battery",
};

static struct power_supply bq27x00_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = bq27x00_ac_supplied_to,
	.num_supplicants = ARRAY_SIZE(bq27x00_ac_supplied_to),
	.properties = bq27x00_ac_props,
	.num_properties = ARRAY_SIZE(bq27x00_ac_props),
	.get_property = bq27x00_ac_get_property,
};

static int bq27x00_usb_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	//struct bq27x00_device_info *di = to_bq27x00_device_info_usb(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = false;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* USB power emulated, not used */

static enum power_supply_property bq27x00_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *bq27x00_usb_supplied_to[] = {
	"main-battery",
};

static struct power_supply bq27x00_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = bq27x00_usb_supplied_to,
	.num_supplicants = ARRAY_SIZE(bq27x00_usb_supplied_to),
	.properties = bq27x00_usb_props,
	.num_properties = ARRAY_SIZE(bq27x00_usb_props),
	.get_property = bq27x00_usb_get_property,
};

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info_batt(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = di->health;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->voltage_uV;
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->charge_rsoc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		val->intval = di->tte_avg;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = di->ttf_avg;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = NULL;

	di->mains = bq27x00_ac;
	di->usb = bq27x00_usb;
}

static void itaniumpack_battery_work(struct work_struct *work)
{
	struct bq27x00_device_info *di;

	di = container_of(work, struct bq27x00_device_info, work.work);

	di->temp_C = bq27x00_battery_temperature(di);
	if (di->temp_C < 0) {
		//di->work_delay = ITANIUMPACK_READ_DELAY_ERR;
		//printk(KERN_ERR
		//	   "itaniumpack: updating work every %dms\n",
		//	   di->work_delay);
		//goto work_out;
	}

	di->voltage_uV = bq27x00_battery_voltage(di);
	di->current_uA = bq27x00_battery_current(di);
	di->charge_rsoc = bq27x00_battery_rsoc(di);
	di->tte_avg = bq27x00_battery_time_to_empty_avg(di);
	di->ttf_avg = bq27x00_battery_time_to_full_avg(di);
	di->status = bq27x00_battery_status(di);
	di->health = bq27x00_battery_health(di);
	di->ac_online = bq27x00_battery_ac_online(di);

	di->req_charge_current = bq27x00_battery_reg_value(di, BQ20Z80_REG_REQ_CHG_CURRENT, false);
	di->req_charge_voltage = bq27x00_battery_reg_value(di, BQ20Z80_REG_REQ_CHG_VOLTAGE, false);
	di->battery_status = bq27x00_battery_reg_value(di, BQ20Z80_REG_BATT_STATUS, false);
	di->cycle_count = bq27x00_battery_reg_value(di, BQ20Z80_REG_CYCLE_COUNT, false);
	di->charging_status = bq27x00_battery_reg_value(di, BQ20Z80_REG_CHARGING_STATUS, false);
	di->man_status = bq27x00_battery_man_access_reg_value(di, BQ20Z80_REG_MAN_STATUS, false);
	di->man_dev_type = bq27x00_battery_man_access_reg_value(di, BQ20Z80_REG_MAN_DEV_TYPE, false);
	di->man_firm_ver = bq27x00_battery_man_access_reg_value(di, BQ20Z80_REG_MAN_FIRM_VER, false);
	di->current_now = bq27x00_battery_reg_value(di, BQ20Z80_REG_CURR, true);
	di->safety_status = bq27x00_battery_reg_value(di, BQ20Z80_REG_SAFETY_STATUS, false);

#ifdef DEBUG
	printk(KERN_ERR "temp_C:%d, voltage_uV:%d, current_uA:%d, \
					  charge_rsoc:%d, tte_avg:%d, ttf_avg:%d, \
					  ac_online:%d, status:%d, health:%d\n",
					  di->temp_C, di->voltage_uV, di->current_uA,
					  di->charge_rsoc, di->tte_avg, di->ttf_avg,
					  di->ac_online, di->status, di->health);
#endif

	power_supply_changed(&di->bat);
	power_supply_changed(&di->mains);
	power_supply_changed(&di->usb);

work_out:
	schedule_delayed_work(&di->work, di->work_delay);
}


struct bq27x00_device_info *di;

#define BITF(char_array, bit_array, index, name) char_array[index] = (bit_array[index] == 1) ? name : "#" name;

static ssize_t show_ac_online_time(struct device *dev, struct device_attribute *attr, char *buf) {
    return sprintf(buf, "%d mS\n", di->ac_online_time);
}

static ssize_t show_ac_current_limit(struct device *dev, struct device_attribute *attr, char *buf) {
    return sprintf(buf, "%d mA\n", di->ac_current_limit);
}

static ssize_t show_req_charge_current(struct device *dev, struct device_attribute *attr, char *buf) {
    return sprintf(buf, "%d mA\n", di->req_charge_current);
}

static ssize_t show_req_charge_voltage(struct device *dev, struct device_attribute *attr, char *buf) {
    return sprintf(buf, "%d mV\n", di->req_charge_voltage);
}

static ssize_t show_battery_status(struct device *dev, struct device_attribute *attr, char *buf) {
    u16 val = di->battery_status;
	u8 val_lb = val & 0xFF;
	u8 val_hb = (val >> 8) & 0xFF;
    u8 i;
	u8 blb[8];
	u8 bhb[8];
	char *slb[8];
	char *shb[8];

    for (i=0; i<8; i++) {
        blb[i] = (val_lb >> i) & 0x01;
    }
	for (i=0; i<8; i++) {
        bhb[i] = (val_hb >> i) & 0x01;
    }

	BITF(shb, bhb, 0, "RTA");
	BITF(shb, bhb, 1, "RCA");
	BITF(shb, bhb, 2, "RSVD");
	BITF(shb, bhb, 3, "TDA");
	BITF(shb, bhb, 4, "OTA");
	BITF(shb, bhb, 5, "RSVD");
	BITF(shb, bhb, 6, "TCA");
	BITF(shb, bhb, 7, "OCA");

	BITF(slb, blb, 0, "EC0");
	BITF(slb, blb, 1, "EC1");
	BITF(slb, blb, 2, "EC2");
	BITF(slb, blb, 3, "EC3");
	BITF(slb, blb, 4, "FD");
	BITF(slb, blb, 5, "FC");
	BITF(slb, blb, 6, "DSG");
	BITF(slb, blb, 7, "INIT");

    return sprintf(buf, "HB=0x%X, LB=0x%X\n" \
						"|%10s|%10s|%10s|%10s|%10s|%10s|%10s|%10s|\n" \
						"|%10s|%10s|%10s|%10s|%10s|%10s|%10s|%10s|\n", ((val >> 8) & 0xFF), val & 0xFF,
							 shb[7], shb[6], shb[5], shb[4], shb[3], shb[2], shb[1], shb[0],
							 slb[7], slb[6], slb[5], slb[4], slb[3], slb[2], slb[1], slb[0]);
}

static ssize_t show_cycle_count(struct device *dev, struct device_attribute *attr, char *buf) {
    return sprintf(buf, "%d\n", di->cycle_count);
}

static ssize_t show_charging_status(struct device *dev, struct device_attribute *attr, char *buf) {
    u16 val = di->charging_status;
	u8 val_lb = val & 0xFF;
	u8 val_hb = (val >> 8) & 0xFF;
    u8 i;
	u8 blb[8];
	u8 bhb[8];
	char *slb[8];
	char *shb[8];

    for (i=0; i<8; i++) {
        blb[i] = (val_lb >> i) & 0x01;
    }
	for (i=0; i<8; i++) {
        bhb[i] = (val_hb >> i) & 0x01;
    }

	BITF(shb, bhb, 0, "PULSE");
	BITF(shb, bhb, 1, "FCHG");
	BITF(shb, bhb, 2, "TCHG2");
	BITF(shb, bhb, 3, "TCHG1");
	BITF(shb, bhb, 4, "MCHG");
	BITF(shb, bhb, 5, "PCHG");
	BITF(shb, bhb, 6, "CHGSUSP");
	BITF(shb, bhb, 7, "XCHG");

	BITF(slb, blb, 0, "XCHGLV");
	BITF(slb, blb, 1, "OC");
	BITF(slb, blb, 2, "OCHGI");
	BITF(slb, blb, 3, "OCHGV");
	BITF(slb, blb, 4, "FCMTO");
	BITF(slb, blb, 5, "PCMTO");
	BITF(slb, blb, 6, "CB");
	BITF(slb, blb, 7, "PULSEOFF");

    return sprintf(buf, "HB=0x%X, LB=0x%X\n" \
						"|%10s|%10s|%10s|%10s|%10s|%10s|%10s|%10s|\n" \
						"|%10s|%10s|%10s|%10s|%10s|%10s|%10s|%10s|\n", ((val >> 8) & 0xFF), val & 0xFF,
							 shb[7], shb[6], shb[5], shb[4], shb[3], shb[2], shb[1], shb[0],
							 slb[7], slb[6], slb[5], slb[4], slb[3], slb[2], slb[1], slb[0]);
}


static char* get_man_status_state(char* buf, u8 state) {
	switch (state) {
	case 0x00:
		sprintf(buf, "Wake up");
		break;
	case 0x03:
		sprintf(buf, "Pre-charge");
		break;
	case 0x07:
		sprintf(buf, "Terminate charge");
		break;
	case 0x05:
		sprintf(buf, "Normal charge");
		break;
	case 0x01:
		sprintf(buf, "Normal discharge");
		break;
	case 0x0E:
		sprintf(buf, "Depleted||Depeleted AC||Overhead discharge||Overeheat charge||Batt fail overcharge||Batt fail low temp");
		break;
	case 0x08:
		sprintf(buf, "Batt fail charger termination");
		break;
	case 0x0C:
		sprintf(buf, "Batt fail");
		break;
	case 0x0A:
		sprintf(buf, "Batt fail charge||Bat fail discharge");
		break;
	case 0x0B:
		sprintf(buf, "Overheat charge||Overheat discharge");
		break;
	case 0x0F:
		sprintf(buf, "Removed");
		break;
	case 0x0D:
		sprintf(buf, "Sleep");
		break;
	case 0x09:
		sprintf(buf, "Permanent failure");
		break;
	}

	return buf;
}

static ssize_t show_man_status(struct device *dev, struct device_attribute *attr, char *buf) {
    u16 val = di->man_status;
	u8 val_lb = val & 0xFF;
	u8 val_hb = (val >> 8) & 0xFF;
    u8 i;
	u8 blb[8];
	u8 bhb[8];
	char *slb[8];
	char *shb[8];
	char state[255];

	get_man_status_state(&state[0], val_hb & 0x0F);

    for (i=0; i<8; i++) {
        blb[i] = (val_lb >> i) & 0x01;
    }
	for (i=0; i<8; i++) {
        bhb[i] = (val_hb >> i) & 0x01;
    }

	BITF(shb, bhb, 0, "STATE0");
	BITF(shb, bhb, 1, "STATE1");
	BITF(shb, bhb, 2, "STATE2");
	BITF(shb, bhb, 3, "STATE3");
	BITF(shb, bhb, 4, "PF0");
	BITF(shb, bhb, 5, "PF1");
	BITF(shb, bhb, 6, "FET0");
	BITF(shb, bhb, 7, "FET1");

	BITF(slb, blb, 0, "0");
	BITF(slb, blb, 1, "0");
	BITF(slb, blb, 2, "0");
	BITF(slb, blb, 3, "0");
	BITF(slb, blb, 4, "0");
	BITF(slb, blb, 5, "0");
	BITF(slb, blb, 6, "1");
	BITF(slb, blb, 7, "0");

    return sprintf(buf, "HB=0x%X, LB=0x%X\n" \
						"|%10s|%10s|%10s|%10s|%10s|%10s|%10s|%10s|\n" \
						"|%10s|%10s|%10s|%10s|%10s|%10s|%10s|%10s|\n" \
						"STATE=%s\n", ((val >> 8) & 0xFF), val & 0xFF,
							 shb[7], shb[6], shb[5], shb[4], shb[3], shb[2], shb[1], shb[0],
							 slb[7], slb[6], slb[5], slb[4], slb[3], slb[2], slb[1], slb[0],
						state);
}

static ssize_t show_man_dev_type(struct device *dev, struct device_attribute *attr, char *buf) {
    u16 val = di->man_dev_type;
    return sprintf(buf, "HB=0x%X, LB=0x%X\n", ((val >> 8) & 0xFF), val & 0xFF);
}

static ssize_t show_man_firm_ver(struct device *dev, struct device_attribute *attr, char *buf) {
    return sprintf(buf, "HB=0x%X, LB=0x%X\n", ((di->man_firm_ver >> 8) & 0xFF),
                   di->man_firm_ver & 0xFF);
}

static ssize_t show_current_now(struct device *dev, struct device_attribute *attr, char *buf) {
    return sprintf(buf, "HB=0x%X, LB=0x%X\n" \
				   "current_now=%d\n", ((di->current_now >> 8) & 0xFF),
                   di->current_now & 0xFF,
				   di->current_now);
}

static ssize_t show_safety_status(struct device *dev, struct device_attribute *attr, char *buf) {
    u16 val = di->safety_status;
	u8 val_lb = val & 0xFF;
	u8 val_hb = (val >> 8) & 0xFF;
    u8 i;
	u8 blb[8];
	u8 bhb[8];
	char *slb[8];
	char *shb[8];

    for (i=0; i<8; i++) {
        blb[i] = (val_lb >> i) & 0x01;
    }
	for (i=0; i<8; i++) {
        bhb[i] = (val_hb >> i) & 0x01;
    }

	BITF(shb, bhb, 0, "POV");
	BITF(shb, bhb, 1, "PUV");
	BITF(shb, bhb, 2, "OCC2");
	BITF(shb, bhb, 3, "OCD2");
	BITF(shb, bhb, 4, "OCC");
	BITF(shb, bhb, 5, "OCD");
	BITF(shb, bhb, 6, "OTC");
	BITF(shb, bhb, 7, "OTD");

	BITF(slb, blb, 0, "SCD");
	BITF(slb, blb, 1, "SCC");
	BITF(slb, blb, 2, "AOCD");
	BITF(slb, blb, 3, "WDF");
	BITF(slb, blb, 4, "HWDG");
	BITF(slb, blb, 5, "PF");
	BITF(slb, blb, 6, "COV");
	BITF(slb, blb, 7, "CUV");

    return sprintf(buf, "HB=0x%X, LB=0x%X\n" \
						"|%10s|%10s|%10s|%10s|%10s|%10s|%10s|%10s|\n" \
						"|%10s|%10s|%10s|%10s|%10s|%10s|%10s|%10s|\n", ((val >> 8) & 0xFF), val & 0xFF,
							 shb[7], shb[6], shb[5], shb[4], shb[3], shb[2], shb[1], shb[0],
							 slb[7], slb[6], slb[5], slb[4], slb[3], slb[2], slb[1], slb[0]);
}

static DEVICE_ATTR(ac_online_time, 0444, show_ac_online_time, NULL);
static DEVICE_ATTR(ac_current_limit, 0444, show_ac_current_limit, NULL);
static DEVICE_ATTR(req_charge_current, 0444, show_req_charge_current, NULL);
static DEVICE_ATTR(req_charge_voltage, 0444, show_req_charge_voltage, NULL);
static DEVICE_ATTR(battery_status, 0444, show_battery_status, NULL);
static DEVICE_ATTR(cycle_count, 0444, show_cycle_count, NULL);
static DEVICE_ATTR(charging_status, 0444, show_charging_status, NULL);
static DEVICE_ATTR(man_status, 0444, show_man_status, NULL);
static DEVICE_ATTR(man_dev_type, 0444, show_man_dev_type, NULL);
static DEVICE_ATTR(man_firm_ver, 0444, show_man_firm_ver, NULL);
static DEVICE_ATTR(current_now, 0444, show_current_now, NULL);
static DEVICE_ATTR(safety_status, 0444, show_safety_status, NULL);

static void create_sysfs_entry(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_ac_online_time))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_ac_current_limit))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_req_charge_current))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_req_charge_voltage))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_battery_status))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_cycle_count))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_charging_status))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_man_status))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_man_dev_type))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_man_firm_ver))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_current_now))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
	if (device_create_file(dev, &dev_attr_safety_status))
		printk(KERN_ERR "itaniumpack_battery: "
			"failed to create sysfs entry \n");
}


static int itaniumpack_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;

	struct bq27x00_access_methods *bus;
	int num;
	int retval = 0;

	printk(KERN_INFO "itaniumpack_battery_probe\n");

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "itaniumpack-%d", num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = name;
	bus->read = &bq20z80_read;
	di->bus = bus;
	di->client = client;

	/* Init the time */
	di->ac_online_time = 0;

	bq27x00_powersupply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}

	retval = power_supply_register(&client->dev, &di->mains);
	if (retval) {
		dev_err(&client->dev, "failed to register ac mains\n");
		goto batt_failed_4;
	}

	retval = power_supply_register(&client->dev, &di->usb);
	if (retval) {
		dev_err(&client->dev, "failed to register usb power\n");
		goto batt_failed_4;
	}

	create_sysfs_entry(&di->dev[0]);

	INIT_DELAYED_WORK_DEFERRABLE(&di->work, itaniumpack_battery_work);

	di->work_delay = ITANIUMPACK_READ_DELAY;
	schedule_delayed_work(&di->work, di->work_delay);

	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int itaniumpack_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->bat);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

/*
 * Module stuff
 */

static const struct i2c_device_id itaniumpack_id[] = {
	{ "itaniumpack_battery", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, itaniumpack_id);

static struct i2c_driver itaniumpack_battery_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name = "itaniumpack_battery",
	},
	.probe = itaniumpack_battery_probe,
	.remove = itaniumpack_battery_remove,
	.id_table = itaniumpack_id,
};

static int __init itaniumpack_battery_init(void)
{
	int ret;
	printk(KERN_INFO "itaniumpack_battery: init\n");

	ret = i2c_add_driver(&itaniumpack_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register itaniumpack driver\n");

	return ret;
}
module_init(itaniumpack_battery_init);

static void __exit itaniumpack_battery_exit(void)
{
	i2c_del_driver(&itaniumpack_battery_driver);
}
module_exit(itaniumpack_battery_exit);

MODULE_AUTHOR("Chris Ladden <chris.ladden@liquidware.com>");
MODULE_DESCRIPTION("Liquidware ItaniumPack battery monitor driver");
MODULE_LICENSE("GPL");
