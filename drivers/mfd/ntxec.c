/*
 * max8998.c - mfd core driver for the Maxim 8998
 *
 *  Copyright (C) 2009-2010 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *  Marek Szyprowski <m.szyprowski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max8998.h>
#include <linux/mfd/max8998-private.h>


#define NTXEC_REG_VERSION 0

struct ntxec_chip {
	struct device *dev;
	struct i2c_client *client;
	int version;
};

static struct mfd_cell ntxec_devs[] = {
	{
		.name = "ntxec-pmic",
	}, {
		.name = "ntxec-watchdog",
	}, {
		.name = "ntxec-rtc",
	}, {
		.name = "ntxec-backlight",
	},
};

int ntxec_read_reg(struct ntxec_chip *ntxec, unsigned int reg)
{
	struct i2c_client *client = ntxec->client;
	int ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "couldn't read register\n");
		return ret;
	}

	return ((ret & 0xff) << 8) | ((ret >> 8) & 0xff);
}

/*
int ntxec_read_reg(struct i2c_client *i2c, u8 reg, u16 *dest)
{
	struct ntxec_chip *ntxec = i2c_get_clientdata(i2c);
	int ret;

//	mutex_lock(&max8998->iolock);
	ret = i2c_smbus_read_word_data(i2c, reg);
//	mutex_unlock(&max8998->iolock);
	if (ret < 0)
		return ret;

	ret &= 0xff;
	*dest = ret;
	return 0;
}
EXPORT_SYMBOL(max8998_read_reg);
*/

/*
int max8998_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct max8998_dev *max8998 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&max8998->iolock);
	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&max8998->iolock);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(max8998_bulk_read);

int max8998_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct max8998_dev *max8998 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&max8998->iolock);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&max8998->iolock);
	return ret;
}
EXPORT_SYMBOL(max8998_write_reg);

int max8998_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct max8998_dev *max8998 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&max8998->iolock);
	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&max8998->iolock);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(max8998_bulk_write);

int max8998_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct max8998_dev *max8998 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&max8998->iolock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&max8998->iolock);
	return ret;
}
EXPORT_SYMBOL(max8998_update_reg);
*/

static int ntxec_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
/*	struct max8998_dev *max8998 = i2c_get_clientdata(i2c);

	if (max8998->wakeup)
		irq_set_irq_wake(max8998->irq, 1);*/
	return 0;
}

static int ntxec_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct max8998_dev *max8998 = i2c_get_clientdata(i2c);

//	if (max8998->wakeup)
//		irq_set_irq_wake(max8998->irq, 0);
	/*
	 * In LP3974, if IRQ registers are not "read & clear"
	 * when it's set during sleep, the interrupt becomes
	 * disabled.
	 */
//	return max8998_irq_resume(i2c_get_clientdata(i2c));
	return 0;
}

static const struct dev_pm_ops ntxec_pm = {
	.suspend = ntxec_suspend,
	.resume = ntxec_resume,
};

static int ntxec_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
//	struct max8998_platform_data *pdata = i2c->dev.platform_data;
	struct ntxec_chip *ntxec;
	int ret = 0;
	u8 buf[2];

	ntxec = kzalloc(sizeof(struct ntxec_chip), GFP_KERNEL);
	if (ntxec == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, ntxec);
	ntxec->dev = &client->dev;
	ntxec->client = client;

//firmware sollte 0xb83a sein

	ret = ntxec_read_reg(ntxec, NTXEC_REG_VERSION);
	if (ret < 0)
		goto err;
	ntxec->version = ret;
	printk("ntxec-version: %x\n", ntxec->version);
ret = 0;
/*	if (pdata) {
		max8998->ono = pdata->ono;
		max8998->irq_base = pdata->irq_base;
		max8998->wakeup = pdata->wakeup;
	}*/
//	mutex_init(&max8998->iolock);

//	max8998->rtc = i2c_new_dummy(i2c->adapter, RTC_I2C_ADDR);
//	i2c_set_clientdata(max8998->rtc, max8998);

//	max8998_irq_init(max8998);

/*	pm_runtime_set_active(max8998->dev);

	switch (id->driver_data) {
	case TYPE_LP3974:
		ret = mfd_add_devices(max8998->dev, -1,
				lp3974_devs, ARRAY_SIZE(lp3974_devs),
				NULL, 0);
		break;
	case TYPE_MAX8998:
		ret = mfd_add_devices(max8998->dev, -1,
				max8998_devs, ARRAY_SIZE(max8998_devs),
				NULL, 0);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0)
		goto err;*/

	return ret;

err:
/*	mfd_remove_devices(max8998->dev);
	max8998_irq_exit(max8998);
	i2c_unregister_device(max8998->rtc);
	kfree(max8998);*/
	return ret;
}

static int ntxec_i2c_remove(struct i2c_client *i2c)
{
	struct ntxec_chip *ntxec = i2c_get_clientdata(i2c);

	mfd_remove_devices(ntxec->dev);
//	ntxec_irq_exit(ntxec);
	kfree(ntxec);

	return 0;
}

static const struct i2c_device_id ntxec_i2c_id[] = {
	{ "ntxec", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max8998_i2c_id);

static struct i2c_driver ntxec_i2c_driver = {
	.driver = {
		   .name = "ntxec",
		   .owner = THIS_MODULE,
		   .pm = &ntxec_pm,
	},
	.probe = ntxec_i2c_probe,
	.remove = ntxec_i2c_remove,
	.id_table = ntxec_i2c_id,
};

static int __init ntxec_i2c_init(void)
{
	return i2c_add_driver(&ntxec_i2c_driver);
}
/* init early as we need to shutdown the watchdog */
subsys_initcall(ntxec_i2c_init);

static void __exit ntxec_i2c_exit(void)
{
	i2c_del_driver(&ntxec_i2c_driver);
}
module_exit(ntxec_i2c_exit);

MODULE_DESCRIPTION("Netronix multi-function core driver");
MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_LICENSE("GPL");
