/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * es9039q2m-i2c.c  -- es9039q2m DAC driver - I2C
 *
 * Copyright 2024 Blue Rock Software Inc.
 *
 * Author: Yash Gandhi <yash@bluerocksoft.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>

#include "es9039q2m.h"

static int es9039q2m_i2c_probe(struct i2c_client *i2c)
{
    struct regmap *regmap;
    
    regmap = devm_regmap_init_i2c(i2c, &es9039q2m_regmap_config);
    if (IS_ERR(regmap))
        return PTR_ERR(regmap);

    return es9039q2m_probe(&i2c->dev, regmap);

}

static void es9039q2m_i2c_remove(struct i2c_client *i2c)
{
    es9039q2m_remove(&i2c->dev);
}

static const struct i2c_device_id es9039q2m_i2c_id[] = {
	{ "es9039q2m", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c,es9039q2m_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id es9039q2m_of_match[] = {
    { .compatible = "ess,es9039q2m", },
    { }
};
MODULE_DEVICE_TABLE(of,es9039q2m_of_match);
#endif

static struct i2c_driver es9039q2m_i2c_driver = {
    .driver = {
        .name = "es9039q2m",
        .pm = &es9039q2m_pm,
        .of_match_table = of_match_ptr(es9039q2m_of_match),
    },
    .probe_new = es9039q2m_i2c_probe,
    .remove = es9039q2m_i2c_remove,
    .id_table = es9039q2m_i2c_id
};

module_i2c_driver(es9039q2m_i2c_driver);

MODULE_DESCRIPTION("ASoC ES9039Q2M driver - I2C");
MODULE_AUTHOR("Yash Gandhi <yash@bluerocksoft.com>");
MODULE_LICENSE("GPL");
