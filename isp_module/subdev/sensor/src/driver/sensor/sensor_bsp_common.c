#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/of_gpio.h>
#include "sensor_bsp_common.h"

int sensor_bp_init(sensor_bringup_t* sbp, struct device* dev) {
	pr_err("sensor bsp init\n");
	//struct sensor_bringup* sensor_bp;
	sbp->dev = dev;
	sbp->np = dev->of_node;
	sbp->vana = 0;
	sbp->vdig = 0;
	sbp->power = 0;
	sbp->reset = 0;
	return 0;
}

int pwr_am_enable(sensor_bringup_t* sensor_bp, const char* propname, int val) {
	pr_err("pwr_enable: %s\n",propname);
	struct device_node *np;
	int ret;
	np = sensor_bp->np;
	sensor_bp->vana = of_get_named_gpio(np, propname, 0);
	ret = sensor_bp->vana;
	if (ret >= 0) {
		devm_gpio_request(sensor_bp->dev, sensor_bp->vana, "POWER");
		if (gpio_is_valid(sensor_bp->vana)) {
			pr_err("pwr_enable: power gpio init\n");
			gpio_direction_output(sensor_bp->vana, val);
			return ret;
		} else {
			pr_err("pwr_enable: gpio %s is not valid\n", propname);
			return -1;
		}
	} else {
		pr_err("pwr_enable: get_named_gpio %s fail\n", propname);
		return ret;
	}
}

int reset_am_enable(sensor_bringup_t* sensor_bp, const char* propname, int val) {
	pr_err("reset_enable: %s\n",propname);
	struct device_node *np;
	int ret;
	np = sensor_bp->np;
	sensor_bp->reset = of_get_named_gpio(np, propname, 0);
	ret = sensor_bp->reset;
	if (ret >= 0) {
		devm_gpio_request(sensor_bp->dev, sensor_bp->reset, "RESET");
		if (gpio_is_valid(sensor_bp->reset)) {
			pr_err("reset init\n");
			gpio_direction_output(sensor_bp->reset, val);
			return ret;
		} else {
			pr_err("reset_enable: gpio %s is not valid\n", propname);
			return -1;
		}
	} else {
		pr_err("reset_enable: get_named_gpio %s fail\n", propname);
		return ret;
	}
}

int clk_am_enable(sensor_bringup_t* sensor_bp, const char* propname) {
	struct clk *clk;
	int clk_val;
	clk = devm_clk_get(sensor_bp->dev, propname);
	if (IS_ERR(clk)) {
	    pr_err("cannot get %s clk\n", propname);
	    clk = NULL;
	    return -1;
	}

	clk_prepare_enable(clk);
	clk_val = clk_get_rate(clk);
	pr_err("isp init clock is %d MHZ\n",clk_val/1000000);
	return 0;
}