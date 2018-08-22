#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/of_gpio.h>

typedef struct sensor_bringup {
    struct device *dev;
    struct device_node *np;
	int vana;
	int vdig;
	int	power;
	int reset;
	struct clk *mclk;
}sensor_bringup_t;

int sensor_bp_init(sensor_bringup_t* sbp, struct device* dev);
int pwr_am_enable(sensor_bringup_t* sensor_bp, const char* propname, int val);
int pwr_am_disable(sensor_bringup_t* sensor_bp);
int reset_am_enable(sensor_bringup_t* sensor_bp, const char* propname, int val);
int reset_am_disable(sensor_bringup_t *sensor_bp);
int clk_am_enable(sensor_bringup_t* sensor_bp, const char* propname);
int clk_am_disable(sensor_bringup_t *sensor_bp);