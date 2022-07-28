/* © 2022 Aquarobur Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(modem_sara_r4_pwr, CONFIG_MODEM_LOG_LEVEL);

#include <zephyr.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include "drivers/modem/ublox_sara_r4.h"

#include "modem_context.h"

/* pin settings */
enum mdm_control_pins {
#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_power_gpios)
	MDM_POWER = 0,
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_pwr_on_gpios)
	MDM_PWR_ON,
#endif
	MDM_RESET,
	MDM_VINT,
#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_dtr_gpios)
	MDM_DTR,
#endif
};

static const struct gpio_dt_spec modem_pins[] = {
#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_power_gpios)
	/* MDM_POWER */
	GPIO_DT_SPEC_GET(DT_NODELABEL(ublox_mdm), mdm_power_gpios),
#endif

#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_pwr_on_gpios)
	/* MDM_PWR_ON */
	GPIO_DT_SPEC_GET(DT_NODELABEL(ublox_mdm), mdm_pwr_on_gpios),
#endif

	/* MDM_RESET */
	GPIO_DT_SPEC_GET(DT_NODELABEL(ublox_mdm), mdm_reset_gpios),

	/* MDM_VINT */
	GPIO_DT_SPEC_GET(DT_NODELABEL(ublox_mdm), mdm_vint_gpios),

#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_dtr_gpios)
	/* MDM_DTR */
	GPIO_DT_SPEC_GET(DT_NODELABEL(ublox_mdm), mdm_dtr_gpios),
#endif
};


#define MDM_POWER_ENABLE		1
#define MDM_POWER_DISABLE		0
#define MDM_RESET_NOT_ASSERTED		1
#define MDM_RESET_ASSERTED		0

static struct gpio_callback vint_cb;
static int reset_asserted = 0;
static int v_int = -1;
static struct k_mutex power_lock;



/*----------------------------------------------------------------------------*/
/* Infrastructure for interrupt-based VINT checking */
/*----------------------------------------------------------------------------*/
static void vint_handler(const struct device *port, struct gpio_callback *cb,
			 gpio_port_pins_t pins)
{
	v_int = gpio_pin_get_dt(&modem_pins[MDM_VINT]);
	LOG_DBG("vint handler: V_INT = %d", v_int);
}
/*----------------------------------------------------------------------------*/

static void enable_vint_isr(void)
{
	gpio_pin_interrupt_configure_dt(&modem_pins[MDM_VINT], GPIO_INT_EDGE_BOTH);
}

static void setup_vint_isr(void)
{
	__ASSERT_NO_MSG(modem_pins[MDM_VINT].port);

	gpio_init_callback(&vint_cb, vint_handler, BIT(modem_pins[MDM_VINT].pin));
	gpio_add_callback(modem_pins[MDM_VINT].port, &vint_cb);
	v_int = gpio_pin_get_dt(&modem_pins[MDM_VINT]);
}

static void pin_config(void)
{
#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_power_gpios)
	gpio_pin_configure_dt(&modem_pins[MDM_POWER], GPIO_OUTPUT);
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_pwr_on_gpios)
	gpio_pin_configure_dt(&modem_pins[MDM_PWR_ON], GPIO_OUTPUT);
#endif
	gpio_pin_configure_dt(&modem_pins[MDM_RESET], GPIO_OUTPUT);
	gpio_pin_configure_dt(&modem_pins[MDM_VINT], GPIO_INPUT);
#if DT_NODE_HAS_PROP(DT_NODELABEL(ublox_mdm), mdm_dtr_gpios)
	gpio_pin_configure_dt(&modem_pins[MDM_DTR], GPIO_OUTPUT);
#endif
}

static inline void modem_power_lock(void)
{
	k_mutex_lock(&power_lock, K_FOREVER);
}

static inline void modem_power_unlock(void)
{
	k_mutex_unlock(&power_lock);
}

/* Control the 'pwron' pin */
static inline void pin_pwron_control(bool on)
{
	int assert_time;

	/* Toggle power control pin */
	gpio_pin_set_dt(&modem_pins[MDM_PWR_ON], 0);
	LOG_DBG("MODEM_PWR_ON -> ASSERTED");
	assert_time = on? 200 : 3000;
	if (k_is_in_isr()) {
		k_busy_wait(assert_time * 1000);
	} else {
		k_sleep(K_MSEC(assert_time));
	}
	gpio_pin_set_dt(&modem_pins[MDM_PWR_ON], 1);
	LOG_DBG("MODEM_PWR_ON -> NOT_ASSERTED");
}

static inline void pin_reset_control(bool on)
{
	if (on) {
		LOG_DBG("MODEM_RESET -> ASSERTED");
		gpio_pin_set_dt(&modem_pins[MDM_RESET], MDM_RESET_ASSERTED);
	} else {
		LOG_DBG("MODEM_RESET -> NOT_ASSERTED");
		gpio_pin_set_dt(&modem_pins[MDM_RESET], MDM_RESET_NOT_ASSERTED);
	}
	reset_asserted = on;
}

static inline int vint_wait(int status, k_timeout_t timeout) {

	int ret = 0;
	int counter = k_ticks_to_ms_near32(timeout.ticks) / 100;

	LOG_DBG("Waiting for MDM_modem_pins[MDM_VINT] = %d", status);
	do {
		if (k_is_in_isr()) {
			k_busy_wait(100000);
			v_int = gpio_pin_get_dt(&modem_pins[MDM_VINT]);
		} else {
			k_sleep(K_MSEC(100));
		}

		if (--counter <= 0) {
			/* Timer has expired */
			LOG_DBG("Timeout while waiting for MDM_modem_pins[MDM_VINT]!");
			ret = -ETIMEDOUT;
			break;
		}
	} while (v_int != status);
	LOG_DBG("v_int = %d", v_int);

	return ret;
}

int ublox_sara_r4_pwr_on(void)
{
	int ret = 0;

	k_mutex_lock(&power_lock, K_FOREVER);

	if (gpio_pin_get_dt(&modem_pins[MDM_VINT]) == 1) {
		LOG_DBG("modem already powered ON");
		goto unlock;
	}

	LOG_DBG("modem ON");

	if (reset_asserted) {
		pin_reset_control(0);
		k_sleep(K_SECONDS(3));
	}

	pin_pwron_control(1);

	ret = vint_wait(1, K_SECONDS(30));
	if (ret) {
		LOG_DBG("V_INT is not high!");
		k_sleep(K_MSEC(500));
	}

unlock:
	modem_power_unlock();
	return ret;
}

int ublox_sara_r4_pwr_off(void)
{
	int ret = 0;

	modem_power_lock();

	if (gpio_pin_get_dt(&modem_pins[MDM_VINT]) == 0) {
		LOG_DBG("modem already powered OFF");
		goto unlock;
	}

	LOG_DBG("modem OFF");

	pin_pwron_control(0);
	ret = vint_wait(0, K_SECONDS(30));
	if (ret) {
		LOG_DBG("v_int is not low!");
		k_sleep(K_MSEC(500));
	}

	pin_reset_control(1);

unlock:
	modem_power_unlock();
	return ret;
}

int ublox_sara_r4_pwr_off_force(void)
{
	int ret = -1;

	modem_power_lock();

	pin_reset_control(1);
	pin_pwron_control(0);
	k_sleep(K_MSEC(500));
	pin_pwron_control(0);
	ret = vint_wait(0, K_SECONDS(30));
	if (ret) {
		LOG_DBG("vint is not low!");
		k_sleep(K_MSEC(500));
	}

	modem_power_unlock();
	return ret;
}

int ublox_sara_r4_pwr_wait(int on, k_timeout_t timeout)
{
	int ret = 0;

	if (on) {
		ret = ublox_sara_r4_pwr_on();
	} else {
		ret = ublox_sara_r4_pwr_off();
	}
	if (ret > 0) {
		ret = vint_wait(on, timeout);
	}

	return ret;
}

static int ublox_sara_r4_pwr_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	k_mutex_init(&power_lock);

	/* Make sure modem is powered off at boot for maximum determinism */

	pin_config();

	setup_vint_isr();
	enable_vint_isr();

	if (ublox_sara_r4_pwr_off()) {
		ublox_sara_r4_pwr_off_force();
	}
	ublox_sara_r4_pwr_on();

	return 0;
}

BUILD_ASSERT(CONFIG_MODEM_UBLOX_PWR_INIT_PRIORITY >
	     CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#if CONFIG_MODEM_GSM_INIT_PRIORITY
BUILD_ASSERT(CONFIG_MODEM_UBLOX_PWR_INIT_PRIORITY <
	     CONFIG_MODEM_GSM_INIT_PRIORITY);
#endif

SYS_INIT(ublox_sara_r4_pwr_init, POST_KERNEL,
	 CONFIG_MODEM_UBLOX_PWR_INIT_PRIORITY);
