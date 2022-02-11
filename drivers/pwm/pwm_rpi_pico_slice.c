/*
 * Copyright (c) 2022, Joep Buruma
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT raspberrypi_pico_pwm_slice

#include <device.h>
#include <drivers/pwm.h>
#include <drivers/pinctrl.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(pwm_rpi_pico_slice, CONFIG_PWM_LOG_LEVEL);

/* pico-sdk includes */
#include <hardware/pwm.h>
#include <hardware/structs/pwm.h>

#define PWM_RPI_PICO_COUNTER_TOP_MAX UINT16_MAX

struct pwm_rpi_divider_setting {
	uint8_t integral;
	uint8_t frac;
};

struct pwm_rpi_config {
	/* pwm_controller is the start address of the pwm peripheral.
	 * This address is used to get a refference to pwm_slice_hw,
	 * which contains all registers for configuring a single slice
	 */
	pwm_hw_t *pwm_controller;
	uint8_t pwm_slice_nr;
	struct pwm_rpi_divider_setting divider;
	const struct pinctrl_dev_config *pcfg;
};

struct pwm_rpi_data {
	struct pwm_slice_hw *slice;
};

static float pwm_rpi_get_clkdiv(const struct device *dev)
{
	const struct pwm_rpi_config *cfg = dev->config;

	/* the divider is a fixed point 8.4 convert to float for use in pico-sdk */
	return (float)cfg->divider.integral + (float)cfg->divider.integral / 16.0;
}

static int pwm_rpi_get_cycles_per_sec(const struct device *dev, uint32_t ch, uint64_t *cycles)
{
	float f_clock_in = (float)sys_clock_hw_cycles_per_sec();

	/* No need to check for divide by 0 since the minimum value of
	 * returned is 1
	 */
	*cycles = (uint64_t)(f_clock_in / pwm_rpi_get_clkdiv(dev));
	return 0;
}

/* The pico_sdk only allows setting the polarity of both channels at once.
 * This is a convenience function to make setting the polarity of a single
 * channel easier.
 */
static void pwm_rpi_set_channel_polarity(const struct device *dev, uint32_t chan, bool inverted)
{
	struct pwm_rpi_data *data = dev->data;
	const struct pwm_rpi_config *cfg = dev->config;
	bool pwm_polarity_a = (data->slice->csr & PWM_CH0_CSR_A_INV_BITS) > 0;
	bool pwm_polarity_b = (data->slice->csr & PWM_CH0_CSR_B_INV_BITS) > 0;

	if (chan == PWM_CHAN_A) {
		pwm_polarity_a = inverted;
	} else if (chan == PWM_CHAN_B) {
		pwm_polarity_b = inverted;
	}

	pwm_set_output_polarity(cfg->pwm_slice_nr, pwm_polarity_a, pwm_polarity_b);
}

static int pwm_rpi_pin_set(const struct device *dev, uint32_t ch, uint32_t period_cycles,
			   uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_rpi_config *cfg = dev->config;

	if (period_cycles > PWM_RPI_PICO_COUNTER_TOP_MAX ||
	    pulse_cycles > PWM_RPI_PICO_COUNTER_TOP_MAX) {
		return -EINVAL;
	}

	pwm_rpi_set_channel_polarity(dev, ch,
				     (flags && PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED);
	pwm_set_wrap(cfg->pwm_slice_nr, period_cycles);
	pwm_set_chan_level(cfg->pwm_slice_nr, ch, pulse_cycles);

	return 0;
};

struct pwm_driver_api pwm_rpi_driver_api = {
	.get_cycles_per_sec = pwm_rpi_get_cycles_per_sec,
	.pin_set = pwm_rpi_pin_set,
};

static int pwm_rpi_init(const struct device *dev)
{
	const struct pwm_rpi_config *cfg = dev->config;
	struct pwm_rpi_data *data = dev->data;
	pwm_config slice_cfg;
	int err;

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to configure pins for PWM. err=%d", err);
		return err;
	}

	data->slice = &cfg->pwm_controller->slice[cfg->pwm_slice_nr];

	slice_cfg = pwm_get_default_config();
	pwm_config_set_clkdiv_mode(&slice_cfg, PWM_DIV_FREE_RUNNING);
	pwm_init(cfg->pwm_slice_nr, &slice_cfg, false);

	pwm_set_clkdiv_int_frac(cfg->pwm_slice_nr, cfg->divider.integral, cfg->divider.frac);

	pwm_set_enabled(cfg->pwm_slice_nr, true);

	return 0;
}

#define PWM_RPI_INIT(idx)									 \
	BUILD_ASSERT(DT_INST_PROP(idx, divider_int) <= 255);					 \
	BUILD_ASSERT(DT_INST_PROP(idx, divider_int) >= 1);					 \
	BUILD_ASSERT(DT_INST_PROP(idx, divider_frac) <= 15);					 \
												 \
	PINCTRL_DT_INST_DEFINE(idx);								 \
	static const struct pwm_rpi_config pwm_rpi_config_##idx = {				 \
		.pwm_controller = (pwm_hw_t *)DT_REG_ADDR(DT_INST_PARENT(idx)),			 \
		.pwm_slice_nr = DT_INST_REG_ADDR(idx),						 \
		.divider = {									 \
			.integral = DT_INST_PROP(idx, divider_int),				 \
			.frac = DT_INST_PROP(idx, divider_frac),				 \
		},										 \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx)					 \
	};											 \
												 \
	DEVICE_DT_INST_DEFINE(idx, pwm_rpi_init, NULL, NULL, &pwm_rpi_config_##idx, POST_KERNEL, \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &pwm_rpi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_RPI_INIT);
