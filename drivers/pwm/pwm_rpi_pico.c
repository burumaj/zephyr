/*
 * Copyright (c) 2022, Joep Buruma
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT raspberrypi_pico_pwm

#include <device.h>
#include <drivers/pwm.h>
#include <drivers/pinctrl.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(pwm_rpi_pico, CONFIG_PWM_LOG_LEVEL);

/* pico-sdk includes */
#include <hardware/pwm.h>
#include <hardware/structs/pwm.h>

#define PWM_RPI_PICO_COUNTER_TOP_MAX UINT16_MAX

struct pwm_rpi_config {
	pwm_hw_t *pwm_controller;
	const struct pinctrl_dev_config *pcfg;
};

struct pwm_rpi_data {
	pwm_config channel_config[NUM_PWM_SLICES];
};

static int pwm_rpi_get_cycles_per_sec(const struct device *dev, uint32_t ch, uint64_t *cycles)
{
	int f_clock_in = sys_clock_hw_cycles_per_sec();

	*cycles = f_clock_in;
	return 0;
}

static int pwm_rpi_pin_set(const struct device *dev, uint32_t ch, uint32_t period_cycles,
			   uint32_t pulse_cycles, pwm_flags_t flags)
{
	struct pwm_rpi_data *data = dev->data;
	const struct pwm_rpi_config *cfg = dev->config;
	uint8_t slice_nr = pwm_gpio_to_slice_num(ch);
	uint8_t channel_nr = pwm_gpio_to_channel(ch);
	bool pwm_polarity_a;
	bool pwm_polarity_b;
	pwm_config *pwm_cfg;

	if(period_cycles > PWM_RPI_PICO_COUNTER_TOP_MAX || pulse_cycles > PWM_RPI_PICO_COUNTER_TOP_MAX) {
		return -EINVAL;
	}

	pwm_cfg = &data->channel_config[slice_nr];
	pwm_config_set_clkdiv_int(pwm_cfg, 1);
	pwm_config_set_clkdiv_mode(pwm_cfg, PWM_DIV_FREE_RUNNING);
	pwm_config_set_wrap(pwm_cfg, period_cycles);

	pwm_polarity_a = (pwm_cfg->csr & PWM_CH0_CSR_A_INV_BITS) > 0;
	pwm_polarity_b = (pwm_cfg->csr & PWM_CH0_CSR_B_INV_BITS) > 0;

	if (channel_nr == PWM_CHAN_A) {
		pwm_polarity_a = (flags & PWM_POLARITY_MASK) > 0;
	} else if (channel_nr == PWM_CHAN_B) {
		pwm_polarity_b = (flags & PWM_POLARITY_MASK) > 0;
	}
	pwm_config_set_output_polarity(pwm_cfg, pwm_polarity_a, pwm_polarity_b);

	pwm_init(slice_nr, pwm_cfg, false);
	// set counter compare values after the pwm init because
	// the pico-sdk pwm-init resets capture compare values.
	int32_t cc_values = cfg->pwm_controller->slice[slice_nr].cc;
	if (channel_nr == PWM_CHAN_A) {
		// preserve counter compare in channel B
		cc_values &= BIT_MASK(16) << 16;
		cc_values |= pulse_cycles;
	} else if (channel_nr == PWM_CHAN_B) {
		// preserve counter compare in channel A
		cc_values &= BIT_MASK(16);
		cc_values |= pulse_cycles << 16;
	}
	cfg->pwm_controller->slice[slice_nr].cc = cc_values;

	pwm_set_enabled(slice_nr, true);
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
	int err;

	for(int i = 0; i < NUM_PWM_SLICES; i++){
		data->channel_config[i] = pwm_get_default_config();
	}

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to configure pins for PWM");
		return err;
	}

	return 0;
}

#define PWM_RPI_INIT(idx)                                                                          \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	static const struct pwm_rpi_config pwm_rpi_config_##idx = {                                \
		.pwm_controller = (pwm_hw_t *)DT_INST_REG_ADDR(idx),                               \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx)                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, pwm_rpi_init, NULL, NULL, &pwm_rpi_config_##idx, POST_KERNEL,   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &pwm_rpi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_RPI_INIT);