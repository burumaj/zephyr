/*
 * Copyright (c) 2022 Joep Buruma
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT raspberrypi_pico_dmac

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_rpi_pico, CONFIG_DMA_LOG_LEVEL);

/* pico-sdk includes */
#include <hardware/dma.h>
#include <hardware/regs/dma.h>
#include <hardware/regs/dreq.h>
#include <hardware/structs/dma.h>

#define DMA_DEBUG_OFFSET (0x800)
#define DMA_RPI_PICO_NUM_DREQS (39U)

#define DMA_CH_CTRL_CHANNEL_ENABLE_BIT	LSB_GET(DMA_CH0_CTRL_TRIG_EN_BITS)
#define DMA_CH_CTRL_STATUS_MASK		DMA_CH0_CTRL_TRIG_BUSY_BITS
#define DMA_CH_CTRL_BUSY_MASK		DMA_CH0_CTRL_TRIG_BUSY_BITS
#define DMA_CH_CTRL_INCR_READ_MASK	DMA_CH0_CTRL_TRIG_INCR_READ_BITS
#define DMA_CH_CTRL_INCR_WRITE_MASK	DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS
#define DMA_CH_CTRL_TREQ_SEL_MASK	DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS
#define DMA_CH_CTRL_CHAIN_TO_MASK	DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS
#define DMA_CH_CTRL_AHB_ERROR_MASK	DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS
#define DMA_CH_CTRL_DATA_SIZE_MASK	DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS

typedef void(dma_irq_configure_t)(const struct device *dev);

struct dma_rpi_pico_data {
	struct dma_context ctx;
	struct dma_config channels[NUM_DMA_CHANNELS];
};

struct dma_rpi_pico_cfg {
	dma_hw_t *dma_base;
	dma_debug_hw_t *dma_debug;
	dma_irq_configure_t *irq0_config_func;
	dma_irq_configure_t *irq1_config_func;
};

static inline void dma_rpi_pico_isr_common(const struct device *dev, uint8_t irq_nr)
{
	struct dma_rpi_pico_data *data = dev->data;
	const struct dma_rpi_pico_cfg *cfg = dev->config;
	int channel_error = 0;

	for (int i = 0; i < NUM_DMA_CHANNELS; i++) {
		/* description of hardware registers */
		dma_channel_hw_t *dma_hw_channel_cfg = &cfg->dma_base->ch[i];
		struct dma_config *dma_channel_cfg = &data->channels[i];

		if (dma_irqn_get_channel_status(irq_nr, i) && dma_channel_cfg != NULL) {
			/* clear the interrupt for this dma channel */
			dma_irqn_acknowledge_channel(irq_nr, i);

			/* channel's not configured */
			if (dma_channel_cfg == NULL || dma_channel_cfg->dma_callback == NULL) {
				continue;
			}

			/* reading from a trigger register does not start dma channel */
			channel_error = FIELD_GET(DMA_CH_CTRL_AHB_ERROR_MASK, dma_hw_channel_cfg->ctrl_trig);

			if (dma_channel_cfg->error_callback_en && channel_error) {
				dma_channel_cfg->dma_callback(dev, dma_channel_cfg->user_data, i, channel_error);
			} else if (dma_channel_cfg->complete_callback_en){
				dma_channel_cfg->dma_callback(dev, dma_channel_cfg->user_data, i, 0);
			}
		}
	}
}

static void dma_rpi_isr0(const struct device *dev)
{
	dma_rpi_pico_isr_common(dev, 0);
}

static void dma_rpi_isr1(const struct device *dev)
{
	dma_rpi_pico_isr_common(dev, 1);
}

static int dma_rpi_pico_configure(const struct device *dev, uint32_t channel,
				  struct dma_config *ch_cfg)
{
	struct dma_rpi_pico_data *data = dev->data;
	const struct dma_rpi_pico_cfg *cfg = dev->config;
	dma_channel_hw_t *hw_channel;
	uint32_t chan_ctrl = 0;

	enum dma_channel_transfer_size transfer_size;

	if (channel > NUM_DMA_CHANNELS) {
		return -EINVAL;
	}

	if (ch_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		return -EINVAL;
	}
	chan_ctrl |= FIELD_PREP(DMA_CH_CTRL_INCR_READ_MASK, 
			ch_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT);


	if (ch_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		return -EINVAL;
	}
	chan_ctrl |= FIELD_PREP(DMA_CH_CTRL_INCR_WRITE_MASK,
			ch_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT);

	if (ch_cfg->dma_slot > DMA_RPI_PICO_NUM_DREQS) {
		return -EINVAL;
	}

	/* for mem 2 mem transfer always force */
	/* TODO: Add option to use pacing timers */
	if (ch_cfg->channel_direction == MEMORY_TO_MEMORY) {
		chan_ctrl |= FIELD_PREP(DMA_CH_CTRL_TREQ_SEL_MASK, 
				DREQ_FORCE);
	} else {
		chan_ctrl |= FIELD_PREP(DMA_CH_CTRL_TREQ_SEL_MASK, 
				ch_cfg->dma_slot);
	}

	if (ch_cfg->source_chaining_en) {
		return -EINVAL;
	} else if (ch_cfg->dest_chaining_en) {
		chan_ctrl |= FIELD_PREP(DMA_CH_CTRL_CHAIN_TO_MASK, 
				ch_cfg->linked_channel);
	} else {
		/* Link to own channel to disable triggering other channels. */
		chan_ctrl |= FIELD_PREP(DMA_CH_CTRL_CHAIN_TO_MASK, 
				channel);
	}

	switch (ch_cfg->dest_data_size) {
	case sizeof(uint8_t):
		transfer_size = DMA_SIZE_8;
		break;
	case sizeof(uint16_t):
		transfer_size = DMA_SIZE_16;
		break;
	case sizeof(uint32_t):
		transfer_size = DMA_SIZE_32;
		break;
	default:
		return -EINVAL;
	}
	chan_ctrl |= FIELD_PREP(DMA_CH_CTRL_DATA_SIZE_MASK, transfer_size);

	hw_channel = &cfg->dma_base->ch[channel];

	hw_channel->read_addr = ch_cfg->head_block->source_address;
	hw_channel->write_addr = ch_cfg->head_block->dest_address;
	hw_channel->transfer_count = ch_cfg->head_block->block_size;

	/* write to an alias of ctrl register to not trigger dma transfer */
	hw_channel->al1_ctrl = chan_ctrl | DMA_CH_CTRL_CHANNEL_ENABLE_BIT;

	data->channels[channel] = *ch_cfg;

	return 0;
}

static int dma_rpi_pico_reload(const struct device *dev, uint32_t channel, uint32_t src,
			       uint32_t dst, size_t size)
{
	const struct dma_rpi_pico_cfg *cfg = dev->config;
	dma_channel_hw_t *hw_channel;

	if (channel > NUM_DMA_CHANNELS) {
		return -EINVAL;
	}
	hw_channel = &cfg->dma_base->ch[channel];

	hw_channel->read_addr = src;
	hw_channel->write_addr = dst;
	hw_channel->transfer_count = size;

	return 0;
}

static int dma_rpi_pico_start(const struct device *dev, uint32_t channel)
{
	const struct dma_rpi_pico_cfg *cfg = dev->config;

	if (channel > NUM_DMA_CHANNELS) {
		return -EINVAL;
	}

	/* enable channel interupt */
	WRITE_BIT(cfg->dma_base->inte0, channel, 1);

	/* write to the control register to trigger the first dma transfer. */
	WRITE_BIT(cfg->dma_base->ch[channel].ctrl_trig, DMA_CH_CTRL_CHANNEL_ENABLE_BIT, 1);
	return 0;
}

static int dma_rpi_pico_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_rpi_pico_cfg *cfg = dev->config;

	if (channel > NUM_DMA_CHANNELS) {
		return -EINVAL;
	}

	WRITE_BIT(cfg->dma_base->abort, channel, 1);
	while (cfg->dma_base->abort & BIT(channel)) {
		/* wait till abort bit is cleared */
	}

	/* disable interrupt for channel */
	WRITE_BIT(cfg->dma_base->inte0, channel, 0);
	WRITE_BIT(cfg->dma_base->inte1, channel, 0);

	return 0;
}

static int dma_rpi_pico_get_status(const struct device *dev, uint32_t channel,
				   struct dma_status *status)
{
	const struct dma_rpi_pico_cfg *cfg = dev->config;
	struct dma_rpi_pico_data *data = dev->data;

	if (channel > NUM_DMA_CHANNELS) {
		return -EINVAL;
	}

	status->busy = FIELD_GET(DMA_CH_CTRL_STATUS_MASK, cfg->dma_base->ch[channel].al1_ctrl);
	status->dir = data->channels[channel].channel_direction;
	status->pending_length = cfg->dma_base->ch[channel].transfer_count;
	return 0;
}

static int dma_rpi_pico_init(const struct device *dev)
{
	const struct dma_rpi_pico_cfg *cfg = dev->config;

	// disable all channel interrupts
	cfg->dma_base->inte0 = 0;
	cfg->dma_base->inte1 = 0;

	cfg->irq0_config_func(dev);
	cfg->irq1_config_func(dev);
	return 0;
}

static struct dma_driver_api dma_rpi_pico_api = {
	.config = dma_rpi_pico_configure,
	.start = dma_rpi_pico_start,
	.stop = dma_rpi_pico_stop,
	.reload = dma_rpi_pico_reload,
	.get_status = dma_rpi_pico_get_status,
};

#define RPI_DMA_IRQ_CONFIG_INIT(inst, n) .irq##n##_config_func = dma_##inst##_irq##n##_config_func

#define RPI_PICO_DMA_INIT(inst)                                                                    \
	static void dma_##inst##_irq0_config_func(const struct device *dma)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, dma_irq_0, irq),                             \
			    DT_INST_IRQ_BY_NAME(inst, dma_irq_0, priority), dma_rpi_isr0,          \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, dma_irq_0, irq));                             \
	}                                                                                          \
                                                                                                   \
	static void dma_##inst##_irq1_config_func(const struct device *dma)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, dma_irq_1, irq),                             \
			    DT_INST_IRQ_BY_NAME(inst, dma_irq_1, priority), dma_rpi_isr1,          \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, dma_irq_1, irq));                             \
	}                                                                                          \
                                                                                                   \
	static struct dma_rpi_pico_data dma_rpi_pico_data_##inst = {                               \
                                                                                                   \
	};                                                                                         \
                                                                                                   \
	static const struct dma_rpi_pico_cfg dma_rpi_pico_config_##inst = {                        \
		.dma_base = (dma_hw_t *)DT_INST_REG_ADDR(inst),                                    \
		.dma_debug = (dma_debug_hw_t *)(DT_INST_REG_ADDR(inst) + DMA_DEBUG_OFFSET),        \
		RPI_DMA_IRQ_CONFIG_INIT(inst, 0),                                                  \
		RPI_DMA_IRQ_CONFIG_INIT(inst, 1),                                                  \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &dma_rpi_pico_init, NULL, &dma_rpi_pico_data_##inst,           \
			      &dma_rpi_pico_config_##inst, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY, \
			      &dma_rpi_pico_api);

DT_INST_FOREACH_STATUS_OKAY(RPI_PICO_DMA_INIT);
