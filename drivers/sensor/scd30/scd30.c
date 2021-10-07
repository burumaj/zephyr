/*
 * Copyright (c) 2021 Advanced Climate Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT sensirion_scd30

#include <device.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(SCD30, CONFIG_SCD30_LOG_LEVEL);

#include "scd30.h"

/**
 * @brief check the crc from src. If the crc is good
 * 		copy the data word to dst
 * 
 */
#define SCD30_COPY_DATA_RETURN_ERR(src, dst)    \
	rc = scd30_fill_data_buf(rx_buf, data_buf); \
	if (rc != 0)                                \
	{                                           \
		return -EIO;                            \
	}


static const struct scd30_config scd30_0_cfg = {
	.bus_name = DT_INST_BUS_LABEL(0),
	.base_address = DT_INST_REG_ADDR(0),
};

struct scd30_data scd30_0_data;

static inline const struct device *scd30_i2c_device(const struct device *dev)
{
	const struct scd30_data *ddp = dev->data;

	return ddp->bus;
}

static inline uint8_t scd30_i2c_address(const struct device *dev)
{
	const struct scd30_config *dcp = dev->config;

	return dcp->base_address;
}

/**
 * @brief Take the received  wordt (16 bit), if the data is valid, add it to dst
 * 
 * @param data data received from sensor
 * @param dst buffer to store data
 * @return int 
 */
static int scd30_fill_data_buf(uint8_t *data, uint8_t *dst)
{
	if (scd30_check_crc(data, 2, data[2]))
	{
		return -EIO;
	}

	dst[0] = data[0];
	dst[1] = data[1];

	return 0;
}

uint16_t scd30_bytes_to_uint16_t(const uint8_t* bytes) {
    return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

uint32_t scd30_bytes_to_uint32_t(const uint8_t* bytes) {
    return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
           (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

float scd30_bytes_to_float(const uint8_t* bytes) {
    union {
        uint32_t u32_value;
        float float32;
    } tmp;

    tmp.u32_value = scd30_bytes_to_uint32_t(bytes);
    return tmp.float32;
}

static int scd30_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	uint8_t rx_buf[6 * (SCD30_WORD_SIZE + SCD30_CRC8_LEN)] = {};
	uint8_t data_buf[6 * SCD30_WORD_SIZE] = {};

	struct scd30_data *data = dev->data;
	int rc = 0;

	rc = scd30_write_command(dev, SCD30_CMD_GET_DATA_READY);
	if (rc != 0)
	{
		return -EIO;
	}

	rc = i2c_read(
		scd30_i2c_device(dev),
		rx_buf, SCD30_WORD_SIZE + SCD30_CRC8_LEN,
		scd30_i2c_address(dev));

	SCD30_COPY_DATA_RETURN_ERR(rx_buf, data);

	uint16_t data_ready = scd30_bytes_to_uint16_t(data_buf);
	if (!data_ready)
	{
		return -ENODATA;
	}

	rc = scd30_write_command(dev, SCD30_CMD_READ_MEASUREMENT);
	if (rc != 0)
	{
		LOG_DBG("Failed to send command. (rc = %d)", rc);
		return rc;
	}

	// delay for 3 msec as per datasheet.
	k_busy_wait(3000);

	rc = i2c_read(
		scd30_i2c_device(dev),
		rx_buf, ARRAY_SIZE(rx_buf),
		scd30_i2c_address(dev));

	if (rc != 0)
	{
		LOG_DBG("Failed to read data. (rc = %d)", rc);
		return rc;
	}

	/* C02 data */
	SCD30_COPY_DATA_RETURN_ERR(rx_buf, data_buf);
	SCD30_COPY_DATA_RETURN_ERR(rx_buf + 3, data_buf + 2);

	/* temperature data */
	SCD30_COPY_DATA_RETURN_ERR(rx_buf + 6, data_buf + 4);
	SCD30_COPY_DATA_RETURN_ERR(rx_buf + 9, data_buf + 6);

	/* relative humidity */
	SCD30_COPY_DATA_RETURN_ERR(rx_buf + 12, data_buf + 8);
	SCD30_COPY_DATA_RETURN_ERR(rx_buf + 15, data_buf + 10);

	float co2_ppm, temp, rel_hum;
	co2_ppm =	scd30_bytes_to_float(data_buf);
	temp = 		scd30_bytes_to_float(data_buf + 4);
	rel_hum = 	scd30_bytes_to_float(data_buf + 8);

	sensor_value_from_double(&data->co2_ppm, co2_ppm);
	sensor_value_from_double(&data->temp, temp);
	sensor_value_from_double(&data->rel_hum, rel_hum);

	return 0;
}

static int scd30_channel_get(const struct device *dev,
							 enum sensor_channel chan,
							 struct sensor_value *val)
{
	struct scd30_data *data = dev->data;
	switch (chan)
	{
	case SENSOR_CHAN_CO2:
		memcpy(val, &data->co2_ppm, sizeof(struct sensor_value));
		break;

	case SENSOR_CHAN_AMBIENT_TEMP:
		memcpy(val, &data->temp, sizeof(struct sensor_value));
		break;

	case SENSOR_CHAN_HUMIDITY:
		memcpy(val, &data->rel_hum, sizeof(struct sensor_value));
		break;

	default:
		return -1;
	}
	return 0;
}

static const struct sensor_driver_api scd30_driver_api = {
	.sample_fetch = scd30_sample_fetch,
	.channel_get = scd30_channel_get,
};

int scd30_write_command(const struct device *dev, uint16_t cmd)
{
	uint8_t tx_buf[2] = {cmd >> 8, cmd & 0xFF};

	return i2c_write(
		scd30_i2c_device(dev),
		tx_buf, sizeof(tx_buf),
		scd30_i2c_address(dev));
}

static uint8_t scd30_compute_crc(uint8_t *data, uint8_t data_len)
{
	uint16_t current_byte;
    uint8_t crc = SCD30_CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < data_len; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ SCD30_CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

int scd30_check_crc(uint8_t *data, uint8_t data_len, uint8_t checksum)
{
	uint8_t actual_crc = scd30_compute_crc(data, data_len);
	
	if (checksum != actual_crc)
	{
		LOG_DBG("CRC check failed");
		return -EIO;
	}

	return 0;
}

int scd30_write_register(const struct device *dev, uint16_t cmd, uint16_t val)
{
	uint8_t tx_buf[SCD30_CMD_SINGLE_WORD_BUF_LEN];

	tx_buf[0] = cmd >> 8;
	tx_buf[1] = cmd & 0xFF;
	tx_buf[2] = val >> 8;
	tx_buf[3] = val & 0xFF;
	tx_buf[4] = scd30_compute_crc(tx_buf, sizeof(tx_buf));

	return i2c_write(
		scd30_i2c_device(dev),
		tx_buf, ARRAY_SIZE(tx_buf),
		scd30_i2c_address(dev));
}

static int scd30_init(const struct device *dev)
{
	LOG_DBG("Initializing SCD30");
	struct scd30_data *data = dev->data;
	const struct scd30_config *cfg = dev->config;
	const struct device *i2c = device_get_binding(cfg->bus_name);

	if (i2c == NULL)
	{
		LOG_DBG("Failed to get pointer to %s device!", log_strdup(cfg->bus_name));
		return -EINVAL;
	}
	data->bus = i2c;

	if (!cfg->base_address)
	{
		LOG_DBG("No I2C address");
		return -EINVAL;
	}
	data->dev = dev;

	LOG_DBG("Starting periodic measurements");
	int rc = scd30_write_command(dev, SCD30_CMD_START_PERIODIC_MEASUREMENT);
	if (rc != 0)
	{
		return -EIO;
	}

	LOG_DBG("Setting measurement interval");
	rc = scd30_write_register(dev, SCD30_CMD_SET_MEASUREMENT_INTERVAL, CONFIG_SCD30_MEASUREMENT_INTERVAL);

	return rc;
}

DEVICE_DT_INST_DEFINE(0, scd30_init, device_pm_control_nop,
					  &scd30_0_data, &scd30_0_cfg,
					  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
					  &scd30_driver_api);
