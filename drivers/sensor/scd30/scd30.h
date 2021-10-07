#ifndef ZEPHYR_DRIVER_SENSOR_SCD30_SCD30_H_
#define ZEPHYR_DRIVER_SENSOR_SCD30_SCD30_H_

#include <device.h>
#include <kernel.h>
#include <drivers/gpio.h>

#define SCD30_CMD_START_PERIODIC_MEASUREMENT 0x0010
#define SCD30_CMD_STOP_PERIODIC_MEASUREMENT 0x0104
#define SCD30_CMD_READ_MEASUREMENT 0x0300
#define SCD30_CMD_SET_MEASUREMENT_INTERVAL 0x4600
#define SCD30_CMD_GET_DATA_READY 0x0202
#define SCD30_CMD_SET_TEMPERATURE_OFFSET 0x5403
#define SCD30_CMD_SET_ALTITUDE 0x5102
#define SCD30_CMD_SET_FORCED_RECALIBRATION 0x5204
#define SCD30_CMD_AUTO_SELF_CALIBRATION 0x5306
#define SCD30_CMD_READ_SERIAL 0xD033
#define SCD30_SERIAL_NUM_WORDS 16
#define SCD30_WRITE_DELAY_US 20000

#define SCD30_MEASUREMENT_DATA_WORDS 6

#define SCD30_CRC8_POLYNOMIAL 0x31
#define SCD30_CRC8_INIT 0xFF
#define SCD30_CRC8_LEN 1

#define SCD30_WORD_SIZE 2
#define SCD30_COMMAND_SIZE 2

#define SCD30_MAX_BUFFER_WORDS 24
#define SCD30_CMD_SINGLE_WORD_BUF_LEN \
    (SCD30_COMMAND_SIZE + SCD30_WORD_SIZE + SCD30_CRC8_LEN)

struct scd30_config
{
    char *bus_name;
    uint8_t base_address;

};

struct scd30_data
{
    uint16_t error;
    const struct device* dev;
    const struct device* bus;
    char serial[SCD30_SERIAL_NUM_WORDS +1];
    struct sensor_value co2_ppm;
    struct sensor_value temp;
    struct sensor_value rel_hum;
};

int scd30_write_command(const struct device *dev, uint16_t cmd);
int scd30_write_register(const struct device *dev, uint16_t cmd, uint16_t val);
static inline uint8_t scd30_i2c_address(const struct device *dev);

int scd30_check_crc(uint8_t *data, uint8_t data_len, uint8_t checksum);
uint8_t scd30_generate_crc(uint8_t *data, uint8_t data_len);

#endif /* ZEPHYR_DRIVER_SENSOR_SCD30_SCD30_H_ */