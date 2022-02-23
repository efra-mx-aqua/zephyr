/**
 * @file
 * @brief BME280 sensor API extension.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_BME280_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_BME280_H_

/**
 * Configure BME280 sensor with the same settings
 * as were written during initialization.
 */
int bme280_configure(const struct device *dev);

#endif
