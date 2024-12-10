#ifndef I2C_H
#define I2C_H



#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"


#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/i2c.h"


i2c_port_t i2c_get_port(void);
esp_err_t i2c_slave_init(i2c_port_t port, i2c_slave_config_t *config, i2c_slave_dev_handle_t *handle);
esp_err_t i2c_master_init(i2c_port_t port, i2c_master_bus_config_t *config, i2c_master_bus_handle_t *handle);

#endif // I2C_H
