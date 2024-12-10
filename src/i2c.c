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

#define I2C_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_SDA_IO 5
#define I2C_MPU6050_SLAVE_SCL_IO 5
#define I2C_MPU6050_SLAVE_SDA_IO 4
#define I2C_MCP4551_SLAVE_SCL_IO 22
#define I2C_MCP4551_SLAVE_SDA_IO 21



//master config
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};

i2c_master_bus_handle_t bus_handle;
ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

i2c_device_config_t mpu6050_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x60,
    .scl_speed_hz = 100000,
};

i2c_device_config_t mcp4551_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x58,
    .scl_speed_hz = 100000,
};
//slaves config

//MPU6050 config
i2c_slave_config_t i2c_slv_config_mpu6050 = {
    .addr_bit_len = I2C_ADDR_BIT_LEN_7,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .send_buf_depth = 256,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .slave_addr = 0x58,
};

//MCP4551 config
i2c_slave_config_t i2c_slv_config_mcp4551 = {
    .addr_bit_len = I2C_ADDR_BIT_LEN_7,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .send_buf_depth = 256,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .slave_addr = 0x58,
};


void app_main(void)
{
i2c_master_dev_handle_t dev_handle;
ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu6050_dev_cfg, &dev_handle));
ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mcp4551_dev_cfg, &dev_handle));

i2c_slave_dev_handle_t slave_handle;
ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config_mpu6050, &slave_handle));
ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config_mcp4551, &slave_handle));

}