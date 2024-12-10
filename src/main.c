#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "variable.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;


static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

/**
 * @brief i2c master initialization and sensor mpu6050 initialization
 *
 * This function is used to initialize I2C master and create a sensor mpu6050 handle.
 * Then configure the sensor mpu6050 with the range of accelerometer and gyroscope.
 * Finally wake up the sensor mpu6050 and start to measure the values.
 */
static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    // Initialize the I2C bus
    i2c_bus_init();

    // Create an MPU6050 sensor handle
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    // Configure the MPU6050 sensor with accelerometer and gyroscope settings
    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    // Wake up the MPU6050 sensor
    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/**
 * @brief mpu6050 task
 *
 * This task is used to read the mpu6050 data and print it.
 */
static void mpu6050_task(void *arg)
{
    esp_err_t ret;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    while (1) {
        ret = mpu6050_get_acce(mpu6050, &acce);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

        ret = mpu6050_get_gyro(mpu6050, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        ret = mpu6050_get_temp(mpu6050, &temp);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "t:%.2f \n", temp.temp);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


/**
 * @brief mcp4551 task
 *
 * This task is used to send control value to MCP4551.
 */
static void mcp4551_task(void *arg)
{
    uint8_t value;
    while (1) {
        switch (control.drive_mode) {
            case forward:
                value = 128 + control.throttle;
                if (value > 255) {
                    value = 255;
                }
                break;
            case resvere:
                value = 128 - control.throttle;
                if (value < 0) {
                    value = 0;
                }
                break;
            case neutral:
                value = 128;
                break;
        }
        i2c_mcp4551_write(MCP4551_I2C_ADDRESS, value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void app_main() {
    
    
mpu6050_int_config_t mpu_int_config = {
    
    .interrupt_pin = GPIO_NUM_10,
    .pin_mode = INTERRUPT_PIN_OPEN_DRAIN,
    .active_level = INTERRUPT_PIN_ACTIVE_LOW,
    .interrupt_latch = INTERRUPT_LATCH_50US,
    .interrupt_clear_behavior = INTERRUPT_CLEAR_ON_ANY_READ
};

i2c_sensor_mpu6050_init();


ESP_ERROR_CHECK(mpu6050_config_interrupts(mpu6050, &mpu_int_config));

ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050, &sensor_data.acceleration));

ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050, &sensor_data.gyro));

ESP_ERROR_CHECK(mpu6050_complimentory_filter(mpu6050, &sensor_data.acceleration, &sensor_data.gyro, &sensor_data.angles));


}