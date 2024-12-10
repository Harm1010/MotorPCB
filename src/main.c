#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "unity.h"

#include "variable.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define MCP4551_I2C_ADDRESS 0x58

static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;

SemaphoreHandle_t data_ready_semaphore;


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

void IRAM_ATTR gpio_isr_handler(void *arg) {
    // Give the semaphore when the interrupt occurs
    xSemaphoreGiveFromISR(data_ready_semaphore, NULL);
}

void mpu6050_task(void *arg) {
    while (1) {
        ESP_LOGI(TAG, "mpu6050 task");
        // Wait for the semaphore indicating data is ready
        if (xSemaphoreTake(data_ready_semaphore, portMAX_DELAY)) {
            // Read accelerometer and gyroscope data
            ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050, &sensor_data.acceleration));
            ESP_LOGI(TAG, "acce_x: %.2f, acce_y: %.2f, acce_z: %.2f",
                     sensor_data.acceleration.acce_x,
                     sensor_data.acceleration.acce_y,
                     sensor_data.acceleration.acce_z);

            ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050, &sensor_data.gyro));
            ESP_LOGI(TAG, "gyro_x: %.2f, gyro_y: %.2f, gyro_z: %.2f",
                     sensor_data.gyro.gyro_x,
                     sensor_data.gyro.gyro_y,
                     sensor_data.gyro.gyro_z);

            // Optionally apply complementary filter
            ESP_ERROR_CHECK(mpu6050_complimentory_filter(mpu6050, &sensor_data.acceleration, &sensor_data.gyro, &sensor_data.angles));
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief mcp4551 task
 *
 * This task is used to send control value to MCP4551.
 */
// static void mcp4551_task(void *arg)
// {   
//     control.drive_mode = neutral;

//     uint8_t value;
//     while (1) {
//         switch (control.drive_mode) {
//             case forward:
//                 value = 128 + control.throttle;
//                 if (value > 255) {
//                     value = 255;
//                 }
//                 break;
//             case resvere:
//                 value = 128 - control.throttle;
//                 if (value < 0) {
//                     value = 0;
//                 }
//                 break;
//             case neutral:
//                 value = 128;
//                 break;
//         }
//         mcp4551_write(MCP4551_I2C_ADDRESS, value);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

    
void app_main(void) {
    // Create a semaphore
    data_ready_semaphore = xSemaphoreCreateBinary();
    if (data_ready_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    // Configure MPU6050 interrupt
    mpu6050_int_config_t mpu_int_config = {
        .interrupt_pin = GPIO_NUM_10,
        .pin_mode = INTERRUPT_PIN_OPEN_DRAIN,
        .active_level = INTERRUPT_PIN_ACTIVE_LOW,
        .interrupt_latch = INTERRUPT_LATCH_50US,
        .interrupt_clear_behavior = INTERRUPT_CLEAR_ON_ANY_READ
    };
    ESP_ERROR_CHECK(mpu6050_config_interrupts(mpu6050, &mpu_int_config));

    // Install GPIO interrupt handler
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Trigger on falling edge
        .pin_bit_mask = (1ULL << GPIO_NUM_10),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_10, gpio_isr_handler, NULL);



    //master config
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };



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
        .i2c_port = I2C_MASTER_NUM,
        .send_buf_depth = 256,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .slave_addr = 0x58,
    };

    //MCP4551 config
    i2c_slave_config_t i2c_slv_config_mcp4551 = {
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .send_buf_depth = 256,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .slave_addr = 0x58,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));   
    
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu6050_dev_cfg, &dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mcp4551_dev_cfg, &dev_handle));

    i2c_slave_dev_handle_t slave_handle;
    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config_mpu6050, &slave_handle));
    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config_mcp4551, &slave_handle));


    // Create the task
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 10, NULL);

}