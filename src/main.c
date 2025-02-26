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
#include "driver/twai.h"
#include "unity.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "mpu6050.h"
#include "variable.h"
#include "can.h"

#define I2C_MASTER_SCL_IO GPIO_NUM_7    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_6    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define MCP4551_I2C_ADDRESS 0x2Eu
#define MCP4551_CMD_VOLATILE_WIPER 0x00u  // Command to write to the volatile wiper
#define MCP4551_CMD_NON_VOLATILE_WIPER 0x02u  // Command to write to the non-volatile wiper

static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;

// Define and initialize the global variable structs.
control_t control = {
    .drive_mode = neutral,
    .throttle = 0,
    .sdrive_enable = false
};

sensor_data_t sensor_data = {
    .odometer = 0.0,
    .speed = 0,
    .acceleration = {
        .acce_x = 0.0f,
        .acce_y = 0.0f,
        .acce_z = 0.0f
    },
    .gyro = {
        .gyro_x = 0.0f,
        .gyro_y = 0.0f,
        .gyro_z = 0.0f
    },
    .temp = {
        .temp = 0.0f
    },
    .angles = {
        .roll = 0.0f,
        .pitch = 0.0f
    }
};



/**
 * @brief Initializes the I2C bus
 *
 * This function is used to configure the I2C bus and install the driver.
 */
static void i2c_bus_init(void) {

    ESP_LOGI(TAG, "Configuring I2C bus...");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,  /*!< I2C mode: master or slave */
        .sda_io_num = I2C_MASTER_SDA_IO,  /*!< GPIO number for I2C data signal */
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  /*!< Enable internal pull-up on SDA line */
        .scl_io_num = I2C_MASTER_SCL_IO,  /*!< GPIO number for I2C clock signal */
        .scl_pullup_en = GPIO_PULLUP_ENABLE,  /*!< Enable internal pull-up on SCL line */
        .master.clk_speed = I2C_MASTER_FREQ_HZ  /*!< I2C master clock frequency */
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
    }
}


/**
 * @brief i2c master initialization and sensor mpu6050 initialization
 *
 * This function is used to initialize I2C master and create a sensor mpu6050 handle.
 * Then configure the sensor mpu6050 with the range of accelerometer and gyroscope.
 * Finally wake up the sensor mpu6050 and start to measure the values.
 */
static void i2c_sensor_mpu6050_init(void) {

    ESP_LOGI(TAG, "Initializing MPU6050...");

    ESP_LOGI(TAG, "Creating MPU6050 instance...");
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "MPU6050 instance creation failed");
        return;
    }

    ESP_LOGI(TAG, "Configuring MPU6050...");
    esp_err_t ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 configuration failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Waking up MPU6050...");
    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake-up failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "MPU6050 initialization complete.");
}


/**
 * @brief Writes a value to the specified MCP4551 register
 *
 * This function is used to send a command and value to the MCP4551
 * digital potentiometer. It creates an I2C command link, sends the
 * device address with write flag, command byte, and data byte, and
 * then stops the I2C communication. It also executes the command link
 * and deletes the command link when finished.
 *
 * @param i2c_address The I2C address of the MCP4551 device
 * @param command The command byte to send to the device, including
 *                the memory address and operation
 * @param value The data byte to send to the device (wiper value)
 *
 * @return ESP_OK if the function completed successfully, otherwise
 *         an error code from the ESP-IDF
 */
esp_err_t mcp4551_write(uint8_t i2c_address, uint8_t command, uint8_t value)
{
    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    // Start I2C communication
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("MCP4551", "I2C start failed: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    // Send the device address with write flag
    ret = i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) {
        ESP_LOGE("MCP4551", "I2C address write failed: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    // Send the command byte (with memory address and operation)
    ret = i2c_master_write_byte(cmd, command << 4, true);
    if (ret != ESP_OK) {
        ESP_LOGE("MCP4551", "Command byte write failed: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    // Send the data byte (wiper value)
    ret = i2c_master_write_byte(cmd, value, true);
    if (ret != ESP_OK) {
        ESP_LOGE("MCP4551", "Data byte write failed: %s", esp_err_to_name(ret));
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    // Stop I2C communication
    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("MCP4551", "I2C stop failed: %s", esp_err_to_name(ret));
    }

    // Execute the command link
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE("MCP4551", "I2C command execution failed: %s", esp_err_to_name(ret));
    }

    // Delete the command link
    i2c_cmd_link_delete(cmd);

    return ret;
}


/**
 * @brief mpu6050 task
 *
 * This task is used to read the mpu6050 data and print it.
 * It continuously reads the accelerometer and gyroscope data, and
 */
void mpu6050_task(void *arg) {

    i2c_sensor_mpu6050_init();

    while (1) {
        ESP_LOGI(TAG, "mpu6050 task");

        // Wait for the semaphore indicating data is ready
        // Read accelerometer data
        ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050, &sensor_data.acceleration));
        ESP_LOGI(TAG, "acce_x: %.2f, acce_y: %.2f, acce_z: %.2f",
             sensor_data.acceleration.acce_x,
             sensor_data.acceleration.acce_y,
             sensor_data.acceleration.acce_z);

        // Read gyroscope data
        ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050, &sensor_data.gyro));
        ESP_LOGI(TAG, "gyro_x: %.2f, gyro_y: %.2f, gyro_z: %.2f",
             sensor_data.gyro.gyro_x,
             sensor_data.gyro.gyro_y,
             sensor_data.gyro.gyro_z);

        // calculate angles
        ESP_ERROR_CHECK(mpu6050_complimentory_filter(mpu6050, &sensor_data.acceleration, &sensor_data.gyro, &sensor_data.angles));

        ESP_LOGI(TAG, "roll: %.2f, pitch: %.2f", 
             sensor_data.angles.roll,
             sensor_data.angles.pitch);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    /**
     * @brief Complementary filter
     *
     * This function applies the complementary filter to the data read from the
     * accelerometer and gyroscope. It returns the filtered roll and pitch angles.
     */
}


/**
 * @brief mcp4551 task
 *
 * This task is used to send control value to MCP4551 (digital potentiometer).
 * It adjusts the wiper value based on the throttle and drive mode.
 */
void mcp4551_task(void *arg) { 
    uint8_t wiper_value = 128; // Initial wiper value at midpoint
    control.drive_mode = neutral; // Initialize drive mode to neutral

    while (1) {
        // Limit throttle to a maximum value of 0xDC
        if(control.throttle > 0xDC){
            control.throttle = 0xDC;
        }

        // Determine wiper value based on drive mode and throttle
        switch (control.drive_mode) {
            case forward:
                // Increase wiper value for forward direction
                wiper_value = 127 + (control.throttle / 2);
                break;
            case resvere:
                // Decrease wiper value for reverse direction
                wiper_value = 127 - (control.throttle / 2);
                break;
            case neutral:
                // Set wiper value to midpoint for neutral
                wiper_value = 128;
                break;
        }

        // Write the calculated wiper value to the MCP4551 volatile wiper (digital potentiometer)
        mcp4551_write(MCP4551_I2C_ADDRESS, MCP4551_CMD_VOLATILE_WIPER, wiper_value);

        // Delay for 100ms before the next iteration
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief sdrive enable task
 *
 * This task is responsible for enabling/disabling the sdrive based on the
 * sdrive enable flag in the control struct. It also sets the GPIO 5 pin to
 * reflect the state of the sdrive.
 */
void sdrive_enable_task(void *arg) {

    // Configure GPIO 5 as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 5),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    while (1) {
            
            if (control.sdrive_enable) {
                gpio_set_level(GPIO_NUM_5, 1); // Turn GPIO 5 ON
                ESP_LOGI(TAG, "sdrive enabled, GPIO 5 ON");
            } else {
                gpio_set_level(GPIO_NUM_5, 0); // Turn GPIO 5 OFF
                ESP_LOGI(TAG, "sdrive disabled, GPIO 5 OFF");
            }


        // Delay for 100ms before the next iteration
        vTaskDelay(pdMS_TO_TICKS(60));
    }
}

/**
 * @brief Task to read ADC data and convert it to voltage
 *
 * This task reads data from the ADC1 unit on the ESP32 and converts the
 * raw data to a voltage value. The voltage is then scaled by 10 for the
 * resistor divider. The results are logged to the console.
 */
void adc_task(void *pvParameters)
{

    // Variables for ADC reading
    int adc_raw = 0;
    float voltage = 0;
    sensor_data.battery_voltage = 0;

    // ADC1 init
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC1 config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));


    while (1) {
        // Read ADC
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw));
        
        // Convert ADC reading to voltage (in mV)
        voltage = (float)(adc_raw * 3300) / 4095.f; // Assuming 12-bit ADC (4095) and 3.3V reference
        
        // Scale voltage by 10 for the resistor devider
        sensor_data.battery_voltage = voltage / 100.f;

        // Log the results
        ESP_LOGI(TAG, "ADC Raw: %d", adc_raw);
        ESP_LOGI(TAG, "Voltage: %f mV", voltage);
        ESP_LOGI(TAG, "Scaled Voltage: %f mV", sensor_data.battery_voltage);

        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Cleanup (this part won't be reached in this example)
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}

void app_main(void) {

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_config_t io_conf2 = {
        .pin_bit_mask = (1ULL << 10),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf2);
    gpio_set_level(GPIO_NUM_10, 0); // Turn GPIO 10 OFF
    
    i2c_bus_init();
    // Create the tasks
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 8, NULL);

    xTaskCreate(sdrive_enable_task, "sdrive_enable_task", 2048, NULL, 9, NULL);

    xTaskCreate(mcp4551_task, "mcp4551_task", 2048, NULL, 10, NULL);

    xTaskCreate(can_sensor_task, "can_sensor", 2048, NULL, 10, NULL);

    xTaskCreate(adc_task, "adc_task", 4096, NULL, 7, NULL);

    //vTaskDelay(10000 / portTICK_PERIOD_MS);
    //test_can();

}

