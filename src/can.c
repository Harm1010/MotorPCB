
/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a master node in a TWAI network. The master
 * node is responsible for initiating and stopping the transfer of data messages.
 * The example will execute multiple iterations, with each iteration the master
 * node will do the following:
 * 1) Start the TWAI driver
 * 2) Repeatedly send ping messages until a ping response from slave is received
 * 3) Send start command to slave and receive data messages from slave
 * 4) Send stop command to slave and wait for stop response from slave
 * 5) Stop the TWAI driver
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "can.h"
#include "variable.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define PING_PERIOD_MS          250
#define NO_OF_DATA_MSGS         50
#define NO_OF_ITERS             3
#define ITER_DELAY_MS           1000
#define RX_TASK_PRIO            8
#define TX_TASK_PRIO            9
#define CTRL_TSK_PRIO           10
#define TX_GPIO_NUM             GPIO_NUM_8
#define RX_GPIO_NUM             GPIO_NUM_20
#define EXAMPLE_TAG             "TWAI Master"

#define ID_MASTER_STOP_CMD      0x0A0
#define ID_MASTER_START_CMD     0x0A1
#define ID_MASTER_PING          0x0A2
#define ID_SLAVE_STOP_RESP      0x0B0
#define ID_SLAVE_DATA           0x0B1
#define ID_SLAVE_PING_RESP      0x0B2

#define TAG "CAN_SENSOR"

// CAN message IDs
#define ID_CONTROL_MSG         0x123   // Incoming control messages
#define ID_MPU_ACCEL_MSG       0x204   // Outgoing accelerometer data
#define ID_MPU_GYRO_MSG        0x205   // Outgoing gyroscope data
#define ID_MPU_ANGLE_MSG       0x206   // Outgoing filtered angle data



/* --------------------------- Tasks and Functions -------------------------- */


void can_sensor_task(void *arg)
{   

    twai_message_t rx_msg = {0};
    twai_message_t tx_msg = {0};
    
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");
    
    while (1) {
        // Receive control messages
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (rx_msg.identifier == ID_CONTROL_MSG) {
                // Parse control message
                control.throttle = rx_msg.data[0];
                control.sdrive_enable = rx_msg.data[1];
                control.drive_mode = rx_msg.data[2];
                
                ESP_LOGI(TAG, "Received control - Mode: %d, Throttle: %d, Enable: %d",
                    control.drive_mode, control.throttle, control.sdrive_enable);
            }
        }

        static long last_send_time = 0;
        long curr_time = xTaskGetTickCount();
        if ((curr_time - last_send_time) >= 1000) {
            last_send_time = curr_time;

            // Send accelerometer data
            tx_msg.identifier = ID_MPU_ACCEL_MSG;
            tx_msg.data_length_code = 6;
            
            // Pack accelerometer data (2 bytes each for x, y, z)
            int16_t accel_x = (int16_t)(sensor_data.acceleration.acce_x * 100); // Scale by 100 for 2 decimal places
            int16_t accel_y = (int16_t)(sensor_data.acceleration.acce_y * 100);
            int16_t accel_z = (int16_t)(sensor_data.acceleration.acce_z * 100);
            
            tx_msg.data[0] = (uint8_t)(accel_x >> 8);
            tx_msg.data[1] = (uint8_t)(accel_x);
            tx_msg.data[2] = (uint8_t)(accel_y >> 8);
            tx_msg.data[3] = (uint8_t)(accel_y);
            tx_msg.data[4] = (uint8_t)(accel_z >> 8);
            tx_msg.data[5] = (uint8_t)(accel_z);
            
            ESP_LOGI(TAG, "Sending accelerometer data - x: %d, y: %d, z: %d", accel_x, accel_y, accel_z);
            twai_transmit(&tx_msg, pdMS_TO_TICKS(100));

            // Send gyroscope data
            tx_msg.identifier = ID_MPU_GYRO_MSG;
            tx_msg.data_length_code = 6;
            
            // Pack gyroscope data (2 bytes each for x, y, z)
            int16_t gyro_x = (int16_t)(sensor_data.gyro.gyro_x * 100);
            int16_t gyro_y = (int16_t)(sensor_data.gyro.gyro_y * 100);
            int16_t gyro_z = (int16_t)(sensor_data.gyro.gyro_z * 100);
            
            tx_msg.data[0] = (uint8_t)(gyro_x >> 8);
            tx_msg.data[1] = (uint8_t)(gyro_x);
            tx_msg.data[2] = (uint8_t)(gyro_y >> 8);
            tx_msg.data[3] = (uint8_t)(gyro_y);
            tx_msg.data[4] = (uint8_t)(gyro_z >> 8);
            tx_msg.data[5] = (uint8_t)(gyro_z);
            
            ESP_LOGI(TAG, "Sending gyroscope data - x: %d, y: %d, z: %d", gyro_x, gyro_y, gyro_z);
            twai_transmit(&tx_msg, pdMS_TO_TICKS(100));

            // Send filtered angle data
            tx_msg.identifier = ID_MPU_ANGLE_MSG;
            tx_msg.data_length_code = 4;
            
            // Pack angle data (2 bytes each for roll and pitch)
            int16_t roll = (int16_t)(sensor_data.angles.roll * 100);
            int16_t pitch = (int16_t)(sensor_data.angles.pitch * 100);
            
            tx_msg.data[0] = (uint8_t)(roll >> 8);
            tx_msg.data[1] = (uint8_t)(roll);
            tx_msg.data[2] = (uint8_t)(pitch >> 8);
            tx_msg.data[3] = (uint8_t)(pitch);
            
            ESP_LOGI(TAG, "Sending filtered angle data - roll: %d, pitch: %d", roll, pitch);
            twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
        }

    }
    vTaskDelay(pdMS_TO_TICKS(10));
}



