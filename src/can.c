
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
#define ID_CONTROL_MSG         0x103   // Incoming control messages
#define ID_MPU_ACCEL_MSG       0x204   // Outgoing accelerometer data
#define ID_MPU_GYRO_MSG        0x205   // Outgoing gyroscope data
#define ID_MPU_ANGLE_MSG       0x206   // Outgoing filtered angle data

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t stop_ping_sem;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t done_sem;

// TWAI messages
static const twai_message_t ping_message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = ID_MASTER_PING,
    .data_length_code = 0,
    .data = {0},
};

static const twai_message_t start_message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 0,                // Not single shot
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = ID_MASTER_START_CMD,
    .data_length_code = 0,
    .data = {0},
};

static const twai_message_t stop_message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 0,                // Not single shot
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = ID_MASTER_STOP_CMD,
    .data_length_code = 0,
    .data = {0},
};

/* --------------------------- Tasks and Functions -------------------------- */

void twai_receive_task(void *arg)
{
    while (1) {
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);

        if (action == RX_RECEIVE_PING_RESP) {
            //Listen for ping response from slave
            while (1) {
                twai_message_t rx_msg;
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_SLAVE_PING_RESP) {
                    xSemaphoreGive(stop_ping_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_RECEIVE_DATA) {
            //Receive data messages from slave
            uint32_t data_msgs_rec = 0;
            while (data_msgs_rec < NO_OF_DATA_MSGS) {
                twai_message_t rx_msg;
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_SLAVE_DATA) {
                    uint32_t data = 0;
                    for (int i = 0; i < rx_msg.data_length_code; i++) {
                        data |= (rx_msg.data[i] << (i * 8));
                    }
                    ESP_LOGI(EXAMPLE_TAG, "Received data value %"PRIu32, data);
                    data_msgs_rec ++;
                }
            }
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == RX_RECEIVE_STOP_RESP) {
            //Listen for stop response from slave
            while (1) {
                twai_message_t rx_msg;
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_SLAVE_STOP_RESP) {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

void twai_transmit_task(void *arg)
{
    while (1) {
        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_PINGS) {
            //Repeatedly transmit pings
            ESP_LOGI(EXAMPLE_TAG, "Transmitting ping");
            while (xSemaphoreTake(stop_ping_sem, 0) != pdTRUE) {
                twai_transmit(&ping_message, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(PING_PERIOD_MS));
            }
        } else if (action == TX_SEND_START_CMD) {
            //Transmit start command to slave
            twai_transmit(&start_message, portMAX_DELAY);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted start command");
        } else if (action == TX_SEND_STOP_CMD) {
            //Transmit stop command to slave
            twai_transmit(&stop_message, portMAX_DELAY);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted stop command");
        } else if (action == TX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action;
    rx_task_action_t rx_action;

    for (int iter = 0; iter < NO_OF_ITERS; iter++) {
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(EXAMPLE_TAG, "Driver started");

        //Start transmitting pings, and listen for ping response
        tx_action = TX_SEND_PINGS;
        rx_action = RX_RECEIVE_PING_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

        //Send Start command to slave, and receive data messages
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        tx_action = TX_SEND_START_CMD;
        rx_action = RX_RECEIVE_DATA;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

        //Send Stop command to slave when enough data messages have been received. Wait for stop response
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        tx_action = TX_SEND_STOP_CMD;
        rx_action = RX_RECEIVE_STOP_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
        vTaskDelay(pdMS_TO_TICKS(ITER_DELAY_MS));
    }
    //Stop TX and RX tasks
    tx_action = TX_TASK_EXIT;
    rx_action = RX_TASK_EXIT;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    //Delete Control task
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}

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
                control.sdrive_enable = rx_msg.data[1] & 0x01;
                control.drive_mode = (rx_msg.data[1] >> 1) & 0x03;
                
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
}


void test_can(void) {

    //Create tasks, queues, and semaphores
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    ctrl_task_sem = xSemaphoreCreateBinary();
    stop_ping_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);

    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    xSemaphoreGive(ctrl_task_sem);              //Start control task
    xSemaphoreTake(done_sem, portMAX_DELAY);    //Wait for completion

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vQueueDelete(rx_task_queue);
    vQueueDelete(tx_task_queue);
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(stop_ping_sem);
    vSemaphoreDelete(done_sem);
}

