#ifndef VARIABLE_H
#define VARIABLE_H

/**
 * @file variable.h
 * @brief The file contains all the variables' definitions which are used in the program.
 * @details This file contains the definitions of the variables used in the program. The variables include the control commands, sensor data, and CAN message.
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "mpu6050.h"


typedef enum {
    neutral,
    forward,
    resvere,
} drive_direction_t;

typedef struct {    
    uint8_t throttle;   // 0-255
    drive_direction_t drive_mode;
    bool sdrive_enable;
} control_t;

typedef struct {   
    double odometer;                    // cm
    uint16_t speed;                     // mm/s
    mpu6050_acce_value_t acceleration;  // mm/s^2
    mpu6050_gyro_value_t gyro;          
    mpu6050_temp_value_t temp;          // °C
    complimentary_angle_t angles;       // °
    float battery_voltage;             // V
} sensor_data_t;

// Declare variables as external
extern control_t control;
extern sensor_data_t sensor_data;


#endif