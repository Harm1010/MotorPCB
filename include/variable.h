#ifndef VARIABLE_H
#define VARIABLE_H

/**
 * @file variable.h
 * @brief The file contains all the variables' definitions which are used in the program.
 * @author Wataru Yoshida
 * @date 2021/10/19
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


/**
 * @brief The enum for the drive direction.
 */
typedef const enum{
    resvere,
    forward,
    neutral,
} drive_direction_t;

/**
 * @brief The struct for the control commands.
 */
struct control
{    
    drive_direction_t drive_mode;
    uint8_t throttle;

}control;

/**
 * @brief The struct for the sensor data.
 */
struct sensor_data
{   
    uint8_t mpu6050_deviceid;
    double odometer;   //cm
    double angle[3];  //degree
    mpu6050_acce_value_t acceleration; //mm/s^2
    uint16_t speed;  //mm/s
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t angles;
    mpu6050_temp_value_t temp; 

}sensor_data;

/**
 * @brief The struct for the CAN message.
 */
struct CAN_message
{
    uint32_t id;
    uint8_t data[8];

}CAN_message;

#endif