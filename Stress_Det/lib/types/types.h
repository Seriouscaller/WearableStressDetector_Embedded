#pragma once
#include <stdio.h>

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;  
    uint16_t company_id; 
    uint16_t gsr;        
    uint16_t temp_raw;   
    uint32_t ppg_green;  
    int16_t acc_x;       
    int16_t acc_y;       
    int16_t acc_z;       
    int16_t gyr_x;       
    int16_t gyr_y;       
    int16_t gyr_z;       
} sensor_data_t;

typedef struct {
    uint32_t red_raw;
    uint32_t ir_raw;
    uint32_t green_raw; 
} max30101_data_t;

typedef struct {
    uint32_t temp;
} tmp117_data_t;

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z; 
} bmi160_data_t;


