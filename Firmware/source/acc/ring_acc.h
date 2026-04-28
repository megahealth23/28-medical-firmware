/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_acc.h
* @author:
* @modifing:li tianzheng
* @date 2020.07.29
* @version
* @brief
*/


#ifndef _RING_ACC_H  
#define _RING_ACC_H  

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "lis2dw12_reg.h"


#ifdef __cplusplus  
extern "C"
{
#endif

#define LIS2DW12_I2C_ADDR						LIS2DW12_I2C_ADD_H

#define ACC_SAMPLE_TIME_MS				320

#define ACC_UNIT						0.031
#define TAP_TIME_UNIT
#define TAP_TIME_LIMIT					3
#define TAP_LATENCY						14
#define TAP_WINDOW						30
#define TAP_THRESHOLD					1.2/ACC_UNIT //1.5/ACC_UNIT

#define WAKE_HIGH						45//50
#define WAKE_LOW						33


#define IMU_I2C_ADDR                    BMI160_I2C_ADDR // 0x6A LSM6DS3 
	
#define MIN_ODR(x, y)					(x < y ? x : y)
#define MAX_ODR(x, y)					(x > y ? x : y)
#define IMU_ODR_LSB_TO_HZ(_odr)			(_odr ? (13 << (_odr - 1)) : 0)

/***************************************
* structure used to acc
*/
typedef struct _acc_axis
{
	int16_t x;				//	asix_x of data raw acceleration
	int16_t y;				//	asix_y of data raw acceleration
	int16_t z;				//	asix_z of data raw acceleration
} acc_axis;

#define ACC_FIFOLEN 32	// 100Hz   Timer 320ms   320/10

typedef struct _sixaxis_fifo
{
	int n; 
	acc_axis axis[ACC_FIFOLEN];
} accFifo_t;

typedef enum
{
	IMU_NONE_MODE = 0x00,
	IMU_POWEROFF,
	IMU_STANDBY,
	IMU_STEPS,
	IMU_SPO2,
	IMU_EHR,
	IMU_PTT
//	IMU_GESTURE	
} ring_acc_mode_t;


enum acc_downsample_rate_e
{
/*  
*   in our project, ACC_SAMPLE_TIME_MS is setting to 320ms, in which is regularly read per 320ms.
*   1s = 1000ms  
*   Thus, max(ACC_FREQUENCY) is about 3Hz.	
*/   
	ACC_DOWNSAMPLE_NONE	= 0,
	ACC_DOWNSAMPLE_1HZ	= 3,
	ACC_DOWNSAMPLE_2HZ	= 2,
	ACC_DOWNSAMPLE_3HZ	= 1,
	
	ACC_DOWNSAMPLE_INVALID = 4
};

typedef enum {
    e_acc_IDLE = 0,
    e_acc_SPO2,
    e_acc_EHR,
    e_acc_STEP,
} E_ACC_PROC;
	
/**************************************/
#define      ACC_DEV_NONE     0
#define      ACC_DEV_OK       1
/******     sensor data  by downsampling per second    **********/	
#define 	ACC_SAMPLE_NONEDATA		0
#define 	ACC_SAMPLE_NEWDATA		1
#define 	ACC_SAMPLE_USEDDATA		2
/* 
*    BMI160  Xg=(9.81*raw)/sensitivity=(Grange*9.81*raw)/2^15
*	 So 4G range.  Xmg= 1000*4*9.81*raw/2^15= 9.81*raw/8192 = raw*1000/835= raw*0.122
*/
//#define FS4G_TO_MG(x) ((float_t)(x) * 122.0f / 1000.0f)
#define FS4G_TO_MG(x) ((int32_t)(x) * 122 / 1000)
#define FS2G_TO_MG(x) ((float_t)(x) * 61.0f / 1000.0f)
#define FS2000DPS_TO_MDPS(x) ((float_t)(x) * 70.0f)

struct lis2dw12_sensor_data
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;

};

struct acc_dev_info_s
{
	int8_t    i2c_line;
	int8_t    int1_line;
	int8_t    int2_line;

	int16_t max_fifo_len;

	bool smp_data_used;
		
	uint16_t sample_acc_g; 
	uint32_t app_sec;		/* uint32_t app_get_sec(void) */

        struct lis2dw12_sensor_data sample_data;
	
/*****     time  domain        *********/
	int32_t    smp_period;
	int32_t    smp_times;
};

struct ring_acc_s
{
/*******      factor test domain     ************/
	struct acc_dev_info_s        dev_info;
};
	
struct acc_signal_s
{
	uint16_t start;
	uint16_t read;
	uint16_t stop;
	uint16_t int1_flag;
	uint16_t int2_flag;
	uint16_t pair_isr;
};
extern struct acc_signal_s acc_signal;

typedef struct sensor_lis2dw12_s{
	uint8_t enable;
	lis2dw12_odr_t odr;
	lis2dw12_mode_t mode;
	lis2dw12_fs_t fs;
} lis2dw12_sensor_t;


/*********************************************************************
* FUNCTIONS
*/
ring_acc_mode_t acc_get_work_mode(void);
bool acc_reset(void);
bool acc_power_down(void);
bool acc_enable(void);

bool acc_config_wake_int(bool enable, uint8_t value);
bool is_wakeup_int(void);
bool acc_config_Tap(bool);
bool is_tap_int(void);

bool acc_start(ring_acc_mode_t);
bool acc_stop(void);


	
//	bool acc_config_double_tap(bool );
//	bool acc_config_single_tap(bool );
/*
* read acc data, 3-axis
*/
int8_t read_axis(E_ACC_PROC statu);

void set_acc_int1_status(void);
void set_acc_int2_status(void);
struct acc_dev_info_s show_acc_dev_info(void);
bool acc_is_new_data(void);
void set_acc_downsample_rate(enum acc_downsample_rate_e downsample_rate);
extern lis2dw12_sensor_t sensor;

#ifdef __cplusplus  
}
#endif  
#endif /* _RING_ACC_H */  

