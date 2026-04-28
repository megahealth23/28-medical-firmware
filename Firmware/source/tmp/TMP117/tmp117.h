/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file tmp117.h
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#ifndef _TMP117_H_
#define _TMP117_H_

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C" {
#endif


#include <stdbool.h>
#include "TMP117\tmp117_defs.h"

/*********************** User function prototypes ************************/

/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of TMP117 sensor.
 *
 *  @param[in,out] dev : Structure instance of tmp117_dev
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t tmp117_init(struct tmp117_dev *dev);

/*!
 * @brief This API configures the power mode, range and bandwidth
 * of sensor.
 *
 * @param[in] dev    : Structure instance of tmp117_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t tmp117_set_sens_conf(struct tmp117_dev *dev);

/*!
 * @brief This API reads sensor data, stores it in
 * the tmp117_get_sensor_data structure pointer passed by the user.
 * The user can ask for Temperature data. 
 *
 * @param[out] accel    : Structure pointer to store Temperature data
 * @param[in] dev       : Structure instance of tmp117_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int8_t tmp117_get_sensor_data(bool local_convert, struct tmp117_dev *dev);
/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif /* _TMP117_H_ */
/** @}*/
