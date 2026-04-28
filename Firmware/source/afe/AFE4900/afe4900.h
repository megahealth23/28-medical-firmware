/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_afe.h
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#ifndef _AFE4900_H_
#define _AFE4900_H_

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "AFE4900\afe4900_defs.h"

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
int8_t afe4900_init(struct afe4900_dev *dev);

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
int8_t afe4900_set_sens_conf(struct afe4900_dev *dev);



int8_t afe4900_set_fifo_flush(struct afe4900_dev *dev);



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
int8_t afe4900_get_fifo_data(struct afe4900_dev *dev);

int8_t afe4900_extract_sensor_data(struct afe4900_sensor_data *sensor_data, uint16_t *frame_length, bool local_convert, struct afe4900_dev *dev);

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif /* _TMP117_H_ */
/** @}*/




