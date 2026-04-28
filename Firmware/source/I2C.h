/* Copyright (c) 2016 MegaHealth. All Rights Reserved.
 * I2C.h
 * AUTHOR:
 * DATE:
 *
 */

#ifndef __I2C_H__
#define __I2C_H__

#include "app_error.h"

bool i2c_init(void);
bool i2c_uninit(void);
//bool i2c_write(uint8_t device_address, uint8_t register_address, uint8_t* value, uint8_t number_of_bytes);
//bool i2c_read(uint8_t device_address, uint8_t register_address, uint8_t* destination, uint8_t number_of_bytes);
ret_code_t i2c_write(uint8_t , uint8_t , uint8_t* value, uint8_t );
ret_code_t i2c_read(uint8_t , uint8_t , uint8_t* value, uint8_t );


#endif /* __I2C_H__ */
