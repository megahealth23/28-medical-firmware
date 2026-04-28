#ifndef  _BQ21080_H_
#define  _BQ21080_H_

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus  
extern "C"
{
#endif

#define BQ21080_I2C_ADDR_7B 0x6A
#define BQ21080_I2C_ADDR_8B 0xD4

#define BQ21080_REG_MASK_ID 0xCU

bool bq21080_init();

bool bq21080_i2c_read(uint8_t reg_addr, uint8_t* p_data);

bool bq21080_i2c_write(uint8_t reg_addr, uint8_t* p_data);

bool bq21080_get_mask_id(uint8_t* p_mask_id);


#ifdef __cplusplus  
{
#endif

#endif
