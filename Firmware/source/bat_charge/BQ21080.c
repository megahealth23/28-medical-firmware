#include "BQ21080.h"

#include "I2C.h"
#include "mTask.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

bool bq21080_i2c_read(uint8_t reg_addr, uint8_t* p_data){
  ret_code_t rslt;

  rslt = i2c_read(BQ21080_I2C_ADDR_7B, reg_addr, p_data, 1);
  return (int8_t)rslt;
}

bool bq21080_i2c_write(uint8_t reg_addr, uint8_t* p_data){
  int8_t rslt = 0;
  uint8_t writebuf[4];
	
  writebuf[0] = reg_addr;		//重要
  memcpy(writebuf+1, p_data, 1);
	
  rslt = i2c_write(BQ21080_I2C_ADDR_7B, reg_addr, writebuf, (uint8_t)2);
  return rslt;
}

bool bq21080_get_mask_id(uint8_t* p_mask_id){
  return bq21080_i2c_read(BQ21080_REG_MASK_ID, p_mask_id);
}

bool bq21080_init() {
  uint8_t mask_id = 0;
  uint8_t rslt = 0;

  rslt = bq21080_get_mask_id(&mask_id);

  NRF_LOG_INFO("bq21080_init = 0x%02X,  rslt=%d", mask_id, rslt);

  if ((0 == rslt) && (0xC0U == mask_id))
    return true;
  else
    return false;
}