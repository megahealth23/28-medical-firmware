#ifndef __STATE_MODULE_MANAGER_H__
#define __STATE_MODULE_MANAGER_H__
#include "state_module_def.h"
#include "reports_def.h"

ret_code_t state_modules_init();
ret_code_t state_modules_switch(E_STATE_MODULE e_module, E_REPORT_END_TYPE e_end);
void state_modules_uninit();
void set_adv_scan_rsp_data(uint8_t* data);

void state_modules_SYS_process(void *pMsg);
void state_modules_I2C_process(void *pMsg);
bool state_modules_BLE_process(uint8_t *pR, uint8_t *out);

bool switch_module(E_STATE_MODULE module, uint8_t end_type);

E_STATE_MODULE cur_module_type();

#endif