#ifndef __STATE_MODULE_H__
#define __STATE_MODULE_H__

#include "state_module_def.h"
#include "inner_flash_ram.h"
#include "ring_afe.h"

#define MODULE_FUNC_DEF(_module)                    \
    ret_code_t _module##_init();                    \
    void _module##_uninit(E_REPORT_END_TYPE e_end); \
    uint8_t _module##_type();                       \
    void _module##_process_sys(void* pMsg);         \
    void _module##_process_i2c(void* pMsg);         \
    bool _module##_process_ble(uint8_t *pR, uint8_t *out);   \
    void _module##_set_adv_data(uint8_t *data);
    
extern uint8_t accval;
extern NIRAM_t NIramCfg;
extern uint32_t s_sys_module_evt;
extern uint32_t s_sys_report_evt;

MODULE_FUNC_DEF(IDLE)
MODULE_FUNC_DEF(LP)
MODULE_FUNC_DEF(CHARG)
MODULE_FUNC_DEF(LP)
MODULE_FUNC_DEF(PA)
MODULE_FUNC_DEF(PS)
MODULE_FUNC_DEF(DS)
MODULE_FUNC_DEF(EXER)
MODULE_FUNC_DEF(DAILY)
MODULE_FUNC_DEF(REAL)
MODULE_FUNC_DEF(STRESS)
MODULE_FUNC_DEF(LPS)
MODULE_FUNC_DEF(BP)
//MODULE_FUNC_DEF(base_module)

#endif