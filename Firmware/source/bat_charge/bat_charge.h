#ifndef _BAT_CHARGE_H_  
#define _BAT_CHARGE_H_

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"


#ifdef __cplusplus  
extern "C"
{
#endif


bool bat_charge_start();

bool bat_charge_stop();

#ifdef __cplusplus  
}
#endif  
#endif /* _RING_ACC_H */  

