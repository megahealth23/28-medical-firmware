/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * system_clock.h
 * AUTHOR:
 * DATE:
 * http://www.megahealth.cn
 *
 */

#ifndef _SYSTEM_CLOCK_H 
#define _SYSTEM_CLOCK_H
#include <stdint.h>

//#define MSEC_FUCNTION_ENABLE

#ifdef __cplusplus
extern "C"
{
#endif

	void rtc_start(void);
	void rtc_stop(void);

	void app_set_sec(uint32_t sec);
	uint32_t app_get_sec(void);
	uint32_t app_get_ticks(void);

#ifdef MSEC_FUCNTION_ENABLE
	uint32_t app_get_msec(void);
#endif


#ifdef __cplusplus  
}
#endif  
#endif /* _SYSTEM_CLOCK_H */  

