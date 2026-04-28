/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file HRI.h
* @author:li tianzheng
* @modifing:
* @date 2020.0.0
* @version
* @brief Human-Ring Interaction
*/
#ifndef _HUMAN_RING_INTERACTION_H_
#define _HUMAN_RING_INTERACTION_H_
 
#ifdef __cplusplus  
extern "C"
{
#endif 

#include "ring_handonled.h"

#define	CURRENT_STATUS_NONE 					0
#define CURRENT_STATUS_CHARGING 			1
#define CURRENT_STATUS_CHARGEFULL 		2
#define CURRENT_STATUS_NORMAL					3
	
void ring_topled_on(void);
void ring_topled_off(void);
void ring_charging_indicate(void);
void ring_charging_led_on(void);
void ring_charging_led_off(void);
void ring_chargefull_indicate(void);
void ring_findme_indicate(void);
void ring_pair_indicate(void);
void ring_stop_indicate(void);
#ifdef __cplusplus  
}
#endif  
#endif /* _HUMAN_RING_INTERACTION_H_ */
