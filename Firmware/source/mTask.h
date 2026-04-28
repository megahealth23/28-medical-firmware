/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * async_task.h
 * AUTHOR:zhao mengshou
 * DATE:2019/9/11 11:42:53
 * http://www.megahealth.cn
 *
 */

#ifndef MTASK_H
#define MTASK_H

 //  V8->zheng
#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
//


#ifdef __cplusplus  
extern "C"
{
#endif

	typedef struct _Event
	{
		uint16_t	evt;		// EventId
		void*		payload;	// payload point to the static Memory.
	} sEvent;

/************** SYS *********************/
#define     SYS_ONE_SECOND      1
#define     SYS_MODULE          2
#define     SYS_SHIP            3
#define     SYS_RESET           4

#define     SYS_FLASH_FULL      1
#define     SYS_BLE_DISCONNECT  2
#define     SYS_UPDATE_NI_FLASH 3
#define     SYS_UPDATE_NI_FLASH_LOCAL 4
#define     SYS_SWITCH_MODULE   5
#define     SYS_DEL_REPORT      6
#define     SYS_BLE_CONNECT     7
/************** SYS *********************/

	typedef void (*asyncCallback)(void*);
	//

	/***********    I2C    ******************/
#define 	ACC_READ					1
#define 	ACC_INT1SRC_READ			2
#define 	ACC_INT2SRC_READ			3
#define 	TMP0_ONE_SHOT_START			4          //  V8->zheng	
#define 	TMP1_ONE_SHOT_START			5          //  V8->zheng		
#define 	TMP0_READ					6          //  V8->zheng	
#define 	TMP1_READ					7          //  V8->zheng	
#define 	AFE_FIFORDY_READ        	8         //  V8->zheng		
	/*********** Factor test  ******************
	*  padload[15:0]:
	* 1) padload[15:12]:
	* 2) padload[11:8]:
	* 3) padload[8:4]:
	* 4) padload[3:0]:
	*******************/
#define		MAJOR_DEV_MASK				(0x0F<<12)
#define		MAJOR_DEV_ACC				(0x01<<12)
#define		MAJOR_DEV_AFE				(0x02<<12)
#define		MAJOR_DEV_TMP				(0x03<<12)
#define		MAJOR_DEV_BQ				(0x04<<12)

#define		MINOR_DEV_MASK				(0x0F<<8)
#define		SET_MINOR_DEV(x)			((x)<<8)     //  MINOR_DEV <=> padload[11:8], x=0,1,...,15
#define		GET_MINOR_DEV(x)			((uint8_t)(((x)&MINOR_DEV_MASK)>>8))     //  MINOR_DEV <=> padload[11:8], x=0,1,...,15

#define		OP_MODE_MASK				(0x0F<<4)    
#define		OP_MODE_START				(0x01<<4)    
#define		OP_MODE_STOP				(0x02<<4)     	
#define		OP_READ_SENSOR_DATA			(0x03<<4)     
#define		OP_READ_INT_STATUS			(0x04<<4)     

#define	    OP_PARM_MASK				(0x0F<<0)    
#define	    SET_OP_PARM(x)				((x)<<0)     
#define	    GET_OP_PARM(x)				((uint8_t)((x)&OP_PARM_MASK))     

	//#define		TEST_AFE_LED_PRE		1		  //  V8->zheng 
	//#define		TEST_AFE_PD_PRE 		2		  //  V8->zheng 
	//#define		TEST_AFE_LED			3		  //  V8->zheng 
	//#define		TEST_AFE_PD 			4		  //  V8->zheng 
	//#define		TEST_SHOW_BATT_INFO 	11		   //  V8->zheng	
	//#define		TEST_SHOW_TMP0_INFO 	12		   //  V8->zheng	
	//#define		TEST_SHOW_TMP1_INFO 	13		   //  V8->zheng		
	//#define		TEST_SHOW_ACC_INFO		14		   //  V8->zheng	
	//#define		TEST_SHOW_AFE_INFO		15		   //  V8->zheng	

	enum queueId
	{
		SYS_QID = 0X0001,		//one sec system wakeup

		I2C_QID,				//I2C operation queue identifier

		SPI_QID,				//SPI operation queue ID
		RTT_QID,				//RTT sample operation queue ID
		BLECmd_QID,			//protocol message operation queue ID

		FCTTst_QID,         ////factor test operation queue I

		/*****     should not beyond here       ************/
		MAX_QID
	};

	void taskRegister(uint16_t, asyncCallback);
	void taskUnregister(uint16_t);
	void taskInit(void);
	ret_code_t taskPush(sEvent*, bool);
	ret_code_t taskPop(sEvent*, bool);
	void taskProcess(void);

#ifdef __cplusplus  
}
#endif  
#endif /* APP_QUEUE_H */  

