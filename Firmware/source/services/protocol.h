/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * protocol.h
 * AUTHOR:zhao mengshou
 * DATE:2019/11/22 17:16:53
 *
 */

#ifndef _RING_PROTOCOL_H
#define _RING_PROTOCOL_H
#include <stdint.h>
#include "ring_utc.h"

#ifdef __cplusplus  
extern "C"
{
#endif

	typedef struct {
		union {
			struct {
				uint32_t byte0:8;		//LSB
				uint32_t byte1:8;
				uint32_t byte2:8;
				uint32_t byte3:8;		//MSB
			} field;
			uint32_t bytes;
		} value;
	} uint32_4bytes_t;

	typedef enum {
		//megaring private pair event
		PAIR_USE_NEW_RND_PARAM = 0,
		PAIR_USE_OLD_RND_PARAM,
		PAIR_REP_USER_CLICK,
		//PAIR_FORBIT_WHEN_LOWPR,
		//PAIR_WAIT_USR_CHOOSE,
		//PAIR_MEET_SOME_ERROR
	} pair_cmd;

	enum { /* I2C */
		MR_I2C_ENABLED = 1,
		MR_ACC_READ,
		MR_AFE_READ,
		MR_OPEN_PAIR_ISR,
		MR_PAIR_ISR_TIMEOUT,
		MR_ACC_INT1SRC_READ,
		MR_ACC_INT2SRC_READ
	};

	/*App <------> RING   cmd*/
	typedef enum
	{
        APP_GET_REPORTS_LIST        = 0x10, //get reports list, split to 2 packges, sn 0, 1, all data is 32 byte
        APP_GET_REPORT_BLOCK_CNT    = 0x11,
        APP_GET_REPORT_BLOCK        = 0x12,
        RING_SYNC_REPORT_BLOCK      = 0x14,
        APP_DELETE_REPORT           = 0x15,
        APP_SET_DEVICE_SHIP         = 0x17,
        APP_GET_CHIP_STATUS         = 0x20,

		APP_PAIR_CMD 				= 0xB0,
		APP_SET_DEVICE_SHUTDOWN 	= 0xB2,
		APP_SET_MAC_SN              = 0xB3,
		APP_SET_CHARGE_ENABLE       = 0xB9,

        APP_SET_PS_CMD              = 0xD0, //APP SET module to professional sleep module
		APP_GET_BATT_CMD 			= 0xD2,
		APP_KEEPALIVE_CMD 			= 0xD3,
        APP_SET_STRESS_CMD          = 0xD4,
		APP_SET_SPORT_CMD 			= 0xD5,
		APP_SET_DAILY_CMD			= 0xD6,
        RING_STOP_STRESS_CMD        = 0xD8,
		APP_SET_AUTO_MONITOR_CMD	= 0xD9,	// Set auto period of SpO2 monitor. APP_SET_AUTO_TIMER_MONITOR_CMD
        APP_SET_AUTO_STRESS_CMD     = 0xDA,
        APP_SET_LOW_POWER_CMD       = 0xDB,
        APP_SET_AIRPLANE_CMD        = 0xDC,
        APP_START_BP_CMD            = 0xDD, // start blood pressure module
        APP_HARD_RESET_RING_CMD     = 0xDE,
		
		APP_SET_TIME_CMD			= 0xE0,
		APP_GET_TIME_CMD			= 0xE1,
		APP_SET_DEVICE_RESET_CMD	= 0xE2,
		APP_SET_USER_INFO_CMD		= 0xE3,
		APP_GET_USER_INFO_CMD		= 0xEA,
		APP_GET_RT_DATA_CMD			= 0xED,
		APP_GET_RING_CRASH_DATA		= 0xF3,
		APP_GET_SYSTEM_SEC			= 0xF7,
	} E_RING_CMD_OPT;

	typedef enum
	{
		SET_MONITOR			= 0xA0,
		QUERY_MONITOR		= 0xA1,
		RECV_MONITOR_DATA	= 0xA2,

	} protocol_cmd_t;

	enum
	{
		SYNC_IDLE,
		SYNC_INDICATE_BEGAIN,
		SYNC_INDICATE_END,
		SYNC_NOTIFY_BEGAIN,
		SYNC_NOTIFY_RESEND,
		SYNC_NOTIFY_WAIT_RSP,
		SYNC_NOTIFY_END,
		SYNC_INDICATE_PAUSE,
		SYNC_INDICATE_OVER
	};

	//master device type
	enum
	{
		ANDROID_DEVICE	= 0x00,
		IOS_DEVICE		= 0x20,
		SA_DEVICE		= 0x40,
		NA_DEVICE		= 0x80, 	// something use simple command

		FAULT_DEVICE = 0xFF,
	};


	//ble ret_code_t
	typedef enum
	{
		BLE_SUCCESS = 0x00,
		BLE_NOTIFYING,
		BLE_INDICATING,
		BLE_NOTIFY_TIMEOUT,
		BLE_INDICATE_TIMEOUT,
		BLE_ERROR,
	}bleRspCode_t;


	/**************************************************************
	 *
	 *	Status 操作错误�
	 *	0x00-0x1f:	操作正确回复
	 *	0x20-0x9f:	APP端操�
	 *	0xa0-0xff:	设备端操作结�
	*/
	enum {
		CMD_SUCCESS			= 0x00,
        CMD_NO_RETURN       = 0x01,
		NO_RECORDS_DATA		= 0x02,

		SLEEPID_ERR			= 0x20, 	// sleepid err
		PARAM_NOT_RESOLVE	= 0x21, 	// 参数无法解析 

		RECORDS_CTRL_ERR	= 0x23, 	// 记录数据操作出错
		AFE_MONITORING		= 0x24, 	// AFE44XX检测记录已经开�
		AFE_SPORTING		= 0x25, 	// AFE44XX检测记录已经开�
		REPEATE_ERR			= 0x26, 	// 重复操作

		UNKNOWN_CMD			= 0x9F, 	// app cmd err

		BATTERY_IS_LOW		= 0xA1, 	// 设备低电

		FLASH_ERR			= 0xA3, 	// flash 操作失败
		OPERA_DISALLOW		= 0xA4, 	// operation disallowed

		AFE44XX_FAULT		= 0xA5, 	// afe44xx操作失败
		GSENSOR_FAULT		= 0xA6, 	// gsensor操作失败
		BQ25120_IS_FAULT	= 0xA7, 	// BQ25120故障

		NO_ALLY_REGISTER	= 0xA8,

        MODULE_SWITCH_ERR   = 0xC2,     // switch state module fault 
        REPORT_LIST_OVER    = 0xC3,     // get report list, group id overflow, 2 groups which 16 bytes.
        REPORT_IS_NULL      = 0xC4,
        REPORT_BLOCK_ERR    = 0xC5,
        REPORT_BLOCK_SYNC_ERR    = 0xC6,
        MODULE_NOT_ALLOW    = 0xC7,

		DEVICE_UNKNOWN_ERR	= 0xFF,
	};

	typedef enum {
		BLE_WRITE,
		BLE_WRITE_NO_RSP,
	} bleMasterOpcode_t;

	typedef enum {
		BLE_INDICATE,
		BLE_NOTIFY,
	} bleSlaveOpcode_t;
		

#define	BLE_MAX_SN_NUM				(16)
#define PDU_LENGTH					(19)
#define BLE_MAX_PACKETS_NUM			(107)

#define BLE_MAX_SECTOR_SIZE			(2048) // (BLE_MAX_PACKETS_NUM * PDU_LENGTH)

#define BLE_MAX_BUFFER_NUM			(BLE_MAX_PACKETS_NUM + 1)
#define APP_PAIR_TIMEOUT			30	// 30s pair timeout.


	typedef struct
	{
		uint8_t maxSec;
		uint32_t timeStart;
	} keepAlive_t;
	extern keepAlive_t keepAlive;

	typedef int16_t				 protocol_error_t;

#define PROTO_OK				(0)
#define PROTO_ERROR_OPERATION	(-1)
#define PROTO_ERROR_LENGTH		(-2)
#define PROTO_ERROR_MEMOUT		(-3)
#define PROTO_ERROR_EMPTY		(-4)


	typedef enum
	{
		BLUETOOTH_MSG,
		AT_COMMAND_MSG,
		SYSTEM_INTERNAL_MSG,

		SPECIAL_DEFINE_MSG = 0xFF	// user define special message struct
	} message_source_t;

	enum raw_data_start_word
	{
		NONE_WORD	= 0x5A,

		ACC_WORD	= 0x5B,
		GYRO_WORD	= 0x5C,
		IMU_WORD	= 0x5D,
		SPO2_WORD	= 0x5E,
		EHR_WORD	= 0x5F,
		PTT_WORD	= 0x60,
        SPO2_1_WORD = 0x61,
		
#define 			PULSEIMP_FRAME_TYPE				(0x0C)
		PULSE_WORD  = (SPO2_WORD + PULSEIMP_FRAME_TYPE)       //   = 0x6A 
	};

#if 0
	enum liveType {
		SPO2 = 1,
		EHR,
		DAILY,
		RTSHOW,
		BP,
		PULSE,
		TEMP
	};
#endif
	enum recordStatus {
		NO_RECORDING,
		RECORD_SPO2,
		RECORD_EHR,
		RECORD_TEMP,
		RECORD_BP
	};

	typedef struct _protocol_message_t
	{
		message_source_t source;
		void* message;
	} protocol_message_t;

	typedef struct
	{
		uint32_t time;
		uint8_t cmd;
		uint8_t status;
		uint8_t data[20]; // Like message.
	} st_bleRsp_t;
	

	protocol_error_t protocol_msg_push(message_source_t src, uint8_t* p_data, uint8_t length);
	protocol_error_t protocol_user_msg_push(message_source_t src, void* usr_msg);
	protocol_error_t protocol_msg_pop(protocol_message_t* msg);

	void proTaskProc(void* pMsg);

	void AlivePing(void);

	void sendLiveData(int);
	void sendRawdata(uint8_t rawType, uint8_t* buf, uint8_t len);

	void notifyPower(void);
	void bleProcess( uint8_t*);

	void autoMonitor_poll(void);

    uint8_t get_ble_msg_sn();

#ifdef __cplusplus  
}
#endif  
#endif /* _RING_PROTOCOL_H */  

