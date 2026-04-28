/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief
*/
#ifndef BLE_RAWDATA_H
#define BLE_RAWDATA_H

#include "stdint.h"
#include "ring_tmp.h"
#include "ring_afe.h"
#include "acc\ring_acc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "protocol.h"
#include "acc\ring_acc.h"
#include "ble_service.h"
#include "app_timer.h"

struct ring_raw_data_header_s
{
	uint8_t  frame_id;
	uint8_t  frame_length;
};

struct ring_raw_data_tmp_frame_s
{
	uint8_t  tmp_msb;
	uint8_t  tmp_lsb;
};

struct ring_raw_data_ptt_frame_s
{
	uint8_t  green_msb;
	uint8_t  green_midb;
	uint8_t  green_lsb;
	uint8_t  ecg_msb;
	uint8_t  ecg_midb;
	uint8_t  ecg_lsb;
};

typedef struct monitorRawdata_s
{
	uint8_t  LED2_msb;      //infrared LED
	uint8_t  LED2_midb;
	uint8_t  LED2_lsb;
	
	uint8_t  LED3_msb;      //green LED
	uint8_t  LED3_midb;
	uint8_t  LED3_lsb;
	
	uint8_t  LED1_msb;      //red LED
	uint8_t  LED1_midb;
	uint8_t  LED1_lsb;
} monRawdata_t;

struct ring_raw_data_flag_s
{
	uint8_t enable;
	
	enum raw_data_start_word data_type;

	uint8_t writting:1;
	uint8_t flush:1;
	uint8_t none:6;
};

struct pulseimp_frame_s{
   uint8_t      green_msb;     //  PHASE3   ->    GREEN_LED
   uint8_t      green_midb;
   uint8_t      green_lsb;
   uint8_t      infra_msb;      //   PHASE1  ->    INFRARED_LED
   uint8_t      infra_midb;
   uint8_t      infra_lsb;
};

#pragma pack(1)
struct  ring_raw_data_pulseimp_frame_s{
        uint8_t    raw_type;
        uint8_t    frame_type;        //  0x06:   1 frame;          0x0C:     2 frame     
#define 			PULSEIMP_FRAME_SIZE				(2)
        struct  pulseimp_frame_s     pulseimp_frame[PULSEIMP_FRAME_SIZE];

//   (1+1)+(6+6)+6   = 20,    pack[6]  can be used to extend for other data structure
		uint8_t handon_flag;
        uint8_t   pack[5];
};
#pragma pack()


#if 0
#define RAW_PAYLOAD_LEN	238 // 247 - 3 - 6 (B0:type, B1:protocol, B2~B5:sent)
#define RAWDATA_MTU 244 // 247 - 3
#else
#define RAW_PAYLOAD_LEN	176 // 185 - 3 - 6 (B0:type, B1:protocol, B2~B5:sent)
#define RAWDATA_MTU 182 // 185 - 3

#endif

#define RAWDATA_OFFSET   (RAWDATA_MTU-RAW_PAYLOAD_LEN)

struct ring_raw_data_s
{
	struct ring_raw_data_flag_s  rdata_flag;	
	uint8_t elemt_size;
	uint8_t elemt_count;
	uint8_t raw_data[RAWDATA_MTU*10];
};

void copy_to_log(void *log_bpt, uint8_t length);
void copy_to_raw_data(enum raw_data_start_word rawType, void *sensor_data_bpt, uint8_t frame_length);

void ring_rawdata_init(void);
void ring_rawdata_start(void);
void ring_rawdata_stop(void);
bool is_enable_rawdata(void);

#endif


