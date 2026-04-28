#ifndef  __INNER_FLASH_RAM_H__
#define  __INNER_FLASH_RAM_H__
#include "app_error.h"
#include "state_module_def.h"
#include "reports.h"
#include "ring_utc.h"
#include <sdk_common.h>
#include "ble_rawdata.h"

enum crashType {
  HARDDEFAULT_ERROR = 0,
  NRF_ERROR_RESET,
  LIBC_PRINTF_RESET,
  WATCHDOG_RESET,
  BQ_ERROR_RESET,
  ACC_ERROR_RESET,
  AFE_ERROR_RESET, 
  ACC_NOISR_RESET,
  AFE_NOISR_RESET,
  MEMORY_OVERFLOW // 9
};

#pragma pack(1)
typedef struct st_dump {
  uint32_t error_type;
  uint32_t time;
  uint32_t lr;
  uint32_t pc;
} dump_t;

typedef struct
{
  //eAutoStatus status;
  uint8_t peroid;
  uint8_t drv_on;
  uint32_t max_time;
  uint32_t curr_start_time;
  uint32_t next_start_time;
  uint32_t colding_time;
  UTCTimeStruct utc_time;
} autoMonitor_t;

typedef struct
{
  uint32_t lastTime;          // system tick,ϵͳ����ʱ��
  uint8_t data_version;       // data parse version
  uint8_t interval;           // unit : min

  uint8_t overide;
  int8_t valid;
  uint16_t wr_index;

  uint32_t refTime;
  uint32_t steps_ref;
} dailyRecord_t;

// RAM area where noinit
// RAM StartAddress: 0x2000 0000;
// Total RAM,256 KB = 256 * 1024 bytes = 262144 bytes = 0x40000 bytes.
// 0x2000 0000 + 0x40000 - 0x600 = 0x2003FA00
#define NIRAM_BASE 0x2003FA00
#define NIRAM_SIZE 0x600

#define SKIP_FT_STEP true

#define RECORDS_PERSON_DESP_LENGTH (5)

#define RID(x) user_desp->random_id[(x)]

#define RECORD_STEPS_MINS   15 //record how many minutes step in the list

typedef struct {
    uint8_t NIflag[4]; // NIRA 4

    uint32_t systme_sec; // 4
    uint32_t utc_sec;   //  4

    uint16_t sysStatus;      // System working status 2
    uint16_t recordStopType; // recordsErrType; 2

    uint32_t recordRunTime; //  4
    uint32_t keyIndex;  //  4
    uint32_t recordIndex;   //  4

    dump_t dump;    //16

    uint8_t cur_module; // 1
    uint8_t cur_sub_module;    //1
    uint16_t cur_sec_report_id; //2

    uint32_t cur_module_start_time; //4

    uint16_t last_sec_report_key;   //2
    uint16_t reserve10;//for align    //2

    uint16_t last_hrv_report_key;   //2
    uint16_t reserve7;//for align    //2

    uint8_t sec_report_ids[32]; //32

    uint16_t cur_daily_report_id;   //2
    uint16_t last_daily_report_key; //2

    uint16_t last_daily_report_hrv_key; //2
    uint8_t high_precision_mark; //0:open, 1:close   //1
    uint8_t reserve8;//for align    //1

    ST_REPORT_BUFF sec_report_buff; //514
    uint16_t reserve2;//for align   //2

    ST_REPORT_BUFF hrv_report_buff; //514
    uint16_t reserve3;//for align   //2

    ST_REPORT_BUFF daily_report_buff;   //514
    uint16_t reserve4;//for align   //2

    ST_REPORT_BUFF daily_hrv_report_buff;   //514
    uint16_t reserve0;//for align   //2

    uint8_t auto_sleep_mark; //1: on, 0: off    //1
    uint8_t auto_sleep_timezone_hour;    //1
    uint8_t auto_sleep_timezone_min;    //1
    uint8_t auto_sleep_start_hour;    //1

    uint8_t auto_sleep_start_min;    //1
    uint8_t auto_sleep_end_hour;    //1
    uint8_t auto_sleep_end_min;    //1
    uint8_t auto_stress_mark; //1: on, 0: off    //1

    uint32_t last_auto_sleep_close_time;    //4

    uint32_t step_in_day;   //4

    uint16_t step_in_min;   //2
    uint16_t last_second_steps; //2

    uint32_t last_total_steps;//4

    uint32_t steps_count_by_module;//4

    uint16_t steps_per_min_list[RECORD_STEPS_MINS]; //30
    uint8_t reserve9;//for align   //2
    uint8_t charged_percent;

    uint8_t _min_list_index;    //1
    uint8_t last_charge_percent;//for align    //1
    uint16_t step_in_15_min;    //2
    uint32_t last_start_time;
    uint32_t last_charge_time;
} NIRAM_t;

typedef struct
{
  uint8_t warn_enable;
  uint8_t warn_hr_min;
  uint8_t warn_hr_max;
  uint8_t warn_spo2_min;
} warn_data_t;

#define IS_USER_ID_VALID(id) (id[0]|id[1]|id[2]|id[3]|id[4]|id[5]|id[6]|id[7]|id[8]|id[9]|id[10]|id[11])

#define USRD_ID_LEN 12
typedef struct _user_descriptor {
    uint32_t time;                       // 4
    uint8_t user_id[USRD_ID_LEN];        // 12
    uint8_t random_id[BLE_GAP_ADDR_LEN]; // 6
    uint8_t age;
    uint8_t sex;
    uint8_t height;
    uint8_t weight;
    uint8_t step_size;
    uint8_t pair_rnd_type; // 6
    uint8_t user_num;      // Ring only records one user, this value reserved.
    uint8_t ring_type;
    uint8_t reserved1[2]; // for 4byte align
    warn_data_t warn_data;
} user_descriptor;
		
typedef struct {
    unsigned short year_5 : 5;
    unsigned short snVer : 3;
    unsigned short reserved : 3;
    unsigned short month : 4;
    unsigned short year_1 : 1;
} VerYyMm; 		// 2 bytes present ver|year|month

typedef	struct {
    unsigned char mark : 2;	
    unsigned char size : 3;	
    unsigned char type : 3;
} size_type;	

typedef struct {
    VerYyMm VYM;
    unsigned char serial[3];
    size_type SizeType;
} sn_t; //6 bytes.

#define 	NVUSER_DATA_SECTOR_COUNT 		(NVUD_SECTOR_ID_MAX)//the sector's count of the non-volatile user data  
#define 	NVUSER_DATA_SECTOR_SIZE 		(0x20)				 //the sector's size of the non-volatile user data.  consider to sizeof(ft_info_t) = 0x18

#define     INITIATIVE_HARD_RESET           2

typedef struct _productMacSn
{
    uint16_t header;	// "MR" 0x4D52
    uint16_t ftFlag;	// 6+6+6+2=20 bytes
    uint8_t mac[6];
    sn_t sn;
    sn_t oriSn;
    uint8_t pack0[2];
    uint16_t clibration[20];
} productMacSn;

typedef  struct st_ni_flash_data{
    user_descriptor user;

    uint32_t is_dfu;

    uint16_t cur_sec_report_id;
    uint16_t cur_daily_report_id;
    uint8_t sec_report_ids[32];
    uint16_t mark;
    uint8_t initative_hard_reset;
    uint8_t padding[3];
    uint16_t crc;
}ST_NI_FLASH_DATA;

typedef struct _ring_info_t
{
	uint8_t hwVer;			// Hardware
	uint8_t swVer;			// Firmware
	uint16_t rev;			// 2 bytes.SVN revision number= REVH<<8 | REVL
	uint8_t btVer;			// Bootloader version.
	sn_t	SN; 			// SN. 6 bytes. yy\mm\serial3\size:5&type:3 	
	uint8_t sensorStatus;	// Bitfield 1ok,0error. TMP117:AFE:GS:I2C
	uint8_t recordStatus;	// Obsolete.
	uint8_t power;			// Percentage of power.
	uint8_t padding[6]; 	// Above 20 bytes are ring info replied to the app.

	uint8_t peerMac[6]; // Peer MAC The app.
	uint8_t Mac[6]; 	// Ring MAC. Read from UICR 
	uint8_t ringSize;

} RingInfo_t;	

#pragma pack()

void init_NI_flash();
void initNIRAM();
void SaveCrash(struct st_dump *dp);
void updateNIClock();
void update_NI_flash_CMD();
void update_NI_flash_local_CMD();

#endif