/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_afe.h
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#ifndef _RING_AFE_H  
#define _RING_AFE_H  

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "AFE4900\afe4900_defs.h"
#include "AFE4900\afe4900.h"

#ifdef __cplusplus  
extern "C"
{
#endif

	//       AFE4900 datasheet  
#define     AFE4900_FIFO_WORD_SIZE              (3) 

/*
*   FIFO setting ==> reg_addr: 0x42;  See <afe4900_defs.h> for more details about the setting 0f FIFO_PERIOD
*   Normal FIFO mode: the value of FIFO_PERIOD (periodicity of the FIFO write cycle) is set by the decimal value of REG_FIFO_PERIOD + 1.
*/
#define     AFE4900_SPO2_FIFO_PERIOD            SPO2_FIFO_PERIOD
#define     AFE4900_EHR_FIFO_PERIOD             EHR_FIFO_PERIOD
#define     AFE4900_DAILY_FIFO_PERIOD           DAILY_FIFO_PERIOD
#define     AFE4900_PTT_FIFO_PERIOD             PTT_FIFO_PERIOD
#define     AFE4900_FIFO_MAX_NPHASE             (4)      // max(PTT_NPHASE, PPG_NPHASE)   in which:  PTT_NPHASE = 2  RST_NPHASE = 4

/* PPG FIFO setting   */
#define     AFE4900_FIFO_SPO2_NPHASE            (4)
#define     AFE4900_FIFO_SPO2_LEN_BYTES         (AFE4900_SPO2_FIFO_PERIOD * ( AFE4900_FIFO_WORD_SIZE* AFE4900_FIFO_SPO2_NPHASE) )

// #define     AFE4900_FIFO_EHR_NPHASE             (2)
// #define     AFE4900_FIFO_EHR_LEN_BYTES          (AFE4900_EHR_FIFO_PERIOD * ( AFE4900_FIFO_WORD_SIZE* AFE4900_FIFO_EHR_NPHASE) )

#define     AFE4900_FIFO_EHR_NPHASE             (1)
#define     AFE4900_FIFO_EHR_LEN_BYTES          (AFE4900_EHR_FIFO_PERIOD * ( AFE4900_FIFO_WORD_SIZE* AFE4900_FIFO_EHR_NPHASE) )


#define     AFE4900_FIFO_DAILY_NPHASE             (1)
#define     AFE4900_FIFO_DAILY_LEN_BYTES          (AFE4900_DAILY_FIFO_PERIOD * ( AFE4900_FIFO_WORD_SIZE* AFE4900_FIFO_DAILY_NPHASE) )


/* PTT FIFO setting   */
#define     AFE4900_FIFO_PTT_NPHASE             (2)
#define     AFE4900_FIFO_PTT_LEN_BYTES          (AFE4900_PTT_FIFO_PERIOD * ( AFE4900_FIFO_WORD_SIZE* AFE4900_FIFO_PTT_NPHASE) )

	enum afe_downsample_rate_e
	{
		/*
		*   In our project, the value of AFE4900_FIFO_PERIOD or REG_FIFO_PERIOD is setting to 10, and the PRF Cycle is setting to 100Hz
		*	AFE_SAMPLE_TIME_MS  =  100ms    (responding by the interrupt function of AFE_ADCREADY_PIN pin)
		*   1s = 1000ms
		*   Thus, max(AFE_FREQUENCY) is about 10Hz.
		*/
		AFE_DOWNSAMPLE_NONE = 0,
		AFE_DOWNSAMPLE_1HZ = 10,
		AFE_DOWNSAMPLE_2HZ = 5,
		AFE_DOWNSAMPLE_5HZ = 2,
		AFE_DOWNSAMPLE_10HZ = 1,
		AFE_DOWNSAMPLE_INVALID = 11
	};

	typedef enum
	{
		RING_AFE_NONE_MODE = 0x00,
		RING_AFE_POWEROFF = (0x01<<0x00),
		RING_AFE_SPO2_MODE = (0x01<<0x01),
		RING_AFE_SPORT_MODE = (0x01<<0x02),
		RING_AFE_DAILY_MODE = (0x01<<0x03),
		RING_AFE_BP_MODE= (0x01<<0x04),
		RING_AFE_HRV_ON_MODE = (0x01<<0x05),
		RING_AFE_HRV_OFF_MODE = (0x01<<0x06),
		RING_AFE_MAX_MODE = (0x01<<0x07),
	} ring_afe_mode_t;

	//
#define      AFE_DEV_NONE     0
#define      AFE_DEV_OK       1
/*******   sensor data	by downsampling per second	  *********/
#define 	AFE_SAMPLE_NONEDATA		0
#define 	AFE_SAMPLE_NEWDATA		1
#define 	AFE_SAMPLE_USEDDATA		2

	struct afe_dev_info_s
	{
		int8_t    i2c_line;
		int8_t    fifordy_line;

		bool smp_data_used;
		struct   afe4900_sensor_data     sample_data;
		uint32_t app_sec;		/* uint32_t app_get_sec(void) */
		
		/*************       time  domain         ********************/
		int32_t    smp_period;
		int32_t    smp_times;
	};

#define 	AFE4900_I2C_ADDR         0x5B
#if AFE4900_SPO2_FIFO_PERIOD > AFE4900_PTT_FIFO_PERIOD
#define 	AFE4900_MAX_FIFO_PERIOD AFE4900_SPO2_FIFO_PERIOD
#else
#define 	AFE4900_MAX_FIFO_PERIOD AFE4900_PTT_FIFO_PERIOD
#endif
//#define     AFE_FIFO_MAX_LEN_BYTES	(AFE4900_MAX_FIFO_PERIOD * ( AFE4900_FIFO_WORD_SIZE * AFE4900_FIFO_MAX_NPHASE) ) 
//#define     AFE_FIFO_MAX_LEN_BYTES	(AFE4900_EHR_FIFO_PERIOD * ( AFE4900_FIFO_WORD_SIZE * AFE4900_FIFO_MAX_NPHASE) ) 
#define     AFE_FIFO_MAX_LEN_BYTES	(MODE_MAX_FIFO_PERIOD * ( AFE4900_FIFO_WORD_SIZE * AFE4900_FIFO_MAX_NPHASE) ) 


	struct ring_afe_s
	{
		/*********  control domain   ************/
		bool m_pwr;
		ring_afe_mode_t curMode;

		/*********   device domain	 ************/
		struct   afe4900_dev      				sensor;

		/*********   data domain     *************/
  
		uint8_t  afe_fifo_buff[AFE_FIFO_MAX_LEN_BYTES];
		struct   afe4900_fifo_frame  		 fifo_frame;
		//struct   afe4900_sensor_data      data[AFE4900_MAX_FIFO_PERIOD]; // 100Hz
		struct   afe4900_sensor_data      data[100]; // 100Hz 

	/*********    device information domain  *******/
		struct   afe_dev_info_s           dev_info;
	};

struct afe_signal_s
{
	uint16_t start;
	uint16_t read;
	uint16_t stop;
};
extern struct afe_signal_s afe_signal;

bool afe_start(ring_afe_mode_t mode);
bool afe_stop(void);
ring_afe_mode_t afe_get_work_mode(void);

	/*
	*  usage:
	*
	*   take PPG as a example:
	*
	*   the data memory mapping for PPG with 4 phase is as following show, in which symbol "+" stand for a phase.
	*   ------------------------------------
	*   |++++|++++|++++|++++|++++|++++|++++|
	*   ------------------------------------
	*   ^    ^    ^    ^    ^    ^    ^
	*   0    1   ...   x   ...  ...  (AFE4900_FIFO_PERIOD-1)
	*                b_pt       c_pt
	*                 |-- offset --|
	*  code:
	*
	*  int main()
	*  {
	*     	struct afe4900_sensor_data *test_bpt,  *test_cpt;
	*       int offset = 0;
	*       int green_pd, red_pd, ired_pd
	*       int16_t frame_length = AFE4900_FIFO_PERIOD;
	*	      read_afe_value(&test_bpt, x, &frame_length, AFE4900_READ_WRITE_LENGHT_INVALID);
	*       test_cpt = test_bpt + offset;
	*				green_pd = test_cpt->phase1;
	*  }
	*
	*/
	int8_t read_afe_value(struct afe4900_sensor_data** this_data, int16_t start_frame, int16_t* length, _Bool local_convert, ring_afe_mode_t currMode);
	struct afe_dev_info_s show_afe_dev_info(void);
	bool afe_is_new_data(void);
	void set_afe_fifordy_status(void);
	void set_afe_downsample_rate(enum afe_downsample_rate_e downsample_rate);
	int8_t get_afe_reg(uint8_t addr, uint32_t* val);
	int8_t set_afe_reg(uint8_t addr, uint32_t val);
int8_t set_afe_iled_format(uint8_t iled1, uint8_t iled2, uint8_t iled3);



#ifdef __cplusplus  
}
#endif  
#endif /* _RING_AFE_H */
