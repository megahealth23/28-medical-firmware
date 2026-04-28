/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_tmp.c
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#include "ring_bsp.h"
#include "ring_tmp.h"
#include "app_timer.h"
#include "I2C.h"
#include "mTask.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "system_clock.h"

APP_TIMER_DEF(m_read_tmp117_timer);

#define TEMP_INTERVAL_MS		1000 //interval_ms
#define FAST_TEMP_INTERVAL_MS		5000 //interval_ms

struct tmp_signal_s tmp0_signal;
struct tmp_signal_s tmp1_signal;

struct ring_tmp_s  moudle_tmp = 
{
	.addr     = TMP117_I2C_ADDR_GRP,
	.curMode  = {RING_TMP_NONE_MODE/**/, RING_TMP_NONE_MODE/**/},
	.m_pwr    = {false/**/, false/**/},

	.dev_info =
    {
		{
			.smp_data_used = TMP_SAMPLE_NONEDATA,
			.i2c_line = TMP_DEV_NONE,
			.smp_period  = TMP_DOWNSAMPLE_NONE,
			.smp_times  = 0,
		},
				
		{
			.smp_data_used = TMP_SAMPLE_NONEDATA,
			.i2c_line = TMP_DEV_NONE,
			.smp_period  = TMP_DOWNSAMPLE_NONE,
			.smp_times  = 0,
		}
	},
	.timer_running = false,
	.interval_ms = TEMP_INTERVAL_MS,
};

static void read_tmp117_handler(void* p_context)
{
	UNUSED_PARAMETER(p_context);
	sEvent tx_evt;
	
	uint32_t tmp0_sum = 0;
	uint32_t tmp1_sum = 0;

	tx_evt.evt = I2C_QID;
	tmp0_signal.start = (MAJOR_DEV_TMP | SET_MINOR_DEV(RING_OUTSIDE_TMP117) | OP_MODE_START | SET_OP_PARM(RING_TMP_ONE_SHOT_MODE));
	tmp1_signal.start = (MAJOR_DEV_TMP | SET_MINOR_DEV(RING_INSIDE_TMP117) | OP_MODE_START | SET_OP_PARM(RING_TMP_ONE_SHOT_MODE));
	tmp0_signal.read = ( MAJOR_DEV_TMP | SET_MINOR_DEV(RING_OUTSIDE_TMP117) | OP_READ_SENSOR_DATA );
	tmp1_signal.read = ( MAJOR_DEV_TMP | SET_MINOR_DEV(RING_INSIDE_TMP117) | OP_READ_SENSOR_DATA );
	
	//NRF_LOG_INFO("read_tmp117_handler");
#define TEMP_DETECT_INTERVAL  60000 /*300000*/
	switch(moudle_tmp.current_status)
	{
		case TMP117_MEASURING_REPEAT_START:
			//NRF_LOG_INFO("TMP117_MEASURING_REPEAT_START");

			tx_evt.payload = (void*)&tmp1_signal.start;
			taskPush(&tx_evt, false);			
			
			app_timer_start(m_read_tmp117_timer, APP_TIMER_TICKS(500), NULL);
			moudle_tmp.current_status = TMP117_MEASURING_REPEAT_GETTING_RESULTS;		
			break;

		case TMP117_MEASURING_REPEAT_GETTING_RESULTS:
			//NRF_LOG_INFO("TMP117_MEASURING_REPEAT_GETTING_RESULTS");		

			if(	TMP117_DEV_ONE_SHOT_MODE == moudle_tmp.sensor[1].dev_mode ) {
				tx_evt.payload = (void*)&tmp1_signal.read;
				taskPush(&tx_evt, false);
			}	
		
			app_timer_start(m_read_tmp117_timer, APP_TIMER_TICKS(moudle_tmp.interval_ms - 500), NULL);
			moudle_tmp.current_status = TMP117_MEASURING_REPEAT_START;					
			break;		
			
	}
}


bool tmp117_measure_init(bool create_timer, bool start_timer, measure_mode_t mode)
{
	ret_code_t err_code;

	if(	create_timer )	{
		err_code = app_timer_create(&m_read_tmp117_timer, APP_TIMER_MODE_SINGLE_SHOT, read_tmp117_handler);
		APP_ERROR_CHECK(err_code);
	}		

	if(start_timer)
	{
		app_timer_stop(m_read_tmp117_timer);
		
		if( MEASURE_MODE_MONITOR == mode)
		{
			moudle_tmp.interval_ms = 1000; 
			err_code = app_timer_start(m_read_tmp117_timer, APP_TIMER_TICKS(1000), NULL);
			APP_ERROR_CHECK(err_code);
		}else if(MEASURE_MODE_PTT_MONITOR == mode){
			moudle_tmp.interval_ms = FAST_TEMP_INTERVAL_MS; // test 30000
			err_code = app_timer_start(m_read_tmp117_timer, APP_TIMER_TICKS(1000), NULL);
			APP_ERROR_CHECK(err_code);
		}else
		{
			moudle_tmp.interval_ms = TEMP_INTERVAL_MS; // test 30000
			err_code = app_timer_start(m_read_tmp117_timer, APP_TIMER_TICKS(1000), NULL);
			APP_ERROR_CHECK(err_code);
		}
		moudle_tmp.current_status = TMP117_MEASURING_REPEAT_START;
		moudle_tmp.timer_running = true;
	}

	return true;
}
bool tmp117_measure_deinit(void)
{
	if(moudle_tmp.timer_running)	{
			app_timer_stop(m_read_tmp117_timer);
			moudle_tmp.timer_running = false;
	}
	return true;
}

/*******************************/
/* Body temperature range is [300, 450] */
/* Refer to GB/T 21416-2008 Clinical electronic thermometer range is [350, 410]*/
/*******************************/
//uint8_t gCurrent_body_temperature()
//{
//    //return 1;
////	NRF_LOG_INFO("interval_ms:<%d>,dev_id:%d, temp_val:%d", moudle_tmp.interval_ms, dev_id, moudle_tmp.data[dev_id].temp_val);
//    double f = moudle_tmp.data[RING_INSIDE_TMP117].celcius * 10;
//    uint16_t temp = round(f) - 300; 
//    if(temp > 150){
//        temp = 150;
//    }

//	return  temp&0x00FF;
//}

uint8_t gCurrent_body_temperature()
{
    //return 1;
//	NRF_LOG_INFO("interval_ms:<%d>,dev_id:%d, temp_val:%d", moudle_tmp.interval_ms, dev_id, moudle_tmp.data[dev_id].temp_val);
    double f = moudle_tmp.data[RING_INSIDE_TMP117].celcius * 10;
    uint16_t temp = round(f) > 200 ? round(f) - 200 : 0; 
    if(temp > 250){
        temp = 250;
    }

	return  temp&0x00FF;
}

static void TMP117_delay_ms(uint32_t period)
{
	// delay time
	nrf_delay_ms(period);
} // user_delay_ms()

static int8_t TMP117I2Cwrite(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
	int8_t rslt = 0;

	uint8_t writebuf[4];
	if (len > 3)
	{
		return -1;
	}
	writebuf[0] = reg_addr;     //MUST
	memcpy(writebuf + 1, reg_data, len);

	rslt = i2c_write(dev_id, reg_addr, writebuf, (uint8_t)len + 1);
	return rslt;
}

static int8_t TMP117I2Cread(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
	ret_code_t rslt;
	rslt = i2c_read(dev_id, reg_addr, reg_data, len);
	return (int8_t)rslt;

}

static void TMP117_init(int8_t dev_id)
{
		int8_t rslt = TMP117_OK;
		moudle_tmp.sensor[dev_id].read = TMP117I2Cread;
		moudle_tmp.sensor[dev_id].write = TMP117I2Cwrite;
		moudle_tmp.sensor[dev_id].delay_ms = TMP117_delay_ms;
		moudle_tmp.sensor[dev_id].id = moudle_tmp.addr[dev_id];
		moudle_tmp.sensor[dev_id].sensor_data = &moudle_tmp.data[dev_id];
		rslt = tmp117_init(&(moudle_tmp.sensor[dev_id]));
		
		if( (TMP117_OK == rslt) && (0x0117 == moudle_tmp.sensor[dev_id].chip_id))
				moudle_tmp.dev_info[dev_id].i2c_line = TMP_DEV_OK;
		//NRF_LOG_INFO("id = %d, tmp117_sensor = %d,  rslt=%d", dev_id, moudle_tmp.sensor[dev_id].chip_id, rslt);
}

void TMP117_unInit(int8_t dev_id)
{	
	moudle_tmp.sensor[dev_id].delay_ms = NULL;
	moudle_tmp.sensor[dev_id].read = NULL;
	moudle_tmp.sensor[dev_id].write = NULL;
	moudle_tmp.sensor[dev_id].sensor_data = NULL;
}

static bool tmp_set_mode(ring_tmp_mode_t mode, int8_t dev_id)
{
	int8_t rslt = TMP117_OK;

	switch (mode)
	{
		case RING_TMP_SHUTDOWN_MODE:
			tmp117_measure_deinit();
			moudle_tmp.sensor[dev_id].dev_mode = TMP117_DEV_SHUTDOWN_MODE;
			NRF_LOG_INFO("SET RING_TMP_SHUTDOWN_MODE\r");
			break;

		case RING_TMP_ONE_SHOT_MODE:
			moudle_tmp.sensor[dev_id].dev_mode = TMP117_DEV_ONE_SHOT_MODE;
			//NRF_LOG_INFO("SET RING_TMP_ONE_SHOT_MODE\r");
			break;

		default:
					rslt = TMP117_E_COM_FAIL;
	}

	if( TMP117_OK != rslt )
				goto tmp_set_mode_end;
	
	/* Set the sensor configuration */
	rslt = tmp117_set_sens_conf(&moudle_tmp.sensor[dev_id]);
	
tmp_set_mode_end:
	return ( TMP117_OK == rslt ) ? ( moudle_tmp.curMode[dev_id] = mode, true): false;
}


ring_tmp_mode_t tmp_get_work_mode(int8_t dev_id)
{
	return moudle_tmp.curMode[dev_id];
}

bool tmp_start(ring_tmp_mode_t mode, int8_t dev_id)
{
	if (moudle_tmp.m_pwr[dev_id]) { moudle_tmp.m_pwr[dev_id] = false; }

	if (!i2c_init()) { return moudle_tmp.m_pwr[dev_id]; }
	
	TMP117_init(dev_id);

	if (!tmp_set_mode(mode, dev_id)) { return moudle_tmp.m_pwr[dev_id]; }

	moudle_tmp.m_pwr[dev_id] = true;
	return moudle_tmp.m_pwr[dev_id];
}

bool tmp_stop(int8_t dev_id)
{
	if (moudle_tmp.m_pwr[dev_id]) {

			if (!i2c_init()) { return false; }
			
			if (!tmp_set_mode(RING_TMP_SHUTDOWN_MODE, dev_id)) { return false; }
			
			NRF_LOG_INFO("tmp close.\r\n");
			moudle_tmp.m_pwr[dev_id] = false;
	}
	return true;
}

/*******************************************************************************

*  usage: 
*
*  int main()
*  {
*     	struct tmp117_sensor_data *test_pt;
*	      read_tmp_value(&test_pt, x, RING_TMP_ONE_SHOT_MODE);
*				test_pt->temp_val;
*				test_pt->celcius;
*  }
*
*******************************************************************************/
int8_t read_tmp_value(struct tmp117_sensor_data **this_data,  int8_t dev_id, bool local_convert, ring_tmp_mode_t curMode)
{
    i2c_init();
	int8_t rslt = TMP117_OK;
	//struct tmp117_sensor_data *tmp_data;
		
	if( (NULL==this_data) || (DEVICE_NUM_MAX <= dev_id))
	{
		rslt = TMP117_E_NULL_PTR;
	}
	else
	{
		if((curMode != RING_TMP_ONE_SHOT_MODE) || (moudle_tmp.curMode[dev_id] != RING_TMP_ONE_SHOT_MODE))
		{
			rslt = TMP117_E_COM_FAIL;
			goto read_tmp_value_end;
		}
			
		rslt = tmp117_get_sensor_data( local_convert, &(moudle_tmp.sensor[dev_id]) );
		if(TMP117_OK != rslt)
			goto read_tmp_value_end;
				
		(*this_data) = moudle_tmp.sensor[dev_id].sensor_data;
				
		if( (TMP_DOWNSAMPLE_NONE != moudle_tmp.dev_info[dev_id].smp_period)
			&& (0 == (moudle_tmp.dev_info[dev_id].smp_times++) % moudle_tmp.dev_info[dev_id].smp_period) )
		{
			moudle_tmp.dev_info[dev_id].smp_data_used = TMP_SAMPLE_NEWDATA;
			moudle_tmp.dev_info[dev_id].sample_data = moudle_tmp.data[dev_id];
		}
				 				
	}

read_tmp_value_end:
		return rslt;
}

struct tmp_dev_info_s show_tmp_dev_info(int8_t dev_id)
{
#if 1
	NRF_LOG_INFO("<tmp id:0x%1X, i2c:%d,  <"RING_TMP_MARKER", %d>",
					moudle_tmp.sensor[dev_id].id,
					moudle_tmp.dev_info[dev_id].i2c_line,  
					TMP_SHOW(moudle_tmp.dev_info[dev_id].sample_data.celcius),
					moudle_tmp.dev_info[dev_id].sample_data.temp_val );
#endif	
	moudle_tmp.dev_info[dev_id].smp_data_used = TMP_SAMPLE_USEDDATA;
	return moudle_tmp.dev_info[dev_id];
}

bool tmp_is_new_data(int8_t dev_id)
{
	if( TMP_SAMPLE_NEWDATA == moudle_tmp.dev_info[dev_id].smp_data_used)
		return true;
	else
		return false;
}

void set_tmp_downsample_rate(enum tmp_downsample_rate_e downsample_rate, int8_t dev_id)
{
	if(TMP_DOWNSAMPLE_INVALID <= downsample_rate)
		moudle_tmp.dev_info[dev_id].smp_period = TMP_DOWNSAMPLE_NONE;
	else
		moudle_tmp.dev_info[dev_id].smp_period = downsample_rate;
}
