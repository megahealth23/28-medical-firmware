/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief
*/
#include "ble_rawdata.h"

//bool rawdata_enable = false;

APP_TIMER_DEF(m_notify_timer);

//static void acc_copy_to_raw_data(struct afe4900_sensor_data *afe_bpt, uint8_t frame_length);
static void spo2_copy_to_raw_data(struct afe4900_sensor_data *afe_bpt, uint8_t frame_length);
static void ehr_copy_to_raw_data(struct afe4900_sensor_data *afe_bpt, uint8_t frame_length);
static void ptt_copy_to_raw_data(struct afe4900_sensor_data *afe_bpt, uint8_t frame_length);
static void pulseimp_copy_to_raw_data(struct afe4900_sensor_data *afe_bpt, uint8_t frame_length);
static void spo_1_copy_to_raw_data(struct afe4900_sensor_data *sensor_data_bpt, uint8_t frame_length);


struct ring_raw_data_s  module_rawdata =
{
	.rdata_flag =
	{
		.enable 	= 0,
		
		.data_type = NONE_WORD,

		.writting  = 0,
		.flush 	= 0,
	},
	.elemt_count = 0,
	.elemt_size = 0,
};

extern uint8_t saveGluFlag;


/*
*   a Sequence diagram of the interrupt event for the notify timer 
*    
*    [______________________________+++++++++++++++++++++++++++++++++_______________________]
*              ^                    ^               ^               ^            ^    
*             (1)            (writting:0->1)       (2)        (writting:1->0)   (3)
*
*(1) The first case< (0 == writting) and  (0 == flush) >: 
*      There are no raw data to send. 
*
*(2) The second case< (1 == writting) and  (0(or 1) == flush) >:  
*      In this case, we must ensure that a write operation have a high-priority.
*
*(2) The third case< (0 == writting) and  (1 == flush) >:  
*	   The best time to send raw data is now.
*/
static void rawdata_notify_handler(void* p_context)
{
	UNUSED_PARAMETER(p_context);
	
	//NRF_LOG_INFO("rawdata_notify_handler()");
	
	if( (module_rawdata.elemt_count > 0) 
			&& (0 == module_rawdata.rdata_flag.writting) 
				&& (1 == module_rawdata.rdata_flag.flush))
	{
		sendRawdata( module_rawdata.rdata_flag.data_type, 
						(uint8_t *)module_rawdata.raw_data,
							module_rawdata.elemt_count * module_rawdata.elemt_size + sizeof(module_rawdata.elemt_count) + RAWDATA_OFFSET );

		if( 1 == saveGluFlag ) // When saveGLU, do not clear rawdata buffer.
			return;
		
		memset((uint8_t *)module_rawdata.raw_data, 0, sizeof(module_rawdata.raw_data));
		
	//	NRF_LOG_INFO("module_rawdata.elemt_count:%d", module_rawdata.elemt_count);

		module_rawdata.elemt_count = 0;
		module_rawdata.rdata_flag.flush = 0;
	}

}

static void rawdata_notify_timer_start(bool create_timer, bool is_start)
{
	ret_code_t err_code;
	if(create_timer)
	{
		//err_code = app_timer_create(&m_notify_timer, APP_TIMER_MODE_SINGLE_SHOT, rawdata_notify_handler);
		//APP_ERROR_CHECK(err_code);
	}

	if(is_start)
	{
		//err_code = app_timer_stop(m_notify_timer);
		//err_code = app_timer_start(m_notify_timer, APP_TIMER_TICKS(100), NULL);
		//APP_ERROR_CHECK(err_code);
		module_rawdata.rdata_flag.enable = 1;
	}
}

static void rawdata_notify_timer_stop(void)
{
	ret_code_t err_code;

	//err_code = app_timer_stop(m_notify_timer);
	//APP_ERROR_CHECK(err_code);
	
	module_rawdata.rdata_flag.enable = 0;	
	memset((uint8_t *)module_rawdata.raw_data, 0, sizeof(module_rawdata.raw_data));
	module_rawdata.rdata_flag.data_type = NONE_WORD;
	module_rawdata.elemt_count = 0;
	module_rawdata.rdata_flag.flush = 0;
}

void ring_rawdata_init(void)
{
	rawdata_notify_timer_start(true, false);
}

void ring_rawdata_start(void)
{
	memset((uint8_t *)module_rawdata.raw_data, 0, sizeof(module_rawdata.raw_data));
	module_rawdata.rdata_flag.data_type = NONE_WORD;
	module_rawdata.elemt_count = 0;
	module_rawdata.rdata_flag.flush = 0;
	
	rawdata_notify_timer_start(false, true);
}

void ring_rawdata_stop(void)
{
	rawdata_notify_timer_stop();
}

bool is_enable_rawdata(void)
{
	//if( 1 == saveGluFlag ) // When saveGLU, copy rawdata.
	//	return true;

	return ( ring_conn_status()
							&& (1==module_rawdata.rdata_flag.enable))?true:false;
}

/*******************
*void main()
*{
*     uint8_t log_info[10];
*   copy_to_log(log_info, sizeof(log_info));
*}
*******************/
#define  MAX_LOG_BUFFER_SIZE		(16)
void copy_to_log(void *log_bpt, uint8_t length)
{
#define  LOG_BUFFER_OFFSET			(RAWDATA_MTU - MAX_LOG_BUFFER_SIZE)  //  Tip: matlab start index: LOG_BUFFER_OFFSET+1
	
	uint8_t log_len = (length>MAX_LOG_BUFFER_SIZE)? MAX_LOG_BUFFER_SIZE: length;
	
	 //if( 1 == saveGluFlag ) // When saveGLU, do not clear rawdata buffer.
		//	return;
		
	 memset(&module_rawdata.raw_data[LOG_BUFFER_OFFSET], 0, MAX_LOG_BUFFER_SIZE);
	 memcpy(&module_rawdata.raw_data[LOG_BUFFER_OFFSET], log_bpt, log_len);
}

void copy_to_raw_data(enum raw_data_start_word rawType, void *sensor_data_bpt, uint8_t frame_length)
{
	module_rawdata.rdata_flag.writting = 1;	
	if( rawType != module_rawdata.rdata_flag.data_type ) 
	{
		memset((uint8_t *)module_rawdata.raw_data, 0, sizeof(module_rawdata.raw_data));
		switch(rawType)
		{
			case SPO2_WORD:
				module_rawdata.elemt_size = sizeof(monRawdata_t);
				break;
			case EHR_WORD:
            case SPO2_1_WORD:
				module_rawdata.elemt_size = sizeof(monRawdata_t);
				break;
			default:
				break;
		}
		module_rawdata.elemt_count = 0;
		module_rawdata.rdata_flag.data_type = rawType;
	}
	switch(rawType)
	{
		case SPO2_WORD:
			spo2_copy_to_raw_data((struct afe4900_sensor_data *)sensor_data_bpt, frame_length);
			break;
		case EHR_WORD:
			ehr_copy_to_raw_data((struct afe4900_sensor_data *)sensor_data_bpt, frame_length);
            break;
        case SPO2_1_WORD:
            spo_1_copy_to_raw_data((struct afe4900_sensor_data *)sensor_data_bpt, frame_length);
			break;
		
		default:
			break;
	}
	module_rawdata.rdata_flag.writting = 0;

	//if((1 == module_rawdata.rdata_flag.enable) && (module_rawdata.elemt_count>=10)){
	//	ret_code_t err_code;

	//	err_code = app_timer_stop(m_notify_timer);
	//	err_code = app_timer_start(m_notify_timer, APP_TIMER_TICKS(10), NULL);
	//	APP_ERROR_CHECK(err_code);
	//}

        if( (module_rawdata.elemt_count > 0) 
			&& (0 == module_rawdata.rdata_flag.writting) 
				&& (1 == module_rawdata.rdata_flag.flush))
	{
		sendRawdata( module_rawdata.rdata_flag.data_type, 
						(uint8_t *)module_rawdata.raw_data,
							module_rawdata.elemt_count * module_rawdata.elemt_size + sizeof(module_rawdata.elemt_count) + RAWDATA_OFFSET );

		//if( 1 == saveGluFlag ) // When saveGLU, do not clear rawdata buffer.
		//	return;
		
		memset((uint8_t *)module_rawdata.raw_data, 0, sizeof(module_rawdata.raw_data));
		
	//	NRF_LOG_INFO("module_rawdata.elemt_count:%d", module_rawdata.elemt_count);

		module_rawdata.elemt_count = 0;
		module_rawdata.rdata_flag.flush = 0;
	}
}

/*
 | FIFO_PARTITION | FIFO_NPHASE | PHASE1       | PHASE2      | PHASE3      | PHASE4 | PHASE5 | PHASE6      |
 | (0000)B        | 4           | LED2         | Amb2<->LED3 | LED1        | Amb1   |    /   |   /         |  <- PPG Mode 
 | (0010)B        | 2           | LED1         | Amb1<->ECG  |   /         |   /    |    /   |   /         |  <- PTT Mode

*(1) Data marked as Ambient2 are replaced by LED3 when the third LED is enabled. Data marked as Ambient1 are replaced by LED4 when the fourth LED is enabled.
*(2) When decimation mode data are stored in the FIFO, the phases correspond to the corresponding decimation mode outputs.
*(3) When in PTT mode, the ECG signal is available in the phase marked as Ambient1.
*/

static void spo2_copy_to_raw_data(struct afe4900_sensor_data *sensor_data_bpt, uint8_t frame_length)
{
	uint8_t *frame_pt = NULL;
	uint8_t *elecnt_pt = NULL;
	int32_t  infra_val = 0, red_val = 0, acc_val = 0;
	
	struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)sensor_data_bpt;
	monRawdata_t *spo2_bpt = NULL;	
	
	frame_pt = (uint8_t *)module_rawdata.raw_data + RAWDATA_OFFSET;

	elecnt_pt = (uint8_t *)frame_pt;	
	spo2_bpt = (monRawdata_t *)(frame_pt + sizeof(module_rawdata.elemt_count));	
    extern int test_accval;
    extern uint8_t red_regval;
    extern uint8_t infra_regval;

	for(uint8_t ite = 0; ite < frame_length; ite++ )
	{
/* [Hardware R&D]-[Software R&D]-Interface */
/* | FIFO_PARTITION | FIFO_NPHASE | PHASE1       | PHASE2      | PHASE3      | PHASE4 | PHASE5 | PHASE6      |
*  | (0000)B        | 4           | LED2         | Amb2<->LED3 | LED1        | Amb1   |    /   |   /         |  <- PPG Mode */
		infra_val = afe_bpt[ite].phase1;
		red_val = afe_bpt[ite].phase2;
		acc_val = test_accval;

	#if 1
		spo2_bpt[module_rawdata.elemt_count + ite].LED2_msb = (uint8_t)(((afe_bpt[ite].phase1>>16)&0xFF));  //  infra
		spo2_bpt[module_rawdata.elemt_count + ite].LED2_midb = (uint8_t)((afe_bpt[ite].phase1>>8)&0xFF);
		spo2_bpt[module_rawdata.elemt_count + ite].LED2_lsb = (uint8_t)((afe_bpt[ite].phase1>>0)&0xFF);
			
		spo2_bpt[module_rawdata.elemt_count + ite].LED3_msb = (uint8_t)((afe_bpt[ite].phase2>>16)&0xFF);    //  red
		spo2_bpt[module_rawdata.elemt_count + ite].LED3_midb = (uint8_t)((afe_bpt[ite].phase2>>8)&0xFF);
		spo2_bpt[module_rawdata.elemt_count + ite].LED3_lsb = (uint8_t)((afe_bpt[ite].phase2>>0)&0xFF);
		
		spo2_bpt[module_rawdata.elemt_count + ite].LED1_msb = (uint8_t)((afe_bpt[ite].phase3>>16)&0xFF);	//green
		spo2_bpt[module_rawdata.elemt_count + ite].LED1_midb = (uint8_t)((afe_bpt[ite].phase3>>8)&0xFF);
		spo2_bpt[module_rawdata.elemt_count + ite].LED1_lsb =(uint8_t)((afe_bpt[ite].phase3>>0)&0xFF);
		extern uint32_t offdac_led2;
		extern uint32_t offdac_led3;
//		spo2_bpt[module_rawdata.elemt_count + ite].LED1_msb = 0;	//green
//		spo2_bpt[module_rawdata.elemt_count + ite].LED1_midb = (uint8_t)((offdac_led2)&0xFF);
//		spo2_bpt[module_rawdata.elemt_count + ite].LED1_lsb = (uint8_t)((offdac_led3)&0xFF);
	#else
		spo2_bpt[module_rawdata.elemt_count + ite].LED2_msb = 0;
		spo2_bpt[module_rawdata.elemt_count + ite].LED2_midb = 1;
		spo2_bpt[module_rawdata.elemt_count + ite].LED2_lsb = 2;
			
		spo2_bpt[module_rawdata.elemt_count + ite].LED3_msb = 3;
		spo2_bpt[module_rawdata.elemt_count + ite].LED3_midb = 4;
		spo2_bpt[module_rawdata.elemt_count + ite].LED3_lsb = 5;
		
		spo2_bpt[module_rawdata.elemt_count + ite].LED1_msb = 6;
		spo2_bpt[module_rawdata.elemt_count + ite].LED1_midb = 7;
		spo2_bpt[module_rawdata.elemt_count + ite].LED1_lsb = 8;
	#endif	
//        spo2_bpt[module_rawdata.elemt_count + ite].LED1_msb = (test_accval >> 16);
//        spo2_bpt[module_rawdata.elemt_count + ite].LED1_midb = (test_accval >> 8);
//        spo2_bpt[module_rawdata.elemt_count + ite].LED1_lsb = test_accval;
	//NRF_LOG_INFO("p1=%d,p2=%d,p3=%d", afe_bpt[ite].phase1, afe_bpt[ite].phase2, afe_bpt[ite].phase3);
		
	}
	
	module_rawdata.elemt_count += frame_length;
	*elecnt_pt = module_rawdata.elemt_count;

	if(module_rawdata.elemt_count > 190 )
		module_rawdata.elemt_count = 0;
//	NRF_LOG_INFO("elemt_count = %d", module_rawdata.elemt_count);
	
	module_rawdata.rdata_flag.flush = 1;
}
#include "../algorithm/DataProcEHR.h"

static void ehr_copy_to_raw_data(struct afe4900_sensor_data *sensor_data_bpt, uint8_t frame_length)
{
	uint8_t *frame_pt = NULL;
	uint8_t *elecnt_pt = NULL;
	
	int32_t  green_val = 0, acc_val = 0;

	struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)sensor_data_bpt;
	monRawdata_t *EHRraw = NULL;	
	
	frame_pt = (uint8_t *)module_rawdata.raw_data + RAWDATA_OFFSET;

	elecnt_pt = (uint8_t *)frame_pt;
	EHRraw = (monRawdata_t *)(frame_pt + sizeof(module_rawdata.elemt_count));
	
    extern int test_accval;
	for(uint8_t i = 0; i < frame_length; i++ )
	{
/* [Hardware R&D]-[Software R&D]-Interface */
/* | FIFO_PARTITION | FIFO_NPHASE | PHASE1       | PHASE2      | PHASE3      | PHASE4 | PHASE5 | PHASE6      |
*  | (0000)B        | 4           | LED2         | Amb2<->LED3 | LED1        | Amb1   |    /   |   /         |  <- PPG Mode */		
		green_val = afe_bpt[i].phase3;
		acc_val = test_accval;

		EHRraw[module_rawdata.elemt_count + i].LED2_msb = (uint8_t)(((afe_bpt[i].phase1>>16)&0xFF));  //  green
		EHRraw[module_rawdata.elemt_count + i].LED2_midb = (uint8_t)((afe_bpt[i].phase1>>8)&0xFF);
		EHRraw[module_rawdata.elemt_count + i].LED2_lsb = (uint8_t)((afe_bpt[i].phase1>>0)&0xFF);
			
		EHRraw[module_rawdata.elemt_count + i].LED3_msb = (uint8_t)((afe_bpt[i].phase2>>16)&0xFF);    //  embient
		EHRraw[module_rawdata.elemt_count + i].LED3_midb = (uint8_t)((afe_bpt[i].phase2>>8)&0xFF);
		EHRraw[module_rawdata.elemt_count + i].LED3_lsb = (uint8_t)((afe_bpt[i].phase2>>0)&0xFF);

        //EHRraw[module_rawdata.elemt_count + i].LED1_msb = g_afe.ehr_hr;
        //EHRraw[module_rawdata.elemt_count + i].LED1_midb = g_afe.ehr_flag;
        //EHRraw[module_rawdata.elemt_count + i].LED1_lsb = test_accval;

        EHRraw[module_rawdata.elemt_count + i].LED1_msb = (uint8_t)((afe_bpt[i].phase3>>16)&0xFF);
        EHRraw[module_rawdata.elemt_count + i].LED1_midb = (uint8_t)((afe_bpt[i].phase3>>8)&0xFF);
        EHRraw[module_rawdata.elemt_count + i].LED1_lsb = (uint8_t)((afe_bpt[i].phase3>>0)&0xFF);
	//NRF_LOG_INFO("p1=%d,p2=%d,p3=%d", afe_bpt[i].phase1, afe_bpt[i].phase2, afe_bpt[i].phase3);
		
	}
	module_rawdata.elemt_count += frame_length;
	*elecnt_pt = module_rawdata.elemt_count;	
	module_rawdata.rdata_flag.flush = 1;
}

static void spo_1_copy_to_raw_data(struct afe4900_sensor_data *sensor_data_bpt, uint8_t frame_length)
{
	uint8_t *frame_pt = NULL;
	uint8_t *elecnt_pt = NULL;
	
	int32_t  green_val = 0, acc_val = 0;

	struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)sensor_data_bpt;
	monRawdata_t *EHRraw = NULL;	
	
	frame_pt = (uint8_t *)module_rawdata.raw_data + RAWDATA_OFFSET;

	elecnt_pt = (uint8_t *)frame_pt;
	EHRraw = (monRawdata_t *)(frame_pt + sizeof(module_rawdata.elemt_count));
	
    extern int test_accval;
	for(uint8_t i = 0; i < frame_length; i++ )
	{
/* [Hardware R&D]-[Software R&D]-Interface */
/* | FIFO_PARTITION | FIFO_NPHASE | PHASE1       | PHASE2      | PHASE3      | PHASE4 | PHASE5 | PHASE6      |
*  | (0000)B        | 4           | LED2         | Amb2<->LED3 | LED1        | Amb1   |    /   |   /         |  <- PPG Mode */		
		green_val = afe_bpt[i].phase3;
		acc_val = test_accval;

		EHRraw[module_rawdata.elemt_count + i].LED2_msb = (uint8_t)(((afe_bpt[i].phase1>>16)&0xFF));  //  green
		EHRraw[module_rawdata.elemt_count + i].LED2_midb = (uint8_t)((afe_bpt[i].phase1>>8)&0xFF);
		EHRraw[module_rawdata.elemt_count + i].LED2_lsb = (uint8_t)((afe_bpt[i].phase1>>0)&0xFF);
			
		EHRraw[module_rawdata.elemt_count + i].LED3_msb = (uint8_t)((afe_bpt[i].phase2>>16)&0xFF);    //  embient
		EHRraw[module_rawdata.elemt_count + i].LED3_midb = (uint8_t)((afe_bpt[i].phase2>>8)&0xFF);
		EHRraw[module_rawdata.elemt_count + i].LED3_lsb = (uint8_t)((afe_bpt[i].phase2>>0)&0xFF);
	//NRF_LOG_INFO("p1=%d,p2=%d,p3=%d", afe_bpt[i].phase1, afe_bpt[i].phase2, afe_bpt[i].phase3);
		
	}
	module_rawdata.elemt_count += frame_length;
	*elecnt_pt = module_rawdata.elemt_count;	
	module_rawdata.rdata_flag.flush = 1;
}

static void ptt_copy_to_raw_data(struct afe4900_sensor_data *sensor_data_bpt, uint8_t frame_length)
{
	uint8_t *frame_pt = NULL;
	uint8_t *elecnt_pt = NULL;
	
	int32_t  green_val = 0, ecg_val = 0;
	
	struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)sensor_data_bpt;
	struct ring_raw_data_ptt_frame_s *ptt_bpt = NULL;	
	
	frame_pt = (uint8_t *)module_rawdata.raw_data + RAWDATA_OFFSET;

	elecnt_pt = (uint8_t *)frame_pt;	
	ptt_bpt = (struct ring_raw_data_ptt_frame_s *)(frame_pt + sizeof(module_rawdata.elemt_count));	
				
	for(uint8_t ite = 0; ite < frame_length; ite++ )
	{
/*
		This is the Coding-Type Demo.
		This [Hardware R&D]-[Software R&D]-Interface are provided by Embedded-Engineer.

		Both <PHASEx(x=1,2..)> and <LEDy(y=1,2...)> is a fixed relation for a optimal configurations of <FIFO_PARTITION> in Datasheet, and the user of register must be know.
		Thus, according to the different hardware design and the AFEXXX-register configuration,  This Coding-Type Demo are more portable . 

		| FIFO_PARTITION | FIFO_NPHASE | PHASE1       | PHASE2      | PHASE3      | PHASE4 | PHASE5 | PHASE6      |
    | (0010)B        | 2           | LED1         | Amb1<->ECG  |   /         |   /    |    /   |   /         |  <- PTT Mode
*/		
		green_val/*user level*/ = afe_bpt[ite].phase1/*register level*/;
		ecg_val/*user level*/ = afe_bpt[ite].phase2/*register level*/;
		
		ptt_bpt[module_rawdata.elemt_count + ite].green_msb = (uint8_t)(((green_val>>16)&0xFF));
		ptt_bpt[module_rawdata.elemt_count + ite].green_midb = (uint8_t)((green_val>>8)&0xFF);
		ptt_bpt[module_rawdata.elemt_count + ite].green_lsb = (uint8_t)((green_val>>0)&0xFF);
			
		ptt_bpt[module_rawdata.elemt_count + ite].ecg_msb = (uint8_t)((ecg_val>>16)&0xFF);
		ptt_bpt[module_rawdata.elemt_count + ite].ecg_midb = (uint8_t)((ecg_val>>8)&0xFF);
		ptt_bpt[module_rawdata.elemt_count + ite].ecg_lsb = (uint8_t)((ecg_val>>0)&0xFF);
		NRF_LOG_INFO("p1=%d,p2=%d", afe_bpt[ite].phase1, afe_bpt[ite].phase2);

	}
	
	module_rawdata.elemt_count += frame_length;
	*elecnt_pt = module_rawdata.elemt_count;
	
	module_rawdata.rdata_flag.flush = 1;
}


void pulseimp_copy_to_raw_data(struct afe4900_sensor_data *sensor_data_bpt, uint8_t frame_length)
{
	uint8_t *frame_pt = NULL;	
	int32_t  green_val = 0, infra_val = 0;
	
	struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)sensor_data_bpt;
	struct ring_raw_data_pulseimp_frame_s *pulseimp_bpt = NULL;	
	
	frame_pt = (uint8_t *)module_rawdata.raw_data;
	pulseimp_bpt = (struct ring_raw_data_pulseimp_frame_s *)frame_pt;	
	
	module_rawdata.rdata_flag.flush = 0;
	module_rawdata.elemt_count = 0;
	
	uint8_t tx_len = (frame_length/PULSEIMP_FRAME_SIZE)*module_rawdata.elemt_size;
        NRF_LOG_INFO("pulseimp> tx_len:%d", tx_len);

	for(uint8_t outer_ite = 0; outer_ite < frame_length; outer_ite+=PULSEIMP_FRAME_SIZE )
	{
			uint8_t frame_idx = outer_ite/PULSEIMP_FRAME_SIZE;
		
			pulseimp_bpt[frame_idx].raw_type = SPO2_WORD;
			pulseimp_bpt[frame_idx].frame_type = PULSEIMP_FRAME_TYPE;
		
			for(uint8_t inner_ite = 0; inner_ite < PULSEIMP_FRAME_SIZE; inner_ite++ )
			{
				green_val = sensor_data_bpt[outer_ite+inner_ite].phase3;
				infra_val = sensor_data_bpt[outer_ite+inner_ite].phase1;
				
				pulseimp_bpt[frame_idx].pulseimp_frame[inner_ite].green_msb = green_val>>16;
				pulseimp_bpt[frame_idx].pulseimp_frame[inner_ite].green_midb = green_val>>8;
				pulseimp_bpt[frame_idx].pulseimp_frame[inner_ite].green_lsb = green_val>>0;

				pulseimp_bpt[frame_idx].pulseimp_frame[inner_ite].infra_msb = infra_val>>16;
				pulseimp_bpt[frame_idx].pulseimp_frame[inner_ite].infra_midb = infra_val>>8;
				pulseimp_bpt[frame_idx].pulseimp_frame[inner_ite].infra_lsb = infra_val>>0;
			}
		//TODO:
			pulseimp_bpt[frame_idx].handon_flag = pulse_offline_flag();
		//  pulseimp_bpt[frame_idx].pack[0...4]
	}

	//TODO:
	// send data
	if ( aux_raw_notify(frame_pt, tx_len)== NRF_SUCCESS)
	{
		NRF_LOG_INFO("Raw sent success.")
	}
	else
	{
		NRF_LOG_ERROR("Raw sent error.")
	}
	memset((uint8_t *)module_rawdata.raw_data, 0, sizeof(module_rawdata.raw_data));

}

	
