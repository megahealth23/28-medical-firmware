/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_afe.c
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#include "ring_bsp.h"
#include "ring_afe.h"
#include "app_timer.h"
#include "I2C.h"
#include "mTask.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "system_clock.h"

#include "algorithm\DataProcSPO2.h"
#include "algorithm\DataProcEHR.h"

struct afe_signal_s afe_signal;

static struct ring_afe_s module_afe = 
{
	.m_pwr = false,
	.curMode = RING_AFE_NONE_MODE,
	.dev_info = 
	{
		.smp_data_used = AFE_SAMPLE_NONEDATA,
		.i2c_line     = AFE_DEV_NONE,
		.fifordy_line = AFE_DEV_NONE,
		.smp_period   = AFE_DOWNSAMPLE_NONE,  
		.smp_times    = 0,
	}
};

static void AFE4900_delay_ms(uint32_t period)
{
	// delay time
	nrf_delay_ms(period);
} // user_delay_ms()

static int8_t AFE4900I2Cwrite(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
    i2c_init();
	int8_t rslt = 0;

	uint8_t writebuf[4];
	if (len > 3)
	{
		return -1;
	}
	writebuf[0] = reg_addr;     //MUST
	memcpy(writebuf + 1, reg_data, len);

	rslt = i2c_write(dev_id, reg_addr, writebuf, (uint8_t)len + 1);
    i2c_uninit();
	return rslt;
        //return -1;
}

static int8_t AFE4900I2Cread(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
    i2c_init();
	ret_code_t rslt;
	rslt = i2c_read(dev_id, reg_addr, reg_data, len);
    i2c_uninit();
	return (int8_t)rslt;

        //return  -1;

}

void AFE4900_SW_reset(void)
{
    i2c_init();
    uint8_t tx_data[3];
    tx_data[0] = 0x00;
    tx_data[1] = 0x00;
    tx_data[2] = 0x28;  
    module_afe.sensor.write(module_afe.sensor.id, 0x00, &tx_data[0], 3);
    nrf_delay_ms(3);

    memset(tx_data, 0, sizeof(tx_data));
    module_afe.sensor.read(module_afe.sensor.id, 0x00, &tx_data[0], 3);
    NRF_LOG_INFO("tx_data 0x00: 0x%02X, 1:0x%02X, 2:0x%02X", tx_data[0], tx_data[1], tx_data[2]);
    
    i2c_uninit();
}

static uint8_t FIFO_TOGGLE = 0;
bool AFE4900_check_FIFO_TOGGLE(void)
{
    i2c_init();
    uint8_t rx_data[3] = {0}; 
    module_afe.sensor.read(module_afe.sensor.id, 0x28, &rx_data[0], 3);
    
    i2c_uninit();

    bool ret = FIFO_TOGGLE != rx_data[0]?true:false;

    FIFO_TOGGLE = rx_data[0];
    return ret;
}

uint8_t AFE4900_get_FIFO_TOGGLE(void)
{
    i2c_init();
    uint8_t rx_data[3] = {0}; 
    module_afe.sensor.read(module_afe.sensor.id, 0x28, &rx_data[0], 3);
    
    i2c_uninit();
    return rx_data[0];
}

#define 	PPG_PARM_CNT       			(80)
static AFE4900_addr_t ppg_addrs[PPG_PARM_CNT] = {  0x00/*(1)CONTROL0*/, 0x01/*(2)LED2STC*/, 0x02/*(3)LED2ENDC*/, 0x03/*(4)LED1LEDSTC*/, 0x04/*(5)LED1LEDENDC*/,
    0x05/*(6)ALED2STC*/,  0x06/*(7)ALED2ENDC*/, 0x07/*(8)LED1STC*/,  0x08/*(9)LED1ENDC*/, 0x09/*(10)LED2LEDSTC*/,
	0x0A/*(11)LED2LEDENDC*/, 0x0B/*(12)ALED1STC*/,  0x0C/*(13)ALED1ENDC*/, 0x0D/*(14)LED2CONVST*/, 0x0E/*(15)LED2CONVEND*/,
	0x0F/*(16)ALED2CONVST*/,  0x10/*(17)ALED2CONVEND*/, 0x11/*(18)LED1CONVST*/,  0x12/*(19)LED1CONVEND*/, 0x13/*(20)ALED1CONVST*/,
	0x14/*(21)ALED1CONVEND*/,   0x1D/*(22)PRPCOUNT*/, 0x1F/*(24)TIAGAIN_2_3*/, 0x20/*(25)TIAGAIN*/,
	0x21/*(26)TIA_AMB_GAIN*/, 0x22/*(27)LEDCNTRL1*/, 0x23/*(28)CONTROL2*/,  0x24/*(29)LEDCNTRL2*/, 0x28/*(30)TOGGLE*/,
	0x29/*(31)CLKDIV1*/, 0x2A/*(32)LED2VAL*/, 0x2B/*(33)ALED2VAL*/,  0x2C/*(34)LED1VAL*/, 0x2D/*(35)ALED1VAL*/,
	0x2E/*(36)LED2-ALED2VAL*/,  0x2F/*(37)LED1-ALED1VAL*/, 0x31/*(38)CONTROL3*/,  0x34/*(39)PROG_INT2_STC*/, 0x35/*(40)PROG_INT2_ENDC*/,
	0x36/*(41)LED3LEDSTC*/, 0x37/*(42)LED3LEDENDC*/, 0x39/*(43)CLKDIV2*/,   0x3A/*(44)OFFDAC*/, 0x3B/*(45)THRDETLOW*/,
	0x3C/*(46)THRDETHIGH*/,  0x3D/*(47)THRDET*/, 0x3E/*(48)I_OFFDAC*/,  0x3F/*(49)AVG_LED2_ALED2VAL*/, 0x40/*(50)AVG_LED1_ALED1VAL    */,
	0x42/*(51)FIFO*/,  0x43/*(52)LED4LEDSTC*/, 0x44/*(53)LED4LEDENDC*/, 0x45/*(54)TG_PD1STC*/, 0x46/*(55)TG_PD1ENDC*/,
	0x47/*(56)TG_PD2STC*/,  0x48/*(57)TG_PD2ENDC*/, 0x49/*(58)TG_PD3STC*/,  0x4A/*(59)TG_PD3ENDC*/, 0x4B/*(60)CONTROL4*/,
	0x4E/*(61)DUAL_PD*/,  0x50/*(62)CONTROL5*/, 0x51/*(63)FIFO_OFFSET*/, 0x52/*(64)DATA_RDY_STC*/, 0x53/*(65)DATA_RDY_ENDC*/,
	0x57/*(66)PROG_INT1_STC*/, 0x58/*(67)PROG_INT1_ENDC*/, 0x60/*(68)AMB_CANCELLATION*/,  0x64/*(69)DYN_TIA_STC*/, 0x65/*(70)DYN_TIA_ENDC*/,
	0x66/*(71)DYN_ADC_STC*/,  0x67/*(72)DYN_ADC_ENDC*/, 0x68/*(73)DYN_CLOCK_STC*/,  0x69/*(74)DYN_CLOCK_ENDC*/, 0x6A/*(75)DEEP_SLEEP_STC*/,
	0x6B/*(76)DEEP_SLEEP_ENDC*/,  0x6C/*(77)PD_SHORT*/, 0x6D/*(78)REG_POINTER*/, 0x72/*(79)LED_DRIVER_CONTROL*/, 0x73/*(80)THR_DETECT_LOGIC*/,
	0x1E/*(23)CONTROL1*/};

static AFE4900_Reg_t ppg_vals[PPG_PARM_CNT]  = {0x000020/*(1)CONTROL0: *StartSampling* */,  1/*(2)LED2STC*/, 2/*(3)LED2ENDC*/, 6/*(4)LED1LEDSTC*/, 16/*(5)LED1LEDENDC*/,
		5/*(6)ALED2STC*/, 5/*(7)ALED2ENDC*/, 10/*(8)LED1STC*/,  16/*(9)LED1ENDC*/, 0/*(10)LED2LEDSTC*/,
		2/*(11)LED2LEDENDC*/,  37/*(12)ALED1STC*/, 41/*(13)ALED1ENDC*/, 3/*(14)LED2CONVST*/, 3/*(15)LED2CONVEND*/,
		6/*(16)ALED2CONVST*/, 6/*(17)ALED2CONVEND*/, 17/*(18)LED1CONVST*/, 41/*(19)LED1CONVEND*/, 42/*(20)ALED1CONVST*/, 
		50/*(21)ALED1CONVEND*/, 1279/*(22)PRPCOUNT*/, 0/*(24)TIAGAIN_2_3*/, 0x008011/*(25)TIAGAIN*/,
		0x000011/*(26)TIA_AMB_GAIN*/, 0x000000|(2 << 0) /*(27)LEDCNTRL1*/, 0x104218/*(28)CONTROL2*/, 0x000000/*(29)LEDCNTRL2*/, 0x000000/*(30)TOGGLE*/,
		0x000000/*(31)CLKDIV1*/,  0x000000/*(32)LED2VAL*/, 0x000000/*(33)ALED2VAL*/, 0x000000/*(34)LED1VAL*/, 0x000000/*(35)ALED1VAL*/,
		0x000000/*(36)LED2-ALED2VAL*/, 0x000000/*(37)LED1-ALED1VAL*/, 0x000000/*(38)CONTROL3*/, 0x000000/*(39)PROG_INT2_STC*/, 0x000000/*(40)PROG_INT2_ENDC*/,
		3/*(41)LED3LEDSTC*/, 5/*(42)LED3LEDENDC*/, 0x000000/*(43)CLKDIV2*/, 0x000000/*(44)OFFDAC*/, 0x000000/*(45)THRDETLOW*/,
		0x000000/*(46)THRDETHIGH*/, 0x000000/*(47)THRDET*/, 0x000000/*(48)I_OFFDAC*/, 0x000000/*(49)AVG_LED2_ALED2VAL*/,0x000000/*(50)AVG_LED1_ALED1VAL*/ , 
                0x000000|(DAILY_REG_FIFO_PERIOD << 6)|(0x02 << 4)|(0 << 0) /*(51)FIFO*/, 46/*(52)LED4LEDSTC*/, 46/*(53)LED4LEDENDC*/, 0x000000/*(54)TG_PD1STC*/, 0x000014/*(55)TG_PD1ENDC*/,
		0x000016/*(56)TG_PD2STC*/, 0x00002B/*(57)TG_PD2ENDC*/, 0x000000/*(58)TG_PD3STC*/, 0x000000/*(59)TG_PD3ENDC*/, 0x00000F/*(60)CONTROL4*/,
		0x000008/*(61)DUAL_PD*/, 0x000028/*(62)CONTROL5*/, 0x000000/*(63)FIFO_OFFSET*/, 56/*(64)DATA_RDY_STC*/, 56/*(65)DATA_RDY_ENDC*/,
		0x000000/*(66)PROG_INT1_STC*/, 0x000000/*(67)PROG_INT1_ENDC*/, 0x000000 /*(68)AMB_CANCELLATION*/, 0 /*(69)DYN_TIA_STC*/, 52/*(70)DYN_TIA_ENDC*/,
		0/*(71)DYN_ADC_STC*/, 52/*(72)DYN_ADC_ENDC*/, 0/*(73)DYN_CLOCK_STC*/, 52/*(74)DYN_CLOCK_ENDC*/, 61/*(75)DEEP_SLEEP_STC*/,
		1249/*(76)DEEP_SLEEP_ENDC*/, 0x000000/*(77)PD_SHORT*/, 0x000000/*(78)REG_POINTER*/, 0x000000/*(79)LED_DRIVER_CONTROL*/, 0x000000/*(80)THR_DETECT_LOGIC*/,
		0x000102/*(23)CONTROL1*/};

void AFE4900_HWshutdown(void)
{
    AFE4900_SW_reset();

    i2c_init();
    uint8_t tx_data[3];
    tx_data[0] = 0x00;
    tx_data[1] = 0x00;
    tx_data[2] = 0x20;  
    module_afe.sensor.write(module_afe.sensor.id, 0x00, &tx_data[0], 3);

	uint8_t txdata[3] = { 0 };
    for(int ite = 0; ite < PPG_PARM_CNT; ite++)
    {
        txdata[0] = (uint8_t)( ppg_vals[ite] >> 16);
        txdata[1] = (uint8_t)( ppg_vals[ite] >> 8);
        txdata[2] = (uint8_t)( ppg_vals[ite] );
        module_afe.sensor.write(module_afe.sensor.id, ppg_addrs[ite], &txdata[0], 3);
        nrf_delay_us(10);
    }

    tx_data[0] = 0x00;
    tx_data[1] = 0x05;
    //tx_data[2] = 0x19; 
    tx_data[2] = 0x00;  
    module_afe.sensor.write(module_afe.sensor.id, 0x64, &tx_data[0], 3);
    nrf_delay_us(10);

    tx_data[0] = 0x00;
    tx_data[1] = 0x05;
    //tx_data[2] = 0x19; 
    tx_data[2] = 0x00;  
    module_afe.sensor.write(module_afe.sensor.id, 0x66, &tx_data[0], 3);
    nrf_delay_us(10);

    tx_data[0] = 0x00;
    tx_data[1] = 0x05;
    //tx_data[2] = 0x19; 
    tx_data[2] = 0x00;  
    module_afe.sensor.write(module_afe.sensor.id, 0x68, &tx_data[0], 3);
    nrf_delay_us(10);

    tx_data[0] = 0x00;
    tx_data[1] = 0x00;
    //tx_data[2] = 0x19; 
    tx_data[2] = 0x00;  
    module_afe.sensor.write(module_afe.sensor.id, 0x6A, &tx_data[0], 3);
    nrf_delay_us(10);

    tx_data[0] = 0x00;
    tx_data[1] = 0x04;
    //tx_data[2] = 0x19; 
    tx_data[2] = 0xFF;  
    module_afe.sensor.write(module_afe.sensor.id, 0x6B, &tx_data[0], 3);
    nrf_delay_us(10);

    tx_data[0] = 0x10;
    tx_data[1] = 0x42;
    //tx_data[2] = 0x19; 
    tx_data[2] = 0x1B;  
    module_afe.sensor.write(module_afe.sensor.id, 0x23, &tx_data[0], 3);
    nrf_delay_us(1000);

    memset(tx_data, 0, sizeof(tx_data));
    module_afe.sensor.read(module_afe.sensor.id, 0x23, &tx_data[0], 3);
    NRF_LOG_INFO("tx_data 0: 0x%02X, 1:0x%02X, 2:0x%02X", tx_data[0], tx_data[1], tx_data[2]);
    i2c_uninit();
    return;
}

static void afe4900_free(void)
{
	EHR_free();
	afe_spo2_free();
	afe_wave_free();
}

static void AFE4900_init(void)
{
	int8_t rslt = AFE4900_OK;
	module_afe.sensor.read = AFE4900I2Cread;
	module_afe.sensor.write = AFE4900I2Cwrite;
	module_afe.sensor.delay_ms = AFE4900_delay_ms;
	module_afe.sensor.sw_reset = AFE4900_SW_reset;
	module_afe.sensor.id = AFE4900_I2C_ADDR;
    int retry_cnt = 4;

    do{
        rslt = afe4900_init(&(module_afe.sensor));
        if(AFE4900_OK == rslt)
            break;

        module_afe.sensor.id = module_afe.sensor.id == 0x5A ? 0x5B : 0x5A;

        //mark = module_afe.sensor.id == 0x5A ? 0 : 1;
    }while(--retry_cnt);
	
	if(AFE4900_OK == rslt)
		  module_afe.dev_info.i2c_line = AFE_DEV_OK;
	NRF_LOG_INFO("afe4900_sensor = %d,  rslt=%d", module_afe.sensor.chip_id, rslt);
}

static void AFE4900_unInit(void)
{	
	module_afe.sensor.read = NULL;
	module_afe.sensor.write = NULL;
	module_afe.sensor.delay_ms = NULL;
}

static bool afe_set_mode(ring_afe_mode_t mode)
{
	int8_t rslt = AFE4900_OK;
	
	module_afe.fifo_frame.maxsize  =   AFE_FIFO_MAX_LEN_BYTES;
	module_afe.fifo_frame.data     =   module_afe.afe_fifo_buff;
	
    bool ret;
	switch(mode)
	{
		case  RING_AFE_POWEROFF:
			module_afe.sensor.dev_mode = AFE4900_DEV_POWER_DOWN_MODE;
			NRF_LOG_INFO("SET RING_AFE_POWEROFF\r");
            AFE4900_HWshutdown();
			goto afe_set_mode_end;
			break;
			
		case  RING_AFE_HRV_ON_MODE:
			module_afe.curMode = module_afe.curMode & (~(RING_AFE_HRV_ON_MODE|RING_AFE_HRV_OFF_MODE));
			mode = module_afe.curMode | mode;
                        NRF_LOG_INFO("SET RING_AFE_HRV_ON_MODE");
			goto afe_set_mode_end;
			break;
		
		case  RING_AFE_HRV_OFF_MODE:
			module_afe.curMode = module_afe.curMode & (~(RING_AFE_HRV_ON_MODE|RING_AFE_HRV_OFF_MODE));
			mode = module_afe.curMode | mode;
                        NRF_LOG_INFO("SET RING_AFE_HRV_OFF_MODE");
			goto afe_set_mode_end;
			break;
		case  RING_AFE_SPO2_MODE:
			module_afe.sensor.dev_mode = AFE4900_DEV_PPG_ONLY_MODE | AFE4900_DEV_PPG_SPO2_MODE;
			module_afe.fifo_frame.fifo_partition = AFE4900_FIFO_PARTITION_PH4_L2_A2_L1_A1;
                        afe4900_free();
                        afe_spo2_malloc();
                        ret = afe_spo2_init();
			NRF_LOG_INFO("SET RING_AFE_SPO2_MODE\r");
			break;
        case RING_AFE_BP_MODE:
            module_afe.sensor.dev_mode = AFE4900_DEV_PPG_ONLY_MODE | AFE4900_DEV_PPG_BP_MODE;
			module_afe.fifo_frame.fifo_partition = AFE4900_FIFO_PARTITION_PH4_L2_A2_L1_A1;
                        afe4900_free();
                        afe_spo2_malloc();
                        ret = afe_spo2_init();
			NRF_LOG_INFO("SET RING_AFE_SPO2_MODE\r");
			break;
		case  RING_AFE_SPORT_MODE:
			module_afe.sensor.dev_mode = AFE4900_DEV_PPG_ONLY_MODE | AFE4900_DEV_PPG_SPORT_MODE;
			module_afe.fifo_frame.fifo_partition = AFE4900_FIFO_PARTITION_PH1_DIFFL1A1;
                        afe4900_free();
                        EHR_malloc();
                        afe_ehr_init();
			NRF_LOG_INFO("SET RING_AFE_SPORT_MODE\r");
			break;	
		case  RING_AFE_DAILY_MODE:
			// module_afe.sensor.dev_mode = AFE4900_DEV_PPG_ONLY_MODE | AFE4900_DEV_PPG_SPORT_MODE;
			module_afe.sensor.dev_mode = AFE4900_DEV_PPG_ONLY_MODE | AFE4900_DEV_PPG_DAILY_MODE;
			module_afe.fifo_frame.fifo_partition = AFE4900_FIFO_PARTITION_PH1_DIFFL1A1;
                        afe_daily_free();
                        afe_daily_malloc();
                        afe_daily_init();
						EHR_free();
						DAILY_point_EHR_malloc();
                        afe_ehr_init();
			NRF_LOG_INFO("SET RING_AFE_DAILY_MODE\r");
			break;	
		//case  RING_AFE_PULSE_MODE:
		//	module_afe.sensor.dev_mode = AFE4900_DEV_PPG_ONLY_MODE | AFE4900_DEV_PPG_PULSE_MODE;
		//	module_afe.fifo_frame.fifo_partition = AFE4900_FIFO_PARTITION_PH4_L2_A2_L1_A1;
  //                      afe4900_free();
  //                      //afe_spo2_malloc();
  //                      ret = afe_pulse_init();
		//	NRF_LOG_INFO("SET RING_AFE_PULSE_MODE\r");
		//	break;
		default:
			rslt = AFE4900_E_COM_FAIL;
			break;
	}
	if(AFE4900_OK != rslt)
		goto afe_set_mode_end;
		
	module_afe.sensor.fifo = &module_afe.fifo_frame;
	rslt = afe4900_set_sens_conf(&module_afe.sensor);
	if(AFE4900_OK == rslt)
		goto afe_set_mode_end;	
	
	rslt = afe4900_set_fifo_flush(&module_afe.sensor);

    AFE4900_check_FIFO_TOGGLE();

afe_set_mode_end:
    if(AFE4900_OK == rslt){
        module_afe.curMode = mode;
    }
	return true;
}



ring_afe_mode_t afe_get_work_mode(void)
{
	return module_afe.curMode;
}

bool afe_start(ring_afe_mode_t mode)
{
	if (module_afe.m_pwr) { module_afe.m_pwr = false; }

	i2c_init();

	if((mode == RING_AFE_HRV_ON_MODE) || (mode == RING_AFE_HRV_OFF_MODE)) 
          goto set_hrv_mode_label;

	AFE4900_init();

set_hrv_mode_label:
	if (!afe_set_mode(mode))  { return module_afe.m_pwr; }

	i2c_uninit();
	module_afe.m_pwr = true;
	
	return module_afe.m_pwr;
}

bool afe_stop(void)
{
	if (module_afe.m_pwr) {
		i2c_init();
	   
        afe4900_free();
        memset((void*)&g_afe, 0, sizeof(g_afe));
		
		//if (!afe_set_mode( RING_AFE_POWEROFF ))  {   return false;  }
        //afe_set_mode(RING_AFE_POWEROFF);
        AFE4900_HWshutdown();
        //module_afe.curMode = 

        i2c_uninit();
		NRF_LOG_INFO("afe_stop.\r\n");
		module_afe.m_pwr = false;
	}
	return true;
}

bool afe_stop_without_free(void){
	if (module_afe.m_pwr) {
		i2c_init();
	   
        // afe4900_free();
        // memset((void*)&g_afe, 0, sizeof(g_afe));
		
        //afe_set_mode(RING_AFE_POWEROFF);
        AFE4900_HWshutdown();

        i2c_uninit();
		NRF_LOG_INFO("afe_stop.\r\n");
		module_afe.m_pwr = false;
	}
	return true;
}

void set_afe_fifordy_status(void)
{
	module_afe.dev_info.fifordy_line = AFE_DEV_OK;
}


/*****************************************************************************************
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
*	      read_afe_value(&test_bpt, x, &frame_length, bool, AFE4900_READ_WRITE_LENGHT_INVALID);
*       test_cpt = test_bpt + offset;
*				green_pd = test_cpt->phase1;
*  }
*
* Warn: 
* 1) start_frame(x) must be belong to [0, AFE4900_FIFO_PERIOD)
* 2) frame_length(length) must be belong to [0, (AFE4900_FIFO_PERIOD-x-1))
*
*/
int8_t read_afe_value(struct afe4900_sensor_data **this_data, int16_t start_frame, int16_t *length, bool local_convert, ring_afe_mode_t curMode)
{	
	int8_t rslt = AFE4900_OK;
	uint16_t max_frame_length;
//	uint16_t ite = 0; 
	(*length) = 0;
	if( (curMode != ((RING_AFE_HRV_ON_MODE-1) & module_afe.curMode)) || (NULL==this_data) || (NULL==length))
	{
		rslt = AFE4900_E_NULL_PTR;
	}
	else
	{
		switch(curMode)
		{
			case RING_AFE_SPO2_MODE:
				max_frame_length = AFE4900_SPO2_FIFO_PERIOD;
				module_afe.sensor.read_write_len = AFE4900_FIFO_SPO2_LEN_BYTES;
				break;
            case RING_AFE_BP_MODE:
				max_frame_length = AFE4900_SPO2_FIFO_PERIOD;
				module_afe.sensor.read_write_len = AFE4900_FIFO_SPO2_LEN_BYTES;
				break;
			case RING_AFE_DAILY_MODE:
				max_frame_length = AFE4900_DAILY_FIFO_PERIOD;
				module_afe.sensor.read_write_len = AFE4900_FIFO_DAILY_LEN_BYTES;
				break;
			case RING_AFE_SPORT_MODE:
				max_frame_length = AFE4900_EHR_FIFO_PERIOD;
                                module_afe.sensor.read_write_len = AFE4900_FIFO_EHR_LEN_BYTES;
				break;
			//case RING_AFE_PULSE_MODE:
			//	max_frame_length = AFE4900_SPO2_FIFO_PERIOD;
			//	module_afe.sensor.read_write_len = AFE4900_FIFO_SPO2_LEN_BYTES;
			//	break;
			default:
				module_afe.sensor.read_write_len = 0;
				break;
		}
		
		if((start_frame<0) || (start_frame > max_frame_length) /*|| (start_frame+(*length) > frame_length)*/)
		{
			rslt = AFE4900_READ_WRITE_LENGHT_INVALID;
			goto read_afe_value_end;
		}
		
		afe4900_set_fifo_flush(&module_afe.sensor);

		rslt = afe4900_get_fifo_data( &module_afe.sensor );
		if(AFE4900_OK != rslt)
			goto read_afe_value_end;

		rslt = afe4900_extract_sensor_data( &(module_afe.data[start_frame]), &max_frame_length, local_convert, &(module_afe.sensor));
		if(AFE4900_OK != rslt)
			goto read_afe_value_end;
				 
		(*length) =  max_frame_length;
		(*this_data) = &(module_afe.data[start_frame]);

//		NRF_LOG_INFO("AFE frame_length = %d\n",frame_length);
		if( (AFE_DOWNSAMPLE_NONE != module_afe.dev_info.smp_period)
				&&  (0 == (module_afe.dev_info.smp_times++) % module_afe.dev_info.smp_period ))
		{
			module_afe.dev_info.smp_data_used = AFE_SAMPLE_NEWDATA;
			module_afe.dev_info.sample_data = module_afe.data[start_frame];
		}
				 
#if 0
//for(ite = 0; ite < AFE4900_FIFO_PERIOD; ite++)
{
//		NRF_LOG_INFO("<green, ecg>:<%d, %d>\r\n", afeData[ite].phase1, afeData[ite].phase2);
}
#endif
	}
read_afe_value_end:
	return rslt;
}


int8_t set_afe_reg(uint8_t addr, uint32_t val)
{
	int8_t rslt = AFE4900_OK;
	uint8_t tx_data[3];
	
	if( module_afe.m_pwr 
		  && (AFE4900_DEV_PPG_ONLY_MODE == (AFE4900_DEV_PPG_ONLY_MODE & module_afe.sensor.dev_mode))  )
	{
		
		tx_data[0] = (uint8_t)(val >> 16);
		tx_data[1] = (uint8_t)(val >> 8);
		tx_data[2] = (uint8_t)(val);		
		module_afe.sensor.write(module_afe.sensor.id, addr, &tx_data[0], 3);
	}
	return rslt;
}

int8_t get_afe_reg(uint8_t addr, uint32_t *val)
{
	int8_t rslt = AFE4900_OK;
	uint8_t rx_data[3];

	module_afe.sensor.read(module_afe.sensor.id, addr, &rx_data[0], 3);
	*val = (rx_data[0]<<16)+(rx_data[1]<<8)+(rx_data[2]);	

	return rslt;
}

struct afe_dev_info_s show_afe_dev_info(void)
{
#if 1
	NRF_LOG_INFO("<afe id:0x%1X  i2c:%d,  fifordy:%d, <%d, %d>",  
	                module_afe.sensor.id,
					module_afe.dev_info.i2c_line,  
					module_afe.dev_info.fifordy_line,
					module_afe.dev_info.sample_data.phase1,
					module_afe.dev_info.sample_data.phase2);
#endif
	module_afe.dev_info.smp_data_used = AFE_SAMPLE_USEDDATA;
	return module_afe.dev_info;
}

bool afe_is_new_data(void)
{
	if( AFE_SAMPLE_NEWDATA == module_afe.dev_info.smp_data_used)
		return true;
	else
		return false;
}

void set_afe_downsample_rate(enum afe_downsample_rate_e downsample_rate)
{
	if(AFE_DOWNSAMPLE_INVALID <= downsample_rate)
		module_afe.dev_info.smp_period = AFE_DOWNSAMPLE_NONE;
	else
		module_afe.dev_info.smp_period = downsample_rate;
}

/*

*/
int8_t set_afe_iled_format(uint8_t iled1, uint8_t iled2, uint8_t iled3)
{ 
	int8_t rslt = AFE4900_OK;
	
	uint32_t  iled, iled_lsb, iled_msb;
	uint32_t  tx_iled = 0;
	
	i2c_init();
	NRF_LOG_INFO(">>>>>>>set_afe_iled_format<set_afe_iled_format<<<<<<<");
/************  ILED1 **************/	 
	iled_lsb = iled1 & 0x03;
	iled_msb = iled1 >> 2;
	tx_iled |= (iled_lsb<<18) | (iled_msb<<0);

/************  ILED2 **************/	
	iled_lsb = iled2 & 0x03;
	iled_msb = iled2 >> 2;
	tx_iled |= (iled_lsb<<20) | (iled_msb<<6);

/************  ILED3 **************/	 
	iled_lsb = iled3 & 0x03;
	iled_msb = iled3 >> 2;
	tx_iled |= (iled_lsb<<22) | (iled_msb<<12);

	set_afe_reg(0x22, tx_iled);
	i2c_uninit();
set_afe_iled_format_end:
	return rslt;
}
