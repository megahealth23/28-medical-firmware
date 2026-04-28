
#include "afe\AFE4900\afe4900_defs.h"
#include "DataProcSPO2.h"
#include "DataProcPTT.h"
#include "../services/ble_rawdata.h"

int ptt_changecnt = 0;
int ptt_offline_ledflg = 0;
DETECT_OFFLINE ptt_ofline;
int ptt_offlineflg = 0;

void ptt_send_persecond(void)
{
    uint8_t frame_buf[16];
    frame_buf[0] = 18;
    frame_buf[1] = 18;
	
    frame_buf[2] = ptt_offlineflg; //extend write
	
    copy_to_log(frame_buf, 16);
}

int ptt_offline_fifo(void)
{
	int flag = 0;
    if(ptt_ofline.offline_num >= 4)                   // is off hand
    {
        ring_topled_off();                          // turn off top led
		flag = 1;
    }
    else                                            // is on hand
    {
        ring_topled_off();                           // turn off top led
		flag = 0;
    }
    ptt_ofline.offline_num = 0;
	ptt_offline_ledflg = OFFLINE_LED_OFF;
	
    return flag;
}

uint8_t PTT_bufProc_wavelet(void *afe_in, uint8_t frame_length)
{	
    int tmpledgreen = 0;
//    int tmp_Infra_orig = 0;
//    int tmp_Red_orig = 0;
    uint8_t Regret = 0;
    uint8_t Regret2 = 0;
    struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)afe_in;
//    dt_ofline.min_ambval = afe_bpt[0].phase4;
	for(int i = 0; i < frame_length; i++)
	{
//        tmp_Infra_orig = afe_bpt[i].phase1;
//        tmp_Red_orig = afe_bpt[i].phase2;
//		int tmpledInfra = tmp_Infra_orig;//tmp_Infra_orig - tmpledamb;
//		int tmpledRed = tmp_Red_orig;//tmp_Red_orig - tmpledamb;

        tmpledgreen = afe_bpt[i].phase1;
//        dt_ofline.min_ambval = min(dt_ofline.min_ambval, tmpledamb);
        if(ptt_offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 1)
            {
                if(tmpledgreen > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && tmpledgreen > (afe_bpt[i + 1].phase1 + OFFLINE_THRE))
                {
                    ptt_ofline.offline_num++;
                }
            }
//            dt_ofline.is_offline = Detect_ohhand(tmpledamb - dt_ofline.min_ambval);
//            if(dt_ofline.is_offline == 1)  // is off hand
//            {
//                dt_ofline.offline_num++;
//            }
        }        
	}
	ptt_changecnt++;
	
//off hand detect
    if(!(ptt_changecnt%(100/frame_length)))
    {
        if(ptt_offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
//            get_afe_reg(0x22, &reg22val);
//            int greenlowreg = reg22val >> 18;
//            int tmpredreg = (reg22val - (greenlowreg << 18)) >> 12;
            ptt_offlineflg = ptt_offline_fifo();
        }

        int change_step = 300/frame_length;
//            if(state_entry.afe_who == AFE_RTSHOW) {
//                change_step = 300/AFE4900_ARRAY_SIZE;
//            }
        if(!(ptt_changecnt%change_step))   //turn on top green led
        {
            if(ptt_offline_ledflg == OFFLINE_LED_OFF)
            {
                ring_topled_on();
                ptt_offline_ledflg = OFFLINE_LED_ON;
            }
        }
		
		ptt_send_persecond();
    }
}