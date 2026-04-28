#include "DataProcEHR.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "ring_afe.h"
#include "nrf_log.h"
#include "DataProcHealth.h"
#include "DataProcSPO2.h"
#include "./gesture/mring_gesture.h"
#include "filter_datapre.h"



#define get_max(input, len, max, maxpos) arm_max_f32((input), (len), (max), (maxpos));
#define get_min(input, len, min, minpos) arm_min_f32((input), (len), (min), (minpos));

#define get_mean(input, len, meanval) arm_mean_f32((input), (len), (meanval));

#define max(x1,x2) ((x1)>(x2)?(x1):(x2))
#define min(x1,x2) ((x1)<(x2)?(x1):(x2))

#define complex_pow(x) ((x)[0]*(x)[0] + (x)[1]*(x)[1])
#define complex_abs(x) sqrt(complex_pow(x))

#define EHR_OFFLINE_THRE     10000
EHR_OFFLINE ehr_off_hand;
ProcPara DataProcPara;
//Queue Red_Queue;
//Queue Green_Queue;
//Queue2 Acc_Queue;
FIFO32 EHR_green_fifo;
//FIFO_acc EHR_acc_fifo;
//EHR_STATE ehr_accstate;
MOVE_HR_HISTPara GHistPara;
int gFirstProcFlg = 1;
int frameno;
//int Ch_red[MAXFRAMENUM];
//int Ch_green[MAXFRAMENUM];
//float Ch_fft_inputBuf[Ch_FFT_SIZE];
Off_Dac_Para offdac_para;
//static float *Ch_red = NULL;
static int16_t *Ch_Acc= NULL;
static int *Ch_green = NULL;
static float *Ch_fft_inputBuf = NULL;
static float *Acc_FFT_buf = NULL;
float *FFT_In_buf = NULL;
float *FFT_Out_buf = NULL;

int datano = 0;
//uint8_t redvalflag;
//int ledambthre;

int tmpgreeninbuf[7] = {0};
double tmpgreenoutbuf[7] = { 0, 0, 0, 0, 0, 0, 0 };
//Proc_shedule

float heartratehist = 70;

#define EHR_LEDGREEN_THRES      2.1e6

#define ONHAND_Ehr_Proc  1
uint8_t offline_ehr_ledflg = OFFLINE_LED_OFF;
DETECT_OFFLINE ehr_dt_ofline;

//add poits proc
#define ONHAND_Point_Proc  1
#define Point_ONHAND_INTERVAL   6000

static void get_MoveHr(int *chg, int16_t *acc,int inlen, MOVE_HR_HISTPara *HPara, float *ShowHr);
void send_rawdata_steps();

int EHR_STEP_NUM = 0;
int get_EHR_step(void)
{
    if(EHR_STEP_NUM < 0)
    {
        EHR_STEP_NUM = 0;
    }
    return EHR_STEP_NUM;
}
int EHR_malloc(void)
{
    nrf_mem_init();
	//(2100+2100+3072)*4 = 29088, 29k
	//Ch_red = (float*)malloc(MAXFRAMENUM*sizeof(float));
    Ch_Acc = (int16_t*)nrf_malloc((Ch_ACC_SIZE)*sizeof(int16_t));
	Ch_green = (int*)nrf_malloc((MAXFRAMENUM)*sizeof(int));
	Ch_fft_inputBuf = (float*)nrf_malloc(Ch_FFT_SIZE*sizeof(float));
    Acc_FFT_buf = (float*)nrf_malloc(FFT_MAX_OUT_LEN*sizeof(float));
//    test_acc_buf = (uint16_t*)malloc(200*sizeof(uint16_t));
	return 0;
}
int DAILY_point_EHR_malloc(void)
{
    // nrf_mem_init();
	//(2100+2100+3072)*4 = 29088, 29k
	//Ch_red = (float*)malloc(MAXFRAMENUM*sizeof(float));
    Ch_Acc = (int16_t*)nrf_malloc((Ch_ACC_SIZE)*sizeof(int16_t));
	Ch_green = (int*)nrf_malloc((MAXFRAMENUM)*sizeof(int));
	Ch_fft_inputBuf = (float*)nrf_malloc(Ch_FFT_SIZE*sizeof(float));
    Acc_FFT_buf = (float*)nrf_malloc(FFT_MAX_OUT_LEN*sizeof(float));
//    test_acc_buf = (uint16_t*)malloc(200*sizeof(uint16_t));
	return 0;
}


void EHR_free(void)
{   
	NRF_LOG_INFO("EHR_free() EHR_STEP_NUM=%d,  steps_num=%d",  EHR_STEP_NUM,  steps_num() );
	
    EHR_STEP_NUM = steps_num() - EHR_STEP_NUM;
    
    if (Ch_Acc != NULL) {
		nrf_free(Ch_Acc);
		Ch_Acc = NULL;
	}
    if (Ch_green != NULL) {
		nrf_free(Ch_green);
		Ch_green = NULL;
	}
	if (Ch_fft_inputBuf != NULL) {
		nrf_free(Ch_fft_inputBuf);
		Ch_fft_inputBuf = NULL;
	}
    if (Acc_FFT_buf != NULL) {
		nrf_free(Acc_FFT_buf);
		Acc_FFT_buf = NULL;
	}
//    if (test_acc_buf != NULL) {
//		free(test_acc_buf);
//		test_acc_buf = NULL;
//	}
    ring_topled_off();                          
    offline_ehr_ledflg = OFFLINE_LED_OFF;
}

void fifo_acc_init(FIFO_acc *fifo, int size, short *buf)
{
	fifo->buf = buf;
	fifo->flags = 0;          
	fifo->size = size;
	fifo->putP = 0;                                   

	return;
}
int fifo_acc_recover(FIFO_acc *fifo,int num)
{
    if (num < 0 || num >= fifo->putP)
    {
        return -1;
    }
    int moveofflen = (fifo->putP - num);
    for(unsigned int i=0;i<num;i++)
    {
        fifo->buf[i] = fifo->buf[i+moveofflen];
    }
    fifo->putP = num;
    return 0;

}

void EHR_Init_ProcPara(ProcPara *Para, int procsamplerate)
{
	Para->samplerate = procsamplerate;
	arm_rfft_fast_init_f32(&(Para->fftPara), MAX_FFT_SIZE);
}

static void pre_Hr(void)
{
    FFT_In_buf = Ch_fft_inputBuf;
    FFT_Out_buf = Ch_fft_inputBuf;
}

void EHR_dataproc_init(void)
{
    gFirstProcFlg = 1;
    fifo32_init(&EHR_green_fifo,MAXFRAMENUM, Ch_green);
    fifo_acc_init(&EHR_acc_fifo, Ch_ACC_SIZE, Ch_Acc);

    offwrist_init();
    frameno = 0;
    datano = 0;
    //initproc = 0;
    //procbrandflg = 0;
    heartratehist = 70;
    for (int i = 0; i < 7; i++)
    {
//		tmpredinbuf[i] = 0;
//		tmpredoutbuf[i] = 0;
        tmpgreeninbuf[i] = 0;
        tmpgreenoutbuf[i] = 0;
    }

    memset((void*)&g_afe, 0, sizeof(g_afe));
    g_afe.ehr_flag = SPO2_HR_READYING;
    g_afe.ehr_hrorig = 68;
    g_afe.ehr_hr = 68;
    g_afe.point_hr = 68;
    frameno = 0;
    
    EHR_Init_ProcPara(&DataProcPara, AFE_SAMPLERATE);
    pre_Hr();
    //memset(&tmpPara, 0, sizeof(MOVE_HR_Para));
    //offline detect
    memset((uint8_t*)&ehr_off_hand, 0, sizeof(ehr_off_hand));

    GHistPara.MinHrpeace = 300;
    GHistPara.MaxHrmove = 80;
    GHistPara.SavePos = 0;
}

double EHR_get_filter(int *input, double *a, double *b, int cofflen, double *output)
{
	double tmpsum = 0;
	for (int j = 0; j < cofflen; j++)
	{
		tmpsum += b[j] * input[cofflen - 1 - j];
	}
	for (int j = 1; j < cofflen; j++)
	{
		tmpsum -= a[j] * output[cofflen - 1 - j];
	}
	return tmpsum;
}

void fft_proc(float* input, int inputlen, int outputlen, float *output)
{
	uint32_t ifftFlag = 0;

	arm_rfft_fast_f32(&DataProcPara.fftPara, input, output, ifftFlag);

	output[1] = 0;
    for (int i = 0; i < outputlen; i++)
	{
		output[i] = complex_pow(output + 2 * i);
	}
}

float get_sum(float* input, int inlen)
{
	float sum = 0;
	for (int i = 0; i < inlen; i++)
	{
		sum += input[i];
	}
	return sum;
}
//int test_acc_num = 0;
//int test_acc_buf_readno = 0;

//float Chred_fft_inputBuf[MAX_FFT_SIZE];
//float Heartratehist_buf[50] = { 0 };
//int Heartratehistindex = 0;
//int mixmove = 0;
//float ACC_pwr_buf[2] = {0};
uint8_t test_ehr_flg = 0;
int test_datano = 0;
uint8_t green_sp = 0;
uint8_t acc_sp = 0;
uint8_t clc_flg = 0;
int ch_green_ep = 0;
uint8_t green_putp = 0;
uint8_t test_rlp = 0;
uint8_t test_rhp = 0;
uint8_t log_GPar = 0;
int log_Gmp = 0;
uint8_t test_diff_green = 0;
uint8_t log_accpwr = 0;
uint8_t log_accnum = 0;
uint8_t test_moveflag = 0;
#if 1 

extern uint32_t g_ACC_num;

void hr_algorithm_time_acc_sample_num_claer(void)
{
	g_ACC_num = 0;
}

uint32_t hr_algorithm_time_acc_sample_num_get(void)
{
	return g_ACC_num;
}


void EHR_Proc_shedule(int16_t *ch_acc, int *ch_green, int datasamplerate, unsigned char *heartrate)
{
    log_accpwr = 0;
    test_rlp = 0;
    test_rhp = 0;
    log_Gmp = 0;
    test_ehr_flg = 1;
//	g_afe.time = swtimer_getSec();

	int samplerate = DataProcPara.samplerate;
	int FFT_Size = MAX_FFT_SIZE;
	float heartrateshow = 0;

    test_diff_green = 0;
    for(int i = 1; i < MAXFRAMENUM && i < datano; i++)
    {
        if(ch_green[i] - ch_green[i-1] > 1000)
        {
            test_diff_green = 1;
            break;
        }
    }
//    DEBUG_LOG("beforeclc,green putP : %d, acc putP: %d\r\n",EHR_green_fifo.putP,EHR_acc_fifo.putP);
    test_datano = EHR_green_fifo.putP;
    int green_startpt = 0;
    int ehr_proc_len = MIN(MAX_FFT_SIZE, test_datano);
    int acc_startpt = EHR_acc_fifo.putP - ehr_proc_len;

    if (acc_startpt < 0)
    {
        log_accpwr = 1;
        for (int i = EHR_acc_fifo.putP - 1; i >= 0; i--)
        {
            EHR_acc_fifo.buf[i-acc_startpt] = EHR_acc_fifo.buf[i];
        }
        for (int i = 0; i < - acc_startpt; i++)
        {
            EHR_acc_fifo.buf[i] = 980;
        }
        EHR_acc_fifo.putP = EHR_acc_fifo.putP - acc_startpt;
        acc_startpt = 0;
    }
//        DEBUG_LOG("22, acc putP: %d, acc_startpt: %d\r\n",EHR_acc_fifo.putP,acc_startpt);
    green_sp = (uint8_t)green_startpt;
    acc_sp = (uint8_t)acc_startpt;
    float Hr = 0;
	get_MoveHr_sport(ch_green, ch_acc, ehr_proc_len, &GHistPara, &Hr);
    
//    NRF_LOG_INFO("ehr_hr: %d, acc: %d, greenval: %d\r\n", Hr,ch_acc[20], ch_green[20]);
    
//memcpy(&GPara,&tmpPara, sizeof(MOVE_HR_Para));
//    EHR_send_persecond();

    *heartrate = (unsigned char)Hr;
//    if(Hr == 0)
//    {
//        g_afe.ehr_flag = SPO2_HR_READYING;
//    }
//    else
//    {
//        if(g_afe.ehr_flag != SPO2_HR_VALID)
//        {
//            g_afe.ehr_flag = SPO2_HR_VALID;
//        }
//    }

    if(EHR_green_fifo.putP >= MAX_FFT_SIZE)
    {
        fifo32_recover(&EHR_green_fifo,EHR_green_fifo.putP - 100);
		fifo_acc_recover(&EHR_acc_fifo, EHR_acc_fifo.putP - hr_algorithm_time_acc_sample_num_get());
		//NRF_LOG_INFO("===hr_algorithm_time_acc_sample_num_get() = %d\n",hr_algorithm_time_acc_sample_num_get());
		// g_ACC_num = 0;
		hr_algorithm_time_acc_sample_num_claer();
        // fifo_acc_recover(&EHR_acc_fifo, MAX_FFT_SIZE);
    }
//    DEBUG_LOG("green putP : %d, acc putP: %d\r\n",EHR_green_fifo.putP,EHR_acc_fifo.putP);
}

void daily_point_EHR_Proc_shedule(int16_t *ch_acc, int *ch_green, int datasamplerate, unsigned char *heartrate)
{
    log_accpwr = 0;
    test_rlp = 0;
    test_rhp = 0;
    log_Gmp = 0;
    test_ehr_flg = 1;
//	g_afe.time = swtimer_getSec();

	int samplerate = DataProcPara.samplerate;
	int FFT_Size = MAX_FFT_SIZE;
	float heartrateshow = 0;

    test_diff_green = 0;
    for(int i = 1; i < MAXFRAMENUM && i < datano; i++)
    {
        if(ch_green[i] - ch_green[i-1] > 1000)
        {
            test_diff_green = 1;
            break;
        }
    }
//    DEBUG_LOG("beforeclc,green putP : %d, acc putP: %d\r\n",EHR_green_fifo.putP,EHR_acc_fifo.putP);
    test_datano = EHR_green_fifo.putP;
    int green_startpt = 0;
    int ehr_proc_len = MIN(MAX_FFT_SIZE, test_datano);
    int acc_startpt = EHR_acc_fifo.putP - ehr_proc_len;

    if (acc_startpt < 0)
    {
        log_accpwr = 1;
        for (int i = EHR_acc_fifo.putP - 1; i >= 0; i--)
        {
            EHR_acc_fifo.buf[i-acc_startpt] = EHR_acc_fifo.buf[i];
        }
        for (int i = 0; i < - acc_startpt; i++)
        {
            EHR_acc_fifo.buf[i] = 980;
        }
        EHR_acc_fifo.putP = EHR_acc_fifo.putP - acc_startpt;
        acc_startpt = 0;
    }
//        DEBUG_LOG("22, acc putP: %d, acc_startpt: %d\r\n",EHR_acc_fifo.putP,acc_startpt);
    green_sp = (uint8_t)green_startpt;
    acc_sp = (uint8_t)acc_startpt;
    float Hr = 0;
	// get_MoveHr_sport_daily_point(ch_green, ch_acc, ehr_proc_len, &GHistPara, &Hr);
    get_MoveHr_sport_daily_point_new(ch_green, ch_acc, ehr_proc_len, &GHistPara, &Hr);
//    NRF_LOG_INFO("ehr_hr: %d, acc: %d, greenval: %d\r\n", Hr,ch_acc[20], ch_green[20]);
    
//memcpy(&GPara,&tmpPara, sizeof(MOVE_HR_Para));
//    EHR_send_persecond();

    *heartrate = (unsigned char)Hr;
//    if(Hr == 0)
//    {
//        g_afe.ehr_flag = SPO2_HR_READYING;
//    }
//    else
//    {
//        if(g_afe.ehr_flag != SPO2_HR_VALID)
//        {
//            g_afe.ehr_flag = SPO2_HR_VALID;
//        }
//    }

    if(EHR_green_fifo.putP >= MAX_FFT_SIZE)
    {
        fifo32_recover(&EHR_green_fifo,EHR_green_fifo.putP - 100);
		fifo_acc_recover(&EHR_acc_fifo, EHR_acc_fifo.putP - hr_algorithm_time_acc_sample_num_get());
		//NRF_LOG_INFO("===hr_algorithm_time_acc_sample_num_get() = %d\n",hr_algorithm_time_acc_sample_num_get());
		// g_ACC_num = 0;
		hr_algorithm_time_acc_sample_num_claer();
        // fifo_acc_recover(&EHR_acc_fifo, MAX_FFT_SIZE);
    }
//    DEBUG_LOG("green putP : %d, acc putP: %d\r\n",EHR_green_fifo.putP,EHR_acc_fifo.putP);
}




uint8_t g_stress_heart = 0;
extern void afe_ehr_proc(void)
{
    if(g_afe.ehr_flag != SPO2_HR_INVALID)
    {
        
		if(hrv_health_process_flag_get() == HRV_HEALTH_PROCESS_START_FLAG)
		{
			if(fifo32_status(&EHR_green_fifo) >= HRV_PROCESS_DATA_PERIOD)
			{
				stress_modules_heart_get(EHR_green_fifo.buf, HRV_PROCESS_DATA_PERIOD, &g_stress_heart);
				fifo32_recover(&EHR_green_fifo, HRV_PROCESS_DATA_PERIOD - HRV_PROCESS_DATA_HEART_PERIOD);
				g_afe.ehr_hrorig = g_stress_heart;
			}

		}
		else
		{
			EHR_Proc_shedule(EHR_acc_fifo.buf, EHR_green_fifo.buf,100,&g_afe.ehr_hrorig);
		}
        g_afe.ehr_hr = g_afe.ehr_hrorig;
    }
    else
    {
        g_afe.ehr_hr = 0;
    }

	// g_ACC_num = 0;
	hr_algorithm_time_acc_sample_num_claer();
//    send_rawdata_steps();
//    NRF_LOG_INFO("ehr_hr: %d, acc: %d, greenval: %d\r\n", g_afe.ehr_hr,EHR_acc_fifo.buf[20], EHR_green_fifo.buf[20]);
}

#endif

int test_offdac_led1 = 0;
void set_off_dac(int val)
{
    uint32_t regdata3A;
    get_afe_reg(0x3A, &regdata3A);
    uint32_t regdata3E;
    get_afe_reg(0x3E, &regdata3E);
    uint32_t offdac_led1_msb = regdata3E >> 3;
    uint32_t offdac_led1_lsb = (regdata3E & 0x04) >> 2;
    uint32_t offdac_led1_mid = (regdata3A & 0x1E0) >> 5;
    uint32_t offdac_led1 = (offdac_led1_msb << 5) + (offdac_led1_mid << 1) + offdac_led1_lsb;
    uint32_t pol = regdata3A >> 9;
    
    if(val < 0) // off_dc  -1
    {
        if(pol == 0)
        {
            pol = 1;
        }
        offdac_led1++;
    }
    else if(val == 0)// off_dc set 0
    {
        pol = 0;
        offdac_led1 = 0;
    }
    test_offdac_led1 = offdac_led1;
    offdac_led1_msb = offdac_led1 >> 5;
    offdac_led1_mid = (offdac_led1 & 0x1E) >> 1;
    offdac_led1_lsb = offdac_led1 & 0x01;
    regdata3E = (offdac_led1_msb << 3) + (offdac_led1_lsb << 2);
    regdata3A = (pol << 9) + (offdac_led1_mid << 5);
    set_afe_reg(0x3A, regdata3A);
    set_afe_reg(0x3E, regdata3E);
}

int Detect_offline_ehr(void)
{
    if(ehr_dt_ofline.offline_num >= 2)                   // is off hand
    {
        set_afe_reg(0x22, 0x000000);
        g_afe.ehr_flag = SPO2_HR_INVALID;
        g_afe.offhand_flg = 1;
		daily_sleep_offhand_flag_set(g_afe.offhand_flg);
    }
    else                                            // is on hand
    {
        if(g_afe.offhand_flg == 1)
        {
//            memset(Ch_green, 0, MAXFRAMENUM*sizeof(int));
//            memset(Ch_Acc, 0, Ch_ACC_SIZE*sizeof(int16_t));
//            EHR_dataproc_init();

            set_afe_reg(0x22, 0x000003);
            g_afe.ehr_flag = SPO2_HR_READYING;
            g_afe.offhand_flg = 0;
        }
		daily_sleep_offhand_flag_set(g_afe.offhand_flg);
    }
    ring_topled_off();                          // turn off top led
    offline_ehr_ledflg = OFFLINE_LED_OFF;
    ehr_dt_ofline.offline_num = 0;
}


//sport heart ACC record 
//extern uint16_t g_sport_acc_cnt;
//extern short g_sport_acc[AAC_RECORD_MAX_NUM];		


//int testdiff;
int testreg22val = 0;
int testreg3Aval = 0;
extern FIFO32 green_health_fifo;
uint8_t EHR_data_bufProc(void *afe_in, uint8_t frame_length)
{
    if(afe_in == NULL || frame_length <= 0)
        return 0;

    struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)afe_in;
    int tmpredval;
    int tmpledamb = afe_bpt[0].phase1;
    //int test_green = afe_bpt[0].phase3;
    int led1val = 0;
    int cofflen = 7;
	float filterPPGData = 0;
    ehr_dt_ofline.min_ambval = afe_bpt[0].phase1;
    for(int i = 0; i < frame_length; i++)
    {

		//HRV health get
		if(hrv_health_process_flag_get() == HRV_HEALTH_PROCESS_START_FLAG)
		{
			//fifo32_putPut(&green_health_fifo, afe_bpt[i].phase1 - afe_bpt[i].phase2);
			fifo32_putPut(&green_health_fifo, afe_bpt[i].phase1);
			HRV_health_data_get();
			//HRV_health_data_get_new();
		}


        led1val = afe_bpt[i].phase1;
        offdac_para.max_led1_val = max(offdac_para.max_led1_val, led1val);
        double coffa[7] = { 1.000000000000000e+000, -5.547044191350068e+000, 1.285541954468899e+001, -1.593478169199447e+001, 1.114327437040806e+001, -4.168627535934345e+000, 6.517601971221265e-001 };
        double coffb[7] = { 9.951735617670138e-004, 0, -2.985520685301042e-003, 0, 2.985520685301042e-003, 0, -9.951735617670138e-004 };
        
//            if(ehr_off_hand.error_data_num == 0)
//            {
        // if (datano < cofflen - 1)
        // {
        //     // tmpgreeninbuf[datano] = afe_bpt[i].phase1 - afe_bpt[i].phase2;
		// 	tmpgreeninbuf[datano] = afe_bpt[i].phase1;
        //     fifo32_putPut(&EHR_green_fifo, 0);
        //     ch_green_ep++;
        //     datano++;
        // }
        // else
        // {
        //     if(afe_bpt[i].phase1 > EHR_LEDGREEN_THRES)
        //     {
        //         tmpgreeninbuf[6] = tmpgreeninbuf[5];
        //     }
        //     else
        //     {
        //         // tmpgreeninbuf[6] = afe_bpt[i].phase1 - afe_bpt[i].phase2;
		// 		tmpgreeninbuf[6] = afe_bpt[i].phase1;
        //     }
        //     datano++;
        //     ch_green_ep++;
        //     tmpgreenoutbuf[6] = EHR_get_filter(tmpgreeninbuf, coffa, coffb, cofflen, tmpgreenoutbuf);

			

        //     fifo32_putPut(&EHR_green_fifo, (int)tmpgreenoutbuf[6]);
        //     memcpy(tmpgreeninbuf, tmpgreeninbuf + 1, sizeof(int)*(cofflen - 1));
        //     memcpy(tmpgreenoutbuf, tmpgreenoutbuf + 1, sizeof(double)*(cofflen - 1));
        // }
        filterPPGData = afe_bpt[i].phase1;
		filterPPGData = IIR_bandpass_chebyshev_Filter_heart_sport(filterPPGData);
		// afe_bpt[i].phase1 = (int)filterPPGData;
		fifo32_putPut(&EHR_green_fifo, (int)filterPPGData);

        tmpledamb = afe_bpt[i].phase1;
        if(offline_ehr_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 2)
            {
                if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase1 + OFFLINE_THRE))
                {
                    ehr_dt_ofline.offline_num++;
                }
                else
                {
                    if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && afe_bpt[i + 1].phase1 > (afe_bpt[i + 2].phase1 + OFFLINE_THRE))
                    {
                        ehr_dt_ofline.offline_num++;
                    }
                }
            }
        }
        ehr_dt_ofline.min_ambval = min(ehr_dt_ofline.min_ambval, tmpledamb);
        //if(offline_ehr_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        //{
        //    ehr_dt_ofline.is_offline = Detect_ohhand(tmpledamb - ehr_dt_ofline.min_ambval);
        //    if(ehr_dt_ofline.is_offline == 1)  // is off hand
        //    {
        //        ehr_dt_ofline.offline_num++;
        //    }
        //}
    }
 #if 0   
	if(g_sport_acc_cnt >= frame_length)
	{
		for(int i = 0; i < frame_length; i++)
		{
			afe_bpt[i].phase1 = afe_bpt[i].phase1 - afe_bpt[i].phase2;
			afe_bpt[i].phase2 = (int32_t)g_sport_acc[i];
			//NRF_LOG_INFO("======afe_bpt[i].phase2 = %d\n",afe_bpt[i].phase2);
			//NRF_LOG_INFO("======g_sport_acc[i] = %d\n",g_sport_acc[i]);
		}
		memcpy(g_sport_acc,&g_sport_acc[frame_length],(g_sport_acc_cnt - frame_length)*sizeof(short));
		g_sport_acc_cnt = g_sport_acc_cnt - frame_length;
	}

#endif

//off hand detect
#if 0
#if ONHAND_Ehr_Proc
    if(!(frameno%(190/frame_length)))
    {
        if(offline_ehr_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            uint32_t ehr_reg22val = 0;
            get_afe_reg(0x22, &ehr_reg22val);
//            int greenlowreg = ehr_reg22val >> 18;
            int greenreg = ehr_reg22val & 0x3f;
            Detect_offline_ehr();
        }

        // int change_step = 6000/frame_length;
		int change_step = 12000/frame_length;
//            if(state_entry.afe_who == AFE_RTSHOW) {
//                change_step = 300/AFE4900_ARRAY_SIZE;
//            }
        if(!(frameno%change_step))   //turn on top green led
        {
            if(offline_ehr_ledflg == OFFLINE_LED_OFF)
            {
                ring_topled_on();
                offline_ehr_ledflg = OFFLINE_LED_ON;
            }
        }
    }
#endif
#endif
    frameno++;
    int frameno_num = frameno * frame_length;
        
    if(!(frameno_num % AFE_SAMPLERATE)) //1s
    {
        #if 1
        if(!(frameno_num % off_dc_time))//2s
        {
            if(frameno_num > 2000)  //detect dc
            {
                if(offdac_para.max_led1_val > 1.9e6)
                {
                    set_off_dac(-1);//off_dc -1
                }
                if(offdac_para.max_led1_val < -9e5)
                {
                    set_off_dac(0);//off_dc set 0
                }
            }
            offdac_para.max_led1_val = -2e6;
        }
        #endif
        
        if(g_afe.ehr_flag == SPO2_HR_INVALID)
        {
            g_afe.ehr_hr = 0;
        }
        else
        {
            // if(frameno_num < 800) // 8s
			if(frameno_num < 2000) // 8s
            {
                g_afe.ehr_flag = SPO2_HR_READYING;
            }
            else
            {
                if(g_afe.ehr_flag != SPO2_HR_VALID)
                {
                    g_afe.ehr_flag = SPO2_HR_VALID;
                }
                return 1;
            }
        }
    }
    return 0;
}

//uint8_t rawdata_buf_ehr[244] = {0};
//void send_rawdata_steps()
//{
//    rawdata_buf_ehr[0] = 95;
//    rawdata_buf_ehr[1] = 18;
//    
////    rawdata_buf_ehr[2] = (uint8_t)(test_clcnum >> 16);
////    rawdata_buf_ehr[3] = (uint8_t)(test_clcnum >> 8);
////    rawdata_buf_ehr[4] = (uint8_t)(test_clcnum >> 0);
//    rawdata_buf_ehr[5] = (uint8_t)(steps_num() >> 16);
//    rawdata_buf_ehr[6] = (uint8_t)(steps_num() >> 8);
//    rawdata_buf_ehr[7] = (uint8_t)(steps_num() >> 0);
//    
//    aux_raw_notify(rawdata_buf_ehr, 244);
//}

#if 0
int log_RLp = 0;
void EHR_Send_HR_Rawdata(int reddata[], int greendata[], int num)
{
	static int cnt = 0;
	uint8_t frame_buf[20];
	frame_buf[0] = EHR_WORD;
	if (num == 1) {
		frame_buf[1] = 6;
	} else {
		frame_buf[1] = 12;
	}

	frame_buf[2] = (char)((greendata[0]) >> 16);
	frame_buf[3] = (char)((greendata[0]) >> 8);
	frame_buf[4] = (char)((greendata[0]));
	frame_buf[5] = (char)(reddata[0] >> 16);
	frame_buf[6] = (char)(reddata[0] >> 8);
	frame_buf[7] = (char)(reddata[0]);
//    frame_buf[5] = g_afe.ehr_hr;
//    frame_buf[6] = GPara.HrPos;
//    frame_buf[7] = GPara.MvPos;

	if (num == 2) {
		frame_buf[8] = (char)((greendata[1]) >> 16);
		frame_buf[9] = (char)((greendata[1]) >> 8);
		frame_buf[10] = (char)((greendata[1]));
		frame_buf[11] = (char)(reddata[1] >> 16);
		frame_buf[12] = (char)(reddata[1] >> 8);
		frame_buf[13] = (char)(reddata[1]);
//        frame_buf[11] = g_afe.ehr_hr;
//        frame_buf[12] = GPara.HrPos;
//        frame_buf[13] = GPara.MvPos;
	} else {
		frame_buf[8] =  test_ehr_flg;
		frame_buf[9] =  g_afe.ehr_hr;
//		frame_buf[10] =  GPara.HrPos;
//		frame_buf[11] =  GPara.Hrvld;
//		frame_buf[12] =  GPara.MvPos;
		frame_buf[13] =  test_moveflag;
	}
    
//    int greenreg = testreg22val & 0x3f;
//    int redreg = testreg22val >> 12;
//    frame_buf[14] = testreg3Aval;
//	frame_buf[15] = (char)(testreg3Aval >> 8);
//    frame_buf[16] = (char)(testreg22val);
    
    uint16_t acc_val = 0;
	acc_val = acc_3axis_log_get();
	frame_buf[14] = acc_val >> 8;
	frame_buf[15] = acc_val;
	acc_val = acc_3axis_log_get();
	frame_buf[16] = acc_val >> 8;
	frame_buf[17] = acc_val;
    acc_val = acc_3axis_log_get();
	frame_buf[18] = acc_val >> 8;
	frame_buf[19] = acc_val;
	cnt++;
	BLE_RAW_BLOCK_SEND(frame_buf, 20);
}
int log_hr = 0;
uint8_t test_flg = 0;
void EHR_send_persecond(void)
{
    uint8_t frame_buf[20];
	frame_buf[0] = EHR_WORD;
    frame_buf[1] = 18;
    
    frame_buf[2] = test_moveflag;
    frame_buf[3] = g_afe.ehr_hr;
    int tmppos = 0;
    if(GHistPara.SavePos > 0)
    {
        tmppos = (GHistPara.SavePos-1) % MAX_SAVED_HIST_LEN;
    }
    frame_buf[4] = tmppos;
    frame_buf[5] = GHistPara.HistParaVect[tmppos].MvPos;
    frame_buf[6] = GHistPara.HistParaVect[tmppos].Accpwr / 4e9;
    frame_buf[7] = GHistPara.HistParaVect[tmppos].showHr;
    frame_buf[8] = GHistPara.HistParaVect[tmppos].Hrpos[0];
    frame_buf[9] = GHistPara.HistParaVect[tmppos].Hrvld[0];
    frame_buf[10] = GHistPara.HistParaVect[tmppos].cnt;
    frame_buf[11] = green_putp;
//    frame_buf[12] = GPara.Accpwr/1e10;
    frame_buf[13] = EHR_acc_fifo.flags;
//    frame_buf[14] = testreg3Aval;
    
    int log_tespnum = steps_num();
    frame_buf[15] = testflg;
    frame_buf[16] = log_tespnum >> 8;
    frame_buf[17] = log_tespnum;
    frame_buf[18] = test_offdac_led1;
	DEBUG_LOG("log_tespnum = %d", log_tespnum);
    BLE_RAW_BLOCK_SEND(frame_buf, 20);
}

static void afe_InitReg_ehr(void)
{
	//********660nm****940nm*************两个LED ---- LED1: red; LED2: green
	afe_WriteReg(0x00, 0x40);
	
	afe_WriteReg(0x36, 1000);//afe_WriteReg(0x36, 802);		//AFE_LED3LEDSTC c 点亮
	afe_WriteReg(0x37, 1999);//afe_WriteReg(0x37, 1602);		//AFE_LED3LEDENDC c 
	afe_WriteReg(0x1E, 0x000107);//afe_WriteReg(0x1E, 0x000100);	//AFE_CONTROL1 TimerEN = 1; NUMAV = 8
	afe_WriteReg(0x20, 0x008000);//afe_WriteReg(0x20, 0x008006);	//AFE_TIA_SEP_GAIN (LED2) ENSEPGAIN = 1; LED2/LED3 gain = 500K；500k 5pF；若统一控制，写0
	afe_WriteReg(0x21, 0x000000);//afe_WriteReg(0x21, 0x000006);	//AFE_TIA_GAIN (LED1) LED1/LED1AMB gain = 500K；cf = 25pf
//	afe_WriteReg(0x3A, 0x000220);	//AFE_DAC_SETTING_REG DA极性偏转  088000  -1; 090000  -2; 098000  -3; 0A0000  -4;
//	afe_WriteReg(0x3E, 0x000004);   //  OFFDAC -0.25
    //afe_WriteReg(0x22, 0x000882);	//电流为50mA 0x03FFFF 104018；25ma--0x020820  ---12.8ma 0x010410 0x008208  =6.4ma  9.6ma-0x00C30C
									//(20 * 0.8 )ma--0000 0001 0100 0101 0001 0100    
	//afe_WriteReg(0x22, 0x022002);   //afe_WriteReg(0x22, 0x022002);
	//afe_WriteReg(0x22, 0x006008);  // LED1:red;   LED3:green;	
	afe_WriteReg(0x22, ehr_reg22); 
	afe_WriteReg(0x23, 0x104218);	//DYN1, LEDCurr, DYN2, OSC, DYN3, DYN4 //0x000200); - 0x200 Osc mode //AFE_CONTROL2；0-50mA
									//afe_WriteReg(0x31, 5);// CLKDIV_EXTMODE = 5(4MHz)
	afe_WriteReg(0x39, 0); 			//CLKDIV_PRF 4M
	afe_WriteReg(0x32, 8487);//afe_WriteReg(0x32, 6072);  		//AFE_DPD1STC	POEW 软件低功耗；
	afe_WriteReg(0x33, 39199); 		//AFE_DPD1E//软件低功耗 测量；NDC
	
	int reg42h = (1 << 20) + ((AFE4405_ARRAY_SIZE-1) << 6) + (1 << 5);  //for test
	afe_WriteReg(0x42, reg42h);
	afe_WriteReg(0x4B, 0x000100); //for test
	afe_WriteReg(0x52, 8087);//afe_WriteReg(0x52, 5575);
	afe_WriteReg(0x53, 8118);//afe_WriteReg(0x53, 5606);
}
#endif

extern bool afe_ehr_init(void)
{
	if(Ch_green == NULL || Ch_Acc == NULL ||Ch_fft_inputBuf == NULL || Acc_FFT_buf == NULL)
		return false;
	
	EHR_dataproc_init();
	EHR_STEP_NUM = steps_num();
//	DEBUG_LOG("afe_ehr_init() EHR_STEP_NUM=%d", EHR_STEP_NUM);
	return true;
}


void get_max_val(AlgData_t *input, int len, AlgData_t *max, int *maxpos)
{
	*max = input[0];
	*maxpos = 0;
	for (int i = 1; i < len; i++)
	{
		if (input[i] > *max)
		{
			*max = input[i];
			*maxpos = i;
		}
	}
}

uint8_t Is_move(float* Acc, int inlen, int MLp, int MHp, float Amax, int Amp, int HistMpos)
{
	if (Amax < 5e8)
	{
		return 0;
	}

	int moveflg = 0;
	float Amean = 0;
	int Acnt = 0;
	int tlen = Amp / 20;
	for (int i = MAX(3, Amp - 8 - tlen); i < MAX(3, Amp - 4 - tlen); i++)
	{
		Amean += Acc_FFT_buf[i];
		Acnt++;
	}

	for (int i = MIN(Amp + 4 + tlen, MHp); i < MIN(Amp + 8 + tlen, MHp); i++)
	{
		Amean += Acc_FFT_buf[i];
		Acnt++;
	}

	float Abasemean = 0;
	int tcnt = 0;
	
	for (int i = MLp; i < MHp; i++)
	{
		Abasemean += Acc_FFT_buf[i];
		tcnt++;
	}
	Abasemean /= tcnt;

	if (Acnt > 0)
	{
		float tmpr = Amax / (Amean / Acnt);
		if (tmpr > 15 || Amax > 8 * Abasemean)
		{
			moveflg = 1;
		}
		else
		{
			if (HistMpos > 0 && (tmpr > 10 || Amax > 6 * Abasemean))
			{
				moveflg = 1;
			}
		}
	}
	return moveflg;
}

int Trim_Peakaround(float *in, int inlen, int Procid, int offlen, int Proclen)
{
	float tmpsum = 0;
	int tmpcnt = 0;
	for (int i = MAX(0,Procid - Proclen); i < Procid; i++)
	{
		tmpsum += in[i];
		tmpcnt++;
	}
	for (int i = Procid + 1; i < MIN(Procid + Proclen, inlen); i++)
	{
		tmpsum += in[i];
		tmpcnt++;
	}
	if (tmpsum == 0 || tmpcnt == 0)
	{
		return 1;
	}
	float tmpmean = tmpsum / tmpcnt;
	
	int sp = Procid - Proclen;
	for (int i = Procid - offlen; i > Procid - Proclen; i--)
	{
		if (in[i - 1] > in[i] && in[i] < tmpmean)
		{
			sp = i;
			break;
		}
	}

	int ep = Procid + Proclen;
	for (int i = Procid + offlen; i < Procid + Proclen; i++)
	{
		if (in[i + 1] > in[i] && in[i] < tmpmean)
		{
			ep = i;
			break;
		}
	}

	for (int i = MAX(0,sp); i <= MIN(ep, inlen); i++)
	{
		in[i] = 0;
	}
	return 0;
}

void Trim_GFFT(float*chg, int Amp, int moveflg, int RHp, int RLp, int HistGmp, int *Gmp, int *cnt)
{
	*cnt = 0;
	if (RLp >= RHp - 10)
	{
		return;
	}
	while (*cnt < 4)
	{
		float Gtmpmax;
		get_max_val(chg + RLp, RHp - RLp, &Gtmpmax, Gmp + *cnt);
		Gmp[*cnt] += RLp;

		if (Gtmpmax == 0)
		{
			break;
		}
        
		int NZCnt = 0;
		for (int i = RLp; i < RHp; i++)
		{
			if (chg[i] > 0)
			{
				NZCnt++;
			}
		}
		if (NZCnt < 6)
		{
			return;
		}


		int procid = Gmp[*cnt];

		float tmax = chg[procid];
		int tlen = 6 + procid / 30;
		int offlen = MIN(1 + procid / 30 + 0.3, 4);

		int ret = Trim_Peakaround(chg, RHp, procid, offlen, tlen);

		if (ret == 1)
		{
			break;
		}

		//if (abs(Gmp[*cnt] - HistGmp) >= 2)
		{
			if (moveflg == 1)
			{
				float offvar = Amp * 0.05 + 1;
				if (Gmp[*cnt] >= Amp)
				{
					if (Gmp[*cnt] - Amp <= offvar || fabs(Gmp[*cnt] * 0.5 - Amp) <= offvar || fabs(Gmp[*cnt] * 0.25 - Amp) <= offvar || fabs(Gmp[*cnt] * 0.66 - Amp) <= offvar)
					{
						procid = Gmp[*cnt];
						*cnt = *cnt - 1;
					}
				}
				else
				{
					if (Amp - Gmp[*cnt] <= offvar || fabs(Amp * 0.5 - Gmp[*cnt]) <= offvar || fabs(Amp*0.25 - Gmp[*cnt]) <= offvar)
					{
						procid = Gmp[*cnt];
						*cnt = *cnt - 1;
					}
				}
			}
		}		
		*cnt = *cnt + 1;

	}
}


float get_PAR(float *in, int sp, int ep, float Gmax, int Gmp)
{
	float Gmean = 0;
	int Gcnt = 0;
	float GVld;
	int offlen = 6 + Gmp / 30;
	for (int i = MAX(sp, Gmp - offlen); i < Gmp; i++)
	{
		if (in[i] < Gmax * 0.5 || (Gmp - i > 2))
		{
			Gmean += in[i];
			Gcnt++;
		}
	}

	for (int i = Gmp + 1; i <= MIN(Gmp + offlen, ep); i++)
	{
		if (in[i] < Gmax * 0.5 || (i - Gmp > 3))
		{
			Gmean += in[i];
			Gcnt++;
		}
	}
	if (Gcnt > 0)
	{
		Gmean = Gmean / Gcnt;
		GVld = Gmax / Gmean;
	}
	else
	{
		GVld = 0;
	}
	return GVld;
}

float get_HrFreq(float *in, int inlen, int Gmp, int sp, int ep, float Gmax)
{
	float GPMval = 0;
	float GMval = 0;
	int GCnt = 0;
	for (int i = MAX(sp, Gmp - 3); i < MIN(ep, Gmp + 3); i++)
	{
		//if (in[i] > 0.6*Gmax)
		{
			GPMval += in[i] * i;
			GMval += in[i];
			GCnt++;
		}
	}
	if (GCnt > 0)
	{
		return (GPMval + 0.5) / GMval*60.0*100.0 / inlen;
	}
	else
	{
		return (Gmp + 0.5)*6000.0 / inlen;
	}
}

int16_t get_MaxMin(int16_t *acc, int inlen)
{
	int16_t tmax = -10000;
	int16_t tmin = 10000;
	for (int i = 0; i < inlen; i++)
	{
		if (acc[i] > tmax)
		{
			tmax = acc[i];
		}
		else
		{
			if (acc[i] < tmin)
			{
				tmin = acc[i];
			}
		}
	}
	return tmax - tmin;
}
int get_intMaxMin(int *in, int inlen)
{
	int tmax = -10000000;
	int tmin = 10000000;
	for (int i = 0; i < inlen; i++)
	{
		if (in[i] > tmax)
		{
			tmax = in[i];
		}
		else
		{
			if (in[i] < tmin)
			{
				tmin = in[i];
			}
		}
	}
	return tmax - tmin;
}

void get_peaceHr(int *in, int inlen, float *Hr)
{
	int tp = 0;
	int tlensum = 0;
	int tcnt = 0;
	int tlenvect[20];
	for (int i = 0; i < inlen - 1; i++)
	{
		if (in[i] <= 0 && in[i + 1] > 0)
		{
			if (tp == 0)
			{
				tp = i;
			}
			else
			{
				if (i > tp + 30)
				{
					tlenvect[tcnt] = i - tp;
					tlensum += i - tp;
					tcnt++;
					tp = i;
				}
			}
		}
	}
	tp = 0;
	for (int i = 0; i < inlen - 1; i++)
	{
		if (in[i] >= 0 && in[i + 1] < 0)
		{
			if (tp == 0)
			{
				tp = i;
			}
			else
			{
				if (i > tp + 30)
				{
					tlenvect[tcnt] = i - tp;
					tlensum += i - tp;
					tcnt++;
					tp = i;
				}
			}
		}
	}

	if (tcnt == 0)
	{
		*Hr = 0;
		return;
	}

	int tlenmean = tlensum / tcnt;
	int tcnt1 = 0;
	tlensum = 0;
	for (int i = 0; i < tcnt; i++)
	{
		if (abs(tlenmean - tlenvect[i]) < 10)
		{
			tlensum += tlenvect[i];
			tcnt1++;
		}
	}

	if (tcnt1 > 1)
	{
		*Hr = 6000.0 * tcnt1 / tlensum;
	}
	else
	{
		*Hr = 0;
	}
}

void get_HR_inpeace(int *chg, int16_t *acc, int inlen, MOVE_HR_Para *OutPara)
{
	float tHrVect[20];
	int tHrCnt = 0;
	OutPara->cnt = 0;
	OutPara->Accpwr = 0;
	OutPara->MvPos = 0;

	for (int i = 0; i < inlen - 300; i += 100)
	{
		int16_t tAccVar = get_MaxMin(acc + i, 300);
		if (tAccVar < 100)
		{
			// Segment in peace
			get_peaceHr(chg + i, 300, tHrVect + tHrCnt);
			if (tHrVect[tHrCnt] > 0)
			{
				tHrCnt++;
			}
		}
	}

	int tmpcnt = 0;
	for (int i = 0; i < tHrCnt; i++)
	{
		if (tHrVect[i] > 42 && tHrVect[i] < 150)
		{
			tHrVect[tmpcnt++] = tHrVect[i];
		}
	}

	tHrCnt = tmpcnt;

	if (tHrCnt >= 2)
	{
		float tmean = 0;
		for (int i = 0; i < tHrCnt; i++)
		{
			tmean += tHrVect[i];
		}
		tmean = tmean / tHrCnt;

		float tsum = 0;
		tmpcnt = 0;
		for (int i = 0; i < tHrCnt - 1; i++)
		{
			if (fabs(tHrVect[i] - tHrVect[i + 1]) < 15 && fabs(tmean - tHrVect[i]) < 15)
			{
				tsum += tHrVect[i];
				tHrVect[tmpcnt] = tHrVect[i];
				tmpcnt++;
			}
		}
		
		if (fabs(tHrVect[tHrCnt - 1] - tHrVect[tHrCnt - 2]) < 15 && fabs(tmean - tHrVect[tHrCnt - 1]) < 15)
		{
			tsum += tHrVect[tHrCnt - 1];
			tHrVect[tmpcnt] = tHrVect[tHrCnt - 1];
			tmpcnt++;
		}

		if (tmpcnt >= 1)
		{
			float tHr = tsum / tmpcnt;
			OutPara->Hrvld[0] = 0;
			for (int i = 0; i < tmpcnt; i++)
			{
				if (fabs(tHrVect[i] - tHr) < 10)
				{
					OutPara->Hrvld[0] = OutPara->Hrvld[0] + 1;
				}
			}
			
			//if (OutPara->Hrvld[0] > 3)
			{
				OutPara->cnt = 1;
				OutPara->showHr = tHr;
				OutPara->Hrpos[0] = (tHr * FFT_PROC_LEN / 6000 + 0.5);
			}
		}
	}
}

int Is_Gmp_NeedTrim(float *Chg, int inlen, int Gmp, int Amp)
{
	int tmplen = 3;
	if (Gmp > 40 || Amp > 40)
	{
		tmplen++;
	}

	if (abs(Gmp - Amp) <= tmplen)
	{
		return 1;
	}

	if (Gmp > Amp)
	{
		if (abs(Gmp / 2 - Amp) <= tmplen || abs(Gmp / 3 - Amp) <= tmplen - 1)
		{
			return 1;
		}
	}
	else
	{
		if (abs(Amp / 2 - Gmp) <= tmplen || abs(Amp / 3 - Gmp) <= tmplen - 1)
		{
			return 1;
		}
	}
	return 0;
}

void Trim_FFT_IN(int *chg, float *FFT_In_buf, int inlen, int moveflg)
{
	int tHrCnt = 0;
	int tHrVect[20];
	for (int i = 0; i < inlen - 300; i += 100)
	{
		int tChgVar = get_intMaxMin(chg + i, 300);
		tHrVect[tHrCnt++] = tChgVar;
	}

	int tmax1 = 0;
	int tmin1 = 100000000;
	float tmean1 = 0;
	for (int i = 0; i < tHrCnt; i++)
	{
		if (tHrVect[i] > tmax1)
		{
			tmax1 = tHrVect[i];
		}
		if (tHrVect[i] < tmin1 && tHrVect[i] > 0)
		{
			tmin1 = tHrVect[i];
		}
		tmean1 += tHrVect[i];
	}

	tmean1 = tmean1 / tHrCnt;

	int timezoneflg = 0;
	if (tmax1 > tmin1 * 5 || tmax1 > tmean1 * 2.5)
	{
		timezoneflg = 1;
		float threshold = MAX(MIN(tmin1 * 3, tmean1), tmean1 * 0.6);
		if (moveflg == 1)
		{
			threshold = MAX(MIN(tmin1 * 5, tmean1*1.5), tmean1);
		}

		int tp = 0;
		while (tp < inlen - 1)
		{
			if (chg[tp] > threshold)
			{
				int tsp = 0;
				int tep = inlen - 1;
				for (int j = tp; j > 1; j--)
				{
					if (chg[j] >= 0 && chg[j - 1] < 0)
					{
						tsp = j - 1;
						break;
					}
				}

				for (int j = tp; j < inlen - 1; j++)
				{
					if (chg[j] >= 0 && chg[j + 1] < 0)
					{
						tep = j + 1;
						break;
					}
				}

				for (int j = tsp; j < tep; j++)
				{
					FFT_In_buf[j] = 0;
				}
				tp = tep;
			}

			else
			{
				if (chg[tp] < -threshold)
				{
					int tsp = 0;
					int tep = inlen - 1;
					for (int j = tp; j > 1; j--)
					{
						if (chg[j] <= 0 && chg[j - 1] > 0)
						{
							tsp = j - 1;
							break;
						}
					}

					for (int j = tp; j < inlen - 1; j++)
					{
						if (chg[j] <= 0 && chg[j + 1] > 0)
						{
							tep = j + 1;
							break;
						}
					}

					for (int j = tsp; j < tep; j++)
					{
						FFT_In_buf[j] = 0;
					}
					tp = tep;

				}
				else
				{
					tp++;
				}
			}
		}
	}
}

void get_LowHigh_Threshold(float *Chg_FFT_buf, float * Acc_FFT_buf, float Accpwr, int Amp, 
	MOVE_HR_HISTPara *HPara, int moveflg, int preshowid, int *RLp, int *RHp)
{
	*RLp = 0.9 * FFT_PROC_LEN / 100;
	*RHp = 3.5 * FFT_PROC_LEN / 100;

	if (HPara->KeepCnt > 20)
	{
		return;
	}


	int tRLp = *RLp;
	float Gmean = 0;
	for (int i = *RLp; i < *RHp; i++)
	{
		Gmean += Chg_FFT_buf[i];
	}
	Gmean /= (*RHp - *RLp);

	while (Chg_FFT_buf[tRLp] > Gmean * 2 && tRLp <= 26)
	{
		tRLp++;
	}

	int MinHrpeacepos = 0; 
	int MAXHrpos = *RHp;
	int MAXHistHrpos = *RHp;
	if (HPara->MinHrpeace < 100)
	{
		MinHrpeacepos = HPara->MinHrpeace * FFT_PROC_LEN / 6000 - 1;
	}

	//if (HPara->MaxHrmove > 100 && HPara->MaxHrmove < 200)
	if (HPara->MaxHrmove > 0 && HPara->MaxHrmove < 200)
	{
		MAXHrpos = HPara->MaxHrmove *FFT_PROC_LEN / 6000 + MIN(Accpwr/2e9, 4);
	}

	if (HPara->HistHr > 0)
	{
		MAXHistHrpos = HPara->HistHr * FFT_PROC_LEN / 6000 + 1;
	}
	

	if (Accpwr > 1e9 && MinHrpeacepos > tRLp)
	{
		tRLp = MinHrpeacepos;
	}

	int tAmp = MAX(Amp, HPara->HistMvPos);

	if (moveflg == 1)
	{
		if (tAmp >= 2 * (*RLp) + 6)
		{
			int Amph = Amp / 2;
			float Amax = Acc_FFT_buf[Amp];
			if (Acc_FFT_buf[Amph] > Amax*0.4)
			{
				tRLp = MAX(tRLp, Amph);
			}
			*RLp = MIN(MAX(tRLp, 32), preshowid - 2);
			*RHp = MAX(MIN(*RHp, tAmp + 3), *RLp+15);
		}
		else
		{
			*RLp = MIN(MIN(tRLp + Accpwr/1e10, 25), preshowid - 2);

			*RHp = MAX(MIN(tAmp * 2 - 3, MAXHrpos), *RLp+20);
			/*
			if (Amp < *RLp)
			{
				*RHp = MAX(MIN(*RHp, Amp * 2 + 5), *RLp + 20);
			}
			else
			{
				*RHp = MIN(*RHp, Amp * 2 - 3);
			}
			*/
		}
	}
	else
	{
		if (Accpwr > 2e10)
		{
			*RLp = MAX(tRLp, 24);
			*RHp = MIN(MAXHrpos + Accpwr/1e10, *RHp);
		}
		else
		{
			*RLp = MAX(tRLp, 16);
			*RHp = MIN(MAXHrpos, *RHp);
		}
	}

	if (HPara->MvLastTime > 0)
	{
		if (HPara->HistMvPos > Amp + 6)
		{
			*RHp = MIN(*RHp, MAXHistHrpos);
			*RHp = MIN(*RHp, MAXHrpos);
		}
		
		if (HPara->HistMvPos < Amp - 6 && HPara->HistHr > 0)
		{
			*RLp = MAX(*RLp, MAXHistHrpos);
		}
	}
}

float fit_HrCurve(MOVE_HR_HISTPara *HPara, int inlen, float Hr)
{
	if (inlen < 2)
	{
		return 0;
	}

	int proclen = MIN(MAX_SAVED_HIST_LEN - 1, inlen);
	MOVE_HR_Para *HistParaVect = HPara->HistParaVect;

	float tmpmean = Hr;
	
	for (int i = 0; i < proclen; i++)
	{
		int tmpid = (inlen - proclen + i) % MAX_SAVED_HIST_LEN;
		tmpmean += HistParaVect[tmpid].showHr;
	}
	
	tmpmean = tmpmean / (proclen + 1);

	//fit_HrCurve with line
	//xielv
	float a = (Hr - tmpmean) / ((proclen + 1) / 2);
	float b = tmpmean * 2 - Hr;

	//calc var
	float varsum = 0;
	for (int i = 0; i < proclen; i++)
	{
		int tmpid = (inlen - proclen + i) % MAX_SAVED_HIST_LEN;
		varsum += fabs((a*i + b - HistParaVect[tmpid].showHr));
	}

	varsum += fabs(a*proclen + b - Hr);
	float varmean = varsum / (proclen + 1);
	return varmean;
}

void Is_HrVld(float *Chg_FFT_buf, int inlen ,MOVE_HR_HISTPara *HPara, int tpos, float *ShowHr)
{
	*ShowHr = 0;
	int id = HPara->SavePos % MAX_SAVED_HIST_LEN;
	int preid = (HPara->SavePos - 1) % MAX_SAVED_HIST_LEN;
	MOVE_HR_Para *HistParaVect = HPara->HistParaVect;

	float Hr;
	Hr = get_HrFreq(Chg_FFT_buf, inlen, tpos, tpos - 2, tpos + 2, Chg_FFT_buf[tpos]);

	if (fabs(Hr - HistParaVect[preid].showHr) < MIN(HistParaVect[preid].showHr*0.06, 8) || HPara->SavePos < 3)
	{
		*ShowHr = Hr * 0.5 + HistParaVect[preid].showHr * 0.5;
		HPara->VldScale = HPara->VldScale + 1;
	}
	else
	{
		int tmpsamecnt = 0;
		float meanshowHr = 0;
		int meancnt = 0;
		
		for (int i = MAX(0, HPara->SavePos - 9); i < HPara->SavePos; i++)
		{
			int tid = i % MAX_SAVED_HIST_LEN;
			meanshowHr += HistParaVect[tid].showHr;
			meancnt++;
						
			for (int j = 0; j < HistParaVect[tid].cnt; j++)
			{
				if (abs(HistParaVect[tid].Hrpos[j] - tpos) <= 2 && HistParaVect[tid].Hrvld[j] > 3)
				{
					tmpsamecnt++;
					break;
				}
			}
		}

		if (tmpsamecnt >= 4)
		{
			meanshowHr = meanshowHr / meancnt;

			if (fabs(meanshowHr - Hr) < 10)
			{
				*ShowHr = Hr * 0.2 + meanshowHr * 0.8;
			}
			else
			{
				if (HPara->MvLastTime > 0 || HPara->KeepCnt >= 10)
				{
					float tscale = 1 / MIN(fabs(meanshowHr - Hr), 15);
					*ShowHr = Hr * tscale + HistParaVect[preid].showHr*(1 - tscale);
				}
				else
				{
					if (fabs(meanshowHr - Hr) < 30)
					{
						float tscale = 1 / fabs(meanshowHr - Hr);
						*ShowHr = Hr * tscale + HistParaVect[preid].showHr*(1 - tscale);
					}
				}
			}

			HPara->VldScale = 0;
		}
	}

	if (*ShowHr > 0)
	{
		HistParaVect[id].showHr = *ShowHr;
		HPara->SavePos++;
		if (*ShowHr < HPara->MinHrpeace)
		{
			HPara->MinHrpeace = *ShowHr;
		}
		if (*ShowHr > HPara->MaxHrmove)
		{
			HPara->MaxHrmove = *ShowHr;
		}
	}
}

void get_HR_inmove(int *chg, int16_t *acc, int inlen, MOVE_HR_HISTPara *HPara, float *ShowHr)
{
	float samplerate = 100;
	int fftoutlen = FFT_MAX_OUT_LEN;
	int id = HPara->SavePos % MAX_SAVED_HIST_LEN;
	MOVE_HR_Para *HistParaVect = HPara->HistParaVect;
	int moveflg = 0;
	float Accpwr = 0;

	int RLp = 0.7 * inlen / samplerate;
	int RHp = 3.5 * inlen / samplerate;

	int MLp = 1.1 * inlen / samplerate;
	int MHp = 5 * inlen / samplerate;

	for (int i = 0; i < inlen; i++)
	{
		FFT_In_buf[i] = acc[i];
	}

	fft_proc(FFT_In_buf, inlen, fftoutlen, FFT_Out_buf);

	for (int i = 0; i < fftoutlen; i++)
	{
		Acc_FFT_buf[i] = FFT_Out_buf[i];
	}

	Accpwr = get_sum(Acc_FFT_buf + MLp, MHp / 2 - MLp);

	float Amax;
	int Amp;
	get_max_val(Acc_FFT_buf + MLp, MHp - MLp, &Amax, &Amp);
	Amp += MLp;

	int HistMpos = 0;
	if (HPara->SavePos > 0)
	{
		HistMpos = HistParaVect[(HPara->SavePos - 1) % MAX_SAVED_HIST_LEN].MvPos;
	}

	moveflg = Is_move(Acc_FFT_buf, fftoutlen, MLp, MHp, Amax, Amp, HistMpos);

	HistParaVect[id].Accpwr = Accpwr;
//	Amp = (int)(Amp*1.05 + 0.5);

	HistParaVect[id].MvPos = Amp;

	if (moveflg == 0)
	{
		HistParaVect[id].MvPos = 0;
	}

	if (HPara->SavePos > 0)
	{
		if (HPara->MvLastTime > 0)
		{
			HPara->MvLastTime--;
			if (HPara->MvLastTime == 0)
			{
				HPara->HistMvPos = HistParaVect[id].MvPos;
			}
			if (abs(HistParaVect[id].MvPos - HPara->HistMvPos) < 6)
			{
				HPara->HistMvPos = HistParaVect[id].MvPos;
				HPara->MvLastTime = 0;
			}
		}
		else
		{
			if (abs(HistParaVect[id].MvPos - HPara->HistMvPos) >= 8)
			{
				HPara->MvLastTime = 30;
			}
			else
			{
				HPara->HistMvPos = HistParaVect[id].MvPos;
			}
		}
	}
	else
	{
		HPara->HistMvPos = HistParaVect[id].MvPos;
		HPara->MvLastTime = 0;
	}

	for (int i = 0; i < inlen; i++)
	{
		FFT_In_buf[i] = chg[i];
	}

	if (moveflg == 0 && Accpwr < 5e4)
	{
		Trim_FFT_IN(chg, FFT_In_buf, inlen, moveflg);
	}

	fft_proc(FFT_In_buf, inlen, fftoutlen, FFT_Out_buf);

	float* Chg_FFT_buf = FFT_Out_buf;
	float *Chg_FFT_Bckbuf = FFT_Out_buf + fftoutlen;
	for (int i = 0; i < fftoutlen; i++)
	{
		Chg_FFT_Bckbuf[i] = Chg_FFT_buf[i];
	}

	//trim RLp
	int Accvar = get_MaxMin(acc, inlen);
	int Chgvar = get_intMaxMin(chg, inlen);
#if 0
	int Chgvarvect[4];
	for (int i = 0; i < 4; i++)
	{
		Chgvarvect[i] = get_intMaxMin(chg+i*inlen/4, inlen/4);
	}

	int ChgMaxvar = 0;
	int ChgMinvar = 1e10;
	for (int i = 0; i < 4; i++)
	{
		if (Chgvarvect[i] > ChgMaxvar)
		{
			ChgMaxvar = Chgvarvect[i];
		}

		if (Chgvarvect[i] < ChgMinvar)
		{
			ChgMinvar = Chgvarvect[i];
		}
	}

	if (ChgMaxvar > 1e6 && ChgMaxvar > 5*ChgMinvar)
	{
		//special proc
		if (HPara->SavePos > 0)
		{
			int preid = (HPara->SavePos - 1) % MAX_SAVED_HIST_LEN;
			*ShowHr = HistParaVect[preid].showHr;
			HistParaVect[id].cnt = 0;
			HistParaVect[id].showHr = HistParaVect[preid].showHr;
			HPara->SavePos++;
		}
		else
		{
			*ShowHr = 0;
			HistParaVect[id].cnt = 0;
			HistParaVect[id].showHr = 0;
		}
		return;
	}
#endif

	// find the 3rd max peaks between RLP & RHP
	int preid = (HPara->SavePos - 1) % MAX_SAVED_HIST_LEN;
	int preshowid = 0;
	if (HPara->SavePos > 0)
	{
		preshowid = HistParaVect[preid].showHr * 2048 / 6000 + 0.8;
	}

	int tpresid = preshowid;
	if (tpresid == 0)
	{
		tpresid = 100;
	}
	if (Accvar > 100 || Chgvar > 5000)
	{
		get_LowHigh_Threshold(Chg_FFT_buf, Acc_FFT_buf, Accpwr, Amp, HPara, moveflg, tpresid, &RLp, &RHp);
	}

	if (RLp >= RHp - 10)
	{
		RLp = 0.7 * inlen / samplerate;
		RHp = 2.5 * inlen / samplerate;
	}

	int *Gmpvect = HistParaVect[id].Hrpos;
	float *GParvect = HistParaVect[id].Hrvld;

	HistParaVect[id].cnt = 0;

	Trim_GFFT(Chg_FFT_Bckbuf, Amp, moveflg, RHp, RLp, preshowid, Gmpvect, &(HistParaVect[id].cnt));

	if (HistParaVect[id].cnt == 0)
	{
		if (HPara->SavePos > 0)
		{
			int preid = (HPara->SavePos - 1) % MAX_SAVED_HIST_LEN;
			*ShowHr = HistParaVect[preid].showHr;
			HistParaVect[id].showHr = HistParaVect[preid].showHr;
			HPara->SavePos++;
			HPara->KeepCnt++;
		}
		else
		{
			*ShowHr = 0;
			HistParaVect[id].showHr = 0;
		}
		return;
	}

	
	for (int i = 0; i < HistParaVect[id].cnt; i++)
	{
		GParvect[i] = get_PAR(Chg_FFT_buf, RLp - 6, RHp + 6, Chg_FFT_buf[Gmpvect[i]], Gmpvect[i]);
	}
	

	int minidvar = 100;
	int minidpos = 0;
	for (int i = 0; i < HistParaVect[id].cnt; i++)
	{
		if (abs(Gmpvect[i] - preshowid) < minidvar)
		{
			minidvar = abs(Gmpvect[i] - preshowid);
			minidpos = i;
		}
	}

	if (HPara->MvLastTime == 0 && ((minidvar < 2 && GParvect[minidpos] > 3) || (minidvar == 0 && HPara->VldScale > 10)))
	{
		int tpos = Gmpvect[minidpos];
		float Hr = get_HrFreq(Chg_FFT_buf, inlen, tpos, tpos - 2, tpos + 2, Chg_FFT_buf[tpos]);

		*ShowHr = Hr * 0.5 + HistParaVect[preid].showHr * 0.5;
		HistParaVect[id].showHr = *ShowHr;
		HPara->SavePos++;
		HPara->VldScale = HPara->VldScale + 1;
		if (*ShowHr < HPara->MinHrpeace)
		{
			HPara->MinHrpeace = *ShowHr;
		}
		if (*ShowHr > HPara->MaxHrmove)
		{
			HPara->MaxHrmove = *ShowHr;
		}
		HPara->KeepCnt = 0;
		if (HPara->MvLastTime == 0)
		{
			HPara->HistHr = *ShowHr;
		}
		return;
	}

	if (HPara->MvLastTime == 0 && (minidvar <= 2 && ((HPara->VldScale > 10 && GParvect[minidpos] > 3) || GParvect[minidpos] > 13)))
	{
		int tpos = Gmpvect[minidpos];
		float Hr = get_HrFreq(Chg_FFT_buf, inlen, tpos, tpos - 2, tpos + 2, Chg_FFT_buf[tpos]);

		*ShowHr = Hr * 0.1 + HistParaVect[preid].showHr * 0.9;
		HistParaVect[id].showHr = *ShowHr;
		HPara->SavePos++;
		HPara->VldScale = HPara->VldScale * 0.9;
		if (*ShowHr < HPara->MinHrpeace)
		{
			HPara->MinHrpeace = *ShowHr;
		}
		if (*ShowHr > HPara->MaxHrmove)
		{
			HPara->MaxHrmove = *ShowHr;
		}
		HPara->KeepCnt = 0;
		if (HPara->MvLastTime == 0)
		{
			HPara->HistHr = *ShowHr;
		}
		return;
	}

	float tmpmultimax = 0;
	int tmpmultimaxid = 0;
	float tmpParmax = 0;
	int tmpParmaxid = 0;
	for (int i = 0; i < HistParaVect[id].cnt; i++)
	{
		if (GParvect[i] > tmpParmax)
		{
			tmpParmax = GParvect[i];
			tmpParmaxid = i;
		}

		if (tmpParmax > 15)
		{
			break;
		}
	}

	for (int i = 0; i < HistParaVect[id].cnt; i++)
	{
		if (GParvect[i] * Chg_FFT_buf[Gmpvect[i]] > tmpmultimax)
		{
			tmpmultimax = GParvect[i] * Chg_FFT_buf[Gmpvect[i]];
			tmpmultimaxid = i;
		}
	}

	if (HPara->SavePos == 0)
	{
		HPara->VldScale = 1;
		int fpos = Gmpvect[tmpmultimaxid];
		if (tmpParmax > 10)
		{
			fpos = Gmpvect[tmpParmaxid];
		}
		if (fpos > 20 && fpos < 50)
		{
			*ShowHr = get_HrFreq(Chg_FFT_buf, inlen, fpos, fpos - 2, fpos + 2, Chg_FFT_buf[fpos]);
			HistParaVect[id].showHr = *ShowHr;
			HPara->SavePos++;
		}
		return;
	}

	int fpos = Gmpvect[tmpParmaxid];

	if (tmpParmax > 13 && (fpos > 20 && fpos < 70))
	{
		for (int i = 0; i < tmpParmaxid; i++)
		{
			if (abs(Gmpvect[i] * 2 - Gmpvect[tmpParmaxid]) <= 2 || abs(Gmpvect[i] * 3 - Gmpvect[tmpParmaxid]) <= 3)
			{
				tmpParmaxid = i;
				break;
			}
		}
		fpos = Gmpvect[tmpParmaxid];

		if (moveflg == 0)
		{
			if ((Accpwr > 2e9 && abs(fpos - Amp) > 2 && abs(fpos - Amp * 2) > 3 && abs(fpos * 2 - Amp) > 3) || Accpwr <= 2e9)
			{
				*ShowHr = get_HrFreq(Chg_FFT_buf, inlen, fpos, fpos - 2, fpos + 2, Chg_FFT_buf[fpos]);
				if (fabs(*ShowHr - HistParaVect[preid].showHr) > 10)
				{
					*ShowHr = *ShowHr * 0.1 + HistParaVect[preid].showHr * 0.9;
					HPara->VldScale = 1;
				}
				else
				{
					if (fabs(*ShowHr - HistParaVect[preid].showHr) > 6)
					{
						*ShowHr = *ShowHr * 0.2 + HistParaVect[preid].showHr * 0.8;
						HPara->VldScale = 2;
					}
					else
					{
						HPara->VldScale = HPara->VldScale + 1;
					}
				}
				HistParaVect[id].showHr = *ShowHr;
				HPara->SavePos++;

				if (*ShowHr > HPara->MaxHrmove)
				{
					HPara->MaxHrmove = *ShowHr;
				}
				HPara->KeepCnt = 0;
				if (HPara->MvLastTime == 0)
				{
					HPara->HistHr = *ShowHr;
				}
				return;
			}
		}
		else
		{
			if (abs(fpos - HistParaVect[preid].MvPos) >= 3 && abs(HistParaVect[preid].MvPos - fpos * 2) >= 2 && abs(fpos * 2 - HistParaVect[preid].MvPos) >= 2)
			{
				*ShowHr = get_HrFreq(Chg_FFT_buf, inlen, fpos, fpos - 2, fpos + 2, Chg_FFT_buf[fpos]);
				if (fabs(*ShowHr - HistParaVect[preid].showHr) > 16)
				{
					*ShowHr = *ShowHr * 0.1 + HistParaVect[preid].showHr * 0.9;
					HPara->VldScale = 1;
				}
				else
				{
					if (fabs(*ShowHr - HistParaVect[preid].showHr) > 8)
					{
						*ShowHr = *ShowHr * 0.3 + HistParaVect[preid].showHr * 0.7;
						HPara->VldScale = 2;
					}
					else
					{
						HPara->VldScale = HPara->VldScale + 1;
					}
				}
				HistParaVect[id].showHr = *ShowHr;
				HPara->SavePos++;
				
				if (*ShowHr > HPara->MaxHrmove)
				{
					HPara->MaxHrmove = *ShowHr;
				}
				HPara->KeepCnt = 0;
				if (HPara->MvLastTime == 0)
				{
					HPara->HistHr = *ShowHr;
				}
				return;
            }
		}
	}

	//if (tmpParmaxid == tmpmultimaxid && tmpParmax > 3)
	if (GParvect[tmpParmaxid] > 3)
	{
		int tpos = Gmpvect[tmpParmaxid];

		Is_HrVld(Chg_FFT_buf, inlen, HPara, tpos, ShowHr);
		if (*ShowHr > 0)
		{
			HPara->KeepCnt = 0;
			if (HPara->MvLastTime == 0)
			{
				HPara->HistHr = *ShowHr;
			}
			return;
		}
	}

	if (tmpmultimaxid != tmpParmaxid && GParvect[tmpmultimaxid] > 3)
	{
		int tpos = Gmpvect[tmpmultimaxid];

		Is_HrVld(Chg_FFT_buf, inlen, HPara, tpos, ShowHr);
		if (*ShowHr > 0)
		{
			HPara->KeepCnt = 0;
			if (HPara->MvLastTime == 0)
			{
				HPara->HistHr = *ShowHr;
			}
			return;
		}
	}

	float GVarvect[5];
	float Hrvect[5];
	float tmpminvar = 1000000;
	int tmpminid = 0;
	for (int i = 0; i < HistParaVect[id].cnt; i++)
	{
		float Hr;
		int tpos = Gmpvect[i];
		Hrvect[i] = get_HrFreq(Chg_FFT_buf, inlen, tpos, tpos - 2, tpos + 2, Chg_FFT_buf[tpos]);
		GVarvect[i] = fit_HrCurve(HPara, HPara->SavePos - 1, Hrvect[i]);
		if (GVarvect[i] < tmpminvar)
		{
			tmpminvar = GVarvect[i];
			tmpminid = i;
		}
	}

	if (tmpminvar < 2)
	{
		if (Hrvect[tmpminid] < HistParaVect[preid].showHr + 12 && Hrvect[tmpminid] > HistParaVect[preid].showHr - 4)
		{
			*ShowHr = Hrvect[tmpminid] * 0.2 + HistParaVect[preid].showHr*0.8;
			HistParaVect[id].showHr = *ShowHr;
			HPara->SavePos++;
			HPara->KeepCnt = 0;
			if (HPara->MvLastTime == 0)
			{
				HPara->HistHr = *ShowHr;
			}
			return;
		}
	}

	*ShowHr = HistParaVect[preid].showHr;
	HistParaVect[id].showHr = *ShowHr;
	HPara->VldScale = HPara->VldScale * 0.8;
	HPara->SavePos++;
	HPara->KeepCnt++;
	if (HPara->MvLastTime == 0)
	{
		HPara->HistHr = *ShowHr;
	}
	return;

}

void smooth_data_sport_int(int* frame_data, int smooth_span, int data_len, int* frame_data_smooth)
{
	int smooth_data_windows[SPORT_HR_SPAN] = { 0 };
	if (smooth_span > SPORT_HR_SPAN)
	{
		NRF_LOG_INFO("smooth_span is than 5\n");
		return;
	}

	//int sum_num = 0;
	int add_num = 0;
	int add_data_num_half = 0;
	int sum = 0;
	if (smooth_span % 2 == 1)
	{
		add_data_num_half = (smooth_span - 1) / 2;
	}
	else
	{
		add_data_num_half = (smooth_span - 2) / 2;
	}

	for (int frame_data_num = 0; frame_data_num < data_len; frame_data_num++)
	{
		if (frame_data_num < SPORT_HR_SPAN)
		{
			smooth_data_windows[frame_data_num] = frame_data[frame_data_num];
		}
		else
		{
			for (int window_num = 0; window_num < SPORT_HR_SPAN - 1; window_num++)
			{
				smooth_data_windows[window_num] = smooth_data_windows[window_num + 1];
			}
			smooth_data_windows[SPORT_HR_SPAN - 1] = frame_data[frame_data_num];
		}
		

		add_num = add_data_num_half;
		if (frame_data_num < add_data_num_half)
		{
			add_num = frame_data_num;
		}
		else if ((data_len - 1 - frame_data_num) < add_data_num_half)
		{
			add_num = data_len - frame_data_num - 1;
		}
		sum = 0;

		//for (sum_num = frame_data_num - add_num; sum_num <= frame_data_num + add_num; sum_num++)
		//{
		//	sum += frame_data[sum_num];
		//}
		for (int window_num = 0; window_num < 2 * add_num + 1; window_num++)
		{
			sum += smooth_data_windows[window_num];
		}
		frame_data_smooth[frame_data_num] = sum / (add_num * 2 + 1);

	}

}


void smooth_data_rest_hr_int(int* frame_data, int smooth_span, int data_len, int* frame_data_smooth)
{
	int smooth_data_windows[SPORT_REST_HR_SPAN] = { 0 };
	if (smooth_span > SPORT_REST_HR_SPAN)
	{
		NRF_LOG_INFO("smooth_span is than 15\n");
		return;
	}

	//int sum_num = 0;
	int add_num = 0;
	int add_data_num_half = 0;
	int sum = 0;
	if (smooth_span % 2 == 1)
	{
		add_data_num_half = (smooth_span - 1) / 2;
	}
	else
	{
		add_data_num_half = (smooth_span - 2) / 2;
	}

	for (int frame_data_num = 0; frame_data_num < data_len; frame_data_num++)
	{
		if (frame_data_num < SPORT_REST_HR_SPAN)
		{
			smooth_data_windows[frame_data_num] = frame_data[frame_data_num];
		}
		else
		{
			for (int window_num = 0; window_num < SPORT_REST_HR_SPAN - 1; window_num++)
			{
				smooth_data_windows[window_num] = smooth_data_windows[window_num + 1];
			}
			smooth_data_windows[SPORT_REST_HR_SPAN - 1] = frame_data[frame_data_num];
		}
		

		add_num = add_data_num_half;
		if (frame_data_num < add_data_num_half)
		{
			add_num = frame_data_num;
		}
		else if ((data_len - 1 - frame_data_num) < add_data_num_half)
		{
			add_num = data_len - frame_data_num - 1;
		}
		sum = 0;

		//for (sum_num = frame_data_num - add_num; sum_num <= frame_data_num + add_num; sum_num++)
		//{
		//	sum += frame_data[sum_num];
		//}
		for (int window_num = 0; window_num < 2 * add_num + 1; window_num++)
		{
			sum += smooth_data_windows[window_num];
		}
		frame_data_smooth[frame_data_num] = sum / (add_num * 2 + 1);

	}

}




void rest_flag_get(int16_t* acc,int inlen,uint8_t* rest_flag, uint8_t* rest_start_second)
{
	float ACC_var[SPORT_DATA_TIME] = {0};//20s
	int varNum = 0;
	float var1Mean = 0;
	float var1 = 0;
	// for (int ACC_num = 0;ACC_num < SPORT_HR_DATA_LEN - 300; ACC_num = ACC_num + 100)
	// {
	// 	var1Mean = 0;
	// 	var1 = 0;
	// 	for (int ACC_num_3 = ACC_num; ACC_num_3 < ACC_num + 300; ACC_num_3++)
	// 	{
	// 		var1Mean = var1Mean + acc[ACC_num_3];
	// 	}
	// 	var1Mean = var1Mean / REST_FLAG_DATA_LEN;
	// 	for (int ACC_num_3 = ACC_num; ACC_num_3 < ACC_num + 300; ACC_num_3++)
	// 	{
	// 		var1 = var1 + fabs(acc[ACC_num_3] - var1Mean);
	// 	}
	// 	if(varNum < SPORT_DATA_TIME)
	// 	{
	// 		ACC_var[varNum] = var1;
	// 		varNum++;
	// 	}

	// }

	for (int ACC_num = SPORT_HR_DATA_LEN - 1; ACC_num > REST_FLAG_DATA_LEN; ACC_num = ACC_num - SECOND_DATA_LEN)
	{
		for (int timeNum = 0; timeNum < REST_FLAG_DATA_LEN / SECOND_DATA_LEN; timeNum++)
		{
			var1Mean = 0;
			var1 = 0;
			for (int ACC_num_3 = ACC_num - timeNum * SECOND_DATA_LEN; ACC_num_3 > ACC_num - (timeNum + 1) * SECOND_DATA_LEN; ACC_num_3--)
			{
				var1Mean = var1Mean + acc[ACC_num_3];
			}
			var1Mean = var1Mean / SECOND_DATA_LEN;
			for (int ACC_num_3 = ACC_num - timeNum * SECOND_DATA_LEN; ACC_num_3 > ACC_num - (timeNum + 1) * SECOND_DATA_LEN; ACC_num_3--)
			{
				var1 = var1 + fabs(acc[ACC_num_3] - var1Mean);
			}
			if (var1 > ACC_MOVE_THRESHOLD)
			{
				break;
			}
		}
		if (varNum < SPORT_DATA_TIME)
		{
			ACC_var[varNum] = var1;
			varNum++;
		}


	}

	*rest_flag = 0;
	*rest_start_second = 0;


	for (int num = 0;num < varNum;num++)
	{
		if (ACC_var[num] < ACC_MOVE_THRESHOLD)
		{
			*rest_flag = REST_FLAG;
			*rest_start_second = num;
			break;
		}
	}

}


uint16_t PPG_mean_pos_rest[REST_FLAG_DATA_LEN / 2] = { 0 };
void PPG_data_RR_peaks_get_rest_heart(int* PPGData, int DataLen, uint16_t* RRData, int* RRLen, int* upPeakValueMean)
{

	uint16_t mean_pos_cnt = 0;
	int meanValue = 0;
	int meanValue_two = 0;
	uint16_t meanValue_two_cnt = 0;
	int all_peaks_mean = 0;
	int upPeakMaxValueMean = 0;
	uint16_t upPeakCnt = 0;
	int downPeakMaxValueMean = 0;
	uint16_t downPeakCnt = 0;
	int maxValue = 0;
	int minValue = 0;
	uint16_t peaceDataCnt = 0;
	memset(PPG_mean_pos_rest, 0, (REST_FLAG_DATA_LEN / 2) * sizeof(uint16_t));
	// vaild ppt data
	for (int num = 0; num < DataLen - 1; num++)
	{
		if(mean_pos_cnt < REST_FLAG_DATA_LEN / 2)
		{
			if (PPGData[num] < 0 && PPGData[num + 1] >= 0)
			{
				PPG_mean_pos_rest[mean_pos_cnt] = num;//zeros point
				mean_pos_cnt++;
			}
			else if (PPGData[num] >= 0 && PPGData[num + 1] < 0)
			{
				PPG_mean_pos_rest[mean_pos_cnt] = num + 1;//zeros point
				mean_pos_cnt++;
			}

		}

	}
	// NRF_LOG_INFO("==mean_pos_cnt= %d\n", mean_pos_cnt);

	for (int num = 1; num < mean_pos_cnt; num++)
	{
		maxValue = PPG_data_max_value_int(&PPGData[PPG_mean_pos_rest[num]], PPG_mean_pos_rest[num] - PPG_mean_pos_rest[num - 1] - 1);
		minValue = PPG_data_min_value_int(&PPGData[PPG_mean_pos_rest[num]], PPG_mean_pos_rest[num] - PPG_mean_pos_rest[num - 1] - 1);
		if (maxValue < 5000 && maxValue > 0)//finger move data delete
		{
			upPeakMaxValueMean = upPeakMaxValueMean + maxValue;
			upPeakCnt = upPeakCnt + 1;
		}

		if (minValue < -200 && minValue > -4000)//finger move data delete
		{
			downPeakMaxValueMean = downPeakMaxValueMean + minValue;
			downPeakCnt = downPeakCnt + 1;
		}

	}
	// NRF_LOG_INFO("==upPeakCnt= %d\n", upPeakCnt);
	// NRF_LOG_INFO("==downPeakCnt= %d\n", downPeakCnt);
	if (upPeakCnt != 0)
	{
		upPeakMaxValueMean = upPeakMaxValueMean / upPeakCnt;
	}
	if (downPeakCnt != 0)
	{
		downPeakMaxValueMean = downPeakMaxValueMean / downPeakCnt;
	}

	// NRF_LOG_INFO("==upPeakMaxValueMean= %d\n", (int)upPeakMaxValueMean);
	// NRF_LOG_INFO("==downPeakMaxValueMean= %d\n", (int)downPeakMaxValueMean);

	*upPeakValueMean = upPeakMaxValueMean;

	peaceDataCnt = 0;
	for (int num = 0; num < DataLen; num++)
	{
		if (PPGData[num] < upPeakMaxValueMean && PPGData[num] > downPeakMaxValueMean)
		{
			meanValue = meanValue + PPGData[num];
			peaceDataCnt = peaceDataCnt + 1;
		}

	}
	if (peaceDataCnt != 0)
	{
		meanValue = meanValue / peaceDataCnt;
	}

	for (int num = 0; num < DataLen; num++)
	{

		if (PPGData[num] < meanValue && PPGData[num] < upPeakMaxValueMean && PPGData[num] > downPeakMaxValueMean)
		{
			meanValue_two = meanValue_two + PPGData[num];
			meanValue_two_cnt = meanValue_two_cnt + 1;
		}

	}

	if (meanValue_two_cnt != 0)
	{
		meanValue_two = meanValue_two / meanValue_two_cnt;
	}
	// NRF_LOG_INFO("==meanValue_two= %d\n", (int)meanValue_two);
	memset(PPG_mean_pos_rest, 0, (REST_FLAG_DATA_LEN / 2) * sizeof(uint16_t));
	mean_pos_cnt = 0;
	for (int num = 0; num < DataLen - 1; num++)
	{
		if(mean_pos_cnt < REST_FLAG_DATA_LEN / 2)
		{
			if (PPGData[num] >= meanValue_two && PPGData[num + 1] < meanValue_two)
			{
				PPG_mean_pos_rest[mean_pos_cnt] = num;
				mean_pos_cnt++;
			}
			else if (PPGData[num] < meanValue_two && PPGData[num + 1] >= meanValue_two)
			{
				PPG_mean_pos_rest[mean_pos_cnt] = num + 1;
				mean_pos_cnt++;
			}
		}

	}

	int min_value = 10000;
	uint16_t min_value_pos = 0;
	for (int num = 0; num < mean_pos_cnt; num++)
	{
		min_value = 10000;
		min_value_pos = 0;
		for (int pos_num = PPG_mean_pos_rest[num]; pos_num < PPG_mean_pos_rest[num + 1]; pos_num++)
		{
			if (min_value > PPGData[pos_num])
			{
				min_value = PPGData[pos_num];
				min_value_pos = pos_num;
			}
		}
		//printf("===min_value = %f,min_value_pos = %d\n", min_value, min_value_pos);
		// if (min_value < meanValue_two && min_value > -5000 && min_value < -100)
		if (min_value < meanValue_two && min_value > -5000)
		{
			RRData[*RRLen] = min_value_pos;
			*RRLen = *RRLen + 1;
		}

	}

}

void PPG_data_RR_get_rest_heart(uint16_t* RRData, int RRLen, RR_DATA_RECORD* RR_data_out, int* PPGData, int dataLen, int upPeakValueMean)
{
	int pre_flag = 0;
	int pre_rr_end_pos = 0;
	pre_rr_end_pos = RR_data_out->RR_num;
	for (int num = 0; num < RRLen - 1; num++)
	{
		if (pre_flag == 1)
		{
			pre_flag = 0;
			continue;
		}
		//if (RRData[num + 1] - RRData[num] < 30)
		if(RR_data_out->RR_num < REST_MAX_RR_NUM)
		{
			if (RRData[num + 1] - RRData[num] < 27)
			{
				RR_data_out->green_RR[RR_data_out->RR_num] = RRData[num + 1];
				RR_data_out->RR_num++;
				pre_flag = 1;
			}
			else
			{
				RR_data_out->green_RR[RR_data_out->RR_num] = RRData[num];
				RR_data_out->RR_num++;
			}
		}

	}
	// NRF_LOG_INFO("111=== RR_data_out->RR_num = %d\n",RR_data_out->RR_num);

	if (RRLen > 2)
	{
		//if(RRData[RRLen - 1] - RRData[RRLen - 2] > 30)
		if(RR_data_out->RR_num < REST_MAX_RR_NUM)
		{
			if (RRData[RRLen - 1] - RRData[RRLen - 2] >= 27) //27 : max hr = (1000/270)*60 = 222
			{
				RR_data_out->green_RR[RR_data_out->RR_num] = RRData[RRLen - 1];
				RR_data_out->RR_num++;
			}
		}

	}
	int maxValue = 0;
	uint16_t RRNum = 0;

	// NRF_LOG_INFO("pre_rr_end_pos = %d\n",pre_rr_end_pos);
	// NRF_LOG_INFO("2222=== RR_data_out->RR_num = %d\n",RR_data_out->RR_num);
	for (int num = pre_rr_end_pos; num < RR_data_out->RR_num - 1;)
	{
		// NRF_LOG_INFO("== RR_data_out->green_RR[num] =  %d\n",RR_data_out->green_RR[num]);
		// NRF_LOG_INFO("== RR_data_out->green_RR[num + 1] - RR_data_out->green_RR[num] =  %d\n",RR_data_out->green_RR[num + 1] - RR_data_out->green_RR[num]);
		maxValue = PPG_data_max_value_int(&PPGData[RR_data_out->green_RR[num]], RR_data_out->green_RR[num + 1] - RR_data_out->green_RR[num]);
		
		// NRF_LOG_INFO("== maxValue = %d\n",maxValue);
		// NRF_LOG_INFO("== upPeakValueMean = %d\n",(int)upPeakValueMean);
		if (maxValue > 0 && maxValue - upPeakValueMean < 5000)
		{
			if(RRNum < REST_MAX_RR_NUM)
			{
				RR_data_out->green_RR[RRNum] = (RR_data_out->green_RR[num + 1] - RR_data_out->green_RR[num]);
				RRNum++;
			}

			num++;

		}
		else
		{
			num = num + 2;
		}

	}
	RR_data_out->RR_num = RRNum;
}





uint16_t green_RR_peaks_rest[REST_FLAG_DATA_MAX_PEAKS] = {0};
int greenRRLen_peace = 0;
RR_DATA_RECORD g_rr_data_record = {0};
// void get_resting_hr(float* chg, int inlen, uint8_t rest_start_second,MOVE_HR_Para* OutPara)
void get_resting_hr(int* chg, int inlen, uint8_t rest_start_second,MOVE_HR_Para* OutPara)
{
	int ppg_data_start_pos = 0;
	int ppg_data_end_pos = 0;
	// float upPeakValueMean = 0;
	int upPeakValueMean = 0;
	//ppg process
	// ppg_data_start_pos = rest_start_second * REST_FLAG_DATA_MOVE_LEN;
	// ppg_data_end_pos = (rest_start_second + 3) * REST_FLAG_DATA_MOVE_LEN;


	ppg_data_start_pos = SPORT_HR_DATA_LEN - (rest_start_second + REST_FLAG_DATA_LEN/ SECOND_DATA_LEN) * REST_FLAG_DATA_MOVE_LEN;
	ppg_data_end_pos = SPORT_HR_DATA_LEN - rest_start_second * REST_FLAG_DATA_MOVE_LEN;


	greenRRLen_peace = 0;
	memset(green_RR_peaks_rest,0, REST_FLAG_DATA_MAX_PEAKS*sizeof(uint16_t));
	// NRF_LOG_INFO("==chg[ppg_data_start_pos]= %d\n", (int)chg[ppg_data_start_pos]);
	PPG_data_RR_peaks_get_rest_heart(&chg[ppg_data_start_pos], ppg_data_end_pos - ppg_data_start_pos, green_RR_peaks_rest, &greenRRLen_peace, &upPeakValueMean);
	// NRF_LOG_INFO("==greenRRLen_peace= %d\n", greenRRLen_peace);
	memset(&g_rr_data_record, 0, sizeof(RR_DATA_RECORD));

	PPG_data_RR_get_rest_heart(green_RR_peaks_rest, greenRRLen_peace, &g_rr_data_record, &chg[ppg_data_start_pos], ppg_data_end_pos - ppg_data_start_pos, upPeakValueMean);

	// NRF_LOG_INFO("==g_rr_data_record.RR_num = %d\n", g_rr_data_record.RR_num);
	uint16_t rr_mean = 0;
	uint16_t rr_mean_cnt = 0;
	uint16_t rr_min = 500;
	uint16_t rr_max = 0;
	for (int num = 0;num < g_rr_data_record.RR_num; num++)
	{
		if (g_rr_data_record.green_RR[num] > SPORT_MIN_RR && g_rr_data_record.green_RR[num] < SPORT_MAX_RR)
		{
			rr_mean = rr_mean + g_rr_data_record.green_RR[num];
			rr_mean_cnt = rr_mean_cnt + 1;
			if (rr_min > g_rr_data_record.green_RR[num])
			{
				rr_min = g_rr_data_record.green_RR[num];
			}
			if (rr_max < g_rr_data_record.green_RR[num])
			{
				rr_max = g_rr_data_record.green_RR[num];
			}
		}

	}

	if (rr_mean_cnt > 3)
	{
		// rr_mean = (rr_mean - rr_min)*1.0 / (rr_mean_cnt - 1);
		rr_mean = (rr_mean - rr_min - rr_max) * 1.0 / (rr_mean_cnt - 2);
	}
	else if (rr_mean_cnt != 0)
	{
		rr_mean = rr_mean*1.0 / rr_mean_cnt;
	}
	// NRF_LOG_INFO("==rr_mean = %d\n", rr_mean);
	if (rr_mean != 0)
	{
		OutPara->showHr = 100.0 / rr_mean * 60;
	}
	else
	{
		OutPara->showHr = OutPara->Hrvld[3];
	}
	
	//printf("==OutPara->showHr = %f\n", OutPara->showHr);


}





MOVE_HR_Para g_OutPara = { 0 };
uint8_t g_pre_peak_pos = 0;
uint8_t g_point_pre_peak_pos = 24;// 24*100/2048*60 = 70.3
void sport_heart_algorithm_init(void)
{
	g_pre_peak_pos = 0;
	memset(&g_OutPara,0,sizeof(MOVE_HR_Para));
}

float fft_amplitude_search_raw[FFT_SEARCH_LEN] = { 0 };
float fft_amplitude_search_new[FFT_SEARCH_LEN] = { 0 };
float fft_amplitude_acc[FFT_SEARCH_LEN] = {0};
uint8_t ACC_pos[FFT_SEARCH_LEN] = {0};
// uint8_t g_sport_alg_cnt = 0;
int ppg_diff[MAX_FFT_SIZE] = {0};
void get_MoveHr_sport(int* chg, int16_t* acc, int inlen, MOVE_HR_HISTPara* HPara, float* ShowHr)
{
	uint8_t rest_flag = 0;
	uint8_t rest_start_second = 0;
	// NRF_LOG_INFO("111111\n\r");
	memset(ppg_diff,0,MAX_FFT_SIZE*sizeof(int));
	smooth_data_sport_int(chg,SPORT_HR_SPAN,inlen,ppg_diff);
	// NRF_LOG_INFO("222222\n\r");
	// NRF_LOG_INFO("chg[0] = %d\n\r",(int)chg[0]);
	// NRF_LOG_INFO("inlen = %d\n\r",inlen);

	for (int num = 0;num < inlen - 1;num++)
	{
		ppg_diff[num] = ppg_diff[num + 1] - ppg_diff[num];
	}
	ppg_diff[inlen - 1] = ppg_diff[inlen - 2];


	// // fft process hr data get
	// for (int num = 0;num < inlen;num++)
	// {
	// 	FFT_In_buf[num] = ppg_diff[num];
	// }






	//g_sport_alg_cnt = 1;
	
	// else if(g_sport_alg_cnt == 1 && inlen < MAX_FFT_SIZE)
	// else if(g_sport_alg_cnt == 1)
	// {

	// 	for (int num = inlen - SPORT_ALG_DATA_PERIOD;num < inlen - 1;num++)
	// 	{
	// 		chg[num] = chg[num + 1] - chg[num];
	// 	}
	// 	chg[inlen - 1] = chg[inlen - 2];
	// }
	// else if(g_sport_alg_cnt == 1 && inlen >= MAX_FFT_SIZE)
	// {
	// 	for (int num = inlen - SPORT_ALG_DATA_PERIOD;num < inlen - 1;num++)
	// 	{
	// 		chg[num] = chg[num + 1] - chg[num];
	// 	}
	// 	chg[inlen - 1] = chg[inlen - 2];
	// }

	rest_flag_get(acc, inlen, &rest_flag, &rest_start_second);
	// NRF_LOG_INFO("rest_flag = %d\n\r",rest_flag);
	// NRF_LOG_INFO("rest_start_second = %d\n\r",rest_start_second);
	if (rest_flag == REST_FLAG)
	{
		//rest hr process data get

		memset(ppg_diff,0,MAX_FFT_SIZE*sizeof(int));
		smooth_data_rest_hr_int(chg,SPORT_REST_HR_SPAN,inlen,ppg_diff);
		for (int num = 0;num < inlen - 1;num++)
		{
			ppg_diff[num] = ppg_diff[num + 1] - ppg_diff[num];
		}
		ppg_diff[inlen - 1] = ppg_diff[inlen - 2];

	
		// NRF_LOG_INFO("3333\n\r");
		// get_resting_hr(FFT_In_buf, inlen, rest_start_second, &g_OutPara);
		get_resting_hr(ppg_diff, inlen, rest_start_second, &g_OutPara);
		// NRF_LOG_INFO("44444\n\r");
		// NRF_LOG_INFO("==g_OutPara.showHr = %d\n\r",(int)g_OutPara.showHr);
		// NRF_LOG_INFO("==g_OutPara.cnt  = %d\n\r",g_OutPara.cnt);
		*ShowHr = g_OutPara.showHr;
		if (g_OutPara.cnt >= 3)
		{
			if (fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) > 10 && fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) < 20)
			{
				*ShowHr = (*ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 4;
			}
			else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] > 20)
			{
				//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 + 3;
				*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 + 10;
			}
			else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] < -20)
			{
				//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 - 3;
				*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 - 10;
			}

		}

		// for (int num = 0; num < g_OutPara.cnt; num++)
		// {
		// 	*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		// }
		// *ShowHr = *ShowHr / (g_OutPara.cnt + 1);
		// if(g_OutPara.cnt >= 1)
		// {
		// 	*ShowHr = *ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1];
		// 	*ShowHr = *ShowHr / 2;
		// }

		if(g_OutPara.cnt >= 2)
		{
			*ShowHr = *ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2];
			*ShowHr = *ShowHr / 3;
		}

		if (g_OutPara.cnt == 0 && *ShowHr != 0)
		{
			*ShowHr = (*ShowHr + 68) / 2;
		}
		else if(g_OutPara.cnt == 0 && *ShowHr == 0)
		{
			*ShowHr = 68;
		}


		
		// *ShowHr = *ShowHr / (g_OutPara.cnt + 1);

		// NRF_LOG_INFO("rest===== *ShowHr = %d\n",(int)(*ShowHr));
		//printf("222==test_hr_cnt = %d\n", test_hr_cnt);
		//printf("222==*ShowHr = %f\n", *ShowHr);
		//printf("222==g_OutPara.cnt = %d\n", g_OutPara.cnt);
		if(*ShowHr > SPORT_MIN_HEART && *ShowHr < SPORT_MAX_HEART)
		{
			g_pre_peak_pos = floor(*ShowHr / 60 * 2048 / 100);
			g_pre_peak_pos = g_pre_peak_pos - 1;
			//record hr
			g_OutPara.showHr = *ShowHr;
			if (g_OutPara.cnt < 4)
			{
				g_OutPara.Hrvld[g_OutPara.cnt] = g_OutPara.showHr;
				g_OutPara.cnt++;
			}
			else if (g_OutPara.cnt >= 4)
			{
				g_OutPara.Hrvld[0] = g_OutPara.Hrvld[1];
				g_OutPara.Hrvld[1] = g_OutPara.Hrvld[2];
				g_OutPara.Hrvld[2] = g_OutPara.Hrvld[3];
				g_OutPara.Hrvld[3] = g_OutPara.showHr;
			}

		}

		return;
	}

	//printf("start pre 333==g_pre_peak_pos = %d\n", g_pre_peak_pos);


	// for (int num = 0;num < inlen;num++)
	// {
	// 	fft_in_data[num] = chg[num];
	// }
	// fft_proc(fft_in_data, FFT_LENGTH);
	// for (int num = 0; num < FFT_LENGTH/2; num++)
	// {

	// 	fft_amplitude[num] = complex_abs(fft_in_data + num * 2);

	// }
	// for (int num = 0;num < inlen;num++)
	// {
	// 	FFT_In_buf[num] = chg[num];
	// }

	for (int num = 0;num < inlen;num++)
	{
		FFT_In_buf[num] = ppg_diff[num];
	}
	fft_proc(FFT_In_buf, inlen, FFT_SEARCH_LEN, FFT_Out_buf);
	for (int num = 0; num < FFT_SEARCH_LEN; num++)
	{

		fft_amplitude_search_raw[num] = sqrt(FFT_Out_buf[num]);

	}


	// for (int num = 0; num < inlen; num++)
	// {
	// 	fft_in_data[num] = acc[num];
	// }
	// fft_proc(fft_in_data, FFT_LENGTH);
	// for (int num = 0; num < FFT_LENGTH / 2; num++)
	// {

	// 	fft_amplitude_acc[num] = complex_abs(fft_in_data + num * 2);

	// }
	for (int num = 0; num < inlen; num++)
	{
		FFT_In_buf[num] = (float)acc[num];
	}
	fft_proc(FFT_In_buf, inlen, FFT_SEARCH_LEN, FFT_Out_buf);
	for (int num = 0; num < FFT_SEARCH_LEN; num++)
	{
		fft_amplitude_acc[num] = sqrt(FFT_Out_buf[num]);
	}
	int ACC_pos_cnt = 0;
	for (int num = 0;num < FFT_SEARCH_LEN;num++)
	{
		//if (fft_amplitude_acc[num] > 35000 && (num < g_pre_peak_pos - 3 || num > g_pre_peak_pos + 3))
		if (fft_amplitude_acc[num] > 35000 && (num < g_pre_peak_pos - 1 || num > g_pre_peak_pos + 1))
		{
			ACC_pos[ACC_pos_cnt] = num;
			ACC_pos_cnt++;
		}
	}
	memcpy(fft_amplitude_search_new, fft_amplitude_search_raw, FFT_SEARCH_LEN * sizeof(float));
	for (int num = 0;num < ACC_pos_cnt;num++)
	{
		fft_amplitude_search_new[ACC_pos[num]] = 0;
	}



	float peaks_value = 0;
	uint8_t peaks_pos = 0;
	float peaks_max_value = 0;
	int peaks_max_pos = 0;
	uint8_t all_zeros_flag = 0;
	if (g_pre_peak_pos == 0)
	{
		for (int num = 24;num < 75;num++)//heart 24:heart = 24*100/2048*60  75:heart = 75*100/2048*60;
		{
			if (fft_amplitude_search_new[num] > fft_amplitude_search_new[num - 1] && fft_amplitude_search_new[num] >= fft_amplitude_search_new[num + 1])
			{
				peaks_value = fft_amplitude_search_new[num];
				peaks_pos = num;
			}
			if (peaks_max_value < peaks_value)
			{
				peaks_max_value = peaks_value;
				peaks_max_pos = peaks_pos;
			}

		}
		g_pre_peak_pos = peaks_max_pos;
	}
	else
	{
		all_zeros_flag = 0;
		peaks_max_pos = 0;
		peaks_max_value = 0;
		for (int num = g_pre_peak_pos - HR_FFT_SEARCH_LEN_TWO; num < g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO; num++)
		{
			if (fft_amplitude_search_new[num] != 0)
			{
				all_zeros_flag = 1;
			}
		}

		if (all_zeros_flag == 1)
		{

			max_get(&fft_amplitude_search_new[g_pre_peak_pos - HR_FFT_SEARCH_LEN],2* HR_FFT_SEARCH_LEN + 1, &peaks_max_value,&peaks_max_pos);

			if (peaks_max_value > 300000)
			{
				g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN + peaks_max_pos;
			}
			else
			{
				if (g_pre_peak_pos - HR_FFT_SEARCH_LEN_TWO <= 26 && g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO >= 26)
				{
					max_get(&fft_amplitude_search_raw[26], g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO - 26, &peaks_max_value, &peaks_max_pos);
					g_pre_peak_pos = 26 + peaks_max_pos;
				}
				else
				{
					max_get(&fft_amplitude_search_raw[g_pre_peak_pos - HR_FFT_SEARCH_LEN_THREE], 2 * HR_FFT_SEARCH_LEN_THREE + 1, &peaks_max_value, &peaks_max_pos);
					g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN_THREE + peaks_max_pos;
				}
			}

		}
		else
		{
			max_get(&fft_amplitude_search_raw[g_pre_peak_pos - HR_FFT_SEARCH_LEN], 2 * HR_FFT_SEARCH_LEN + 1, &peaks_max_value, &peaks_max_pos);
			g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN + peaks_max_pos;

		}

	}


	*ShowHr = (g_pre_peak_pos + 1) * 1.0 * PPG_SAMPLE / FFT_LENGTH * 60;

	// NRF_LOG_INFO("move===== *ShowHr = %d\n",*ShowHr);
	//printf("333==g_pre_peak_pos = %d\n", g_pre_peak_pos);
	//printf("333==*ShowHr = %f\n", *ShowHr);
	//printf("333==g_OutPara.cnt = %d\n", g_OutPara.cnt);
	// if (g_OutPara.cnt < 2)
	// {
	// 	for (int num = 0; num < g_OutPara.cnt; num++)
	// 	{
	// 		*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
	// 	}
	// 	*ShowHr = *ShowHr / (g_OutPara.cnt + 1);

	// }
	// else
	// {
	// 	for (int num = g_OutPara.cnt - 2; num < g_OutPara.cnt; num++)
	// 	{
	// 		*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
	// 	}
	// 	*ShowHr = *ShowHr / 3;
	// }

	if (g_OutPara.cnt < 3)
	{
		for (int num = 0; num < g_OutPara.cnt; num++)
		{
			*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		}
		*ShowHr = *ShowHr / (g_OutPara.cnt + 1);

	}
	else
	{
		for (int num = g_OutPara.cnt - 3; num < g_OutPara.cnt; num++)
		{
			*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		}
		*ShowHr = *ShowHr / 4;
	}


	if (g_OutPara.cnt == 0)
	{
		*ShowHr = (*ShowHr + 68) / 2;
	}


	if(*ShowHr > SPORT_MIN_HEART && *ShowHr < SPORT_MAX_HEART)
	{
		g_pre_peak_pos = floor(*ShowHr / 60 * 2048 / 100);
		g_pre_peak_pos = g_pre_peak_pos - 1;
		//record hr
		g_OutPara.showHr = *ShowHr;
		if (g_OutPara.cnt < 4)
		{
			g_OutPara.Hrvld[g_OutPara.cnt] = g_OutPara.showHr;
			g_OutPara.cnt++;
		}
		else if (g_OutPara.cnt >= 4)
		{
			g_OutPara.Hrvld[0] = g_OutPara.Hrvld[1];
			g_OutPara.Hrvld[1] = g_OutPara.Hrvld[2];
			g_OutPara.Hrvld[2] = g_OutPara.Hrvld[3];
			g_OutPara.Hrvld[3] = g_OutPara.showHr;
		}
	}


}






void get_MoveHr_sport_daily_point(int* chg, int16_t* acc, int inlen, MOVE_HR_HISTPara* HPara, float* ShowHr)
{
	uint8_t rest_flag = 0;
	uint8_t rest_start_second = 0;
	// NRF_LOG_INFO("111111\n\r");
	memset(ppg_diff,0,MAX_FFT_SIZE*sizeof(int));
	smooth_data_sport_int(chg,SPORT_HR_SPAN,inlen,ppg_diff);
	// NRF_LOG_INFO("222222\n\r");
	// NRF_LOG_INFO("chg[0] = %d\n\r",(int)chg[0]);
	// NRF_LOG_INFO("inlen = %d\n\r",inlen);

	for (int num = 0;num < inlen - 1;num++)
	{
		ppg_diff[num] = ppg_diff[num + 1] - ppg_diff[num];
	}
	ppg_diff[inlen - 1] = ppg_diff[inlen - 2];
	//g_sport_alg_cnt = 1;
	
	// else if(g_sport_alg_cnt == 1 && inlen < MAX_FFT_SIZE)
	// else if(g_sport_alg_cnt == 1)
	// {

	// 	for (int num = inlen - SPORT_ALG_DATA_PERIOD;num < inlen - 1;num++)
	// 	{
	// 		chg[num] = chg[num + 1] - chg[num];
	// 	}
	// 	chg[inlen - 1] = chg[inlen - 2];
	// }
	// else if(g_sport_alg_cnt == 1 && inlen >= MAX_FFT_SIZE)
	// {
	// 	for (int num = inlen - SPORT_ALG_DATA_PERIOD;num < inlen - 1;num++)
	// 	{
	// 		chg[num] = chg[num + 1] - chg[num];
	// 	}
	// 	chg[inlen - 1] = chg[inlen - 2];
	// }
#if 1
	rest_flag_get(acc, inlen, &rest_flag, &rest_start_second);
	// NRF_LOG_INFO("rest_flag = %d\n\r",rest_flag);
	// NRF_LOG_INFO("rest_start_second = %d\n\r",rest_start_second);

	if (rest_flag == REST_FLAG)
	{
		// NRF_LOG_INFO("3333\n\r");
		// get_resting_hr(FFT_In_buf, inlen, rest_start_second, &g_OutPara);
		get_resting_hr(ppg_diff, inlen, rest_start_second, &g_OutPara);
		// NRF_LOG_INFO("44444\n\r");
		// NRF_LOG_INFO("==g_OutPara.showHr = %d\n\r",(int)g_OutPara.showHr);
		// NRF_LOG_INFO("==g_OutPara.cnt  = %d\n\r",g_OutPara.cnt);
		*ShowHr = g_OutPara.showHr;
		if (g_OutPara.cnt >= 3)
		{
			if (fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) > 10 && fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) < 20)
			{
				*ShowHr = (*ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 4;
			}
			else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] > 20)
			{
				//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 + 3;
				*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 + 10;
			}
			else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] < -20)
			{
				//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 - 3;
				*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 - 10;
			}

		}

		// for (int num = 0; num < g_OutPara.cnt; num++)
		// {
		// 	*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		// }
		// *ShowHr = *ShowHr / (g_OutPara.cnt + 1);
		// if(g_OutPara.cnt >= 1)
		// {
		// 	*ShowHr = *ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1];
		// 	*ShowHr = *ShowHr / 2;
		// }

		if(g_OutPara.cnt >= 2)
		{
			*ShowHr = *ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2];
			*ShowHr = *ShowHr / 3;
		}

		if (g_OutPara.cnt == 0 && *ShowHr != 0)
		{
			*ShowHr = (*ShowHr + 68) / 2;
		}
		else if(g_OutPara.cnt == 0 && *ShowHr == 0)
		{
			*ShowHr = 68;
		}


		
		// *ShowHr = *ShowHr / (g_OutPara.cnt + 1);

		// NRF_LOG_INFO("rest===== *ShowHr = %d\n",(int)(*ShowHr));
		//printf("222==test_hr_cnt = %d\n", test_hr_cnt);
		//printf("222==*ShowHr = %f\n", *ShowHr);
		//printf("222==g_OutPara.cnt = %d\n", g_OutPara.cnt);
		if(*ShowHr > SPORT_MIN_HEART && *ShowHr < SPORT_MAX_HEART)
		{
			g_pre_peak_pos = floor(*ShowHr / 60 * 2048 / 100);
			g_pre_peak_pos = g_pre_peak_pos - 1;
			//record hr
			g_OutPara.showHr = *ShowHr;
			if (g_OutPara.cnt < 4)
			{
				g_OutPara.Hrvld[g_OutPara.cnt] = g_OutPara.showHr;
				g_OutPara.cnt++;
			}
			else if (g_OutPara.cnt >= 4)
			{
				g_OutPara.Hrvld[0] = g_OutPara.Hrvld[1];
				g_OutPara.Hrvld[1] = g_OutPara.Hrvld[2];
				g_OutPara.Hrvld[2] = g_OutPara.Hrvld[3];
				g_OutPara.Hrvld[3] = g_OutPara.showHr;
			}

		}

		return;
	}

#endif
	//printf("start pre 333==g_pre_peak_pos = %d\n", g_pre_peak_pos);


	// for (int num = 0;num < inlen;num++)
	// {
	// 	fft_in_data[num] = chg[num];
	// }
	// fft_proc(fft_in_data, FFT_LENGTH);
	// for (int num = 0; num < FFT_LENGTH/2; num++)
	// {

	// 	fft_amplitude[num] = complex_abs(fft_in_data + num * 2);

	// }
	// for (int num = 0;num < inlen;num++)
	// {
	// 	FFT_In_buf[num] = chg[num];
	// }

	for (int num = 0;num < inlen;num++)
	{
		FFT_In_buf[num] = ppg_diff[num];
	}
	fft_proc(FFT_In_buf, inlen, FFT_SEARCH_LEN, FFT_Out_buf);
	for (int num = 0; num < FFT_SEARCH_LEN; num++)
	{

		fft_amplitude_search_raw[num] = sqrt(FFT_Out_buf[num]);

	}


	// for (int num = 0; num < inlen; num++)
	// {
	// 	fft_in_data[num] = acc[num];
	// }
	// fft_proc(fft_in_data, FFT_LENGTH);
	// for (int num = 0; num < FFT_LENGTH / 2; num++)
	// {

	// 	fft_amplitude_acc[num] = complex_abs(fft_in_data + num * 2);

	// }
	for (int num = 0; num < inlen; num++)
	{
		FFT_In_buf[num] = (float)acc[num];
	}
	fft_proc(FFT_In_buf, inlen, FFT_SEARCH_LEN, FFT_Out_buf);
	for (int num = 0; num < FFT_SEARCH_LEN; num++)
	{
		fft_amplitude_acc[num] = sqrt(FFT_Out_buf[num]);
	}
	int ACC_pos_cnt = 0;
	for (int num = 0;num < FFT_SEARCH_LEN;num++)
	{
		//if (fft_amplitude_acc[num] > 35000 && (num < g_pre_peak_pos - 3 || num > g_pre_peak_pos + 3))
		if (fft_amplitude_acc[num] > 35000 && (num < g_pre_peak_pos - 1 || num > g_pre_peak_pos + 1))
		{
			ACC_pos[ACC_pos_cnt] = num;
			ACC_pos_cnt++;
		}
	}
	memcpy(fft_amplitude_search_new, fft_amplitude_search_raw, FFT_SEARCH_LEN * sizeof(float));
	for (int num = 0;num < ACC_pos_cnt;num++)
	{
		fft_amplitude_search_new[ACC_pos[num]] = 0;
	}



	float peaks_value = 0;
	uint8_t peaks_pos = 0;
	float peaks_max_value = 0;
	int peaks_max_pos = 0;
	uint8_t all_zeros_flag = 0;
	if (g_pre_peak_pos == 0)
	{
		for (int num = 17;num < 45;num++)//heart 17:heart = 17*100/2048*60  45:heart = 45*100/2048*60;
		{
			if (fft_amplitude_search_new[num] > fft_amplitude_search_new[num - 1] && fft_amplitude_search_new[num] >= fft_amplitude_search_new[num + 1])
			{
				peaks_value = fft_amplitude_search_new[num];
				peaks_pos = num;
			}
			if (peaks_max_value < peaks_value)
			{
				peaks_max_value = peaks_value;
				peaks_max_pos = peaks_pos;
			}

		}
		g_pre_peak_pos = peaks_max_pos;
	}
	else
	{
		all_zeros_flag = 0;
		peaks_max_pos = 0;
		peaks_max_value = 0;
		for (int num = g_pre_peak_pos - HR_FFT_SEARCH_LEN_TWO; num < g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO; num++)
		{
			if (fft_amplitude_search_new[num] != 0)
			{
				all_zeros_flag = 1;
			}
		}

		if (all_zeros_flag == 1)
		{

			if(g_pre_peak_pos - HR_FFT_SEARCH_LEN < 17)
			{
				g_pre_peak_pos = 17 + HR_FFT_SEARCH_LEN;
			}
			if(g_pre_peak_pos +  HR_FFT_SEARCH_LEN + 1 > 45)
			{
				g_pre_peak_pos = 45 - 1 - HR_FFT_SEARCH_LEN;
			}

			max_get(&fft_amplitude_search_new[g_pre_peak_pos - HR_FFT_SEARCH_LEN],2* HR_FFT_SEARCH_LEN + 1, &peaks_max_value,&peaks_max_pos);

			if (peaks_max_value > 300000)
			{
				g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN + peaks_max_pos;
			}
			else
			{
				if (g_pre_peak_pos - HR_FFT_SEARCH_LEN_TWO <= 17 && g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO >= 17)
				{
					max_get(&fft_amplitude_search_raw[17], g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO - 17, &peaks_max_value, &peaks_max_pos);
					g_pre_peak_pos = 17 + peaks_max_pos;
				}
				else
				{
					max_get(&fft_amplitude_search_raw[g_pre_peak_pos - HR_FFT_SEARCH_LEN_THREE], 2 * HR_FFT_SEARCH_LEN_THREE + 1, &peaks_max_value, &peaks_max_pos);
					g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN_THREE + peaks_max_pos;
				}
			}

		}
		else
		{
			if(g_pre_peak_pos - HR_FFT_SEARCH_LEN < 17)
			{
				g_pre_peak_pos = 17 + HR_FFT_SEARCH_LEN;
			}
			if(g_pre_peak_pos +  HR_FFT_SEARCH_LEN + 1 > 45)
			{
				g_pre_peak_pos = 45 - 1 - HR_FFT_SEARCH_LEN;
			}
			max_get(&fft_amplitude_search_raw[g_pre_peak_pos - HR_FFT_SEARCH_LEN], 2 * HR_FFT_SEARCH_LEN + 1, &peaks_max_value, &peaks_max_pos);
			g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN + peaks_max_pos;

		}

	}

	if(g_pre_peak_pos == 0)
	{
		*ShowHr = 68;
	}
	else{
		*ShowHr = (g_pre_peak_pos + 1) * 1.0 * PPG_SAMPLE / FFT_LENGTH * 60;
	}
	
	
	// NRF_LOG_INFO("move===== *ShowHr = %d\n",*ShowHr);
	//printf("333==g_pre_peak_pos = %d\n", g_pre_peak_pos);
	//printf("333==*ShowHr = %f\n", *ShowHr);
	//printf("333==g_OutPara.cnt = %d\n", g_OutPara.cnt);
	// if (g_OutPara.cnt < 2)
	// {
	// 	for (int num = 0; num < g_OutPara.cnt; num++)
	// 	{
	// 		*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
	// 	}
	// 	*ShowHr = *ShowHr / (g_OutPara.cnt + 1);

	// }
	// else
	// {
	// 	for (int num = g_OutPara.cnt - 2; num < g_OutPara.cnt; num++)
	// 	{
	// 		*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
	// 	}
	// 	*ShowHr = *ShowHr / 3;
	// }

	if (g_OutPara.cnt < 3)
	{
		for (int num = 0; num < g_OutPara.cnt; num++)
		{
			*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		}
		*ShowHr = *ShowHr / (g_OutPara.cnt + 1);

	}
	else
	{
		for (int num = g_OutPara.cnt - 3; num < g_OutPara.cnt; num++)
		{
			*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		}
		*ShowHr = *ShowHr / 4;
	}


	if (g_OutPara.cnt == 0)
	{
		*ShowHr = (*ShowHr + 68) / 2;
	}


	if (g_OutPara.cnt >= 1)
	{
		if (fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) > 10 && fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) < 20)
		{
			*ShowHr = (*ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1]) / 2;
		}
		else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] > 20)
		{
			//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 + 3;
			*ShowHr = g_OutPara.Hrvld[g_OutPara.cnt - 1] + 7;
		}
		else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] < -20)
		{
			//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 - 3;
			*ShowHr = g_OutPara.Hrvld[g_OutPara.cnt - 1] - 7;
		}

	}



	if(*ShowHr > SPORT_MIN_HEART && *ShowHr < SPORT_MAX_HEART)
	{
		g_pre_peak_pos = floor(*ShowHr / 60 * 2048 / 100);
		g_pre_peak_pos = g_pre_peak_pos - 1;
		//record hr
		g_OutPara.showHr = *ShowHr;
		if (g_OutPara.cnt < 4)
		{
			g_OutPara.Hrvld[g_OutPara.cnt] = g_OutPara.showHr;
			g_OutPara.cnt++;
		}
		else if (g_OutPara.cnt >= 4)
		{
			g_OutPara.Hrvld[0] = g_OutPara.Hrvld[1];
			g_OutPara.Hrvld[1] = g_OutPara.Hrvld[2];
			g_OutPara.Hrvld[2] = g_OutPara.Hrvld[3];
			g_OutPara.Hrvld[3] = g_OutPara.showHr;
		}
	}


}


void get_MoveHr_sport_daily_point_new(int* chg, int16_t* acc, int inlen, MOVE_HR_HISTPara* HPara, float* ShowHr)
{
	uint8_t hr_flag = 0;
	uint8_t rest_flag = 0;
	uint8_t rest_start_second = 0;
	// NRF_LOG_INFO("111111\n\r");
	memset(ppg_diff,0,MAX_FFT_SIZE*sizeof(int));
	smooth_data_sport_int(chg,SPORT_HR_SPAN,inlen,ppg_diff);
	// NRF_LOG_INFO("222222\n\r");
	// NRF_LOG_INFO("chg[0] = %d\n\r",(int)chg[0]);
	// NRF_LOG_INFO("inlen = %d\n\r",inlen);

	for (int num = 0;num < inlen - 1;num++)
	{
		ppg_diff[num] = ppg_diff[num + 1] - ppg_diff[num];
	}
	ppg_diff[inlen - 1] = ppg_diff[inlen - 2];
	//g_sport_alg_cnt = 1;
	
	// else if(g_sport_alg_cnt == 1 && inlen < MAX_FFT_SIZE)
	// else if(g_sport_alg_cnt == 1)
	// {

	// 	for (int num = inlen - SPORT_ALG_DATA_PERIOD;num < inlen - 1;num++)
	// 	{
	// 		chg[num] = chg[num + 1] - chg[num];
	// 	}
	// 	chg[inlen - 1] = chg[inlen - 2];
	// }
	// else if(g_sport_alg_cnt == 1 && inlen >= MAX_FFT_SIZE)
	// {
	// 	for (int num = inlen - SPORT_ALG_DATA_PERIOD;num < inlen - 1;num++)
	// 	{
	// 		chg[num] = chg[num + 1] - chg[num];
	// 	}
	// 	chg[inlen - 1] = chg[inlen - 2];
	// }
#if 1
	rest_flag_get(acc, inlen, &rest_flag, &rest_start_second);
	// NRF_LOG_INFO("rest_flag = %d\n\r",rest_flag);
	// NRF_LOG_INFO("rest_start_second = %d\n\r",rest_start_second);

	if (rest_flag == REST_FLAG)
	{
		// NRF_LOG_INFO("3333\n\r");
		// get_resting_hr(FFT_In_buf, inlen, rest_start_second, &g_OutPara);
		get_resting_hr(ppg_diff, inlen, rest_start_second, &g_OutPara);
		// NRF_LOG_INFO("44444\n\r");
		// NRF_LOG_INFO("==g_OutPara.showHr = %d\n\r",(int)g_OutPara.showHr);
		// NRF_LOG_INFO("==g_OutPara.cnt  = %d\n\r",g_OutPara.cnt);

		//rest heart process 2
		g_OutPara.showHr = (int)(g_OutPara.showHr)/2*2;

		*ShowHr = g_OutPara.showHr;
		// if(*ShowHr > 100)
		// {
		// 	hr_flag = 1;
		// }
		// if(hr_flag == 1)
		// {
		// 	*ShowHr = 188;
		// }



		// if (g_OutPara.cnt >= 3)
		// {
		// 	if (fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) > 10 && fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) < 20)
		// 	{
		// 		*ShowHr = (*ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 4;
		// 	}
		// 	else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] > 20)
		// 	{
		// 		//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 + 3;
		// 		*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 + 10;
		// 	}
		// 	else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] < -20)
		// 	{
		// 		//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 - 3;
		// 		*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 - 10;
		// 	}

		// }

		// for (int num = 0; num < g_OutPara.cnt; num++)
		// {
		// 	*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		// }
		// *ShowHr = *ShowHr / (g_OutPara.cnt + 1);
		// if(g_OutPara.cnt >= 1)
		// {
		// 	*ShowHr = *ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1];
		// 	*ShowHr = *ShowHr / 2;
		// }

		// if(g_OutPara.cnt >= 2)
		// {
		// 	*ShowHr = *ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2];
		// 	*ShowHr = *ShowHr / 3;
		// }

		// if (g_OutPara.cnt == 0 && *ShowHr != 0)
		// {
		// 	*ShowHr = (*ShowHr + 68) / 2;
		// }
		// else if(g_OutPara.cnt == 0 && *ShowHr == 0)
		// {
		// 	*ShowHr = 68;
		// }

		if(g_OutPara.cnt == 0 && *ShowHr == 0)
		{
			*ShowHr = 68;
		}
		
		// *ShowHr = *ShowHr / (g_OutPara.cnt + 1);

		// NRF_LOG_INFO("rest===== *ShowHr = %d\n",(int)(*ShowHr));
		//printf("222==test_hr_cnt = %d\n", test_hr_cnt);
		//printf("222==*ShowHr = %f\n", *ShowHr);
		//printf("222==g_OutPara.cnt = %d\n", g_OutPara.cnt);
		if(*ShowHr > SPORT_MIN_HEART && *ShowHr < SPORT_MAX_HEART)
		{
			g_point_pre_peak_pos = floor(*ShowHr / 60 * 2048 / 100);
			g_point_pre_peak_pos = g_point_pre_peak_pos - 1;
			// //record hr
			// g_OutPara.showHr = *ShowHr;
			// if (g_OutPara.cnt < 4)
			// {
			// 	g_OutPara.Hrvld[g_OutPara.cnt] = g_OutPara.showHr;
			// 	g_OutPara.cnt++;
			// }
			// else if (g_OutPara.cnt >= 4)
			// {
			// 	g_OutPara.Hrvld[0] = g_OutPara.Hrvld[1];
			// 	g_OutPara.Hrvld[1] = g_OutPara.Hrvld[2];
			// 	g_OutPara.Hrvld[2] = g_OutPara.Hrvld[3];
			// 	g_OutPara.Hrvld[3] = g_OutPara.showHr;
			// }

		}

		return;
	}

#endif
	//printf("start pre 333==g_pre_peak_pos = %d\n", g_pre_peak_pos);


	// for (int num = 0;num < inlen;num++)
	// {
	// 	fft_in_data[num] = chg[num];
	// }
	// fft_proc(fft_in_data, FFT_LENGTH);
	// for (int num = 0; num < FFT_LENGTH/2; num++)
	// {

	// 	fft_amplitude[num] = complex_abs(fft_in_data + num * 2);

	// }
	// for (int num = 0;num < inlen;num++)
	// {
	// 	FFT_In_buf[num] = chg[num];
	// }

	for (int num = 0;num < inlen;num++)
	{
		FFT_In_buf[num] = ppg_diff[num];
	}
	fft_proc(FFT_In_buf, inlen, FFT_SEARCH_LEN, FFT_Out_buf);
	for (int num = 0; num < FFT_SEARCH_LEN; num++)
	{

		fft_amplitude_search_raw[num] = sqrt(FFT_Out_buf[num]);

	}


	// for (int num = 0; num < inlen; num++)
	// {
	// 	fft_in_data[num] = acc[num];
	// }
	// fft_proc(fft_in_data, FFT_LENGTH);
	// for (int num = 0; num < FFT_LENGTH / 2; num++)
	// {

	// 	fft_amplitude_acc[num] = complex_abs(fft_in_data + num * 2);

	// }


	g_pre_peak_pos = g_point_pre_peak_pos;

	for (int num = 0; num < inlen; num++)
	{
		FFT_In_buf[num] = (float)acc[num];
	}
	fft_proc(FFT_In_buf, inlen, FFT_SEARCH_LEN, FFT_Out_buf);
	for (int num = 0; num < FFT_SEARCH_LEN; num++)
	{
		fft_amplitude_acc[num] = sqrt(FFT_Out_buf[num]);
	}
	int ACC_pos_cnt = 0;
	for (int num = 0;num < FFT_SEARCH_LEN;num++)
	{
		//if (fft_amplitude_acc[num] > 35000 && (num < g_pre_peak_pos - 3 || num > g_pre_peak_pos + 3))
		if (fft_amplitude_acc[num] > 35000 && (num < g_pre_peak_pos - 1 || num > g_pre_peak_pos + 1))
		{
			ACC_pos[ACC_pos_cnt] = num;
			ACC_pos_cnt++;
		}
	}
	memcpy(fft_amplitude_search_new, fft_amplitude_search_raw, FFT_SEARCH_LEN * sizeof(float));
	for (int num = 0;num < ACC_pos_cnt;num++)
	{
		fft_amplitude_search_new[ACC_pos[num]] = 0;
	}



	float peaks_value = 0;
	uint8_t peaks_pos = 0;
	float peaks_max_value = 0;
	int peaks_max_pos = 0;
	uint8_t all_zeros_flag = 0;
	if (g_pre_peak_pos == 0)
	{
		for (int num = 17;num < 45;num++)//heart 17:heart = 17*100/2048*60  45:heart = 45*100/2048*60;
		{
			if (fft_amplitude_search_new[num] > fft_amplitude_search_new[num - 1] && fft_amplitude_search_new[num] >= fft_amplitude_search_new[num + 1])
			{
				peaks_value = fft_amplitude_search_new[num];
				peaks_pos = num;
			}
			if (peaks_max_value < peaks_value)
			{
				peaks_max_value = peaks_value;
				peaks_max_pos = peaks_pos;
			}

		}
		g_pre_peak_pos = peaks_max_pos;
	}
	else
	{
		all_zeros_flag = 0;
		peaks_max_pos = 0;
		peaks_max_value = 0;
		for (int num = g_pre_peak_pos - HR_FFT_SEARCH_LEN_TWO; num < g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO; num++)
		{
			if (fft_amplitude_search_new[num] != 0)
			{
				all_zeros_flag = 1;
			}
		}

		if (all_zeros_flag == 1)
		{

			if(g_pre_peak_pos - HR_FFT_SEARCH_LEN < 17)
			{
				g_pre_peak_pos = 17 + HR_FFT_SEARCH_LEN;
			}
			if(g_pre_peak_pos +  HR_FFT_SEARCH_LEN + 1 > 45)
			{
				g_pre_peak_pos = 45 - 1 - HR_FFT_SEARCH_LEN;
			}

			max_get(&fft_amplitude_search_new[g_pre_peak_pos - HR_FFT_SEARCH_LEN],2* HR_FFT_SEARCH_LEN + 1, &peaks_max_value,&peaks_max_pos);

			if (peaks_max_value > 300000)
			{
				g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN + peaks_max_pos;
			}
			else
			{
				if (g_pre_peak_pos - HR_FFT_SEARCH_LEN_TWO <= 17 && g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO >= 17)
				{
					max_get(&fft_amplitude_search_raw[17], g_pre_peak_pos + HR_FFT_SEARCH_LEN_TWO - 17, &peaks_max_value, &peaks_max_pos);
					g_pre_peak_pos = 17 + peaks_max_pos;
				}
				else
				{
					max_get(&fft_amplitude_search_raw[g_pre_peak_pos - HR_FFT_SEARCH_LEN_THREE], 2 * HR_FFT_SEARCH_LEN_THREE + 1, &peaks_max_value, &peaks_max_pos);
					g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN_THREE + peaks_max_pos;
				}
			}

		}
		else
		{
			if(g_pre_peak_pos - HR_FFT_SEARCH_LEN < 17)
			{
				g_pre_peak_pos = 17 + HR_FFT_SEARCH_LEN;
			}
			if(g_pre_peak_pos +  HR_FFT_SEARCH_LEN + 1 > 45)
			{
				g_pre_peak_pos = 45 - 1 - HR_FFT_SEARCH_LEN;
			}
			max_get(&fft_amplitude_search_raw[g_pre_peak_pos - HR_FFT_SEARCH_LEN], 2 * HR_FFT_SEARCH_LEN + 1, &peaks_max_value, &peaks_max_pos);
			g_pre_peak_pos = g_pre_peak_pos - HR_FFT_SEARCH_LEN + peaks_max_pos;

		}

	}

	if(g_pre_peak_pos == 0)
	{
		*ShowHr = 68;
	}
	else{
		*ShowHr = (g_pre_peak_pos + 1) * 1.0 * PPG_SAMPLE / FFT_LENGTH * 60;
	}
	
	
	// NRF_LOG_INFO("move===== *ShowHr = %d\n",*ShowHr);
	//printf("333==g_pre_peak_pos = %d\n", g_pre_peak_pos);
	//printf("333==*ShowHr = %f\n", *ShowHr);
	//printf("333==g_OutPara.cnt = %d\n", g_OutPara.cnt);
	// if (g_OutPara.cnt < 2)
	// {
	// 	for (int num = 0; num < g_OutPara.cnt; num++)
	// 	{
	// 		*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
	// 	}
	// 	*ShowHr = *ShowHr / (g_OutPara.cnt + 1);

	// }
	// else
	// {
	// 	for (int num = g_OutPara.cnt - 2; num < g_OutPara.cnt; num++)
	// 	{
	// 		*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
	// 	}
	// 	*ShowHr = *ShowHr / 3;
	// }

	if (g_OutPara.cnt < 3)
	{
		for (int num = 0; num < g_OutPara.cnt; num++)
		{
			*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		}
		*ShowHr = *ShowHr / (g_OutPara.cnt + 1);

	}
	else
	{
		for (int num = g_OutPara.cnt - 3; num < g_OutPara.cnt; num++)
		{
			*ShowHr = *ShowHr + g_OutPara.Hrvld[num];
		}
		*ShowHr = *ShowHr / 4;
	}


	if (g_OutPara.cnt == 0)
	{
		*ShowHr = (*ShowHr + 68) / 2;
	}


	if (g_OutPara.cnt >= 1)
	{
		if (fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) > 10 && fabs(*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1]) < 20)
		{
			*ShowHr = (*ShowHr + g_OutPara.Hrvld[g_OutPara.cnt - 1]) / 2;
		}
		else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] > 20)
		{
			//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 + 3;
			*ShowHr = g_OutPara.Hrvld[g_OutPara.cnt - 1] + 7;
		}
		else if (*ShowHr - g_OutPara.Hrvld[g_OutPara.cnt - 1] < -20)
		{
			//*ShowHr = (g_OutPara.Hrvld[g_OutPara.cnt - 1] + g_OutPara.Hrvld[g_OutPara.cnt - 2] + g_OutPara.Hrvld[g_OutPara.cnt - 3]) / 3 - 3;
			*ShowHr = g_OutPara.Hrvld[g_OutPara.cnt - 1] - 7;
		}

	}

	*ShowHr = (int)(*ShowHr)/2*2 + 1;

	if(*ShowHr > SPORT_MIN_HEART && *ShowHr < SPORT_MAX_HEART)
	{
		g_pre_peak_pos = floor(*ShowHr / 60 * 2048 / 100);
		g_pre_peak_pos = g_pre_peak_pos - 1;
		//record hr
		g_OutPara.showHr = *ShowHr;
		if (g_OutPara.cnt < 4)
		{
			g_OutPara.Hrvld[g_OutPara.cnt] = g_OutPara.showHr;
			g_OutPara.cnt++;
		}
		else if (g_OutPara.cnt >= 4)
		{
			g_OutPara.Hrvld[0] = g_OutPara.Hrvld[1];
			g_OutPara.Hrvld[1] = g_OutPara.Hrvld[2];
			g_OutPara.Hrvld[2] = g_OutPara.Hrvld[3];
			g_OutPara.Hrvld[3] = g_OutPara.showHr;
		}
	}


}




float g_pre_showHr = 0;
void get_MoveHr(int *chg, int16_t *acc,int inlen, MOVE_HR_HISTPara *HPara, float *ShowHr)
{
	float samplerate = 100;
	MOVE_HR_Para *HistParaVect = HPara->HistParaVect;
	*ShowHr = 0;

	int moveflg = 0;
	float Accpwr = 0;
	int fftoutlen = FFT_MAX_OUT_LEN;
	static int MovePosCntHist = 0;
	static int MovePosHist = 0;

	int id = HPara->SavePos % MAX_SAVED_HIST_LEN;
	get_HR_inpeace(chg, acc, inlen, HPara->HistParaVect+id);

	if (HPara->HistParaVect[id].cnt > 0)
	{
		if (HPara->SavePos == 0 && HistParaVect[id].Hrvld[0] > 0)
		{
			*ShowHr = HistParaVect[id].showHr;
			if (HistParaVect[id].Hrvld[0] >= 4)
			{
				HPara->SavePos++;
				if (*ShowHr < HPara->MinHrpeace)
				{
					HPara->MinHrpeace = *ShowHr;
				}
				HPara->HistMvPos = 0;
				HPara->MvLastTime = 0;
				HPara->KeepCnt = 0;
			}
            g_pre_showHr = *ShowHr;
			return;
		}

		if (HPara->SavePos == 0)
		{
    		g_pre_showHr = HistParaVect[id].showHr;
			*ShowHr = g_pre_showHr;
			return;
		}

		int preid = (HPara->SavePos - 1) % MAX_SAVED_HIST_LEN;
		if (fabs(HistParaVect[id].showHr - HistParaVect[preid].showHr) < 10)
		{
			*ShowHr = (HistParaVect[id].showHr + HistParaVect[preid].showHr) / 2;
			HistParaVect[id].showHr = *ShowHr;
			HistParaVect[id].Hrvld[0] = 20;
			HPara->SavePos++;
			if (HistParaVect[id].showHr < HPara->MinHrpeace)
			{
				HPara->MinHrpeace = HistParaVect[id].showHr;
			}
			HPara->HistMvPos = 0;
			HPara->MvLastTime = 0;
			HPara->KeepCnt = 0;
            g_pre_showHr = *ShowHr;
			return;
		}
		else
		{
			if (HistParaVect[id].Hrvld[0] > 3)
			{
				*ShowHr = HistParaVect[id].showHr * 0.2 + HistParaVect[preid].showHr * 0.8;
				HistParaVect[id].showHr = *ShowHr;
				HPara->SavePos++;
				if (HistParaVect[id].showHr < HPara->MinHrpeace)
				{
					HPara->MinHrpeace = HistParaVect[id].showHr;
				}
				HPara->HistMvPos = 0;
				HPara->MvLastTime = 0;
				HPara->KeepCnt = 0;
                g_pre_showHr = *ShowHr;
				return;
			}
			*ShowHr = g_pre_showHr;
            return;
		}
	}

	if (inlen < FFT_PROC_LEN)
	{
    	*ShowHr = g_pre_showHr;
		return;
	}

	get_HR_inmove(chg, acc, inlen, HPara, ShowHr);


}


uint8_t g_off_hand_check_flag = OFF_HAND_OPEN;
void off_hand_check_set(uint8_t flag)
{
	g_off_hand_check_flag = flag;
}
uint8_t off_hand_check_get(void)
{
	return g_off_hand_check_flag;
}
//point mode
extern int frameno_daily;
uint8_t point_data_bufProc(void *afe_in, uint8_t frame_length, uint8_t *offhand_flag)
{
    if(afe_in == NULL || frame_length <= 0)
        return 0;

    *offhand_flag = g_afe.offhand_flg;
    struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)afe_in;
    //int tmpredval;
    //int tmpledamb = afe_bpt[0].phase2;
    //int test_green = afe_bpt[0].phase3;
    int led1val = 0;
    int cofflen = 7;
	float filterPPGData = 0;
    //ehr_dt_ofline.min_ambval = afe_bpt[0].phase2;
    for(int i = 0; i < frame_length; i++)
    {
		//stress and fatigue daily modules
		if(hrv_health_process_flag_get() == HRV_HEALTH_PROCESS_START_FLAG)
		{
			// fifo32_putPut(&green_health_fifo, afe_bpt[i].phase1 - afe_bpt[i].phase2);
			fifo32_putPut(&green_health_fifo, afe_bpt[i].phase1);// daily L1-A1,PD2:sample Ambient  no again delete ambient
			HRV_health_data_get();
			//HRV_health_data_get_new();
			//HRV_health_data_get_daily_algorithm_rr();
		}

        led1val = afe_bpt[i].phase1;
        offdac_para.max_led1_val = max(offdac_para.max_led1_val, led1val);
        double coffa[7] = { 1.000000000000000e+000, -5.547044191350068e+000, 1.285541954468899e+001, -1.593478169199447e+001, 1.114327437040806e+001, -4.168627535934345e+000, 6.517601971221265e-001 };
        double coffb[7] = { 9.951735617670138e-004, 0, -2.985520685301042e-003, 0, 2.985520685301042e-003, 0, -9.951735617670138e-004 };
        
        if (datano < cofflen - 1)
        {
            tmpgreeninbuf[datano] = afe_bpt[i].phase1;
            fifo32_putPut(&EHR_green_fifo, 0);
            ch_green_ep++;
            datano++;
        }
        else
        {
            if(afe_bpt[i].phase1 > EHR_LEDGREEN_THRES)
            {
                tmpgreeninbuf[6] = tmpgreeninbuf[5];
            }
            else
            {
                tmpgreeninbuf[6] = afe_bpt[i].phase1;
				tmpgreeninbuf[6] = afe_bpt[i].phase1 - afe_bpt[i].phase2;
            }
            datano++;
            ch_green_ep++;
            // tmpgreenoutbuf[6] = EHR_get_filter(tmpgreeninbuf, coffa, coffb, cofflen, tmpgreenoutbuf);
			// fifo32_putPut(&EHR_green_fifo, (int)tmpgreenoutbuf[6]);

			filterPPGData = afe_bpt[i].phase1;// daily L1-A1,PD2:sample Ambient  no again delete ambient
			filterPPGData = IIR_bandpass_chebyshev_Filter_heart_sport(filterPPGData);
            fifo32_putPut(&EHR_green_fifo, (int)filterPPGData);

            memcpy(tmpgreeninbuf, tmpgreeninbuf + 1, sizeof(int)*(cofflen - 1));
            memcpy(tmpgreenoutbuf, tmpgreenoutbuf + 1, sizeof(double)*(cofflen - 1));
        }

        int tmpledamb = afe_bpt[i].phase1;
        if(offline_ehr_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 2)
            {
                // if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase1 + OFFLINE_THRE))
				if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase1 + OFFLINE_THRE_TWO))
                {
                    ehr_dt_ofline.offline_num++;
                }
                else
                {
                    if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && afe_bpt[i + 1].phase1 > (afe_bpt[i + 2].phase1 + OFFLINE_THRE))
                    {
                        ehr_dt_ofline.offline_num++;
                    }
                }
            }
        }  
    }

	if(off_hand_check_get() == OFF_HAND_OPEN)
	{
		//off hand detect
		#if ONHAND_Point_Proc
			// if(!(frameno_daily%(190/frame_length)))
			// {
				// if(offline_ehr_ledflg == OFFLINE_LED_ON && frameno_daily >= 2)  //when top green led is on, do offhand detect
				if(offline_ehr_ledflg == OFFLINE_LED_ON && frameno_daily >= 1)  //when top green led is on, do offhand detect
				{
					Detect_offline_ehr();
					*offhand_flag = g_afe.offhand_flg;
				}

				int change_step = Point_ONHAND_INTERVAL/frame_length;

				if(!(frameno_daily%change_step))   //turn on top green led
				{
					if(offline_ehr_ledflg == OFFLINE_LED_OFF)
					{
						ring_topled_on();
						offline_ehr_ledflg = OFFLINE_LED_ON;
					}
				}
			// }
		#endif
	}


    frameno_daily++;
    int frameno_num =  frameno_daily * frame_length;
        
    if(!(frameno_num % AFE_SAMPLERATE)) //1s
    {
        #if 0
        if(!(frameno_num % off_dc_time))//2s
        {
            if(frameno_num > 1000)  //detect dc
            {
                if(offdac_para.max_led1_val > 1.9e6)
                {
                    set_off_dac(-1);//off_dc -1
                }
                if(offdac_para.max_led1_val < -9e5)
                {
                    set_off_dac(0);//off_dc set 0
                }
            }
            offdac_para.max_led1_val = -2e6;
        }
        #endif
        
        if(g_afe.ehr_flag == SPO2_HR_INVALID)
        {
            g_afe.ehr_hr = 0;
        }
        else
        {
            // if(frameno_num < 800) // 8s
			if(frameno_num < 2000) // 8s
            {
                g_afe.ehr_flag = SPO2_HR_READYING;
            }
            else
            {
                if(g_afe.ehr_flag != SPO2_HR_VALID)
                {
                    g_afe.ehr_flag = SPO2_HR_VALID;
                }
                return 1;
            }
        }
    }
    return 0;
}

uint8_t afe_point_proc(void)
{
    uint8_t prvalflg = 0;

    if(g_afe.ehr_flag != SPO2_HR_INVALID)
    {
        // EHR_Proc_shedule(EHR_acc_fifo.buf, EHR_green_fifo.buf,100,&g_afe.point_hr);
		daily_point_EHR_Proc_shedule(EHR_acc_fifo.buf, EHR_green_fifo.buf,100,&g_afe.point_hr);
    }
    else
    {
        g_afe.point_hr = 0;
    }
    
    //int thres_frameno = 25*100/50;
    if(g_afe.point_hr != 0 && frameno_daily > POINT_DAILY_HEART_PROCESS_TIME)
    {        
        if(g_afe.offhand_flg == 0)
        {
            prvalflg = 1;
        }
    }

    NRF_LOG_INFO("hr : %d, prvalflg: %d\r\n",g_afe.point_hr,prvalflg);
    return prvalflg;
}