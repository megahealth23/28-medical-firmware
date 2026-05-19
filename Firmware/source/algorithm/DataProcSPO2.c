/*********************************************************************
* @fn      afe_drv.c
*
* @brief   afe4404 driver.
*
*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "nrf_delay.h"
#include <math.h>
#include "arm_math.h"

#include "DataProcSPO2.h"
#include "DataProcEHR.h"
#include "filter_datapre.h"
#include "ring_afe.h"



//#include "ring_acc.h"
//#include "MRingBLEService.h"
//#include "mring_config.h"
//#include "mring_queue.h"
//#include "mring_debug.h"
#include "../WaveSrc/wavelib.h"
//#include "mring_bsp.h"
#include "I2C.h"
#include "nrf_log.h"
#include "../services/ble_rawdata.h"

//#include "DataProcEHR.h"
//#include "MringBleLog\mring_bleLog.h"
//#include "mring_acc.h"
//#include "mring_state.h"

#include "DataProcHealth.h"


#ifndef DEBUG_RAWDATA

#define DEBUG_RAWDATA
#define DEBUG_FLuke 
#define DEBUG_FLuke_2 

#endif
static int *spo2_red = NULL;
static int *spo2_infra = NULL;
static int *spo2_green = NULL;
static int *wrcoefbuf = NULL;
//add 21.9.1
static int *wrcoefbuf_base = NULL;

static int *AccData = NULL;
uint8_t *HistSpo2 = NULL;
uint8_t *HistAcc = NULL;
static int *Acc_Z_Buf = NULL;

//add  23.7.12
static int *red_filter = NULL;
static int *infra_filter = NULL;
#define MAX_ZEROS_NUM 100
static int *zerosVect = NULL;//[MAX_ZEROS_NUM];
//add 23.7.15
#define MAX_ACDC_NUM 50
static float *infra_acdc = NULL;
static float *red_acdc = NULL;
static float *hr_rate = NULL;

AlgData_t Hist_Hr_Rate = 0;

static int HRV_cnt = 2;
// static int HRV_cnt = 0;

uint16_t totalHrCnt = 0;
// uint32_t totalHrCnt = 0;
uint8_t HRV_vect[MAXNUM_HRVvect] = {0};
// uint16_t HRV_vect[MAXNUM_HRVvect] = {0};

int MotionState[MS_datalength] = {0};
int length_accval = 13;
AlgProc_Para AlgPara;
//LEDRED_DETECT red_detec;

FIFO32 spo2_red_fifo, spo2_infra_fifo, spo2_green_fifo;
FIFO32 spo2_red_filter, spo2_infra_filter;
//init_wavlet wavelet_db4={1024,8};
ACCData accdata = {0};
uint8_t accval = 0;
uint8_t Moveflg = 0;
uint8_t summotionstate = 0;
#define acc_thre   10
uint8_t acc_flag = 0;
uint8_t acc_duration = 0;
uint16_t g_rr_mean = 0;//daily sleep heart / spo2 sleep heart / realtime heart
//AlgData_t *wrcoefbuf = Ch_fft_inputBuf;
wave_object obj = NULL;
wt_object wt = NULL;

T_AFE_DATA g_afe = {
    .spo2 = 0xF4, 
    .hr = 0x44,
    .Spo2show = 0xF4,
    .shr = 0x44,
    .sleep_status = INVALID_STAGE,
    .rr_var = 0,
    .breathrate = 0
};
int AFE44xx_SPO2_Data_buf[2] = { 0 };

bool LED1readyfor_calc = false,LED2readyfor_calc = false;

int meandatanum = 0;
static int *meanredpwrbuf = NULL;
static int *meaninfrapwrbuf = NULL;
int redlowernum = 0;
uint8_t startchange = 0;
int changecnt = 0;
int cuaddval = 0;
uint8_t cuinitflag = 0;
uint8_t cuchangeflag = 0;
#ifndef FIXED_CURRENT
uint32_t reg22val = 0x000000|(10 << 12)|(10 << 6);
#else
uint32_t reg22val = 0x000000|(12 << 12)|(12 << 6)|12;
#endif
int initdonelength = 0;
int onestep = 0;
int preredval = 0;
// int infrathreslow = 3.5e5;
int infrathreslow = 5e5;
int infrathreshigh = 1.8e6;
AlgData_t HistRateVect[(int)(WAVE_PROC_LEN / 100)] = { 0,0,0,0,0,0,0,0,0,0 };
int HistRateCnt = 0;
//AlgData_t HistSRateVect[(int)(WAVE_PROC_LEN / 100)] = { 0,0,0,0,0,0,0,0,0,0 };
int HistShowCnt = 0;

AlgData_t RateVect[(int)(WAVE_PROC_LEN / 100)];

DETECT_OFFLINE dt_ofline;
LED_Current_Level led_reg = {0, 0};
uint8_t offline_ledflg = OFFLINE_LED_OFF;

//2020.11.10 add hr show
#define HIST_HR_LEN  2//4 //60
uint8_t HistHeartrate[HIST_HR_LEN];
int HistHrsavepos = 0;

#define ONHAND_Spo2_Proc  1
#define ONHAND_Spo2Daily_Proc  1
#define Spo2_ONHAND_INTERVAL   3000//(30000)//(12000)//6000

#ifndef diffvalue

#define max(x1,x2) ((x1)>(x2)?(x1):(x2))
#define min(x1,x2) ((x1)<(x2)?(x1):(x2))
#define diffvalue(x1,x2) (x1)>(x2)?(x1-x2):(x2-x1) 

#endif

// 2021.04.15 add
#define Tired_Proc   0

#define HRV_PROC_STEP 2
#define FFT_SPO2_LEN 512
//float SPO2_FFT_In[FFT_SPO2_LEN + 10];   //522*4
//float SPO2_FFT_Out[FFT_SPO2_LEN + 10];  //522*4
float *SPO2_FFT_In = NULL;
float *SPO2_FFT_Out = NULL;
int Spo2deflg = 0;

//uint8_t HistRR[MAX_HIST_LEN];           //600
uint8_t *HistRR = NULL;
int HistRRId = 0;
//float HistHRVVect[MAX_HIST_LEN];        //600*4
//uint8_t HistHRVMVect[MAX_HIST_LEN];     //600
float *HistHRVVect = NULL;
uint8_t *HistHRVMVect = NULL;
int HistHRVId = 0;
//uint8_t HistHrVld[HIST_HR_LEN];         //60
//uint8_t HistHrVect[FFT_SPO2_LEN + 60];  //572
uint8_t *HistHrVld = NULL;
uint8_t *HistHrVect = NULL;
int HistHrVectId = 0;
ProcPara SPO2ProcPara;

#define complex_pow(x) ((x)[0]*(x)[0] + (x)[1]*(x)[1])
#define complex_abs(x) sqrt(complex_pow(x))

#define MAXVAR_THRESHOLD 0.002
#define MAXVAR_CNT 30
AlgData_t Varthreshold = MAXVAR_THRESHOLD / 2;
AlgData_t HisVarvect[MAXVAR_CNT];
int HisVarCnt = 0;

AlgData_t HistRate = 0;

//21.11.11  add acc for HRV
#define acc_var_size 	0 //2
int meanacc_cnt = 0;
static int *acc_mean_buf = NULL;
int acc_mean_tail[MAX_RR_Cnt] = {0};

//23.8.7  add offdac
typedef struct {
	uint8_t current_flg;
	uint8_t offdac_flg;
	uint8_t flag;
}DC_ADJUST;
DC_ADJUST dc_adj;
#define offdc_deta 		4.15e5//2.08e5//8.3e5//218450//(436910*0.8)
uint32_t offdac_led2 = 0;
uint32_t offdac_led3 = 0;
uint32_t offdac_led1 = 0;
uint8_t offdc_gain_flg = 0;

//21.11.29
#define max_bufproc_num      250
uint8_t count_spo2_bufproc = 0;

//22.1.21   add HRV time stamp
uint32_t HRV_timestamp = 0;

//22.5.19  add the number of spo2 calculation
uint32_t num_calculation = 0;
int rr_startpos = 0;
int rr_endpos = 0;
int RRvar_cnt = 0;

//22.12.23 add breath wavelet
#define MAXRESAMPLENUM			1000
#define BREATHRATE_PROCTIME  	10//10s
#define breath_recover (MAXRESAMPLENUM - 10*BREATHRATE_PROCTIME)
#define MAX_BREATHRATE_BUFSIZE	6
int *BREATH_BUF = NULL;
wave_object obj_br = NULL;
wt_object wt_br = NULL;
FIFO32 BREATH_fifo;
uint8_t breathratehist = 0;
int tmpbr_datano = 0;
uint8_t breath_flg = 0;
uint8_t BR_vect[MAX_BREATH_CNT] = {0};
int BR_cnt = 0;
uint32_t BR_timestamp = 0;//BR time stamp
uint8_t breathrate_buf[MAX_BREATHRATE_BUFSIZE] = {0};
int breathrate_cnt = 0;

//23.7.17
static int *daily_green = NULL;
FIFO32 daily_green_fifo;
/*******************************************************************************
 * private function declaration
 */
static uint8_t	currentReg(int addval);
static void 	afe_InitReg_spo2(void);
static void 	afe_partinit(void);
static void 	afeproc_init(void);
static void 	afe_AddRawdata_wavelet(int spo2rawdata1, int spo2rawdata2);
static void 	ble_send_persecond(void);
//static unsigned char afe_CheckSum(uint8_t byte[], int len);
static uint8_t	data_compress(float src_data);

static void SPO2_Init_ProcPara(ProcPara *Para, int procsamplerate);
static int get_tiredFlg(int Proccnt);

static void BREATH_Proc_shedule(int *indata, int inlen, unsigned char *breathrate);

void Init_para()
{
	HistRateCnt = 0;
	HistHrsavepos = 0;
	HistHrVectId = 0;
	HistHRVId = 0;
	HistRRId = 0;
	HisVarCnt = 0;
	//HistShowCnt = 0;
    Hist_Hr_Rate = 0;
#if Tired_Proc
    SPO2_Init_ProcPara(&SPO2ProcPara, AFE_SAMPLERATE);
#endif
}

float coefa[5] = { 1,-3.76341591984671,5.36261433245638,-3.43315515452697,0.834045845369074}; 
float coefb[5] = { 0.00917775443917120,0,-0.0183555088783424,0,0.00917775443917120};
float spo2_filter_x1[5] = { 0,0,0,0,0 };
float spo2_filter_y1[5] = { 0,0,0,0,0 };
float spo2_filter_x2[5] = { 0,0,0,0,0 };
float spo2_filter_y2[5] = { 0,0,0,0,0 };

float spo2_iirbpfilter(float input, float *x1, float *y1)
{
    x1[0] = input;
    y1[0] = (x1[0])*coefb[0] + (x1[1])*coefb[1] + (x1[2])*coefb[2] + (x1[3])*coefb[3] + (x1[4])*coefb[4] - (y1[1]) *coefa[1] - (y1[2])*coefa[2] - (y1[3])*coefa[3] - (y1[4])*coefa[4];
    x1[4] = x1[3];
    x1[3] = x1[2];
    x1[2] = x1[1];
    x1[1] = x1[0];

    y1[4] = y1[3];
    y1[3] = y1[2];
    y1[2] = y1[1];
    y1[1] = y1[0];
    return y1[0];
}

static uint8_t currentReg(int addval)
{
    uint32_t regdata22 = 0;
    get_afe_reg(0x22,&regdata22);
    int redreg;
    int infrareg;
    int ambient;
    int greenlow;
    regdata22 += addval;
    greenlow = regdata22 >> 18;
    redreg = (regdata22 - (greenlow << 18)) >> 12;
    infrareg = (regdata22 - (greenlow << 18) - (redreg << 12)) >> 6;
    ambient = regdata22 - (greenlow << 18) - (redreg << 12) - (infrareg << 6);

    //if(infrareg > 15 || redreg > 15)
    //{
    //    infrareg = 15;
    //    redreg = 15;
    //}
    infrareg = min(infrareg, 12);
    redreg = min(redreg, 12);
    //NRF_LOG_INFO("redreg = %d, infrareg = %d\r\n",redreg,infrareg);
    regdata22 = (greenlow << 18) + (redreg << 12) + (infrareg << 6) + ambient;
    if(redreg < 0 || infrareg < 0)
    {
            regdata22 -= addval;
            return 1;
    }
    if(redreg <= 60 && infrareg <= 60)
    {
            set_afe_reg(0x22, regdata22);				
            return 0;
    }
    return 1;
}
extern FIFO32 green_health_fifo;
extern int health_green[HRV_PROCESS_DATA_PERIOD];
static void afe_partinit(void)
{
//wavelet init;
	fifo32_init(&spo2_red_fifo,length_rawdata, spo2_red);
	fifo32_init(&spo2_infra_fifo, length_rawdata, spo2_infra);
	fifo32_init(&spo2_green_fifo, length_rawdata, spo2_green);
    fifo32_init(&spo2_red_filter, length_wavelet, red_filter);
    fifo32_init(&spo2_infra_filter, length_wavelet, infra_filter);
	fifo32_init(&green_health_fifo, HRV_PROCESS_DATA_PERIOD, health_green);
//offwrist init;
	offwrist_init();
//afeproc init
	afeproc_init();
//	memset((uint8_t*)&red_detec, 0, sizeof(red_detec));
//tired
    Init_para();
}

void Accproc_init(void)
{
	accval = 0;
	Moveflg = 0;
	memset((uint8_t*)&accdata, 0, sizeof(accdata));
	summotionstate = 0;
	//22.03.11
	acc_flag = 0;
	acc_duration = 0;
}

static void afeproc_init(void)
{
    AlgPara.Proccnt = 0;
    AlgPara.heartratehist = 68;
    AlgPara.Spo2 = 98;
    AlgPara.heartrate = 68;
    AlgPara.Sheartrate = 68;
    AlgPara.tiredflg = 0;
    LED1readyfor_calc = false;
    LED2readyfor_calc = false;

    g_afe.spo2 = 244;
    g_afe.hr = 68;
    g_afe.flag = SPO2_HR_READYING;
    g_afe.Spo2show = 244;
    g_afe.shr = 68;
    g_afe.spo2_error = 0;
    g_afe.sleep_status = INVALID_STAGE;
    g_afe.tiredflg = 0;
    g_afe.offhand_flg = 0;

    startchange = 0;
    changecnt = 0;
    cuaddval = 0;
    meandatanum = 0;
    redlowernum = 0;
    cuinitflag = 0;
    cuchangeflag = 0;
    initdonelength = 0;
    // infrathreslow = 3.5e5;
    infrathreslow = 5e5;
    infrathreshigh = 1.8e6;

    length_accval = 13;
    HRV_cnt = 2;
    // HRV_cnt = 0;
    HRV_timestamp = 0;
    totalHrCnt = 0;

    HistRateCnt = 0;
    HistHrsavepos = 0;

    count_spo2_bufproc = 0;
    rr_startpos = 0;
    rr_endpos = 0;
    RRvar_cnt = 0;

    offdac_led2 = 0;
    offdac_led3 = 0;
    offdac_led1 = 0;
    offdc_gain_flg = 0;
    dc_adj.current_flg = 0;
    dc_adj.offdac_flg = 0;
    dc_adj.flag = 0;
	g_rr_mean = 0;
    Accproc_init();
}

bool afe_breath_init(void)
{
	if(BREATH_BUF == NULL)
		return false;
	
	int N = 1000;
	int J = 6;//WAVELET_LEVEL
	char *name = "db4";
	obj_br = wave_init(name);	//peak: 218+32
	wt_br = wt_init(obj_br, "dwt", N, J);// Initialize the wavelet transform object, peak:0x10AC	
	setDWTExtension(wt_br, "sym");// Options are "per" and "sym". Symmetric is the default option
	setWTConv(wt_br, "direct");

	breathratehist = 0;
	fifo32_init(&BREATH_fifo,MAXRESAMPLENUM, BREATH_BUF);
	tmpbr_datano = 0;
	breath_flg = 0;
	BR_cnt = 0;
	BR_timestamp = 0;
	breathrate_cnt = 0;
	
	return true;
}

bool afe_spo2_init(void)
{
#ifdef FIXED_CURRENT
  reg22val = 0x000000|(12 << 12)|(12 << 6)|12;
	
#else
  if (RING_AFE_HRV_ON_MODE == (RING_AFE_HRV_ON_MODE & afe_get_work_mode())) {
    reg22val = 0x000000|(10 << 12)|(10 << 6) | (3 << 0);
  } else {
    reg22val = 0x000000|(10 << 12)|(10 << 6);
  }
#endif
	set_afe_reg(0x22, reg22val);

//test offdac
    //offdac_led2 = 10;
    //offdac_led3 = 10;
    //off_dac_set_circle();
    //dc_adj.offdac_flg = 1;

    afe_wave_free();
    if(spo2_red == NULL || spo2_infra == NULL || spo2_green == NULL 
        || wrcoefbuf == NULL || AccData == NULL || HistSpo2 == NULL || HistAcc == NULL || Acc_Z_Buf == NULL
        || red_filter == NULL || infra_filter == NULL)
        return false;
    
	afe_partinit();
	//wt init;
	int N = length_wavelet - 2*MARLEN;
	int J = WAVELET_LEVEL;
	char *name = "db4";
	obj = wave_init(name);	//peak: 218+32
	wt = wt_init(obj, "dwt", N, J);// Initialize the wavelet transform object, peak:0x10AC	
	setDWTExtension(wt, "sym");// Options are "per" and "sym". Symmetric is the default option
	setWTConv(wt, "direct");

    SPO2_Init_ProcPara(&SPO2ProcPara, AFE_SAMPLERATE);
	num_calculation = 0;
	
	bool breath = afe_breath_init();
	if(breath == false)
		return false;
	
	return true;
}

/*********************************************************************
* @fn      afe_AddRawdata_wavelet()
*
* @brief   add data to fifo buffer.
*
* @param   raw data:AD data
* @return  none.
*/
static void afe_AddRawdata_wavelet(int spo2rawdata1, int spo2rawdata2)
{
	fifo32_putPut(&spo2_red_fifo, spo2rawdata1);
	fifo32_putPut(&spo2_infra_fifo, spo2rawdata2);
}

/*********************************************************************
* @fn      data_compress();
*
* @brief   compress.
*    float to unsigned char.
* @param   float value
* @return  unsigned char
*/
static uint8_t data_compress(float src_data)
{
	int a = (int)src_data;
	int b = (int)((src_data - a) * 10);

	uint8_t integer = 0;
	uint8_t noninteger = 0;
	//****37-100***
	if (a < 37) 
	{
		integer = a;
        accval = accval | 0x40;//  set bit6:1
	}
	else
	{
		if (a > 99) 
		{
			a = 99;
		}
		integer = a - 37;
        accval = accval & 0xBF;
	}

	uint8_t map[] = { 0,0,0,1,1,2,2,3,3,3 };
	noninteger = map[b];

	uint8_t dst_data = (integer << 2) + noninteger;

	return dst_data;
	//uint8_t spo2 = (g_afe.spo2 >> 2) + 37;
}

int get_peakval(int* input, int inlen)
{
	int maxval = input[0];
	int minval = input[0];
	for (int i = 1; i < inlen; i++)
	{
		if (maxval < input[i])
		{
			maxval = input[i];
		}
		if (minval > input[i])
		{
			minval = input[i];
		}
	}
	return maxval - minval;
}

void get_RR(int *input, int inlen, uint8_t heartrate, uint8_t *HRCnt, uint8_t *HRVect)
{
	*HRCnt = 0;
	float Maxscale = 1.2;
	float Minscale = 0.8;


	if (heartrate > 240 || heartrate < 30)
	{
		if (*HRCnt == 0)
		{
			HRVect[*HRCnt] = 253;
			(*HRCnt)++;
		}
		return;
		//printf("debug err\n");
	}

	float RRmean = 6000 / heartrate;

	int tcnt = 0;
	for (int i = 0; i < inlen - 1; i++)
	{
		if (input[i] <= 0 && input[i + 1] >= 0)
		{
			zerosVect[tcnt++] = i;
		}
		if (tcnt >= MAX_ZEROS_NUM)
		{
			break;
		}
	}

	int ttcnt = 2;
	for (int i = 1; i < tcnt - 2; i++)
	{
		if (zerosVect[i + 1] - zerosVect[i] < 15)
		{
			if (zerosVect[ttcnt - 1] - zerosVect[ttcnt - 2] <= zerosVect[i + 2] - zerosVect[i + 1])
			{
				zerosVect[ttcnt - 1] = zerosVect[i + 1];
			}
			else
			{
				zerosVect[ttcnt] = zerosVect[i + 2];
				ttcnt++;
				i++;
			}
		}
		else
		{
			zerosVect[ttcnt] = zerosVect[i + 1];
			ttcnt++;
		}
	}

	if (zerosVect[tcnt - 1] - zerosVect[tcnt - 2] > 20 && zerosVect[tcnt - 1] - zerosVect[tcnt - 2] < 200)
	{
		zerosVect[ttcnt] = zerosVect[tcnt - 1];
		ttcnt++;
	}

	tcnt = ttcnt;

	int i = 0;
	while (i < tcnt)
	{
		if (zerosVect[i] >= 400)
		{
			break;
		}
		i++;
	}

	//if (i >= tcnt - 1 || i <= 1)
	//{
	//	if (*HRCnt == 0)
	//	{
	//		HRVect[*HRCnt] = 254;
	//		(*HRCnt)++;
	//	}
	//	return;
	//}

	while (i < tcnt)
	{
		if (zerosVect[i] >= 600 || *HRCnt >= MAX_RR_Cnt)
		{
			break;
		}

		int tp1 = zerosVect[i - 1];
		int tmpval = 0;
		for (int j = zerosVect[i - 1]; j < zerosVect[i]; j++)
		{
			if (input[j] >= tmpval)
			{
				tmpval = input[j];
				tp1 = j;
			}
		}

		int tp2 = zerosVect[i];
		tmpval = 0;
		for (int j = zerosVect[i]; j < zerosVect[i+1]; j++)
		{
			if (input[j] >= tmpval)
			{
				tmpval = input[j];
				tp2 = j;
			}
		}

		int rrtmp = tp2 - tp1;
		if (rrtmp < 256)
		{
			if (rrtmp > RRmean * Minscale && rrtmp < RRmean * Maxscale)
			{
				HRVect[*HRCnt] = rrtmp;
				(*HRCnt)++;
			}
			else
			{
				if (i > 1)
				{
					rrtmp = zerosVect[i] - zerosVect[i - 2];
					if (rrtmp > RRmean * Minscale && rrtmp < RRmean * Maxscale)
					{
						HRVect[*HRCnt] = rrtmp;
						(*HRCnt)++;
					}
				}
				if (*HRCnt == 0)
				{
					if (i < tcnt - 1)
					{
						rrtmp = zerosVect[i + 1] - zerosVect[i - 1];
						if (rrtmp > RRmean * Minscale && rrtmp < RRmean * Maxscale)
						{
							HRVect[*HRCnt] = rrtmp;
							(*HRCnt)++;
						}
					}
				}
			}

		}
		i++;
	}

	if (*HRCnt == 0)
	{
		HRVect[*HRCnt] = 255;
		(*HRCnt)++;
	}
	return;
}

int get_spo2_mean(int* input, int inlen)
{
	if (inlen <= 0)
	{
		return 0;
	}
	int sum = 0;
	for (int i = 0; i < inlen; i++)
	{
		sum += input[i];
	}
	return sum / inlen;
}

int get_ZerosVectvar(int *input, int inlen)
{
	if (inlen < 3)
	{
		return 0;
	}
	
	int tmean = 0;
	int tvldcnt = 0;

	for (int i = 0; i < inlen - 5; i += 2)
	{
		tmean += abs((input[i+4]-input[i+2]) - (input[i+2] - input[i]));
		tvldcnt++;
	}
	if (tvldcnt == 0)
	{
		return 0;
	}
	return tmean / tvldcnt;
}

#if 1


void PPG_data_RR_peaks_get_heart(float* PPGData, int DataLen, uint16_t* RRData, int* RRLen)
{
    if(PPGData == NULL || DataLen <= 0)
        return;

    uint16_t PPG_mean_pos[HRV_PROCESS_DATA_PERIOD / 2] = { 0 };
    uint16_t mean_pos_cnt = 0;
    float meanValue = 0;
    float meanValue_two = 0;
    uint16_t meanValue_two_cnt = 0;
    for (int num = 0; num < DataLen;num++)
    {
        meanValue = meanValue + PPGData[num];
    }
    meanValue = meanValue / DataLen;
    //meanValue = meanValue - 100;
    //meanValue = meanValue - 200;

    for (int num = 0; num < DataLen; num++)
    {
        //if (PPGData[num] < meanValue)
		if (PPGData[num] < meanValue && PPGData[num] > -1*PPG_ACC_MAX_VALUE)
        {
            meanValue_two = meanValue_two + PPGData[num];
            meanValue_two_cnt = meanValue_two_cnt + 1;
        }
       
    }
	if(meanValue_two_cnt != 0 )
	{
		meanValue_two = meanValue_two / meanValue_two_cnt;
	}


    //printf("====meanValue_two = %f\n", meanValue_two);
    for (int num = 0;num < DataLen - 1;num++)
    {
		if(mean_pos_cnt < HRV_PROCESS_DATA_PERIOD / 2)
		{
			if (PPGData[num] >= meanValue_two && PPGData[num + 1] < meanValue_two)
			{
				PPG_mean_pos[mean_pos_cnt] = num;
				mean_pos_cnt++;
			}
			else if (PPGData[num] < meanValue_two && PPGData[num + 1] >= meanValue_two)
			{
				PPG_mean_pos[mean_pos_cnt] = num + 1;
				mean_pos_cnt++;
			}
		}

    }

    float min_value = 10000;
    uint16_t min_value_pos = 0;
    for (int num = 0;num < mean_pos_cnt;num++)
    {
        min_value = 10000;
        min_value_pos = 0;
        for (int pos_num = PPG_mean_pos[num]; pos_num < PPG_mean_pos[num + 1]; pos_num++)
        {
            if (min_value > PPGData[pos_num])
            {
                min_value = PPGData[pos_num];
                min_value_pos = pos_num;
            }
        }

        if (min_value < meanValue_two)
        {
            RRData[*RRLen] = min_value_pos;
            *RRLen = *RRLen + 1;
        }

    }



}



float PPG_data_max_value(float* PPGData,int dataLen)
{
    float max_value = -10000;
    for (int num = 0;num < dataLen;num++)
    {
        if (max_value < PPGData[num])
        {
            max_value = PPGData[num];
        }
    }
    return max_value;
}

float PPG_data_min_value(float* PPGData, int dataLen)
{
    float min_value = 10000;
    for (int num = 0; num < dataLen; num++)
    {
        if (min_value > PPGData[num])
        {
            min_value = PPGData[num];
        }
    }
    return min_value;
}


int PPG_data_max_value_int(int* PPGData,int dataLen)
{
    int max_value = -10000;
    for (int num = 0;num < dataLen;num++)
    {
        if (max_value < PPGData[num])
        {
            max_value = PPGData[num];
        }
    }
    return max_value;
}

int PPG_data_min_value_int(int* PPGData, int dataLen)
{
    int min_value = 10000;
    for (int num = 0; num < dataLen; num++)
    {
        if (min_value > PPGData[num])
        {
            min_value = PPGData[num];
        }
    }
    return min_value;
}


uint16_t PPG_mean_pos[HRV_PROCESS_DATA_PERIOD / 2] = { 0 };
void PPG_data_RR_peaks_get_heart_2(int* PPGData, int DataLen, uint16_t* RRData, int* RRLen,int*upPeakValueMean)
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
	memset(PPG_mean_pos,0,(HRV_PROCESS_DATA_PERIOD / 2)*sizeof(uint16_t));
    // vaild ppt data
    for (int num = 0;num < DataLen - 1;num++)
    {
        if (PPGData[num] < 0 && PPGData[num + 1] >= 0)
        {
            PPG_mean_pos[mean_pos_cnt] = num;//zeros point
            mean_pos_cnt++;
        }
        else if (PPGData[num] >= 0 && PPGData[num + 1] < 0)
        {
            PPG_mean_pos[mean_pos_cnt] = num + 1;//zeros point
            mean_pos_cnt++;
        }

    }

    for (int num = 0;num < mean_pos_cnt - 1;num++)
    {
        // maxValue = PPG_data_max_value_int(&PPGData[PPG_mean_pos[num]], PPG_mean_pos[num] - PPG_mean_pos[num - 1] - 1);
        // minValue = PPG_data_min_value_int(&PPGData[PPG_mean_pos[num]], PPG_mean_pos[num] - PPG_mean_pos[num - 1] - 1);
        maxValue = PPG_data_max_value_int(&PPGData[PPG_mean_pos[num]], PPG_mean_pos[num + 1] - PPG_mean_pos[num]);
        minValue = PPG_data_min_value_int(&PPGData[PPG_mean_pos[num]], PPG_mean_pos[num + 1] - PPG_mean_pos[num]);
        if (maxValue < 20000 && maxValue > 0)//finger move data delete
        {
            upPeakMaxValueMean = upPeakMaxValueMean + maxValue;
            upPeakCnt = upPeakCnt + 1;
        }

        // if (minValue < -2000 && minValue > -20000)//finger move data delete
        // if (minValue < -200 && minValue > -20000)//finger move data delete
        //if (minValue < -1000 && minValue > -20000)//finger move data delete
        if (minValue < -300 && minValue > -20000)//finger move data delete
        {
            downPeakMaxValueMean = downPeakMaxValueMean + minValue;
            downPeakCnt = downPeakCnt + 1;
        }

    }
    if (upPeakCnt != 0)
    {
        upPeakMaxValueMean = upPeakMaxValueMean / upPeakCnt;
    }
    if (downPeakCnt != 0)
    {
        downPeakMaxValueMean = downPeakMaxValueMean / downPeakCnt;
    }

    *upPeakValueMean = upPeakMaxValueMean;

    peaceDataCnt = 0;
    for (int num = 0; num < DataLen;num++)
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

    memset(PPG_mean_pos,0,(HRV_PROCESS_DATA_PERIOD / 2)*sizeof(uint16_t));
    for (int num = 0;num < DataLen - 1;num++)
    {
        if (PPGData[num] >= meanValue_two && PPGData[num + 1] < meanValue_two)
        {
            PPG_mean_pos[mean_pos_cnt] = num;
            mean_pos_cnt++;
        }
        else if (PPGData[num] < meanValue_two && PPGData[num + 1] >= meanValue_two)
        {
            PPG_mean_pos[mean_pos_cnt] = num + 1;
            mean_pos_cnt++;
        }
    }

    int min_value = 10000;
    uint16_t min_value_pos = 0;
    for (int num = 0;num < mean_pos_cnt - 1;num++)
    {
        min_value = 10000;
        min_value_pos = 0;
        for (int pos_num = PPG_mean_pos[num]; pos_num < PPG_mean_pos[num + 1]; pos_num++)
        {
            if (min_value > PPGData[pos_num])
            {
                min_value = PPGData[pos_num];
                min_value_pos = pos_num;
            }
        }

        if (min_value < meanValue_two)
        {
            RRData[*RRLen] = min_value_pos;
            *RRLen = *RRLen + 1;
        }

    }

}




void PPG_data_RR_get_heart(uint16_t* RRData, int RRLen, HRV_Para* hrv_health_para_out)
{
    if(RRData == NULL || RRLen <= 0)
        return;

    int pre_flag = 0;
    int pre_rr_end_pos = 0;
    pre_rr_end_pos = hrv_health_para_out->RR_num;
    for (int num = 0;num < RRLen - 1;num++)
    {
        if (pre_flag == 1)
        {
            pre_flag = 0;
            continue;
        }
        if (RRData[num + 1] - RRData[num] < 27)//27:max hr (100/27)*60
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num + 1];
            hrv_health_para_out->RR_num++;
            pre_flag = 1;
        }
        else
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num];
            hrv_health_para_out->RR_num++;
        }
  
    }
    if(RRLen > 2)
    {
        if(RRData[RRLen - 1] - RRData[RRLen - 2] >= 27)//27:max hr (100/27)*60
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[RRLen - 1];
            hrv_health_para_out->RR_num++;
        }
    }


    for (int num = pre_rr_end_pos;num < hrv_health_para_out->RR_num - 1;num++)
    {
        hrv_health_para_out->green_RR[num] = (hrv_health_para_out->green_RR[num + 1] - hrv_health_para_out->green_RR[num]);
    }
    //hrv_health_para_out->green_RR[hrv_health_para_out->RR_num - 1] = (hrv_health_para_out->green_RR[hrv_health_para_out->RR_num - 2] + hrv_health_para_out->green_RR[hrv_health_para_out->RR_num - 3]) / 2;
}


void PPG_data_RR_get_heart_2(uint16_t* RRData, int RRLen, HRV_Para* hrv_health_para_out, int* PPGData,int dataLen,int upPeakValueMean)
{
    int pre_flag = 0;
    int pre_rr_end_pos = 0;
    pre_rr_end_pos = hrv_health_para_out->RR_num;
    for (int num = 0;num < RRLen - 1;num++)
    {
        if (pre_flag == 1)
        {
            pre_flag = 0;
            continue;
        }
        //if (RRData[num + 1] - RRData[num] < 30)
        if (RRData[num + 1] - RRData[num] < 27)
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num + 1];
            hrv_health_para_out->RR_num++;
            pre_flag = 1;
        }
        else
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num];
            hrv_health_para_out->RR_num++;
        }
  
    }

    if (RRLen > 2)
    {
        //if(RRData[RRLen - 1] - RRData[RRLen - 2] > 30)
        if (RRData[RRLen - 1] - RRData[RRLen - 2] >= 27) //27 : max hr = (1000/270)*60 = 222
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[RRLen - 1];
            hrv_health_para_out->RR_num++;
        }
    }
    int maxValue = 0;
    uint16_t RRNum = 0;
    for (int num = pre_rr_end_pos;num < hrv_health_para_out->RR_num - 1;)
    {
        maxValue = PPG_data_max_value_int(&PPGData[hrv_health_para_out->green_RR[num]], hrv_health_para_out->green_RR[num + 1] - hrv_health_para_out->green_RR[num]);
        if (maxValue > 0 && maxValue - upPeakValueMean < 10000)
        {

            hrv_health_para_out->green_RR[RRNum] = (hrv_health_para_out->green_RR[num + 1] - hrv_health_para_out->green_RR[num]);
            RRNum++;
            num++;

        }
        else
        {
            num = num + 2;
        }
        
    }
    hrv_health_para_out->RR_num = RRNum;
}


void PPG_data_RR_get_heart_point_2(uint16_t* RRData, int RRLen, HRV_Para* hrv_health_para_out, int* PPGData,int dataLen,int upPeakValueMean)
{
    int pre_flag = 0;
    int pre_rr_end_pos = 0;
    pre_rr_end_pos = hrv_health_para_out->RR_num;
    if(pre_rr_end_pos+RRLen >= HRV_MAX_DATA_LEN)
        return;

    for (int num = 0;num < RRLen - 1;num++)
    {
        if (pre_flag == 1)
        {
            pre_flag = 0;
            continue;
        }
        //if (RRData[num + 1] - RRData[num] < 30)
        if (RRData[num + 1] - RRData[num] < 27)
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num + 1];
            hrv_health_para_out->RR_num++;
            pre_flag = 1;
        }
        else
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num];
            hrv_health_para_out->RR_num++;
        }
  
    }

    if (RRLen > 2)
    {
        //if(RRData[RRLen - 1] - RRData[RRLen - 2] > 30)
        if (RRData[RRLen - 1] - RRData[RRLen - 2] >= 27) //27 : max hr = (1000/270)*60 = 222
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[RRLen - 1];
            hrv_health_para_out->RR_num++;
        }
    }
    float maxValue = 0;
    // uint16_t RRNum = 0;
    for (int num = pre_rr_end_pos;num < hrv_health_para_out->RR_num - 1;)
    {
        maxValue = PPG_data_max_value_int(&PPGData[hrv_health_para_out->green_RR[num]], hrv_health_para_out->green_RR[num + 1] - hrv_health_para_out->green_RR[num]);
        if (maxValue > 0 && maxValue - upPeakValueMean < 10000 && pre_rr_end_pos < HRV_MAX_DATA_LEN)
        {

            hrv_health_para_out->green_RR[pre_rr_end_pos] = (hrv_health_para_out->green_RR[num + 1] - hrv_health_para_out->green_RR[num]);
            pre_rr_end_pos++;
            num++;

        }
        else
        {
            num = num + 2;
        }
        
    }
    hrv_health_para_out->RR_num = pre_rr_end_pos;
}








//"get_hr_process" add by xuan
extern int green_smooth[HRV_PROCESS_DATA_PERIOD];
extern uint16_t green_RR_peaks[HRV_MAX_PEAKS_DATA_LEN];
extern int greenRRLen;
extern HRV_Para hrv_health_para;
uint8_t ACC_time_cnt = 0;
uint16_t g_hr_rr_invalid_cnt = 0;
void get_hr_process(int *input, int inlen, int Tcc, uint8_t heartratehist, uint8_t *heartrate)
{
	//NRF_LOG_INFO("22222222===========Tcc = %d\n",Tcc);
	// heart no updata 3s when bodymove
	int upPeakValueMean = 0;
	uint8_t ACC_value = 0;
	greenRRLen = 0;
	memset(green_smooth,0,HRV_PROCESS_DATA_PERIOD*sizeof(int));
	memset(green_RR_peaks,0,HRV_MAX_PEAKS_DATA_LEN*sizeof(uint16_t));
	ACC_value = data_compress16(Tcc,Moveflg,0);
	if(ACC_time_cnt > 0)
	{
		ACC_time_cnt = ACC_time_cnt - 1;
	}
	if(ACC_value > 3)
	{
		*heartrate = heartratehist;
		ACC_time_cnt = 5;
		return;
	}
	else if(ACC_time_cnt > 0)
	{
		*heartrate = heartratehist + pow(-1,ACC_time_cnt)*(rand()%2);
		//NRF_LOG_INFO("22222222===========*heartrate = %d\n",*heartrate);
		return;
	}
	smooth_data_int_out_int(input, HRV_SMOOTH_SPAN, HRV_PROCESS_DATA_PERIOD, green_smooth);

    //PPG_data_wavelet(green_smooth, HRV_PROCESS_DATA_PERIOD, green_smooth);
	// PPG_data_RR_peaks_get_heart(green_smooth, HRV_PROCESS_DATA_PERIOD, green_RR_peaks, &greenRRLen);
	PPG_data_RR_peaks_get_heart_2(green_smooth, HRV_PROCESS_DATA_PERIOD, green_RR_peaks, &greenRRLen,&upPeakValueMean);
    //PPG_data_RR_peaks_get(green_smooth, HRV_PROCESS_DATA_PERIOD, green_RR_peaks, &greenRRLen);
	memset(&hrv_health_para,0,sizeof(HRV_Para));
    // PPG_data_RR_get_heart(green_RR_peaks, greenRRLen, &hrv_health_para);
	PPG_data_RR_get_heart_2(green_RR_peaks, greenRRLen, &hrv_health_para, green_smooth,HRV_PROCESS_DATA_PERIOD,upPeakValueMean);

	// if(hrv_health_para.RR_num < 7)
	// {
	// 	// *heartrate = heartratehist;
	// 	*heartrate = heartratehist + pow(-1,ACC_time_cnt);
	// 	return;
	// }

	uint16_t rr_mean = 0;
    uint16_t rr_mean_cnt = 0;
    uint16_t RRMaxValue = 0;
    uint16_t RRMinValue = 0;
    uint16_t RRMaxValuePos = 0;
    uint16_t RRMinValuePos = 0;
    uint16_t rr_vaild_cnt = 0;
    if (hrv_health_para.RR_num > 1)
    {
        RRMaxValue = hrv_health_para.green_RR[0];
        RRMinValue = hrv_health_para.green_RR[0];
        RRMaxValuePos = 0;
        RRMinValuePos = 0;
    }

    for (int num = 0; num < hrv_health_para.RR_num - 1; num++)
    {
        if (RRMaxValue < hrv_health_para.green_RR[num])
        {
            RRMaxValue = hrv_health_para.green_RR[num];
            RRMaxValuePos = num;
        }

        if (RRMinValue > hrv_health_para.green_RR[num])
        {
            RRMinValue = hrv_health_para.green_RR[num];
            RRMinValuePos = num;
        }

    }


#if 1
    //printf("g_rr_mean = %d\n", g_rr_mean);
    for (int num = 0; num < hrv_health_para.RR_num - 1; num++)
    {
        if (g_rr_mean != 0)
        {
            if(g_hr_rr_invalid_cnt < HR_INVALID_RR_CNT_MAX)
            {
                if (num != RRMaxValuePos && num != RRMinValuePos && abs(g_rr_mean - hrv_health_para.green_RR[num]) <= HEART_RR_CHANGE_MAX_VALUE && hrv_health_para.green_RR[num] > 30 && hrv_health_para.green_RR[num] < 200)
                {
                    rr_mean = rr_mean + hrv_health_para.green_RR[num];
                    rr_mean_cnt = rr_mean_cnt + 1;
                    //printf("rr = %d\n", hrv_health_para.green_RR[num]);
                }

            }
            else{
                if (num != RRMaxValuePos && num != RRMinValuePos && hrv_health_para.green_RR[num] > 30 && hrv_health_para.green_RR[num] < 200)
                {
                    rr_mean = rr_mean + hrv_health_para.green_RR[num];
                    rr_mean_cnt = rr_mean_cnt + 1;
                    //printf("rr = %d\n", hrv_health_para.green_RR[num]);
                }
            }

            if(num != RRMaxValuePos && num != RRMinValuePos && hrv_health_para.green_RR[num] > 30 && hrv_health_para.green_RR[num] < 200)
            {
                rr_vaild_cnt = rr_vaild_cnt + 1;
            }
        }
        else
        {
            if (num != RRMaxValuePos && num != RRMinValuePos && hrv_health_para.green_RR[num] > 30 && hrv_health_para.green_RR[num] < 200)
            {
                rr_mean = rr_mean + hrv_health_para.green_RR[num];
                rr_mean_cnt = rr_mean_cnt + 1;
                //printf("rr = %d\n", hrv_health_para.green_RR[num]);
            }
        }


    }
#endif
#if 0
    for (int num = 0; num < hrv_health_para.RR_num - 1; num++)
    {
        if (num != RRMaxValuePos && num != RRMinValuePos)
        {
            rr_mean = rr_mean + hrv_health_para.green_RR[num];
            rr_mean_cnt = rr_mean_cnt + 1;
        }

    }
#endif
    if (rr_mean_cnt > 0 )
    //if (rr_mean_cnt > 5 )
    {
        rr_mean = rr_mean / rr_mean_cnt;
        if(rr_vaild_cnt > 5)//PPG signal is good
        {
            g_rr_mean = rr_mean;
        }
		g_hr_rr_invalid_cnt = 0;
    }
    else
    {
        if(g_hr_rr_invalid_cnt < HR_INVALID_RR_CNT_MAX)
        {
            g_hr_rr_invalid_cnt = g_hr_rr_invalid_cnt + 1;
        }
        
    }
    if (rr_mean != 0)
    {
        *heartrate = (uint8_t)(1000 * 60 / (rr_mean * 10));
    }
	else{
        *heartrate = heartratehist;
    }
	//heartratehist = *heartrate;

	memset(green_smooth,0,HRV_PROCESS_DATA_PERIOD*sizeof(int));
	memset(green_RR_peaks,0,HRV_MAX_PEAKS_DATA_LEN*sizeof(uint16_t));
	greenRRLen = 0;
	memset(&hrv_health_para,0,sizeof(HRV_Para));
}
void get_hr(int *input, int inlen, int Tcc, uint8_t heartratehist, uint8_t *heartrate, uint8_t *VldFlg, uint8_t *HRCnt, uint8_t *HRVect)
{
    if(input == NULL || inlen <=0)
        return;

	int Lowthreshold = 150;
	int Highthreshold = 34;
	int threshold = 0;
	int tcnt = 0;
	int sum = 0;
	for (int i = 0; i < inlen; i++)
	{
		if ((input[i] <= threshold && input[i + 1] > threshold) || (input[i] >= threshold && input[i + 1] < threshold))
		{
			zerosVect[tcnt++] = i;
		}
		if (tcnt >= MAX_ZEROS_NUM)
		{
			break;
		}
	}
	if (tcnt <= 7)
	{
		*VldFlg = 0;
		*heartrate = heartratehist;
		*HRCnt = 0;
		return;
	}

	int ZerosVar1 = get_ZerosVectvar(zerosVect, tcnt);//周期差和均值
	int ZerosVar2 = get_ZerosVectvar(zerosVect+1, tcnt-1);
	int loopcnt = 0;
	do
	{
		loopcnt++;
		AlgData_t tmpmean = 0;
		int tmeancnt = 0;
		int tmean1 = inlen / (tcnt - 1);
		for (int i = 0; i < tcnt - 1; i++)
		{
			tmpmean += zerosVect[i + 1] - zerosVect[i];
			tmeancnt++;
		}
		if (tmeancnt > 0)
		{
			tmpmean = tmpmean / tmeancnt;
		}
		else
		{
			*VldFlg = 0;
			*heartrate = heartratehist;
			*HRCnt = 0;
			return;
		}
		int tthreshold = MIN(MAX(tmpmean*0.6, 6),15);
		int ttcnt = 2;
		int tsp = 0;
		while (tsp < tcnt - 2 && zerosVect[tsp + 1] - zerosVect[tsp] < tthreshold)
		{
			zerosVect[0] = zerosVect[tsp+1];
			tsp++;
		}
		zerosVect[1] = zerosVect[tsp+1];
		for (int i = tsp + 1; i < tcnt - 2; i++)
		{
			if (zerosVect[i + 1] - zerosVect[i] < tthreshold)
			{
				if (zerosVect[i + 2] - zerosVect[i] < 2.2 * tthreshold)
				{
					zerosVect[ttcnt] = zerosVect[i + 3];
					ttcnt++;
					i += 2;
				}
				else
				{
					if (zerosVect[i + 2] - zerosVect[ttcnt - 1] < 2.2 * tthreshold)
					{
						//printf("debug");
					}
					else
					{
						zerosVect[ttcnt++] = zerosVect[i + 1];
					}
				}
			}
			else
			{
				zerosVect[ttcnt] = zerosVect[i + 1];
				ttcnt++;
			}
		}

		tcnt = ttcnt;
		int tZerosVar1 = get_ZerosVectvar(zerosVect, tcnt);
		int tZerosVar2 = get_ZerosVectvar(zerosVect+1, tcnt-1);
		if (tZerosVar1 == ZerosVar1 && tZerosVar2 == ZerosVar2)
		{
			break;
	}
		ZerosVar1 = tZerosVar1;
		ZerosVar2 = tZerosVar2;
	} while (loopcnt < 3 && abs(ZerosVar1 - ZerosVar2) > 15);

	int ttcnt = 0;
	if (ZerosVar1 < ZerosVar2)
	{
		for (int i = 0; i < tcnt / 2; i++)
		{
			zerosVect[ttcnt++] = zerosVect[i*2];
		}
	}
	else
	{

	
		for (int i = 0; i < (tcnt-1) / 2; i++)
		{
			zerosVect[ttcnt++] = zerosVect[i * 2 + 1];
		}
	}

	for (int i = 0; i < ttcnt - 1; i++)
	{
		zerosVect[i] = zerosVect[i + 1] - zerosVect[i];
	}

	tcnt = ttcnt - 1;

    int cnt = 0;
	for (int i = 0; i < tcnt; i++)
	{
		if (zerosVect[i] < Lowthreshold && zerosVect[i] > Highthreshold)
		{
			zerosVect[cnt++] = zerosVect[i];
		}
	}

	int tmpmean = get_spo2_mean(zerosVect, cnt);
	tcnt = cnt;
	cnt = 0;
	if (abs(zerosVect[0] - tmpmean) < MAX(tmpmean*0.1, 10))
	{
		cnt++;
	}
	for (int i = 1; i < tcnt - 1; i++)
	{
		if (zerosVect[i] >= Lowthreshold || zerosVect[i] <= Highthreshold)
		{
			cnt = MAX(0, cnt - 1);
		}
		else
		{
			if (zerosVect[i] > zerosVect[i - 1] * 0.7 && zerosVect[i] < zerosVect[i - 1] * 1.3 && zerosVect[i] > zerosVect[i + 1] * 0.7 && zerosVect[i] < zerosVect[i + 1] * 1.3)
			{
				zerosVect[cnt++] = zerosVect[i];
			}
		}
	}

	if (abs(zerosVect[tcnt - 1] - tmpmean) < MAX(tmpmean*0.1, 10))
	{
		zerosVect[cnt++] = zerosVect[tcnt - 1];
	}

	AlgData_t vldsum = 0;
	AlgData_t vldcnt = 0;
	for (int i = 0; i < cnt; i++)
	{
		vldsum += zerosVect[i];
		vldcnt += 1;
	}

	uint8_t tmphr = 0;
	if (vldcnt > 0 && vldsum > 0)
	{
		tmphr = (int)(6000 / (vldsum / vldcnt) + 0.4);
	}

	if (tmphr > 0)
	{
				
		*heartrate = tmphr;
	}
	else
	{
			*heartrate = heartratehist;
	}
	*VldFlg = vldcnt;

	get_RR(input, inlen, *heartrate, HRCnt, HRVect);
}
#endif




void smooth_data_int(int* frame_data, int smooth_span, int data_len, int* frame_data_smooth)
{

	int sum_num = 0;
	int add_num = 0;
	int add_data_num_half = 0;
	int64_t sum = 0;
	if (smooth_span % 2 == 1)
	{
		add_data_num_half = (smooth_span - 1) / 2;
	}
	else
	{
		add_data_num_half = (smooth_span - 2) / 2;
	}
    int smooth_data_len = 0;
	for (int frame_data_num = 0; frame_data_num < data_len; frame_data_num++)
    // for (int frame_data_num = RR_SPAN; frame_data_num < data_len; frame_data_num++)
	{

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
		for (sum_num = frame_data_num - add_num; sum_num <= frame_data_num + add_num; sum_num++)
		{
			sum += frame_data[sum_num];
		}
		frame_data_smooth[smooth_data_len] = (int)(sum / (add_num * 2 + 1));
        smooth_data_len++;

	}

}

int pre_data_len = 0;
int period_data[RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM + RR_PROCESS_DATA_OFFSET + RR_SPAN] = {0};
uint16_t threshold_end_pos[RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM] = {0};
uint16_t period_peak_pos[RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM] = {0};


int period_data_amp[RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM] = { 0 };
uint16_t period_amp_peak_value[RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM] = { 0 };
uint16_t period_amp_peak_pos[RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM] = { 0 };

uint16_t period_amp_diff_peak_pos[RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM] = { 0 };


void Sleep_PPG_data_RR_get(int* input,int dataLen, int* Acc, int Acclen, AlgProc_Para* AlgPara)
{
    int period_data_start_pos = 0;
    int threshold_end_cnt = 0;
    int period_peak_cnt = 0;
    float mean_value = 0;
    float mean_value_two = 0;
    int mean_value_cnt = 0;
    int min_value = 5000;
    float threshold_value = 0;
    int midDataLen = 0;
    uint16_t min_pos = 0;
    uint16_t RR_data_process_len = 0;
    AlgPara->HrCnt = 0;
    float left_keep_up_change_value = 0;
    float right_keep_up_change_value = 0;
    memset(period_data,0,(RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM)*sizeof(int));
    if (pre_data_len > PRE_RR_PROCESS_DATA_NUM)
    {
        pre_data_len = PRE_RR_PROCESS_DATA_NUM;
    }
    period_data_start_pos = dataLen - RR_PROCESS_DATA_NUM - pre_data_len - RR_SPAN;
    if(period_data_start_pos < 0)
    {
        period_data_start_pos = 0;
    }
    RR_data_process_len = RR_PROCESS_DATA_NUM + pre_data_len + RR_SPAN;
    smooth_data_int(&input[period_data_start_pos], RR_SPAN, RR_data_process_len, period_data);
    int start_pos = 0;
    int end_pos = RR_data_process_len - 1 - RR_SPAN;

    for (int num = start_pos;num < end_pos;num++)
    {
        period_data[midDataLen] = period_data[num + 1] - period_data[num];
        midDataLen = midDataLen + 1;
    }

    for (int num = 0; num < midDataLen; num++)
    {
        mean_value = mean_value + period_data[num];
        if (min_value > period_data[num])
        {
            min_value = period_data[num];
        }
    }

    if(midDataLen != 0)
    {
        mean_value = mean_value/midDataLen;
    }
    for (int num = 0; num < midDataLen; num++)
    {
        if (period_data[num] < mean_value)
        {
            mean_value_two = mean_value_two + period_data[num];
            mean_value_cnt = mean_value_cnt + 1;
        }
    }

    if (mean_value_cnt != 0)
    {
        mean_value_two = mean_value_two / mean_value_cnt;
    }
    else
    {
        mean_value_two = mean_value;
    }
	
    threshold_value = (min_value + mean_value_two) / 3;
    memset(threshold_end_pos, 0, (RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM) * sizeof(uint16_t));
    for (int num = 0; num < midDataLen - 1; num++)
    {
        if (period_data[num] >= threshold_value && period_data[num + 1] < threshold_value)
        {
            threshold_end_pos[threshold_end_cnt] = num;
            threshold_end_cnt = threshold_end_cnt + 1;
        }
        else if (period_data[num] <= threshold_value && period_data[num + 1] > threshold_value)
        {
            threshold_end_pos[threshold_end_cnt] = num + 1;
            threshold_end_cnt = threshold_end_cnt + 1;
        }

    }
    memset(period_peak_pos, 0, (RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM) * sizeof(uint16_t));
    period_peak_cnt = 0;
    for (int num = 0;num < threshold_end_cnt - 1;num++)
    {
        min_value = 1000;
        min_pos = 0;
        get_min_valid(&period_data[threshold_end_pos[num]], threshold_end_pos[num+1] - threshold_end_pos[num], &min_value, &min_pos);
        min_pos = min_pos + threshold_end_pos[num];

        //get real RR peaks
        left_keep_up_change_value = 0;
        right_keep_up_change_value = 0;

        for(int num_value_change = min_pos;num_value_change > 1;num_value_change--)
        {
            if (period_data[num_value_change - 1] - period_data[num_value_change] > 0)
            {
                left_keep_up_change_value = period_data[num_value_change-1] - period_data[min_pos];
            }
            else{
                break;
            }
        }
        for(int num_value_change = min_pos;num_value_change < RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM - 1;num_value_change++)
        {
            if (period_data[num_value_change + 1] - period_data[num_value_change] > 0)
            {
                right_keep_up_change_value = period_data[num_value_change + 1] - period_data[min_pos];
            }
            else{
                break;
            }
        }


		//if (min_value < threshold_value)
        if (min_value < threshold_value && min_value <-1*LOW_PERFUSION_RATIO_THRESHOLD && (left_keep_up_change_value > VAILD_HEART_RR_LEFT_RIGHT_UP_MIN_VALUE || right_keep_up_change_value > VAILD_HEART_RR_LEFT_RIGHT_UP_MIN_VALUE))
        {
            period_peak_pos[period_peak_cnt] = min_pos;
            period_peak_cnt = period_peak_cnt + 1;
        }
    }
    if (period_peak_cnt > 0)
    {
        pre_data_len = RR_data_process_len - period_peak_pos[period_peak_cnt - 1] + 50;
    }
    else 
    {
        pre_data_len = 50;
    }
	//rr get
    for (int num = 0;num < period_peak_cnt - 1;num++)
    {
        if (AlgPara->HrCnt < MAX_RR_Cnt)
        {
            if(period_peak_pos[num + 1] - period_peak_pos[num] < 256)
            {
                if(period_peak_pos[num + 1] - period_peak_pos[num] == 255)//0xFF
                {
                    AlgPara->HrVect[AlgPara->HrCnt] = 254;//0xFE
                    AlgPara->HrCnt++;
                }
                else
                {
                    AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(period_peak_pos[num + 1] - period_peak_pos[num]);
                    AlgPara->HrCnt++;
                }

            }
            else{
                AlgPara->HrVect[AlgPara->HrCnt] = 255;//0xFF
                AlgPara->HrCnt++;
                AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(((period_peak_pos[num + 1] - period_peak_pos[num])>>8) | 0xF0);
                AlgPara->HrCnt++;
                AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(((period_peak_pos[num + 1] - period_peak_pos[num])) & 0xFF);
                AlgPara->HrCnt++;
            }

        }
    }


}


uint16_t g_pre_rr = 0;
void Sleep_PPG_data_RR_get_new(int* input, int dataLen, int* Acc, int Acclen, AlgProc_Para* AlgPara)
{
	int period_data_start_pos = 0;
	int threshold_end_cnt = 0;
	int period_peak_cnt = 0;
	float mean_value = 0;
	float mean_value_two = 0;
	int mean_value_cnt = 0;
	int min_value = 5000;
	float threshold_value = 0;
	int midDataLen = 0;
	uint16_t min_pos = 0;
	uint16_t RR_data_process_len = 0;
	AlgPara->HrCnt = 0;
    float left_keep_up_change_value = 0;
    float right_keep_up_change_value = 0;
	memset(period_data,0,(RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM + RR_PROCESS_DATA_OFFSET + RR_SPAN)*sizeof(int));
	memset(period_data_amp, 0, (RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM) * sizeof(int));
	if (pre_data_len > PRE_RR_PROCESS_DATA_NUM)
	{
		pre_data_len = PRE_RR_PROCESS_DATA_NUM;
	}
	period_data_start_pos = dataLen - RR_PROCESS_DATA_NUM - pre_data_len - RR_SPAN - RR_PROCESS_DATA_OFFSET;
    if(period_data_start_pos < 0)
    {
        period_data_start_pos = 0;
    }
	RR_data_process_len = RR_PROCESS_DATA_NUM + pre_data_len + RR_SPAN + RR_PROCESS_DATA_OFFSET;
	smooth_data_int(&input[period_data_start_pos], RR_SPAN, RR_data_process_len, period_data);

    RR_data_process_len = RR_data_process_len - RR_SPAN - RR_PROCESS_DATA_OFFSET;
    memcpy(period_data,&period_data[RR_SPAN],RR_data_process_len*sizeof(int));
	int start_pos = 0;
	int end_pos = RR_data_process_len - 1;



	memcpy(period_data_amp, period_data, RR_data_process_len * sizeof(int));
	int amp_period_peaks_cnt = 0;
	for (int num = 3; num < RR_data_process_len - 3; num++)
	{
		if (period_data_amp[num - 3] <= period_data_amp[num - 2] && period_data_amp[num - 2] <= period_data_amp[num - 1] && period_data_amp[num - 1] <= period_data_amp[num] && period_data_amp[num] >= period_data_amp[num + 1] && period_data_amp[num + 1] >= period_data_amp[num + 2] && period_data_amp[num + 2] >= period_data_amp[num + 3])
		{
			period_amp_peak_pos[amp_period_peaks_cnt] = num;
			period_amp_peak_value[amp_period_peaks_cnt] = period_data_amp[num];
			amp_period_peaks_cnt = amp_period_peaks_cnt + 1;
		}
	}


	for (int num = start_pos;num < end_pos;num++)
	{
		period_data[midDataLen] = period_data[num + 1] - period_data[num];
		midDataLen = midDataLen + 1;
	}

	for (int num = 0; num < midDataLen; num++)
	{
		mean_value = mean_value + period_data[num];
		if (min_value > period_data[num])
		{
			min_value = period_data[num];
		}
	}

    if(midDataLen != 0)
    {
        mean_value = mean_value/midDataLen;
    }
	for (int num = 0; num < midDataLen; num++)
	{
		if (period_data[num] < mean_value)
		{
			mean_value_two = mean_value_two + period_data[num];
			mean_value_cnt = mean_value_cnt + 1;
		}
	}

	if (mean_value_cnt != 0)
	{
		mean_value_two = mean_value_two / mean_value_cnt;
	}
	else
	{
		mean_value_two = mean_value;
	}
	
	threshold_value = (min_value + mean_value_two) / 3;
	memset(threshold_end_pos, 0, (RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM) * sizeof(uint16_t));
	for (int num = 0; num < midDataLen - 1; num++)
	{
		if (period_data[num] >= threshold_value && period_data[num + 1] < threshold_value)
		{
			threshold_end_pos[threshold_end_cnt] = num;
			threshold_end_cnt = threshold_end_cnt + 1;
		}
		else if (period_data[num] <= threshold_value && period_data[num + 1] > threshold_value)
		{
			threshold_end_pos[threshold_end_cnt] = num + 1;
			threshold_end_cnt = threshold_end_cnt + 1;
		}

	}
	memset(period_peak_pos, 0, (RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM) * sizeof(uint16_t));
	period_peak_cnt = 0;
	for (int num = 0;num < threshold_end_cnt - 1;num++)
	{
		min_value = 1000;
		min_pos = 0;
		// get_min_valid(&period_data[threshold_end_pos[num]], threshold_end_pos[num+1] - threshold_end_pos[num], &min_value, &min_pos);
        get_min_valid(&period_data[threshold_end_pos[num]], threshold_end_pos[num+1] - threshold_end_pos[num] + 1, &min_value, &min_pos);
		min_pos = min_pos + threshold_end_pos[num];

        //get real RR peaks
        left_keep_up_change_value = 0;
        right_keep_up_change_value = 0;

        for(int num_value_change = min_pos;num_value_change > 1;num_value_change--)
        {
            if (period_data[num_value_change - 1] - period_data[num_value_change] > 0)
            {
                left_keep_up_change_value = period_data[num_value_change-1] - period_data[min_pos];
            }
            else{
                break;
            }
        }
        for(int num_value_change = min_pos;num_value_change < RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM - 1;num_value_change++)
        {
            if (period_data[num_value_change + 1] - period_data[num_value_change] > 0)
            {
                right_keep_up_change_value = period_data[num_value_change + 1] - period_data[min_pos];
            }
            else{
                break;
            }
        }


		//if (min_value < threshold_value)
        if (min_value < threshold_value && min_value <-1*LOW_PERFUSION_RATIO_THRESHOLD && (left_keep_up_change_value > VAILD_HEART_RR_LEFT_RIGHT_UP_MIN_VALUE || right_keep_up_change_value > VAILD_HEART_RR_LEFT_RIGHT_UP_MIN_VALUE))
		{
			period_peak_pos[period_peak_cnt] = min_pos;
			period_peak_cnt = period_peak_cnt + 1;
		}
	}
	//down amp and diff get RR
	int period_amp_diff_peak_cnt = 0;
	float rangePos = 0;
	int amp_peaks_max = -10000;
	uint16_t amp_peaks_max_pos = 0;
	uint16_t startPos = 0;
	memset(period_amp_diff_peak_pos, 0, (RR_PROCESS_DATA_NUM + PRE_RR_PROCESS_DATA_NUM) * sizeof(uint16_t));
	for (int num = 0; num < period_peak_cnt; num++)
	{
		if (g_pre_rr > 0)
		{
			rangePos = g_pre_rr / 2;
		}
		else
		{
			rangePos = RNAGE_POS;
		}
		if (rangePos > RNAGE_POS_MAX)
		{
			rangePos = RNAGE_POS;
		}
		amp_peaks_max = -10000;
		amp_peaks_max_pos = 0;
		for (int amp_num = 0; amp_num < amp_period_peaks_cnt; amp_num++)
		{
			startPos = period_peak_pos[num] - rangePos;
			if (startPos < 0)
			{
				startPos = 0;
			}
			if (period_amp_peak_pos[amp_num] > startPos && period_amp_peak_pos[amp_num] < period_peak_pos[num])
			{
				if (period_amp_peak_value[amp_num] > amp_peaks_max)
				{
					amp_peaks_max = period_amp_peak_value[amp_num];
					amp_peaks_max_pos = period_amp_peak_pos[amp_num];

				}

			}
		}
		if (amp_peaks_max_pos == 0)
		{
			for (int amp_num = amp_period_peaks_cnt; amp_num > 0; amp_num--)
			{
				if (period_amp_peak_pos[amp_num] < period_peak_pos[num])
				{
					amp_peaks_max = period_amp_peak_value[amp_num];
					amp_peaks_max_pos = period_amp_peak_pos[amp_num];
					break;
				}

			}
		}
		if (amp_peaks_max_pos != 0)
		{
			period_amp_diff_peak_pos[period_amp_diff_peak_cnt] = amp_peaks_max_pos;
			period_amp_diff_peak_cnt = period_amp_diff_peak_cnt + 1;
		}

	}





	if (period_peak_cnt > 0 && period_amp_diff_peak_cnt > 0)
	{
        if(period_peak_cnt == period_amp_diff_peak_cnt)
        {
            pre_data_len = RR_data_process_len - period_peak_pos[period_peak_cnt - 1] + 20;
        }
        else
        {
            pre_data_len = RR_data_process_len - period_peak_pos[period_amp_diff_peak_cnt - 1] + 20;
        }
		
	}
	else 
	{
		pre_data_len = 50;
	}



	//rr get
    if(period_peak_cnt >= 2)
    {
        for (int num = 0;num < period_peak_cnt - 1;num++)
        {
            if (AlgPara->HrCnt < MAX_RR_Cnt)
            {
                if(period_peak_pos[num + 1] - period_peak_pos[num] < 256)
                {
                    if(period_peak_pos[num + 1] - period_peak_pos[num] == 255)//0xFF
                    {
                        AlgPara->HrVect[AlgPara->HrCnt] = 254;//0xFE
                        AlgPara->HrCnt++;
                        
                    }
                    else
                    {
                        AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(period_peak_pos[num + 1] - period_peak_pos[num]);
                        AlgPara->HrCnt++;
                    }

                }
                else{
                    AlgPara->HrVect[AlgPara->HrCnt] = 255;//0xFF
                    AlgPara->HrCnt++;
                    AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(((period_peak_pos[num + 1] - period_peak_pos[num])>>8) | 0xF0);
                    AlgPara->HrCnt++;
                    AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(((period_peak_pos[num + 1] - period_peak_pos[num])) & 0xFF);
                    AlgPara->HrCnt++;
                }
                g_pre_rr = period_peak_pos[num + 1] - period_peak_pos[num];
            }
        }

    }
    else
    {
        AlgPara->HrVect[AlgPara->HrCnt] = 255;//0xFF
        AlgPara->HrCnt++;
        AlgPara->HrVect[AlgPara->HrCnt] = 255;//0xFF
        AlgPara->HrCnt++;

    }
	
    #if 0
    if(period_amp_diff_peak_cnt >= 2)
    {
        for (int num = 0;num < period_amp_diff_peak_cnt - 1;num++)
        {
            if (AlgPara->HrCnt < MAX_RR_Cnt)
            {
                if(period_amp_diff_peak_pos[num + 1] - period_amp_diff_peak_pos[num] < 256)
                {
                    if(period_amp_diff_peak_pos[num + 1] - period_amp_diff_peak_pos[num] == 255)//0xFF
                    {
                        AlgPara->HrVect[AlgPara->HrCnt] = 254;//0xFE
                        AlgPara->HrCnt++;
                        
                    }
                    else
                    {
                        AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(period_amp_diff_peak_pos[num + 1] - period_amp_diff_peak_pos[num]);
                        AlgPara->HrCnt++;
                    }

                }
                else{
                    AlgPara->HrVect[AlgPara->HrCnt] = 255;//0xFF
                    AlgPara->HrCnt++;
                    AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(((period_amp_diff_peak_pos[num + 1] - period_amp_diff_peak_pos[num])>>8) | 0xF0);
                    AlgPara->HrCnt++;
                    AlgPara->HrVect[AlgPara->HrCnt] = (uint8_t)(((period_amp_diff_peak_pos[num + 1] - period_amp_diff_peak_pos[num])) & 0xFF);
                    AlgPara->HrCnt++;
                }
                g_pre_rr = period_amp_diff_peak_pos[num + 1] - period_amp_diff_peak_pos[num];
            }
        }

    }
    else
    {
        AlgPara->HrVect[AlgPara->HrCnt] = 255;//0xFF
        AlgPara->HrCnt++;
        AlgPara->HrVect[AlgPara->HrCnt] = 255;//0xFF
        AlgPara->HrCnt++;

    }
	
    #endif

}


#if 1

uint8_t get_zeropoints(int *input, int inlen, uint8_t *ZeroCnt)
{
    *ZeroCnt = 0;
    int threshold = 0;
    int tcnt = 0;
//找出过0点 zerosVect
    for (int i = 0; i < inlen; i++)
    {
        if ((input[i] <= threshold && input[i + 1] > threshold) || (input[i] >= threshold && input[i + 1] < threshold))
        {
            zerosVect[tcnt++] = i;
        }
        if (tcnt >= MAX_ZEROS_NUM)
        {
            break;
        }
    }
    if (tcnt <= 7)
    {
        return 1;
    }

    int ZerosVar1 = get_ZerosVectvar(zerosVect, tcnt);//周期差和均值
    int ZerosVar2 = get_ZerosVectvar(zerosVect+1, tcnt-1);
    int loopcnt = 0;
    do
    {
        loopcnt++;
//计算过0点 差和均值 tmpmean
        AlgData_t tmpmean = 0;
        int tmeancnt = 0;
        //int tmean1 = inlen / (tcnt - 1);
        for (int i = 0; i < tcnt - 1; i++)
        {
            tmpmean += zerosVect[i + 1] - zerosVect[i];
            tmeancnt++;
        }
        if (tmeancnt > 0)
        {
            tmpmean = tmpmean / tmeancnt;
        }
        else
        {
            return 2;
        }
        int tthreshold = MIN(MAX(tmpmean*0.6, 6),15);
        int ttcnt = 2;
        int tsp = 0;
        while (tsp < tcnt - 2 && zerosVect[tsp + 1] - zerosVect[tsp] < tthreshold)
        {
                zerosVect[0] = zerosVect[tsp+1];
                tsp++;
        }
        zerosVect[1] = zerosVect[tsp+1];
        for (int i = tsp + 1; i < tcnt - 2; i++)
        {
            if (zerosVect[i + 1] - zerosVect[i] < tthreshold)
            {
                if (zerosVect[i + 2] - zerosVect[i] < 2.2 * tthreshold)
                {
                    zerosVect[ttcnt] = zerosVect[i + 3];
                    ttcnt++;
                    i += 2;
                }
                else
                {
                    if (zerosVect[i + 2] - zerosVect[ttcnt - 1] < 2.2 * tthreshold)
                    {
                        //printf("debug");
                    }
                    else
                    {
                        zerosVect[ttcnt++] = zerosVect[i + 1];
                    }
                }
            }
            else
            {
                    zerosVect[ttcnt] = zerosVect[i + 1];
                    ttcnt++;
            }
        }
        tcnt = ttcnt;

        int tZerosVar1 = get_ZerosVectvar(zerosVect, tcnt);
        int tZerosVar2 = get_ZerosVectvar(zerosVect+1, tcnt-1);
        if (tZerosVar1 == ZerosVar1 && tZerosVar2 == ZerosVar2)
        {
                break;
        }
        ZerosVar1 = tZerosVar1;
        ZerosVar2 = tZerosVar2;
    } while (loopcnt < 3 && abs(ZerosVar1 - ZerosVar2) > 15);

    int ttcnt = 0;
    if (ZerosVar1 < ZerosVar2)
    {
        for (int i = 0; i < tcnt / 2; i++)
        {
            zerosVect[ttcnt++] = zerosVect[i*2];
        }
    }
    else
    {
        for (int i = 0; i < (tcnt-1) / 2; i++)
        {
            zerosVect[ttcnt++] = zerosVect[i * 2 + 1];
        }
    }

    *ZeroCnt = ttcnt;
    return 0;
}
int test_acmeanval = 0;
int get_acdcval(int *dcin, int *acin, int inlen, int zeronum, float *acdcbuf)
{
    test_acmeanval = 0;
    if(zeronum < 2)
    {
        return 0;
    }
    int num = 0;
    for (int i = 0; i < zeronum - 1; i++)
    {
        int tmpsp = zerosVect[i];
        int tmpep = zerosVect[i+1];
        int clclen = tmpep - tmpsp;
        int maxval = 0;
        int minval = 0;
        get_max_min(acin + tmpsp, clclen, &maxval, &minval);
        int tmpac = maxval - minval;
        test_acmeanval = test_acmeanval + tmpac;

        int tmpdc = get_int_mean(dcin + tmpsp, clclen);
        if(tmpdc != 0 && num < MAX_ACDC_NUM)
        {
            acdcbuf[num++] = (float)tmpac*1.0/tmpdc;
        }
    }
    test_acmeanval = test_acmeanval/(zeronum - 1);
    return num;
}

uint8_t get_correct_rate(float *tmpr, int inleno, float *rate)
{
//add jude rrbuf
    AlgData_t minScale = 0.7;
    AlgData_t maxScale = 1.22;

    int inlen = 0;
    for (int i = 0; i < inleno; i++)
    {
        if (tmpr[i] > MIN_ADRATE && tmpr[i] < MAX_ADRATE)
        {
            tmpr[inlen] = tmpr[i];
            inlen++;
        }
        //else
        //{
        //    if (tmpr[i] <= MIN_ADRATE)
        //    {
        //        tmpr[i] = MIN_ADRATE;
        //    }
        //    else
        //    {
        //        tmpr[i] = MAX_ADRATE;
        //    }
        //}
    }

    if (inlen < 5)
    {
        *rate = Hist_Hr_Rate;
        return 0;// Untrusted
    }

    AlgData_t minval = 100;
    uint8_t minpos = 1;
    for (int i = 0; i < inlen; i++)
    {
        if (minval > tmpr[i])
        {
            minval = tmpr[i];
            minpos = i;
        }
    }

    AlgData_t maxval = 0;
    uint8_t maxpos = 0;
    for (int i = 0; i < inlen; i++)
    {
        if (maxval < tmpr[i])
        {
            maxval = tmpr[i];
            maxpos = i;
        }
    }

    AlgData_t tmpmeanval = 0;
    for (int i = 0; i < inlen; i++)
    {
        tmpmeanval += tmpr[i];
    }
    tmpmeanval = tmpmeanval / inlen;

    if(tmpr[0] > tmpmeanval*1.3 )
    {
        tmpr[0] = tmpmeanval;
    }
    if(tmpr[inlen-1] > tmpmeanval*1.3 )
    {
        tmpr[inlen-1] = tmpmeanval;
    }
    for (int i = 1; i < inlen-1; i++)  //first
    {
        if(tmpr[i] > tmpmeanval*1.3)
        {
            tmpr[i] = (tmpr[i+1] + tmpr[i-1])/2;
        }

    }
    for (int i = 1; i < inlen-1; i++)  //second
    {
        if(tmpr[i] > tmpmeanval*1.3 || (tmpr[i] > 1.25*tmpr[i+1] && tmpr[i] > 1.25*tmpr[i-1]))
        {
            tmpr[i] = (tmpr[i+1] + tmpr[i-1])/2;
        }

    }
    for (int i = 1; i < inlen-1; i++)  //
    {
        float tmpSpo20 = -21.174*tmpr[i-1]*tmpr[i-1] - 2.1003*tmpr[i-1] + 105.06 + 0.8;
        float tmpSpo21 = -21.174*tmpr[i]*tmpr[i] - 2.1003*tmpr[i] + 105.06 + 0.8;
        float tmpSpo22 = -21.174*tmpr[i+1]*tmpr[i+1] - 2.1003*tmpr[i+1] + 105.06 + 0.8;
        if(tmpSpo21 + 3 < tmpSpo20 && tmpSpo21 + 3 < tmpSpo22)
        {
            tmpr[i] = (tmpr[i+1] + tmpr[i-1])/2;
        }

    }
    
	    
    AlgData_t tmeanval = 0;
    for (int i = 0; i < inlen; i++)
    {
        tmeanval += tmpr[i];
    }
    tmeanval = tmeanval / inlen;


    minval = 100;
    maxval = 0;
    for (int i = 0; i < inlen; i++)
    {
        if (minval > tmpr[i])
        {
            minval = tmpr[i];
        }
        if(maxval < tmpr[i])
        {
            maxval = tmpr[i];
        }
    }
	
    AlgData_t minval_last3minit = 100;
    int sp = inlen - 3;
    for (int i = sp; i < inlen; i++)
    {
        if (minval_last3minit > tmpr[i])
        {
            minval_last3minit = tmpr[i];
        }
    }

    if (maxval < minval * 2)
    {
        if (Hist_Hr_Rate > 0)
        {
        #if 1
            if (tmeanval > Hist_Hr_Rate * minScale && tmeanval < Hist_Hr_Rate * maxScale)
            {
                *rate = tmeanval;
            }
            else
            {
                *rate = tmeanval * 0.2 + Hist_Hr_Rate * 0.8;
            }
        #else
            if(minval_last3minit > Hist_Hr_Rate * minScale && minval_last3minit < Hist_Hr_Rate * maxScale) 
            { 
                *rate = minval_last3minit;
            }
            else
            {
                *rate = minval_last3minit* 0.2 + Hist_Hr_Rate * 0.8;
            }
        #endif
        }
        else
        {
            *rate = tmeanval;
        }
        Hist_Hr_Rate = *rate;

        return 1;// trusted
    }
    else
    {
        *rate = Hist_Hr_Rate;
    }
    Hist_Hr_Rate = *rate;
    return 0;// Untrusted
 //add end
}
#endif
int get_var(int *input, int inlen)
{
	if (inlen <= 0)
	{
		return 0;
	}

	int sum = 0;
	for (int i = 0; i < inlen; i++)
	{
		sum += input[i];
	}

	int mean = sum / inlen;

	int var = 0;
	for (int i = 0; i < inlen; i++)
	{
		var += abs(input[i] - mean);
	}

	var = var / inlen;
	return var;
}

AlgData_t get_DC(int *in, int inlen)
{
	AlgData_t DC = 0;
	for (int i = 0; i < inlen; i++)
	{
		DC += in[i];
	}
	return DC;
}

int get_DCVect(int *red, int *infra, int inlen, AlgData_t *DCVect, int* DCVCnt)
{
	*DCVCnt = 0;
	AlgData_t RedDCvect[(int)(WAVE_PROC_LEN / 100)];
	AlgData_t InfraDCvect[(int)(WAVE_PROC_LEN / 100)];
	RedDCvect[0] = get_DC(red, inlen);
	InfraDCvect[0] = get_DC(infra, inlen);
	// RedDCvect[0] = get_DC(&red[400], inlen-400);
	// InfraDCvect[0] = get_DC(&infra[400], inlen-400);
	*DCVCnt = *DCVCnt + 1;
	for (int i = 1; i < inlen - 200; i += 100)
	// for (int i = 401; i < inlen - 200; i += 100)
	{
		RedDCvect[*DCVCnt] = get_DC(red + i, 200);
		InfraDCvect[*DCVCnt] = get_DC(infra + i, 200);
		*DCVCnt = *DCVCnt+1;
	}

	int flg = 0;
	for (int i = 0; i < *DCVCnt; i++)
	{
		if (InfraDCvect[i] == 0)
		{
			flg = -1;
			return flg;
		}
		if (InfraDCvect[i] > 0)
		{
			DCVect[i] = RedDCvect[i] * 1.0 / InfraDCvect[i];
		}
		else
		{
			DCVect[i] = 1;
		}
	}
	return flg;
}

float get_abs_sum(int *in, int inlen)
{
	float sum = 0;
	for (int i = 0; i < inlen; i++)
	{
		sum += abs(in[i]);
	}
	return sum;
}

void get_PPvect(int *in, int inlen, int steplen, int proclen, int *PPVect, int *PPCnt)
{
	*PPCnt = 0;
	for (int i = 0; i < inlen - proclen; i+=steplen)
	{
		PPVect[*PPCnt] = get_peakval(in+i, proclen);
		*PPCnt = *PPCnt + 1;
	}
}

void get_Pwrvect(int *in, int inlen, int steplen, int proclen, AlgData_t *PwrVect, int *PwrCnt)
{
	*PwrCnt = 0;
	PwrVect[*PwrCnt] = get_abs_sum(in, inlen);
	// PwrVect[*PwrCnt] = get_abs_sum(&in[400], inlen - 400);
	*PwrCnt = *PwrCnt + 1;
	for (int i = 1; i < inlen - proclen; i += steplen)
	// for (int i = 401; i < inlen - proclen; i += steplen)
	{
		PwrVect[*PwrCnt] = get_abs_sum(in + i, proclen);
		*PwrCnt = *PwrCnt + 1;
	}
}

void get_max_min(int *in, int inlen, int *max, int *min)
{
	*max = 0;
	*min = 1e9;
	for (int i = 0; i < inlen; i++)
	{
		if (*max < in[i])
		{
			*max = in[i];
		}
		if (*min > in[i])
		{
			*min = in[i];
		}
	}
}

void wavelet_baseProc(int *in, int inlen, int slevel, int elevel, int *out)
{
	dwt(wt, in);// Perform DWT   

	int sp = 0;
	for (int i = 0; i < slevel; i++)
	{
		sp += wt->length[i];
	}

	int ep = sp;
	for (int i = slevel; i < elevel; i++)
	{
		ep += wt->length[i];
	}
	for (int i = 0; i < sp; i++)
	{
		wt->output[i] = 0;
	}

	int totallen = ep;
	for (int i = elevel; i < 9; i++)
	{
		totallen += wt->length[i];
	}

	for (int i = ep; i < totallen; i++)
	{
		wt->output[i] = 0;
	}

	idwt(wt, out);
}

uint8_t test_res = 0;
void waveletProc(int *in, int inlen, int slevel, int elevel, int *out, float *PeakVect, int *Peakcnt, uint8_t clcflg, uint8_t *zerocnt)
{
	wavelet_baseProc(in, inlen, slevel, elevel, out);

	get_Pwrvect(out, inlen, 100, 200, PeakVect, Peakcnt);

    uint8_t zcnt = 0;
    if(clcflg == 1)
    {
    	uint8_t res = get_zeropoints(out, inlen, &zcnt);
    	test_res = res;
    }
    *zerocnt = zcnt;
}

#if 1
void get_Spo2Rate(AlgData_t *tmpr, AlgData_t *histr, int inleno, int Acc, AlgData_t Varrate, AlgData_t *Rate)
{
	AlgData_t minScale = 0.7;
	AlgData_t maxScale = 1.22;
	int vflg = 0;
	static AlgData_t meanVarbck = 0;
	static AlgData_t Accbck = 0;
	inleno = inleno - 2;
	if (meanVarbck == 0)
	{
		meanVarbck = Varrate;
	}
	else
	{
		meanVarbck = meanVarbck * 0.95 + Varrate * 0.05;
	}
	AlgData_t tmpVar = MAX(meanVarbck, Varrate);

	if (Accbck == 0)
	{
		Accbck = Acc;
	}
	else
	{
		Accbck = Acc * 0.15 + Accbck * 0.85;
	}
	Accbck = MIN(Accbck, 20);

	AlgData_t tmpAcc = MAX(Accbck, Acc);

	if ((tmpAcc < 5 && Varrate < MAXVAR_THRESHOLD) || (Varrate < MIN(MAXVAR_THRESHOLD, Varthreshold * 3)))
	{
		int savepos = HisVarCnt % MAXVAR_CNT;
		HisVarvect[savepos] = Varrate;
		HisVarCnt++;
		int callen = MAXVAR_CNT;
		if (HisVarCnt < MAXVAR_CNT)
		{
			callen = HisVarCnt;
		}

		AlgData_t tmpmeanVar = 0;
		for (int i = 0; i < callen; i++)
		{
			tmpmeanVar += HisVarvect[i];
		}
		tmpmeanVar /= callen;
		meanVarbck = tmpmeanVar;

		Varthreshold = Varthreshold * 0.95 + tmpmeanVar * 0.05;
	}

	//if ((tmpAcc < 5 && tmpVar < MAX(MIN(MAXVAR_THRESHOLD, Varthreshold*2),1e-3)) || (tmpAcc >= 5 && tmpVar < MAX(MIN(MAXVAR_THRESHOLD, Varthreshold*1.5),6e-4)))
	if ((tmpAcc < 5 && tmpVar < MAX(MIN(MAXVAR_THRESHOLD, Varthreshold*2),5e-4)) || (tmpAcc >= 5 && tmpVar < MAX(MIN(MAXVAR_THRESHOLD, Varthreshold*1.5),3e-4)))
	{
		vflg = 1;
	}
	else
	{
		//if ((Acc < 5 && Varrate < MAX(MIN(MAXVAR_THRESHOLD, Varthreshold*1.8), 8e-4)) || (Acc >= 5 && Varrate < MAX(MIN(MAXVAR_THRESHOLD, Varthreshold*1.3), 6e-4)))
		if ((Acc < 5 && Varrate <MIN(MAXVAR_THRESHOLD, Varthreshold*1.8)) || (Acc >= 5 && Varrate < MIN(MAXVAR_THRESHOLD, Varthreshold*1.3)))
		{
			minScale = 0.75;
			maxScale = 1.14;

		}
		else
		{
			minScale = 0.8;
			maxScale = 1.1;

		}
	}

	int inlen = 0;
	for (int i = 1; i < inleno; i++)
	{
		if (tmpr[i] > MIN_ADRATE && tmpr[i] < MAX_ADRATE)
		{
			tmpr[inlen++] = tmpr[i];
		}
		else
		{
			if (tmpr[i] <= MIN_ADRATE)
			{
				tmpr[inlen++] = MIN_ADRATE;
			}
		}
	}

	if (inlen < 4)
	// if (inlen < 2)
	{
		*Rate = HistRate;
		return;
	}

	AlgData_t minval = 100;
	for (int i = 0; i < inlen; i++)
	{
		if (minval > tmpr[i])
		{
			minval = tmpr[i];
		}
	}

	AlgData_t maxval = 0;
	for (int i = 0; i < inlen; i++)
	{
		if (maxval < tmpr[i])
		{
			maxval = tmpr[i];
		}
	}

	AlgData_t tmeanval = 0;
	for (int i = 0; i < inlen; i++)
	{
		tmeanval += tmpr[i];
	}
	tmeanval = tmeanval / inlen;
	
	if (vflg == 1 && maxval < minval * 1.4)
	{
		if ((tmpr[0] > HistRate * 0.98 && tmpr[0] < HistRate * 1.02))
		{
			*Rate = tmpr[0];
		}
		else
		{
			if (HistRate > 0)
			{
				if (tmeanval > HistRate * minScale && tmeanval < HistRate * maxScale)
				{
					*Rate = tmeanval;
				}
				else
				{
					*Rate = tmeanval * 0.2 + HistRate * 0.8;
				}
			}
			else
			{
				*Rate = tmeanval;
			}
		}
		memcpy(histr, tmpr, inlen * sizeof(AlgData_t));
		HistRateCnt = inlen;
		HistRate = *Rate;

		return;
	}

	AlgData_t tmeanvval = 0;
	if (HistRate > 0)
	{
		int tmpvldcnt = 0;
		for (int i = 0; i < inlen; i++)
		{
			if (tmpr[i] > HistRate * minScale && tmpr[i] < HistRate * maxScale)
			{
				tmeanvval += tmpr[i];
				tmpvldcnt++;
			}
		}
		if (tmpvldcnt > 0)
		{
			tmeanvval = tmeanvval / tmpvldcnt;
		}
	}

	if (tmeanvval > 0)
	{
		if (vflg == 1)
		{
			if (tmeanvval < HistRate)
			{
				*Rate = tmeanvval * 0.5 + HistRate * 0.5;
			}
			else
			{
				*Rate = tmeanvval * 0.3 + HistRate * 0.7;
			}
		}
		else
		{
			if (tmeanvval < HistRate)
			{
				*Rate = tmeanvval * 0.3 + HistRate * 0.7;
			}
			else
			{
				*Rate = tmeanvval * 0.05 + HistRate * 0.95;
			}
		}

	}
	else
	{
		if (HistRateCnt > 1)
		{
			if (histr[1] > HistRate*minScale && histr[1] < HistRate*maxScale)
			{
				*Rate = histr[1] * 0.5 + HistRate * 0.5;
				memcpy(histr, histr + 1, (HistRateCnt - 1) * 4);
				HistRateCnt--;
			}
			else
			{
				*Rate = tmeanval * 0.01 + HistRate * 0.99;
			}
		}
		else
		{
			if (HistRate > 0)
			{
				if (tmeanval < HistRate)
				{
					*Rate = tmeanval * 0.1 + HistRate * 0.9;
				}
				else
				{
					*Rate = tmeanval * 0.01 + HistRate * 0.99;
				}
			}
			else
			{
				if (maxval < minval * 1.2)
				{
					*Rate = tmeanval;
				}
				else
				{
					*Rate = HistRate;
				}
			}
		}
	}

	HistRate = *Rate;

}
#endif

#define UP_FLAG (1)
#define NO_UP_FLAG (0)
#define DOWN_FLAG (1)
#define NO_DOWN_FLAG (0)
void breath_rate_get(uint8_t* rrData,int rrLen,uint8_t*breathRate)
{
	int rr_sum = 0;
	int up_flag = 0;
	int down_flag = 0;
	for (int num = rrLen - 1;num >= 0;num--)
	{
		rr_sum = rr_sum + rrData[num];
		if (rr_sum >= BREATH_RATE_TIME)
		{
			for (int num1 = num + 1;num1 < rrLen - 1;num1++)
			{
				if (rrData[num1] > rrData[num1 - 1])
				{
					up_flag = UP_FLAG;
				}
				if (up_flag == UP_FLAG && rrData[num1] > rrData[num1 + 1])
				{
					down_flag = DOWN_FLAG;
				}

				if (up_flag == UP_FLAG && down_flag == DOWN_FLAG)
				{
					*breathRate = *breathRate + 1;
					up_flag = NO_UP_FLAG;
					down_flag = NO_DOWN_FLAG;
				}

			}
			break;
		}
	}
}

float spo2_fortest = 0;
int test_trustflg = 0;
int test_infra_acdc_cnt = 0;
uint8_t test_infra_zecnt = 0;
extern int health_green[HRV_PROCESS_DATA_PERIOD];

uint16_t g_HrRRNum[HR_ALG_RR_NUM] = {0};
uint8_t g_HrRRCnt = 0;
uint8_t g_breath_rate_rr[BREATH_RATE_ALG_RR_NUM] = {0};
uint32_t g_BreathRRCnt = 0;

void get_spo2(int *red, int *infra, int *green, int *red_fltbuf, int *infra_fltbuf, int inlen, int *Acc, int Acclen, AlgProc_Para *AlgPara)
{
    int Tcc = get_var(Acc, Acclen);

    AlgPara->Acc = Tcc;

    int Marglen = MARLEN;
    int Waveletlen = inlen - 2 * Marglen;

    AlgData_t DCRateVect[(int)(WAVE_PROC_LEN/100)];
    int tmpcnt = 0;
    int flg;
    flg = get_DCVect(red + Marglen, infra + Marglen, Waveletlen, DCRateVect, &tmpcnt);
    if (flg != 0)
    {
            return;
    }

    AlgData_t Owpeakvect[(int)(WAVE_PROC_LEN / 100)];
    int peakcnt = 0;
    uint8_t infra_zecnt = 0;

    int slevel = 2;
    int elevel = 6;

    wavelet_baseProc(infra_fltbuf+Marglen, Waveletlen, 1, 8, wrcoefbuf_base);
    //if (RING_AFE_HRV_ON_MODE == (RING_AFE_HRV_ON_MODE & afe_get_work_mode())) {
        //waveletProc(green + Marglen, Waveletlen, slevel, elevel, wrcoefbuf, Owpeakvect, &peakcnt, 1, &infra_zecnt);
        waveletProc(infra_fltbuf + Marglen, Waveletlen, slevel, elevel, wrcoefbuf, Owpeakvect, &peakcnt, 1, &infra_zecnt);
    //} else {
     //   waveletProc(infra + Marglen, Waveletlen, slevel, elevel, wrcoefbuf, Owpeakvect, &peakcnt, 1, &infra_zecnt);
    //}
	
    test_infra_zecnt = infra_zecnt;
    int* tmpbuf = (int*)wrcoefbuf;

#ifdef points_clc
    int infra_acdc_cnt = get_acdcval(infra, tmpbuf, Waveletlen, infra_zecnt, infra_acdc);
    test_infra_acdc_cnt = infra_acdc_cnt;
    for (int i = 0; i < infra_zecnt && i < MAX_ACDC_NUM; i++)
    {
    	red_acdc[i] = zerosVect[i];
    }

    //uint8_t green_zecnt;
    //waveletProc(green + Marglen, Waveletlen, slevel, elevel, wrcoefbuf, Owpeakvect, &peakcnt, 0, &green_zecnt);
    //uint8_t vldflg;
    //int* greentmpbuf = (int*)wrcoefbuf;

    //get_hr(greentmpbuf, Waveletlen, Tcc,AlgPara->heartratehist, &(AlgPara->heartrate), &vldflg, &(AlgPara->HrCnt), AlgPara->HrVect);
	
    
    //if (RING_AFE_HRV_ON_MODE == (RING_AFE_HRV_ON_MODE & afe_get_work_mode())) {
            //waveletProc(infra_fltbuf + Marglen, Waveletlen, slevel, elevel, wrcoefbuf, Owpeakvect, &peakcnt, 0, &zecnt);
    //} 
    tmpbuf = wrcoefbuf;
#endif
#if 1
    //if (AlgPara->heartrate > 48)
    //{
    //        int tmplen = 0;
    //        for (int i = 0; i < slevel; i++)
    //        {
    //                tmplen += wt->length[i];
    //        }
    //        for (int i = tmplen; i < tmplen + wt->length[slevel]; i++)
    //        {
    //                wt->output[i] = 0;
    //        }
    //        slevel++;
    //        idwt(wt, tmpbuf);
    //        get_Pwrvect(tmpbuf, Waveletlen, 100, 200, Owpeakvect, &peakcnt);
    //}

    float WaveletDv = 0;
    for (int i = 0; i < Waveletlen; i++)
    {
            wrcoefbuf_base[i] -= wrcoefbuf[i];
    }
    WaveletDv = get_abs_sum(wrcoefbuf_base, Waveletlen);

    float Wavesum = 0;
    for (int i = 0; i < Waveletlen; i++)
    {
            Wavesum += infra[Marglen + i];
    }

    AlgData_t WaveDvrate = WaveletDv * 1.0 / Wavesum;

//	printf("%f ", WaveDvrate);

    AlgData_t Rwpeakvect[(int)(WAVE_PROC_LEN / 100)];
    peakcnt = 0;
    uint8_t zecnt = 0;
    waveletProc(red_fltbuf + Marglen, Waveletlen, slevel, elevel, wrcoefbuf, Rwpeakvect, &peakcnt, 0, &zecnt);
#ifdef points_clc
    float hr_rateval = 0;
    tmpbuf = wrcoefbuf;
    for (int i = 0; i < infra_zecnt && i < MAX_ACDC_NUM; i++)
    {
    	zerosVect[i] = red_acdc[i];
    }
    int red_acdc_cnt = get_acdcval(red, tmpbuf, Waveletlen, infra_zecnt, red_acdc);
    for (int i = 0; i < infra_acdc_cnt; i++)
    {
    	if(infra_acdc[i] > 0)
    	{
            hr_rate[i] = red_acdc[i]/infra_acdc[i];
	
    	}
    }
    uint8_t trustflg = get_correct_rate(hr_rate, infra_acdc_cnt, &hr_rateval);
        
    test_trustflg = trustflg;
#endif

    AlgData_t Rate;
    for (int i = 0; i < peakcnt; i++)
    {
            if (Owpeakvect[i] > 0 && DCRateVect[i] > 0)
            {
                    RateVect[i] = Rwpeakvect[i] * 1.0 / Owpeakvect[i] / DCRateVect[i];
            }
            else
            {
                    RateVect[i] = MIN_ADRATE;
            }
    }
    
#ifdef FDA_CHECK
	get_Spo2Rate(RateVect, HistRateVect, peakcnt, 0, WaveDvrate, &Rate);
#else
    get_Spo2Rate(RateVect, HistRateVect, peakcnt, Tcc, WaveDvrate, &Rate);
#endif
      //NRF_LOG_INFO("RateVect: %3.3f,%3.3f,%3.3f,%3.3f,%3.3f", RateVect[0], RateVect[1], RateVect[2], RateVect[3], RateVect[4]);
#ifdef points_clc
    Rate = hr_rateval;
#endif

    AlgData_t min_ADRate = MIN_ADRATE;
    AlgData_t max_ADRate = MAX_ADRATE;

    if (Rate < min_ADRate)
    {
            Rate = min_ADRate;
    }
    if (Rate > max_ADRate)
    {
            Rate = max_ADRate;
    }
    
    AlgData_t tmpr = Rate;
    AlgPara->Spo2 = -21.174*tmpr*tmpr - 2.1003*tmpr + 105.06 + 0.8;

    if (AlgPara->Spo2 < 80)
    {
            AlgPara->Spo2 = 0.0011*AlgPara->Spo2*AlgPara->Spo2 - 0.5144*AlgPara->Spo2 + 34.825 + AlgPara->Spo2;
    }


	// spo2 data compensation
	// AlgPara->Spo2 = AlgPara->Spo2 + 1;
	// if(AlgPara->Spo2 > 99)
	// {
	// 	AlgPara->Spo2 = 99;
	// }


#ifdef points_clc
    spo2_fortest = -21.174*hr_rateval*hr_rateval - 2.1003*hr_rateval + 105.06 + 0.8;
    if(spo2_fortest < 80)
    {
        spo2_fortest = 0.0011*spo2_fortest*spo2_fortest - 0.5144*spo2_fortest + 34.825 + spo2_fortest;
    }
#endif
//get hr
    uint8_t green_zecnt;
    waveletProc(green + Marglen, Waveletlen, slevel, elevel, wrcoefbuf, Owpeakvect, &peakcnt, 0, &green_zecnt);
    uint8_t vldflg;
    int* greentmpbuf = (int*)wrcoefbuf;

   // get_hr(greentmpbuf, Waveletlen, Tcc,AlgPara->heartratehist, &(AlgPara->heartrate), &vldflg, &(AlgPara->HrCnt), AlgPara->HrVect);
	// Sleep_PPG_data_RR_get(green + Marglen,Waveletlen, Acc, Acclen,AlgPara);
    Sleep_PPG_data_RR_get_new(green + Marglen,Waveletlen, Acc, Acclen,AlgPara);

    for(int num = 0;num < AlgPara->HrCnt - 1;)
    {
        if(AlgPara->HrVect[num] == 255 && AlgPara->HrVect[num + 1] == 255)
        {
            num = num + 2;
        }
        else if(AlgPara->HrVect[num] == 255 && AlgPara->HrVect[num + 1] != 255)
        {
            num = num + 3;
        }
        else
        {
            if(AlgPara->HrVect[num] < MAX_RR && AlgPara->HrVect[num] > MIN_RR)
            {
                g_HrRRNum[g_HrRRCnt] = AlgPara->HrVect[num];
                g_HrRRCnt = g_HrRRCnt + 1;
                if(g_HrRRCnt >= HR_ALG_RR_NUM)
                {
                    g_HrRRCnt = 0;
                }
            }

            if(AlgPara->HrVect[num] < BREATH_MAX_RR && AlgPara->HrVect[num] > BREATH_MIN_RR)
            {
                // calculation breath rate rr store
                if(g_BreathRRCnt < BREATH_RATE_ALG_RR_NUM)
                {
                    g_breath_rate_rr[g_BreathRRCnt] = AlgPara->HrVect[num];
                    g_BreathRRCnt = g_BreathRRCnt + 1;
                }
                else
                {
                    //shift left 1
                    for(int RRnum = 0;RRnum < BREATH_RATE_ALG_RR_NUM - 1;RRnum++)
                    {
                        g_breath_rate_rr[RRnum] = g_breath_rate_rr[RRnum + 1];
                    }
                    g_breath_rate_rr[BREATH_RATE_ALG_RR_NUM - 1] = AlgPara->HrVect[num];
                }
            }

            num = num + 1;
        }
    }
    if(AlgPara->HrVect[AlgPara->HrCnt - 1] < MAX_RR && AlgPara->HrVect[AlgPara->HrCnt - 1] > MIN_RR)
    {
        g_HrRRNum[g_HrRRCnt] = AlgPara->HrVect[AlgPara->HrCnt - 1];
        g_HrRRCnt = g_HrRRCnt + 1;
        if(g_HrRRCnt >= HR_ALG_RR_NUM)
        {
            g_HrRRCnt = 0;
        }


        // calculation breath rate rr store
        if(g_BreathRRCnt < BREATH_RATE_ALG_RR_NUM)
        {
            g_breath_rate_rr[g_BreathRRCnt] = AlgPara->HrVect[AlgPara->HrCnt - 1];
            g_BreathRRCnt = g_BreathRRCnt + 1;
        }
        else
        {
            //shift left 1
            for(int num = 0;num < BREATH_RATE_ALG_RR_NUM - 1;num++)
            {
                g_breath_rate_rr[num] = g_breath_rate_rr[num + 1];
            }
            g_breath_rate_rr[BREATH_RATE_ALG_RR_NUM - 1] = AlgPara->HrVect[AlgPara->HrCnt - 1];
        }

    }
//rr calculation heart rate
    // get_hr_process(health_green, Waveletlen, Tcc,AlgPara->heartratehist, &(AlgPara->heartrate));
    uint8_t vaild_rr_num = 0;
    uint16_t RR_mean = 0;
    for(int num = 0;num < HR_ALG_RR_NUM;num++)
    {
        if(g_HrRRNum[num] != 0)
        {
            RR_mean = RR_mean + g_HrRRNum[num];
            vaild_rr_num = vaild_rr_num + 1;
        }
    }

    if(vaild_rr_num != 0)
    {
        AlgPara->heartrate = 1000*60/(10*RR_mean/vaild_rr_num);
    }
//rr calculation breath rate
    int rrLen = 0;
    if(g_BreathRRCnt < BREATH_RATE_ALG_RR_NUM)
    {
        rrLen = g_BreathRRCnt;
    }
    else
    {
        rrLen = BREATH_RATE_ALG_RR_NUM;
    }

    uint8_t breathRate = BREATH_START_VALUE;
    breath_rate_get(g_breath_rate_rr, rrLen,&breathRate);
    AlgPara->breathrate = breathRate;

    static int Procid = 0;
    static AlgData_t ShowSpo2[4];
    static int static_cnt = 0;
    static uint8_t Static_time[10] = { 0 };    
#ifdef FDA_CHECK
    Tcc = 0;
#endif
	if (Tcc >= 10)
	{
		Static_time[static_cnt % 10] = 1;
	}
	else
	{
		Static_time[static_cnt % 10] = 0;
	}
	static_cnt++;
	int tmpstatic_sum = 0;
	for (int cnt = 0; cnt < 10; cnt++)
	{
		tmpstatic_sum += Static_time[cnt];
	}

	if (tmpstatic_sum > 0 && Procid > 0)
	{
		//smooth spo2
		if (ShowSpo2[(Procid - 1) % 4] > AlgPara->Spo2 + 1.7)
		{
			AlgData_t max_show_spo2 = 0;
			ShowSpo2[Procid % 4] = ShowSpo2[(Procid - 1) % 4];
		}
		else
		{
			ShowSpo2[Procid % 4] = AlgPara->Spo2;
		}
	}
	else
	{
		ShowSpo2[Procid % 4] = AlgPara->Spo2;
	}

	Procid++;

	
	int tmpmeanlen = MIN(MAX(Procid,1),4);

	AlgData_t tmpmeanval = 0;
	for (int j = 0; j < tmpmeanlen; j++)
	{
		tmpmeanval += ShowSpo2[j];
	}
	tmpmeanval = tmpmeanval / tmpmeanlen;
	AlgPara->SSpo2 = tmpmeanval;
#endif
    //smooth show hr

    // if(AlgPara->heartrate > 40)
    if(AlgPara->heartrate >= LOWEST_HEART_VALUE)
    {
        if (HistHrsavepos == 0)
        {
            AlgPara->Sheartrate = (AlgPara->Sheartrate + AlgPara->heartrate)/2;
            //HistHeartrate[HistHrsavepos] = AlgPara->heartrate;
            HistHeartrate[HistHrsavepos] = AlgPara->Sheartrate;
        }
        else
        {
                int tmphrmean = 0;
                int tmpcnt1 = 0;
                if (HistHrsavepos < HIST_HR_LEN)
                {
                    for (int i = 0; i < HistHrsavepos; i++)
                    {
                        tmphrmean += HistHeartrate[i];
                        tmpcnt1++;
                    }
                }
                else
                {
                    for (int i = 0; i < HIST_HR_LEN; i++)
                    {
                        tmphrmean += HistHeartrate[i];
                        tmpcnt1++;
                    }
                }

                if (tmpcnt1 == 0)
                {
                    AlgPara->Sheartrate = AlgPara->heartratehist;
                }
                else
                {
                    //tmphrmean = tmphrmean / tmpcnt1;
                    AlgPara->Sheartrate = (AlgPara->heartrate + tmphrmean)/(tmpcnt1 + 1);
                    //if (vldflg < 2 || abs(tmphrmean - AlgPara->heartrate) > 15)
                    HistHeartrate[HistHrsavepos % HIST_HR_LEN] = AlgPara->Sheartrate;
                    HistHrsavepos++;
                    
                }

        }

	}

	// if (HistHrsavepos == 0)
	// {
	// 	if (vldflg <= 2)
	// 	{
	// 		AlgPara->Sheartrate = AlgPara->heartratehist;
	// 	}
	// 	else
	// 	{
	// 		HistHeartrate[HistHrsavepos] = AlgPara->heartrate;
	// 		HistHrVld[HistHrsavepos] = vldflg;
	// 		HistHrsavepos++;
	// 	}
	// }
	// else
	// {
	// 	int tmphrmean = 0;
	// 	int tmpcnt1 = 0;
	// 	if (HistHrsavepos < HIST_HR_LEN)
	// 	{
	// 		for (int i = 0; i < HistHrsavepos; i++)
	// 		{
	// 			tmphrmean += HistHeartrate[i];
	// 			tmpcnt1++;
	// 		}
	// 	}
	// 	else
	// 	{
	// 		for (int i = 0; i < HIST_HR_LEN; i++)
	// 		{
	// 			tmphrmean += HistHeartrate[i];
	// 			tmpcnt1++;
	// 		}
	// 	}

	// 	if (tmpcnt1 == 0)
	// 	{
	// 		AlgPara->Sheartrate = AlgPara->heartratehist;
	// 	}
	// 	else
	// 	{
	// 		tmphrmean = tmphrmean / tmpcnt1;

	// 		//if (vldflg < 2 || abs(tmphrmean - AlgPara->heartrate) > 15)
	// 		if (vldflg < 2)
	// 		{
	// 			AlgPara->Sheartrate = AlgPara->heartratehist;
	// 			}
	// 		else
	// 		{
	// 			HistHeartrate[HistHrsavepos % HIST_HR_LEN] = AlgPara->heartrate;
	// 			HistHrVld[HistHrsavepos % HIST_HR_LEN] = vldflg;
	// 			HistHrsavepos++;
	// 		}
	// 	}
	// }
    
#if Tired_Proc
	int savepos = AlgPara->Proccnt % MAX_HIST_LEN;
	HistSpo2[savepos] = (uint8_t)AlgPara->Spo2;

	uint8_t Tcctmp;
	if (Tcc < 5)
	{
		Tcctmp = 0;
	}
	else
	{
		if (Tcc < 20)
		{
			Tcctmp = 1;
		}
		else
		{
			Tcctmp = 2;
		}
	}
	HistAcc[savepos] = Tcctmp;
	AlgPara->Proccnt++;

	for (int j = 0; j < AlgPara->HrCnt; j++)
	{
		int RRSavepos = HistRRId % MAX_HIST_LEN;
		HistRR[RRSavepos] = AlgPara->HrVect[j];
		HistRRId++;
	}

	AlgPara->tiredflg = get_tiredFlg(AlgPara->Proccnt);
#endif
}

int get_InSleep(uint8_t *Acc, uint8_t *Spo2, int Proccnt)
{
	if (Proccnt < MAX_HIST_LEN)
	{
		return 0;
	}

	int Newpos = Proccnt % MAX_HIST_LEN;

	int Accsum = 0;
	int sumcnt = 0;
	int tmpvldlen = 0;
	int tmpvldlenbck = 0;
	int convldflg = 0;
	for (int i = Newpos; i > 0; i--)
	{
		Accsum += Acc[i];
		if (Acc[i] == 0 && convldflg == 0)
		{
			tmpvldlen++;
		}
		else
		{
			convldflg = 1;
		}

		sumcnt++;
		if (sumcnt > 300)
		{
			break;
		}
	}

	if (sumcnt < 300)
	{
		for (int i = MAX_HIST_LEN - 1; i > Newpos; i--)
		{
			Accsum += Acc[i];

			if (Acc[i] == 0 && convldflg == 0)
			{
				tmpvldlen++;
			}
			else
			{
				convldflg = 1;
			}

			sumcnt++;
			if (sumcnt > 300)
			{
				break;
			}
		}
	}

	if ((Accsum < 60 && tmpvldlen > 60) || Accsum < 50 || tmpvldlen > 150)
	{
		return 1;
	}

	uint8_t maxSpo2 = 0;
	uint8_t minSpo2 = 200;
	for (int i = 0; i < MAX_HIST_LEN; i++)
	{
		if (Spo2[i] > maxSpo2)
		{
			maxSpo2 = Spo2[i];
		}
		if (Spo2[i] < minSpo2 && Acc[i] == 0)
		{
			minSpo2 = Spo2[i];
		}
	}

	maxSpo2 = MIN(maxSpo2, 96);
	if (minSpo2 < maxSpo2 - 5)
	{
		uint8_t threshold = maxSpo2 - 3;

		int eventcnt = 0;
		int eventflg = 0;
		int proclen = 0;
		for (int i = Newpos; i > 0; i--)
		{
			if (Spo2[i] < threshold && Acc[i] == 0)
			{
				eventflg++;
			}
			else
			{
				if (eventflg > 10)
				{
					eventflg = 0;
					eventcnt++;
				}
			}
			proclen++;
		}

		for (int i = MAX_HIST_LEN - 1; i > Newpos; i--)
		{
			if (Spo2[i] < threshold && Acc[i] == 0)
			{
				eventflg++;
			}
			else
			{
				if (eventflg > 10)
				{
					eventflg = 0;
					eventcnt++;
				}
			}
		}

		if (eventcnt > 8 || (Accsum < 100 && eventcnt > 4))
		{
			return 1;  //sleep status
		}
	}
	return 0; // wake status
}

void SPO2_Init_ProcPara(ProcPara *Para, int procsamplerate)
{
	Para->samplerate = procsamplerate;
	arm_rfft_fast_init_f32(&(Para->fftPara), FFT_SPO2_LEN);
}

void fftabs_proc(float* input, int inputlen, int outputlen, float *output)
{
	uint32_t ifftFlag = 0;

	arm_rfft_fast_f32(&SPO2ProcPara.fftPara, input, output, ifftFlag);

	output[1] = 0;
    for (int i = 0; i < outputlen; i++)
	{
		output[i] = complex_pow(output + 2 * i);
	}
}

int get_tiredFlg(int Proccnt)
{
	static int Tiredflg = 0;
	uint8_t *Acc = HistAcc;
    NRF_LOG_INFO("Proccnt = %d\r\n",Proccnt);
	if (Proccnt < MAX_HIST_LEN / 2)
	{
		return 0;
	}

	int Newpos = Proccnt % MAX_HIST_LEN;

	float Accsum = 0;
	if (Newpos < 60)
	{
		for (int i = 0; i < Newpos; i++)
		{
			Accsum += Acc[i];
		}
		for (int i = MAX_HIST_LEN - (60 - Newpos); i < MAX_HIST_LEN; i++)
		{
			Accsum += Acc[i];
		}
	}
	else
	{
		for (int i = Newpos - 60; i < Newpos; i++)
		{
			Accsum += Acc[i];
		}
	}

	float Accnewmean = (float)Accsum / 60;
	Accsum = 0;
	int tvldcnt = MIN(Proccnt, MAX_HIST_LEN);
	for (int i = 0; i < MIN(Proccnt,MAX_HIST_LEN); i++)
	{
		Accsum += Acc[i];
	}
	float Accmean = (float)Accsum / tvldcnt;

	if (Accnewmean > MAX(10, Accmean) && Tiredflg == 0)
	{
		return Tiredflg;
	}

	uint8_t *Spo2 = HistSpo2;
	float Spo2sum = 0;
	for (int i = 0; i < tvldcnt; i++)
	{
		Spo2sum += Spo2[i];
	}
	float Spo2mean = (float)Spo2sum / tvldcnt;


	Spo2sum = 0;
	if (Newpos < 60)
	{
		for (int i = 0; i < Newpos; i++)
		{
			Spo2sum += Spo2[i];
		}
		for (int i = MAX_HIST_LEN - (60 - Newpos); i < MAX_HIST_LEN; i++)
		{
			Spo2sum += Spo2[i];
		}
	}
	else
	{
		for (int i = Newpos - 60; i < Newpos; i++)
		{
			Spo2sum += Spo2[i];
		}
	}

	float Spo2newmean = (float)Spo2sum / 60;


	uint8_t *RR = HistRR;
	float RRsum = 0;
	int RRProcLen = MIN(MAX_HIST_LEN, HistRRId);
	for (int i = 0; i < RRProcLen; i++)
	{
		RRsum += RR[i];
	}

	float RRmean = RRsum / RRProcLen;

	RRsum = 0;
	int NewposRR = HistRRId % MAX_HIST_LEN;
	if (NewposRR < 60)
	{
		for (int i = 0; i < NewposRR; i++)
		{
			RRsum += RR[i];
		}
		for (int i = MAX_HIST_LEN - (60 - NewposRR); i < MAX_HIST_LEN; i++)
		{
			RRsum += RR[i];
		}
	}
	else
	{
		for (int i = NewposRR - 60; i < NewposRR; i++)
		{
			RRsum += RR[i];
		}
	}

	float RRnewmean = RRsum / 60;


	//calc HRV fft
	if (HistRRId < 512)
	{
		return 0;
	}

	int sp = (HistRRId - 512) % MAX_HIST_LEN;
	if (HistRRId < MAX_HIST_LEN)
	{
		for (int i = sp, j = 0; i < HistRRId; i++, j++)
		{
			SPO2_FFT_In[j] = HistRR[i];
		}
	}
	else
	{
		int j = 0;
		for (int i = sp; i < MAX_HIST_LEN; i++)
		{
			SPO2_FFT_In[j++] = HistRR[i];
		}

		for (int i = 0; i < (HistRRId % MAX_HIST_LEN); i++)
		{
			SPO2_FFT_In[j++] = HistRR[i];
		}
	}

//	fftabs_proc(FFT_In_buf, 512, FFT_Out_buf, 180);
    fftabs_proc(SPO2_FFT_In, 512, 200, SPO2_FFT_Out);
    
	float maxpwr = 0;
	int maxpwrid = 0;
	for (int i = 10; i < 200; i++)
	{
		if (SPO2_FFT_Out[i] > maxpwr)
		{
			maxpwr = SPO2_FFT_Out[i];
			maxpwrid = i;
		}
	}

	int Omaxpwrid = maxpwrid;

	if (maxpwrid < 50)
	{
		maxpwr = 0;
		maxpwrid = 0;
		for (int i = 50; i < 200; i++)
		{
			if (SPO2_FFT_Out[i] > maxpwr)
			{
				maxpwr = SPO2_FFT_Out[i];
				maxpwrid = i;
			}
		}
	}

	int sp1 = MAX(10, maxpwrid - 50);
	int ep1 = MAX(200, maxpwrid - 10);

	int sp2 = MIN(200, maxpwrid + 10);
	int ep2 = MIN(200, maxpwrid + 50);

	float meanpwr = 0;
	for (int i = sp1; i < ep1; i++)
	{
		meanpwr += SPO2_FFT_Out[i];
	}

	for (int i = sp2; i < ep2; i++)
	{
		meanpwr += SPO2_FFT_Out[i];
	}

	meanpwr = meanpwr / ((ep1 - sp1) + (ep2 - sp2));

	float Pmrate = maxpwr / meanpwr;

	HistHRVVect[HistHRVId % MAX_HIST_LEN] = maxpwr;
	HistHRVMVect[HistHRVId % MAX_HIST_LEN] = maxpwrid;
	HistHRVId++;

	if (RRnewmean < RRmean && Spo2newmean > Spo2mean && Tiredflg == 0)
	{
		return Tiredflg;
	}

	if (Omaxpwrid > 50 && Omaxpwrid < 180 && Pmrate > 8)
	{
		Tiredflg = 3;
	}
    else
	{
		if (RRnewmean > RRmean && ((Omaxpwrid > 50 && Omaxpwrid < 180 && Pmrate > 6) || (maxpwrid < 180 && Pmrate > 8)))
		{
			Tiredflg = 2;
		}
		else
		{
			if (RRnewmean > RRmean && ((Omaxpwrid > 50 && Omaxpwrid < 180 && Pmrate > 3) || (maxpwrid < 180 && Pmrate > 5)))
			{
				Tiredflg = 1;
			}
			else
			{
				Tiredflg = 0;
			}
		}
	}
    
	return Tiredflg;

}

int get_int_mean(int* input, int inlen)
{
	if (inlen <= 0)
	{
		return 0;
	}
	int sum = 0;
	for (int i = 0; i < inlen; i++)
	{
		sum += input[i];
	}
	return sum / inlen;
}

uint8_t data_compress16(int src_data,uint8_t flag,uint8_t flag_spo2)
{
	uint8_t dst_data;
	if(flag > 0)
	{
		flag = 128;
	}
    int tmp_data = src_data >> 2;
//    if(src_data > 40 && src_data <= 160)
//    {
//        tmp_data = (src_data >> 3) + 5;
//    }
//    else if(src_data > 160)
//    {
//        tmp_data = (src_data >> 4) + 15;
//    }
	if(src_data >= 252)
	{
		dst_data = 63 + flag;
	}
	else
	{
		dst_data = tmp_data + flag;
	}
    
    if(flag_spo2 > 0)
    {
        dst_data = dst_data | 0x40;
    }
	return dst_data;
}

int offline_proc(int* ledamb, int len)
{
    int isoffline = 0;
    int offline_num = 0;
    for(int i = 0; i < len; i++)
    {
        isoffline = Detect_ohhand(ledamb[i]);
        if(isoffline == 1)
        {
            offline_num++;
        }
    }
    if(offline_num >= 4)
    {
        return 1;        // is off hand
    }
    return 0;            // is on hand
}

int Detect_offline_fifo(ring_afe_mode_t curMode)
{
    // if(dt_ofline.offline_num >= 2)                   // is off hand
    if(dt_ofline.offline_num >= 3)                   // is off hand
    {
        set_afe_reg(0x22, 0x000000);// turn off led
        set_afe_reg(0x20, 0x008013);// init gain
        set_afe_reg(0x3A, 0x000000);// init offdac
        set_afe_reg(0x3E, 0x000000);// init offdac
        ring_topled_off();                          // turn off top led
        offline_ledflg = OFFLINE_LED_OFF;
        g_afe.flag = SPO2_HR_INVALID;
        g_afe.offhand_flg = 1;
		daily_sleep_offhand_flag_set(g_afe.offhand_flg);
    }
    else                                            // is on hand
    {
        ring_topled_off();                           // turn off top led
        offline_ledflg = OFFLINE_LED_OFF;
        if(g_afe.offhand_flg == 1)
        {
            if(curMode == RING_AFE_SPO2_MODE)
            {
                memset(spo2_red, 0, length_rawdata*sizeof(int));
                memset(spo2_infra, 0, length_rawdata*sizeof(int));
                afeproc_init();

                if (RING_AFE_HRV_ON_MODE == (RING_AFE_HRV_ON_MODE & afe_get_work_mode())) {
                        uint8_t Regret = currentReg(10*4096 + 10*64 + 3);
                } else {
                        uint8_t Regret = currentReg(10*4096 + 10*64);
                }
            }
            else
            {
                memset(daily_green, 0, length_rawdata*sizeof(int));
                afeproc_init();
                //dailyproc_init();
                uint8_t Regret = currentReg(3);
            }
            startchange = 1;
            g_afe.flag = SPO2_HR_READYING;
            g_afe.offhand_flg = 0;
        }
		daily_sleep_offhand_flag_set(g_afe.offhand_flg);
    }
    dt_ofline.offline_num = 0;
//    dt_ofline.online_num = 0;
    Moveflg = g_afe.offhand_flg;
	uint8_t spo2_flag = accval & 0x40;
			
    accval = data_compress16(0, Moveflg, spo2_flag);
	//NRF_LOG_INFO("Detect_offline_fifo=========================Moveflg  = %d", Moveflg);
}

int Detect_ohhand(int LED1val)
{
    if(LED1val > OFFLINE_THRE)
    {
        return 1;
    }
    return 0;
}

uint8_t top_led_status(void)
{
    return offline_ledflg;
}

void off_dac_set_circle(void)
{
    if(offdac_led2 <= 127 && offdac_led3 <= 127 && offdac_led1 <= 127)
    {
        uint8_t offdac_led2_msb = (offdac_led2 & 0x40) >> 6;
        uint8_t offdac_led2_mid = (offdac_led2 & 0x3C) >> 2;
        uint8_t offdac_led2_lsb = (offdac_led2 & 0x02) >> 1;
        uint8_t offdac_led2_lsbext = offdac_led2 & 0x01;
        uint8_t offdac_led3_msb = (offdac_led3 & 0x40) >> 6;
        uint8_t offdac_led3_mid = (offdac_led3 & 0x3C) >> 2;
        uint8_t offdac_led3_lsb = (offdac_led3 & 0x02) >> 1;
        uint8_t offdac_led3_lsbext = offdac_led3 & 0x01;
        uint32_t regdata3A = 0x000000|(1 << 19)|(offdac_led2_mid << 15)|(1 << 4)|(offdac_led3_mid & 0x0f)|(1 << 20);
        uint32_t regdata3E = 0x000000|(offdac_led2_msb << 7)|(offdac_led2_lsb << 6)|(offdac_led3_msb << 1)|(offdac_led3_lsb)|(offdac_led2_lsbext << 11)|(offdac_led3_lsbext << 8);
        set_afe_reg(0x3A, regdata3A);
        set_afe_reg(0x3E, regdata3E);
    }
    else
    {
        //offdac_led2 = 0;
        //offdac_led3 = 0;
    } 
}
#if 0
void off_dac_set(int addval2, int addval3)
{
	offdac_led2 = offdac_led2 + addval2;
	offdac_led3 = offdac_led3 + addval3;
	if(offdac_led2 == 0 || offdac_led3 == 0)
	{
		return;
	}
	if(offdac_led2 < 127 && offdac_led3 < 127)
	{
		uint8_t offdac_led2_msb = offdac_led2 >> 6;
		uint8_t offdac_led2_mid = (offdac_led2 & 0x3C) >> 2;
		uint8_t offdac_led2_lsb = (offdac_led2 & 0x02) >> 1;
		uint8_t offdac_led3_msb = offdac_led3 >> 6;
		uint8_t offdac_led3_mid = (offdac_led3 & 0x3C) >> 2;
		uint8_t offdac_led3_lsb = (offdac_led3 & 0x02) >> 1;
		uint32_t regdata3A = 0x000000|(1 << 19)|((offdac_led2_mid & 0x0f) << 15)|(1 << 4)|(offdac_led3_mid & 0x0f)|(1 << 20);
		uint32_t regdata3E = 0x000000|(offdac_led2_msb << 7)|(offdac_led2_lsb << 6)|(offdac_led3_msb << 1)|(offdac_led3_lsb);
		set_afe_reg(0x3A, regdata3A);
		set_afe_reg(0x3E, regdata3E);
	}
	if(offdac_led2 >= 127)
	{
		offdac_led2 = 127;
	}
	if(offdac_led3 >= 127)
	{
		offdac_led3 = 127;
	}
}
#endif

#define MAX_CUR_VAL    12
/*********************************************************************
* @fn     SPO2_bufProc_wavelet()
*
* @brief   get raw date ready and pre_filter.
*   update after filter;
* @param   none
* @return  flag ready to calculate.
*/
int test_greenval;
uint8_t red_regval;
uint8_t infra_regval;
int thres_max_led = 1.5e6;
int tmp_breathratebuf[10] = {0};
extern FIFO32 green_health_fifo;
uint8_t  SPO2_bufProc_wavelet(void *afe_in, uint8_t frame_length)
{
    if(count_spo2_bufproc < max_bufproc_num)
    {
            count_spo2_bufproc++;//spo2 rawdata calculate num
    }
    if(!(changecnt%(100/frame_length)))
    {
            num_calculation++;
    }	
	
    int tmpledamb = 0;
    int tmp_Infra_orig = 0;
    int tmp_Red_orig = 0;
    uint8_t Regret = 0;
    uint8_t Regret2 = 0;
    struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)afe_in;
    dt_ofline.min_ambval = afe_bpt[0].phase4;
    int tmpledInfra;
    int tmpledRed;
    int tmpledgreen;
    float green_de_ambient = 0;
    for(int i = 0; i < frame_length; i++)
    {
        tmp_Infra_orig = afe_bpt[i].phase1;
        tmp_Red_orig = afe_bpt[i].phase2;
        tmpledInfra = tmp_Infra_orig;//tmp_Infra_orig - tmpledamb;
        tmpledRed = tmp_Red_orig;//tmp_Red_orig - tmpledamb;
        tmpledgreen = afe_bpt[i].phase3;//green or ambient
        tmpledamb = afe_bpt[i].phase4;

		//HRV heart and RR get
        green_de_ambient = tmpledgreen - afe_bpt[i].phase4;
		fifo32_putPut(&green_health_fifo, (int)(IIR_bandpass_chebyshev_Filter_heart(green_de_ambient)));
		//HRV_health_data_get();
		//HRV_health_data_get_new();


        //tmpledInfra = (tmp_Infra_orig + offdc_deta*offdac_led2 - (offdc_deta/5)*tmpledamb) * 0.1;
        //tmpledRed = (tmp_Red_orig + offdc_deta*offdac_led3 - (offdc_deta/5)*tmpledamb) * 0.1;

        
        tmpledInfra = (tmp_Infra_orig + offdc_deta*offdac_led2 - 4*tmpledamb) * 0.1;
        tmpledRed = (tmp_Red_orig + offdc_deta*offdac_led3 - 4*tmpledamb) * 0.1;

        tmpledInfra = (int)smoothFilter1(afe_IIRLPFilter1(tmpledInfra));
        tmpledRed = (int)smoothFilter2(afe_IIRLPFilter2(tmpledRed));

        int tmpfilter_red = (int)spo2_iirbpfilter(tmp_Red_orig, spo2_filter_x1, spo2_filter_y1);
        int tmpfilter_infra = (int)spo2_iirbpfilter(tmp_Infra_orig, spo2_filter_x2, spo2_filter_y2);

        // int tmpfilter_red = tmpledRed;
        // int tmpfilter_infra = tmpledInfra;

        // if (RING_AFE_HRV_ON_MODE & afe_get_work_mode()) {
        //         tmpledgreen = (int)smoothFilter3(afe_IIRLPFilter3(tmpledgreen));
        // }
        tmpledgreen = (int)IIR_bandpass_chebyshev_Filter_heart_sport(green_de_ambient);
		
        AFE44xx_SPO2_Data_buf[0] = tmpledInfra;
        AFE44xx_SPO2_Data_buf[1] = tmpledRed;

        dt_ofline.min_ambval = min(dt_ofline.min_ambval, tmpledamb);
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 2)
            {
                if(tmpledamb > (afe_bpt[i - 1].phase4 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase4 + OFFLINE_THRE_TWO))
                {
                    dt_ofline.offline_num++;
                }
                else
                {
                    if(tmpledamb > (afe_bpt[i - 1].phase4 + OFFLINE_THRE) && afe_bpt[i + 1].phase4 > (afe_bpt[i + 2].phase4 + OFFLINE_THRE_TWO))
                    {
                        dt_ofline.offline_num++;
                    }
                }

            }
//            dt_ofline.is_offline = Detect_ohhand(tmpledamb - dt_ofline.min_ambval);
//            if(dt_ofline.is_offline == 1)  // is off hand
//            {
//                dt_ofline.offline_num++;
//            }
        }        
        afe_AddRawdata_wavelet(AFE44xx_SPO2_Data_buf[1], AFE44xx_SPO2_Data_buf[0]);
        fifo32_putPut(&spo2_green_fifo, tmpledgreen);
        fifo32_putPut(&spo2_red_filter, tmpfilter_red);
		fifo32_putPut(&spo2_infra_filter, tmpfilter_infra);
        
        tmp_breathratebuf[tmpbr_datano%10] = AFE44xx_SPO2_Data_buf[0];
        tmpbr_datano++;
        if(!(tmpbr_datano%10))
        {
                //tmpgreen_datano = 0;
                int green_mean = get_int_mean(tmp_breathratebuf, 10);
                
                fifo32_putPut(&BREATH_fifo, green_mean);
        }
//		afe_raw_send->recipes[i].ledInfra = tmpledInfra;
//		afe_raw_send->recipes[i].ledRed = tmpledRed;		
    }
	
//off hand detect
#ifndef FIXED_CURRENT
#if ONHAND_Spo2_Proc
    //if(!(changecnt%(20/frame_length)))
    //{
		if(offline_ledflg == OFFLINE_LED_ON && changecnt > 6)  //when top green led is on, do offhand detect
        {
            //get_afe_reg(0x22, &reg22val);
            //int greenlowreg = reg22val >> 18;
            //int tmpredreg = (reg22val - (greenlowreg << 18)) >> 12;
            Detect_offline_fifo(RING_AFE_SPO2_MODE);
        }

        int change_step = Spo2_ONHAND_INTERVAL/frame_length;
//            if(state_entry.afe_who == AFE_RTSHOW) {
//                change_step = 300/AFE4900_ARRAY_SIZE;
//            }
        if(!(changecnt%change_step))   //turn on top green led
        {
            if(offline_ledflg == OFFLINE_LED_OFF)
            {
                ring_topled_on();
                offline_ledflg = OFFLINE_LED_ON;
            }
        }
    
#endif
#endif
	
    changecnt++;
    //if(offdac_led2 != 10)
    //{
    //    offdac_led2 = 10;
    //    offdac_led3 = 10;
    //    off_dac_set_circle();
    //}
    //if(!(changecnt%(50/frame_length)))
    //{
    //    offdac_led2++;
    //    offdac_led3++;
    //    off_dac_set_circle();
    //}

#if 1
#ifndef FIXED_CURRENT
    if(g_afe.offhand_flg == 0)
    { 
        if(!(changecnt%(100/frame_length)))
        {	
            //NRF_LOG_INFO("RED DATA : %d; INFRA DATA : %d; length: %d", tmp_Red_orig,tmp_Infra_orig,frame_length);
            get_afe_reg(0x22,&reg22val);
            if (RING_AFE_HRV_ON_MODE & afe_get_work_mode()) {
                if(g_afe.offhand_flg == 0)
                {	
                    set_afe_reg(0x22,reg22val | 3);
                }
                get_afe_reg(0x22,&reg22val);
            } else {
                if(reg22val & 0x03)
                {
                    set_afe_reg(0x22,(reg22val - 3));
                    get_afe_reg(0x22,&reg22val);
                }
            }
        
            int redreg;
            int infrareg;
            int greenlowreg;
            greenlowreg = reg22val >> 18;
            redreg = (reg22val - (greenlowreg << 18)) >> 12;
            infrareg = (reg22val - (greenlowreg << 18) - (redreg << 12)) >> 6;
            red_regval = redreg;
            infra_regval = infrareg;


#if 1
            if(dc_adj.current_flg == 0 && changecnt > 10)
            {
                if(redreg >= MAX_CUR_VAL || infrareg >= MAX_CUR_VAL)
                {
                    dc_adj.current_flg = 1;
                    //Regret = currentReg(4096 + 64);
                }
                int meansp = fifo32_status(&spo2_red_fifo) - test_data_num;
                int meanredpwr = get_int_mean(spo2_red + meansp, test_data_num);
                int meaninfrapwr = get_int_mean(spo2_infra + meansp, test_data_num);
                float meandeta = 0;
                int deta_red_infra = tmp_Infra_orig - tmp_Red_orig; //infra - red
                //int infrastep;
                //int tmpthre;
                //int redthre;
        
                if(meandatanum < MeanDataNUM - 1)
                {
                    meanredpwrbuf[meandatanum] = meanredpwr;
                    meaninfrapwrbuf[meandatanum] = meaninfrapwr;
                    for(int i = 0; i < meandatanum; i++)
                    {
                        meanredpwr += meanredpwrbuf[i];
                        meaninfrapwr += meaninfrapwrbuf[i];
                    }
                    meandatanum++;
                    meandeta = meaninfrapwr/meandatanum - meanredpwr/meandatanum;
                }
                else
                {
                    meanredpwrbuf[MeanDataNUM - 1] = meanredpwr;
                    meaninfrapwrbuf[MeanDataNUM - 1] = meaninfrapwr;
                    for(int i = 0; i < MeanDataNUM - 1; i++)
                    {
                        meanredpwr += meanredpwrbuf[i];
                        meaninfrapwr += meaninfrapwrbuf[i];
                    }
                    meandeta = meaninfrapwr/MeanDataNUM - meanredpwr/MeanDataNUM;
                    memcpy(meanredpwrbuf, meanredpwrbuf + 1, sizeof(int)*(MeanDataNUM - 1));
                    memcpy(meaninfrapwrbuf, meaninfrapwrbuf + 1, sizeof(int)*(MeanDataNUM - 1));
                }
                if(cuaddval != 0) // culculate onestep
                {
                    int custep = abs(cuaddval) >> 6;
                    onestep = abs(tmp_Infra_orig - preredval)/custep;
                    cuaddval = 0;
                    //infrastep = abs(infraN);
                    //oneinfrastep = abs(tmp_Infra_orig - preinfraval)/infrastep;
                }
                //tmpthre = (int)(0.2 * AFE44xx_SPO2_Data_buf[0]);
                //redthre = max(onestep,tmpthre);
                //if(startchange > 0 && g_afe.flag == SPO2_HR_VALID)
                //{
                //    g_afe.flag = SPO2_HR_READYING;
                //}
			
                if(redreg == 0)
                {
                    Regret = currentReg(4096 + 64);
                    startchange = 1;
                    //testcu = 25;
                    g_afe.flag = SPO2_HR_READYING;
                }
                //else if(AFE44xx_SPO2_Data_buf[0] < infrathreslow && infrareg < 20)
                else if(tmp_Red_orig < infrathreslow && redreg < MAX_CUR_VAL)
                {
                    //infraN = (infrathreslow - AFE44xx_SPO2_Data_buf[0])/oneinfrastep;
                    Regret = currentReg(4096);
                    startchange = 1;
                    //testcu = 1;
                }
                else if(tmp_Red_orig > 2e6 && redreg >= MAX_CUR_VAL)
                {
                    Regret = currentReg(-4096*6);
                    startchange = 1;
                    //testcu = 1;
                }
                else if(tmp_Red_orig > infrathreshigh)
                {
                    if(redreg >= MAX_CUR_VAL)
                    {
                        Regret = currentReg(-4096*3);
                        startchange = 1;
                    }
                    else
                    {
                        Regret = currentReg(-4096);
                        startchange = 1;
                    }
                    //testcu = 1;
                }

                if(startchange == 1 && cuchangeflag == 1)
                {
                    cuchangeflag = 0;
                }
                //change infra 			
                if(tmp_Infra_orig < infrathreslow && tmp_Red_orig > infrathreslow && infrareg < MAX_CUR_VAL)
                {
                    Regret2 = currentReg(64);
                    cuaddval = 64;
                }
                else
                {
                    if(abs(deta_red_infra) > 3*onestep)
                    {
                        if(deta_red_infra > 0)
                        {
                            Regret2 = currentReg(-64*4);
                            cuaddval = -64*4;
                        }
                        else
                        {
                            Regret2 = currentReg(64*4);
                            cuaddval = 64*4;
                        }
                        startchange = 2;
            
                    }
                    else if(abs(deta_red_infra) > 2*abs(onestep))
                    {	
                        //redN = abs(deta_red_infra)/onestep;
                        //cuaddval = abs(deta_red_infra)/onestep;
                        if(deta_red_infra > 0)
                        {			
                            Regret2 = currentReg(-64*2);
                            cuaddval = -64*2;
                            //prelowerredval = AFE44xx_SPO2_Data_buf[1];
                        }
                        else
                        {
                            Regret2 = currentReg(64*2);
                            cuaddval = 64*2;
                        }
                        startchange = 2;
                    }
                    else if(abs(deta_red_infra) > onestep) //平静
                    {
                        if(cuchangeflag == 1)
                        {
                            redlowernum++;
                            if(redlowernum >= 600 && fabs(meandeta) > 0.8*onestep)
                            {
                                startchange = 2;
                                redlowernum = 0;
                                if(deta_red_infra > 0)
                                {
                                    Regret2 = currentReg(-64);
                                    //startchange = 2;
                                    cuaddval = -64;
                                }
                                else
                                {
                                    Regret2 = currentReg(64);
                                    cuaddval = 64;
                                }					
                            }
                        }
                        else
                        {
                            if(deta_red_infra > 0)
                            {
                                Regret2 = currentReg(-64);
                                cuaddval = -64;
                                //prelowerredval = AFE44xx_SPO2_Data_buf[1];
                            }
                            else
                            {
                                Regret2 = currentReg(64);
                                cuaddval = 64;
                            }
                            startchange = 2;
                        }
                    }
                    else		//first change has been done	
                    {
                        redlowernum = 0;
                        if(startchange == 2)
                        {
                            startchange = 0;
                            cuaddval = 0;
                        }
                    }
                }
                //判断初始电流是否调整完毕
                //if((AFE44xx_SPO2_Data_buf[0] >= infrathreslow && AFE44xx_SPO2_Data_buf[0] <= infrathreshigh) || infrareg == 20)
                if((tmp_Red_orig >= infrathreslow && tmp_Red_orig <= infrathreshigh) || redreg == infrathreslow)
                {
                    if(tmp_Infra_orig > 2e6 && cuaddval >= 0)
                    {
                        Regret2 = currentReg(-64);
                        cuaddval = cuaddval -64;
                    }
                    if(cuchangeflag == 0 || cuinitflag == 0)
                    {
                        if(redreg >= infrathreslow)
                        {
                            if(abs(deta_red_infra) < onestep  || infrareg >= infrathreslow)
                            {
                                cuchangeflag = 1;
                                if(cuinitflag == 0 && g_afe.flag != SPO2_HR_VALID)
                                {
                                    initdonelength++;
                                }
                                else
                                    initdonelength = 0;
                            }
                        }
                        else
                        {
                            if(abs(deta_red_infra) < onestep || infrareg >= infrathreslow)
                            {
                                dc_adj.current_flg = 1;
                                cuchangeflag = 1;
                                if(cuinitflag == 0 && g_afe.flag != SPO2_HR_VALID)
                                {
                                    initdonelength++;
                                }
                                else
                                    initdonelength = 0;
                            }
                        }
                    }
                }
				
                if(initdonelength >= 10 && cuinitflag == 0)
                {
                    cuinitflag = 1;
                    initdonelength = 0;
                }
                preredval = tmp_Infra_orig;
            }

#endif
#if 1
            else if (dc_adj.current_flg == 1)
            {
                if(!(changecnt%(50/frame_length)))
                {	
                    if(dc_adj.offdac_flg == 0)
                    {	
                        if(changecnt > 50)
                        {
                            int offdc_50k_deta = 20800;
                            offdac_led2 = tmp_Infra_orig/offdc_50k_deta + 1;
                            offdac_led3 = tmp_Red_orig/offdc_50k_deta + 1;
                            offdac_led2 = max(offdac_led2,offdac_led3);
                            offdac_led3 = max(offdac_led2,offdac_led3);
                            off_dac_set_circle();
                            dc_adj.offdac_flg = 1;
                        }
                    }
                    else if(offdc_gain_flg == 0)
                    {
                        if(changecnt > 60)
                        {	
                            set_afe_reg(0x20, 0x008016);
                            //set_afe_reg(0x20, 0x008017);
                            offdc_gain_flg = 1;
                        }
                    }
                    if(changecnt > 65)
                    {
                        int offdc_500k_deta = 4.15e5;
                        if(tmp_Infra_orig > 1.8e6)
                        {
                            offdac_led2 = offdac_led2 + tmp_Infra_orig/offdc_500k_deta + 1;
                            off_dac_set_circle();
                        }
                        if(tmp_Red_orig > 1.8e6)
                        {
                            offdac_led3 = offdac_led3 + tmp_Red_orig/offdc_500k_deta + 1;
                            off_dac_set_circle();
                        }
                        if(tmp_Infra_orig < -1.8e6)
                        {
                            offdac_led2 = offdac_led2 - abs(tmp_Infra_orig/offdc_500k_deta);
                            off_dac_set_circle();
                        }
                        if(tmp_Red_orig < -1.8e6)
                        {
                            offdac_led3 = offdac_led3 - abs(tmp_Red_orig/offdc_500k_deta);
                            off_dac_set_circle();
                        }
                    }	
                    if(changecnt > 200)
                    {
                        if(offdac_led2 > 127 || offdac_led3 > 127)
                        {
                            afeproc_init();
                            set_afe_reg(0x22, 0x000000|(2 << 12)|(2 << 6));// init led
                            set_afe_reg(0x20, 0x008013);// init gain
                            set_afe_reg(0x3A, 0x000000);// init offdac
                            set_afe_reg(0x3E, 0x000000);// init offdac
                        }
                    }

                }
            }
#endif
            uint8_t testred = redreg;
            uint8_t testinfra = infrareg;
            led_reg.red_reg = testred;
            led_reg.infra_reg = testinfra;
            //testambient = reg22val - (redreg << 12) - (infrareg << 6);
            //ledamb = afe_ReadReg(0x2C);
            //ble_send_persecond();
        }
    }
    //changecnt++;
#endif   
#endif
    if (fifo32_status(&BREATH_fifo) >= MAXRESAMPLENUM)
    {
        breath_flg = 1;
    }
    
    if (fifo32_status(&spo2_red_fifo)>=length_rawdata)
    {
        LED1readyfor_calc = 1;
    }	
    if (fifo32_status(&spo2_infra_fifo)>=length_rawdata)
    {
        LED2readyfor_calc = 1;      
    }	
    if(LED1readyfor_calc==1 && LED2readyfor_calc == 1 )
    {
        //    int redreg;
        //    int infrareg;
        //    int greenlowreg;
        //    greenlowreg = reg22val >> 18;
        //    redreg = (reg22val - (greenlowreg << 18)) >> 12;
        //    infrareg = (reg22val - (greenlowreg << 18) - (redreg << 12)) >> 6;
        //    led_reg.red_reg = redreg;
        //    led_reg.infra_reg = infrareg;
        ble_send_persecond();
        return 1;	//data ready .ready to calculate the result for SPO2 and PR;
            //go to ====>static void spo2_process_wavlet(void* pText)
    }     
    return 0;  
}

#if 0
/*********************************************************************
* @fn      ble_send_afe_raw_data
*
* @brief   send raw data through ble notify chanel.
*
* @param   ...
*
* @return  none
* 
*/
int test_accval = 0;
void ble_send_afe_raw_data(int infradata[], int reddata[], int num)
{
	static int cnt = 0;
	uint8_t frame_buf[20];
	frame_buf[0] = SPO2_WORD;
	if (num == 1) {
		frame_buf[1] = 6;
	} else {
		frame_buf[1] = 12;
	}
	frame_buf[2] = (char)(reddata[0] >> 16);
	frame_buf[3] = (char)(reddata[0] >> 8);
	frame_buf[4] = (char)(reddata[0]);
	frame_buf[5] = (char)(infradata[0] >> 16);
	frame_buf[6] = (char)(infradata[0] >> 8);
	frame_buf[7] = (char)(infradata[0]);

	if (num == 2) {
		frame_buf[8] = (char)(reddata[1] >> 16);
		frame_buf[9] = (char)(reddata[1] >> 8);
		frame_buf[10] = (char)(reddata[1]);
		frame_buf[11] = (char)(infradata[1] >> 16);
		frame_buf[12] = (char)(infradata[1] >> 8);
		frame_buf[13] = (char)(infradata[1]);
	} else {
		frame_buf[8] = led_reg.infra_reg; //extend write
		frame_buf[9] = led_reg.red_reg; //extend write
		frame_buf[10] = g_afe.flag; //extend write
		frame_buf[11] = g_afe.shr;//red_detec.offhand; //extend write
		frame_buf[12] = g_afe.hr; //extend write
		frame_buf[13] = AlgPara.Spo2; //extend write
	}

	frame_buf[14] = (char)(test_accval >> 8);
	frame_buf[15] = (char)(test_accval);//acc_para.stdaccval
	
	frame_buf[16] = led_reg.infra_reg;
	frame_buf[17] = led_reg.red_reg;
	frame_buf[18] = cnt >> 8;
	frame_buf[19] = cnt & 0xff;
	cnt++;
	BLE_RAW_BLOCK_SEND(frame_buf, 20);
}

void ble_send_persecond(void)
{
    uint8_t frame_buf[20];
	frame_buf[0] = SPO2_WORD;
    frame_buf[1] = 18;
    
    frame_buf[2] = led_reg.infra_reg; //extend write
    frame_buf[3] = led_reg.red_reg; //extend write
    frame_buf[4] = g_afe.flag; //extend write
    frame_buf[5] = g_afe.shr;//red_detec.offhand; //extend write
    frame_buf[6] = g_afe.hr; //extend write
    frame_buf[7] = AlgPara.Spo2; //extend write
//    frame_buf[8] = test_Tcc;
//    frame_buf[9] = test_Tcc_sum;
    frame_buf[10] = g_afe.spo2;

	frame_buf[8] = totalHrCnt>>8;
	frame_buf[9] = totalHrCnt&0xFF;
	
    BLE_RAW_BLOCK_SEND(frame_buf, 20);
}
#endif

/*********************************************************************
* @fn      afe_malloc
*
* @brief   malloc deap data for spo2 process.
*
* @param   ...
*
* @return  0
* 
*/
int afe_spo2_malloc(void)
{
    nrf_mem_init();
	// (1050+1050+1000+100)*4 = 12640, 13K
	spo2_red = (int*)nrf_malloc(length_rawdata*sizeof(int));
	spo2_infra = (int*)nrf_malloc(length_rawdata*sizeof(int));
	spo2_green = (int*)nrf_malloc(length_rawdata*sizeof(int));
	
	wrcoefbuf = (int*)nrf_malloc(length_wavelet*sizeof(int));
	AccData = (int*)nrf_malloc(length_accval*sizeof(int));
    HistSpo2 = (uint8_t*)nrf_malloc(MAX_HIST_LEN*sizeof(uint8_t));
    HistAcc = (uint8_t*)nrf_malloc(MAX_HIST_LEN*sizeof(uint8_t));
    Acc_Z_Buf = (int*)nrf_malloc(ACC_Z_LENGTH*sizeof(int));
    
    meanredpwrbuf = (int*)nrf_malloc(MeanDataNUM*sizeof(int));
    meaninfrapwrbuf = (int*)nrf_malloc(MeanDataNUM*sizeof(int));
    zerosVect = (int*)nrf_malloc(MAX_ZEROS_NUM*sizeof(int));

    HistHrVld = (uint8_t*)nrf_malloc(HIST_HR_LEN*sizeof(uint8_t));    

//tired
#if Tired_Proc
    SPO2_FFT_In = (float*)nrf_malloc((FFT_SPO2_LEN + 10)*sizeof(float));
    SPO2_FFT_Out = (float*)nrf_malloc((FFT_SPO2_LEN + 10)*sizeof(float));
    HistRR = (uint8_t*)nrf_malloc(MAX_HIST_LEN*sizeof(uint8_t));
    HistHRVVect = (float*)nrf_malloc(MAX_HIST_LEN*sizeof(float));
    HistHRVMVect = (uint8_t*)nrf_malloc(MAX_HIST_LEN*sizeof(uint8_t));
    HistHrVect = (uint8_t*)nrf_malloc((FFT_SPO2_LEN + 60)*sizeof(uint8_t));
#endif

//add 21.9.1
    wrcoefbuf_base = (int*)nrf_malloc(WAVE_PROC_LEN*sizeof(int));
	
//add 21.11.11
	acc_mean_buf = (int*)nrf_malloc(MAXNUM_HRVvect*sizeof(int));
	
//add 22.12.23
	BREATH_BUF = (int*)nrf_malloc(MAXRESAMPLENUM*sizeof(int));
//add 23.7.12
    red_filter = (int*)nrf_malloc(length_wavelet*sizeof(int));
    infra_filter = (int*)nrf_malloc(length_wavelet*sizeof(int));
	
	//add 23.7.15
    infra_acdc = (float*)nrf_malloc(MAX_ACDC_NUM*sizeof(float));
    red_acdc = (float*)nrf_malloc(MAX_ACDC_NUM*sizeof(float));
    hr_rate = (float*)nrf_malloc(MAX_ACDC_NUM*sizeof(float));

	return 0;
}

void afe_spo2_free(void)
{
	if (spo2_red != NULL) {
		nrf_free(spo2_red);
		spo2_red = NULL;
	}

	if (spo2_infra != NULL) {
		nrf_free(spo2_infra);
		spo2_infra = NULL;
	}

	if (spo2_green != NULL) {
		nrf_free(spo2_green);
		spo2_green = NULL;
	}
	
	if (wrcoefbuf != NULL) {
		nrf_free(wrcoefbuf);
		wrcoefbuf = NULL;
	}

	if (AccData != NULL) {
		nrf_free(AccData);
		AccData = NULL;
	}
    if (HistSpo2 != NULL) {
		nrf_free(HistSpo2);
		HistSpo2 = NULL;
	}
    if (HistAcc != NULL) {
		nrf_free(HistAcc);
		HistAcc = NULL;
	}
    if (Acc_Z_Buf != NULL) {
		nrf_free(Acc_Z_Buf);
		Acc_Z_Buf = NULL;
	}
    if (meanredpwrbuf != NULL) {
		nrf_free(meanredpwrbuf);
		meanredpwrbuf = NULL;
	}
    if (meaninfrapwrbuf != NULL) {
		nrf_free(meaninfrapwrbuf);
		meaninfrapwrbuf = NULL;
	}
    if (zerosVect != NULL) {
		nrf_free(zerosVect);
		zerosVect = NULL;
	}
    //tired
    if (SPO2_FFT_In != NULL) {
		nrf_free(SPO2_FFT_In);
		SPO2_FFT_In = NULL;
	}
    if (SPO2_FFT_Out != NULL) {
		nrf_free(SPO2_FFT_Out);
		SPO2_FFT_Out = NULL;
	}
    if (HistRR != NULL) {
		nrf_free(HistRR);
		HistRR = NULL;
	}
    if (HistHRVVect != NULL) {
		nrf_free(HistHRVVect);
		HistHRVVect = NULL;
	}
    if (HistHRVMVect != NULL) {
		nrf_free(HistHRVMVect);
		HistHRVMVect = NULL;
	}
    if (HistHrVld != NULL) {
		nrf_free(HistHrVld);
		HistHrVld = NULL;
	}
    if (HistHrVect != NULL) {
		nrf_free(HistHrVect);
		HistHrVect = NULL;
	}
	if (wrcoefbuf_base != NULL) {
		nrf_free(wrcoefbuf_base);
		wrcoefbuf_base = NULL;
	}
	//21.11.11
	if (acc_mean_buf != NULL) {
		nrf_free(acc_mean_buf);
		acc_mean_buf = NULL;
	}
	
	//22.12.23
	if (obj_br != NULL)
	{
		wave_free(obj_br);
		obj_br = NULL;
	}
	if(wt_br != NULL)
	{
		wt_free(wt_br);
		wt_br = NULL;
	}
	if (BREATH_BUF != NULL) {
		free(BREATH_BUF);
            BREATH_BUF = NULL;
    }

    //add 23.7.12
    if (red_filter != NULL) {
            free(red_filter);
            red_filter = NULL;
    }
    if (infra_filter != NULL) {
            free(infra_filter);
            infra_filter = NULL;
    }
	
	//add 23.7.15
    if (infra_acdc != NULL) {
    	free(infra_acdc);
    	infra_acdc = NULL;
	}
    if (red_acdc != NULL) {
    	free(red_acdc);
    	red_acdc = NULL;
	}
    if (hr_rate != NULL) {
    	free(hr_rate);
    	hr_rate = NULL;
	}
    // turn off top led
    ring_topled_off();                          
    offline_ledflg = OFFLINE_LED_OFF;
}

int test_afe_proc = 0;
void get_max_valid(int *input, int inlen, int *max, int *maxid)
{
	*max = 0;
	*maxid = 0;
    if(inlen < 1)
    {
        return;
    }
	for (int i = 0; i < inlen; i++)
	{
		if (*max < input[i])
		{
			*max = input[i];
            *maxid = i;
		}
	}
}
void get_min_valid(int *input, int inlen, int *min, uint16_t *minid)
{
	*min = input[0];
	*minid = 0;
    if(inlen < 1)
    {
        return;
    }
	for (uint16_t i = 0; i < inlen; i++)
	{
		if (*min > input[i])
		{
			*min = input[i];
            *minid = i;
		}
	}
}
int test_accval;
extern uint8_t g_sleep_flag;
extern uint8_t g_sleep_end_check_flag;
void SpO2_Acc_Proc(acc_axis*pt)
{	
	int64_t xyz;
    if(pt->x == acc_error_data1 && pt->y == acc_error_data1 && pt->z == acc_error_data1)
    {
        return;
    }
    int16_t accstandard = 1000;
    uint16_t accthre1 = 100;
    uint8_t accthre2 = 30;
    float tmpaccval = 0;
    int tmpsumacc = 0;
    int meanaccval;
	int sleep_acc = 0;
    int16_t ACC_x, ACC_y, ACC_z;
    if(length_accval != ACC_13_SAMPLERATE && sensor.odr == LIS2DW12_XL_ODR_12Hz5 ) // 12.5 Hz ODR
    {
        length_accval = ACC_13_SAMPLERATE;
        accdata.accstfp = 0;
    }

    if(pt->x == acc_error_data2 && pt->y == acc_error_data2 && pt->z == acc_error_data2)
    {
        accdata.ACC_g = defined_accg;
        AccData[accdata.accstfp%length_accval] = accdata.pre_ACC_g;
        accdata.accstfp++;
        uint8_t spo2_flag = accval & 0x40;
        accval = data_compress16(tmpaccval,Moveflg,spo2_flag);
		
        acc_flag = 1;
    }
    else
    {
		ACC_x = pt->x;
        ACC_y = pt->y;
        ACC_z = pt->z;

		accdata.ACCx = ACC_x;
		accdata.ACCy = ACC_y;
		accdata.ACCz = ACC_z;
        
        //accdata.ACC_g = (int)pow(abs((int)ACC_x*ACC_x + (int)ACC_y*ACC_y + (int)ACC_z*ACC_z), 0.5);

		xyz = (int)ACC_x*ACC_x + (int)ACC_y*ACC_y + (int)ACC_z*ACC_z;
		//xyz = 9900;
		accdata.ACC_g = (int)(sqrt_bv(xyz*100)/10);


/*
        NRF_LOG_INFO("SpO2_Acc_Proc x = %d, y = %d, z = %d\n ACC_g=%d\n",accdata.ACCx,
			accdata.ACCy, accdata.ACCz, accdata.ACC_g);
*/

        AccData[accdata.accstfp%length_accval] = accdata.ACC_g;
        accdata.accstfp++;
        test_greenval = accdata.ACC_g;

//add data to acc_z && start gesture in spo2
        if( sensor.odr == LIS2DW12_XL_ODR_12Hz5 )
        {
            if(accdata.acc_znum < ACC_Z_LENGTH)
            {
                Acc_Z_Buf[accdata.acc_znum] = ACC_z;
                accdata.acc_znum++;
            }
            else
            {
//                if(gesture_support() == 1 && acc_get_work_mode() != IMU_GESTURE)
//                {
//                    if(gesture_start_inspo2(Acc_Z_Buf,ACC_Z_LENGTH) == 1)
//                    {
//                        new_acc_start(IMU_GESTURE);
//                        length_accval = ACC_104_SAMPLERATE;
//                        gesture_init();
//                        test_afe_proc = 0;
//                    }
//                }
                for (int z_no = 0; z_no < (ACC_Z_LENGTH - ACC_13_SAMPLERATE); z_no++)
                {
                    Acc_Z_Buf[z_no] = Acc_Z_Buf[z_no + ACC_13_SAMPLERATE];
                }
                accdata.acc_znum = ACC_Z_LENGTH - ACC_13_SAMPLERATE;
//                if(acc_get_work_mode() == IMU_GESTURE)
//                {
//                    accdata.acc_znum = 0;
//                }
            }
        }
        
        if(abs(accdata.ACC_g - accstandard) > accthre1 || (abs(accdata.ACC_g - accdata.pre_ACC_g) > accthre2 && accdata.pre_ACC_g > 0))
        {
            accdata.movetime++;
        }
        if(!(accdata.accstfp%length_accval))
        {
            tmpaccval = get_var(AccData, length_accval);
			//get acc mean val
			int acc_sum = 0;
			for (int i = 0; i < length_accval; i++)
			{
				acc_sum += AccData[i];
			}
			acc_mean_buf[meanacc_cnt%MAXNUM_HRVvect] = acc_sum / length_accval;
			meanacc_cnt++;
//            test_accval = tmpaccval;
            //NRF_LOG_INFO("tmpaccval = %d\r\n",tmpaccval);
            uint8_t spo2_flag = accval & 0x40;
            accval = data_compress16(tmpaccval, Moveflg, spo2_flag);
            test_accval = tmpaccval;
            if(tmpaccval > acc_thre){
				acc_duration = 10;
            }
			if(acc_duration > 0)
			{
				acc_flag = 1;
				acc_duration--;
			}
			else
			{
				acc_flag = 0;
			}

            if(accdata.movetime > 0)
            {
                accdata.movement = 1;
            }
            else
            {
                accdata.movement = 0;
            }
            accdata.movetime = 0;
			if(g_sleep_flag == START_SLEEP_MODE)
			{
				sleep_acc = (int)(accval & 0x7F);
				if(acc_get_Sleep_flag() == START_SLEEP_MODE)
				{
					Sleep_end_judge_acc_get(sleep_acc);
				}
				
			}
			
			//NRF_LOG_INFO("1111==========accval = %d\n",(accval & 0x7F));
			//NRF_LOG_INFO("1111======acc_get_Sleep_flag(void) = %d\n",acc_get_Sleep_flag());

        }
        accdata.pre_ACC_g = accdata.ACC_g;
    }


}

int test_Tcc = 0;
int test_maxtcc = 0;
void ble_send_persecond(void)
{
    uint8_t frame_buf[16];
	//frame_buf[0] = SPO2_WORD;
    frame_buf[0] = 18;
    frame_buf[1] = 18;
    frame_buf[2] = led_reg.infra_reg; //extend write
    frame_buf[3] = led_reg.red_reg; //extend write
    frame_buf[4] = g_afe.flag; //extend write
    frame_buf[5] = g_afe.shr;//red_detec.offhand; //extend write
    frame_buf[6] = g_afe.hr; //extend write
    frame_buf[7] = AlgPara.SSpo2; //extend write
    frame_buf[8] = spo2_fortest;//g_afe.breathrate;
    frame_buf[9] = test_trustflg;//BR_vect[BR_cnt-1];
    frame_buf[10] = offdac_led2;//est_infra_acdc_cnt;//BR_cnt;
    frame_buf[11] = offdac_led3;//test_infra_zecnt;
    //frame_buf[12] = test_res;
    frame_buf[12] = (char)(test_Tcc >> 8);
    frame_buf[13] = (char)(test_Tcc); 
//	frame_buf[9] = (char)(test_accval >> 8);
//	frame_buf[10] = (char)(test_accval);    
    frame_buf[14] = (char)(test_maxtcc >> 8);
    frame_buf[15] = (char)(test_maxtcc);
	
//	frame_buf[8] = totalHrCnt>>8;
//	frame_buf[9] = totalHrCnt&0xFF;
    //frame_buf[12] = dc_adj.current_flg;
    //frame_buf[13] = g_afe.offhand_flg;
    copy_to_log(frame_buf, 16);
}

int get_rrvar(uint8_t *in, int inlen, int sp, int ep)
{
	if (ep == sp || sp <= 0 || ep <= 0)
	{
		return 254;
	}

	int sum = 0;
	int var = 0;
	if(ep > sp)
	{
		int clclen = ep - sp + 1;
		for (int i = sp; i <= ep; i++)
		{
			sum += in[i];
		}
		int mean = sum / clclen;
		for (int i = sp; i <= ep; i++)
		{
			var += abs(in[i] - mean);
		}
		var = var / clclen;
	}
	else
	{
		int clclen = inlen - sp + ep + 1;
		for (int i = 0; i <= ep; i++)
		{
			sum += in[i];
		}
		for (int i = sp; i < inlen; i++)
		{
			sum += in[i];
		}
		int mean = sum / clclen;
		for (int i = 0; i <= ep; i++)
		{
			var += abs(in[i] - mean);
		}
		for (int i = sp; i < inlen; i++)
		{
			var += abs(in[i] - mean);
		}
		var = var / clclen;		
	}
	return var;
}

uint8_t get_br_mean(uint8_t* input, int inlen)
{
	if(inlen < 0)
	{
		return 0;
	}	
	int sum = 0;
	for(int i = 0; i < inlen; i++)
	{
		sum += input[i];
	}
	return (uint8_t)(sum/inlen);
}

/*********************************************************************
* @fn      afe_spo2_proc_wavelate()
*
* @brief   spo2 calculation ;
*
* @param   raw data
* @return  spo2 and hr.
*/
extern E_ACC_PROC g_ACC_statu;
extern int AccDataRealtime[ACC_13_SAMPLERATE];
extern uint8_t afe_spo2_proc_wavelate(void)
{
    uint8_t spo2valflg = 0;
    if(LED1readyfor_calc==1 && LED2readyfor_calc == 1 )
    {

		if(g_ACC_statu == e_acc_EHR)
		{
			memcpy(AccData,AccDataRealtime,ACC_13_SAMPLERATE*sizeof(int));
		}
		
        LED1readyfor_calc = 0;
        LED2readyfor_calc = 0;
        if(cuinitflag == 1 && g_afe.flag != SPO2_HR_VALID)
        {

            //g_afe.flag = SPO2_HR_VALID;
            cuinitflag = 0;
            startchange = 0;
            //infrathreslow = 1.5e6;
            //infrathreshigh = 1.9e6;
        }	
// offdac add ready time to 15s
        if(g_afe.flag != SPO2_HR_INVALID) 
        {
            if(changecnt <= 150)
            {
                g_afe.flag = SPO2_HR_READYING;				
            }
            else if(g_afe.flag != SPO2_HR_VALID)
            {
                g_afe.flag = SPO2_HR_VALID;
            }
        }	

        int Tcc = get_var(AccData, length_accval);
        test_Tcc = Tcc;
        int tmpmaxtcc = Tcc;
        int tmpmintcc = Tcc;
        static int datano = 0;
        if(datano < MS_datalength)
        {
            MotionState[datano] = Tcc;
            datano++;
        }
        else
        {
            memcpy(MotionState, MotionState + 1, sizeof(int)*(MS_datalength - 1));
            MotionState[MS_datalength - 1] = Tcc;
            
            get_max_min(MotionState+7, MS_datalength-7, &tmpmaxtcc, &tmpmintcc);
            test_maxtcc = tmpmaxtcc;
        }
        
//        summotionstate = 0;
//        for (int i = 6; i < MS_datalength; i++)
//        {
//            summotionstate += MotionState[i];
//        }
        if(g_afe.flag != SPO2_HR_INVALID)
        {
            get_spo2(spo2_red, spo2_infra, spo2_green, red_filter, infra_filter, length_wavelet, AccData, length_accval, &AlgPara);

            if (get_InSleep(HistAcc,HistSpo2,AlgPara.Proccnt)) {
                g_afe.sleep_status = SLEEP_STAGE;
            } else {
                g_afe.sleep_status = WAKE_STAGE;
            }
            
            AlgPara.heartrate = (uint8_t)((AlgPara.heartrate + AlgPara.heartratehist)/2);

//put history hr value
            AlgPara.heartratehist = AlgPara.Sheartrate;
            // AlgPara.heartratehist = AlgPara.heartrate;
            g_afe.hr = AlgPara.heartrate;
            g_afe.shr = AlgPara.Sheartrate;

#ifdef HRV_RECORD
            if (RING_AFE_HRV_ON_MODE == (RING_AFE_HRV_ON_MODE & afe_get_work_mode())) {
      
//      static uint32_t v = 0;
//      AlgPara.HrCnt = 64; // Only for test.
		if(HRV_timestamp == 0)      //add first time stamp
		{
                    if(AlgPara.HrCnt > 0)
                    {
                        HRV_timestamp = UTC_getClock();
                    }	
		}
		
                totalHrCnt += AlgPara.HrCnt;
                for (int i = 0; i < AlgPara.HrCnt; i++) {
                    if (HRV_cnt < MAXNUM_HRVvect) {
                        HRV_vect[HRV_cnt] = AlgPara.HrVect[i];
              //          HRV_vect[HRV_cnt] = (v++) % 256; // 0x01;// Only for test.
                        HRV_cnt++;
                    }
                }
                send_hrv_data(AlgPara.HrVect, AlgPara.HrCnt);
            }
#endif
            g_afe.Spo2show = data_compress(AlgPara.SSpo2);
            g_afe.spo2 = data_compress(AlgPara.Spo2);
            g_afe.tiredflg = AlgPara.tiredflg;

            
            if((AlgPara.Spo2 != 0 && AlgPara.SSpo2 != 0) && AlgPara.heartrate != 0 && changecnt > 100)
            {
                g_afe.flag = SPO2_HR_VALID;
                
                if(g_afe.offhand_flg == 0 && tmpmaxtcc < 20)
                {
                    spo2valflg = 1;
                }
            }
            else
            {
                if(g_afe.flag == SPO2_HR_VALID)
                {
                    g_afe.flag = SPO2_HR_READYING;
                }
            }
        }
        else
        {
            g_afe.hr = 0;
            g_afe.shr = 0;
            g_afe.spo2 = 0;
            g_afe.Spo2show = 0;
            g_afe.sleep_status = 0;
        }
		
//breath proc
        AlgPara.breathrate = (uint8_t)((AlgPara.breathrate + AlgPara.breathratehist)/2);
        AlgPara.breathratehist = AlgPara.breathrate;
        g_afe.breathrate = AlgPara.breathrate;
        //if(breath_flg == 1)
        //{
        //    breath_flg = 0;
        //    BREATH_Proc_shedule(BREATH_fifo.buf,1000,&g_afe.breathrate);
        //    if(g_afe.flag == SPO2_HR_INVALID | g_afe.hr == 0)
        //    {
        //        g_afe.breathrate = 0;
        //    }
        //    breathrate_buf[breathrate_cnt] = g_afe.breathrate;
        //    breathrate_cnt++;
        //    //NRF_LOG_INFO("breathrate=%d, br_cnt = %d", g_afe.breathrate, BR_cnt);
            
        //    if(breathrate_cnt >= MAX_BREATHRATE_BUFSIZE)
        //    {
        //        if(BR_timestamp == 0)      //add first time stamp
        //        {
        //            BR_timestamp = UTC_getClock();
        //        }
        //        if (BR_cnt < MAX_BREATH_CNT) 
        //        {
                        
        //            BR_vect[BR_cnt] = g_afe.breathrate;//get_br_mean(breathrate_buf,breathrate_cnt);
        //            BR_cnt++;
        //        }
        //        breathrate_cnt = 0;
        //    }
        //}
// part 6;before return;
//********recover of point******
        fifo32_recover(&spo2_red_fifo,length_recover);
        fifo32_recover(&spo2_infra_fifo,length_recover);
        fifo32_recover(&spo2_green_fifo,length_recover);
        fifo32_recover(&spo2_red_filter,length_recover);
        fifo32_recover(&spo2_infra_filter,length_recover);
        fifo32_recover(&green_health_fifo,HRV_PROCESS_DATA_PERIOD - test_data_num);
		
    }
    //ble_send_persecond();
    return spo2valflg;
}

void afe_wave_free(void)
{
	if (obj != NULL)
	{
		wave_free(obj);
		obj = NULL;
	}
	if(wt != NULL)
	{
		wt_free(wt);
		wt = NULL;
	}
	fifo32_init(&spo2_red_fifo,length_rawdata, spo2_red);
	fifo32_init(&spo2_infra_fifo, length_rawdata, spo2_infra);
	fifo32_init(&spo2_green_fifo, length_rawdata, spo2_green);
    fifo32_init(&spo2_red_filter, length_wavelet, red_filter);
    fifo32_init(&spo2_infra_filter, length_wavelet, infra_filter);
}

void offwrist_init(void)
{
    memset((uint8_t*)&dt_ofline, 0, sizeof(DETECT_OFFLINE));
}

int HRV_accval(void)
{
	int tmp_var = get_var(acc_mean_buf, min(meanacc_cnt,MAX_HRV_CNT));
	meanacc_cnt = 0;
	return tmp_var;
}

uint8_t HRV_isready_flg(void)
{
	int max_hrvcnt = MAX_HRV_CNT - acc_var_size;
    if(HRV_cnt >= max_hrvcnt)  // >=248
    {
		for(int i = 0; i < HRV_cnt-max_hrvcnt; i++)     //remove HRV vect tail
		{
			acc_mean_tail[i%MAX_RR_Cnt] = HRV_vect[i+max_hrvcnt];
		}
		
		// int tmpacc_var = HRV_accval();
		// HRV_vect[max_hrvcnt] = (uint16_t)(tmpacc_var >> 16); //add acc var 2 byte
		// HRV_vect[max_hrvcnt+1] = (uint16_t)tmpacc_var;

		//add total cnt 2 byte
		HRV_vect[0] = (uint8_t)(totalHrCnt >> 8); 
		HRV_vect[1] = (uint8_t)(totalHrCnt);

        return 1;
    }
    return 0;
}

//int get_HRV_cnt(void)
//{
//	return HRV_cnt;
//}

void HRV_cnt_clear(void)
{
	int sp = 2;
    // int sp = 0;
	int tail_cnt = HRV_cnt - (MAX_HRV_CNT - acc_var_size);
	if(tail_cnt > 0 && tail_cnt <= MAX_RR_Cnt)
	{
		for(int i = 0; i < tail_cnt; i++)     //add HRV
		{
			HRV_vect[i + sp] = acc_mean_tail[i];
		}
		HRV_cnt = tail_cnt + sp;
	}
	else
	{
		HRV_cnt = sp;
	}	
	rr_endpos = HRV_cnt - 1;
}

//add 21.11.22 pulse
uint8_t pulse_offline = 0;
bool afe_pulse_init(void)
{
	dt_ofline.is_offline = 0;
	
    uint32_t pulse_reg22val = 0x000000|(2 << 12)|(2 << 6)|(3 << 0);
	set_afe_reg(0x22, pulse_reg22val);
	return true;
}

uint8_t pulse_offline_flag(void)
{
	return pulse_offline;
}

uint8_t PULSE_bufProc_wavelet(void *afe_in, uint8_t frame_length)
{
	//offline detect
	int tmpledamb = 0;
	struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)afe_in;
	for(int i = 0; i < frame_length; i++)
	{
        tmpledamb = afe_bpt[i].phase4;
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 1)
            {
                if(tmpledamb > (afe_bpt[i - 1].phase4 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase4 + OFFLINE_THRE))
                {
                    dt_ofline.offline_num++;
                }
            }
        }        
	}
	changecnt++;
	
	if(!(changecnt%(50/frame_length)))
    {
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            get_afe_reg(0x22, &reg22val);
            int greenlowreg = reg22val >> 18;
            int tmpredreg = (reg22val - (greenlowreg << 18)) >> 12;
            
			if(dt_ofline.offline_num >= 4)                   // is off hand
			{
//				set_afe_reg(0x22, 0x000000);// turn off led
				ring_topled_off();                          // turn off top led
				offline_ledflg = OFFLINE_LED_OFF;
				pulse_offline = 1;
			}
			else                                            // is on hand
			{
				ring_topled_off();                           // turn off top led
				offline_ledflg = OFFLINE_LED_OFF;
				if(pulse_offline == 1)
				{
//					afe_pulse_init();
					pulse_offline = 0;
				}
			}
			dt_ofline.offline_num = 0;
        }

        int change_step = 300/frame_length;
        if(!(changecnt%change_step))   //turn on top green led
        {
            if(offline_ledflg == OFFLINE_LED_OFF)
            {
                ring_topled_on();
                offline_ledflg = OFFLINE_LED_ON;
            }
        }
    }
	return 1;
}

uint8_t get_spo2_procnum(void)
{
	return count_spo2_bufproc;
}
void init_spo2_procnum(void)
{
	count_spo2_bufproc = 0;
}
uint32_t get_spo2_calculnum(void)
{
	return num_calculation;
}

//22.12.23 add breath proc

void wavelet_breathProc(int *in, int inlen, int slevel, int elevel, int *out)
{
	dwt(wt_br, in);// Perform DWT   

	int sp = 0;
	for (int i = 0; i < slevel; i++)
	{
		sp += wt_br->length[i];
	}

	for (int i = 0; i < sp; i++)
	{
		wt_br->output[i] = 0;
	}
	sp = sp + wt_br->length[2];
	int ep = sp + wt_br->length[3];
	for(int i = sp; i < ep; i++)
	{
		wt_br->output[i] = wt_br->output[i]*0.5;
	}
	
//	ep = sp;
//	for (int i = slevel; i < elevel; i++)
//	{
//		ep += wt_br->length[i];
//	}
	int totallen = ep;
	for (int i = elevel; i < 8; i++)
	{
		totallen += wt_br->length[i];
	}

	for (int i = ep; i < totallen; i++)
	{
		wt_br->output[i] = 0;
	}

	idwt(wt_br, out);
}

int Outliers_Remove(int *input, int inlen)
{
	int outlen = inlen;
	int min_cyclength = 15;
	
	for(int i = 0; i < inlen-1; i++)
	{
		if(input[i+1] - input[i] <= 6)
		{
			for (int j = i; j < inlen-2; j++)
			{
				input[j] = input[j+2];
			}
			outlen = outlen - 2;
		}	
	}
	
//	inlen = outlen;
//	for(int i = 0; i < inlen-3; i+=2)
//	{
//		if(input[i+2] - input[i] < min_cyclength)
//		{
//			for (int j = i+2; j < inlen-3; j++)
//			{
//				input[j] = input[j+1];
//			}
//			i--;
//			outlen--;
//		}	
//	}
	return outlen;
}

void get_breathrate(int *input, int inlen, uint8_t br_hist, uint8_t *breathrate, uint8_t *VldFlg)
{
	int threshold = 0;
	int tcnt = 0;
	for (int i = 0; i < inlen; i++)
	{
		if ((input[i] <= threshold && input[i + 1] > threshold) || (input[i] >= threshold && input[i + 1] < threshold))
		{
			zerosVect[tcnt] = i;
			if(tcnt > 0 && (zerosVect[tcnt] - zerosVect[tcnt-1] <= 5)) {
				tcnt--;
			} 
			else {
				tcnt++;
			}				
		}
		if (tcnt >= MAX_ZEROS_NUM)
		{
			break;
		}
	}
	if (tcnt <= 4)
	{
		*VldFlg = 0;
		*breathrate = br_hist;
		return;
	}
//	zerocnt = tcnt;
	int correct_cnt = Outliers_Remove(zerosVect, tcnt);
//	outcnt = correct_cnt;
	
	int ttcnt = 0;
	for (int i = 0; i < correct_cnt / 2; i++)
	{
		zerosVect[ttcnt++] = zerosVect[i*2];
	}
	for(int i = 0; i < ttcnt-1; i++)
	{
		zerosVect[i] = zerosVect[i + 1] - zerosVect[i];
	}
	tcnt = ttcnt - 1;
	
	int Lowthreshold = 80;
	int Highthreshold = 13;
	int cnt = 0;
	for (int i = 0; i < tcnt; i++)
	{
		if (zerosVect[i] <= Lowthreshold && zerosVect[i] >= Highthreshold)
		{
			zerosVect[cnt++] = zerosVect[i];
		}
	}
//	outcnt1 = cnt;
	
	#if 0
	tcnt = cnt;
	int tmpmean = get_means(zerosVect, tcnt);
	cnt = 0;
	if (abs(zerosVect[0] - tmpmean) < MAX(tmpmean*0.5, 10))
	{
		cnt++;
	}
	
	for (int i = 1; i < tcnt - 1; i++)
	{
		if (zerosVect[i] >= Lowthreshold || zerosVect[i] <= Highthreshold)
		{
			cnt = MAX(0, cnt - 1);
		}
		else
		{
			if (zerosVect[i] > zerosVect[i - 1] * 0.5 && zerosVect[i] < zerosVect[i - 1] * 1.5 && zerosVect[i] > zerosVect[i + 1] * 0.5 && zerosVect[i] < zerosVect[i + 1] * 1.5)
			{
				zerosVect[cnt++] = zerosVect[i];
			}
		}
	}
	if (abs(zerosVect[tcnt - 1] - tmpmean) < MAX(tmpmean*0.5, 10))
	{
		zerosVect[cnt++] = zerosVect[tcnt - 1];
	}
	#endif
	
	AlgData_t vldsum = 0;
	AlgData_t vldcnt = 0;
	for (int i = 0; i < cnt; i++)
	{
		vldsum += zerosVect[i];
		vldcnt += 1;
	}
	uint8_t tmpbr = 0;
	if (vldcnt > 1 && vldsum > 0)
	{
		tmpbr = (int)(600 / (vldsum / vldcnt));
	}
	//tmpbr = max(tmpbr, cnt);
	//tmpbr = min(tmpbr, (correct_cnt+1)/2);
//	outcnt2 = tmpbr;
	//zerocnt = vldcnt;
//	tmpsum = vldsum;
//	uint8_t tmpbr = 0;
//	tmpbr = (int)(correct_cnt/2);

	if (tmpbr > 0 && tmpbr < 40)
	{
		if(br_hist == 0)
		{
			*breathrate = tmpbr;
		}
		else if(tmpbr > br_hist *1.2 || tmpbr < br_hist *0.8)
		{
			*breathrate = tmpbr*0.3 + br_hist*0.7;
		}
		else
		{
			*breathrate = tmpbr*0.5 + br_hist*0.5;
		}	
		
	}
	else
	{
		*breathrate = br_hist;
	}
	*VldFlg = tcnt;

}

void BREATH_Proc_shedule(int *indata, int inlen, unsigned char *breathrate)
{
	//to do
	int Waveletlen = inlen;
	wavelet_breathProc(indata, Waveletlen, 2, 4, wrcoefbuf);
	
	int* tmpbuf = (int*)wrcoefbuf;

	unsigned char tmpbr1 = 0;
	unsigned char tmpbr2 = 0;
	uint8_t vldflg;
	get_breathrate(tmpbuf,Waveletlen,breathratehist, breathrate, &vldflg);
	
	breathratehist = *breathrate;
        //NRF_LOG_INFO("breath rate = %d", breathrate);
	//recover data
	fifo32_recover(&BREATH_fifo,breath_recover);
//	ble_send_persecond();
}	

uint8_t BR_isready_flg(void)
{
    if(BR_cnt >= MAX_BREATH_CNT)
    {
        return 1;
    }
    return 0;
}

void BR_cnt_clear(void)
{
	BR_cnt = 0;
}

uint8_t BR_stopsaving(void)
{
	if(BR_cnt == 0)
	{
		return 0;
	}
	
	if(BR_cnt < MAX_BREATH_CNT)
	{
        for(int i = BR_cnt; i < MAX_BREATH_CNT; i++)
		{
			BR_vect[i] = 0;
		}
    }
	return 1;
}	
//add 23.7.17     dailymode

int afe_daily_malloc(void)
{
    nrf_mem_init();
    
    daily_green = (int*)nrf_malloc(length_rawdata*sizeof(int));
    
    wrcoefbuf = (int*)nrf_malloc(length_wavelet*sizeof(int));
    AccData = (int*)nrf_malloc(length_accval*sizeof(int));
    HistAcc = (uint8_t*)nrf_malloc(MAX_HIST_LEN*sizeof(uint8_t));

    zerosVect = (int*)nrf_malloc(MAX_ZEROS_NUM*sizeof(int));

    HistHrVld = (uint8_t*)nrf_malloc(HIST_HR_LEN*sizeof(uint8_t));    

    wrcoefbuf_base = (int*)nrf_malloc(WAVE_PROC_LEN*sizeof(int));
	
    acc_mean_buf = (int*)nrf_malloc(MAXNUM_HRVvect*sizeof(int));

    return 0;
}

//static void dailyproc_init(void)
//{
    
//}
int frameno_daily = 0;
bool afe_daily_init(void)
{
    reg22val = 0x000000|3;
    set_afe_reg(0x22, reg22val);

    if (obj != NULL)
    {
        wave_free(obj);
        obj = NULL;
    }
    if(wt != NULL)
    {
        wt_free(wt);
        wt = NULL;
    }
    
    if(daily_green == NULL || wrcoefbuf == NULL || AccData == NULL || HistAcc == NULL)
        return false;
   frameno_daily = 0;
//offwrist init;
    offwrist_init();
//afeproc init
    fifo32_init(&daily_green_fifo, length_rawdata, daily_green);
    afeproc_init();
    //dailyproc_init();
//tired
    Init_para();

    //wt init;
    int N = length_wavelet - 2*MARLEN;
    int J = WAVELET_LEVEL;
    char *name = "db4";
    obj = wave_init(name);	//peak: 218+32
    wt = wt_init(obj, "dwt", N, J);// Initialize the wavelet transform object, peak:0x10AC	
    setDWTExtension(wt, "sym");// Options are "per" and "sym". Symmetric is the default option
    setWTConv(wt, "direct");

    return true;
}


uint8_t DAILY_bufProc_wavelet(void *afe_in, uint8_t frame_length)
{
    struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)afe_in;
    int tmp_green_orig = 0;
    int ppgFilter = 0;
    for(int i = 0; i < frame_length; i++)
    {
        int tmpledgreen = afe_bpt[i].phase1;//green
		//HRV health get
		//fifo32_putPut(&green_health_fifo, tmpledgreen - afe_bpt[i].phase2);
		fifo32_putPut(&green_health_fifo, tmpledgreen);
		//HRV_health_data_get();

		//HRV_health_data_get_low_power();

		//HRV_health_data_get_new();
		//NRF_LOG_INFO("=========tmpledgreen = %d\n",tmpledgreen);
		//tmpledgreen = (int)smoothFilter3(IIR_bandpass_Filter(tmpledgreen - afe_bpt[i].phase2));

		// tmpledgreen = (int)smoothFilter3(IIR_bandpass_Filter(tmpledgreen));

        // ppgFilter = IIR_bandpass_Filter(tmpledgreen);
        // tmpledgreen = smoothFilter4_int(ppgFilter);
        tmpledgreen = (int)IIR_bandpass_chebyshev_Filter_heart_sport((float)tmpledgreen);

		//tmpledgreen = (int)afe_IIRLPFilter4(tmpledgreen);
		//NRF_LOG_INFO("2222=========3tmpledgreen = %d\n",tmpledgreen);
		//NRF_LOG_INFO("=========tmpledgreen = %d\n",tmpledgreen);
        // tmpledgreen = (int)smoothFilter3(afe_IIRLPFilter3(tmpledgreen));

        int tmpledamb = afe_bpt[i].phase1;
        //dt_ofline.min_ambval = min(dt_ofline.min_ambval, tmpledamb);
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 2)
            {
                if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase1 + OFFLINE_THRE_TWO))
                {
                    dt_ofline.offline_num++;
                }
                else
                {
                    if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && afe_bpt[i + 1].phase1 > (afe_bpt[i + 2].phase1 + OFFLINE_THRE_TWO))
                    {
                        dt_ofline.offline_num++;
                    }
                
                }
            }

        }        
        fifo32_putPut(&daily_green_fifo, tmpledgreen);       
    }

    changecnt++;
#if ONHAND_Spo2Daily_Proc
    //if(!(changecnt%(100/frame_length))) //1s
    //{
        // if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
		if(offline_ledflg == OFFLINE_LED_ON && changecnt > 3)  //when top green led is on, do offhand detect
        {
            //get_afe_reg(0x22, &reg22val);
            //int greenlowreg = reg22val >> 18;
            //int tmpredreg = (reg22val - (greenlowreg << 18)) >> 12;
            Detect_offline_fifo(RING_AFE_DAILY_MODE);
        }

        int change_step = 30000/frame_length;
//            if(state_entry.afe_who == AFE_RTSHOW) {
//                change_step = 300/AFE4900_ARRAY_SIZE;
//            }
        if(!(changecnt%change_step) || changecnt == 1)   //turn on top green led
        {
            if(offline_ledflg == OFFLINE_LED_OFF)
            {
                ring_topled_on();
                offline_ledflg = OFFLINE_LED_ON;
            }
        }
    //}
#endif

    if (fifo32_status(&daily_green_fifo)>=length_rawdata)
    {
        ble_send_persecond();
        return 1;	
    }	

    NRF_LOG_INFO("length %d", fifo32_status(&daily_green_fifo));
        
    return 0;  
}


uint8_t DAILY_bufProc_wavelet_LDS(void *afe_in, uint8_t frame_length)
{
    struct afe4900_sensor_data *afe_bpt = (struct afe4900_sensor_data *)afe_in;
    int tmp_green_orig = 0;
    for(int i = 0; i < frame_length; i++)
    {
        int tmpledgreen = afe_bpt[i].phase1;//green
		//HRV health get
		// fifo32_putPut(&green_health_fifo, tmpledgreen - afe_bpt[i].phase2);
		fifo32_putPut(&green_health_fifo, tmpledgreen);
		//HRV_health_data_get();

		// HRV_health_data_get_low_power();
		//HRV_health_data_get_new();
		//tmpledgreen = (int)smoothFilter3(afe_IIRLPFilter4(tmpledgreen));
		tmpledgreen = (int)smoothFilter3(IIR_bandpass_Filter(tmpledgreen));
        // tmpledgreen = (int)smoothFilter3(afe_IIRLPFilter3(tmpledgreen));

        int tmpledamb = afe_bpt[i].phase1;
        //dt_ofline.min_ambval = min(dt_ofline.min_ambval, tmpledamb);
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 2)
            {
                if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase1 + OFFLINE_THRE_TWO))
                {
                    dt_ofline.offline_num++;
                }
                else
                {
                    if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && afe_bpt[i + 1].phase1 > (afe_bpt[i + 2].phase1 + OFFLINE_THRE))
                    {
                        dt_ofline.offline_num++;
                    }
                
                }
            }

        }        
        fifo32_putPut(&daily_green_fifo, tmpledgreen);       
    }

    changecnt++;
#if ONHAND_Spo2Daily_Proc
    //if(!(changecnt%(100/frame_length))) //1s
    //{
        // if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
		if(offline_ledflg == OFFLINE_LED_ON && changecnt > 3)  //when top green led is on, do offhand detect
        {
            //get_afe_reg(0x22, &reg22val);
            //int greenlowreg = reg22val >> 18;
            //int tmpredreg = (reg22val - (greenlowreg << 18)) >> 12;
            Detect_offline_fifo(RING_AFE_DAILY_MODE);
        }

        int change_step = 6000/frame_length;
//            if(state_entry.afe_who == AFE_RTSHOW) {
//                change_step = 300/AFE4900_ARRAY_SIZE;
//            }
        if(!(changecnt%change_step) || changecnt == 1)   //turn on top green led
        {
            if(offline_ledflg == OFFLINE_LED_OFF)
            {
                ring_topled_on();
                offline_ledflg = OFFLINE_LED_ON;
            }
        }
    //}
#endif

    if (fifo32_status(&daily_green_fifo)>=length_rawdata)
    {
        ble_send_persecond();
        return 1;	
    }	

    NRF_LOG_INFO("length %d", fifo32_status(&daily_green_fifo));
        
    return 0;  
}

void get_daily(int *green, int inlen, int *Acc, int Acclen, AlgProc_Para *AlgPara)
{
    int Tcc = get_var(Acc, Acclen);

    AlgPara->Acc = Tcc;

    int Marglen = MARLEN;
    int Waveletlen = inlen - 2 * Marglen;

    int slevel = 2;
    int elevel = 6;

   // wavelet_baseProc(green + Marglen, Waveletlen, slevel, elevel, wrcoefbuf);
   // int* tmpbuf = (int*)wrcoefbuf;

//    uint8_t vldflg;
//	get_hr(green + Marglen,Waveletlen, Tcc,AlgPara->heartratehist, &(AlgPara->heartrate), &vldflg, &(AlgPara->HrCnt), AlgPara->HrVect);   
	// Sleep_PPG_data_RR_get(green + Marglen,Waveletlen, Acc, Acclen,AlgPara);
    Sleep_PPG_data_RR_get_new(green + Marglen,Waveletlen, Acc, Acclen,AlgPara);

      // rr calculation hr
    for(int num = 0;num < AlgPara->HrCnt;num++)
    {
        if(AlgPara->HrVect[num] < 200)
        {
            g_HrRRNum[g_HrRRCnt] = AlgPara->HrVect[num];
            g_HrRRCnt = g_HrRRCnt + 1;
            if(g_HrRRCnt >= HR_ALG_RR_NUM)
            {
                g_HrRRCnt = 0;
            }
        }

    }

    uint8_t vaild_rr_num = 0;
    uint16_t RR_mean = 0;
    for(int num = 0;num < HR_ALG_RR_NUM;num++)
    {
        if(g_HrRRNum[num] != 0)
        {
            RR_mean = RR_mean + g_HrRRNum[num];
            vaild_rr_num = vaild_rr_num + 1;
        }
    }

	if(vaild_rr_num != 0)
    {
        AlgPara->heartrate = 1000*60/(10*RR_mean/vaild_rr_num);
    }
    


	// get_hr_process(green + Marglen, Waveletlen, Tcc,AlgPara->heartratehist, &(AlgPara->heartrate));


	//NRF_LOG_INFO("11111111=============== AlgPara->heartrate %d\n",AlgPara->heartrate);
    //get_hr(tmpbuf,Waveletlen, Tcc,AlgPara->heartratehist, &(AlgPara->heartrate), &vldflg, &(AlgPara->HrCnt), AlgPara->HrVect);   

//smooth hr
	// AlgPara->Sheartrate = AlgPara->heartrate;
	//AlgPara->Sheartrate = (AlgPara->Sheartrate + AlgPara->heartrate)/2;

    // if(AlgPara->heartrate > 40)
    if(AlgPara->heartrate >= LOWEST_HEART_VALUE)
    {
        if (HistHrsavepos == 0)
        {
            AlgPara->Sheartrate = (AlgPara->Sheartrate + AlgPara->heartrate)/2;
            //HistHeartrate[HistHrsavepos] = AlgPara->heartrate;
            HistHeartrate[HistHrsavepos] = AlgPara->Sheartrate;
            HistHrsavepos++;
        }
        else
        {
            int tmphrmean = 0;
            int tmpcnt1 = 0;
            if (HistHrsavepos < HIST_HR_LEN)
            {
                for (int i = 0; i < HistHrsavepos; i++)
                {
                    tmphrmean += HistHeartrate[i];
                    tmpcnt1++;
                }
            }
            else
            {
                for (int i = 0; i < HIST_HR_LEN; i++)
                {
                    tmphrmean += HistHeartrate[i];
                    tmpcnt1++;
                }
            }

            if (tmpcnt1 == 0)
            {
                AlgPara->Sheartrate = AlgPara->heartratehist;
            }
            else
            {
                //tmphrmean = tmphrmean / tmpcnt1;
                AlgPara->Sheartrate = (AlgPara->heartrate + tmphrmean)/(tmpcnt1 + 1);
                //if (vldflg < 2 || abs(tmphrmean - AlgPara->heartrate) > 15)
                HistHeartrate[HistHrsavepos % HIST_HR_LEN] = AlgPara->Sheartrate;
                HistHrsavepos++;
                
            }

        }
    }
        

    // AlgPara->Sheartrate = AlgPara->heartrate;
    // if (HistHrsavepos == 0)
    // {
    //     if (vldflg <= 2)
    //     {
    //         AlgPara->Sheartrate = AlgPara->heartratehist;
    //     }
    //     else
    //     {
    //         HistHeartrate[HistHrsavepos] = AlgPara->heartrate;
    //         HistHrVld[HistHrsavepos] = vldflg;
    //         HistHrsavepos++;
    //     }
    // }
    // else
    // {
    //     int tmphrmean = 0;
    //     int tmpcnt1 = 0;
    //     if (HistHrsavepos < HIST_HR_LEN)
    //     {
    //         for (int i = 0; i < HistHrsavepos; i++)
    //         {
    //             tmphrmean += HistHeartrate[i];
    //             tmpcnt1++;
    //         }
    //     }
    //     else
    //     {
    //         for (int i = 0; i < HIST_HR_LEN; i++)
    //         {
    //             tmphrmean += HistHeartrate[i];
    //             tmpcnt1++;
    //         }
    //     }

    //     if (tmpcnt1 == 0)
    //     {
    //         AlgPara->Sheartrate = AlgPara->heartratehist;
    //     }
    //     else
    //     {
    //         tmphrmean = tmphrmean / tmpcnt1;

    //         //if (vldflg < 2 || abs(tmphrmean - AlgPara->heartrate) > 15)
	// 		if (vldflg < 2)
    //         {
    //             AlgPara->Sheartrate = AlgPara->heartratehist;
    //         }
    //         else
    //         {
    //             HistHeartrate[HistHrsavepos % HIST_HR_LEN] = AlgPara->heartrate;
    //             HistHrVld[HistHrsavepos % HIST_HR_LEN] = vldflg;
    //             HistHrsavepos++;
    //         }
    //     }
    // }
}

void afe_daily_proc_wavelate(void)
{
    if(cuinitflag == 1)
    {
        cuinitflag = 0;
        startchange = 0;
    }		

    if(g_afe.flag != SPO2_HR_INVALID)
    {
        get_daily(daily_green, length_wavelet, AccData, length_accval, &AlgPara);
            
        //AlgPara.heartrate = (uint8_t)((AlgPara.heartrate + AlgPara.heartratehist)/2);
		AlgPara.heartrate = (uint8_t)((AlgPara.Sheartrate + AlgPara.heartratehist)/2);

    //put history hr value
        AlgPara.heartratehist = AlgPara.Sheartrate;
        // AlgPara.heartratehist = AlgPara.heartrate;
        g_afe.hr = AlgPara.heartrate;
        g_afe.shr = AlgPara.Sheartrate;

        //NRF_LOG_INFO("daily hr: %d, acc: %d!", g_afe.hr, accval);

#ifdef HRV_RECORD
        if (RING_AFE_HRV_ON_MODE == (RING_AFE_HRV_ON_MODE & afe_get_work_mode())) {
      
            if(HRV_timestamp == 0)      //add first time stamp
            {
                if(AlgPara.HrCnt > 0)
                {
                    HRV_timestamp = UTC_getClock();
                }	
            }
		
            totalHrCnt += AlgPara.HrCnt;
            for (int i = 0; i < AlgPara.HrCnt; i++) {
              if (HRV_cnt < MAXNUM_HRVvect) {
                HRV_vect[HRV_cnt] = AlgPara.HrVect[i];
      //          HRV_vect[HRV_cnt] = (v++) % 256; // 0x01;// Only for test.
                HRV_cnt++;
              }
            }
            // RRvar_cnt++;
            // if(RRvar_cnt%41 == 0)
            // {
            //     rr_endpos = HRV_cnt - 1;
            //     int inlen = MAX_HRV_CNT - acc_var_size;
            //     if(rr_startpos > inlen)
            //     {
            //         rr_startpos = 0;
            //     }
            //     else
            //     {
            //         rr_startpos = max(rr_startpos - 2,0);
            //     }
            //     int ep = max(rr_endpos - 2, 0);
            //     int sp = rr_startpos;
            //     int tmp_rrvar = get_rrvar(HRV_vect+acc_var_size, inlen, sp, ep);
            //     g_afe.rr_var = min(tmp_rrvar, 0xff);
            //     rr_startpos = HRV_cnt;
            //     RRvar_cnt = 0;
            // }
        }
#endif

        if(AlgPara.heartrate != 0)
        {
            g_afe.flag = SPO2_HR_VALID;
        }
        else
        {
            if(g_afe.flag == SPO2_HR_VALID)
            {
                g_afe.flag = SPO2_HR_READYING;
            }
        }
    }
    else
    {
        g_afe.hr = 0;
        g_afe.shr = 0;

    }
		
//        g_afe.time = swtimer_getSec();
// part 6;before return;
//********recover of point******
    fifo32_recover(&daily_green_fifo,length_recover);
//        NRF_LOG_INFO("g_afe.spo2: %d,  g_afe.hr: %d\r\n", AlgPara.Spo2, AlgPara.heartrate);

}

void afe_daily_free(void)
{
    if (daily_green != NULL) {
            nrf_free(daily_green);
            daily_green = NULL;
    }
    
    if (wrcoefbuf != NULL) {
            nrf_free(wrcoefbuf);
            wrcoefbuf = NULL;
    }

    if (AccData != NULL) {
            nrf_free(AccData);
            AccData = NULL;
    }
    
    if (HistAcc != NULL) {
		nrf_free(HistAcc);
		HistAcc = NULL;
	}
    if (zerosVect != NULL) {
		nrf_free(zerosVect);
		zerosVect = NULL;
	}
    if (HistRR != NULL) {
		nrf_free(HistRR);
		HistRR = NULL;
	}
    if (HistHRVVect != NULL) {
		nrf_free(HistHRVVect);
		HistHRVVect = NULL;
	}
    if (HistHRVMVect != NULL) {
		nrf_free(HistHRVMVect);
		HistHRVMVect = NULL;
	}
    if (HistHrVld != NULL) {
		nrf_free(HistHrVld);
		HistHrVld = NULL;
	}
    if (HistHrVect != NULL) {
		nrf_free(HistHrVect);
		HistHrVect = NULL;
	}
    if (wrcoefbuf_base != NULL) {
            nrf_free(wrcoefbuf_base);
            wrcoefbuf_base = NULL;
    }
	
    // turn off top led
    ring_topled_off();                          
    offline_ledflg = OFFLINE_LED_OFF;
}