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
uint16_t totalHrCnt = 0;

uint8_t HRV_vect[MAXNUM_HRVvect] = {0};

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
uint32_t reg22val = 0x000000|(2 << 12)|(2 << 6);
#else
uint32_t reg22val = 0x000000|(12 << 12)|(12 << 6)|12;
#endif
int initdonelength = 0;
int onestep = 0;
int preredval = 0;
int infrathreslow = 2e5;
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
#define HIST_HR_LEN 60
uint8_t HistHeartrate[HIST_HR_LEN];
int HistHrsavepos = 0;

#define ONHAND_Spo2_Proc  1
#define ONHAND_Spo2Daily_Proc  1
#define Spo2_ONHAND_INTERVAL   6000

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
#define acc_var_size 	2
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
#define offdc_deta 		4.15e5//2.08e5//218450//(436910*0.8)
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

        if(infrareg > 15 || redreg > 15)
        {
            infrareg = 15;
            redreg = 15;
        }
        //NRF_LOG_INFO("redreg = %d, infrareg = %d\r\n",redreg,infrareg);
        if(infrareg > 60)
                return 2;
        if(redreg > 60)
                return 2;
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

static void afe_partinit(void)
{
//wavelet init;
	fifo32_init(&spo2_red_fifo,length_rawdata, spo2_red);
	fifo32_init(&spo2_infra_fifo, length_rawdata, spo2_infra);
	fifo32_init(&spo2_green_fifo, length_rawdata, spo2_green);
    fifo32_init(&spo2_red_filter, length_wavelet, red_filter);
    fifo32_init(&spo2_infra_filter, length_wavelet, infra_filter);
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
    infrathreslow = 2e5;
    infrathreshigh = 1.8e6;

    length_accval = 13;
    HRV_cnt = 2;
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
    reg22val = 0x000000|(2 << 12)|(2 << 6) | (3 << 0);
  } else {
    reg22val = 0x000000|(2 << 12)|(2 << 6);
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
void get_hr(int *input, int inlen, int Tcc, uint8_t heartratehist, uint8_t *heartrate, uint8_t *VldFlg, uint8_t *HRCnt, uint8_t *HRVect)
{
	int Lowthreshold = 150;
	int Highthreshold = 34;
	int threshold = 0;
	int tcnt = 0;
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
						printf("debug");
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
                        printf("debug");
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
	*DCVCnt = *DCVCnt + 1;
	for (int i = 1; i < inlen - 200; i += 100)
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
	*PwrCnt = *PwrCnt + 1;
	for (int i = 1; i < inlen - proclen; i += steplen)
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

float spo2_fortest = 0;
int test_trustflg = 0;
int test_infra_acdc_cnt = 0;
uint8_t test_infra_zecnt = 0;
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

    get_hr(greentmpbuf, Waveletlen, Tcc,AlgPara->heartratehist, &(AlgPara->heartrate), &vldflg, &(AlgPara->HrCnt), AlgPara->HrVect);
	
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
    //smooth hr
    AlgPara->Sheartrate = AlgPara->heartrate;
	if (HistHrsavepos == 0)
	{
		if (vldflg <= 2)
		{
			AlgPara->Sheartrate = AlgPara->heartratehist;
		}
		else
		{
			HistHeartrate[HistHrsavepos] = AlgPara->heartrate;
			HistHrVld[HistHrsavepos] = vldflg;
			HistHrsavepos++;
		}
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
			tmphrmean = tmphrmean / tmpcnt1;

			if (vldflg < 2 || abs(tmphrmean - AlgPara->heartrate) > 15)
			{
				AlgPara->Sheartrate = AlgPara->heartratehist;
				}
			else
			{
				HistHeartrate[HistHrsavepos % HIST_HR_LEN] = AlgPara->heartrate;
				HistHrVld[HistHrsavepos % HIST_HR_LEN] = vldflg;
				HistHrsavepos++;
			}
		}
	}
    
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
    if(dt_ofline.offline_num >= 4)                   // is off hand
    {
        set_afe_reg(0x22, 0x000000);// turn off led
        set_afe_reg(0x20, 0x008013);// init gain
        set_afe_reg(0x3A, 0x000000);// init offdac
        set_afe_reg(0x3E, 0x000000);// init offdac
        ring_topled_off();                          // turn off top led
        offline_ledflg = OFFLINE_LED_OFF;
        g_afe.flag = SPO2_HR_INVALID;
        g_afe.offhand_flg = 1;
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
                        uint8_t Regret = currentReg(4096 + 64 + 3);
                } else {
                        uint8_t Regret = currentReg(4096 + 64);
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
    }
    dt_ofline.offline_num = 0;
//    dt_ofline.online_num = 0;
    Moveflg = g_afe.offhand_flg;
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
            offdac_led2 = 0;
            offdac_led3 = 0;
        }
        
}
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
    for(int i = 0; i < frame_length; i++)
    {
        tmp_Infra_orig = afe_bpt[i].phase1;
        tmp_Red_orig = afe_bpt[i].phase2;
        int tmpledInfra = tmp_Infra_orig;//tmp_Infra_orig - tmpledamb;
        int tmpledRed = tmp_Red_orig;//tmp_Red_orig - tmpledamb;
        int tmpledgreen = afe_bpt[i].phase3;//green or ambient

        tmpledInfra = (tmp_Infra_orig + offdc_deta*offdac_led2) * 0.1;
        tmpledRed = (tmp_Red_orig + offdc_deta*offdac_led3) * 0.1;

        tmpledInfra = (int)smoothFilter1(afe_IIRLPFilter1(tmpledInfra));
        tmpledRed = (int)smoothFilter2(afe_IIRLPFilter2(tmpledRed));

        int tmpfilter_red = (int)spo2_iirbpfilter(tmp_Red_orig, spo2_filter_x1, spo2_filter_y1);
        int tmpfilter_infra = (int)spo2_iirbpfilter(tmp_Infra_orig, spo2_filter_x2, spo2_filter_y2);
        if (RING_AFE_HRV_ON_MODE & afe_get_work_mode()) {
                tmpledgreen = (int)smoothFilter3(afe_IIRLPFilter3(tmpledgreen));
        }

        AFE44xx_SPO2_Data_buf[0] = tmpledInfra;
        AFE44xx_SPO2_Data_buf[1] = tmpledRed;

        tmpledamb = afe_bpt[i].phase4;
        dt_ofline.min_ambval = min(dt_ofline.min_ambval, tmpledamb);
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 1)
            {
                if(tmpledamb > (afe_bpt[i - 1].phase4 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase4 + OFFLINE_THRE))
                {
                    dt_ofline.offline_num++;
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
    if(!(changecnt%(60/frame_length)))
    {
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
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

            if(dc_adj.current_flg == 0 && changecnt > 10)
            {
                if(redreg >= 15 || infrareg >= 15)
                {
                    dc_adj.current_flg = 1;
                    Regret = currentReg(4096 + 64);
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
                else if(tmp_Red_orig < infrathreslow && redreg < 60)
                {
                    //infraN = (infrathreslow - AFE44xx_SPO2_Data_buf[0])/oneinfrastep;
                    Regret = currentReg(4096);
                    startchange = 1;
                    //testcu = 1;
                }
                else if(tmp_Red_orig > 2e6 && redreg > 28)
                {
                    Regret = currentReg(-4096*6);
                    startchange = 1;
                    //testcu = 1;
                }
                else if(tmp_Red_orig > infrathreshigh)
                {
                    if(redreg > 10)
                    {
                        Regret = currentReg(-4096*10);
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
                if(tmp_Infra_orig < 1.0e5 && tmp_Red_orig > infrathreslow && infrareg < 60)
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
                if((tmp_Red_orig >= infrathreslow && tmp_Red_orig <= infrathreshigh) || redreg == 60)
                {
                    if(tmp_Infra_orig > 2e6 && cuaddval >= 0)
                    {
                        Regret2 = currentReg(-64);
                        cuaddval = cuaddval -64;
                    }
                    if(cuchangeflag == 0 || cuinitflag == 0)
                    {
                        if(redreg >= 60)
                        {
                            if(abs(deta_red_infra) < onestep  || infrareg >= 59)
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
                            if(abs(deta_red_infra) < onestep || infrareg >= 59)
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
            else if (dc_adj.current_flg == 1)
            {
                if(!(changecnt%(50/frame_length)))
                {	
                    if(dc_adj.offdac_flg == 0)
                    {	
                        if(changecnt > 30)
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
                        if(changecnt > 40)
                        {	
                            set_afe_reg(0x20, 0x008016);
                            offdc_gain_flg = 1;
                        }
                    }
                    if(changecnt > 45)
                    {
                        int offdc_500k_deta = 4.15e5;
                        if(tmp_Infra_orig > 1.8e6 || tmp_Red_orig > 1.8e6)
                        {
                            offdac_led2 = offdac_led2 + tmp_Infra_orig/offdc_500k_deta + 1;
                            offdac_led3 = offdac_led3 + tmp_Red_orig/offdc_500k_deta + 1;
                            off_dac_set_circle();
                        }
                        if(tmp_Infra_orig < -1.8e6 || tmp_Red_orig < -1.8e6)
                        {
                            offdac_led2 = offdac_led2 - 8;
                            offdac_led3 = offdac_led3 - 8;
                            off_dac_set_circle();
                        }
                    }	
                    if(changecnt > 80)
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
void get_min_valid(int *input, int inlen, int *min, int *minid)
{
	*min = input[0];
	*minid = 0;
    if(inlen < 1)
    {
        return;
    }
	for (int i = 0; i < inlen; i++)
	{
		if (*min > input[i])
		{
			*min = input[i];
            *minid = i;
		}
	}
}
int test_accval;
void SpO2_Acc_Proc(acc_axis*pt)
{	
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
        
        accdata.ACC_g = (int)pow(abs((int)ACC_x*ACC_x + (int)ACC_y*ACC_y + (int)ACC_z*ACC_z), 0.5);

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
    frame_buf[7] = AlgPara.Spo2; //extend write
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
extern uint8_t afe_spo2_proc_wavelate(void)
{
    uint8_t spo2valflg = 0;
    if(LED1readyfor_calc==1 && LED2readyfor_calc == 1 )
    {
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
        	RRvar_cnt++;
        	if(RRvar_cnt%41 == 0)
        	{
                    rr_endpos = HRV_cnt - 1;
                    int inlen = MAX_HRV_CNT - acc_var_size;
                    if(rr_startpos > inlen)
                    {
                        rr_startpos = 0;
                    }
                    else
                    {
                        rr_startpos = max(rr_startpos - 2,0);
                    }
                    int ep = max(rr_endpos - 2, 0);
                    int sp = rr_startpos;
                    int tmp_rrvar = get_rrvar(HRV_vect+acc_var_size, inlen, sp, ep);
                    g_afe.rr_var = min(tmp_rrvar, 0xff);
                    rr_startpos = HRV_cnt;
                    RRvar_cnt = 0;
        	}
            }
#endif
            g_afe.Spo2show = data_compress(AlgPara.Spo2);
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
        if(breath_flg == 1)
        {
            breath_flg = 0;
            BREATH_Proc_shedule(BREATH_fifo.buf,1000,&g_afe.breathrate);
            if(g_afe.flag == SPO2_HR_INVALID | g_afe.hr == 0)
            {
                g_afe.breathrate = 0;
            }
            breathrate_buf[breathrate_cnt] = g_afe.breathrate;
            breathrate_cnt++;
            //NRF_LOG_INFO("breathrate=%d, br_cnt = %d", g_afe.breathrate, BR_cnt);
            
            if(breathrate_cnt >= MAX_BREATHRATE_BUFSIZE)
            {
                if(BR_timestamp == 0)      //add first time stamp
                {
                    BR_timestamp = UTC_getClock();
                }
                if (BR_cnt < MAX_BREATH_CNT) 
                {
                        
                    BR_vect[BR_cnt] = g_afe.breathrate;//get_br_mean(breathrate_buf,breathrate_cnt);
                    BR_cnt++;
                }
                breathrate_cnt = 0;
            }
        }
// part 6;before return;
//********recover of point******
        fifo32_recover(&spo2_red_fifo,length_recover);
        fifo32_recover(&spo2_infra_fifo,length_recover);
        fifo32_recover(&spo2_green_fifo,length_recover);
        fifo32_recover(&spo2_red_filter,length_recover);
        fifo32_recover(&spo2_infra_filter,length_recover);
    }
//  ble_send_persecond();
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
		
		int tmpacc_var = HRV_accval();
		HRV_vect[max_hrvcnt] = (char)(tmpacc_var >> 8); //add acc var 2 byte
		HRV_vect[max_hrvcnt+1] = (char)tmpacc_var;
		
		HRV_vect[0] = (char)(totalHrCnt >> 8); //add total cnt 2 byte
		HRV_vect[1] = (char)totalHrCnt;

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
	int tail_cnt = HRV_cnt - (MAX_HRV_CNT - acc_var_size);
	if(tail_cnt > 0 && tail_cnt <= 12)
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
    
    if(daily_green == NULL || wrcoefbuf == NULL || AccData == NULL || HistSpo2 == NULL || HistAcc == NULL)
        return false;
   
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
    for(int i = 0; i < frame_length; i++)
    {
        int tmpledgreen = afe_bpt[i].phase1;//green

        tmpledgreen = (int)smoothFilter3(afe_IIRLPFilter3(tmpledgreen));

        int tmpledamb = afe_bpt[i].phase1;
        //dt_ofline.min_ambval = min(dt_ofline.min_ambval, tmpledamb);
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
        {
            if (i > 0 && i < frame_length - 1)
            {
                if(tmpledamb > (afe_bpt[i - 1].phase1 + OFFLINE_THRE) && tmpledamb > (afe_bpt[i + 1].phase1 + OFFLINE_THRE))
                {
                    dt_ofline.offline_num++;
                }
            }

        }        
        fifo32_putPut(&daily_green_fifo, tmpledgreen);       
    }

    changecnt++;
#if ONHAND_Spo2Daily_Proc
    if(!(changecnt%(100/frame_length))) //1s
    {
        if(offline_ledflg == OFFLINE_LED_ON)  //when top green led is on, do offhand detect
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
        if(!(changecnt%change_step))   //turn on top green led
        {
            if(offline_ledflg == OFFLINE_LED_OFF)
            {
                ring_topled_on();
                offline_ledflg = OFFLINE_LED_ON;
            }
        }
    }
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

    wavelet_baseProc(green + Marglen, Waveletlen, slevel, elevel, wrcoefbuf);
    int* tmpbuf = (int*)wrcoefbuf;

    uint8_t vldflg;

    get_hr(tmpbuf,Waveletlen, Tcc,AlgPara->heartratehist, &(AlgPara->heartrate), &vldflg, &(AlgPara->HrCnt), AlgPara->HrVect);   

//smooth hr
    AlgPara->Sheartrate = AlgPara->heartrate;
    if (HistHrsavepos == 0)
    {
        if (vldflg <= 2)
        {
            AlgPara->Sheartrate = AlgPara->heartratehist;
        }
        else
        {
            HistHeartrate[HistHrsavepos] = AlgPara->heartrate;
            HistHrVld[HistHrsavepos] = vldflg;
            HistHrsavepos++;
        }
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
            tmphrmean = tmphrmean / tmpcnt1;

            if (vldflg < 2 || abs(tmphrmean - AlgPara->heartrate) > 15)
            {
                AlgPara->Sheartrate = AlgPara->heartratehist;
            }
            else
            {
                HistHeartrate[HistHrsavepos % HIST_HR_LEN] = AlgPara->heartrate;
                HistHrVld[HistHrsavepos % HIST_HR_LEN] = vldflg;
                HistHrsavepos++;
            }
        }
    }
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
            
        AlgPara.heartrate = (uint8_t)((AlgPara.heartrate + AlgPara.heartratehist)/2);

    //put history hr value
        AlgPara.heartratehist = AlgPara.Sheartrate;
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
            RRvar_cnt++;
            if(RRvar_cnt%41 == 0)
            {
                rr_endpos = HRV_cnt - 1;
                int inlen = MAX_HRV_CNT - acc_var_size;
                if(rr_startpos > inlen)
                {
                    rr_startpos = 0;
                }
                else
                {
                    rr_startpos = max(rr_startpos - 2,0);
                }
                int ep = max(rr_endpos - 2, 0);
                int sp = rr_startpos;
                int tmp_rrvar = get_rrvar(HRV_vect+acc_var_size, inlen, sp, ep);
                g_afe.rr_var = min(tmp_rrvar, 0xff);
                rr_startpos = HRV_cnt;
                RRvar_cnt = 0;
            }
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