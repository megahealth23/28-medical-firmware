/* Copyright (c) 2016 MegaHealth. All Rights Reserved.
 * "mring_gesture.c"
 * AUTHOR:zhao mengshou
 * DATE:2018/8/15 14:19:50
 * http://www.megahealth.cn
 *
 */

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "mring_gesture.h"
#include "../algorithm/DataProcSPO2.h"

step_para Stepara;
ACC_PARA  acc_para;
uint8_t daily_spo2_flg;
#define accbuf_length   100
//int AccBuf[accbuf_length];

//short accz_buf[gesture_accbuf_length];
short accy_buf[gesture_accbuf_length];
short accx_buf[gesture_accbuf_length];
unsigned short g_buf[step_gbuf_length];
int gnum = 0;

short angrate_ybuf[gesture_angbuf_length];
short angrate_xbuf[gesture_angbuf_length];
short angrate_zbuf[gesture_angbuf_length];
int angratenum = 0;
enum GESTURE_STATUS gest_status;
int gest_dur_num = 0;

#define MAX_ZEROS_POINT   124
FIFO16 acc_g_fifo;
float step_histrate = 0;
float MoveCnt = 0;
int16_t PeakVal[MAX_ZEROS_POINT];
uint16_t PeakvarLen[MAX_ZEROS_POINT];
uint16_t FPPLen[MAX_ZEROS_POINT];
ACC_GYRO_FIFO acc_gyro_fifo;
FIFO_acc EHR_acc_fifo;

static gesture_warning_t gesture_warning = {0};
#ifndef max
#define max(x1,x2) ((x1)>(x2)?(x1):(x2))
#define min(x1,x2) ((x1)<(x2)?(x1):(x2))
#endif

int gesture_on = 0;
int ACC_STEP_SAMPLERATE = 50;
short tmp_acc_inbuf[7] = {0};
double tmp_acc_outbuf[7] = {0};
double EHR_acc_filter(short *input, double *a, double *b, int cofflen, double *output)
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

void acc_gyrofifo_init(ACC_GYRO_FIFO*fifo, int size)
{
    fifo->accx = accx_buf;
    fifo->accy = accy_buf;
//    fifo->accz = accz_buf;
    fifo->gyrox = angrate_xbuf;
    fifo->gyroy = angrate_ybuf;
    fifo->gyroz = angrate_zbuf;
    fifo->flags = 0;          
    fifo->free = size;
    fifo->size = size;
    fifo->putP = 0;                   
    fifo->getP = 0;                   

    return;
}

int acc_gyrofifo_putPut(ACC_GYRO_FIFO *fifo, int16_t data_accx, int16_t data_accy, int16_t data_accz, int16_t angratex, int16_t angratey, int16_t angratez)
{
	if (fifo->free <= 0){
		fifo->flags |= STEP_FLAGS_OVERRUN;
		return -1;
	}
    fifo->accx[fifo->putP] = data_accx;
	fifo->accy[fifo->putP] = data_accy;
//    fifo->accz[fifo->putP] = data_accz;
    fifo->gyrox[fifo->putP] = angratex;
    fifo->gyroy[fifo->putP] = angratey;
    fifo->gyroz[fifo->putP] = angratez;
	fifo->putP++;

	if(fifo->putP >= fifo->size){
		fifo->putP = 0;
	}
	fifo->free--;

	return 0;
}

int acc_gyrofifo_status(ACC_GYRO_FIFO *fifo)
{
    if(fifo->free < 0)
        return fifo->size;
    else if(fifo->free > fifo->size)
        return 0;
    else
        return fifo->size - fifo->free;
}

int acc_gyrofifo_recover_wavelet(ACC_GYRO_FIFO *fifo,int num)
{
    fifo->free = gesture_clc_num;
    fifo->putP = 0;
    for(unsigned int i=0;i<num;i++)
    {
        fifo->accx[i] = fifo->accx[i+gesture_clc_num];
        fifo->accy[i] = fifo->accy[i+gesture_clc_num];
//        fifo->accz[i] = fifo->accz[i+gesture_clc_num];
        fifo->gyrox[i] = fifo->gyrox[i+gesture_clc_num];
        fifo->gyroy[i] = fifo->gyroy[i+gesture_clc_num];
        fifo->gyroz[i] = fifo->gyroz[i+gesture_clc_num];
        fifo->putP++;
    }
    return 0;
}



void gesture_init(void)
{
    gest_status = NONE;
    acc_gyrofifo_init(&acc_gyro_fifo, gesture_accbuf_length);
	fifo16_init(&acc_g_fifo, gesture_accbuf_length, g_buf);
    gest_dur_num = 0;
}

void steps_init(void)
{
	memset((char *)&Stepara, 0, sizeof(step_para));
	memset((char *)&acc_para, 0, sizeof(ACC_PARA));   
    fifo16_init(&acc_g_fifo, step_gbuf_length, g_buf);
    daily_spo2_flg = 0;
}

int get_varval(uint16_t *input, int inlen)
{
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

uint16_t get_mean(uint16_t *input, int inlen)
{
	uint32_t sum = 0;
	for (int i = 0; i < inlen; i++)
	{
		sum += input[i];
	}
	if (inlen <= 0)
	{
		return 0;
	}
	return sum / inlen;
}

uint16_t get_max(uint16_t *input, int inlen)
{
	uint16_t max = 0;
	for (int i = 0; i < inlen; i++)
	{
		if (input[i] > max)
		{
			max = input[i];
		}
	}
	return max;
}

uint16_t get_min(uint16_t *input, int inlen)
{
	uint16_t min = 10000;
	for (int i = 0; i < inlen; i++)
	{
		if (input[i] < min)
		{
			min = input[i];
		}
	}
	return min;
}

uint16_t get_MaxMin_val(uint16_t *input, int inlen)
{
	uint16_t maxval = 0;
	uint16_t minval = 10000;
	for (int i = 0; i < inlen; i++)
	{
		if (input[i] > maxval)
		{
			maxval = input[i];
		}
		else
		{
			if (input[i] < minval)
			{
				minval = input[i];
			}
		}
	}
	return maxval - minval;
}

static void get_minval(int16_t* input, int len, int16_t* minval, int *minid)
{
	int16_t tmpval = input[0];
	int tmpid = 0;
	*minval = tmpval;
	*minid = tmpid;

	if(len < 0) 
	{
		return;
	}
	for(int i = 0; i < len; i++)
	{
		if(input[i] < tmpval)
		{
			tmpval = input[i];
			tmpid = i;
		}
	}
	*minval = tmpval;
	*minid = tmpid;
}
static void get_maxval(int16_t* input, int len, int16_t* maxval, int *maxid)
{
	int16_t tmpval = input[0];
	int tmpid = 0;
	*maxval = tmpval;
	*maxid = tmpid;

	if(len < 0) 
	{
		return;
	}
	for(int i = 0; i < len; i++)
	{
		if(input[i] > tmpval)
		{
			tmpval = input[i];
			tmpid = i;
		}
	}
	*maxval = tmpval;
	*maxid = tmpid;
}

uint8_t testwave_acc = 0;
int testmaxy = 0;
int testminy = 0;
int test_angx = 0;
uint8_t gesture_wave_updown(short *acc_y, short *ang_x, int len)
{
    testwave_acc = 1;
    int pre_angx_thre = 200;
    int thre_maxval = 350;
    int thre_minval = -400;
    int thre_min_dur = 25;
    int thre_max_dur = 120;
    int thre_angx = 350;
    int thre_angx_total = 700;
    int startpt = 0;
    int endpt = 0;
    int zero1 = 0;
    int zero2 = 0;
    uint8_t waveflg = 0;
    int angx_dur = 0;
    int16_t maxval = 0;
    int maxid = 0;
    get_maxval(ang_x, len, &maxval, &maxid);
    testmaxy = ang_x[0];
    test_angx = abs(acc_y[0]/100);
    if(maxval < pre_angx_thre)
    {
        return 0;
    }
    for (int i = 1; i < (len-100); i++)
    {
        if(acc_y[i] <= thre_maxval && acc_y[i+1] > thre_maxval)
        {
            zero1 = i;
            testwave_acc = 2;
            for (int j = zero1; (j < zero1 + 100) && j < len; j++)
            {
                if(acc_y[j] >= thre_maxval && acc_y[j+1] < thre_maxval)
                {
                    zero2 = j;
                    waveflg = 1;
                    break;
                }
            }
        }
        if(waveflg == 1)
        {
            testwave_acc = 3;
            for(int nm = zero1; (nm > zero1-25) && nm > 0; nm--)
            {
                if(acc_y[nm] < thre_minval)
                {
                    startpt = nm;
                    break;
                }
            }
            for(int nm = zero2; (nm < zero2+25) && nm < len; nm++)
            {
                if(acc_y[nm] < thre_minval)
                {
                    endpt = nm;
                    break;
                }
            }
            if(startpt > 0 && endpt > 0)
            {
                testwave_acc = 4;
                int duration = endpt - startpt;
                if(startpt > 0 && duration > thre_min_dur && duration < thre_max_dur)
                {
                    testwave_acc = 5;
    //                int max_accz = 0;
                    int16_t max_angx = ang_x[startpt];
                    int16_t min_angx = ang_x[startpt];
                    int tmpsp = max(startpt - 20, 0);
                    int tmpend = min(endpt + 20, len);
                    for(int nnm = tmpsp; nnm < tmpend; nnm++)
                    {
                        if(max_angx < ang_x[nnm]) {
                            max_angx = ang_x[nnm];
                        }
                        if(min_angx > ang_x[nnm]) {
                            min_angx = ang_x[nnm];
                        }
                    }
                    //testmaxy = max_angx;
                    testminy = min_angx;
                    
                    angx_dur = max_angx - min_angx;
                    if((max_angx > thre_angx || min_angx < -1*thre_angx) && angx_dur > thre_angx_total)
                    {
                        testwave_acc = 6;
                        waveflg = 0;
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}

uint8_t testrotate_acc = 0;
int testmaxval = 0;
int testminval = 0;
uint8_t gesture_rotate(short *input, short *accz, int len)
{
    testrotate_acc = 1;
    int thre_rate_y = 400;
    int thre_time_max = 120;
    int thre_time_min = 20;
    
    short tmpmaxval;
    int maxindex = 0;
    short tmpminval;
    int minindex = 0;
    tmpmaxval = input[0];
    tmpminval = input[0];
    for (int i = 1; i < len; i++)
    {
        if(input[i] > tmpmaxval)
        {
            tmpmaxval = input[i];
            maxindex = i;
        }
        if(input[i] < tmpminval)
        {
            tmpminval = input[i];
            minindex = i;
        }
    }
    if(tmpmaxval > thre_rate_y && tmpminval < -1*thre_rate_y)
    {
        testrotate_acc = 2;
        int amgy_dur = abs(maxindex - minindex);
        if(amgy_dur < thre_time_max && amgy_dur > thre_time_min)
        {
            testrotate_acc = 3;
            return 1;
        }
    }

    return 0;
}
#if 0
uint8_t testknock_acc = 0;
uint8_t knock_peaknum = 0;
uint8_t knock_z_peaknum = 0;
uint8_t test_knock_flg = 0;
uint8_t testproc = 0;
int testabuf1 = 0;
static uint8_t gesture_knock(unsigned short*input, int len)
{
    testknock_acc = 1;
    uint8_t knockflg = 0;
    int max_accval = 3500;
    //int thre_valley = 700;
    int thre_deta = 2500;
    int thre_pv = 3000;
    int thre_valleyindex = 10;
    int deta_max = 60;
    int deta_min = 20;
    int peaknum = 0;
    int zbuf_peaknum = 0;
    int *peak_index = NULL;
    int *peak_val = NULL;
    peak_index = (int*)malloc(40*sizeof(int));
    peak_val = (int*)malloc(40*sizeof(int));
    int ths_max_angz = 200;
    int ths_min_angz = -200;
    //find peak from g buf
    int i = 2;
    while(i < len-2) //(int i = 2; i < len-2;i++)
    {
        int tmpdeta1 = input[i] - input[i-2];
        int tmpdeta2 = input[i] - input[i+2];
        if(input[i] > max_accval && (tmpdeta1 > thre_deta && tmpdeta2 > thre_deta))
        {
            peak_index[peaknum] = i;
            peak_val[peaknum] = input[i];
            peaknum++;
            i += deta_min;
        }
        else
        {
            i++;
        }
    }
    //find peak from z buf
 
    knock_peaknum = peaknum;
    //knock_z_peaknum = zbuf_peaknum;
    testproc = peaknum;
    //if(peaknum >= 3 && zbuf_peaknum >= 3)
    if(peaknum >= 3)
    {
        testknock_acc = 2;
        for(int j = 0; j < peaknum-2; j++)
        {
            int deta1 = peak_index[j+1] - peak_index[j];
            int deta2 = peak_index[j+2] - peak_index[j+1];
            if(deta1 > deta_min && deta1 < deta_max && deta2 > deta_min && deta2 < deta_max)
            {
                testknock_acc = 3;
                int vally1 = get_min(input + peak_index[j+1] - thre_valleyindex, thre_valleyindex);
                int vally2 = get_min(input + peak_index[j+2] - thre_valleyindex, thre_valleyindex);
                if(peak_val[j+1] - vally1 > thre_pv && peak_val[j+2] - vally2 > thre_pv)
                {
                    testknock_acc = 4;
                        knockflg = 1;
                        gest_status = TAP_THREETIMES;
                        if(peak_index != NULL) {
							free(peak_index);
							peak_index = NULL;
						}
                        if(peak_val != NULL) {
							free(peak_val);
							peak_val = NULL;
						}
                        break;
//                    }
                }
            }
        }
    }
    if(peak_index != NULL) free(peak_index);
    if(peak_val != NULL) free(peak_val);
    if(knockflg == 1)
    {
        return 1;
    }
    return 0;
}

int test_wavearound = 0;
uint8_t gesture_wave_around(short *acc_x, short *ang_z, int len)
{
	test_wavearound = 1;
    int thre_maxval = 100;
    int thre_minval = -400;
	int ths_max_angz = 300;
    int ths_min_angz = -300;
    //int ths_angz = 800;
	int ths_dur_max = 60;
	int ths_dur_min = 25;
	int ths_max_accx = 1000;
	//int ths_min_accx = -1100;
	int16_t tmpmaxval = 0;
	int tmpmaxid = 0;
	int16_t tmpminval = 0;
	int tmpminid = 0;
	int startpt = 0;
    int endpt = 0;
	//int pk1_id;
	//int pk2_id;
    //int zero1 = 0, zero2 = 0;
    uint8_t waveflg = 0;
    for(int i = 1; i < (len-60); i++)
    {
        if(ang_z[i] >= ang_z[i-1] && ang_z[i] > ang_z[i+1] && ang_z[i] > ths_max_angz)
        {
            test_wavearound = 2;
            for(int j = i; j < len-1; j++)
            {
                if(ang_z[j] <= ang_z[j-1] && ang_z[j] < ang_z[j+1] && ang_z[j] < ths_min_angz)
                {
                    test_wavearound = 3;
                    int angz_dur_time = j-i;
                    if(angz_dur_time > ths_dur_min && angz_dur_time < ths_dur_max)
                    {
                        test_wavearound = 4;
                        int tmpsp = max(0,i-20);
                        int tmpep = min(j+20,len);
                        int16_t maxval = 0; int16_t minval = 0;
                        int maxid = 0; int minid = 0;
                        get_maxval(acc_x + tmpsp, tmpep - tmpsp, &maxval, &maxid);
                        get_minval(acc_x + tmpsp, tmpep - tmpsp, &minval, &minid);
                        int acc_x_dur_val = maxval - minval;
                        if(acc_x_dur_val > ths_max_accx)
                        {
                            test_wavearound = 5;
                            waveflg = 1;
                            return 1;
                        }
                    }
                }
            }
            if(waveflg == 1)
            {
                return 1;
            }
        }
    }

	return 0;
}

int test_draw = 0;
uint8_t test_dur = 0;
int max_angz = 0;
int min_angz = 0;
uint8_t gesture_draw_circle(short *ang_x, short *ang_z, short *acc_z, int len)
{
    test_draw = 1;
    int ths_max_angx = 300;
    int ths_min_angx = -260;
    int ths_max_angz = 200;
    int ths_min_angz = -200;
    int ths_max_accz = 200;
    int ths_dur_max = 50;
    int ths_dur_min = 20;
    int16_t maxval = 0;
    int maxid = 0;
    int16_t minval = 0;
    int minid = 0;
    int startpt = 0;
    int endpt = 0;
    for (int i = 1; i < (len-20); i++)
    {
        if(ang_x[i] > ths_max_angx && ang_x[i] > ang_x[i-1] && ang_x[i] >= ang_x[i+1])
        {
            test_draw = 2;
            maxval = ang_x[i];
            maxid = i;
            int tmpep = min(maxid+10,len-1);
            for(int j = i; j < tmpep; j++)
            {
                if(maxval < ang_x[j])
                {
                    maxval = ang_x[j];
                    maxid = j;
                }
            }
            int tmpsp = max(maxid-50,0);
            tmpep = min(maxid+50,len-1);
            get_minval(ang_x + tmpsp, tmpep-tmpsp, &minval, &minid);
            if(minval < ths_min_angx)
            {
                test_draw = 3;
                if(maxid > minid)
                {
                    startpt = minid;
                    endpt = maxid;
                }
                else
                {
                    startpt = maxid;
                    endpt = minid;
                }
                int duration = endpt - startpt;
                test_dur = duration;
                if(duration > ths_dur_min && duration < ths_dur_max)
                {
                    test_draw = 4;
                    tmpsp = max(startpt-20,0);
                    tmpep = min(endpt+20,len-1);
                    get_maxval(ang_z + tmpsp, tmpep-tmpsp, &maxval, &maxid);
                    get_minval(ang_z + tmpsp, tmpep-tmpsp, &minval, &minid);
                    max_angz = maxval;
                    min_angz = minval;
                    if(maxval > ths_max_angz && minval < ths_min_angz)
                    {
                        test_draw = 5;
                        get_maxval(acc_z + tmpsp, tmpep-tmpsp, &maxval, &maxid);
                        if(maxval < ths_max_accz)
                        {
                            test_draw = 6;
                        }
                        return 1;
                    }
                }
            }
        }
        
    }
    return 0;
}

int status_flg = 0;
void gesture_proc(void)
{
    if(gest_status == 0)
    {
        uint8_t knock_reg = gesture_knock(acc_g_fifo.buf, gesture_accbuf_length);
        if(gest_status == TAP_THREETIMES)
        {
//            gest_status = TAP_THREETIMES;
            int reg = gesture_status_update(gest_status);
//            memset((char *)&accz_buf, 0, gesture_accbuf_length*sizeof(short));
        }
        else
        {
            uint8_t tmprotatflg = gesture_rotate(angrate_ybuf, angrate_zbuf, gesture_angbuf_length);
            if(tmprotatflg == 1)
            {
                gest_status = ROTATE_OUTANDIN;
                int reg = gesture_status_update(gest_status);
//                memset((char *)&angrate_ybuf, 0, gesture_angbuf_length*sizeof(short));
            }
            else
            {
                uint8_t tmpwaveflg = gesture_wave_updown(accy_buf, acc_gyro_fifo.gyrox, gesture_angbuf_length);
                if(tmpwaveflg == 1)
                {
                    gest_status = WAVE_UP_DOWN;
                    int reg = gesture_status_update(gest_status);
//                    memset((char *)&accy_buf, 0, gesture_angbuf_length*sizeof(short));
                }
				else
				{
//                    DEBUG_LOG("x0=%d,x100=%d,x200=%d\r\n",accx_buf[0],accx_buf[100],accx_buf[200]);
//                    DEBUG_LOG("z0=%d,z100=%d,z200=%d\r\n",angrate_zbuf[0],angrate_zbuf[100],angrate_zbuf[200]);
					uint8_t tmpwave_arond_flg = gesture_wave_around(accx_buf, angrate_zbuf, gesture_angbuf_length);
					if(tmpwave_arond_flg == 1)
					{
						gest_status = WAVE_AROUND;
						int reg = gesture_status_update(gest_status);
					}
//                    else
//                    {
//                        if(gesture_draw_circle(angrate_xbuf, angrate_zbuf, accz_buf, gesture_angbuf_length))
//                        {
//                            gest_status = DRAW_CIRCLE;
//                            int reg = gesture_status_update(gest_status);
//                        }
//                    }
				}
            }
        }
    }
    if(gest_status != 0)
    {
        if(gest_dur_num == 0) {
            state_entry.gesture_on_wait = swtimer_getSec();
        }
        gest_dur_num++;
    }
    if(gest_dur_num >= 3)
    {
        gest_status = 0;
        gest_dur_num = 0;
    }

}
#endif

int testflg = 0;
void Count_Move(uint16_t *input, int inlen, float histRate, int samplerate, float *Rate)
{
    testflg++;
	uint16_t inmean;
	inmean = get_mean(input, inlen);

	int VldThreshold = 60;
	uint16_t tmax[10];

	uint8_t procsec = inlen / samplerate;

	if (histRate > 0)
	{
		VldThreshold = VldThreshold * 0.8;
	}

	uint8_t tcnt = 0;
	for (int i = 0; i < inlen; i+=100)
	{
		tmax[tcnt] = get_MaxMin_val(input + i, 100);
		tcnt++;
	}

	uint16_t maxval = 0;
	uint16_t tvldcnt = 0;
	for (int i = 0; i < tcnt; i++)
	{
		if (tmax[i] > maxval)
		{
			maxval = tmax[i];
		}
		if (tmax[i] > VldThreshold)
		{
			tvldcnt++;
		}
	}
	if ((maxval < VldThreshold * 1.5 || tvldcnt < tcnt * 0.7) && histRate == 0)
	{
		*Rate = 0;
		return;
	}

	uint16_t pid = 0;
	uint8_t peakcnt = 0;
	uint16_t MaxSteplen = samplerate / LOW_FREQ;
	uint16_t MinSteplen = samplerate / HIGH_FREQ;
	for (int i = 0; i < inlen - 1; i++)
	{
		if (input[i] < inmean && input[i + 1] >= inmean && i - pid > 2)
		{
			if (pid > 0)
			{
				PeakvarLen[peakcnt] = i - pid;
				PeakVal[peakcnt] = get_min(input + pid, i - pid + 1) - inmean;
				peakcnt++;
				if (peakcnt >= MAX_ZEROS_POINT)
				{
					*Rate = 0;
					return;
				}
			}
			pid = i;
		}
		if (input[i] > inmean && input[i + 1] <= inmean && i - pid > 2)
		{
			if (pid > 0)
			{
				PeakvarLen[peakcnt] = i - pid;
				PeakVal[peakcnt] = get_max(input + pid, i - pid + 1) - inmean;
				peakcnt++;
				if (peakcnt >= MAX_ZEROS_POINT)
				{
					*Rate = 0;
					return;
				}
			}
			pid = i;
		}	
	}

	int16_t Psumval = 0;
	int16_t Nsumval = 0;
	int16_t Stepsum = 0;
	uint8_t Pcnt = 0;
	uint8_t Ncnt = 0;
	uint8_t stepcnt = 0;
	for (int i = 0; i < peakcnt; i++)
	{
		if (PeakVal[i] > VldThreshold)
		{
			Psumval += PeakVal[i];
			Pcnt++;

			Stepsum += PeakvarLen[i];
			stepcnt++;
		}
		else
		{
			if (PeakVal[i] < -VldThreshold)
			{
				Nsumval += PeakVal[i];
				Ncnt++;

				Stepsum += PeakvarLen[i];
				stepcnt++;
			}
		}
	}

	if (Pcnt == 0 || Ncnt == 0)
	{
		*Rate = 0;
		return;
	}
	uint16_t PVldth = MAX(Psumval / Pcnt * 0.3, VldThreshold*0.8);
	int16_t NVldth = MIN(Nsumval / Ncnt * 0.3, -VldThreshold*0.8);

	uint16_t tmean = get_mean(PeakvarLen, peakcnt);

	if ((histRate == 0 && (tmean < MinSteplen * 0.4 || Stepsum / stepcnt < MinSteplen * 0.6)) || stepcnt < procsec * 1.5 || stepcnt > procsec * 8)
	{
		*Rate = 0;
		return;
	}

	tcnt = 0;
	int i = 0;
	while (i < peakcnt - 2)
	{
		if (PeakVal[i] > PVldth)
		{
			if (PeakVal[i + 1] < NVldth && PeakVal[i + 2] > PVldth)
			{
				if (PeakvarLen[i] < MinSteplen * 1.2)
				{
					if (PeakvarLen[i] + PeakvarLen[i + 1] < MaxSteplen * 1.1)
					{
						FPPLen[tcnt++] = PeakvarLen[i] + PeakvarLen[i + 1];
					}
				}
				else
				{
					FPPLen[tcnt++] = PeakvarLen[i];
				}

				i = i + 1;
			}
			else
			{
				uint16_t tmpsum = PeakvarLen[i] + PeakvarLen[i + 1];
				uint8_t nflg = 0;
				uint8_t vflg = 0;
				int j = i + 2;
				for (; j < MIN(peakcnt, i + 7); j++)
				{
					if (PeakVal[j] <= NVldth)
					{
						nflg = 1;
					}
					if (PeakVal[j] >= PVldth && nflg == 1)
					{
						vflg = 1;
						break;
					}
					tmpsum += PeakvarLen[j];
					if (tmpsum > MaxSteplen)
					{
						break;
					}
				}

				if (vflg == 1 && tmpsum < MaxSteplen)
				{
					FPPLen[tcnt++] = tmpsum;
				}

				i = j - 1;
			}
		}
		else if (PeakVal[i] < NVldth)
		{
			if (PeakVal[i + 1] > PVldth && PeakVal[i + 2] < NVldth)
			{
				if (PeakvarLen[i] < MinSteplen * 1.2)
				{
					if (PeakvarLen[i] + PeakvarLen[i + 1] < MaxSteplen * 1.1)
					{
						FPPLen[tcnt++] = PeakvarLen[i] + PeakvarLen[i + 1];
					}
				}
				else
				{
					FPPLen[tcnt++] = PeakvarLen[i];
				}
				i = i + 1;
			}
			else
			{
				uint16_t tmpsum = PeakvarLen[i] + PeakvarLen[i + 1];
				uint8_t pflg = 0;
				uint8_t vflg = 0;
				int j = i + 2;
				for (; j < MIN(peakcnt, i + 7); j++)
				{
					if (PeakVal[j] >= PVldth)
					{
						pflg = 1;
					}
					if (PeakVal[j] <= NVldth && pflg == 1)
					{
						vflg = 1;
						break;
					}
					tmpsum += PeakvarLen[j];
					if (tmpsum > MaxSteplen)
					{
						break;
					}
				}

				if (vflg == 1 && tmpsum < MaxSteplen)
				{
					FPPLen[tcnt++] = tmpsum;
				}
				i = j - 1;
			}
		}
		else
		{
			i++;
		}
        if (tcnt >= MAX_ZEROS_POINT)
        {
            *Rate = 0;
		    return;
        }
	}
	if (tcnt < procsec * 0.8)
	{
		*Rate = 0;
		return;
	}
	
	uint8_t Fvcnt = 0;
	float tsum = 0;
	float tsumt = 0;
	for (int i = 0; i < tcnt; i++)
	{
		if (FPPLen[i] > MinSteplen && FPPLen[i] < MaxSteplen)
		{
			tsum += FPPLen[i];
			Fvcnt++;
		}
		tsumt += FPPLen[i];
	}
	tsumt = tsumt / tcnt;
	if (Fvcnt <= procsec * 0.6 || tsumt < MinSteplen || tsumt > MaxSteplen)
	{
		*Rate = 0;
		return;
	}
	float tmprate = 0.8;
	if (histRate > 0)
	{
		tmprate = 0.4;
	}

	float Fvmean = tsum / Fvcnt;
	if (Fvcnt < inlen/ Fvmean*tmprate)
	{
		*Rate = 0;
		return;
	}

	uint16_t Maxstep = get_max(FPPLen, tcnt);
	uint16_t Minstep = get_min(FPPLen, tcnt);

	if (Maxstep < Minstep * 2)
	{
		*Rate = 1.00f * samplerate / tsumt;
		return;
	}

	uint8_t Maxth = Maxstep * 0.3;
	uint8_t Minth = Minstep * 2;
	float Maxmean = 0;
	float Minmean = 0;
	uint8_t Maxcnt = 0;
	uint8_t Mincnt = 0;
	for (int i = 0; i < tcnt; i++)
	{
		if (FPPLen[i] >= Maxth)
		{
			Maxmean += FPPLen[i];
			Maxcnt++;
		}
		if (FPPLen[i] < Minth)
		{
			Minmean += FPPLen[i];
			Mincnt++;
		}
	}

    if (Mincnt == 0|| Maxcnt == 0)
    {
 		*Rate = 1.00f * samplerate / tsumt;
		return;       
    }
	Minmean = Minmean / Mincnt;
	Maxmean = Maxmean / Maxcnt;

	if (Maxcnt > Mincnt)
	{
		*Rate = 1.00f * samplerate / Maxmean;
	}
	else
	{
		*Rate = 1.00f * samplerate / Minmean;
		if (100 / Minmean > 3)
		{
			*Rate = 2.00f * samplerate / (Maxmean + Minmean);
		}
	}
	return;
}


void smooth_short_ACC_period_data(short* period_data, int smooth_span, float* period_data_smooth)
{

	int sum_num = 0;
	int add_num = 0;
	int add_data_num_half = 0;
	double sum = 0.0;
	if (smooth_span % 2 == 1)
	{
		add_data_num_half = (smooth_span - 1) / 2;
	}
	else
	{
		add_data_num_half = (smooth_span - 2) / 2;
	}

	for (int frame_data_num = 0; frame_data_num < STEP_COUNT_PERIOD; frame_data_num++)
	{

		add_num = add_data_num_half;
		if (frame_data_num < add_data_num_half)
		{
			add_num = frame_data_num;
		}
		else if ((STEP_COUNT_PERIOD - 1 - frame_data_num) < add_data_num_half)
		{
			add_num = STEP_COUNT_PERIOD - frame_data_num - 1;
		}
		sum = 0.0;
		for (sum_num = frame_data_num - add_num; sum_num <= frame_data_num + add_num; sum_num++)
		{
			sum += period_data[sum_num];
		}
		period_data_smooth[frame_data_num] = sum / (add_num * 2 + 1);

	}

}
void smooth_ACC_period_data(uint16_t* period_data, int smooth_span, float* period_data_smooth)
{

	int sum_num = 0;
	int add_num = 0;
	int add_data_num_half = 0;
	double sum = 0.0;
	if (smooth_span % 2 == 1)
	{
		add_data_num_half = (smooth_span - 1) / 2;
	}
	else
	{
		add_data_num_half = (smooth_span - 2) / 2;
	}

	for (int frame_data_num = 0; frame_data_num < STEP_COUNT_PERIOD; frame_data_num++)
	{

		add_num = add_data_num_half;
		if (frame_data_num < add_data_num_half)
		{
			add_num = frame_data_num;
		}
		else if ((STEP_COUNT_PERIOD - 1 - frame_data_num) < add_data_num_half)
		{
			add_num = STEP_COUNT_PERIOD - frame_data_num - 1;
		}
		sum = 0.0;
		for (sum_num = frame_data_num - add_num; sum_num <= frame_data_num + add_num; sum_num++)
		{
			sum += period_data[sum_num];
		}
		period_data_smooth[frame_data_num] = sum / (add_num * 2 + 1);

	}

}
void mean_get(float *data,int dataLen,float* meanValue)
{
	float data_sum = 0;
	for (int num = 0;num < dataLen;num++)
	{
		data_sum = data_sum + data[num];
	}
	*meanValue = data_sum/ dataLen;

}

float var_get(float* data, int dataLen)
{
	float all_sum = 0;
	float mean_value = 0;
	float var_value = 0;
	float all_data_sum = 0;
	for (int num = 0;num < dataLen;num++)
	{
		all_sum = all_sum + data[num];
	}
	mean_value = (short)(all_sum / dataLen);
	for (int num = 0; num < dataLen; num++)
	{
		all_data_sum = all_data_sum + pow(data[num]- mean_value,2);
	}
	var_value = all_data_sum / (dataLen - 1);
	return var_value;
}

void max_get(float* data,int dataLen,float* maxValue,int* maxValuePos)
{
	*maxValue = data[0];
	*maxValuePos = 0;
	for (int num = 0;num < dataLen;num++)
	{
		if (*maxValue < data[num])
		{
			*maxValue = data[num];
			*maxValuePos = num;
		}

	}
}
void min_get(float* data, int dataLen, float* minValue, int* minValuePos)
{
	*minValue = data[0];
	*minValuePos = 0;
	for (int num = 0;num < dataLen;num++)
	{
		if (*minValue > data[num])
		{
			*minValue = data[num];
			*minValuePos = num;
		}
	}
}



void cross_mean_cnt_get(float* data, int dataLen, float meanValue,int *cross_mean_cnt)
{
	for (int num = 0;num < dataLen - 1;num++)
	{
		if (data[num] < meanValue && data[num + 1] >= meanValue)
		{
			*cross_mean_cnt = *cross_mean_cnt + 1;
		}
		if (data[num] >= meanValue && data[num + 1] < meanValue)
		{
			*cross_mean_cnt = *cross_mean_cnt + 1;
		}

	}


}

void Count_Move_get_new(uint16_t* input, int inlen, int* period_step_count)
{
	float period_data_smooth[STEP_COUNT_PERIOD] = {0};
	float mean_value = 0;
	float mean_pos_DownToUp[STEP_COUNT_PERIOD / 2] = {0};
	float mean_pos_UpToDown[STEP_COUNT_PERIOD / 2] = {0};
	int mean_pos_DownToUp_cnt = 0;
	int mean_pos_UpToDown_cnt = 0;

	MAX_PARA period_max_peak = {0};
	MIN_PARA period_min_peak = { 0 };

	float maxPeakValueMax = 0 ;
	int maxPeakValueMaxPos = 0;
	float minPeakValueMin = 0 ;
	int minPeakValueMinPos = 0;
	float secondMaxPeakValueMax = 0;

	smooth_ACC_period_data(input, SMOOTH_SPAN, period_data_smooth);

	mean_get(period_data_smooth, STEP_COUNT_PERIOD, &mean_value);
	mean_value = mean_value - 100;
	printf("mean_value: %f\n", mean_value);
	for (int num_1 = 0; num_1 < STEP_COUNT_PERIOD - 1; num_1++)
	{
		if (num_1 == 0)
		{
			mean_pos_DownToUp[mean_pos_DownToUp_cnt] = 0;
			mean_pos_DownToUp_cnt++;
		}
		if (period_data_smooth[num_1] < mean_value && period_data_smooth[num_1 + 1] >= mean_value)
		{
			mean_pos_DownToUp[mean_pos_DownToUp_cnt] = num_1;
			mean_pos_DownToUp_cnt++;
		}
		if (period_data_smooth[num_1] >= mean_value && period_data_smooth[num_1 + 1] < mean_value)
		{
			mean_pos_UpToDown[mean_pos_UpToDown_cnt] = num_1;
			mean_pos_UpToDown_cnt++;
		}

	}

	for (int num_2 = 0;num_2 < mean_pos_DownToUp_cnt;num_2++)
	{
		for (int num_3 = 0;num_3 < mean_pos_UpToDown_cnt;num_3++)
		{
			if (mean_pos_DownToUp[num_2] < mean_pos_UpToDown[num_3] && mean_pos_UpToDown[num_3] - mean_pos_DownToUp[num_2] > ACC_SAMPLERATE/5)
			{
				memset(&period_max_peak,0,sizeof(MAX_PARA));
				memset(&period_min_peak, 0, sizeof(MIN_PARA));
				for (int num_4 = mean_pos_DownToUp[num_2];num_4 < mean_pos_UpToDown[num_3];num_4++)
				{
					if (num_4 > 2 && num_4 < STEP_COUNT_PERIOD - 2)
					{
						if (period_data_smooth[num_4] > period_data_smooth[num_4 - 1] && period_data_smooth[num_4] > period_data_smooth[num_4 - 2] && period_data_smooth[num_4] > period_data_smooth[num_4 - 3] && period_data_smooth[num_4] > period_data_smooth[num_4 + 1] && period_data_smooth[num_4] > period_data_smooth[num_4 + 2] && period_data_smooth[num_4] > period_data_smooth[num_4 + 3])
						{
							period_max_peak.maxValue[period_max_peak.maxValueCnt] = period_data_smooth[num_4];
							period_max_peak.maxValuePos[period_max_peak.maxValueCnt] = num_4;
							period_max_peak.maxValueCnt++;
						}
						if (period_data_smooth[num_4] < period_data_smooth[num_4 - 1] && period_data_smooth[num_4] < period_data_smooth[num_4 - 2] && period_data_smooth[num_4] < period_data_smooth[num_4 - 3] && period_data_smooth[num_4] < period_data_smooth[num_4 + 1] && period_data_smooth[num_4] < period_data_smooth[num_4 + 2] && period_data_smooth[num_4] < period_data_smooth[num_4 + 3])
						{
							period_min_peak.minValue[period_min_peak.minValueCnt] = period_data_smooth[num_4];
							period_min_peak.minValuePos[period_min_peak.minValueCnt] = num_4;
							period_min_peak.minValueCnt++;
						}

					}
					if ((num_4 <= 2 && num_4 > 0) || (num_4 >= STEP_COUNT_PERIOD - 2 && num_4 < STEP_COUNT_PERIOD - 1))
					{
						if (period_data_smooth[num_4] > period_data_smooth[num_4 - 1] && period_data_smooth[num_4] > period_data_smooth[num_4 + 1])
						{
							period_max_peak.maxValue[period_max_peak.maxValueCnt] = period_data_smooth[num_4];
							period_max_peak.maxValuePos[period_max_peak.maxValueCnt] = num_4;
							period_max_peak.maxValueCnt++;
						}
						if (period_data_smooth[num_4] < period_data_smooth[num_4 - 1] && period_data_smooth[num_4] < period_data_smooth[num_4 + 1])
						{
							period_min_peak.minValue[period_min_peak.minValueCnt] = period_data_smooth[num_4];
							period_min_peak.minValuePos[period_min_peak.minValueCnt] = num_4;
							period_min_peak.minValueCnt++;
						}

					}




				}

				if (period_max_peak.maxValueCnt >= 1 && period_min_peak.minValueCnt >= 1)
				{
					max_get(period_max_peak.maxValue, period_max_peak.maxValueCnt,&maxPeakValueMax,&maxPeakValueMaxPos);
					min_get(period_min_peak.minValue, period_min_peak.minValueCnt,&minPeakValueMin,&minPeakValueMinPos);

					if (period_max_peak.maxValuePos[maxPeakValueMaxPos] > period_min_peak.minValuePos[minPeakValueMinPos])
					{
						for (int peakMaxNum = 0; peakMaxNum < period_max_peak.maxValueCnt; peakMaxNum++)
						{
							if (secondMaxPeakValueMax < period_max_peak.maxValue[peakMaxNum] && period_max_peak.maxValue[peakMaxNum] != maxPeakValueMax && period_max_peak.maxValuePos[peakMaxNum] < period_min_peak.minValuePos[minPeakValueMinPos])
							{
								secondMaxPeakValueMax = period_max_peak.maxValue[peakMaxNum];
							}
						}
					}
					if (period_max_peak.maxValuePos[maxPeakValueMaxPos] < period_min_peak.minValuePos[minPeakValueMinPos])
					{
						for (int peakMaxNum = 0; peakMaxNum < period_max_peak.maxValueCnt; peakMaxNum++)
						{
							if (secondMaxPeakValueMax < period_max_peak.maxValue[peakMaxNum] && period_max_peak.maxValue[peakMaxNum] != maxPeakValueMax && period_max_peak.maxValuePos[peakMaxNum] > period_min_peak.minValuePos[minPeakValueMinPos])
							{
								secondMaxPeakValueMax = period_max_peak.maxValue[peakMaxNum];
							}
						}
					}

					if (maxPeakValueMax - minPeakValueMin >= 100 && secondMaxPeakValueMax - minPeakValueMin >= 70)
					{
						*period_step_count = *period_step_count + 2;
					}
					else
					{
						*period_step_count = *period_step_count + 1;
					}


				}
				else
				{
					*period_step_count = *period_step_count + 1;
				}



				break;

			}

		}
	}

}




uint8_t into_daily_spo2(uint16_t *input, int inlen)
{
    int thre_peakval = 1500;
    int thre_deta = 400;
    int dur_min = 20;
    int dur_max = 100;
    int clc_dur = 200;
    int i = inlen - clc_dur;
    int peak_index[2] = {0};
    uint16_t peak_val[2] = {0};
    while(i < inlen-dur_min) //(int i = 2; i < len-2;i++)
    {
        int tmpdeta1 = input[i] - input[i-2];
        int tmpdeta2 = input[i] - input[i+2];
        if(input[i] > thre_peakval && (tmpdeta1 > thre_deta && tmpdeta2 > thre_deta)) //find a peak
        {
            peak_index[0] = i;
            peak_val[0] = input[i];
            int tmpsp = i + dur_min;
            int tmpep = min(i + dur_max,inlen);
            for(int j = tmpsp; j < tmpep-2; j++)
            {
                tmpdeta1 = input[j] - input[j-2];
                tmpdeta2 = input[j] - input[j+2];
                if(input[j] > thre_peakval && (tmpdeta1 > thre_deta && tmpdeta2 > thre_deta)) // find second peak
                {
                    return 1;
                }
            }
        }
        i++;
    }
    return 0;
}    


int test_clcnum = 0;
uint8_t test_into_spo2 = 0;

// down add xuan
int period_step_count = 0;
int period_cnt = 0;
float accx_period_data_smooth[STEP_COUNT_PERIOD] = { 0 };
float accy_period_data_smooth[STEP_COUNT_PERIOD] = { 0 };
float accz_period_data_smooth[STEP_COUNT_PERIOD] = { 0 };

int16_t accx_period_data[STEP_COUNT_PERIOD] = { 0 };
int16_t accy_period_data[STEP_COUNT_PERIOD] = { 0 };
int16_t accz_period_data[STEP_COUNT_PERIOD] = { 0 };
uint32_t g_period_data_cnt = 0;
float mean_accx = 0;
float mean_accy = 0;
float mean_accz = 0;

int accx_mean_cnt = 0;
int accy_mean_cnt = 0;
int accz_mean_cnt = 0;

float accx_var = 0;
float accy_var = 0;
float accz_var = 0;

void stepsnum_update(acc_axis *pt, E_ACC_PROC statu)
{
    int detathre = 100;
    int n=1;
    int16_t ACC_x, ACC_y, ACC_z;
    uint16_t gdata;
    if(pt->x == acc_error_data1 && pt->y == acc_error_data1 && pt->z == acc_error_data1 )
    {
        return;
    }
    if(pt->x == acc_error_data2 && pt->y == acc_error_data2 && pt->z == acc_error_data2 )
    {
        gdata = defined_accg;
        if(acc_gyro_fifo.putP > 0)
        {
            ACC_x = acc_gyro_fifo.accx[acc_gyro_fifo.putP-1];
            ACC_y = acc_gyro_fifo.accy[acc_gyro_fifo.putP-1];
            ACC_z = acc_gyro_fifo.accy[acc_gyro_fifo.putP-1];
        }
    }
    else
    {
        ACC_x = pt->x;
        ACC_y = pt->y;
        ACC_z = pt->z;
        gdata = (uint16_t)pow(abs((int)ACC_x*ACC_x + (int)ACC_y*ACC_y + (int)ACC_z*ACC_z), 0.5);
    }
    short todps = 1000;
//    float tmpangrate_x, tmpangrate_y, tmpangrate_z;
//    tmpangrate_x = FS2000DPS_TO_MDPS(pt->gyro[0]) / todps;
//    tmpangrate_y = FS2000DPS_TO_MDPS(pt->gyro[1]) / todps;
//    tmpangrate_z = FS2000DPS_TO_MDPS(pt->gyro[2]) / todps;
//    NRF_LOG_INFO("x = %d, y = %d\r\n",(short)tmpangrate_x,(short)tmpangrate_y);
    
    int tmpintensity = abs(gdata - STD_ACC)/10;
    Stepara.intensity += tmpintensity;

    if(acc_get_work_mode() != IMU_SPO2)   //steps proc
    {
        if(acc_g_fifo.size != step_gbuf_length)
        {
            fifo16_init(&acc_g_fifo, step_gbuf_length, g_buf);
        }
//put data in buf
        fifo16_putPut(&acc_g_fifo, gdata);
      
		accx_period_data[g_period_data_cnt] = ACC_x;
		accy_period_data[g_period_data_cnt] = ACC_y;
		accz_period_data[g_period_data_cnt] = ACC_z;
		g_period_data_cnt = g_period_data_cnt + 1;
		
		
		
        if (fifo16_status(&acc_g_fifo) >= step_gbuf_length)
        {
			g_period_data_cnt = 0;
            if (statu == e_acc_EHR)
            {
                ACC_STEP_SAMPLERATE = 100;
            }
            else if (statu == e_acc_STEP)
            {
                ACC_STEP_SAMPLERATE = 50;
            }
            //steps calculate;
			
            //Count_Move(acc_g_fifo.buf, step_gbuf_length, step_histrate, ACC_STEP_SAMPLERATE, &MoveCnt);
            //step_histrate = MoveCnt;
            //Stepara.step_cnt += (MoveCnt * step_clc_datanum/ACC_STEP_SAMPLERATE);
			
			// down add xuan
			period_step_count = 0;
			Count_Move_get_new(acc_g_fifo.buf, step_gbuf_length, &period_step_count);
			memset(accx_period_data_smooth,0, STEP_COUNT_PERIOD*sizeof(float));
			memset(accy_period_data_smooth, 0, STEP_COUNT_PERIOD * sizeof(float));
			memset(accz_period_data_smooth, 0, STEP_COUNT_PERIOD * sizeof(float));
			smooth_short_ACC_period_data(accx_period_data,SMOOTH_SPAN, accx_period_data_smooth);
			smooth_short_ACC_period_data(accy_period_data, SMOOTH_SPAN, accy_period_data_smooth);
			smooth_short_ACC_period_data(accz_period_data, SMOOTH_SPAN, accz_period_data_smooth);
			mean_accx = 0;
			mean_accy = 0;
			mean_accz = 0;
			mean_get(accx_period_data_smooth, STEP_COUNT_PERIOD,&mean_accx);
			mean_get(accy_period_data_smooth, STEP_COUNT_PERIOD, &mean_accy);
			mean_get(accz_period_data_smooth, STEP_COUNT_PERIOD, &mean_accz);

			accx_mean_cnt = 0;
			accy_mean_cnt = 0;
			accz_mean_cnt = 0;
			cross_mean_cnt_get(accx_period_data_smooth, STEP_COUNT_PERIOD, mean_accx, &accx_mean_cnt);
			cross_mean_cnt_get(accy_period_data_smooth, STEP_COUNT_PERIOD, mean_accy, &accy_mean_cnt);
			cross_mean_cnt_get(accz_period_data_smooth, STEP_COUNT_PERIOD, mean_accz, &accz_mean_cnt);

			accx_var = 0;
			accy_var = 0;
			accz_var = 0;
			accx_var = var_get(accx_period_data_smooth, STEP_COUNT_PERIOD);
			accy_var = var_get(accy_period_data_smooth, STEP_COUNT_PERIOD);
			accz_var = var_get(accz_period_data_smooth, STEP_COUNT_PERIOD);
			
			if (period_step_count * 5 <= accx_mean_cnt || period_step_count * 5 <= accy_mean_cnt || period_step_count * 5 <= accz_mean_cnt)
			{
				period_step_count = 0;
			}
			
			if (accx_mean_cnt > 4 * (STEP_COUNT_PERIOD / ACC_SAMPLERATE) || accy_mean_cnt > 4 * (STEP_COUNT_PERIOD / ACC_SAMPLERATE) || accz_mean_cnt > 4 * (STEP_COUNT_PERIOD / ACC_SAMPLERATE))
			{
				period_step_count = 0;
			}
			
			if ((period_step_count > accx_mean_cnt && period_step_count > accy_mean_cnt) || (period_step_count > accx_mean_cnt && period_step_count > accz_mean_cnt) || (period_step_count > accy_mean_cnt && period_step_count > accz_mean_cnt))
			{
				period_step_count = 0;
			}
			
			if ((accx_var < 20000 && accy_var < 20000 && accz_var < 20000) && ((accx_var < 10000 && accy_var < 10000) || (accx_var < 10000 && accz_var < 10000) || (accy_var < 10000 && accz_var < 10000)))
			{
				period_step_count = 0;
			}
			
			Stepara.step_cnt += period_step_count;
			//up add xuan
            Stepara.stepNum = (int)Stepara.step_cnt;
                    
            if( Stepara.stepNum > 0)
                    NRF_LOG_INFO("Stepara.stepNum = %d", Stepara.stepNum);
            
			//std acc proc
            uint16_t sp_stdacc = step_gbuf_length - accbuf_length;
            acc_para.stdaccval = get_varval(acc_g_fifo.buf + sp_stdacc, accbuf_length);

            //move buf data
            //fifo16_recover(&acc_g_fifo, step_gbuf_length - step_clc_datanum);
			// down add xuan
			fifo16_recover(&acc_g_fifo, step_gbuf_length);
			// up add xuan
        
        }
    }

}

// up add xuan





#if 0
void stepsnum_update(acc_axis *pt, E_ACC_PROC statu)
{
    int detathre = 100;
    int n=1;
    int16_t ACC_x, ACC_y, ACC_z;
    uint16_t gdata;
    if(pt->x == acc_error_data1 && pt->y == acc_error_data1 && pt->z == acc_error_data1 )
    {
        return;
    }
    if(pt->x == acc_error_data2 && pt->y == acc_error_data2 && pt->z == acc_error_data2 )
    {
        gdata = defined_accg;
        if(acc_gyro_fifo.putP > 0)
        {
            ACC_x = acc_gyro_fifo.accx[acc_gyro_fifo.putP-1];
            ACC_y = acc_gyro_fifo.accy[acc_gyro_fifo.putP-1];
            ACC_z = acc_gyro_fifo.accy[acc_gyro_fifo.putP-1];
        }
    }
    else
    {
        ACC_x = pt->x;
        ACC_y = pt->y;
        ACC_z = pt->z;
        gdata = (uint16_t)pow(abs((int)ACC_x*ACC_x + (int)ACC_y*ACC_y + (int)ACC_z*ACC_z), 0.5);
    }
    short todps = 1000;
//    float tmpangrate_x, tmpangrate_y, tmpangrate_z;
//    tmpangrate_x = FS2000DPS_TO_MDPS(pt->gyro[0]) / todps;
//    tmpangrate_y = FS2000DPS_TO_MDPS(pt->gyro[1]) / todps;
//    tmpangrate_z = FS2000DPS_TO_MDPS(pt->gyro[2]) / todps;
//    NRF_LOG_INFO("x = %d, y = %d\r\n",(short)tmpangrate_x,(short)tmpangrate_y);
    
    int tmpintensity = abs(gdata - STD_ACC)/10;
    Stepara.intensity += tmpintensity;

    if(acc_get_work_mode() != IMU_SPO2)   //steps proc
    {
        if(acc_g_fifo.size != step_gbuf_length)
        {
            fifo16_init(&acc_g_fifo, step_gbuf_length, g_buf);
        }
//put data in buf
        fifo16_putPut(&acc_g_fifo, gdata);
      
        if (fifo16_status(&acc_g_fifo) >= step_gbuf_length)
        {
            if (statu == e_acc_EHR)
            {
                ACC_STEP_SAMPLERATE = 100;
            }
            else if (statu == e_acc_STEP)
            {
                ACC_STEP_SAMPLERATE = 50;
            }
            //steps calculate;
            Count_Move(acc_g_fifo.buf, step_gbuf_length, step_histrate, ACC_STEP_SAMPLERATE, &MoveCnt);
            step_histrate = MoveCnt;
            Stepara.step_cnt += (MoveCnt * step_clc_datanum/ACC_STEP_SAMPLERATE);
            Stepara.stepNum = (int)Stepara.step_cnt;
                    
            if( Stepara.stepNum > 0)
                    NRF_LOG_INFO("Stepara.stepNum = %d", Stepara.stepNum);
            
//std acc proc
            uint16_t sp_stdacc = step_gbuf_length - accbuf_length;
            acc_para.stdaccval = get_varval(acc_g_fifo.buf + sp_stdacc, accbuf_length);

            //move buf data
            fifo16_recover(&acc_g_fifo, step_gbuf_length - step_clc_datanum);
        
        }
    }

}
#endif
int steps_num(void)
{
	return Stepara.stepNum;
}

void steps_set(int num)
{
	Stepara.stepNum = num;
}

int gesture_intensity(void)
{
	return Stepara.intensity;
}

void intensity_set(int intensity)
{
	Stepara.intensity = intensity;
}

void steps_clear(void)
{
	Stepara.stepNum = 0;
	Stepara.intensity = 0;
    Stepara.step_cnt = 0;
}

int EHR_fifo16_putPut(FIFO_acc *fifo, short data)
{
	if (fifo->putP >= fifo->size){
		fifo->flags |= FLAGS_OVERRUN;
		return -1;
	}
	fifo->buf[fifo->putP] = data;
	fifo->putP++;
	
	if(fifo->putP >= fifo->size){
		fifo->putP = fifo->size;
	}
	
	return 0;
}

static float ACCgsum = 0;
void EHR_Acc_Proc(acc_axis*pt)
{
    if(pt->x == acc_error_data1 && pt->y == acc_error_data1 && pt->z == acc_error_data1 )
    {
        return;
    }
	extern uint8_t accval;
	float ACCgmean;
    if(pt->x == acc_error_data2 && pt->y == acc_error_data2 && pt->z == acc_error_data2 )
    {
        accdata.ACC_g = defined_accg;
    }
    else
    {
        int16_t ACC_x, ACC_y, ACC_z;
        ACC_x = pt->x;
        ACC_y = pt->y;
        ACC_z = pt->z;
        accdata.ACC_g = (int)pow(abs((int)ACC_x*ACC_x + (int)ACC_y*ACC_y + (int)ACC_z*ACC_z), 0.5);

		//DEBUG_LOG("EHR x=%d, y=%d, z=%d \n", ACC_x,ACC_y,ACC_z);
		//DEBUG_LOG("ACC_g=%d\n",accdata.ACC_g );
		
    }
// filter
    uint8_t cofflen = 7;
    double coffa[7] = { 1.000000000000000e+000, -5.547044191350068e+000, 1.285541954468899e+001, -1.593478169199447e+001, 1.114327437040806e+001, -4.168627535934345e+000, 6.517601971221265e-001 };
    double coffb[7] = { 9.951735617670138e-004, 0, -2.985520685301042e-003, 0, 2.985520685301042e-003, 0, -9.951735617670138e-004 };
    if(EHR_acc_fifo.putP < cofflen - 1)
    {
        tmp_acc_inbuf[EHR_acc_fifo.putP] = (short)accdata.ACC_g;
//        acc_3axis_log_put((uint16_t)accdata.ACC_g);
        EHR_fifo16_putPut(&EHR_acc_fifo, (short)accdata.ACC_g);
    }
    else
    {
        if(accdata.ACC_g > acc_error_data3)
        {
            tmp_acc_inbuf[6] = tmp_acc_inbuf[5];
        }
        else
        {
            tmp_acc_inbuf[6] = (short)accdata.ACC_g;
        }
        tmp_acc_outbuf[6] = EHR_acc_filter(tmp_acc_inbuf, coffa, coffb, cofflen, tmp_acc_outbuf);
//        acc_3axis_log_put((uint16_t)accdata.ACC_g);
        EHR_fifo16_putPut(&EHR_acc_fifo, (short)tmp_acc_outbuf[6]);
        memcpy(tmp_acc_inbuf, tmp_acc_inbuf + 1, sizeof(short)*(cofflen - 1));
        memcpy(tmp_acc_outbuf, tmp_acc_outbuf + 1, sizeof(double)*(cofflen - 1));
    }
    
    ACCgsum += accdata.ACC_g;
    accdata.ACCdatanum++;

    if(accdata.ACCdatanum >= 100)// 100Hz ---- 10
    {
        ACCgmean = ACCgsum/accdata.ACCdatanum;
        accdata.ACCdatanum = 0;
        ACCgsum = 0;
        int tmpaccval = abs(accdata.ACC_g - (int)ACCgmean);
        uint8_t ehr_flg = 0;
        if(g_afe.ehr_flag == SPO2_HR_VALID)
        {
            ehr_flg = 0;
        }
        else
        {
            ehr_flg = 1;
        }
        accval = data_compress16(tmpaccval,ehr_flg,0);
        test_accval = accval;
    }
}

#if 0
void acc_rawdata_send(int *x, int *y, int num)
{
	static uint16_t m_count = 0;
	uint8_t frame_buf[20] = {0};
	if (get_log_type() == RAW_ACC) {
		frame_buf[0] = ACC_WORD;
	} else if (get_log_type() == RAW_GYRO) {
		frame_buf[0] = GYRO_WORD;
	} else if (get_log_type() == RAW_IMU) {
		frame_buf[0] = IMU_WORD;
	}
	
	frame_buf[1] = num*6;	//6 axis, always 3
//    DEBUG_LOG("x:%d, y:%d, z:%d\r\n",x[0],x[1],x[2]);
	for (int i = 0; i < 3*num; ++i) {
		frame_buf[2+2*i] = (x[i] >> 8) & 0xff;
		frame_buf[2+2*i+1] = x[i] & 0xff;

		frame_buf[8+2*i] = (y[i] >> 8) & 0xff;
		frame_buf[8+2*i+1] = y[i] & 0xff;
	}
    frame_buf[14] = test_draw;
    frame_buf[15] = test_wavearound;
    frame_buf[16] = testrotate_acc;
    frame_buf[17] = test_dur;
    
	frame_buf[18] = (m_count >> 8) & 0xFF;
	frame_buf[19] = m_count & 0xFF;
	m_count++;
	BLE_RAW_BLOCK_SEND(frame_buf, 20);
}
int gesture_status_update(enum GESTURE_STATUS status)
{
	if (gesture_warning.curr_gesture == status) {
		return 0;
	}

	if ((swtimer_getSec() >= (gesture_warning.timeout+gesture_warning.base_time))
		|| (gesture_warning.base_time > swtimer_getSec())) {
		gesture_status_update_explicit(status, DEFAULT_GESTURE_TIMEOUT);
		return 0;
	}

	return -1;
}


void gesture_status_update_explicit(enum GESTURE_STATUS status, int timeout)
{
	gesture_warning.base_time = swtimer_getSec();
	gesture_warning.conti_time = 0;
	gesture_warning.timeout = timeout;
	gesture_warning.curr_gesture = status;
}


enum GESTURE_STATUS get_gesture_status(void)
{
	if (gesture_warning.conti_time < gesture_warning.timeout) {
		gesture_warning.conti_time = swtimer_getSec() - gesture_warning.base_time;
		if (gesture_warning.conti_time < 0) {	//protect uint32 value overide
			gesture_warning.conti_time = gesture_warning.timeout;
			gesture_warning.base_time = swtimer_getSec();
		}
	} else {
		gesture_warning.curr_gesture = NONE;
	}

	return gesture_warning.curr_gesture;
}
#endif
int gesture_support(void)
{
	return 0;
	
//	if (gesture_on) {
//		return 1;
//	}
//	
//	if (ally_manage.co == LUMI_CO) {
//		return 1;
//	}

}
