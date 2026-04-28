#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "../WaveSrc/wavelib.h"
#include "DataProcHealth.h"
#include "math.h"
#include "fifo.h"
#include "nrf_log.h"
#include "DataProcSPO2.h"
#include "filter_datapre.h"
//using namespace std;


wave_object WaveobjHealth;
wt_object WavewtHealth;

fft_object FFTobjHealth;
int data_start_pos = 0;
int pre_last_peak_pos = 0;


void smooth_data_int_out_int(int* frame_data, int smooth_span, int data_len, int* frame_data_smooth)
{

    int sum_num = 0;
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
        frame_data_smooth[frame_data_num] = (int)(sum / (add_num * 2 + 1));

    }

}
void smooth_data(int* frame_data, int smooth_span, int data_len, float* frame_data_smooth)
// void smooth_data(f* frame_data, int smooth_span, int data_len, int* frame_data_smooth)
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

    for (int frame_data_num = 0; frame_data_num < data_len; frame_data_num++)
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
        sum = 0.0;
        for (sum_num = frame_data_num - add_num; sum_num <= frame_data_num + add_num; sum_num++)
        {
            sum += frame_data[sum_num];
        }
        frame_data_smooth[frame_data_num] = (sum / (add_num * 2 + 1));

    }

}
void smooth_data_16uint(uint16_t* frame_data, int smooth_span, int data_len, float* frame_data_smooth)
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

    for (int frame_data_num = 0; frame_data_num < data_len; frame_data_num++)
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
        sum = 0.0;
        for (sum_num = frame_data_num - add_num; sum_num <= frame_data_num + add_num; sum_num++)
        {
            sum += frame_data[sum_num];
        }
        frame_data_smooth[frame_data_num] = sum / (add_num * 2 + 1);

    }

}
void smooth_data_f(float* frame_data, int smooth_span, int data_len, float* frame_data_smooth)
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

    for (int frame_data_num = 0; frame_data_num < data_len; frame_data_num++)
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
        sum = 0.0;
        for (sum_num = frame_data_num - add_num; sum_num <= frame_data_num + add_num; sum_num++)
        {
            sum += frame_data[sum_num];
        }
        frame_data_smooth[frame_data_num] = sum / (add_num * 2 + 1);

    }

}


void wavelet_baseProcHealth(float* in, int inlen, int slevel, int elevel, float* out)
{
    dwt_f(WavewtHealth, in);// Perform DWT

    int sp = 0;
    for (int i = 0; i < slevel; i++)
    {
        sp += WavewtHealth->length[i];
    }

    int ep = sp;
    for (int i = slevel; i < elevel; i++)
    {
        ep += WavewtHealth->length[i];
    }
    for (int i = 0; i < sp; i++)
    {
        WavewtHealth->output[i] = 0;
    }

    int totallen = ep;
    for (int i = elevel; i <= WavewtHealth->J; i++)
    {
        totallen += WavewtHealth->length[i];
    }

    for (int i = ep; i < totallen; i++)
    {
        WavewtHealth->output[i] = 0;
    }
    //printf("data wavelet process finish\n");
    idwt_f(WavewtHealth, out);
}

char dwt_1[] = "dwt";
char* dwtW = dwt_1;
char name_1[] = "db4";
char* name = name_1;
void PPG_data_wavelet(float* PPGData,int inLen,float* PPGDataWavelet)
{
    WaveobjHealth = wave_init(name);
    //WavewtHealth = wt_init(WaveobjHealth, "dwt", len, DataProc.strBreathPara.wavelevel);// Initialize the wavelet transform object
    WavewtHealth = wt_init(WaveobjHealth, dwtW, HRV_PROCESS_DATA_PERIOD, PPG_DATA_WAVELET_LEVEL);// Initialize the wavelet transform object


    wavelet_baseProcHealth(PPGData, inLen, PPG_DATA_WAVELET_START_LEVEL, PPG_DATA_WAVELET_END_LEVEL, PPGDataWavelet);

    wt_free(WavewtHealth);
    wave_free(WaveobjHealth);
}



void PPG_data_RR_peaks_get(float *PPGData,int DataLen,uint16_t*RRData,int* RRLen)
{

    int PPG_R_Down_peak_pos[500] = {0};
    int down_left_valid_flag = 1;
    int down_right_valid_flag = 1;
    int down_peak_num = 0;
    for (int num1 = 10;num1 < HRV_PROCESS_DATA_PERIOD - 10;num1++)
    {
        down_left_valid_flag = 1;
        down_right_valid_flag = 1;
        for (int num_left = num1; num_left<=num1; num_left++)
        {
            if (PPGData[num_left]>= PPGData[num_left - 1])
            {
                down_left_valid_flag = 0;
                break;
            }
        }
        for (int num_right = num1; num_right <= num1; num_right++)
        {
            if (PPGData[num_right] >= PPGData[num_right + 1])
            {
                down_right_valid_flag = 0;
                break;
            }
        }
        if (down_left_valid_flag == 1 && down_right_valid_flag == 1)
        {
            PPG_R_Down_peak_pos[down_peak_num] = num1;
            down_peak_num++;
        }

    }


    int pre_left_right_high_flag = 0;
    int left_valid_flag = 1;
    int right_valid_flag = 1;
    for (int num1 = 10;num1 < HRV_PROCESS_DATA_PERIOD - 10;num1++)
    {

        if(*RRLen >= HRV_MAX_DATA_LEN)
        {
            break;
        }
        left_valid_flag = 1;
        right_valid_flag = 1;
        for (int num_left = num1-8; num_left <= num1;num_left++)
        {
            if (PPGData[num_left] < PPGData[num_left - 1])
            {
                left_valid_flag = 0;
                break;
            }
        }
        for (int num_right = num1;num_right <= num1 + 8;num_right++)
        {
            if (PPGData[num_right] < PPGData[num_right + 1])
            {
                right_valid_flag = 0;
                break;
            }
        }

        if ( left_valid_flag == 1 && right_valid_flag == 1 && PPGData[num1] > 0)
        {
            for (int down_num = 0; down_num < down_peak_num - 1; down_num++)
            {
                if (num1 > PPG_R_Down_peak_pos[down_num] && num1 < PPG_R_Down_peak_pos[down_num + 1])
                {
                    if (PPGData[num1] - PPGData[PPG_R_Down_peak_pos[down_num]] > VAILD_PEAK_HIGH_THRESHOLD && PPGData[num1] - PPGData[PPG_R_Down_peak_pos[down_num + 1]] > VAILD_PEAK_HIGH_THRESHOLD)
                    {
                        //RRData[*RRLen] = num1 + data_start_pos;
                        RRData[*RRLen] = num1;
                        *RRLen = *RRLen + 1;
                        pre_left_right_high_flag = 0;
                    }
                    else
                    {
                        if (pre_left_right_high_flag == 1)
                        {
                            //RRData[*RRLen] = num1 + data_start_pos;
                            RRData[*RRLen] = num1;
                            *RRLen = *RRLen + 1;
                            pre_left_right_high_flag = 0;
                        }
                        pre_left_right_high_flag = 1;
                    }
                }



            }

        }
        else if ((left_valid_flag == 1 && PPGData[num1] > PPGData[num1 + 1] && PPGData[num1] > 0)||(right_valid_flag == 1 && PPGData[num1] > PPGData[num1 - 1] && PPGData[num1] > 0))
        {
            for (int down_num = 0; down_num < down_peak_num - 1; down_num++)
            {
                if (num1 > PPG_R_Down_peak_pos[down_num] && num1 < PPG_R_Down_peak_pos[down_num + 1])
                {
                    if (PPGData[num1] - PPGData[PPG_R_Down_peak_pos[down_num]] > VAILD_PEAK_HIGH_THRESHOLD || PPGData[num1] - PPGData[PPG_R_Down_peak_pos[down_num + 1]] > VAILD_PEAK_HIGH_THRESHOLD)
                    {
                        if (pre_left_right_high_flag == 1)
                        {
                            //RRData[*RRLen] = num1 + data_start_pos;
                            RRData[*RRLen] = num1;
                            *RRLen = *RRLen + 1;
                            pre_left_right_high_flag = 0;
                        }

                    }

                }

            }
            pre_left_right_high_flag = 1;

        }


    }
   // pre_last_peak_pos = RRData[*RRLen - 1] - data_start_pos;
    //data_start_pos = RRData[*RRLen - 1] + 11;
}


uint8_t PPG_data_RR_peaks_get_new(float* PPGData, int DataLen, uint16_t* RRData, int* RRLen)
{
    float up_peaks[HRV_PROCESS_DATA_PERIOD / 2] = {0};
    uint8_t up_peaks_pos[HRV_PROCESS_DATA_PERIOD / 2] = { 0 };
    float down_peaks[HRV_PROCESS_DATA_PERIOD / 2] = { 0 };
    uint8_t down_peaks_pos[HRV_PROCESS_DATA_PERIOD / 2] = { 0 };
    float RR_peaks_value[HRV_PROCESS_DATA_PERIOD / 2] = {0};
    uint8_t up_peak_cnt = 0;
    uint8_t down_peak_cnt = 0;
    uint8_t ret = INVALID_PERIOD;
    for (int num = 2;num < DataLen - 3;num++)
    {
        if (PPGData[num] > PPGData[num - 1] && PPGData[num - 1] > PPGData[num - 2] && PPGData[num] > PPGData[num + 1] && PPGData[num + 1] > PPGData[num + 2])
        {
            up_peaks[up_peak_cnt] = PPGData[num];
            up_peaks_pos[up_peak_cnt] = num;
            up_peak_cnt++;
        }
        if (PPGData[num] < PPGData[num - 1] && PPGData[num - 1] < PPGData[num - 2] && PPGData[num] < PPGData[num + 1] && PPGData[num + 1] < PPGData[num + 2])
        {
            down_peaks[down_peak_cnt] = PPGData[num];
            down_peaks_pos[down_peak_cnt] = num;
            down_peak_cnt++;
        }

    }
    //Data of the end  is as down peak.
    down_peaks[down_peak_cnt] = PPGData[DataLen-1];
    down_peaks_pos[down_peak_cnt] = DataLen - 1;
    down_peak_cnt++;

    *RRLen = 0;
    for (int up_num = 0; up_num < up_peak_cnt; up_num++)
    {
        for (int down_num = 0;down_num < down_peak_cnt;down_num++)
        {
            if (up_peaks_pos[up_num] < down_peaks_pos[down_num])
            {
                if (up_peaks[up_num] - down_peaks[down_num] > HRV_PROCESS_DATA_VALID_PEAK_THRESHOLD && up_peaks[up_num] - down_peaks[down_num] < HRV_PROCESS_DATA_VALID_PEAK_THRESHOLD_TWO)
                {
                    RRData[*RRLen] = up_peaks_pos[up_num];
                    RR_peaks_value[*RRLen] = up_peaks[up_num];
                    (*RRLen)++;
                    break;
                }
                break;
            }
        }

    }

    float max_value = 0;
    float min_value = 0;
    if (*RRLen >= 2)
    {
        ret = VALID_PERIOD;
        max_value = RR_peaks_value[0];
        min_value = RR_peaks_value[0];
        for (int num = 0;num < *RRLen;num++)
        {
            if (max_value < RR_peaks_value[num])
            {
                max_value = RR_peaks_value[num];
            }
            if (min_value > RR_peaks_value[num])
            {
                min_value = RR_peaks_value[num];
            }

        }
        if (fabs(max_value - min_value) > 3000)
        {
            memset(RRData,0, (*RRLen)*sizeof(uint16_t));
            *RRLen = 0;
            ret = INVALID_PERIOD;
        }

    }

    float RR_peaks_value_new[HRV_PROCESS_DATA_PERIOD / 2] = { 0 };
    uint16_t RRDataNew[HRV_PROCESS_DATA_PERIOD / 2] = {0};
    int RRLenNew = 0;
    if (*RRLen <= 1)
    {
        ret = INVALID_PERIOD;
        for (int up_num = 0; up_num < up_peak_cnt; up_num++)
        {
            for (int down_num = 0; down_num < down_peak_cnt; down_num++)
            {
                if (up_peaks_pos[up_num] < down_peaks_pos[down_num])
                {
                    if (up_peaks[up_num] - down_peaks[down_num] > HRV_PROCESS_DATA_VALID_PEAK_THRESHOLD_NEW && up_peaks[up_num] - down_peaks[down_num] < HRV_PROCESS_DATA_VALID_PEAK_THRESHOLD_TWO)
                    {
                        RRDataNew[RRLenNew] = up_peaks_pos[up_num];
                        RR_peaks_value_new[RRLenNew] = up_peaks[up_num];
                        RRLenNew++;
                        break;
                    }
                    break;
                }
            }

        }


        max_value = RR_peaks_value_new[0];
        min_value = RR_peaks_value_new[0];
        for (int num = 0; num < RRLenNew; num++)
        {
            if (max_value < RR_peaks_value_new[num])
            {
                max_value = RR_peaks_value_new[num];
            }
            if (min_value > RR_peaks_value_new[num])
            {
                min_value = RR_peaks_value_new[num];
            }

        }
        if (fabs(max_value - min_value) < 2000 && RRLenNew >= 2)
        {
            memset(RRData, 0, (*RRLen) * sizeof(uint16_t));
            *RRLen = 0;
            memcpy(RRData, RRDataNew, RRLenNew*sizeof(uint16_t));
            *RRLen = RRLenNew;
            ret = VALID_PERIOD;
            
        }

    }




    return ret;
}



AlgData_t get_HRV_mean(AlgData_t* input, int inlen)
{
    if (inlen <= 0)
    {
        return 0;
    }
    AlgData_t sum = 0;
    for (int i = 0; i < inlen; i++)
    {
        sum += input[i];
    }
    return sum / inlen;
}

AlgData_t get_rr_var(AlgData_t* input, int inlen)
{
    if (inlen <= 1)
    {
        return 0;
    }
    AlgData_t var = 0;
    AlgData_t mean = get_HRV_mean(input, inlen);
    for (int i = 0; i < inlen; i++)
    {
        var += (input[i] - mean) * (input[i] - mean);
    }
    var = var / (inlen - 1);
    var = sqrt(var);
    return var;
}

AlgData_t get_rootmean(AlgData_t* input, int inlen)
{
    if (inlen <= 2)
    {
        return 0;
    }
    AlgData_t var = 0;
    for (int i = 0; i < inlen - 1; i++)
    {
        var += (input[i] - input[i + 1]) * (input[i] - input[i + 1]);
    }
    var = var / (inlen - 1);
    var = sqrt(var);
    return var;
}
AlgData_t get_health_min(AlgData_t* input, int inlen)
{
    AlgData_t Min = input[0];
    for (int i = 1; i < inlen; i++)
    {
        if (input[i] < Min)
        {
            Min = input[i];
        }
    }
    return Min;
}


int Histragm[100];
AlgData_t get_TrangleIdx(AlgData_t* input, int inlen)
{
    AlgData_t minrr = get_health_min(input, inlen);
    AlgData_t slice = 1000.0 / 128;
    int minsp = (int)(minrr / slice);
    memset(Histragm, 0, sizeof(int) * 100);
    for (int i = 0; i < inlen; i++)
    {
        int tid = (input[i] - minrr) / slice;
        if (tid < 100)
        {
            Histragm[tid]++;
        }
        else
        {
            Histragm[99]++;
        }
    }

    int maxhist = 0;
    for (int i = 0; i < 100; i++)
    {
        if (Histragm[i] > maxhist)
        {
            maxhist = Histragm[i];
        }
    }
    return inlen * 1.0 / maxhist;
}
void get_health_max(AlgData_t* input, int len, AlgData_t* Maxval, int* MaxPos)
{
    *Maxval = 0;
    *MaxPos = 0;
    for (int i = 0; i < len; i++)
    {
        if (input[i] > *Maxval)
        {
            *Maxval = input[i];
            *MaxPos = i;
        }
    }
}

// void get_HrvTimePara(float* rr, int inlen, Time_Para* TimePara)
// {
//     AlgData_t meanrr = get_HRV_mean(rr, inlen);
//     TimePara->RRmean = meanrr;
//     TimePara->meanbpm = 60000.0 / meanrr;
//     TimePara->SDNN = get_rr_var(rr, inlen);
//     TimePara->RMSSD = get_rootmean(rr, inlen);
//     AlgData_t NN = get_HRV_mean(rr, inlen);

//     TimePara->NN50 = 0;
//     for (int i = 0; i < inlen - 1; i++)
//     {
//         if (rr[i] - rr[i + 1] >= 50 || rr[i] - rr[i + 1] <= -50)
//         {
//             TimePara->NN50++;
//         }
//     }
//     TimePara->pNN50 = TimePara->NN50 * 100.0 / inlen;

//     TimePara->TrangleIdx = get_TrangleIdx(rr, inlen);

//     AlgData_t* MaxRR;
//     int* MaxRRPos;
//     get_health_max(rr, inlen, MaxRR, MaxRRPos);

//     TimePara->MaxRR = MaxRR;

//     AlgData_t TimeStamp = 0;
//     for (int i = 0; i < MaxRRPos; i++)
//     {
//         TimeStamp += rr[i];
//     }
//     TimePara->MaxRRTimeStamp = TimeStamp / 1000;

// }

void get_HrvGeomPara(AlgData_t* input, int inlen, Geom_Para* GeomPara)
{
    AlgData_t mean1 = 0;
    AlgData_t mean2 = 0;

    for (int i = 0; i < inlen - 1; i++)
    {
        mean1 += input[i + 1] - input[i];
        mean2 += input[i + 1] + input[i];
    }
    mean1 = mean1 / (inlen - 1);
    mean2 = mean2 / (inlen - 1);

    AlgData_t var1 = 0;
    AlgData_t var2 = 0;
    for (int i = 0; i < inlen - 1; i++)
    {
        var1 += (input[i + 1] - input[i] - mean1) * (input[i + 1] - input[i] - mean1);
        var2 += (input[i + 1] + input[i] - mean2) * (input[i + 1] + input[i] - mean2);
    }

    GeomPara->SD1 = sqrt(var1 / (inlen - 2) / 2);
    GeomPara->SD2 = sqrt(var2 / (inlen - 2) / 2);
    GeomPara->SDRate = GeomPara->SD1 / GeomPara->SD2;
}



//int period_process_data[HRV_PROCESS_DATA_PERIOD] = { 0 };
int green_smooth[HRV_PROCESS_DATA_PERIOD] = { 0 };
//float green_data_wavelet[HRV_PROCESS_DATA_PERIOD] = { 0 };
// uint16_t green_RR[HRV_MAX_DATA_LEN] = { 0 };
//float green_RR_smooth[HRV_MAX_DATA_LEN] = { 0 };
int greenRRLen = 0;
//Time_Para g_green_rrTimePara = { 0 };
//Geom_Para g_green_rrGeomPara = { 0 };
int g_time_cnt = 0;//1:10s
int g_all_time_cnt = 0;//1:10s
HRV_Para hrv_health_para = {0};

uint8_t g_hrv_health_process_flag = HRV_HEALTH_PROCESS_STOP_FLAG;
uint8_t g_type_health = HEALTH_NON_PROFESSIONAL_TYPE;
HRV_Para* hrv_health_process_result_get(void)
{
    if(g_hrv_health_process_flag == HRV_HEALTH_PROCESS_FINISH_FLAG)
    {
        //hrv_health_para.RR_num = 200;
        //for(int i = 0; i < 200; i++){
        //    hrv_health_para.green_RR[i] = 2;
        //}
        return &hrv_health_para;
    }
    else
    {
        return NULL;
    }
}
void hrv_health_process_flag_stop(void)
{
    g_hrv_health_process_flag = HRV_HEALTH_PROCESS_STOP_FLAG;
}
void hrv_health_process_flag_set(uint8_t flag)
{
    g_hrv_health_process_flag = flag;
}

uint8_t hrv_health_process_flag_get(void)
{
    return g_hrv_health_process_flag;
}

Stress_Acc_Para g_stress_acc = {0};
void stress_acc_get(uint8_t acc)
{
    if(hrv_health_process_flag_get() == HRV_HEALTH_PROCESS_START_FLAG && g_stress_acc.acc_len < STRESS_ACC_NUM)
    {
        g_stress_acc.acc_value[g_stress_acc.acc_len] = acc;
        g_stress_acc.acc_len++;
    }

}



extern uint8_t offline_ledflg;

Exercise_Acc_Para g_exercise_acc = {0};
void ACC_Exercise_judge_data_clear(void)
{
    memset(&g_exercise_acc,0,sizeof(Exercise_Acc_Para));
    //NRF_LOG_INFO("ACC_Exercise_judge_data_clear=================================\n");
}
void Exercise_judge_acc_get(int acc)
{
    //NRF_LOG_INFO("==========g_exercise_acc.acc_len = %d\n",g_exercise_acc.acc_len);
    if(g_exercise_acc.acc_len < EXERCISE_ACC_NUM)
    {
        g_exercise_acc.acc_value[g_exercise_acc.acc_len] = acc;
        g_exercise_acc.acc_len++;
    }
    else{
        g_exercise_acc.acc_len = 0;
        g_exercise_acc.acc_value[g_exercise_acc.acc_len] = acc;
        g_exercise_acc.acc_len++;        
    }
}

uint8_t g_exercise_flag = END_EXERCISE_MODE;
uint8_t acc_get_Exercise_flag(void)
{
    uint16_t vaild_acc_cnt = 0;
    float exercise_rate = 0;
    //if(offline_ledflg == OFFLINE_LED_OFF)
    //{
    //    g_exercise_flag = END_EXERCISE_MODE;
    //    return g_exercise_flag;
    //}
    for(uint16_t num = 0; num < EXERCISE_ACC_NUM;num++)
    {
        if(g_exercise_acc.acc_value[num] > EXERCISE_VAILD_ACC_THRESHOLD)
        {
            vaild_acc_cnt = vaild_acc_cnt + 1;
        }
    }
    exercise_rate = vaild_acc_cnt*1.0/EXERCISE_ACC_NUM;
    //NRF_LOG_INFO("333333333333==========exercise_rate = %d\n",exercise_rate);
    if(exercise_rate > 0.9 && g_exercise_flag == END_EXERCISE_MODE)
    {
        // daily ACC sample :50hz
        g_exercise_flag = START_EXERCISE_MODE;
    }
    else if(exercise_rate <= 0.2 && g_exercise_flag == START_EXERCISE_MODE)
    //else if(exercise_rate <= 0.6 && g_exercise_flag == START_EXERCISE_MODE)
    {
        // EHR ACC sample :100hz
        g_exercise_flag = END_EXERCISE_MODE;
    }

   // NRF_LOG_INFO("333333333333==========g_exercise_flag = %d\n",g_exercise_flag);
    return g_exercise_flag;
}

uint8_t g_auto_sleep_start_judge_flag = 0;
//sleep auto start and auto end
Sleep_Acc_Para g_sleep_acc = {0};

//uint8_t g_sleep_flag = END_SLEEP_MODE;
uint8_t g_sleep_flag = END_SLEEP_MODE;
uint32_t g_auto_sleep_offhand_cnt = 0;

uint8_t g_sleep_start_check_flag = 0;
uint8_t g_sleep_end_check_flag = 0;


void start_sleep_auto_start_check(void)
{
    ACC_Sleep_judge_data_clear();
    //g_sleep_flag = END_SLEEP_MODE;
    g_sleep_start_check_flag = 1;
   // g_auto_sleep_start_judge_flag = AUTO_DAILY_SLEEP_CHECK_FLAG;
}

void start_sleep_auto_end_check(void)
{
    ACC_Sleep_judge_data_clear();
    //g_sleep_flag = START_SLEEP_MODE;
    g_sleep_end_check_flag = 1;
    //g_auto_sleep_start_judge_flag = AUTO_DAILY_SLEEP_CHECK_FLAG;
}


uint32_t g_start_auto_lowPowr_sleep_cnt = 0;
void ACC_Sleep_judge_data_clear(void)
{
    memset(&g_sleep_acc,0,sizeof(Sleep_Acc_Para));
    g_auto_sleep_start_judge_flag = 0;
    g_sleep_flag = END_SLEEP_MODE;
    g_auto_sleep_offhand_cnt = 0;
    g_start_auto_lowPowr_sleep_cnt = 0;
}

uint8_t g_offhand_sleep_flag = 0;
void daily_sleep_offhand_flag_set(uint8_t flag)
{
    g_offhand_sleep_flag = flag;
}
uint8_t daily_sleep_offhand_flag_get(void)
{
    return g_offhand_sleep_flag;
}

uint32_t g_sleep_on_keep_time = 0;//s
void Sleep_end_judge_acc_get(uint8_t acc)
{
    //NRF_LOG_INFO("==========g_sleep_acc.acc_len = %d\n",g_sleep_acc.acc_len);
    g_sleep_on_keep_time = g_sleep_on_keep_time + 1;
    if(daily_sleep_offhand_flag_get() == 1 && g_sleep_flag == START_SLEEP_MODE)
    {
        g_auto_sleep_offhand_cnt = g_auto_sleep_offhand_cnt + 1;
        if(g_auto_sleep_offhand_cnt > SLEEP_OFFHAND_NUM)
        {
            g_sleep_flag = END_SLEEP_MODE;
            g_auto_sleep_offhand_cnt = 0;
            memset(&g_sleep_acc,0,sizeof(Sleep_Acc_Para));
            g_auto_sleep_start_judge_flag = 0;
        }
    }
    if(g_sleep_acc.acc_len < SLEEP_ACC_NUM)
    {
        g_sleep_acc.acc_value[g_sleep_acc.acc_len] = acc;
        g_sleep_acc.acc_len++;
    }
    else{
        g_auto_sleep_start_judge_flag = AUTO_DAILY_SLEEP_CHECK_FLAG;
        g_sleep_acc.acc_len = 0;
        g_sleep_acc.acc_value[g_sleep_acc.acc_len] = acc;
        g_sleep_acc.acc_len++;        
    }
}
void Sleep_start_judge_acc_get(uint8_t acc)
{
    g_sleep_on_keep_time = 0;
    //NRF_LOG_INFO("==========g_sleep_acc.acc_len = %d\n",g_sleep_acc.acc_len);
    //NRF_LOG_INFO("==========acc = %d\n",acc);
    if(daily_sleep_offhand_flag_get() == 1 && g_sleep_flag == END_SLEEP_MODE)
    {

        g_sleep_flag = END_SLEEP_MODE;
        memset(&g_sleep_acc,0,sizeof(Sleep_Acc_Para));
        g_auto_sleep_start_judge_flag = 0;

        return;
    }
    if(g_sleep_acc.acc_len < SLEEP_ACC_NUM)
    {
        g_sleep_acc.acc_value[g_sleep_acc.acc_len] = acc;
        g_sleep_acc.acc_len++;
    }
    else{
        g_auto_sleep_start_judge_flag = AUTO_DAILY_SLEEP_CHECK_FLAG;
        g_sleep_acc.acc_len = 0;
        g_sleep_acc.acc_value[g_sleep_acc.acc_len] = acc;
        g_sleep_acc.acc_len++;        
    }

    
   // NRF_LOG_INFO("1111==========g_sleep_acc.acc_len= %d\n",g_sleep_acc.acc_len);
}

uint16_t g_wake_indirect_move_cnt = 0;
void auto_sleep_ON_OFF_threshold_get(Sleep_Acc_Para sleepAcc,uint32_t sleepOnTime,uint16_t* start_sleep_vaild_acc_cnt,uint16_t* end_sleep_vaild_acc_cnt,uint16_t* judge_end_sleep_threshold)
{
    uint16_t sleep_end_threshold_jude_time = 0;
    //start sleep threshold cnt
    for(uint16_t num = 0; num < SLEEP_ACC_NUM;num++)
    {
        if(g_sleep_acc.acc_value[num] > SLEEP_VAILD_ACC_THRESHOLD)
        {
            *start_sleep_vaild_acc_cnt = *start_sleep_vaild_acc_cnt + 1;
        }

    }


    //judge time process
    if(sleepOnTime < SLEEP_KEEP_TIME_ONE)
    {
        sleep_end_threshold_jude_time = SLEEP_END_JUDGE_TIME_ONE;
        *judge_end_sleep_threshold = SLEEP_END_ACC_NUM_ONE;

    }
    else if(sleepOnTime >= SLEEP_KEEP_TIME_ONE && sleepOnTime < SLEEP_KEEP_TIME_TWO)
    {
        sleep_end_threshold_jude_time = SLEEP_END_JUDGE_TIME_TWO;
        *judge_end_sleep_threshold = SLEEP_END_ACC_NUM_TWO;
    }
    else if(sleepOnTime >= SLEEP_KEEP_TIME_TWO && sleepOnTime < SLEEP_KEEP_TIME_THREE)
    {
        sleep_end_threshold_jude_time = SLEEP_END_JUDGE_THREE_THREE;
        *judge_end_sleep_threshold = SLEEP_END_ACC_NUM_THREE;
    }
    else if(sleepOnTime >= SLEEP_KEEP_TIME_THREE && sleepOnTime < SLEEP_KEEP_TIME_FOUR)
    {
        sleep_end_threshold_jude_time = SLEEP_END_JUDGE_THREE_FOUR;
        *judge_end_sleep_threshold = SLEEP_END_ACC_NUM_FOUR;
    }
    else if(sleepOnTime >= SLEEP_KEEP_TIME_FOUR)
    {
        sleep_end_threshold_jude_time = SLEEP_END_JUDGE_THREE_FIVE;
        *judge_end_sleep_threshold = SLEEP_END_ACC_NUM_FIVE;
    }



    if(sleepAcc.acc_len >= sleep_end_threshold_jude_time)
    {
        for(uint16_t num = sleepAcc.acc_len - sleep_end_threshold_jude_time; num < sleepAcc.acc_len;num++)
        {
            if(g_sleep_acc.acc_value[num] > SLEEP_END_VAILD_ACC_THRESHOLD)
            {
                *end_sleep_vaild_acc_cnt = *end_sleep_vaild_acc_cnt + 1;
            }
        }
    }
    else{
        for(uint16_t num = 0; num < sleepAcc.acc_len;num++)
        {
            if(g_sleep_acc.acc_value[num] > SLEEP_END_VAILD_ACC_THRESHOLD)
            {
                *end_sleep_vaild_acc_cnt = *end_sleep_vaild_acc_cnt + 1;
            }
        }

        for(uint16_t num = SLEEP_ACC_NUM - (sleep_end_threshold_jude_time - sleepAcc.acc_len); num < SLEEP_ACC_NUM;num++)
        {
            if(g_sleep_acc.acc_value[num] > SLEEP_END_VAILD_ACC_THRESHOLD)
            {
                *end_sleep_vaild_acc_cnt = *end_sleep_vaild_acc_cnt + 1;
            }
        }
    }


    g_wake_indirect_move_cnt = 0;

    if(sleepOnTime >= SLEEP_KEEP_TIME_FIVE)
    {
        for(uint16_t num = 0; num < sleepAcc.acc_len - 1;num++)
        {
            if(g_sleep_acc.acc_value[num] < WAKE_INDIRECT_MOVE_THRESHOLD && g_sleep_acc.acc_value[num + 1] >= WAKE_INDIRECT_MOVE_THRESHOLD)
            {
                g_wake_indirect_move_cnt = g_wake_indirect_move_cnt + 1;
            }
        }
        
        for(uint16_t num = sleepAcc.acc_len + 1; num < SLEEP_ACC_NUM - 1;num++)
        {
            if(g_sleep_acc.acc_value[num] < WAKE_INDIRECT_MOVE_THRESHOLD && g_sleep_acc.acc_value[num + 1] >= WAKE_INDIRECT_MOVE_THRESHOLD)
            {
                g_wake_indirect_move_cnt = g_wake_indirect_move_cnt + 1;
            }

        }

    }
    


}


uint8_t acc_get_Sleep_flag(void)
{
    uint16_t start_sleep_vaild_acc_cnt = 0;
    uint16_t end_sleep_vaild_acc_cnt = 0;
    float start_sleep_rate = 0;
    float end_sleep_rate = 0;
    uint16_t judge_end_sleep_threshold = SLEEP_END_ACC_NUM_FIVE;




    auto_sleep_ON_OFF_threshold_get(g_sleep_acc,g_sleep_on_keep_time,&start_sleep_vaild_acc_cnt,&end_sleep_vaild_acc_cnt,&judge_end_sleep_threshold);

    //NRF_LOG_INFO("==========start_sleep_vaild_acc_cnt = %d\n",start_sleep_vaild_acc_cnt);
    //NRF_LOG_INFO("==========g_auto_sleep_start_judge_flag = %d\n",g_auto_sleep_start_judge_flag);
    start_sleep_rate = start_sleep_vaild_acc_cnt*1.0/SLEEP_ACC_NUM;

    if(start_sleep_rate < 0.02 && g_sleep_flag == END_SLEEP_MODE && g_auto_sleep_start_judge_flag == AUTO_DAILY_SLEEP_CHECK_FLAG && daily_sleep_offhand_flag_get() == 0)
    {
        g_sleep_flag = START_SLEEP_MODE;
        g_sleep_start_check_flag = 0;
        memset(&g_sleep_acc,0,sizeof(Sleep_Acc_Para));
        g_auto_sleep_start_judge_flag = 0;
    }
    if(end_sleep_vaild_acc_cnt > judge_end_sleep_threshold && g_sleep_flag == START_SLEEP_MODE && g_auto_sleep_start_judge_flag == AUTO_DAILY_SLEEP_CHECK_FLAG)
    {
        g_sleep_flag = END_SLEEP_MODE;
        g_sleep_end_check_flag = 0;
        memset(&g_sleep_acc,0,sizeof(Sleep_Acc_Para));
        g_auto_sleep_start_judge_flag = 0;
    }
    if(g_wake_indirect_move_cnt >= WAKE_INDIRECT_MOVE_CNT && g_sleep_flag == START_SLEEP_MODE && g_auto_sleep_start_judge_flag == AUTO_DAILY_SLEEP_CHECK_FLAG)
    {
        g_sleep_flag = END_SLEEP_MODE;
        g_sleep_end_check_flag = 0;
        memset(&g_sleep_acc,0,sizeof(Sleep_Acc_Para));
        g_auto_sleep_start_judge_flag = 0;
    }

    // if(g_sleep_flag == START_SLEEP_MODE)
    // {
    //     g_start_auto_lowPowr_sleep_cnt = g_start_auto_lowPowr_sleep_cnt + 1;
    // }
    //NRF_LOG_INFO("==========g_sleep_flag = %d\n",g_sleep_flag);
    
    return g_sleep_flag;
}


FIFO32 green_health_fifo;
//int* spo2_green = (int*)malloc(HRV_PROCESS_DATA_PERIOD * sizeof(int));
int health_green[HRV_PROCESS_DATA_PERIOD] = {0};
uint8_t start_circle_proc_hrv(uint8_t type)
{
    uint8_t ret = NRF_SUCCESS;
    g_type_health = type;
    if(g_hrv_health_process_flag == HRV_HEALTH_PROCESS_START_FLAG)
    {
        ret = NRF_ERROR_BUSY;
        return ret;
    }
    // if (health_green != NULL) {
	// 	nrf_free(health_green);
	// 	health_green = NULL;
	// }

    // health_green = (int*)nrf_malloc(HRV_PROCESS_DATA_PERIOD*sizeof(int));
    
    // if (health_green == NULL)
    // {
    //     return NRF_ERROR_DATA_SIZE;
    // }

    fifo32_init(&green_health_fifo, HRV_PROCESS_DATA_PERIOD, health_green);
    fifo32_recover(&green_health_fifo, 0);
    // char name_1[] = "db4";
    // char* name = name_1;
    // WaveobjHealth = wave_init(name);// Initialize the wavelet
    data_start_pos = 0;
    pre_last_peak_pos = 0;
    greenRRLen = 0;
    
    g_time_cnt = 0;
    memset(green_smooth,0,HRV_PROCESS_DATA_PERIOD*sizeof(int));
    // memset(green_RR,0,HRV_MAX_DATA_LEN*sizeof(uint16_t));
    // memset(green_RR_smooth,0,HRV_MAX_DATA_LEN*sizeof(float));
    memset(&hrv_health_para,0,sizeof(HRV_Para));
    hrv_health_process_flag_set(HRV_HEALTH_PROCESS_START_FLAG);
    memset(&g_stress_acc,0,sizeof(Stress_Acc_Para));
    return ret;
    //FFTobjHealth = fft_init(FFT_LENGTH, 1);
}


void PPG_data_RR_get(uint16_t* RRData, int RRLen, HRV_Para* hrv_health_para_out)
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
        if (RRData[num + 1] - RRData[num] < 30)
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
        if(RRData[RRLen - 1] - RRData[RRLen - 2] > 30)
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[RRLen - 1];
            hrv_health_para_out->RR_num++;
        }
    }


    for (int num = pre_rr_end_pos;num < hrv_health_para_out->RR_num - 1;num++)
    {
        hrv_health_para_out->green_RR[num] = (hrv_health_para_out->green_RR[num + 1] - hrv_health_para_out->green_RR[num]);
    }
    hrv_health_para_out->green_RR[hrv_health_para_out->RR_num - 1] = (hrv_health_para_out->green_RR[hrv_health_para_out->RR_num - 2] + hrv_health_para_out->green_RR[hrv_health_para_out->RR_num - 3]) / 2;
}
void PPG_data_RR_get_new(uint16_t* RRData, int RRLen, HRV_Para* hrv_health_para_out)
{

    for (int num = 0;num < RRLen - 1;num++)
    {
        hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num+1] - RRData[num];
        hrv_health_para_out->RR_num++;
    }
#if 0
    int pre_flag = 0;
    int pre_rr_end_pos = 0;
    pre_rr_end_pos = hrv_health_para_out->RR_num;
    for (int num = 0; num < RRLen - 1; num++)
    {
        if (pre_flag == 1)
        {
            pre_flag = 0;
            continue;
        }
        if (RRData[num + 1] - RRData[num] < 30)
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num + 1];
            hrv_health_para_out->RR_num++;
            pre_flag = 1;
        }
        else
        {
            hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num];
            hrv_health_para_out->RR_num++;
            if (num + 1 == RRLen - 1)
            {
                hrv_health_para_out->green_RR[hrv_health_para_out->RR_num] = RRData[num + 1];
                hrv_health_para_out->RR_num++;
            }
        }

    }
    for (int num = pre_rr_end_pos; num < hrv_health_para_out->RR_num - 1; num++)
    {
        hrv_health_para_out->green_RR[num] = (hrv_health_para_out->green_RR[num + 1] - hrv_health_para_out->green_RR[num]);
    }
    hrv_health_para_out->green_RR[hrv_health_para_out->RR_num - 1] = 0;
    hrv_health_para_out->RR_num = hrv_health_para_out->RR_num - 1;
#endif
}

float stress_heart_green_smooth[HRV_PROCESS_DATA_PERIOD] = { 0 };
uint16_t stress_heart_green_RR_peaks[HRV_MAX_PEAKS_DATA_LEN] = { 0 };
uint8_t g_heart_record[2] = {68,68};
//int g_rr_record[3] = { 0 };
// int g_raw_heart[2000] = {0};
// int g_raw_heart_cnt = 0;
void stress_modules_heart_get(int* greenData, int dataLen, uint8_t* heartRate)
{
    int peaks_num = 0;
    memset(stress_heart_green_smooth,0, HRV_PROCESS_DATA_PERIOD*sizeof(float));
    memset(stress_heart_green_RR_peaks, 0, HRV_MAX_PEAKS_DATA_LEN * sizeof(uint16_t));
    smooth_data(greenData, HRV_SMOOTH_SPAN, HRV_PROCESS_DATA_PERIOD, stress_heart_green_smooth);

    PPG_data_wavelet(stress_heart_green_smooth, HRV_PROCESS_DATA_PERIOD, stress_heart_green_smooth);
    //PPG_data_wavelet(green_smooth, HRV_PROCESS_DATA_PERIOD, green_data_wavelet);
    //printf("period algorithm process finish\n");
    PPG_data_RR_peaks_get(stress_heart_green_smooth, HRV_PROCESS_DATA_PERIOD, stress_heart_green_RR_peaks, &peaks_num);
    uint16_t HheartRate = 0;
    for (int num = 0; num < peaks_num - 1; num++)
    {
        HheartRate = HheartRate+ stress_heart_green_RR_peaks[num + 1] - stress_heart_green_RR_peaks[num];

    }
    *heartRate = (uint8_t)(HheartRate / (peaks_num - 1));
    *heartRate = 6000 / (*heartRate);

    // g_raw_heart[g_raw_heart_cnt] = *heartRate;
    // g_raw_heart_cnt++;

    if (g_heart_record[0] != 0 && g_heart_record[1] != 0  && *heartRate - g_heart_record[1] > 5)
    {
        *heartRate = (*heartRate + g_heart_record[0] + g_heart_record[1]) / 3;
    }
    else if(g_heart_record[1] != 0  && g_heart_record[1] - *heartRate > 5)
    {
        *heartRate = (*heartRate + g_heart_record[1]) / 2;
    }
    g_heart_record[0] = g_heart_record[1];
    g_heart_record[1] = *heartRate;

}
uint16_t green_RR_peaks[HRV_MAX_PEAKS_DATA_LEN] = { 0 };


void HRV_health_data_get(void)
{
    int upPeakValueMean = 0;
    int acc_invaild_num = 0;
    float PPGfilter = 0; 
    if (fifo32_status(&green_health_fifo) >= HRV_PROCESS_DATA_PERIOD && hrv_health_process_flag_get() == HRV_HEALTH_PROCESS_START_FLAG)
    {
        // if(greenRRLen >= HRV_MAX_DATA_LEN)
        // {
        //     g_time_cnt = PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD;
        //     return;
        // }
        //memcpy(period_process_data, spo2_green, HRV_PROCESS_DATA_PERIOD * sizeof(int));
        //g_time_cnt = g_time_cnt + 1;
        for(int num_acc = 0;num_acc < g_stress_acc.acc_len;num_acc++)
        {
            if(g_stress_acc.acc_value[num_acc] >= STRESS_VAILD_THRESHOLD)
            {
                acc_invaild_num = acc_invaild_num + 1;
            }
            if(acc_invaild_num > 3)
            {
                memset(&g_stress_acc,0,sizeof(Stress_Acc_Para));
                fifo32_recover(&green_health_fifo, 0);
                hrv_health_para.green_RR[hrv_health_para.RR_num] = 255;
                hrv_health_para.RR_num++;
                g_time_cnt = g_time_cnt + 1;
                return;
            }

        }


        // smooth_data(health_green, HRV_SMOOTH_SPAN, HRV_PROCESS_DATA_PERIOD, green_smooth);
        smooth_data_int_out_int(health_green, HRV_SMOOTH_SPAN, HRV_PROCESS_DATA_PERIOD, green_smooth);
        // PPG_data_wavelet(green_smooth, HRV_PROCESS_DATA_PERIOD, green_smooth);
        for(int num = 0;num < HRV_PROCESS_DATA_PERIOD;num++)
        {
            // green_smooth[num] = IIR_bandpass_Filter(green_smooth[num]);
            PPGfilter = green_smooth[num];
            green_smooth[num] = IIR_bandpass_chebyshev_Filter_heart(PPGfilter);
            
        }

        //PPG_data_wavelet(green_smooth, HRV_PROCESS_DATA_PERIOD, green_data_wavelet);
        //printf("period algorithm process finish\n");


        PPG_data_RR_peaks_get_heart_2(green_smooth, HRV_PROCESS_DATA_PERIOD, green_RR_peaks, &greenRRLen,&upPeakValueMean);
        PPG_data_RR_get_heart_point_2(green_RR_peaks, greenRRLen, &hrv_health_para, green_smooth,HRV_PROCESS_DATA_PERIOD,upPeakValueMean);
        
       // PPG_data_RR_get_heart_2(green_RR_peaks, greenRRLen, &hrv_health_para, green_smooth,HRV_PROCESS_DATA_PERIOD,upPeakValueMean);
       
       // PPG_data_RR_peaks_get(green_smooth, HRV_PROCESS_DATA_PERIOD, green_RR_peaks, &greenRRLen);
      // PPG_data_RR_get(green_RR_peaks, greenRRLen, &hrv_health_para);

        //PPG_data_RR_get(green_smooth, HRV_PROCESS_DATA_PERIOD, &hrv_health_para.green_RR, &greenRRLen);
        //PPG_data_RR_get(green_data_wavelet, HRV_PROCESS_DATA_PERIOD, green_RR, &greenRRLen);
        fifo32_recover(&green_health_fifo, 0);
        //fifo32_recover(&green_health_fifo, HRV_PROCESS_DATA_PERIOD - (pre_last_peak_pos + 10));
        //fifo32_recover(&green_health_fifo, HRV_PROCESS_DATA_PERIOD - (pre_last_peak_pos + 5));
        memset(&g_stress_acc,0,sizeof(Stress_Acc_Para));
        memset(green_RR_peaks,0, HRV_MAX_PEAKS_DATA_LEN*sizeof(uint16_t));
        greenRRLen = 0;
        g_time_cnt = g_time_cnt + 1;
    }
    if(g_type_health == HEALTH_NON_PROFESSIONAL_TYPE)
    {
        if (g_time_cnt >= NON_PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD)
        {
            // int peaks_num = 0;
            // for (peaks_num = 0; peaks_num < greenRRLen - 1; peaks_num++)
            // {
            //     hrv_health_para.green_RR[peaks_num] = (hrv_health_para.green_RR[peaks_num + 1] - hrv_health_para.green_RR[peaks_num]);
            // }
            // hrv_health_para.green_RR[greenRRLen - 1] = 0;
            // hrv_health_para.RR_num = greenRRLen - 1;
            // //smooth_data_f(green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
            // smooth_data_16uint(&hrv_health_para.green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
            // get_HrvTimePara(green_RR_smooth, peaks_num, &hrv_health_para.tTimePara);
            // get_HrvGeomPara(green_RR_smooth, peaks_num, &hrv_health_para.tGeomPara);
            // hrv_health_para.stressIndex = 500 - hrv_health_para.tTimePara.SDNN;
            // hrv_health_para.fatigueIndex = 500 - hrv_health_para.tGeomPara.SD1;
            g_time_cnt = 0;
            hrv_health_process_flag_set(HRV_HEALTH_PROCESS_FINISH_FLAG);

        }
    }
    else if(g_type_health == HEALTH_PROFESSIONAL_TYPE)
    {
        if (g_time_cnt >= PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD)
        {
            // int peaks_num = 0;
            // for (peaks_num = 0; peaks_num < greenRRLen - 1; peaks_num++)
            // {
            //     hrv_health_para.green_RR[peaks_num] = (hrv_health_para.green_RR[peaks_num + 1] - hrv_health_para.green_RR[peaks_num]);
            // }
            // hrv_health_para.green_RR[greenRRLen - 1] = 0;
            // hrv_health_para.RR_num = greenRRLen - 1;
            // //smooth_data_f(green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
            // smooth_data_16uint(&hrv_health_para.green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
            // get_HrvTimePara(green_RR_smooth, peaks_num, &hrv_health_para.tTimePara);
            // get_HrvGeomPara(green_RR_smooth, peaks_num, &hrv_health_para.tGeomPara);
            // hrv_health_para.stressIndex = 500 - hrv_health_para.tTimePara.SDNN;
            // hrv_health_para.fatigueIndex = 500 - hrv_health_para.tGeomPara.SD1;
            g_time_cnt = 0;
            hrv_health_process_flag_set(HRV_HEALTH_PROCESS_FINISH_FLAG);
        }

    }




}

void HRV_health_data_get_daily_algorithm_rr(void)
{
    int upPeakValueMean = 0;
    int acc_invaild_num = 0;
    float PPGfilter = 0; 
    int AccHRV = 0;
    int AccHRVlen = 0;
    AlgProc_Para AlgPara_point = {0};
    if (fifo32_status(&green_health_fifo) >= HRV_PROCESS_DATA_PERIOD && hrv_health_process_flag_get() == HRV_HEALTH_PROCESS_START_FLAG)
    {
        Sleep_PPG_data_RR_get_new(health_green,HRV_PROCESS_DATA_PERIOD, &AccHRV, AccHRVlen,&AlgPara_point);
        // Sleep_PPG_data_RR_get(health_green,HRV_PROCESS_DATA_PERIOD, &AccHRV, AccHRVlen,&AlgPara_point);
        for(int num = 0;num < AlgPara_point.HrCnt;num++)
        {
            hrv_health_para.green_RR[hrv_health_para.RR_num] = AlgPara_point.HrVect[num];
            hrv_health_para.RR_num = hrv_health_para.RR_num + 1;
        }
        

        fifo32_recover(&green_health_fifo, HRV_PROCESS_DATA_PERIOD - HRV_PROCESS_DATA_PERIOD_NOW_ALGORITHM);

        // memset(&g_stress_acc,0,sizeof(Stress_Acc_Para));

        g_time_cnt = g_time_cnt + 1;
    }
    if(g_type_health == HEALTH_NON_PROFESSIONAL_TYPE)
    {
        if (g_time_cnt >= NON_PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD_NOW_ALGORITHM)
        {
            // int peaks_num = 0;
            // for (peaks_num = 0; peaks_num < greenRRLen - 1; peaks_num++)
            // {
            //     hrv_health_para.green_RR[peaks_num] = (hrv_health_para.green_RR[peaks_num + 1] - hrv_health_para.green_RR[peaks_num]);
            // }
            // hrv_health_para.green_RR[greenRRLen - 1] = 0;
            // hrv_health_para.RR_num = greenRRLen - 1;
            // //smooth_data_f(green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
            // smooth_data_16uint(&hrv_health_para.green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
            // get_HrvTimePara(green_RR_smooth, peaks_num, &hrv_health_para.tTimePara);
            // get_HrvGeomPara(green_RR_smooth, peaks_num, &hrv_health_para.tGeomPara);
            // hrv_health_para.stressIndex = 500 - hrv_health_para.tTimePara.SDNN;
            // hrv_health_para.fatigueIndex = 500 - hrv_health_para.tGeomPara.SD1;
            g_time_cnt = 0;
            hrv_health_process_flag_set(HRV_HEALTH_PROCESS_FINISH_FLAG);

        }
    }
    else if(g_type_health == HEALTH_PROFESSIONAL_TYPE)
    {
        if (g_time_cnt >= PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD_NOW_ALGORITHM)
        {
            // int peaks_num = 0;
            // for (peaks_num = 0; peaks_num < greenRRLen - 1; peaks_num++)
            // {
            //     hrv_health_para.green_RR[peaks_num] = (hrv_health_para.green_RR[peaks_num + 1] - hrv_health_para.green_RR[peaks_num]);
            // }
            // hrv_health_para.green_RR[greenRRLen - 1] = 0;
            // hrv_health_para.RR_num = greenRRLen - 1;
            // //smooth_data_f(green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
            // smooth_data_16uint(&hrv_health_para.green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
            // get_HrvTimePara(green_RR_smooth, peaks_num, &hrv_health_para.tTimePara);
            // get_HrvGeomPara(green_RR_smooth, peaks_num, &hrv_health_para.tGeomPara);
            // hrv_health_para.stressIndex = 500 - hrv_health_para.tTimePara.SDNN;
            // hrv_health_para.fatigueIndex = 500 - hrv_health_para.tGeomPara.SD1;
            g_time_cnt = 0;
            hrv_health_process_flag_set(HRV_HEALTH_PROCESS_FINISH_FLAG);
        }

    }




}

// void HRV_health_data_get_low_power(void)
// {
//     int acc_invaild_num = 0;
//     if (fifo32_status(&green_health_fifo) >= HRV_PROCESS_DATA_PERIOD && hrv_health_process_flag_get() == HRV_HEALTH_PROCESS_START_FLAG)
//     {
//         // if(greenRRLen >= HRV_MAX_DATA_LEN)
//         // {
//         //     g_time_cnt = PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD;
//         //     return;
//         // }
//         //memcpy(period_process_data, spo2_green, HRV_PROCESS_DATA_PERIOD * sizeof(int));
//         g_time_cnt = g_time_cnt + 1;
//         for(int num_acc = 0;num_acc < g_stress_acc.acc_len;num_acc++)
//         {
//             if(g_stress_acc.acc_value[num_acc] >= STRESS_VAILD_THRESHOLD)
//             {
//                 acc_invaild_num = acc_invaild_num + 1;
//             }
//             if(acc_invaild_num > 3)
//             {
//                 memset(&g_stress_acc,0,sizeof(Stress_Acc_Para));
//                 fifo32_recover(&green_health_fifo, 0);
//                 hrv_health_para.green_RR[hrv_health_para.RR_num] = 255;
//                 hrv_health_para.RR_num++;
//                 return;
//             }

//         }


//         smooth_data(health_green, HRV_SMOOTH_SPAN, HRV_PROCESS_DATA_PERIOD, green_smooth);

//         PPG_data_wavelet(green_smooth, HRV_PROCESS_DATA_PERIOD, green_smooth);
//         //PPG_data_wavelet(green_smooth, HRV_PROCESS_DATA_PERIOD, green_data_wavelet);
//         //printf("period algorithm process finish\n");
//         PPG_data_RR_peaks_get(green_smooth, HRV_PROCESS_DATA_PERIOD, green_RR_peaks, &greenRRLen);
//         PPG_data_RR_get(green_RR_peaks, greenRRLen, &hrv_health_para);
//         //PPG_data_RR_get(green_smooth, HRV_PROCESS_DATA_PERIOD, &hrv_health_para.green_RR, &greenRRLen);
//         //PPG_data_RR_get(green_data_wavelet, HRV_PROCESS_DATA_PERIOD, green_RR, &greenRRLen);
//         fifo32_recover(&green_health_fifo, 0);
//         //fifo32_recover(&green_health_fifo, HRV_PROCESS_DATA_PERIOD - (pre_last_peak_pos + 10));
//         //fifo32_recover(&green_health_fifo, HRV_PROCESS_DATA_PERIOD - (pre_last_peak_pos + 5));
//         memset(&g_stress_acc,0,sizeof(Stress_Acc_Para));
//         memset(green_RR_peaks,0, HRV_MAX_PEAKS_DATA_LEN*sizeof(uint16_t));
//         greenRRLen = 0;
//         //g_time_cnt = g_time_cnt + 1;
//     }
//     if(g_type_health == HEALTH_NON_PROFESSIONAL_TYPE)
//     {
//         if (g_time_cnt >= NON_PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD)//60s
//         {
//             // int peaks_num = 0;
//             // for (peaks_num = 0; peaks_num < greenRRLen - 1; peaks_num++)
//             // {
//             //     hrv_health_para.green_RR[peaks_num] = (hrv_health_para.green_RR[peaks_num + 1] - hrv_health_para.green_RR[peaks_num]);
//             // }
//             // hrv_health_para.green_RR[greenRRLen - 1] = 0;
//             // hrv_health_para.RR_num = greenRRLen - 1;
//             // //smooth_data_f(green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
//             // smooth_data_16uint(&hrv_health_para.green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
//             // get_HrvTimePara(green_RR_smooth, peaks_num, &hrv_health_para.tTimePara);
//             // get_HrvGeomPara(green_RR_smooth, peaks_num, &hrv_health_para.tGeomPara);
//             // hrv_health_para.stressIndex = 500 - hrv_health_para.tTimePara.SDNN;
//             // hrv_health_para.fatigueIndex = 500 - hrv_health_para.tGeomPara.SD1;
//             g_time_cnt = 0;
//             hrv_health_process_flag_set(HRV_HEALTH_PROCESS_FINISH_FLAG);

//         }
//     }
//     else if(g_type_health == HEALTH_PROFESSIONAL_TYPE)
//     {
//         if (g_time_cnt >= PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD)
//         {
//             // int peaks_num = 0;
//             // for (peaks_num = 0; peaks_num < greenRRLen - 1; peaks_num++)
//             // {
//             //     hrv_health_para.green_RR[peaks_num] = (hrv_health_para.green_RR[peaks_num + 1] - hrv_health_para.green_RR[peaks_num]);
//             // }
//             // hrv_health_para.green_RR[greenRRLen - 1] = 0;
//             // hrv_health_para.RR_num = greenRRLen - 1;
//             // //smooth_data_f(green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
//             // smooth_data_16uint(&hrv_health_para.green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
//             // get_HrvTimePara(green_RR_smooth, peaks_num, &hrv_health_para.tTimePara);
//             // get_HrvGeomPara(green_RR_smooth, peaks_num, &hrv_health_para.tGeomPara);
//             // hrv_health_para.stressIndex = 500 - hrv_health_para.tTimePara.SDNN;
//             // hrv_health_para.fatigueIndex = 500 - hrv_health_para.tGeomPara.SD1;
//             g_time_cnt = 0;
//             hrv_health_process_flag_set(HRV_HEALTH_PROCESS_FINISH_FLAG);
//         }

//     }




// }

uint32_t g_pre_period_end_peaks_pos = 0;

// void HRV_health_data_get_new(void)
// {
//     int acc_invaild_num = 0;
//     float period_mean_value = 0;
//     uint8_t ret = INVALID_PERIOD;
//     if (fifo32_status(&green_health_fifo) >= HRV_PROCESS_DATA_PERIOD && hrv_health_process_flag_get() == HRV_HEALTH_PROCESS_START_FLAG)
//     {
//         // if(greenRRLen >= HRV_MAX_DATA_LEN)
//         // {
//         //     g_time_cnt = PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD;
//         //     return;
//         // }
//         //memcpy(period_process_data, spo2_green, HRV_PROCESS_DATA_PERIOD * sizeof(int));


//         //for(int num_acc = 0;num_acc < g_stress_acc.acc_len;num_acc++)
//         //{
//         //    if(g_stress_acc.acc_value[num_acc] >= STRESS_VAILD_THRESHOLD)
//         //    {
//         //        acc_invaild_num = acc_invaild_num + 1;
//         //    }
//         //    if(acc_invaild_num > 3)
//         //    {
//         //        memset(&g_stress_acc,0,sizeof(Stress_Acc_Para));
//         //        fifo32_recover(&green_health_fifo, 0);
//         //        return;
//         //    }

//         //}


//         g_all_time_cnt = g_all_time_cnt + 1;
//         smooth_data(health_green, HRV_SMOOTH_SPAN, HRV_PROCESS_DATA_PERIOD, green_smooth);

//         PPG_data_wavelet(green_smooth, HRV_PROCESS_DATA_PERIOD, green_smooth);
//         for (int num = 0;num < HRV_PROCESS_DATA_PERIOD;num++)
//         {
//             period_mean_value = period_mean_value + green_smooth[num];
//         }
//         period_mean_value = period_mean_value / HRV_PROCESS_DATA_PERIOD;
//         //printf("period_mean_value = %f\n",period_mean_value);
//         if (period_mean_value > -1*HRV_PROCESS_DATA_PERIOD_VALID_THRESHOLD && period_mean_value < HRV_PROCESS_DATA_PERIOD_VALID_THRESHOLD)
//         {
//             //printf("period algorithm process finish\n");
//             ret = PPG_data_RR_peaks_get_new(green_smooth, HRV_PROCESS_DATA_PERIOD, green_RR_peaks, &greenRRLen);

//             if ((g_all_time_cnt - 1) * HRV_PROCESS_DATA_PERIOD + green_RR_peaks[0] - g_pre_period_end_peaks_pos > 30 && (g_all_time_cnt - 1) * HRV_PROCESS_DATA_PERIOD + green_RR_peaks[0] - g_pre_period_end_peaks_pos < 100)
//             {
//                 hrv_health_para.green_RR[hrv_health_para.RR_num] = (g_all_time_cnt - 1) * HRV_PROCESS_DATA_PERIOD + green_RR_peaks[0] - g_pre_period_end_peaks_pos;
//                 hrv_health_para.RR_num++;
//             }

//             PPG_data_RR_get_new(green_RR_peaks, greenRRLen, &hrv_health_para);

//             g_pre_period_end_peaks_pos = (g_all_time_cnt - 1) * HRV_PROCESS_DATA_PERIOD;
//             if (greenRRLen >= 2)
//             {
//                 g_pre_period_end_peaks_pos = g_pre_period_end_peaks_pos + green_RR_peaks[greenRRLen - 1];
//             }

//             if (ret == VALID_PERIOD)
//             {
//                 NRF_LOG_INFO("stress valid once---------");
//                 g_time_cnt = g_time_cnt + 1;
//             }
            
//         }
//         else
//         {
//             g_pre_period_end_peaks_pos = (g_all_time_cnt - 1) * HRV_PROCESS_DATA_PERIOD;
//         }
//         //PPG_data_RR_get(green_smooth, HRV_PROCESS_DATA_PERIOD, &hrv_health_para.green_RR, &greenRRLen);
//         //PPG_data_RR_get(green_data_wavelet, HRV_PROCESS_DATA_PERIOD, green_RR, &greenRRLen);
//         fifo32_recover(&green_health_fifo, 0);
//         //fifo32_recover(&green_health_fifo, HRV_PROCESS_DATA_PERIOD - (pre_last_peak_pos + 10));
//         //fifo32_recover(&green_health_fifo, HRV_PROCESS_DATA_PERIOD - (pre_last_peak_pos + 5));
//         memset(&g_stress_acc,0,sizeof(Stress_Acc_Para));
//         memset(green_RR_peaks,0, HRV_MAX_PEAKS_DATA_LEN*sizeof(uint16_t));
//         greenRRLen = 0;
//         g_time_cnt = g_time_cnt + 1;
//         NRF_LOG_INFO("g_time_cnt: %d", g_time_cnt);
//     }
//     if(g_type_health == HEALTH_NON_PROFESSIONAL_TYPE)
//     {
//         if (g_time_cnt >= NON_PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD)
//         {
//             // int peaks_num = 0;
//             // for (peaks_num = 0; peaks_num < greenRRLen - 1; peaks_num++)
//             // {
//             //     hrv_health_para.green_RR[peaks_num] = (hrv_health_para.green_RR[peaks_num + 1] - hrv_health_para.green_RR[peaks_num]);
//             // }
//             // hrv_health_para.green_RR[greenRRLen - 1] = 0;
//             // hrv_health_para.RR_num = greenRRLen - 1;
//             // //smooth_data_f(green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
//             // smooth_data_16uint(&hrv_health_para.green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
//             // get_HrvTimePara(green_RR_smooth, peaks_num, &hrv_health_para.tTimePara);
//             // get_HrvGeomPara(green_RR_smooth, peaks_num, &hrv_health_para.tGeomPara);
//             // hrv_health_para.stressIndex = 500 - hrv_health_para.tTimePara.SDNN;
//             // hrv_health_para.fatigueIndex = 500 - hrv_health_para.tGeomPara.SD1;
//             g_time_cnt = 0;
//             hrv_health_process_flag_set(HRV_HEALTH_PROCESS_FINISH_FLAG);

//         }
//     }
//     else if(g_type_health == HEALTH_PROFESSIONAL_TYPE)
//     {
//         if (g_time_cnt >= PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME / HRV_PROCESS_DATA_PERIOD)
//         {
//             // int peaks_num = 0;
//             // for (peaks_num = 0; peaks_num < greenRRLen - 1; peaks_num++)
//             // {
//             //     hrv_health_para.green_RR[peaks_num] = (hrv_health_para.green_RR[peaks_num + 1] - hrv_health_para.green_RR[peaks_num]);
//             // }
//             // hrv_health_para.green_RR[greenRRLen - 1] = 0;
//             // hrv_health_para.RR_num = greenRRLen - 1;
//             // //smooth_data_f(green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
//             // smooth_data_16uint(&hrv_health_para.green_RR, HRV_TIME_PARA_SMOOTH_SPAN, peaks_num, green_RR_smooth);
//             // get_HrvTimePara(green_RR_smooth, peaks_num, &hrv_health_para.tTimePara);
//             // get_HrvGeomPara(green_RR_smooth, peaks_num, &hrv_health_para.tGeomPara);
//             // hrv_health_para.stressIndex = 500 - hrv_health_para.tTimePara.SDNN;
//             // hrv_health_para.fatigueIndex = 500 - hrv_health_para.tGeomPara.SD1;
//             g_time_cnt = 0;
//             hrv_health_process_flag_set(HRV_HEALTH_PROCESS_FINISH_FLAG);
//         }

//     }




// }

