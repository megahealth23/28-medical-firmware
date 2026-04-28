/*********************************************************************
* @fn      filter_datapre.c
*
* @brief   get filter for data
*
*/

#include "filter_datapre.h"

// initialize of filter coefficient.
float x1[5] = { 0,0,0,0,0 };
float y1[5] = { 0,0,0,0,0 };
float x2[5] = { 0,0,0,0,0 };
float y2[5] = { 0,0,0,0,0 };
float x3[5] = { 0,0,0,0,0 };
float y3[5] = { 0,0,0,0,0 };
float x4[5] = { 0,0,0,0,0 };
float y4[5] = { 0,0,0,0,0 };

float x5[5] = { 0,0,0,0,0 };
float y5[5] = { 0,0,0,0,0 };

float tempresp1[5] = { 0,0,0,0,0 };
float tempresp2[5] = { 0,0,0,0,0 };
float tempresp3[5] = { 0,0,0,0,0 };
int tempresp4[5] = { 0,0,0,0,0 };
float d[5] = { 1,-3.1806,3.8612,-2.1122,0.4383 }; //{1,-2.6646,2.6098,-1.2049,0.2600}
float n[5] = { 0.0004,0.0017,0.0025,0.0017,0.0004 }; //{ 0.1396 ,0,-0.2792,0,0.1396}


// bandpass filter 04-5HZ
//float d_IIR[5] = { 1,-3.76341591984671,5.36261433245638,-3.43315515452697,0.834045845369074 }; //{1,-2.6646,2.6098,-1.2049,0.2600}
//float n_IIR[5] = { 0.00917775443917120,0,-0.0183555088783424,0,0.00917775443917120 }; //{ 0.1396 ,0,-0.2792,0,0.1396}
float d_IIR[5] = { 1,-3.7634,5.3626,-3.4331,0.8340 }; //{1,-2.6646,2.6098,-1.2049,0.2600}
float n_IIR[5] = { 0.0091,0,-0.0183,0,0.0091}; //{ 0.1396 ,0,-0.2792,0,0.1396}




float x6[5] = { 0,0,0,0,0 };
float y6[5] = { 0,0,0,0,0 };
float IIR_bandpass_chebyshev_Filter_heart_sport(float EEG)
{
	float trapFilter = 0;
	//%IIR bandpass filter  0.5HZ--5HZ    window:butterworth
	//float b[5] = { 0.0165819316693031,0,-0.0331638633386062,0,0.0165819316693031 };
	//float a[5] = { 1,-3.58623980811691,4.84628980428802,-2.93042721682013,0.670457905953174 };
	//float b[5] = { 0.01658193166,0,-0.03316386333,0,0.01658193166 };
	//float a[5] = {1,-3.586239808,4.846289804,-2.930427216,0.6704579059};

	//IIR bandpass filter  0.5HZ--5HZ    window:butterworth
	//float b[5] = { 0.0165819316693031,0,-0.0331638633386062,0,0.0165819316693031 };
	//float a[5] = { 1,-3.58623980811691,4.84628980428802,-2.93042721682013,0.670457905953174 };

	//IIR passBand filter  0.7HZ--5HZ  window:butterworth
	//float b[5] = { 0.0152584592128717,0,-0.0305169184257434,0,0.0152584592128717 };
	//float a[5] = { 1,-3.59629273837731,4.87962437339592,-2.96564274935047,0.682470382179842 };


	
	// IIR CHebyshev I passBand filter  0.7HZ--5HZ    OK
	float b[5] = { 0.0437391131996750, 0, -0.0874782263993500, 0, 0.0437391131996750 };
	float a[5] = { 1, -3.33364027509393, 4.20782187564081, -2.40797231970520, 0.533930076560356 };


	//IIR passBand filter  0.1HZ--5HZ  0.1dB  chebyshevI
	//float b[5] = { 0.0544766520255661,0,-0.108953304051132,0,0.0544766520255661 };
	//float a[5] = { 1,-3.26784405853578,4.02822993158898,-2.25191069944301,0.491527559628229};
#if 0
	if (x5[0] == 0 && g_cnt == 1)
	{
		x5[0] = EEG;
		y5[0] = b[0] * x5[0];
		trapFilter = y5[0];
		return trapFilter;
	}
	if (x5[1] == 0 && g_cnt == 2)
	{
		x5[1] = EEG;
		y5[1] = b[0] * x5[1] + b[1] * x5[0] - a[1] * y5[0];
		trapFilter = y5[1];
		return trapFilter;
	}
	if (x5[2] == 0 && g_cnt == 3)
	{
		x5[2] = EEG;
		y5[2] = b[0] * x5[2] + b[1] * x5[1] + b[2] * x5[0] - a[2] * y5[0] - a[1] * y5[1];
		trapFilter = y5[2];
		return trapFilter;
	}
	if (x5[3] == 0 && g_cnt == 4)
	{
		x5[3] = EEG;
		y5[3] = b[0] * x5[3] + b[1] * x5[2] + b[2] * x5[1] + b[1] * x5[0] - a[3] * y5[0] - a[2] * y5[1] - a[1] * y5[2];
		trapFilter = y5[3];
		return trapFilter;
	}


	if (g_cnt <= 4)
	{
		g_cnt = g_cnt + 1;
	}
#endif
	x6[4] = EEG;


	y6[4] = (b[0] * x6[4] + b[1] * x6[3] + b[2] * x6[2] + b[3] * x6[1] + b[4] * x6[0] - a[1] * y6[3] - a[2] * y6[2] - a[3] * y6[1] - a[4] * y6[0]) / a[0];
	trapFilter = y6[4];

	x6[0] = x6[1];
	x6[1] = x6[2];
	x6[2] = x6[3];
	x6[3] = x6[4];



	y6[0] = y6[1];
	y6[1] = y6[2];
	y6[2] = y6[3];
	y6[3] = y6[4];

	return trapFilter;
}







/*********************************************************************
* @fn      IIR_bandpass_Filter()
*
* @brief   IIR bandpass filter  0.5HZ--5HZ    window:butterworth
*
* @param   float input
* @return  .
*/

float IIR_bandpass_chebyshev_Filter_heart(float EEG)
{
	float trapFilter = 0;
	//IIR bandpass filter  0.5HZ--5HZ    window:butterworth
	float b[5] = { 0.0165819316693031,0,-0.0331638633386062,0,0.0165819316693031 };
	float a[5] = {1,-3.58623980811691,4.84628980428802,-2.93042721682013,0.670457905953174};
	//float b[5] = { 0.0544766520255661,0,-0.108953304051132,0,0.0544766520255661 };
	//float a[5] = { 1,-3.26784405853578,4.02822993158898,-2.25191069944301,0.491527559628229};

	// if (x5[0] == 0)
	// {
	// 	x5[0] = EEG;
	// 	y5[0] = b[0] * x5[0];
	// 	trapFilter = y5[0];
	// 	return trapFilter;
	// }
	// if (x5[1] == 0)
	// {
	// 	x5[1] = EEG;
	// 	y5[1] = b[0] * x5[1] + b[1] * x5[0] - a[1] * y5[0];
	// 	trapFilter = y5[1];
	// 	return trapFilter;
	// }
	// if (x5[2] == 0)
	// {
	// 	x5[2] = EEG;
	// 	y5[2] = b[0] * x5[2] + b[1] * x5[1] + b[2] * x5[0] - a[2] * y5[0] - a[1] * y5[1];
	// 	trapFilter = y5[2];
	// 	return trapFilter;
	// }
	// if (x5[3] == 0)
	// {
	// 	x5[3] = EEG;
	// 	y5[3] = b[0] * x5[3] + b[1] * x5[2] + b[2] * x5[1] + b[1] * x5[0] - a[3] * y5[0] - a[2] * y5[1] - a[1] * y5[2];
	// 	trapFilter = y5[3];
	// 	return trapFilter;
	// }

	x5[4] = EEG;


	y5[4] = (b[0] * x5[4] + b[1] * x5[3] + b[2] * x5[2] + b[3] * x5[1] + b[4] * x5[0] - a[1] * y5[3] - a[2] * y5[2] - a[3] * y5[1] - a[4] * y5[0]) / a[0];
	trapFilter = y5[4];

	x5[0] = x5[1];
	x5[1] = x5[2];
	x5[2] = x5[3];
	x5[3] = x5[4];



	y5[0] = y5[1];
	y5[1] = y5[2];
	y5[2] = y5[3];
	y5[3] = y5[4];

	return trapFilter;
}










/*********************************************************************
* @fn      afe_IIRLPFilter4()
*
* @brief   bandpass filter 04-5HZ
*
* @param   float input
* @return  .
*/
float afe_IIRLPFilter4(float input)
{
	x4[0] = input;
	y4[0] = (x4[0])*n_IIR[0] + (x4[1])*n_IIR[1] + (x4[2])*n_IIR[2] + (x4[3])*n_IIR[3] + (x4[4])*n_IIR[4] - (y4[1]) *d_IIR[1] - (y4[2])*d_IIR[2] - (y4[3])*d_IIR[3] - (y4[4])*d_IIR[4];
	x4[4] = x4[3];
	x4[3] = x4[2];
	x4[2] = x4[1];
	x4[1] = x4[0];

	y4[4] = y4[3];
	y4[3] = y4[2];
	y4[2] = y4[1];
	y4[1] = y4[0];
	return y4[0];
}

/*********************************************************************
* @fn      IIR_bandpass_Filter()
*
* @brief   IIR bandpass filter 04-5HZ
*
* @param   float input
* @return  .
*/
float IIR_bandpass_Filter(float EEG)
{
	float trapFilter = 0;

	//butterworth band pass 0.4-5HZ
	float b[5] = {0.0172604506154174,0,-0.0345209012308349,0,0.0172604506154174};
	float a[5] = {1,-3.58120537333066,4.82965962202349,-2.91293462248089,0.664531830714488};


	//float b[5] = { 0.00917775443917120,0,-0.0183555088783424,0,0.00917775443917120 };
	//float a[5] = { 1,-3.76341591984671,5.36261433245638,-3.43315515452697,0.834045845369074 };




	// if (x4[0] == 0)
	// {
	// 	x4[0] = EEG;
	// 	y4[0] = b[0] * x4[0];
	// 	trapFilter = y4[0];
	// 	return trapFilter;
	// }
	// if (x4[1] == 0)
	// {
	// 	x4[1] = EEG;
	// 	y4[1] = b[0] * x4[1] + b[1] * x4[0] - a[1] * y4[0];
	// 	trapFilter = y4[1];
	// 	return trapFilter;
	// }
	// if (x4[2] == 0)
	// {
	// 	x4[2] = EEG;
	// 	y4[2] = b[0] * x4[2] + b[1] * x4[1] + b[2] * x4[0] - a[2] * y4[0] - a[1] * y4[1];
	// 	trapFilter = y4[2];
	// 	return trapFilter;
	// }
	// if (x4[3] == 0)
	// {
	// 	x4[3] = EEG;
	// 	y4[3] = b[0] * x4[3] + b[1] * x4[2] + b[2] * x4[1] + b[1] * x4[0] - a[3] * y4[0] - a[2] * y4[1] - a[1] * y4[2];
	// 	trapFilter = y4[3];
	// 	return trapFilter;
	// }

	x4[4] = EEG;


	y4[4] = (b[0] * x4[4] + b[1] * x4[3] + b[2] * x4[2] + b[3] * x4[1] + b[4] * x4[0] - a[1] * y4[3] - a[2] * y4[2] - a[3] * y4[1] - a[4] * y4[0]) / a[0];
	trapFilter = y4[4];
	
	x4[0] = x4[1];
	x4[1] = x4[2];
	x4[2] = x4[3];
	x4[3] = x4[4];
	
	
	
	y4[0] = y4[1];
	y4[1] = y4[2];
	y4[2] = y4[3];
	y4[3] = y4[4];
	
	return trapFilter;
}

// float IIR_bandpass_Filter(float EEG,)
// {
// 	float trapFilter = 0;
// 	float d_IIR[5] = { 1,-3.7634,5.3626,-3.4331,0.8340 }; //{1,-2.6646,2.6098,-1.2049,0.2600}
// 	float n_IIR[5] = { 0.0091,0,-0.0183,0,0.0091}; //{ 0.1396 ,0,-0.2792,0,0.1396}

// 	float b[3] = { 0.7600, - 0.4697, 0.7600 };
// 	float a[3] = { 1.0000, - 0.4697, 0.5201 };
// 	EEGTrapFilter[0] = b[0]* EEG[0];
// 	EEGTrapFilter[1] = b[0]* EEG[1] + b[1]*EEG[0] - a[1]* EEGTrapFilter[0];
// 	//EEGTrapFilterTest[0] = EEGTrapFilter[0];
// 	//EEGTrapFilterTest[1] = EEGTrapFilter[1];
// 	for (int num = 2;num < EEGDataLen-1;num++)
// 	{
// 		trapFilter = (n_IIR[0] * EEG[num] + n_IIR[1] * EEG[num - 1] + n_IIR[2] * EEG[num - 2] - a[1] * EEGTrapFilter[num - 1] - a[2] * EEGTrapFilter[num - 2]) / a[0];
// 		EEGTrapFilter[num] = trapFilter;
// 		//EEGTrapFilterTest[num] = trapFilter;
// 	}

// }

/*********************************************************************
* @fn      afe_IIRLPFilter1()
*
* @brief   filter
*
* @param   float input
* @return  .
*/
float afe_IIRLPFilter1(float input)
{
	x1[0] = input;
	y1[0] = (x1[0])*n[0] + (x1[1])*n[1] + (x1[2])*n[2] + (x1[3])*n[3] + (x1[4])*n[4] - (y1[1]) *d[1] - (y1[2])*d[2] - (y1[3])*d[3] - (y1[4])*d[4];
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

/*********************************************************************
* @fn      afe_IIRLPFilter2()
*
* @brief   filter
*
* @param   float input
* @return  .
*/
float afe_IIRLPFilter2(float input)
{
	x2[0] = input;
	y2[0] = (x2[0])*n[0] + (x2[1])*n[1] + (x2[2])*n[2] + (x2[3])*n[3] + (x2[4])*n[4] - (y2[1]) *d[1] - (y2[2])*d[2] - (y2[3])*d[3] - (y2[4])*d[4];
	x2[4] = x2[3];
	x2[3] = x2[2];
	x2[2] = x2[1];
	x2[1] = x2[0];

	y2[4] = y2[3];
	y2[3] = y2[2];
	y2[2] = y2[1];
	y2[1] = y2[0];
	return y2[0];
}

float afe_IIRLPFilter3(float input)
{
	x3[0] = input;
	y3[0] = (x3[0])*n[0] + (x3[1])*n[1] + (x3[2])*n[2] + (x3[3])*n[3] + (x3[4])*n[4] - (y3[1]) *d[1] - (y3[2])*d[2] - (y3[3])*d[3] - (y3[4])*d[4];
	x3[4] = x3[3];
	x3[3] = x3[2];
	x3[2] = x3[1];
	x3[1] = x3[0];

	y3[4] = y3[3];
	y3[3] = y3[2];
	y3[2] = y3[1];
	y3[1] = y3[0];
	return y3[0];
}

/*********************************************************************
* @fn      smoothFilter1();
*
* @brief   smoothfilter,average by point.
*    uint16_t
* @param   float value
* @return  .
*/
float smoothFilter1(float input)
{
	double smoothresp1 = 0;
	float  butterresp1 = 0; //float  smoothresp1 = 0,butterresp1 =0;
	uint8_t i, num = 5;
	butterresp1 = input;

	tempresp1[0] = butterresp1;
	for (i = 0; i < num; i++)
	{
		smoothresp1 += tempresp1[i];

	}
	butterresp1 = smoothresp1 / num;  //  smoothresp1 = smoothresp1/num; 

	for (i = (num - 1); i > 0; i--)
		tempresp1[i] = tempresp1[i - 1];
	return butterresp1;
}

/*********************************************************************
* @fn      smoothFilter2();
*
* @brief   smoothfilter,average by point.
*
* @param   float value
* @return  .
*/
float smoothFilter2(float input)
{
	double smoothresp2 = 0;
	float butterresp2 = 0;//float smoothresp2 = 0, butterresp2 =0;
	uint8_t i, num = 5;
	butterresp2 = input;
	tempresp2[0] = butterresp2;
	for (i = 0; i < num; i++)
	{
		smoothresp2 += tempresp2[i];

	}
	butterresp2 = smoothresp2 / num;  //smoothresp2 =smoothresp2/num;  
	for (i = (num - 1); i > 0; i--)
		tempresp2[i] = tempresp2[i - 1];
	return butterresp2;
}

float smoothFilter3(float input)
{
	double smoothresp3 = 0;
	float  butterresp3 = 0; //float  smoothresp1 = 0,butterresp1 =0;
	uint8_t i, num = 5;
	butterresp3 = input;

	tempresp3[0] = butterresp3;
	for (i = 0; i < num; i++)
	{
		smoothresp3 += tempresp3[i];

	}
	butterresp3 = smoothresp3 / num;  //  smoothresp1 = smoothresp1/num; 

	for (i = (num - 1); i > 0; i--)
		tempresp3[i] = tempresp3[i - 1];
	return butterresp3;
}


int smoothFilter4_int(int input)
{
	int smoothresp4 = 0;
	int  butterresp4 = 0; //float  smoothresp1 = 0,butterresp1 =0;
	uint8_t i, num = 5;
	butterresp4 = input;

	tempresp4[0] = butterresp4;
	for (i = 0; i < num; i++)
	{
		smoothresp4 += tempresp4[i];
	}
	butterresp4 = smoothresp4 / num;  //  smoothresp1 = smoothresp1/num; 

	for (i = (num - 1); i > 0; i--)
		tempresp4[i] = tempresp4[i - 1];
	return butterresp4;
}

