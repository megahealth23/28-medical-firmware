#ifndef FILTER_DATAPRE_H 
#define FILTER_DATAPRE_H  
  
#ifdef __cplusplus  
extern "C"  
{  
#endif  

#include <stdint.h>
#include <stdbool.h>

extern float afe_IIRLPFilter1(float input);
extern float smoothFilter1(float input);
extern float afe_IIRLPFilter2(float input);
extern float smoothFilter2(float input);
extern float afe_IIRLPFilter3(float input);
extern float smoothFilter3(float input);
extern float afe_IIRLPFilter4(float input);
extern float IIR_bandpass_Filter(float EEG);
extern float IIR_bandpass_chebyshev_Filter_heart(float EEG);
extern float IIR_bandpass_chebyshev_Filter_heart_sport(float EEG);

extern int smoothFilter4_int(int input);

#endif
