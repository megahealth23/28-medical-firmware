#ifndef _DATAPROCPTT_H_
#define _DATAPROCPTT_H_

#include <stdint.h>

extern int ptt_offlineflg;

extern uint8_t PTT_bufProc_wavelet(void *afe_in, uint8_t frame_length);
#endif