#ifndef RING_SWTIMER_H
#define RING_SWTIMER_H

#include "app_timer.h"
#include "nrf_log.h"


#include "ring_bsp.h"

#define APP_TIMER_PRESCALER		0

#define BLE_PAIR_INTERVAL		APP_TIMER_TICKS(1*1000 )	/**< ble pair (ticks). */

#define BLE_NOTI_INTERVAL		APP_TIMER_TICKS(5)	/**< ble notify interval (ticks). */
#define BLE_INDI_INTERVAL		APP_TIMER_TICKS(10)	/**< ble indicate (ticks). */


typedef void (*swtimerCallBack_t)(uint32_t timers);

bool startPairTimer(void);
bool stopPairTimer(void);

void PairTimeout_handler(void * p_context);


/************function decration*************/
bool ring_swtimer_init(void);
void registerOneSecCallBack(swtimerCallBack_t onesCallBack);
void one_sec_report(void);

bool mring_startBLEIndicateTimer(void);
bool mring_startBLENotifyTimer(void);

bool startNotifyWaitTimer(int w);
bool startIndiTimer(int w);

bool startReportTimer(int ms);
bool stopReportTimer();

#endif

