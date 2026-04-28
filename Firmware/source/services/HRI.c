#include "HRI.h"
#include "nrf_delay.h"

static uint8_t current_status = CURRENT_STATUS_NONE;

void ring_topled_on(void)
{
    //advertising_stop();
    nrf_delay_ms(2);
	ring_handonled_init(false, HANDONLED_DUTY_DETECTION);
	ring_handonled_on();
}

void ring_topled_off(void)
{
	ring_handonled_off();
    nrf_delay_ms(2);
    //advertising_start(false);
}

//void ring_charging_indicate(void)
//{
//	if( CURRENT_STATUS_CHARGING == current_status)
//		return;
//	else
//	{
//		current_status = CURRENT_STATUS_CHARGING;
//		ring_handonled_init(false, HANDONLED_DUTY_DETECTION);
//		ring_handonled_flashing_start(false, HANDONLED_PERIOD_2500MS);
//	}
//}

//void ring_charging_led_on(void)
//{
//	ring_handonled_init(false, HANDONLED_DUTY_INDICATE);
//	ring_handonled_on();
//}

//void ring_charging_led_off(void)
//{
//	ring_handonled_off();
//}

//void ring_chargefull_indicate(void)
//{
//	if( CURRENT_STATUS_CHARGEFULL == current_status)
//		return;
//	else
//	{
//		current_status = CURRENT_STATUS_CHARGEFULL;
//		ring_handonled_init(false, HANDONLED_DUTY_INDICATE);
//		ring_handonled_on();
//	}
//}

//void ring_findme_indicate(void)
//{
//	ring_handonled_init(false, HANDONLED_DUTY_INDICATE);
//	ring_handonled_flashing_start(true, HANDONLED_PERIOD_500MS);
//}

//void ring_pair_indicate(void)
//{
//	ring_handonled_init(false, HANDONLED_DUTY_INDICATE);
//	ring_handonled_flashing_start(true, HANDONLED_PERIOD_1000MS);
//}

//void ring_stop_indicate(void)
//{
//	if( CURRENT_STATUS_NORMAL == current_status)
//		return;
//	else
//	{
//		current_status = CURRENT_STATUS_NORMAL;
//		ring_handonled_off();
//	}
//}



