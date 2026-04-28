#ifndef __PERIPHERALS_H__
#define __PERIPHERALS_H__
#include "stddef.h"
#include "stdbool.h"

void peripherals_power_off();
void peripherals_power_off_without_power();
bool peripherals_check(void);
void PPG_board_power_off();
void PPG_board_power_on();

#endif