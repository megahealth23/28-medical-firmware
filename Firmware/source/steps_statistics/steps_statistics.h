#pragma once

#include "stdint.h"
#include "nrf_log.h"
#include "gesture/mring_gesture.h"
#include "system_clock.h"
#include "ring_utc.h"

void update_steps_per_sec(uint32_t now);

uint32_t get_steps_in_day();
uint16_t get_steps_in_15_min();
uint16_t get_steps_in_sec();
uint16_t get_steps_in_min();

void reset_steps_count_by_module();
uint32_t get_steps_count_by_module();