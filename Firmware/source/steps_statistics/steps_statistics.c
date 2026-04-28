#include "steps_statistics.h"
#include "inner_flash_ram.h"

extern NIRAM_t NIramCfg;

void calculate_last_sec_steps(){
    uint32_t _now_steps = steps_num();
    if(NIramCfg.last_total_steps > _now_steps){
        NIramCfg.last_second_steps = _now_steps;
    }else{
        NIramCfg.last_second_steps = _now_steps - NIramCfg.last_total_steps;
    }

    NIramCfg.last_total_steps = _now_steps;
}

void check_for_reset_today_steps(uint32_t _now){
    UTCTimeStruct tm_start = {0};
    UTC_convertUTCTime_1(&tm_start, _now, NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);
    if(tm_start.hour == 23 && tm_start.minutes == 59){
        NIramCfg.step_in_day = 0;
    }
}

void set_steps_to_min_list(uint16_t steps){
    if(NIramCfg._min_list_index >= RECORD_STEPS_MINS){
        NIramCfg._min_list_index = 0;
    }

    NIramCfg.steps_per_min_list[NIramCfg._min_list_index] = steps;
    NIramCfg._min_list_index++;
    if(NIramCfg._min_list_index >= RECORD_STEPS_MINS)
        NIramCfg._min_list_index = 0;
}

uint32_t get_steps_from_min_list(){
    uint32_t steps = 0;
    uint8_t _index = RECORD_STEPS_MINS;
    while(_index){
        _index--;
        steps += NIramCfg.steps_per_min_list[_index];
    }

    return steps;
}

void update_steps_per_sec(uint32_t _now){
    calculate_last_sec_steps();

    NIramCfg.steps_count_by_module += NIramCfg.last_second_steps;

    NIramCfg.step_in_day += NIramCfg.last_second_steps;
    if(_now % 20 == 0){
        check_for_reset_today_steps(_now);
    }

    NIramCfg.step_in_min += NIramCfg.last_second_steps;
    if(_now % 60 == 0){
        set_steps_to_min_list(NIramCfg.step_in_min);
        NIramCfg.step_in_min = 0;
        NIramCfg.step_in_15_min = get_steps_from_min_list();
    }
}

uint32_t get_steps_in_day(){
    return NIramCfg.step_in_day;
}

uint16_t get_steps_in_15_min(){
    return NIramCfg.step_in_15_min;
}

uint16_t get_steps_in_min(){
    if(NIramCfg._min_list_index >= RECORD_STEPS_MINS){
        NIramCfg._min_list_index = 0;
    }

    uint8_t _index = 0;
    if(NIramCfg._min_list_index == 0){
        _index = RECORD_STEPS_MINS - 1;
    }else{
        _index = NIramCfg._min_list_index - 1;
    }

    return NIramCfg.steps_per_min_list[_index];
}

uint16_t get_steps_in_sec(){
    return NIramCfg.last_second_steps;
}

void reset_steps_count_by_module(){
    NIramCfg.steps_count_by_module = 0;
}

uint32_t get_steps_count_by_module(){
    return NIramCfg.steps_count_by_module;
}