/* Copyright (c) 2016 MegaHealth. All Rights Reserved.
 * mring_utc.h
 * AUTHOR:zhao mengshou
 * DATE:2016-11-9 11:40
 * http://www.megahealth.cn
 *
 */
 
#ifndef RING_UTC_H  
#define RING_UTC_H  

#include <stdint.h>  
#include "ring_swtimer.h" 


#ifdef __cplusplus  
extern "C"  
{  
#endif  
  

/********************************************************************* 
 * MACROS 
 */  
/********************************************************************* 
 * TYPEDEFS 
 */  
 // To be used with  
typedef struct  
{  
  uint8_t seconds;  // 0-59  
  uint8_t minutes;  // 0-59  
  uint8_t hour;     // 0-23  
  uint8_t day;      // 0-30  
  uint8_t month;    // 0-11  
  uint16_t year;    // 2000+  
} UTCTimeStruct;  
  
// number of seconds since 0 hrs, 0 minutes, 0 seconds, on the  
// 1st of January 2000 UTC  
typedef uint32_t UTCTime;


void UTC_clockUpdate(void);

/* 
 * @fn      UTC_init
 *
 * @brief   Initialize the UTC clock module.  Sets up and starts the
 *          clock instance.
 *
 * @param   None.
 *
 * @return  None.
 */
extern void UTC_init(void);

/*
 * Set the new time.  This will only set the seconds portion
 * of time and doesn't change the factional second counter.
 *     newTime - number of seconds since 0 hrs, 0 minutes,
 *               0 seconds, on the 1st of January 2000 UTC
 */
extern void UTC_setClock( UTCTime newTime );

/*
 * Gets the current time.  This will only return the seconds
 * portion of time and doesn't include the factional second counter.
 *     returns: number of seconds since 0 hrs, 0 minutes,
 *              0 seconds, on the 1st of January 2000 UTC
 */
extern UTCTime UTC_getClock( void );

/*
 * Converts UTCTime to UTCTimeStruct
 *
 * secTime - number of seconds since 0 hrs, 0 minutes,
 *          0 seconds, on the 1st of January 2000 UTC
 * tm - pointer to breakdown struct
 */
extern void UTC_convertUTCTime( UTCTimeStruct *tm, UTCTime secTime );

extern void UTC_convertUTCTime_1(UTCTimeStruct *tm, UTCTime secTime, uint8_t zone_hour, uint8_t zone_minute);

/*
 * Converts UTCTimeStruct to UTCTime (seconds since 00:00:00 01/01/2000)
 *
 * tm - pointer to UTC time struct
 */
extern UTCTime UTC_convertUTCSecs( UTCTimeStruct *tm );

#ifdef __cplusplus  
}  
#endif  
#endif /* MRING_UTC_H */

