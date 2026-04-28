#include "ring_utc.h"  
#include "app_util_platform.h"
#include "nrf_drv_rtc.h"
#include "nrf_delay.h"
#include "mTask.h"

/*********************************************************************
* MACROS
*/

#define	YearLength(yr)	(IsLeapYear(yr) ? 366 : 365)

/*********************************************************************
* CONSTANTS
*/

#define MAX_RTC_TASKS_DELAY	47		/**< Maximum delay until an RTC task is executed. 52832 */

const  nrf_drv_rtc_t		rtc2 = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
#define RTC2_INTERVAL		1	//uints : sec
#define RTC2_CC_VALUE		(RTC2_INTERVAL * RTC2_CONFIG_FREQUENCY)	
//Determines the RTC interrupt frequency and thereby the utc sampling frequency

// Update every 1000ms
#define UTC_UPDATE_PERIOD  1000

#define IsLeapYear(yr)     (!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))

#define	BEGYEAR	           1970     // UTC started at 00:00:00 January 1, 2000

#define	DAY                86400UL  // 24 hours * 60 minutes * 60 seconds

// Time is the number of seconds since 0 hrs, 0 minutes, 0 seconds, on the
// 1st of January 2000 UTC.
static volatile UTCTime UTC_timeSeconds = 0;
static uint8_t UTC_status = 0;

// Time is the number of system start milliseconds.
static volatile uint32_t runMilliSec = 0;
// Time is the number of system start seconds.
static volatile uint32_t runSec = 0;

//onesec callback
static swtimerCallBack_t onesec_callBack = NULL;
/*********************************************************************
* LOCAL FUNCTION PROTOTYPES
*/
static uint8_t UTC_monthLength(uint8_t lpyr, uint8_t mon);


/*********************************************************************
* @fn      UTC_init
*
* @brief   Initialize the UTC clock module.  Sets up and starts the
*          clock instance.
*
* @param   None.
*
* @return  None.
*/
void UTC_init(void)
{
	if(0 == UTC_status)
	{
		UTCTimeStruct utc;    
        utc.seconds = 0x00;
        utc.minutes = 0x00;
        utc.hour    = 0x00;
        utc.day     = 0x01;
        utc.month   = 0x01;
        utc.year    = 2000;
        UTC_setClock( UTC_convertUTCSecs( &utc ) );
		UTC_status = 1;
	}
	
}

/*********************************************************************
* @fn      UTC_clockUpdate
*
* @brief   Updates the UTC Clock time with elapsed milliseconds.
*
* @param   elapsedMSec - elapsed milliseconds
*
* @return  none
*/
void UTC_clockUpdate()
{
	UTC_timeSeconds++;
}

/*********************************************************************
* @fn      UTC_setClock
*
* @brief   Set a new time.  This will only set the seconds portion
*          of time and doesn't change the factional second counter.
*
* @param   newTime - Number of seconds since 0 hrs, 0 minutes,
*                    0 seconds, on the 1st of January 2000 UTC.
*
* @return  none
*/
void UTC_setClock(UTCTime newTime)
{
	UTC_timeSeconds = newTime;
}

/*********************************************************************
* @fn      UTC_getClock
*
* @brief   Gets the current time.  This will only return the seconds
*          portion of time and doesn't include the factional second
*          counter.
*
* @param   none
*
* @return  number of seconds since 0 hrs, 0 minutes, 0 seconds,
*          on the 1st of January 2000 UTC
*/
UTCTime UTC_getClock(void)
{
	return (UTC_timeSeconds);
}

/*********************************************************************
* @fn      UTC_convertUTCTime
*
* @brief   Converts UTCTime to UTCTimeStruct (from total seconds to exact
*          date).
*
* @param   tm - pointer to breakdown struct.
*
* @param   secTime - number of seconds since 0 hrs, 0 minutes,
*          0 seconds, on the 1st of January 2000 UTC.
*
* @return  none
*/
void UTC_convertUTCTime(UTCTimeStruct *tm, UTCTime secTime)
{
	secTime += 480*60;
	// Calculate the time less than a day - hours, minutes, seconds.
	{
		// The number of seconds that have occured so far stoday.
		uint32_t day = secTime % DAY;

		// Seconds that have passed in the current minute.
		tm->seconds = day % 60UL;
		// Minutes that have passed in the current hour.
		// (seconds per day) / (seconds per minute) = (minutes on an hour boundary)
		tm->minutes = (day % 3600UL) / 60UL;
		// Hours that have passed in the current day.
		tm->hour = day / 3600UL;
	}

	// Fill in the calendar - day, month, year
	{
		uint16_t numDays = secTime / DAY;
		uint8_t monthLen;
		tm->year = BEGYEAR;

		while (numDays >= YearLength(tm->year))
		{
			numDays -= YearLength(tm->year);
			tm->year++;
		}

		// January.
		tm->month = 0;

		monthLen = UTC_monthLength(IsLeapYear(tm->year), tm->month);

		// Determine the number of months which have passed from remaining days.
		while (numDays >= monthLen)
		{
			// Subtract number of days in month from remaining count of days.
			numDays -= monthLen;
			tm->month++;

			// Recalculate month length.
			monthLen = UTC_monthLength(IsLeapYear(tm->year), tm->month);
		}

		// Store the remaining days.
		tm->day = numDays;
	}

	// ADD: day and month from 1 counter
	{
		tm->day++;
		tm->month++;
	}
}

void UTC_convertUTCTime_1(UTCTimeStruct *tm, UTCTime secTime, uint8_t zone_hour, uint8_t zone_minute)
{
    int zone_offset = 1;
    if(zone_hour & 0x80){
        zone_offset = -1;
        zone_hour &= 0x7f;
    }

    zone_offset = zone_offset*(zone_hour*60+zone_minute)*60;
	//secTime += 480*60;
	secTime += zone_offset;
	// Calculate the time less than a day - hours, minutes, seconds.
	{
		// The number of seconds that have occured so far stoday.
		uint32_t day = secTime % DAY;

		// Seconds that have passed in the current minute.
		tm->seconds = day % 60UL;
		// Minutes that have passed in the current hour.
		// (seconds per day) / (seconds per minute) = (minutes on an hour boundary)
		tm->minutes = (day % 3600UL) / 60UL;
		// Hours that have passed in the current day.
		tm->hour = day / 3600UL;
	}

	// Fill in the calendar - day, month, year
	{
		uint16_t numDays = secTime / DAY;
		uint8_t monthLen;
		tm->year = BEGYEAR;

		while (numDays >= YearLength(tm->year))
		{
			numDays -= YearLength(tm->year);
			tm->year++;
		}

		// January.
		tm->month = 0;

		monthLen = UTC_monthLength(IsLeapYear(tm->year), tm->month);

		// Determine the number of months which have passed from remaining days.
		while (numDays >= monthLen)
		{
			// Subtract number of days in month from remaining count of days.
			numDays -= monthLen;
			tm->month++;

			// Recalculate month length.
			monthLen = UTC_monthLength(IsLeapYear(tm->year), tm->month);
		}

		// Store the remaining days.
		tm->day = numDays;
	}

	// ADD: day and month from 1 counter
	{
		tm->day++;
		tm->month++;
	}
}

/*********************************************************************
* @fn      UTC_monthLength
*
* @param   lpyr - 1 for leap year, 0 if not
*
* @param   mon - 0 - 11 (jan - dec)
*
* @return  number of days in specified month
*/
static uint8_t UTC_monthLength(uint8_t lpyr, uint8_t mon)
{
	uint8_t days = 31;

	if (mon == 1) // feb
	{
		days = (28 + lpyr);
	}
	else
	{
		if (mon > 6) // aug-dec
		{
			mon--;
		}

		if (mon & 1)
		{
			days = 30;
		}
	}

	return (days);
}

/*********************************************************************
* @fn      UTC_convertUTCSecs
*
* @brief   Converts a UTCTimeStruct to UTCTime (from exact date to total
*          seconds).
*
* @param   tm - pointer to provided struct.
*
* @return  number of seconds since (UTC).
*/
UTCTime UTC_convertUTCSecs(UTCTimeStruct *tm)
{
	uint32_t seconds;

	// Seconds for the partial day.
	seconds = (((tm->hour * 60UL) + tm->minutes) * 60UL) + tm->seconds;

	// Account for previous complete days.
	{
		// Start with complete days in current month.
		uint16_t days = tm->day - 1;		// NOTE: from 1 counter

		// Next, complete months in current year.
		{
			int8_t month = tm->month-1;		// NOTE: from 1 counter
			while (--month >= 0)
			{
				days += UTC_monthLength(IsLeapYear(tm->year), month);
			}
		}

		// Next, complete years before current year.
		{
			uint16_t year = tm->year;
			while (--year >= BEGYEAR)
			{
				days += YearLength(year);
			}
		}

		// Add total seconds before partial day.
		seconds += (days * DAY);
	}

	return (seconds);
}

UTCTime UTC_convertUTCSecs_1(UTCTimeStruct *tm, uint8_t zone_hour, uint8_t zone_minute)
{
	uint32_t seconds;

	// Seconds for the partial day.
	seconds = (((tm->hour * 60UL) + tm->minutes) * 60UL) + tm->seconds;

	// Account for previous complete days.
	{
		// Start with complete days in current month.
		uint16_t days = tm->day - 1;		// NOTE: from 1 counter

		// Next, complete months in current year.
		{
			int8_t month = tm->month-1;		// NOTE: from 1 counter
			while (--month >= 0)
			{
				days += UTC_monthLength(IsLeapYear(tm->year), month);
			}
		}

		// Next, complete years before current year.
		{
			uint16_t year = tm->year;
			while (--year >= BEGYEAR)
			{
				days += YearLength(year);
			}
		}

		// Add total seconds before partial day.
		seconds += (days * DAY);
	}

    int zone_offset = 1;
    if(zone_hour & 0x80){
        zone_offset = -1;
        zone_hour &= 0x7f;
    }

    zone_offset = zone_offset*(zone_hour*60+zone_minute)*60;
	//secTime += 480*60;
	seconds += (-1*zone_offset);

	return (seconds);
}