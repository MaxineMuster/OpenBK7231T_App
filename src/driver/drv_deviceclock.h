#ifndef __DRV_DEVICECLOCK_H__
#define __DRV_DEVICECLOCK_H__

#include "../httpserver/new_http.h"

int CLOCK_GetWeekDay();
int CLOCK_GetHour();
int CLOCK_GetMinute();
int CLOCK_GetSecond();
int CLOCK_GetMDay();
int CLOCK_GetMonth();
int CLOCK_GetYear();
int CLOCK_GetSunrise();
int CLOCK_GetSunset();
// drv_timed_events.c
int CLOCK_Print_EventList();
void CLOCK_setDeviceTime(uint32_t time);
void CLOCK_setDeviceTimeOffset(int offs);
int CLOCK_GetEventTime(int id);
int CLOCK_RemoveEvent(int id);
int CLOCK_ClearEvents();
void CLOCK_Init();
void CLOCK_OnEverySecond();
extern struct SUN_DATA {  /* sunrise / sunset globals */
	int latitude;  /* latitude * 1000000 */
	int longitude;  /* longitude * 1000000 */
	} sun_data;

#endif /* __DRV_DEVICECLOCK_H__ */

