
#include <time.h>

#include "../new_common.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"
#include "../ota/ota.h"

#include "drv_deviceclock.h"

extern void CLOCK_Init_Events(void);
extern void CLOCK_RunEvents(unsigned int newTime, bool bTimeValid);

#if ENABLE_CLOCK_SUNRISE_SUNSET
extern void CLOCK_CalculateSunrise(byte *outHour, byte *outMinute);
extern void CLOCK_CalculateSunset(byte *outHour, byte *outMinute);
#endif

// leave it for the moment, until a CLOCK logging is possible ...
#define LOG_FEATURE LOG_FEATURE_NTP

#if ENABLE_CLOCK_SUNRISE_SUNSET

/* sunrise/sunset defaults */
#define CFG_DEFAULT_LATITUDE	43.994131
#define CFG_DEFAULT_LONGITUDE -123.095854
#define SUN_DATA_COORD_MULT		1000000

struct SUN_DATA sun_data =
	{
	.latitude = (int) (CFG_DEFAULT_LATITUDE * SUN_DATA_COORD_MULT),
	.longitude = (int) (CFG_DEFAULT_LONGITUDE * SUN_DATA_COORD_MULT),
	};

//Set Latitude and Longitude for sunrise/sunset calc
commandResult_t CLOCK_SetLatlong(const void *context, const char *cmd, const char *args, int cmdFlags) {
    const char *newValue;

    Tokenizer_TokenizeString(args,0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
    newValue = Tokenizer_GetArg(0);
    sun_data.latitude = (int) (atof(newValue) * SUN_DATA_COORD_MULT);
    addLogAdv(LOG_INFO, LOG_FEATURE_NTP, "NTP latitude set to %s", newValue);

    newValue = Tokenizer_GetArg(1);
		sun_data.longitude = (int) (atof(newValue) * SUN_DATA_COORD_MULT);
    addLogAdv(LOG_INFO, LOG_FEATURE_NTP, "NTP longitude set to %s", newValue);
    return CMD_RES_OK;
}

int CLOCK_GetSunrise()
{
	byte hour, minute;
	int sunriseInSecondsFromMidnight;

	CLOCK_CalculateSunrise(&hour, &minute);
	sunriseInSecondsFromMidnight = ((int)hour * 3600) + ((int)minute * 60);
	return sunriseInSecondsFromMidnight;
}

int CLOCK_GetSunset()
{
	byte hour, minute;
	int sunsetInSecondsFromMidnight;

	CLOCK_CalculateSunset(&hour, &minute);
	sunsetInSecondsFromMidnight =  ((int)hour * 3600) + ((int)minute * 60);
	return sunsetInSecondsFromMidnight;
}
#endif

int CLOCK_GetWeekDay() {
	struct tm *ltm;
	time_t act_deviceTime = (time_t)Clock_GetCurrentTime();

	// NOTE: on windows, you need _USE_32BIT_TIME_T 
	ltm = gmtime(&act_deviceTime);

	if (ltm == 0) {
		return 0;
	}

	return ltm->tm_wday;
}
int CLOCK_GetHour() {
	struct tm *ltm;
	time_t act_deviceTime = (time_t)Clock_GetCurrentTime();

	// NOTE: on windows, you need _USE_32BIT_TIME_T 
	ltm = gmtime(&act_deviceTime);

	if (ltm == 0) {
		return 0;
	}

	return ltm->tm_hour;
}
int CLOCK_GetMinute() {
	struct tm *ltm;
	time_t act_deviceTime = (time_t)Clock_GetCurrentTime();

	// NOTE: on windows, you need _USE_32BIT_TIME_T 
	ltm = gmtime(&act_deviceTime);

	if (ltm == 0) {
		return 0;
	}

	return ltm->tm_min;
}
int CLOCK_GetSecond() {
	struct tm *ltm;
	time_t act_deviceTime = (time_t)Clock_GetCurrentTime();

	// NOTE: on windows, you need _USE_32BIT_TIME_T 
	ltm = gmtime(&act_deviceTime);

	if (ltm == 0) {
		return 0;
	}

	return ltm->tm_sec;
}
int CLOCK_GetMDay() {
	struct tm *ltm;
	time_t act_deviceTime = (time_t)Clock_GetCurrentTime();

	// NOTE: on windows, you need _USE_32BIT_TIME_T 
	ltm = gmtime(&act_deviceTime);

	if (ltm == 0) {
		return 0;
	}

	return ltm->tm_mday;
}
int CLOCK_GetMonth() {
	struct tm *ltm;
	time_t act_deviceTime = (time_t)Clock_GetCurrentTime();

	// NOTE: on windows, you need _USE_32BIT_TIME_T 
	ltm = gmtime(&act_deviceTime);

	if (ltm == 0) {
		return 0;
	}

	return ltm->tm_mon+1;
}
int CLOCK_GetYear() {
	struct tm *ltm;
	time_t act_deviceTime = (time_t)Clock_GetCurrentTime();

	// NOTE: on windows, you need _USE_32BIT_TIME_T 
	ltm = gmtime(&act_deviceTime);

	if (ltm == 0) {
		return 0;
	}

	return ltm->tm_year+1900;
}



void CLOCK_setDeviceTime(uint32_t time)
{
	if (g_epochOnStartup < 10) CLOCK_Init();
	g_epochOnStartup = time - g_secondsElapsed;
}

void CLOCK_setDeviceTimeOffset(int offs)
{
	g_UTCoffset = offs;
}



void CLOCK_Init() {

#if ENABLE_CLOCK_SUNRISE_SUNSET
	//cmddetail:{"name":"ntp_setLatLong","args":"[Latlong]",
	//cmddetail:"descr":"Sets the NTP latitude and longitude",
	//cmddetail:"fn":"CLOCK_SetLatlong","file":"driver/drv_ntp.c","requires":"",
	//cmddetail:"examples":"CLOCK_SetLatlong -34.911498 138.809488"}
    CMD_RegisterCommand("clock_setLatLong", CLOCK_SetLatlong, NULL);
    
#pragma message ( "\n\n!!!!!!! C Preprocessor - ENABLE_CLOCK_SUNRISE_SUNSET is true!!!!! \n\n" )
#else
#pragma message ( "\n\n!!!!!!! C Preprocessor - ENABLE_CLOCK_SUNRISE_SUNSET is false!!!!! \n\n" )
#endif
#if ENABLE_CALENDAR_EVENTS
	CLOCK_Init_Events();
#endif


    addLogAdv(LOG_INFO, LOG_FEATURE_NTP, "CLOCK driver initialized.");
}

void CLOCK_OnEverySecond()
{

#if ENABLE_CALENDAR_EVENTS
	CLOCK_RunEvents(Clock_GetCurrentTime(), Clock_IsTimeSynced());
#endif
}


