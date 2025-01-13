#include "../obk_config.h"
#if ENABLE_DRIVER_NEO6M
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "../logging/logging.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "drv_uart.h"

#include "../httpserver/new_http.h"

static unsigned short NEO6M_baudRate = 9600;

#define NEO6M_UART_RECEIVE_BUFFER_SIZE 512


static int H,M,S,SS,DD,MM,YY;
static float Lat_f,Long_f;
static char NS,EW;
static bool gpslocked=false;

enum {
NMEA_TIME,
NMEA_LOCK,
NMEA_LAT,
NMEA_LAT_DIR,
NMEA_LONG,
NMEA_LONG_DIR,
NMEA_SPEED,
NMEA_COURSE,
NMEA_DATE,
NMEA_MAGVAR,
NMEA_MAGVAR_DIR,
NMEA_MODE,
// parse will stop at *, so not needed during parsing
//NMEA_STAR,
//NMEA_CHECKSUM,
NMEA_WORDS
};

#define LEAP_YEAR(Y)  ((!(Y%4) && (Y%100)) || !(Y%400))
static const uint8_t DaysMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
const uint32_t SECS_PER_MIN = 60UL;
const uint32_t SECS_PER_HOUR = 3600UL;
const uint32_t SECS_PER_DAY = 3600UL * 24UL;
const uint32_t MINS_PER_HOUR = 60UL;

//simple "mktime" replacement to calculate epoch from a date
// note: as mktime, month are 0-11 (not 1 - 12) 
uint32_t obkmktime(int yr, uint8_t month, uint8_t day, uint8_t hr, uint8_t min, uint8_t sec) {
  // to avoid mktime - this enlarges the image especially for BL602!
  // so calculate seconds from epoch locally
  // we start by calculating the number of days since 1970
  int i;
  uint16_t days;
  uint32_t t;
  days = (yr - 1970) * 365;			// days per full years
  // add one day every leap year - first leap after 1970 is 1972, possible leap years every 4 years
  for (i=1972; i < yr; i+=4) days += LEAP_YEAR(i);
  for (i=0; i<month; i++){
  	if (i==1 && LEAP_YEAR(yr)){
  		days += 29 ;
  	} else {
  		days += DaysMonth[i];
  	}
  }
  days += day-1;
 // calc seconds from days and add hours
 t = days * SECS_PER_DAY + hr * SECS_PER_HOUR + min * SECS_PER_MIN + sec ;
 return t;
}



void parseGPS(char *data){
	if (! data) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "parseGPS -- no data ! ");
		return;
	}
	char *p = strstr(data, "$GPRMC,");
	if (p){
//ADDLOG_INFO(LOG_FEATURE_DRV, "parseGPS  p is not NULL -- data=%s  ## p=%s  .... ",data,p);

	// Data is like "$GPRMC,142953.00,A,5213.00212,N,02101.98018,E,0.105,,200724,,,A*71"
	// $GPRMC,HHMMSS[.SSS],A,BBBB.BBBB,b,LLLLL.LLLL,l,GG.G,RR.R,DDMMYY,M.M,m,F*PP
	//
		// calculate NMEA checksum
		int NMEACS=103;		// Start with checksum value for "GPRMC," - we don't need to parse it later and strsts makes sure, we have it in string
		int word=0,letter=0;
		char Nvalue[NMEA_WORDS][12]={0};
		gpslocked=0;
		p+=7 ; // so we are after the "," of $GPRMC, and can start calculation and assignment
		//ADDLOG_INFO(LOG_FEATURE_DRV, "calculating CS with p=%s \n",p);
		while (p && p[0] != '*') { 
			NMEACS ^= p[0];
			if (p[0] != ','){
				if (letter < 11 && word < NMEA_WORDS) Nvalue[word][letter++]=p[0];
				else  {
					ADDLOG_INFO(LOG_FEATURE_DRV, "parseGPS: OOps, word/letter error!! letter=%i # word=%i  (data=%s)!!!",letter,word,data);
					return;
				}	
			}
			else {
				Nvalue[word][letter]=0;
				word++;
				letter=0;
			}
			p++;
		}
		// checksum in parsed data
		int readcs=0; 
		if (++p) {
			readcs=(int)strtol(p, NULL, 16);
		} else  ADDLOG_INFO(LOG_FEATURE_DRV, "parseGPS: OOps, no checksum in data!!");

		if (NMEACS != readcs) {        
			ADDLOG_WARN(LOG_FEATURE_DRV, "parseGPS:  calculated CS (%2X) !=  read CS (%2X) ! Data might be invalid!\r\n",NMEACS,readcs); 		
		};
/*		
		float tmpfl=0;
		int tmpi=0;
		
		tmpfl=atof(Nvalue[NMEA_TIME]);
		SS=(int)(tempfl*100)-(int)tempfl*100;
		H=tempfl/10000;
		M=tempfl/100 - H*100;
		S=(int)tempfl%100;		
		
		tmpi=atoi(Nvalue[NMEA_DATE]);
		DD=tmpi/10000;
		MM=tmpi/100 - DD*100;
		YY=2000+tmpi%100;

*/		
		
		
		bool timeread =0, dateread=0;
		if (Nvalue[NMEA_TIME]) timeread = (sscanf (Nvalue[NMEA_TIME],"%2d%2d%2d.%d",&H,&M,&S,&SS) >=3);
		if (Nvalue[NMEA_DATE]) dateread = (sscanf (Nvalue[NMEA_DATE],"%2d%2d%2d",&DD,&MM,&YY) ==3);
		YY+=2000;

		gpslocked=(Nvalue[NMEA_LOCK][0]=='A');
		if (!gpslocked) {        
			ADDLOG_WARN(LOG_FEATURE_DRV, "parseGPS: no GPS lock! %s\r\n", timeread && dateread ? "Date/time might be valid." :""); 
			
		};

		if (Nvalue[NMEA_LAT_DIR][0]) NS=Nvalue[NMEA_LAT_DIR][0];

		// NMEA returns degrees as degree and "minutes" of latitude/longitude 
		// we want it as a decimal representation 
		// e.g. from input value
		// 5213.00212 	= 52.216702
		// ddmm.mmmmm
		// we would need the minutes 13.00212 to be converted
		// the result is value / 60
		// here: 1.03272 / 60 = 0.216702

		float frac;
		int whole=0;						// need to get tw digits as %2d, using "%2f" will take the whole value as float?!?
//		sscanf (Nvalue[NMEA_LAT],"%2f%f",&Lat_f,&frac);			// Lat_f now contains whole part of degrees
//		sscanf (Nvalue[NMEA_LAT],"%2d%f",&whole,&frac);			// whole now contains whole part of degrees
//		sscanf (Nvalue[NMEA_LAT],"%f",&frac);			// frac now contains the complete float (degrees + minutes)
		if (Nvalue[NMEA_LAT]) {
			Lat_f=atof(Nvalue[NMEA_LAT]);
			whole=(int)(Lat_f/100);
			Lat_f=(float)whole + (Lat_f - 100*whole)/60;
		}


		if (Nvalue[NMEA_LONG_DIR][0]) EW=Nvalue[NMEA_LONG_DIR][0];
//		sscanf (Nvalue[NMEA_LONG],"%3f%f",&Long_f,&frac);			// Long_f now contains whole part of degrees
//		sscanf (Nvalue[NMEA_LONG],"%3d%f",&whole,&frac);			// whole now contains whole part of degrees
//		sscanf (Nvalue[NMEA_LONG],"%f",&frac);					// frac now contains the complete float (degrees + minutes)


		if (Nvalue[NMEA_LONG]){
			Long_f = atof(Nvalue[NMEA_LONG]);
			whole = (int)(Long_f/100);
			Long_f=(float)whole + (Long_f - 100*whole)/60;
		}
		
		ADDLOG_INFO(LOG_FEATURE_DRV, 
			    "Read GPS DATA:%02i.%02i.%i - %02i:%02i:%02i.%02i (epoch=%u) LAT=%f%c - LONG=%f%c  \r\n", DD,MM,YY,H,M,S,SS,obkmktime(YY,MM-1,DD,H,M,S),Lat_f,NS,Long_f,EW);

	} else ADDLOG_INFO(LOG_FEATURE_DRV, "parseGPS:  p is  NULL -- data=%s  .... ",data);
//ADDLOG_INFO(LOG_FEATURE_DRV, "... end of parseGPS ");

}


static int UART_GetNextPacket(void) {
//ADDLOG_INFO(LOG_FEATURE_DRV, "UART_GetNextPacket - start \r\n");
	int cs;
	char data[NEO6M_UART_RECEIVE_BUFFER_SIZE+5]={0};
	cs = UART_GetDataSize();
//ADDLOG_INFO(LOG_FEATURE_DRV, "UART_GetNextPacket - cs=%i \r\n",cs);

	int i=0;
	for (i=0; i< cs; i++){
		data[i]=UART_GetByte(i);
	}
	data[i]=0;
//ADDLOG_INFO(LOG_FEATURE_DRV, "UART_GetNextPacket  i=%i  - data=%s\r\n",i,data);
	UART_ConsumeBytes(cs-1);
	parseGPS(data);
	return 0;
}



// send "$EIGPQ,RMC*3A\r\n" to request/poll DATA
static void UART_WritePollReq(void) {
    uint8_t send[]="$EIGPQ,RMC*3A\r\n$EIGPQ,RMC*3A\r\n";

    for (int i = 0; i < sizeof(send); i++) {
        UART_SendByte(send[i]);
    }
}


/*
 # Disabling all NMEA sentences
$PUBX,40,GGA,0,0,0,0*5A   // Disable GGA
$PUBX,40,GLL,0,0,0,0*5C   // Disable GLL
$PUBX,40,GSA,0,0,0,0*4E   // Disable GSA
$PUBX,40,GSV,0,0,0,0*59   // Disable GSV
$PUBX,40,RMC,0,0,0,0*47   // Disable RMC
$PUBX,40,VTG,0,0,0,0*5E   // Disable VTG
$PUBX,40,ZDA,0,0,0,0*44   // Disable ZDA
*/
static void UART_WriteDisableNMEA(void) {
    char send[8][26]={
    	"$PUBX,40,GGA,0,0,0,0*5A\r\n",
    	"$PUBX,40,GGA,0,0,0,0*5A\r\n",
//    	"$PUBX,40,GGA,0,1,0,0*5B\r\n",   // Enable GGA
    	"$PUBX,40,GLL,0,0,0,0*5C\r\n",
    	"$PUBX,40,GSA,0,0,0,0*4E\r\n",
    	"$PUBX,40,GSV,0,0,0,0*59\r\n",
//    	"$PUBX,40,RMC,0,0,0,0*47\r\n",   // Disable RMC
    	"$PUBX,40,RMC,0,1,0,0*46\r\n",   // Enable RMC
    	"$PUBX,40,VTG,0,0,0,0*5E\r\n",
    	"$PUBX,40,ZDA,0,0,0,0*44\r\n"
    	};
    byte b;
    for (int i = 0; i < 7; i++) {
	    for (int j = 0; j < sizeof(send[i]); j++) {
	    	b = (byte)send[i][j];
        	UART_SendByte(b);
            }
    }
}

static void Init(void) {

}

// THIS IS called by 'startDriver NEO6M' command
// You can set alternate baud with 'startDriver NEO6M <rate>' syntax
void NEO6M_UART_Init(void) {
	Init();

	NEO6M_baudRate = Tokenizer_GetArgIntegerDefault(1, 9600);
	UART_InitUART(NEO6M_baudRate, 0);
	UART_InitReceiveRingBuffer(NEO6M_UART_RECEIVE_BUFFER_SIZE);
	UART_WriteDisableNMEA();
	

}



void NEO6M_requestData(void) {
//ADDLOG_INFO(LOG_FEATURE_DRV, "NEO6M_requestData \r\n");

//    UART_InitUART(NEO6M_baudRate, 0);
    UART_GetNextPacket();
//    UART_InitUART(NEO6M_baudRate, 0);

}



void NEO6M_UART_RunEverySecond(void) {

/*
    UART_TryToGetNextPacket();

    UART_InitUART(NEO6M_baudRate, 0);
*/

	int cs= UART_GetDataSize();

/*ADDLOG_INFO(LOG_FEATURE_DRV, 
			    "NEO6M_UART_RunEverySecond: UART_GetDataSize() = %i  \r\n",cs);

*/
	if (g_secondsElapsed % 5 == 0) {	// every 5 seconds
			NEO6M_requestData();
	}
	else { 
		if (cs > 1){
/*ADDLOG_INFO(LOG_FEATURE_DRV, "EO6M_UART_RunEverySecond: UART_ConsumeBytes(%i); \r\n",cs -1);
			UART_ConsumeBytes(cs -1);
*/
		}
	}
/*	if (g_secondsElapsed % 5 == 4) {	// every 5 seconds
		cs=UART_GetDataSize();
		if (cs > 1) UART_ConsumeBytes(cs -1); // empty buffer before polling
ADDLOG_INFO(LOG_FEATURE_DRV, "EO6M_UART_RunEverySecond: calling UART_WritePollReq \r\n");
//		UART_WritePollReq();
	}
*/

}


void NEO6M_AppendInformationToHTTPIndexPage(http_request_t *request)
{
	if (gpslocked) hprintf255(request, "<h5>GPS: %i-%02i-%02iT%02i:%02i:%02i Lat: %f%c Long: %f%c </h5>", YY,MM,DD,H,M,S,Lat_f,NS,Long_f,EW);
}
#endif // ENABLE_DRIVER_NEO6M
