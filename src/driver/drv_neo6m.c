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

#include "../libraries/obktime/obktime.h"


static unsigned short NEO6M_baudRate = 9600;

#define NEO6M_UART_RECEIVE_BUFFER_SIZE 512


static int H,M,S,SS,DD,MM,YY;
static float Lat_f,Long_f;
static char NS,EW;
static bool gpslocked=false;
static char tempstr[50]; 
static char fakelat[12]={0};
static char fakelong[12]={0};

#if ENABLE_LOCAL_CLOCK || ENABLE_NTP
#include "drv_deviceclock.h"
static bool setclock2gps=false;
#if ENABLE_CLOCK_SUNRISE_SUNSET
static bool setlatlong2gps=false;
#endif
#endif

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


void parseGPS(char *data){
	if (! data) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "parseGPS -- no data ! ");
		return;
	}
	char *p = strstr(data, "$GPRMC,");
	if (p){
	while (p) {
ADDLOG_DEBUG(LOG_FEATURE_DRV, "parseGPS  p is not NULL -- data=%s  ## p=%s  .... ",data,p);
	tempstr[0]='\0';

	// Data is like "$GPRMC,142953.00,A,5213.00212,N,02101.98018,E,0.105,,200724,,,A*71"
	// $GPRMC,HHMMSS[.SSS],A,BBBB.BBBB,b,LLLLL.LLLL,l,GG.G,RR.R,DDMMYY,M.M,m,F*PP
	//
		// calculate NMEA checksum
		int NMEACS=103;		// Start with checksum value for "GPRMC," - we don't need to parse it later and strsts makes sure, we have it in string
		int word=0,char_index=0;
		char Nvalue[NMEA_WORDS][12]={0};
		gpslocked=0;
		p+=7 ; // so we are after the "," of $GPRMC, and can start calculation and assignment
		//ADDLOG_INFO(LOG_FEATURE_DRV, "calculating CS with p=%s \n",p);
		while (p && p[0] != '*') { 
			NMEACS ^= p[0];
			if (p[0] != ','){
				if (char_index < 11 && word < NMEA_WORDS) Nvalue[word][char_index++]=p[0];
				else  {
					ADDLOG_INFO(LOG_FEATURE_DRV, "parseGPS: OOps, word/char_index error!! char_index=%i # word=%i  (data=%s)!!!",char_index,word,data);
					return;
				}	
			}
			else {
				Nvalue[word][char_index]=0;
				word++;
				char_index=0;
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
		timeread = (sscanf (Nvalue[NMEA_TIME],"%2d%2d%2d.%d",&H,&M,&S,&SS) >=3);
		dateread = (sscanf (Nvalue[NMEA_DATE],"%2d%2d%2d",&DD,&MM,&YY) ==3);
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
		// here: 13.00212 / 60 = 0.216702

		float frac;
		int whole=0;						// need to get tw digits as %2d, using "%2f" will take the whole value as float?!?
//		sscanf (Nvalue[NMEA_LAT],"%2f%f",&Lat_f,&frac);			// Lat_f now contains whole part of degrees
//		sscanf (Nvalue[NMEA_LAT],"%2d%f",&whole,&frac);			// whole now contains whole part of degrees
//		sscanf (Nvalue[NMEA_LAT],"%f",&frac);			// frac now contains the complete float (degrees + minutes)

		ADDLOG_DEBUG(LOG_FEATURE_DRV,"NEO6M: fakelat=%s  Nvalue[NMEA_LAT]=%s ",fakelat,Nvalue[NMEA_LAT]);		

		if (*fakelat && strlen(fakelat) <= strlen(Nvalue[NMEA_LAT])) memcpy(Nvalue[NMEA_LAT],fakelat,strlen(fakelat));
		Lat_f=atof(Nvalue[NMEA_LAT]);
		whole=(int)(Lat_f/100);
		Lat_f=(float)whole + (Lat_f - 100*whole)/60;



		if (Nvalue[NMEA_LONG_DIR][0]) EW=Nvalue[NMEA_LONG_DIR][0];
//		sscanf (Nvalue[NMEA_LONG],"%3f%f",&Long_f,&frac);			// Long_f now contains whole part of degrees
//		sscanf (Nvalue[NMEA_LONG],"%3d%f",&whole,&frac);			// whole now contains whole part of degrees
//		sscanf (Nvalue[NMEA_LONG],"%f",&frac);					// frac now contains the complete float (degrees + minutes)

		ADDLOG_DEBUG(LOG_FEATURE_DRV,"NEO6M: fakelong=%s  Nvalue[NMEA_LONG]=%s ",fakelong,Nvalue[NMEA_LONG]);		

		if (*fakelong && strlen(fakelong) <= strlen(Nvalue[NMEA_LONG])) { 
			memcpy(Nvalue[NMEA_LONG],fakelong,strlen(fakelong));		// note: we will not copy "\0" but ony overwrite (some) numbers...		 
		}
		Long_f = atof(Nvalue[NMEA_LONG]);
		whole = (int)(Long_f/100);
		Long_f=(float)whole + (Long_f - 100*whole)/60;

		uint32_t epoch_time=dateToEpoch(YY,MM,DD,H,M,S);
#if ENABLE_LOCAL_CLOCK || ENABLE_NTP
		if (setclock2gps && gpslocked && timeread && dateread){
			CLOCK_setDeviceTime(epoch_time);
//			ADDLOG_INFO(LOG_FEATURE_DRV,"local clock set to UTC time read");
			strcat(tempstr, "(clock ");
		}
#endif
#if ENABLE_CLOCK_SUNRISE_SUNSET
		if( setlatlong2gps && gpslocked){
			CLOCK_setLatitude(Lat_f);
			CLOCK_setLongitude(Long_f);
//			ADDLOG_INFO(LOG_FEATURE_DRV,"latitude set to %f, longitude set to %f",Lat_f, Long_f);
			strcat(tempstr,tempstr[0]? "and lat/long " : "(lat/log ");
		}
#endif
		if (tempstr[0]) strcat(tempstr,"set to GPS data)");
		ADDLOG_INFO(LOG_FEATURE_DRV, 
			    "Read GPS DATA:%02i.%02i.%i - %02i:%02i:%02i.%02i (epoch=%u) LAT=%f%c - LONG=%f%c  %s\r\n", DD,MM,YY,H,M,S,SS,epoch_time,Lat_f,NS,Long_f,EW,tempstr);

	p = strstr(++p, "$GPRMC,");
	} //while(p)

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
	uint8_t temp=Tokenizer_GetArgsCount()-1;
	const char* arg;
	const char* fake=NULL;	
	NEO6M_baudRate = 9600;	// default value
	setlatlong2gps = false;
	setclock2gps = false;
	fakelat[0]='\0';
	fakelong[0]='\0';
	for (int i=1; i<=temp; i++) {
		arg = Tokenizer_GetArg(i);
		
		ADDLOG_INFO(LOG_FEATURE_DRV,"NEO6M: argument %i/%i is %s",i,temp,arg);		

		if ( arg && !stricmp(arg,"setclock")) {
#if ENABLE_LOCAL_CLOCK || ENABLE_NTP
		setclock2gps=true;
		ADDLOG_INFO(LOG_FEATURE_DRV,"NEO6M: setting local clock to UTC time read if GPS is synched");
#else
		ADDLOG_INFO(LOG_FEATURE_DRV,"NEO6M: local clock not enabled - ignoring \"setclock\"");
#endif
		} 
		if ( arg && !stricmp(arg,"setlatlong")) {
#if (ENABLE_LOCAL_CLOCK || ENABLE_NTP) && ENABLE_CLOCK_SUNRISE_SUNSET
		setlatlong2gps=true;
		ADDLOG_INFO(LOG_FEATURE_DRV,"NEO6M: setting lat/long to values read by GPS");
#else
		ADDLOG_INFO(LOG_FEATURE_DRV,"NEO6M: local clock not enabled - ignoring \"setlatlong\"");
#endif
		}
		fake=strstr(arg, "fakelat=");
		if ( arg && fake ) {
			int i=0;
			fake += 8;
//			ADDLOG_INFO(LOG_FEATURE_DRV,"fake=%s fakelat=%s",fake,fakelat);
			while(fake[i]){
				fakelat[i]=fake[i];
				i++;
			};
			fakelat[i]='\0';
			ADDLOG_INFO(LOG_FEATURE_DRV,"NEO6M: fakelat=%s",fakelat);
		} 
		fake=NULL;
		fake=strstr(arg, "fakelong=");
		if ( arg && (fake=strstr(arg, "fakelong=")) ) {
			int i=0;
			fake +=9;
//			ADDLOG_INFO(LOG_FEATURE_DRV,"fake=%s fakelong=%s",fake,fakelong);
			while(fake[i]){
				fakelong[i]=fake[i];
				i++;
			};
			fakelong[i]='\0';
			ADDLOG_INFO(LOG_FEATURE_DRV,"NEO6M: fakelong=%s",fakelong);
		} 

		if (Tokenizer_IsArgInteger(i)){
			NEO6M_baudRate = Tokenizer_GetArgInteger(i);
			ADDLOG_INFO(LOG_FEATURE_DRV,"NEO6M: baudrate set to %i",NEO6M_baudRate);
		}
	}			
	UART_InitUART(NEO6M_baudRate, 0, 0);
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
		if (cs > 1 && g_secondsElapsed % 5 <= 4){
			UART_ConsumeBytes(cs -1);
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


void NEO6M_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState)
{
	if (bPreState)
		return;
	if (gpslocked) hprintf255(request, "<h5>GPS: %i-%02i-%02iT%02i:%02i:%02i Lat: %f%c Long: %f%c </h5>", YY,MM,DD,H,M,S,Lat_f,NS,Long_f,EW);
}
#endif // ENABLE_DRIVER_NEO6M
