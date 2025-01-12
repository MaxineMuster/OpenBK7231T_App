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
// parse will stop at *, so not needed during parsing
//NMEA_STAR,
//NMEA_CHECKSUM,
NMEA_WORDS
};


void parseGPS(char *data){

	char *p = strstr(data, "$GPRMC");
	if (p){

	// Data is like "$GPRMC,142953.00,A,5213.00212,N,02101.98018,E,0.105,,200724,,,A*71"
	// $GPRMC,HHMMSS[.SSS],A,BBBB.BBBB,b,LLLLL.LLLL,l,GG.G,RR.R,DDMMYY,M.M,m,F*PP
	//
		// calculate NMEA checksum
		int NMEACS=103;		// Start with checksum value for "GPRMC," - we don't need to parse it later and strsts makes sure, we have it in string
		int word=0,letter=0;
		char Nvalue[NMEA_WORDS][12]={0};
		
		p+=7 ; // so we are after the "," of $GPRMC, and can start calculation and assignment
		//ADDLOG_INFO(LOG_FEATURE_DRV, "calculating CS with p=%s \n",p);
		while (p && p[0] != '*') { 
			NMEACS ^= p[0];
			if (p[0] != ','){
				Nvalue[word][letter++]=p[0]; 
			}
			else {
				word++;
				letter=0;
			}
			p++;
		}
		// checksum in parsed data
		int readcs=(int)strtol(++p, NULL, 16);

		if (NMEACS != readcs) {        
			ADDLOG_WARN(LOG_FEATURE_DRV, "GPS:  calculated CS (%2X) !=  read CS (%2X) ! Data might be invalid!\r\n",NMEACS,readcs); 		
		};
		
		bool timeread = (sscanf (Nvalue[NMEA_TIME],"%2d%2d%2d.%d",&H,&M,&S,&SS) >=3);
		bool dateread = (sscanf (Nvalue[NMEA_DATE],"%2d%2d%2d",&DD,&MM,&YY) ==3);
		YY+=2000;

		gpslocked=(Nvalue[NMEA_LOCK][0]=='A');
		if (!gpslocked) {        
			ADDLOG_WARN(LOG_FEATURE_DRV, "GPS: no lock! %s\r\n", timeread && dateread ? "Date/time might be valid." :""); 
			
		};

		NS=Nvalue[NMEA_LAT_DIR][0];

		// NMEA returns degrees as degree and "minutes" of latitude/longitude 
		// we want it as a decimal representation 
		// e.g. from input value
		// 5213.00212 	= 52.216702
		// ddmm.mmmmm
		// we would need the minutes 13.00212 to be converted
		// the result is value / 60
		// here: 1.03272 / 60 = 0.216702

		float frac;
		int whole;						// need to get tw digits as %2d, using "%2f" will take the wholl value as float?!?
//		sscanf (Nvalue[NMEA_LAT],"%2f%f",&Lat_f,&frac);			// Lat_f now contains whole part of degrees
		sscanf (Nvalue[NMEA_LAT],"%2d%f",&whole,&frac);			// whole now contains whole part of degrees
		Lat_f=whole;							// Lat_f now contains whole part of degrees
		Lat_f+=frac/60;							// add fractional part of degrees - NMEA gives minutes so divide by 60 


		EW=Nvalue[NMEA_LONG_DIR][0];
//		sscanf (Nvalue[NMEA_LONG],"%3f%f",&Long_f,&frac);			// Long_f now contains whole part of degrees
		sscanf (Nvalue[NMEA_LONG],"%3d%f",&whole,&frac);			// whole now contains whole part of degrees
		Long_f = whole;								// Long_f now contains whole part of degrees
		Long_f+=frac/60;							// add fractional part of degrees - NMEA gives minutes so divide by 60 
				
		ADDLOG_INFO(LOG_FEATURE_DRV, 
			    "Read GPS DATA:%02i.%02i.%i - %02i:%02i:%02i.%02i  LAT=%f%c - LONG=%f%c  \r\n", DD,MM,YY,H,M,S,SS,Lat_f,NS,Long_f,EW);
	}
}


static int UART_GetNextPacket(void) {
	int cs;
	char data[NEO6M_UART_RECEIVE_BUFFER_SIZE+5]={0};

	cs = UART_GetDataSize();

	int i=0;
	for (i=0; i< cs; i++){
		data[i]=UART_GetByte(i);
	}
	data[++i]=0;
	UART_ConsumeBytes(UART_GetDataSize()-1);
	parseGPS(data);
	return 0;
}



// send "$EIGPQ,RMC*3A\r\n" to request/poll DATA
static void UART_WritePollReq(void) {
    uint8_t send[]="$EIGPQ,RMC*3A\r\n";

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
//    	"$PUBX,40,RMC,0,0,0,0*47\r\n",
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

    UART_InitUART(NEO6M_baudRate, 0);
    UART_WritePollReq();

    UART_GetNextPacket();

    UART_InitUART(NEO6M_baudRate, 0);

}



void NEO6M_UART_RunEverySecond(void) {

/*
    UART_TryToGetNextPacket();

    UART_InitUART(NEO6M_baudRate, 0);
*/
	if (g_secondsElapsed % 5 == 0) {	// every 5 seconds
		NEO6M_requestData();
	}
	else UART_ConsumeBytes(UART_GetDataSize()-1);

}


void NEO6M_AppendInformationToHTTPIndexPage(http_request_t *request)
{
	if (gpslocked) hprintf255(request, "<h5>GPS: %i-%02i-%02iT%02i:%02i:%02i Lat: %f%c Long: %f%c </h5>", YY,MM,DD,H,M,S,Lat_f,NS,Long_f,EW);
}
#endif // ENABLE_DRIVER_NEO6M
