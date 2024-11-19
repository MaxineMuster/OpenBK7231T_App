#include "drv_bl0942.h"

#include <math.h>
#include <stdint.h>

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

static int32_t Int24ToInt32(int32_t val) {
    return (val & (1 << 23) ? val | (0xFF << 24) : val);
}


static int UART_TryToGetNextPacket(void) {
	int cs;
	int i,LAT,LONG,tempnum;
	int c_garbage_consumed = 0;
	byte checksum,rxb;
	char data[NEO6M_UART_RECEIVE_BUFFER_SIZE+5]={0};
        char DATA1[4],DATA2[8],DATA3[8]; // to deal with leading "0", use characters	DATA1: LAT/LONG; DATA2: minutes; DATA3: fraction of minutes

	cs = UART_GetDataSize();

	i=0;
	for (i=0; i< cs; i++){
		data[i]=UART_GetByte(i);
	}
	data[++i]=0;
	UART_ConsumeBytes(UART_GetDataSize()-1);
//        ADDLOG_INFO(LOG_FEATURE_DRV, "GPS-UART: Consumed %i byte in buffer (cs=%i)\r\n\t -- DATA=%s\n", i,cs, data);
                    
                    
// Data is like "$GPRMC,142953.00,A,5200.03342,N,00830.72518,E,0.105,,200724,,,A*7C"
// $GPRMC,HHMMSS[.SSS],A,BBBB.BBBB,b,LLLLL.LLLL,l,GG.G,RR.R,DDMMYY,M.M,m,F*PP
//
// sscanf (sentence,"%s %*s %d",str,&i);sscanf (sentence,"%s %*s %d",str,&i);
//	int sret= sscanf (data,"$GPRMC,%2i%2i%2i.%i,A,%i.%i,%C,%i.%i,%C,%*s,%*s,%2i%2i%2i,%*s",&H,&M,&S,&SS,&BB,&BBB,b,&LL,&LLL,l,&DD,&MM,&YY);

	// calculate NMEA checksum
	int NMEACS=0;
	char *p = strchr(data, '$')+1 ; // so we are after the "$" of $GPRMC
//	ADDLOG_INFO(LOG_FEATURE_DRV, "calculating CS with p=%s \n",p);
	while (p && p[0] != '*') { NMEACS ^= p[0] ; p++;}
	int readcs=(int)strtol(++p, NULL, 16);
	
	// ADDLOG_INFO(LOG_FEATURE_DRV, "data=%s --  calculated CS=%i (%2X) - read=%i (%2X) ",data,NMEACS,NMEACS,readcs,readcs);
	if (NMEACS != readcs) {        
		ADDLOG_WARN(LOG_FEATURE_DRV, "GPS:  calculated CS (%2X) !=  read CS (%2X) ! Data might be invalid!\r\n",NMEACS,readcs); 
		
	};
	
	
	p = strchr(data, ',');	// so we are after $GPRMC
/*	// we want to use integers, but the numbers might have some leading zero
	// so we do some dirty trick to deal with leading "0": 
	// if number starts with digit 0, just replace with "illegal" number 9 (first two digits are hour, there is no hour 90 - 99)
	if ( p[1] == '0' ) p[1] = '9';
	// let's go for the time - string: ",091211.00" means: 09:12:11 [remember we will have 991211.00 here
 	int sret= sscanf (p+1,"%i.%i,%c",&tempnum,&SS,&NS);
	H=(tempnum/10000) % 90;	//remember to deal with leading 0 made to 9
	M=(tempnum%10000)/100;
	S=(tempnum%100);
*/
	int sret= sscanf (p+1,"%2d%2d%2d.%d,%c",&H,&M,&S,&SS,&NS);
	gpslocked =  (NS == 'A' );
	if (!gpslocked) {        
		ADDLOG_WARN(LOG_FEATURE_DRV, "GPS: no lock! Only date/time might be valid!\r\n"); 
		
	};
//	else {
		p = strchr(p+1, ','); p = strchr(p+1, ',');	// Next is "A" or "V", so continue
/*
		sret= sscanf (p+1,"%i.%i",&LAT,&tempnum);
		ADDLOG_INFO(LOG_FEATURE_DRV, "DEBUG: read %i elements --- p+1=%s \r\nLAT=%i tempnum=%i\r\n",sret,p+1,LAT,tempnum);
		
		sret= sscanf (p+1,"%f,%c",&tempfloat,&NS);

		LAT=(int)tempfloat/100;
		Lat_f=LAT+(tempfloat-(100*LAT))/60;


		ADDLOG_INFO(LOG_FEATURE_DRV, "DEBUG: read %i elements --- p+1=%s \r\ntempfloat=%f\r\n",sret,p+1,tempfloat);
*/

		sret= sscanf (p+1,"%c%c%c%c.%c%c%c%c%c,%c",&DATA1[0],&DATA1[1],&DATA2[0],&DATA2[1],&DATA2[3],&DATA2[4],&DATA2[5],&DATA2[6],&DATA2[7],&NS);
		DATA1[2]=0;
		DATA2[2]='.';
		DATA2[8]=0;
		Lat_f=atoi(DATA1)+atof(DATA2)/60;

//		ADDLOG_INFO(LOG_FEATURE_DRV, "DEBUG GPS: Lat_f=%f\r\n",Lat_f);
		p = strchr(p+1, ','); p = strchr(p+1, ',');	// pass lat and N/S

/*
		sret = sscanf (p+1,"%f,%c",&tempfloat,&EW);
		ADDLOG_INFO(LOG_FEATURE_DRV, "DEBUG: p+1=%s \r\ntempfloat=%f\r\n",p+1,tempfloat);
		LONG=(int)tempfloat/100;
		Long_f=LONG+(tempfloat-(100*LONG))/60;
*/

		sret= sscanf (p+1,"%c%c%c%c%c.%c%c%c%c%c,%c",&DATA1[0],&DATA1[1],&DATA1[2],&DATA2[0],&DATA2[1],&DATA2[3],&DATA2[4],&DATA2[5],&DATA2[6],&DATA2[7],&EW);
		DATA1[3]=0;
		DATA2[2]='.';
		DATA2[8]=0;
		Long_f=atoi(DATA1)+atof(DATA2)/60;

//		ADDLOG_INFO(LOG_FEATURE_DRV, "DEBUG GPS: Lat_f=%f\r\n",Lat_f);


		p = strchr(p+1, ','); p = strchr(p+1, ',');	// pass long and E/W
		p = strchr(p+1, ','); p = strchr(p+1, ',');	// ignore speed and course
//		sret= sscanf (p+1,"%2i%2i%2i",&DD,&MM,&YY);
		// we want to use integers, but the numbers might have some leading zero
		// so we do some dirty trick to deal with leading "0": 
		// if number starts with digit 0, just replace with "illegal" number 9 (first two digits are hour, there is no hour 90 - 99)
		if ( p[1] == '0' ) p[1] = '9';
		// let's go for the time - string: ",091211.00" means: 09:12:11 [remember we will have 991211.00 here
 		sret= sscanf (p+1,"%i",&tempnum);
		DD=(tempnum/10000) % 90;	//remember to deal with leading 0 made to 9
		MM=(tempnum%10000)/100;
		YY=2000+(tempnum%100);




			
		ADDLOG_INFO(LOG_FEATURE_DRV, 
		            "Read GPS DATA:%02i.%02i.%i - %02i:%02i:%02i.%02i  LAT=%f%c - LONG=%f%c  \r\n", DD,MM,YY,H,M,S,SS,Lat_f,NS,Long_f,EW);

//	}
	return 0;
}

// get "minutes" part of latitude/longitude and return decimal fraction
// return value is fractional part, 100.000 times 
// e.g. from input value
// 5201.03273 	= 52.001721
// ddmm mmmmm
// we would need 0103272 to be converted
// returned result is value / 60
// here: 103272 / 60 = 1721,216666667
// co


static void min2deci(int input){
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
    char send[7][26]={
    	"$PUBX,40,GGA,0,0,0,0*5A\r\n",
    	"$PUBX,40,GLL,0,0,0,0*5C\r\n",
    	"$PUBX,40,GSA,0,0,0,0*4E\r\n",
    	"$PUBX,40,GSV,0,0,0,0*59\r\n",
    	"$PUBX,40,RMC,0,0,0,0*47\r\n",
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

    UART_TryToGetNextPacket();

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

}


void NEO6M_AppendInformationToHTTPIndexPage(http_request_t *request)
{
	if (gpslocked) hprintf255(request, "<h5>GPS: %i-%02i-%02iT%02i:%02i:%02i Lat: %f%c Long: %f%c </h5>", YY,MM,DD,H,M,S,Lat_f,NS,Long_f,EW);
}

