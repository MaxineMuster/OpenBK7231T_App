#include "../new_common.h"
#include "../obk_config.h"

#if ENABLE_HTTP_HEADERTIME

#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"

#include "utils_timer.h"
//#include "lite-log.h"
#include "http_client.h"
#include "iot_export_errno.h"
#include "../hal/hal_wifi.h"	// for HAL_GetMyGatewayString()

#include "../libraries/obktime/obktime.h"	// for time functions
#include "../driver/drv_deviceclock.h"



#define HEAD_REQUEST "HEAD / HTTP/1.0\r\nConnection: close\r\n\r\n"

void http_get_headertime(const char* host, unsigned short server_port, int offset) {
    int sock;
    struct sockaddr_in server_addr = { 8, AF_INET, htons(server_port), inet_addr(host)};
    char buffer[512];
    
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return;
        
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(sock);
        return;
    }

    send(sock, HEAD_REQUEST, strlen(HEAD_REQUEST), 0);
    
    char *start=NULL;
    int recv_size = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (recv_size > 0) {
        buffer[recv_size] = '\0';  // Null-terminate the received data
        
        start = strstr(buffer, "Date:");
        if (start) {
            start += 11; // Move past "Date: <Day>," <Day> is always 3 letters 
            char *end = strchr(start, '\r'); // Find end of line
            
            if (end) {
                *end = '\0'; // Null terminate the time string
            }
        }
    }
    
    close(sock);    
    /* parse date like "25 Dec 2025 12:34:56 GMT"  - remember we "stripped" day already */
    const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    int day = 0, year = 0, hour = 0, minute = 0, second = 0, month=0;
    char m[4] = {0};
    uint32_t epoch = 0;
    int parsed = sscanf(start, "%d %3s %d %d:%d:%d", &day, m, &year, &hour, &minute, &second);
    const char *p = strstr(months, m);
    month = p ? (int)((p - months) / 3) + 1 : 0;
    if (month) epoch = dateToEpoch((uint16_t)year, month, day, hour, minute, second);
    ADDLOG_DEBUG(LOG_FEATURE_CMD,"Extracted Date: %s -- epoch:%i\n", start, epoch);
    if (epoch) {
    	TIME_setDeviceTime(epoch+offset);
    	ADDLOG_INFO(LOG_FEATURE_CMD,"Devicetime set to epoch:%i\n", epoch);
    }
    
}
static commandResult_t CMD_GET_HTTP_Headertime(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_ALLOW_ESCAPING_QUOTATIONS);
	int offset = Tokenizer_GetArgIntegerDefault(1, 2);	// 2 seconds offset as default
	unsigned short port = (unsigned short)Tokenizer_GetArgIntegerDefault(2, 80);
	ADDLOG_DEBUG(LOG_FEATURE_CMD, " CMD_GET_HTTP_Headertime%s%s - IP:%s port:%i Offset:%i",Tokenizer_GetArgsCount()==0 ? "" : " - args: ", args,Tokenizer_GetArgsCount()==0 ? HAL_GetMyGatewayString() : Tokenizer_GetArg(0), port, offset);

	http_get_headertime(Tokenizer_GetArgsCount()==0 ? HAL_GetMyGatewayString() : Tokenizer_GetArg(0), port, offset);
	return CMD_RES_OK;
}

void CMD_InitGetHeaderTime() {
	//cmddetail:{"name":"getHeaderTime","args":"[IP to query - Gateway as default] [seconds to add for delay - dafault 2] [Port - default 80]",
	//cmddetail:"descr":"Setting device time to 'Date: ' statement in HTTP header",
	//cmddetail:"fn":"CMD_GET_HTTP_Headertimer","file":"httpclient/http_getheadertime.c","requires":"",
	//cmddetail:"examples":"getHeaderTime 192.168.0.5 3 (gets time from 192.168.0.5 and adds 3 seconds to set device time)"}
	CMD_RegisterCommand("getHeaderTime", CMD_GET_HTTP_Headertime, NULL);
}

#endif // #if ENABLE_HTTP_HEADERTIME
