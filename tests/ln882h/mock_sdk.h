/* Host-only SDK facade. The production HAL source is compiled unchanged.
 * Public Wi-Fi types mirror OpenLN882H wifi.h; OS/network services are fakes.
 */
#ifndef OBK_LN882H_MOCK_SDK_H
#define OBK_LN882H_MOCK_SDK_H
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PLATFORM_LN882H 1
#define PLATFORM_LN8825 0
#define SSID_MAX_LEN 33
#define BSSID_LEN 6
#define LN_FALSE 0
#define LN_TRUE 1
#define OS_WAIT_FOREVER UINT32_MAX
#define OS_OK 0
#define OS_FAIL -1
#define OS_E_TIMEOUT -2
#define WIFI_ERR_NONE 0
#define WIFI_NO_POWERSAVE 0
#define WIFI_SCAN_TYPE_ACTIVE 0
#define WIFI_AUTH_OPEN 0
#define LOG_LVL_INFO 0
#define LOG_LVL_ERROR 1
#define LOG_FEATURE_GENERAL 0
#define NETIF_IDX_STA 0
#define NETIF_IDX_AP 1
#define NETDEV_UP 1
#define SYSPARAM_ERR_NONE 0
#define OBK_FLAG_WIFI_ENHANCED_FAST_CONNECT 1
#define STA_MAC_ADDR0 0
#define STA_MAC_ADDR1 0
#define STA_MAC_ADDR2 0
#define STA_MAC_ADDR3 0
#define STA_MAC_ADDR4 0
#define STA_MAC_ADDR5 0
#define SOFTAP_MAC_ADDR0 0
#define SOFTAP_MAC_ADDR1 0
#define SOFTAP_MAC_ADDR2 0
#define SOFTAP_MAC_ADDR3 0
#define SOFTAP_MAC_ADDR4 0
#define SOFTAP_MAC_ADDR5 0
#define LN_UNUSED(x) (void)(x)
#define MACSTR "%02X:%02X:%02X:%02X:%02X:%02X"
#define MAC2STR(m) (m)[0], (m)[1], (m)[2], (m)[3], (m)[4], (m)[5]
#define LOG(...) mock_log(__VA_ARGS__)
#define ADDLOG_INFO(feature, ...) mock_log(0, __VA_ARGS__)
#define ADDLOG_WARN(feature, ...) mock_log(1, __VA_ARGS__)
#define ADDLOG_ERROR(feature, ...) mock_log(2, __VA_ARGS__)
#define __sprintf(...) ((void)0)
#define os_malloc malloc

typedef struct { bool valid; } OS_Mutex_t;
typedef int OS_Status;
typedef uint32_t OS_Time_t;
typedef struct { int count; bool valid; } OS_Semaphore_t;
typedef int sta_ps_mode_t;
typedef int netif_idx_t;
typedef struct { uint32_t addr; } ip_addr_t;
typedef struct { ip_addr_t ip, netmask, gw; } tcpip_ip_info_t;
struct netif { ip_addr_t ip_addr, netmask, gw; const char *hostname; uint8_t hwaddr[6]; };
typedef struct { uint8_t localIPAddr[4], netMask[4], gatewayIPAddr[4], dnsServerIpAddr[4]; } obkStaticIP_t;
typedef struct { uint8_t bssid[6]; uint8_t channel; uint8_t psk[32]; } obkFastConnectData_t;
typedef struct {
    ip_addr_t server;
    int port, lease, renew, client_max;
    ip_addr_t ip_start, ip_end;
} server_config_t;
typedef enum {
    WIFI_STA_STATUS_STARTUP, WIFI_STA_STATUS_SCANING, WIFI_STA_STATUS_CONNECTING,
    WIFI_STA_STATUS_CONNECTED, WIFI_STA_STATUS_DISCONNECTING, WIFI_STA_STATUS_DISCONNECTED
} wifi_sta_status_t;
typedef enum {
    WIFI_STA_CONN_SUCCESSFUL, WIFI_STA_CONN_WRONG_PWD,
    WIFI_STA_CONN_TARGET_AP_NOT_FOUND, WIFI_STA_CONN_TIMEOUT, WIFI_STA_CONN_REFUSED
} wifi_sta_connect_failed_reason_t;
typedef enum {
    WIFI_MGR_EVENT_STA_STARTUP, WIFI_MGR_EVENT_STA_CONNECTED,
    WIFI_MGR_EVENT_STA_DISCONNECTED, WIFI_MGR_EVENT_STA_SCAN_COMPLETE,
    WIFI_MGR_EVENT_STA_CONNECT_FAILED, WIFI_MGR_EVENT_SOFTAP_STARTUP, WIFI_MGR_EVENT_MAX
} wifi_mgr_event_t;
enum { WIFI_STA_CONNECTED, WIFI_STA_DISCONNECTED, WIFI_STA_CONNECTING,
       WIFI_STA_AUTH_FAILED, WIFI_AP_CONNECTED, WIFI_AP_FAILED };
typedef void (*wifi_mgr_event_cb_t)(void *);
typedef struct { uint8_t channel; int scan_type; uint16_t scan_time; } wifi_scan_cfg_t;
typedef struct { const char *ssid, *pwd; uint8_t *bssid, *psk_value; } wifi_sta_connect_t;
typedef struct {
    const char *ssid, *pwd; uint8_t *bssid;
    struct { uint8_t channel; int authmode, ssid_hidden, beacon_interval; uint8_t *psk_value; } ext_cfg;
} wifi_softap_cfg_t;
typedef struct {
    uint8_t bssid[BSSID_LEN]; char ssid[SSID_MAX_LEN]; uint8_t channel;
    int authmode; uint8_t imode; int8_t rssi; int16_t freq_offset; uint8_t bgn;
    uint8_t wps_en:1, is_hidden:1, rsn_mfpr:1, rsn_mfpc:1, set_wpa_sae_support:1;
} ap_info_t;
typedef struct ln_list { struct ln_list *next, *prev; } ln_list_t;
typedef struct { ln_list_t list; ap_info_t info; uint32_t life_ticks; } ap_info_node_t;
#define LN_LIST_ENTRY(p, type, member) ((type *)((char *)(p) - offsetof(type, member)))
#define LN_LIST_FOR_EACH_ENTRY(p, type, member, head) \
    for (ln_list_t *it_ = (head)->next; it_ != (head) && ((p) = LN_LIST_ENTRY(it_, type, member), 1); it_ = it_->next)

void mock_log(int level, const char *fmt, ...);
OS_Status OS_MutexCreate(OS_Mutex_t *);
OS_Status OS_MutexLock(OS_Mutex_t *, OS_Time_t);
OS_Status OS_MutexUnlock(OS_Mutex_t *);
OS_Status OS_SemaphoreCreate(OS_Semaphore_t *, uint32_t, uint32_t);
OS_Status OS_SemaphoreWait(OS_Semaphore_t *, OS_Time_t);
OS_Status OS_SemaphoreRelease(OS_Semaphore_t *);
OS_Status OS_SemaphoreDelete(OS_Semaphore_t *);
void OS_MsDelay(unsigned);
int wifi_manager_reg_event_callback(wifi_mgr_event_t, wifi_mgr_event_cb_t);
int wifi_manager_ap_list_update_enable(int);
int wifi_manager_get_ap_list(ln_list_t **, uint8_t *);
void wifi_manager_cleanup_scan_results(void);
int wifi_sta_scan(wifi_scan_cfg_t *);
int wifi_get_sta_status(wifi_sta_status_t *);
int wifi_sta_start(uint8_t *, sta_ps_mode_t);
int wifi_sta_connect(wifi_sta_connect_t *, wifi_scan_cfg_t *);
int wifi_sta_disconnect(void);
int wifi_get_sta_conn_info(const char **, const uint8_t **);
int wifi_get_sta_scan_cfg(wifi_scan_cfg_t *);
int wifi_sta_get_rssi(int8_t *);
int wifi_softap_start(wifi_softap_cfg_t *);
void ln_wpa_sae_enable(void);
int ln_psk_calc(const char *, const char *, uint8_t *, size_t);
void hexdump(int, const char *, const void *, size_t);
int ln_kv_has_key(const char *);
int ln_kv_get(const char *, void *, size_t, size_t *);
int ln_kv_set(const char *, const void *, size_t);
int ln_kv_del(const char *);
int CFG_HasFlag(int);
const char *CFG_GetDeviceName(void);
void convert_IP_to_string(char *, const uint8_t *);
char *ip4addr_ntoa(const ip_addr_t *);
uint32_t ipaddr_addr(const char *);
int netdev_got_ip(void);
netif_idx_t netdev_get_active(void);
struct netif *netdev_get_netif(netif_idx_t);
void netdev_set_mac_addr(netif_idx_t, uint8_t *);
void netdev_set_ip_info(netif_idx_t, tcpip_ip_info_t *);
void netdev_set_active(netif_idx_t);
void netdev_set_state(netif_idx_t, int);
void dns_setserver(int, const void *);
int sysparam_sta_mac_get(uint8_t *);
int sysparam_sta_mac_update(const uint8_t *);
void sysparam_sta_hostname_update(const char *);
int sysparam_softap_mac_get(uint8_t *);
int sysparam_softap_mac_update(const uint8_t *);
void ln_generate_random_mac(uint8_t *);
void dhcpd_curr_config_set(server_config_t *);
void HAL_ConnectToWiFi(const char *, const char *, obkStaticIP_t *);
#endif
