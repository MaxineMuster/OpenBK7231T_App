#include "mock_sdk.h"
#include <pthread.h>
#include <sched.h>

#define MAX_APS 8
#define MAX_SCANS 8
static wifi_mgr_event_cb_t callbacks[WIFI_MGR_EVENT_MAX];
static ap_info_t scan_data[MAX_SCANS][MAX_APS];
static unsigned scan_sizes[MAX_SCANS];
static ap_info_node_t nodes[MAX_APS];
static ln_list_t ap_list;
static unsigned node_count;
static int scan_calls, connect_calls, create_calls, delete_calls, wait_calls, clear_calls;
static int fail_scan = -1, timeout_scan = -1, fail_start, fail_create, fail_release, fail_list;
static bool immediate_completion, updates_enabled = true;
static wifi_sta_status_t sta_status = WIFI_STA_STATUS_STARTUP;
static bool connected_with_bssid;
static uint8_t connected_bssid[6], *connected_bssid_pointer, connected_channel;
static uint8_t *connected_psk;
static const char *connected_ssid;
static int kv_present, enhanced_fast_connect, kv_writes;
static size_t kv_length = sizeof(obkFastConnectData_t);
static obkFastConnectData_t kv_data;
static int last_event = -1;
static unsigned status_calls;
static int fail_mutex_create, mutex_depth;
static bool check_list_lock;
static unsigned int release_gate;
static bool duplicate_completion;
static int reject_connection, fail_status;

static void complete_scan(void);
/* This includes the real HAL, not a rewritten copy of its functions. */
#include "hal_wifi_ln882h.c"

void mock_log(int level, const char *fmt, ...) { (void)level; (void)fmt; }
OS_Status OS_MutexCreate(OS_Mutex_t *mutex) {
    if (fail_mutex_create) return OS_FAIL;
    mutex->valid = true; return OS_OK;
}
OS_Status OS_MutexLock(OS_Mutex_t *mutex, OS_Time_t ms) {
    (void)ms; assert(mutex->valid); assert(mutex_depth == 0); ++mutex_depth; return OS_OK;
}
OS_Status OS_MutexUnlock(OS_Mutex_t *mutex) {
    assert(mutex->valid); assert(mutex_depth == 1); --mutex_depth; return OS_OK;
}
static void status_callback(int status) { last_event = status; }
OS_Status OS_SemaphoreCreate(OS_Semaphore_t *sem, uint32_t initial, uint32_t maximum) {
    assert(initial == 0 && maximum == 1); ++create_calls;
    if (fail_create) return OS_FAIL;
    sem->count = 0; sem->valid = true; return OS_OK;
}
OS_Status OS_SemaphoreRelease(OS_Semaphore_t *sem) {
    assert(sem && sem->valid);
    if (__atomic_load_n(&release_gate, __ATOMIC_ACQUIRE) == 1) {
        __atomic_store_n(&release_gate, 2, __ATOMIC_RELEASE);
        while (__atomic_load_n(&release_gate, __ATOMIC_ACQUIRE) == 2) sched_yield();
    }
    if (fail_release || sem->count) return OS_FAIL;
    sem->count = 1; return OS_OK;
}
OS_Status OS_SemaphoreWait(OS_Semaphore_t *sem, OS_Time_t ms) {
    assert(sem && sem->valid);
    if (ms) {
        ++wait_calls;
        if (scan_calls - 1 == timeout_scan) return OS_E_TIMEOUT;
        if (!sem->count) complete_scan();
    }
    if (!sem->count) return OS_E_TIMEOUT;
    sem->count = 0; return OS_OK;
}
OS_Status OS_SemaphoreDelete(OS_Semaphore_t *sem) {
    assert(sem && sem->valid); sem->valid = false; ++delete_calls; return OS_OK;
}
void OS_MsDelay(unsigned ms) { (void)ms; }
int wifi_manager_reg_event_callback(wifi_mgr_event_t event, wifi_mgr_event_cb_t callback) {
    callbacks[event] = callback; return WIFI_ERR_NONE;
}
int wifi_manager_ap_list_update_enable(int enable) {
    updates_enabled = enable != 0; return WIFI_ERR_NONE;
}
int wifi_manager_get_ap_list(ln_list_t **list, uint8_t *count) {
    if (check_list_lock) assert(mutex_depth == 1);
    if (fail_list) return -1;
    *list = &ap_list; *count = (uint8_t)node_count; return WIFI_ERR_NONE;
}
void wifi_manager_cleanup_scan_results(void) {
    if (check_list_lock) assert(mutex_depth == 1);
    ++clear_calls; ap_list.next = ap_list.prev = &ap_list; node_count = 0;
}
static void add_cached_ap(ap_info_t ap) {
    assert(node_count < MAX_APS);
    ap_info_node_t *n = &nodes[node_count++]; n->info = ap;
    n->list.next = &ap_list; n->list.prev = ap_list.prev;
    ap_list.prev->next = &n->list; ap_list.prev = &n->list;
}
static void complete_scan(void) {
    assert(scan_calls > 0);
    for (unsigned i = 0; i < scan_sizes[scan_calls - 1]; ++i)
        add_cached_ap(scan_data[scan_calls - 1][i]);
    sta_status = WIFI_STA_STATUS_DISCONNECTED;
    assert(callbacks[WIFI_MGR_EVENT_STA_SCAN_COMPLETE]);
    callbacks[WIFI_MGR_EVENT_STA_SCAN_COMPLETE](NULL);
    if (duplicate_completion) callbacks[WIFI_MGR_EVENT_STA_SCAN_COMPLETE](NULL);
}
int wifi_sta_scan(wifi_scan_cfg_t *scan) {
    assert(scan->channel == 0); assert(scan_calls < MAX_SCANS);
    ++scan_calls;
    if (scan_calls - 1 == fail_scan) return -1;
    sta_status = WIFI_STA_STATUS_SCANING;
    if (immediate_completion) complete_scan();
    return WIFI_ERR_NONE;
}
int wifi_get_sta_status(wifi_sta_status_t *status) {
    ++status_calls; *status = sta_status; return fail_status ? -1 : WIFI_ERR_NONE;
}
int wifi_sta_start(uint8_t *mac, sta_ps_mode_t mode) { (void)mac; (void)mode; return fail_start ? -1 : WIFI_ERR_NONE; }
int wifi_sta_connect(wifi_sta_connect_t *connect, wifi_scan_cfg_t *scan) {
    ++connect_calls; connected_with_bssid = connect->bssid != NULL;
    connected_bssid_pointer = connect->bssid;
    if (connect->bssid) memcpy(connected_bssid, connect->bssid, 6);
    connected_channel = scan->channel; connected_psk = connect->psk_value;
    connected_ssid = connect->ssid; sta_status = WIFI_STA_STATUS_CONNECTING;
    if (reject_connection == 2) {
        wifi_sta_connect_failed_reason_t reason = WIFI_STA_CONN_TIMEOUT;
        callbacks[WIFI_MGR_EVENT_STA_CONNECT_FAILED](&reason);
    }
    return reject_connection ? -1 : WIFI_ERR_NONE;
}
int wifi_sta_disconnect(void) { sta_status = WIFI_STA_STATUS_DISCONNECTED; return WIFI_ERR_NONE; }
int wifi_get_sta_conn_info(const char **ssid, const uint8_t **bssid) {
    *ssid = connected_ssid; *bssid = connected_bssid; return WIFI_ERR_NONE;
}
int wifi_get_sta_scan_cfg(wifi_scan_cfg_t *scan) { scan->channel = connected_channel; return WIFI_ERR_NONE; }
int wifi_sta_get_rssi(int8_t *rssi) { *rssi = -40; return WIFI_ERR_NONE; }
int wifi_softap_start(wifi_softap_cfg_t *cfg) { (void)cfg; return WIFI_ERR_NONE; }
void ln_wpa_sae_enable(void) { }
int ln_psk_calc(const char *ssid, const char *password, uint8_t *out, size_t len) {
    (void)ssid; (void)password; assert(out); memset(out, 0x23, len); return 0;
}
void hexdump(int level, const char *name, const void *data, size_t len) { (void)level; (void)name; (void)data; (void)len; }
int ln_kv_has_key(const char *key) { (void)key; return kv_present; }
int ln_kv_get(const char *key, void *out, size_t size, size_t *len) {
    (void)key; assert(size == sizeof(kv_data)); memcpy(out, &kv_data, size); *len = kv_length; return 0;
}
int ln_kv_set(const char *key, const void *data, size_t size) {
    (void)key; (void)data; (void)size; ++kv_writes; return 0;
}
int ln_kv_del(const char *key) { (void)key; kv_present = 0; return 0; }
int CFG_HasFlag(int flag) { (void)flag; return enhanced_fast_connect; }
const char *CFG_GetDeviceName(void) { return "OBK test"; }
void convert_IP_to_string(char *out, const uint8_t *ip) { (void)ip; strcpy(out, "192.168.1.2"); }
char *ip4addr_ntoa(const ip_addr_t *ip) { (void)ip; static char s[] = "192.168.1.2"; return s; }
uint32_t ipaddr_addr(const char *s) { (void)s; return 0; }
int netdev_got_ip(void) { return 1; }
netif_idx_t netdev_get_active(void) { return NETIF_IDX_STA; }
struct netif *netdev_get_netif(netif_idx_t id) { (void)id; static struct netif nif; return &nif; }
void netdev_set_mac_addr(netif_idx_t id, uint8_t *mac) { (void)id; (void)mac; }
void netdev_set_ip_info(netif_idx_t id, tcpip_ip_info_t *ip) { (void)id; (void)ip; }
void netdev_set_active(netif_idx_t id) { (void)id; }
void netdev_set_state(netif_idx_t id, int state) { (void)id; (void)state; }
void dns_setserver(int index, const void *ip) { (void)index; (void)ip; }
int sysparam_sta_mac_get(uint8_t *mac) { memset(mac, 0x22, 6); return SYSPARAM_ERR_NONE; }
int sysparam_sta_mac_update(const uint8_t *mac) { (void)mac; return 0; }
void sysparam_sta_hostname_update(const char *name) { (void)name; }
int sysparam_softap_mac_get(uint8_t *mac) { return sysparam_sta_mac_get(mac); }
int sysparam_softap_mac_update(const uint8_t *mac) { (void)mac; return 0; }
void ln_generate_random_mac(uint8_t *mac) { memset(mac, 0x22, 6); }
void dhcpd_curr_config_set(server_config_t *cfg) { (void)cfg; }

static obkStaticIP_t ip;
static ap_info_t ap(const char *ssid, uint8_t id, int rssi, uint8_t channel) {
    ap_info_t result = {0};
    assert(strlen(ssid) < sizeof(result.ssid)); strcpy(result.ssid, ssid);
    result.bssid[0] = 0x82; result.bssid[1] = 0xAB; result.bssid[5] = id;
    result.rssi = (int8_t)rssi; result.channel = channel; return result;
}
static void prepare_pair(void) {
    for (unsigned pass = 0; pass < 2; ++pass) {
        scan_data[pass][0] = ap("target", 1, -75, 1);
        scan_data[pass][1] = ap("other", 9, -20, 6);
        scan_data[pass][2] = ap("target", 2, -35, 11);
        scan_sizes[pass] = 3;
    }
}
static void connect_normal(const char *ssid) { HAL_ConnectToWiFi(ssid, "password", &ip); }
static void assert_unpinned(void) { assert(connect_calls > 0); assert(!connected_with_bssid); assert(connected_channel == 0); }
static void assert_best(uint8_t id, uint8_t channel) {
    assert(connected_with_bssid); assert(connected_bssid[0] == 0x82);
    assert(connected_bssid[1] == 0xAB); assert(connected_bssid[5] == id);
    assert(connected_channel == channel);
}
static void strongest(void) { prepare_pair(); connect_normal("target"); assert_best(2, 11); assert(scan_calls == 2); }
static void no_match(void) { prepare_pair(); connect_normal("missing"); assert_unpinned(); assert(scan_calls == 2); }
static void high_bytes(void) { prepare_pair(); scan_data[0][2].bssid[5] = scan_data[1][2].bssid[5] = 0xFF; connect_normal("target"); assert_best(0xFF, 11); }
static void fresh_second_scan(void) {
    prepare_pair(); scan_data[1][0] = ap("target", 3, -60, 3); scan_sizes[1] = 1;
    connect_normal("target"); assert_best(3, 3); assert(clear_calls == 2);
}
static void cache_is_cleared(void) { prepare_pair(); add_cached_ap(ap("target", 7, -1, 7)); connect_normal("target"); assert_best(2, 11); }
static void second_empty(void) { prepare_pair(); scan_sizes[1] = 0; connect_normal("target"); assert_unpinned(); }
/* Preserve main's existing association attempt when wifi_sta_start reports
 * an error (the SDK may already be started). Do not add optional scans there. */
static void start_failed(void) { fail_start = 1; connect_normal("target"); assert(scan_calls == 0); assert_unpinned(); }
static void allocation_failed(void) { fail_create = 1; connect_normal("target"); assert_unpinned(); assert(scan_calls == 0); assert(delete_calls == 0); }
static void scan_rejected(void) { fail_scan = 0; connect_normal("target"); assert_unpinned(); assert(wait_calls == 0); assert(delete_calls == 0); }
static void second_rejected(void) { prepare_pair(); fail_scan = 1; connect_normal("target"); assert_unpinned(); assert(wait_calls == 1); }
static void timeout_and_late_event(void) {
    prepare_pair(); timeout_scan = 0; connect_normal("target"); assert_unpinned(); assert(delete_calls == 0);
    complete_scan(); /* A real scan may finish after the wait expires. */
    HAL_DisconnectFromWifi(); connect_normal("target");
    assert_unpinned(); assert(scan_calls == 1); assert(create_calls == 1); assert(delete_calls == 0);
}
static void second_timeout(void) { prepare_pair(); timeout_scan = 1; connect_normal("target"); assert_unpinned(); assert(delete_calls == 0); }
static void release_failed(void) { prepare_pair(); fail_release = 1; connect_normal("target"); assert_unpinned(); assert(delete_calls == 0); }
static void synchronous_completion(void) { immediate_completion = true; strongest(); }
static void fast_connect_preserved(void) {
    uint8_t mac[6] = {0x02, 1, 2, 3, 4, 5}, psk[32] = {0x51};
    wifi_init_sta("target", "password", &ip, 6, mac, psk);
    assert(scan_calls == 0); assert(connected_bssid_pointer == mac);
    assert(connected_channel == 6); assert(connected_psk == psk);
}
static void saved_fast_connect(void) {
    kv_present = 1; kv_data.channel = 9; kv_data.bssid[0] = 0x02; kv_data.bssid[5] = 42;
    HAL_FastConnectToWiFi("target", "password", &ip);
    assert(scan_calls == 0); assert(connected_with_bssid); assert(connected_bssid[5] == 42); assert(connected_channel == 9);
}
static void no_saved_fast_connect(void) { prepare_pair(); HAL_FastConnectToWiFi("target", "password", &ip); assert_best(2, 11); }
static void malformed_saved_fast_connect(void) { prepare_pair(); kv_present = 1; kv_length = 1; HAL_FastConnectToWiFi("target", "password", &ip); assert_best(2, 11); }
static void selected_failure_falls_back(void) {
    prepare_pair(); connect_normal("target"); assert_best(2, 11);
    wifi_sta_connect_failed_reason_t reason = WIFI_STA_CONN_TARGET_AP_NOT_FOUND;
    sta_status = WIFI_STA_STATUS_DISCONNECTED;
    callbacks[WIFI_MGR_EVENT_STA_CONNECT_FAILED](&reason);
    connect_normal("target"); assert_unpinned(); assert(scan_calls == 2);
}
static void success_then_reconnect(void) {
    prepare_pair(); connect_normal("target"); assert_best(2, 11);
    sta_status = WIFI_STA_STATUS_CONNECTED; callbacks[WIFI_MGR_EVENT_STA_CONNECTED](NULL);
    HAL_DisconnectFromWifi();
    scan_data[2][0] = scan_data[3][0] = ap("target", 4, -50, 4);
    scan_sizes[2] = scan_sizes[3] = 1; connect_normal("target"); assert_best(4, 4); assert(create_calls == 1);
}
static void long_ssid(void) {
    const char *ssid = "12345678901234567890123456789012";
    scan_data[0][0] = scan_data[1][0] = ap(ssid, 3, -50, 3); scan_sizes[0] = scan_sizes[1] = 1;
    connect_normal(ssid); assert_best(3, 3);
}
static void invalid_candidates(void) {
    for (int pass = 0; pass < 2; ++pass) {
        scan_data[pass][0] = ap("target", 1, -20, 0);
        scan_data[pass][1] = ap("target", 2, -20, 15);
        scan_data[pass][2] = ap("target", 3, -20, 1); scan_data[pass][2].bssid[0] = 0x83;
        scan_data[pass][3] = ap("target", 4, -20, 1); memset(scan_data[pass][3].bssid, 0, 6);
        scan_sizes[pass] = 4;
    }
    connect_normal("target"); assert_unpinned();
}
static void list_failure_restores_updates(void) { prepare_pair(); fail_list = 1; connect_normal("target"); assert_unpinned(); assert(updates_enabled); }
static void already_scanning(void) { sta_status = WIFI_STA_STATUS_SCANING; connect_normal("target"); assert(scan_calls == 0); assert_unpinned(); }
static void selected_storage_is_stable(void) {
    strongest(); uint8_t before[6]; memcpy(before, connected_bssid_pointer, 6);
    callbacks[WIFI_MGR_EVENT_STA_SCAN_COMPLETE](NULL);
    assert(memcmp(before, connected_bssid_pointer, 6) == 0); assert(delete_calls == 0);
}
static void skipped_attempt_keeps_selected_storage(void) {
    strongest();
    uint8_t *original = connected_bssid_pointer, before[6];
    memcpy(before, original, 6);
    /* No success callback: the second attempt uses the unpinned fallback.
     * A third attempt while the SDK is still connecting must not clear the
     * storage that was handed to the asynchronous SDK on the first attempt.
     */
    connect_normal("target");
    connect_normal("target");
    assert(memcmp(original, before, 6) == 0);
}
static void ap_list_access_is_serialized(void) {
    check_list_lock = true; strongest();
    callbacks[WIFI_MGR_EVENT_STA_SCAN_COMPLETE](NULL);
    assert(mutex_depth == 0);
}
static void mutex_allocation_failed(void) {
    fail_mutex_create = 1; connect_normal("target"); assert_unpinned(); assert(scan_calls == 0);
}
static void pending_association_does_not_rescan(void) {
    strongest();
    connect_normal("target"); /* consume the unpinned retry */
    sta_status = WIFI_STA_STATUS_DISCONNECTED;
    /* The SDK has not delivered the terminal connection callback yet. */
    connect_normal("target");
    assert(scan_calls == 2);
}
static void rejected_retry_keeps_prior_association_pending(void) {
    strongest();
    reject_connection = 1;
    connect_normal("target"); /* Rejected while the previous request still owns callbacks. */
    sta_status = WIFI_STA_STATUS_DISCONNECTED;
    connect_normal("target");
    assert(scan_calls == 2); /* No terminal callback: do not rearm a managed scan. */
}
static void rejected_retry_does_not_restore_completed_pending_state(void) {
    strongest();
    reject_connection = 2; /* A terminal SDK callback occurs within the rejected call. */
    connect_normal("target");
    assert(__atomic_load_n(&bssid_association_pending, __ATOMIC_ACQUIRE) == 0);
}
static void nonterminated_ssid_is_not_read_past_limit(void) {
    char ssid[SSID_MAX_LEN]; ln_best_bssid_t candidate;
    memset(ssid, 'x', sizeof(ssid)); ln_bssid_init();
    assert(ln_bssid_select(ssid, &candidate) == LN_BSSID_SKIPPED);
    assert(!candidate.found); assert(scan_calls == 0);
}
static void *notify_scan_thread(void *unused) {
    (void)unused; ln_bssid_scan_complete(); return NULL;
}
static void callback_paused_during_timeout(void) {
    pthread_t thread; ln_best_bssid_t candidate;
    ln_bssid_init();
    __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_WAITING, __ATOMIC_RELEASE);
    __atomic_store_n(&release_gate, 1, __ATOMIC_RELEASE);
    assert(pthread_create(&thread, NULL, notify_scan_thread, NULL) == 0);
    while (__atomic_load_n(&release_gate, __ATOMIC_ACQUIRE) != 2) sched_yield();
    /* Cancellation while the actual callback is inside SemaphoreRelease. */
    __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_DISABLED, __ATOMIC_RELEASE);
    __atomic_store_n(&release_gate, 3, __ATOMIC_RELEASE);
    assert(pthread_join(thread, NULL) == 0);
    assert(ln_bssid_scan_sem.valid); assert(delete_calls == 0);
    assert(ln_bssid_select("target", &candidate) == LN_BSSID_SKIPPED);
    assert(scan_calls == 0); assert(!candidate.found);
}
static void threaded_completion_timeout_interleavings(void) {
    pthread_t thread; ln_bssid_init();
    for (int i = 0; i < 2000; ++i) {
        ln_bssid_scan_sem.count = 0;
        __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_WAITING, __ATOMIC_RELEASE);
        assert(pthread_create(&thread, NULL, notify_scan_thread, NULL) == 0);
        if (i & 1) sched_yield();
        __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_DISABLED, __ATOMIC_RELEASE);
        assert(pthread_join(thread, NULL) == 0);
        assert(__atomic_load_n(&ln_bssid_scan_state, __ATOMIC_ACQUIRE) == LN_BSSID_DISABLED);
        assert(ln_bssid_scan_sem.valid); assert(delete_calls == 0);
    }
}
static void duplicate_completion_is_ignored(void) { duplicate_completion = true; strongest(); }
static void immediate_connection_rejection(void) {
    reject_connection = 1; strongest(); assert(last_event == WIFI_STA_DISCONNECTED);
    reject_connection = 0; sta_status = WIFI_STA_STATUS_DISCONNECTED;
    connect_normal("target"); assert_unpinned(); assert(scan_calls == 2);
}
static void failed_status_query(void) { fail_status = 1; connect_normal("target"); assert_unpinned(); assert(scan_calls == 0); }
static void shuffled_rssi_selection(void) {
    uint32_t rng = 0x2019; ln_bssid_init();
    for (unsigned round = 0; round < 2000; ++round) {
        ln_best_bssid_t result; int highest = -129; uint8_t expected_id = 0;
        wifi_manager_cleanup_scan_results();
        for (unsigned i = 0; i < MAX_APS; ++i) {
            rng = rng * 1664525u + 1013904223u;
            int rssi = -1 - (int)(rng % 127);
            bool match = (rng & 0x100) != 0;
            add_cached_ap(ap(match ? "target" : "other", (uint8_t)(i + 1), rssi, 1));
            if (match && rssi > highest) { highest = rssi; expected_id = (uint8_t)(i + 1); }
        }
        assert(ln_bssid_read_candidates("target", 6, &result));
        assert(result.found == (expected_id != 0));
        if (result.found) { assert(result.bssid[5] == expected_id); assert(result.rssi == highest); }
    }
}
static void static_ip(void) { ip.localIPAddr[0] = 192; strongest(); assert(g_STA_static_IP); }
static void open_network(void) { prepare_pair(); HAL_ConnectToWiFi("target", "", &ip); assert_best(2, 11); assert(connected_psk == NULL); }

static const struct { const char *name; void (*run)(void); } tests[] = {
    {"strongest", strongest}, {"no_match", no_match}, {"high_bytes", high_bytes},
    {"fresh_second_scan", fresh_second_scan}, {"cache_is_cleared", cache_is_cleared},
    {"second_empty", second_empty}, {"start_failed", start_failed}, {"allocation_failed", allocation_failed},
    {"scan_rejected", scan_rejected}, {"second_rejected", second_rejected},
    {"timeout_and_late_event", timeout_and_late_event}, {"second_timeout", second_timeout},
    {"release_failed", release_failed}, {"synchronous_completion", synchronous_completion},
    {"fast_connect_preserved", fast_connect_preserved}, {"saved_fast_connect", saved_fast_connect},
    {"no_saved_fast_connect", no_saved_fast_connect}, {"malformed_saved_fast_connect", malformed_saved_fast_connect},
    {"selected_failure_falls_back", selected_failure_falls_back}, {"success_then_reconnect", success_then_reconnect},
    {"long_ssid", long_ssid}, {"invalid_candidates", invalid_candidates},
    {"list_failure_restores_updates", list_failure_restores_updates}, {"already_scanning", already_scanning},
    {"selected_storage_is_stable", selected_storage_is_stable}, {"skipped_attempt_keeps_selected_storage", skipped_attempt_keeps_selected_storage}, {"ap_list_access_is_serialized", ap_list_access_is_serialized}, {"mutex_allocation_failed", mutex_allocation_failed}, {"pending_association_does_not_rescan", pending_association_does_not_rescan}, {"nonterminated_ssid_is_not_read_past_limit", nonterminated_ssid_is_not_read_past_limit}, {"callback_paused_during_timeout", callback_paused_during_timeout},
    {"threaded_completion_timeout_interleavings", threaded_completion_timeout_interleavings},
    {"duplicate_completion_is_ignored", duplicate_completion_is_ignored},
    {"immediate_connection_rejection", immediate_connection_rejection},
    {"rejected_retry_keeps_prior_association_pending", rejected_retry_keeps_prior_association_pending},
    {"rejected_retry_does_not_restore_completed_pending_state", rejected_retry_does_not_restore_completed_pending_state},
    {"failed_status_query", failed_status_query}, {"shuffled_rssi_selection", shuffled_rssi_selection},
    {"static_ip", static_ip}, {"open_network", open_network}
};
int main(int argc, char **argv) {
    ap_list.next = ap_list.prev = &ap_list;
    HAL_WiFi_SetupStatusCallback(status_callback);
    if (argc != 2) { for (size_t i = 0; i < sizeof(tests)/sizeof(tests[0]); ++i) puts(tests[i].name); return 0; }
    for (size_t i = 0; i < sizeof(tests)/sizeof(tests[0]); ++i) {
        if (strcmp(argv[1], tests[i].name) == 0) {
            tests[i].run(); free(psk_value); printf("PASS %s\n", tests[i].name); return 0;
        }
    }
    fprintf(stderr, "Unknown test: %s\n", argv[1]); return 2;
}
