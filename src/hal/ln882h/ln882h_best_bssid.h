#ifndef OBK_LN882H_BEST_BSSID_H
#define OBK_LN882H_BEST_BSSID_H

/* Private to hal_wifi_ln882h.c. Keep this out of the LN8825/other HALs.
 * Wi-Fi connections are initiated by the application task; scan completion
 * arrives on the SDK's task. Only completion notification crosses that boundary.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t bssid[BSSID_LEN];
    uint8_t channel;
    int8_t rssi;
    bool found;
} ln_best_bssid_t;

typedef enum {
    LN_BSSID_FOUND,
    LN_BSSID_NOT_FOUND,
    LN_BSSID_SKIPPED,
    LN_BSSID_SCAN_FAILED
} ln_bssid_result_t;

enum {
    LN_BSSID_IDLE,
    LN_BSSID_WAITING,
    LN_BSSID_COMPLETE,
    LN_BSSID_DISABLED
};

/* Never destroy this semaphore while the SDK may still deliver a callback.
 * The SDK does not identify scan generations or offer a documented cancellation
 * barrier. After an error/timeout, disable this optional pre-scan until reboot;
 * ordinary SSID association and its retries remain available. A late completion
 * therefore cannot wake a later pre-scan or access a freed semaphore.
 */
static OS_Semaphore_t ln_bssid_scan_sem;
static bool ln_bssid_initialized;
static OS_Mutex_t ln_bssid_list_mutex;
static bool ln_bssid_mutex_created;
static unsigned int ln_bssid_scan_state = LN_BSSID_IDLE;
static unsigned int ln_bssid_scan_busy;

/* Initialize before installing the callback, once on the application task.
 * Retain both objects for the lifetime of the Wi-Fi HAL, including on failure.
 */
static void ln_bssid_init(void)
{
    if (ln_bssid_initialized) {
        return;
    }
    ln_bssid_initialized = true;
    if (OS_MutexCreate(&ln_bssid_list_mutex) != OS_OK) {
        __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_DISABLED, __ATOMIC_RELEASE);
        return;
    }
    ln_bssid_mutex_created = true;
    if (OS_SemaphoreCreate(&ln_bssid_scan_sem, 0, 1) != OS_OK) {
        __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_DISABLED, __ATOMIC_RELEASE);
    }
}

/* The SDK's list getter releases its internal lock before returning a pointer.
 * Serialize our normal logging callback with cache clearing/snapshotting too.
 * Without the optional scan resources, retain the pre-existing logging path;
 * in that case no new code clears or traverses this list concurrently.
 */
static bool ln_bssid_list_acquire(void)
{
    return !ln_bssid_mutex_created ||
        OS_MutexLock(&ln_bssid_list_mutex, OS_WAIT_FOREVER) == OS_OK;
}

static void ln_bssid_list_release(void)
{
    if (ln_bssid_mutex_created) {
        OS_MutexUnlock(&ln_bssid_list_mutex);
    }
}

/* Called by the permanent scan-complete callback. Do not select an AP here. */
static bool ln_bssid_scan_complete(void)
{
    unsigned int expected = LN_BSSID_WAITING;
    if (__atomic_compare_exchange_n(&ln_bssid_scan_state, &expected,
            LN_BSSID_COMPLETE, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        if (OS_SemaphoreRelease(&ln_bssid_scan_sem) != OS_OK) {
            __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_DISABLED, __ATOMIC_RELEASE);
        }
        return true;
    }
    return expected == LN_BSSID_COMPLETE;
}

static bool ln_bssid_candidate_valid(const ap_info_t *ap, const char *ssid, size_t len)
{
    uint8_t any = 0;
    if (ap->ssid[len] != '\0' || memcmp(ap->ssid, ssid, len) != 0 ||
            ap->channel < 1 || ap->channel > 14 || (ap->bssid[0] & 1)) {
        return false;
    }
    for (size_t i = 0; i < BSSID_LEN; ++i) {
        any |= ap->bssid[i];
    }
    return any != 0;
}

/* Take a snapshot from a completed scan, with AP-list updates suspended. */
static bool ln_bssid_read_candidates(const char *ssid, size_t len, ln_best_bssid_t *out)
{
    ln_list_t *list = NULL;
    uint8_t count = 0;
    bool ok;
    memset(out, 0, sizeof(*out));
    if (!ln_bssid_list_acquire()) {
        return false;
    }
    if (wifi_manager_ap_list_update_enable(LN_FALSE) != WIFI_ERR_NONE) {
        ln_bssid_list_release();
        return false;
    }
    ok = wifi_manager_get_ap_list(&list, &count) == WIFI_ERR_NONE && list != NULL;
    if (ok) {
        ln_list_t *entry;
        for (entry = list->next; entry != list; entry = entry->next) {
            const ap_info_t *ap = &LN_LIST_ENTRY(entry, ap_info_node_t, list)->info;
            if (ln_bssid_candidate_valid(ap, ssid, len) &&
                    (!out->found || ap->rssi > out->rssi)) {
                memcpy(out->bssid, ap->bssid, sizeof(out->bssid));
                out->channel = ap->channel;
                out->rssi = ap->rssi;
                out->found = true;
            }
        }
    }
    if (wifi_manager_ap_list_update_enable(LN_TRUE) != WIFI_ERR_NONE) {
        ok = false;
    }
    ln_bssid_list_release();
    return ok;
}

static ln_bssid_result_t ln_bssid_select(const char *ssid, ln_best_bssid_t *out)
{
    const unsigned int scans = 2;
    const OS_Time_t timeout_ms = 2000;
    wifi_scan_cfg_t scan = { .channel = 0, .scan_type = WIFI_SCAN_TYPE_ACTIVE, .scan_time = 30 };
    wifi_sta_status_t status;
    ln_bssid_result_t result = LN_BSSID_SKIPPED;
    unsigned int expected = 0;
    size_t len = 0;

    memset(out, 0, sizeof(*out));
    if (ssid == NULL) {
        return result;
    }
    while (len < SSID_MAX_LEN && ssid[len] != '\0') {
        ++len;
    }
    if (len == 0 || len >= SSID_MAX_LEN) {
        return result;
    }
    if (!__atomic_compare_exchange_n(&ln_bssid_scan_busy, &expected, 1, false,
            __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        return result;
    }
    if (__atomic_load_n(&ln_bssid_scan_state, __ATOMIC_ACQUIRE) == LN_BSSID_DISABLED) {
        goto done;
    }
    /* Do not overlap a scan/association already owned by the SDK. */
    if (wifi_get_sta_status(&status) != WIFI_ERR_NONE ||
            (status != WIFI_STA_STATUS_STARTUP && status != WIFI_STA_STATUS_DISCONNECTED)) {
        goto done;
    }
    for (unsigned int i = 0; i < scans; ++i) {
        /* This is a 60-second SDK cache, not a per-scan result list. Clear it
         * before EACH pass. Use only the last fully completed pass: an AP seen
         * in the first pass may have disappeared or changed channels by now.
         */
        if (!ln_bssid_list_acquire()) {
            goto failed;
        }
        wifi_manager_cleanup_scan_results();
        ln_bssid_list_release();
        __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_WAITING, __ATOMIC_RELEASE);
        if (wifi_sta_scan(&scan) != WIFI_ERR_NONE) {
            goto failed;
        }
        if (OS_SemaphoreWait(&ln_bssid_scan_sem, timeout_ms) != OS_OK ||
                __atomic_load_n(&ln_bssid_scan_state, __ATOMIC_ACQUIRE) != LN_BSSID_COMPLETE) {
            goto failed;
        }
        __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_IDLE, __ATOMIC_RELEASE);
        if (!ln_bssid_read_candidates(ssid, len, out)) {
            goto failed;
        }
    }
    result = out->found ? LN_BSSID_FOUND : LN_BSSID_NOT_FOUND;
    goto done;

failed:
    __atomic_store_n(&ln_bssid_scan_state, LN_BSSID_DISABLED, __ATOMIC_RELEASE);
    memset(out, 0, sizeof(*out));
    result = LN_BSSID_SCAN_FAILED;
done:
    __atomic_store_n(&ln_bssid_scan_busy, 0, __ATOMIC_RELEASE);
    return result;
}

#endif
