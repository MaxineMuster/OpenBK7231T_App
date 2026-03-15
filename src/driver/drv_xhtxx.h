#pragma once
#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------
// drv_xhtxx.h  –  unified I²C temperature/humidity driver
//
// Supported families
//   XHTXX_FAMILY_SHT3X  – SHT30 / SHT31 / SHT35    (0x44 or 0x45)
//   XHTXX_FAMILY_SHT4X  – SHT40 / SHT41 / SHT45    (0x44)
//   XHTXX_FAMILY_AHT2X  – AHT20 / AHT21 / AHT25    (0x38)
//   XHTXX_FAMILY_CHT83XX– CHT8305 / CHT8310 / 8315  (0x40)
//
// Auto-detect probe order (used when family= is omitted or 0):
//   SHT4x @ 0x44 → SHT3x @ 0x44 → SHT3x @ 0x45 → AHT2x @ 0x38 → CHT83xx @ 0x40
//
// startDriver syntax:
//   startDriver XHTXX [SDA=<pin>] [SCL=<pin>]
//                     [family=sht3|sht4|aht2|cht]   ; omit for auto-detect
//                     [address=<hex>]                ; override I²C addr
//                     [chan_t=<ch>] [chan_h=<ch>]
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Feature gates
//
// XHTXX_ENABLE_SERIAL_LOG
//   Stores the SHT serial number in dev->serial and prints it in
//   ListSensors.  4 B RAM per sensor, ~50 B extra flash.
//   Auto-detect and SHT3x/SHT4x discrimination work without this flag
//   because probe_fn always runs the command-response CRC check; only
//   the storage and display of the number are gated here.
//
// XHTXX_ENABLE_SHT3_EXTENDED_FEATURES
//   SHT3x-only: heater, status register, alert limits, periodic capture.
//   Pulls in float for alert bit-math (hot path stays integer-only).
// -----------------------------------------------------------------------
//#define XHTXX_ENABLE_SERIAL_LOG
//#define XHTXX_ENABLE_SHT3_EXTENDED_FEATURES

// -----------------------------------------------------------------------
// Family indices – used as index into g_xhtxx_families[]
// -----------------------------------------------------------------------
#define XHTXX_FAMILY_AUTO    0   // resolve at init (auto-detect)
#define XHTXX_FAMILY_SHT3X   1
#define XHTXX_FAMILY_SHT4X   2
#define XHTXX_FAMILY_AHT2X   3
#define XHTXX_FAMILY_CHT83XX 4
#define XHTXX_FAMILY_COUNT   5   // total (including AUTO sentinel)

// -----------------------------------------------------------------------
// Limits
// -----------------------------------------------------------------------
#define XHTXX_MAX_SENSORS   15

// Default I²C addresses (pre-shifted for Soft_I2C_Start, i.e. addr<<1)
#define XHTXX_ADDR_SHT      (0x44 << 1)
#define XHTXX_ADDR_AHT2X    (0x38 << 1)
#define XHTXX_ADDR_CHT83XX  (0x40 << 1)

// -----------------------------------------------------------------------
// Family-specific config – stored in flash, one entry per family.
//
// probe_fn   : try to talk to the sensor; return true if it responds.
//              Allowed to leave the sensor in a known-good state on
//              success.  Must be non-destructive on failure.
// init_fn    : full initialisation; called once after probe succeeds.
//              Sets dev->isWorking.
// measure_fn : one-shot measurement; updates dev->lastTemp/lastHumid
//              and calls CHANNEL_Set when channels are assigned.
// reset_fn   : soft-reset (called by StopDriver / Reinit).
// name       : human-readable family name (e.g. "AHT2x").
// defaultAddr: pre-shifted 8-bit address.
// -----------------------------------------------------------------------
typedef struct xhtxx_dev_s xhtxx_dev_t;   // forward declaration

typedef struct
{
    bool     (*probe_fn)  (xhtxx_dev_t *dev);
    void     (*init_fn)   (xhtxx_dev_t *dev);
    void     (*measure_fn)(xhtxx_dev_t *dev);
    void     (*reset_fn)  (xhtxx_dev_t *dev);
    const char *name;
    uint8_t   defaultAddr;
} xhtxx_family_t;

// -----------------------------------------------------------------------
// Per-instance state
//
// lastTemp  : 0.1 °C   (e.g. 225 = 22.5 °C)
// lastHumid : 0.1 %RH  (e.g. 456 = 45.6 %RH)
// calTemp   : 0.1 °C calibration offset
// calHum    : 0.1 %RH calibration offset
// familyIdx : one of XHTXX_FAMILY_* (never AUTO after init)
// subtype   : family-specific variant id (CHT sensor_id, future use)
// -----------------------------------------------------------------------
struct xhtxx_dev_s
{
    softI2C_t  i2c;
    int16_t    calTemp;         // 0.1 °C
    int8_t     calHum;          // 0.1 %RH
    int8_t     channel_temp;    // -1 = unused
    int8_t     channel_humid;   // -1 = unused
    uint8_t    i2cAddr;         // pre-shifted
    uint8_t    familyIdx;       // XHTXX_FAMILY_*
    uint16_t   subtype;         // e.g. CHT sensor_id word
    uint8_t    secondsBetween;
    uint8_t    secondsUntilNext;
    bool       isWorking;
    int16_t    lastTemp;        // 0.1 °C
    int16_t    lastHumid;       // 0.1 %RH
#ifdef XHTXX_ENABLE_SERIAL_LOG
    uint32_t   serial;          // SHT serial number (0 for AHT/CHT)
#endif
#ifdef XHTXX_ENABLE_SHT3_EXTENDED_FEATURES
    bool       periodicActive;
#endif
};

// -----------------------------------------------------------------------
// Public API  (matches the existing SHTxx driver entry points)
// -----------------------------------------------------------------------
void XHTXX_Init(void);
void XHTXX_StopDriver(void);
void XHTXX_OnEverySecond(void);
void XHTXX_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);
