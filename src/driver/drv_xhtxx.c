// drv_xhtxx.c  –  unified I²C temperature/humidity driver
//
// Combines SHT3x, SHT4x (Sensirion), AHT2x (Aosong), and
// CHT8305/8310/8315 (Sensylink) into one driver with a shared
// per-sensor state array and a flash-resident family dispatch table.
//
// All measurement arithmetic on the hot path uses integer math only.
// Float is used only in the SHT3x alert commands (user-command path,
// compiled only when XHTXX_ENABLE_SHT3_EXTENDED_FEATURES is defined).

#include "../new_pins.h"
#include "../new_cfg.h"
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_uart.h"
#include "../httpserver/new_http.h"
#include "../hal/hal_pins.h"
#include "drv_xhtxx.h"
#include "../obk_config.h"

#if ENABLE_DRIVER_XHTXX

// -----------------------------------------------------------------------
// Module-level sensor registry
// -----------------------------------------------------------------------
static xhtxx_dev_t g_sensors[XHTXX_MAX_SENSORS];
static uint8_t     g_numSensors = 0;

// -----------------------------------------------------------------------
// Forward-declare family dispatch functions so the table can be defined
// before their bodies.
// -----------------------------------------------------------------------
static bool XHTXX_SHT3x_Probe  (xhtxx_dev_t *dev);
static void XHTXX_SHT3x_Init   (xhtxx_dev_t *dev);
static void XHTXX_SHT3x_Measure(xhtxx_dev_t *dev);
static void XHTXX_SHT3x_Reset  (xhtxx_dev_t *dev);

static bool XHTXX_SHT4x_Probe  (xhtxx_dev_t *dev);
static void XHTXX_SHT4x_Init   (xhtxx_dev_t *dev);
static void XHTXX_SHT4x_Measure(xhtxx_dev_t *dev);
static void XHTXX_SHT4x_Reset  (xhtxx_dev_t *dev);

static bool XHTXX_AHT2x_Probe  (xhtxx_dev_t *dev);
static void XHTXX_AHT2x_Init   (xhtxx_dev_t *dev);
static void XHTXX_AHT2x_Measure(xhtxx_dev_t *dev);
static void XHTXX_AHT2x_Reset  (xhtxx_dev_t *dev);

static bool XHTXX_CHT83xx_Probe  (xhtxx_dev_t *dev);
static void XHTXX_CHT83xx_Init   (xhtxx_dev_t *dev);
static void XHTXX_CHT83xx_Measure(xhtxx_dev_t *dev);
static void XHTXX_CHT83xx_Reset  (xhtxx_dev_t *dev);

// -----------------------------------------------------------------------
// Family dispatch table – stored in flash (const).
// Index 0 (XHTXX_FAMILY_AUTO) is a sentinel; entries start at index 1.
// -----------------------------------------------------------------------
static const xhtxx_family_t g_families[XHTXX_FAMILY_COUNT] =
{
    [XHTXX_FAMILY_AUTO]    = { NULL,                NULL,               NULL,                 NULL,                 "auto",    0                },
    [XHTXX_FAMILY_SHT3X]   = { XHTXX_SHT3x_Probe,  XHTXX_SHT3x_Init,  XHTXX_SHT3x_Measure,  XHTXX_SHT3x_Reset,  "SHT3x",   XHTXX_ADDR_SHT   },
    [XHTXX_FAMILY_SHT4X]   = { XHTXX_SHT4x_Probe,  XHTXX_SHT4x_Init,  XHTXX_SHT4x_Measure,  XHTXX_SHT4x_Reset,  "SHT4x",   XHTXX_ADDR_SHT   },
    [XHTXX_FAMILY_AHT2X]   = { XHTXX_AHT2x_Probe,  XHTXX_AHT2x_Init,  XHTXX_AHT2x_Measure,  XHTXX_AHT2x_Reset,  "AHT2x",   XHTXX_ADDR_AHT2X },
    [XHTXX_FAMILY_CHT83XX] = { XHTXX_CHT83xx_Probe, XHTXX_CHT83xx_Init, XHTXX_CHT83xx_Measure, XHTXX_CHT83xx_Reset, "CHT83xx", XHTXX_ADDR_CHT83XX },
};

// -----------------------------------------------------------------------
// Ordered probe list for auto-detection.
// SHT4x is tried first: its serial command (0x89) is a 1-byte write that
// a SHT3x ignores or NAKs, so the CRC check unambiguously tells them apart.
// SHT3x secondary address (0x45) is tried explicitly after the primary.
// AHT2x and CHT83xx have unique addresses so their order is unimportant.
// -----------------------------------------------------------------------
static const struct { uint8_t family; uint8_t addr; } g_probeOrder[] =
{
    { XHTXX_FAMILY_SHT4X,   XHTXX_ADDR_SHT      },
    { XHTXX_FAMILY_SHT3X,   XHTXX_ADDR_SHT      },
    { XHTXX_FAMILY_SHT3X,   (0x45 << 1)         },
    { XHTXX_FAMILY_AHT2X,   XHTXX_ADDR_AHT2X    },
    { XHTXX_FAMILY_CHT83XX, XHTXX_ADDR_CHT83XX  },
};
#define XHTXX_PROBE_STEPS  (sizeof(g_probeOrder)/sizeof(g_probeOrder[0]))

// =======================================================================
// -----------------------------------------------------------------------
//  Generic I²C primitives
//
//  Three building blocks cover every access pattern in this driver:
//
//  I2C_Write(dev, b0, b1, n)
//    Start → write n bytes (b0, then b1 if n≥2) → Stop
//    Used for: single-byte commands, 2-byte SHT commands, AHT commands,
//              register-pointer-only writes (n=1), CHT config writes.
//
//  I2C_Read(dev, buf, n)
//    Start(read) → read n bytes into buf → Stop
//    Used for: SHT data reads after a command, AHT measurement reads,
//              AHT single-byte status poll.
//
//  I2C_ReadReg(dev, reg, buf, n)
//    Start → write reg → Stop → Start(read) → read n bytes → Stop
//    Used for: CHT register reads.
//
//  Everything else (SHT CRC logic, AHT busy-poll, SHT alert CRC write)
//  is genuine protocol logic that lives in the family functions.
// -----------------------------------------------------------------------

static void I2C_Write(xhtxx_dev_t *dev, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t n)
{
    Soft_I2C_Start(&dev->i2c, dev->i2cAddr);
    Soft_I2C_WriteByte(&dev->i2c, b0);
    if(n >= 2) Soft_I2C_WriteByte(&dev->i2c, b1);
    if(n >= 3) Soft_I2C_WriteByte(&dev->i2c, b2);
    Soft_I2C_Stop(&dev->i2c);
}

static void I2C_Read(xhtxx_dev_t *dev, uint8_t *buf, uint8_t n)
{
    Soft_I2C_Start(&dev->i2c, dev->i2cAddr | 1);
    Soft_I2C_ReadBytes(&dev->i2c, buf, n);
    Soft_I2C_Stop(&dev->i2c);
}

static void I2C_ReadReg(xhtxx_dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t n)
{
    I2C_Write(dev, reg, 0, 0, 1);
    I2C_Read(dev, buf, n);
}

// =======================================================================
// -----------------------------------------------------------------------
//  Shared measurement helpers
// -----------------------------------------------------------------------

// Store 0.1-unit values, update channels, log.
static void XHTXX_StoreAndLog(xhtxx_dev_t *dev, int16_t t10, int16_t h10)
{
    if(h10 <    0) h10 =    0;
    if(h10 > 1000) h10 = 1000;
    dev->lastTemp  = t10;
    dev->lastHumid = h10;
    if(dev->channel_temp  >= 0) CHANNEL_Set(dev->channel_temp,  t10, 0);
    if(dev->channel_humid >= 0) CHANNEL_Set(dev->channel_humid, h10, 0);
    int16_t tf = t10 % 10; if(tf < 0) tf = -tf;
    ADDLOG_INFO(LOG_FEATURE_SENSOR,
                "XHTXX %s (SDA=%i): T=%d.%d°C H=%d.%d%%",
                g_families[dev->familyIdx].name,
                dev->i2c.pin_data,
                t10/10, tf, h10/10, h10%10);
}

// =======================================================================
// -----------------------------------------------------------------------
//  SHT shared helpers (CRC and command+read)
// -----------------------------------------------------------------------

// CRC-8  poly=0x31, init=0xFF  (Sensirion spec)
static bool XHTXX_SHT_CRC8(const uint8_t *data, uint8_t expected)
{
    uint8_t crc = 0xFF ^ data[0];
    for(uint8_t b = 0; b < 8; b++)
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    crc ^= data[1];
    for(uint8_t b = 0; b < 8; b++)
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    return crc == expected;
}

// Send a 1- or 2-byte SHT command, wait delay_ms, read 6 bytes, verify CRCs.
static bool XHTXX_SHT_CmdRead6(xhtxx_dev_t *dev,
                                uint8_t b0, uint8_t b1, uint8_t cmdlen,
                                uint8_t delay_ms, uint8_t *out,
                                const char* func)
{
    I2C_Write(dev, b0, b1, 0, cmdlen);
    if(delay_ms) rtos_delay_milliseconds(delay_ms);
    I2C_Read(dev, out, 6);
    if(!XHTXX_SHT_CRC8(&out[0], out[2]) || !XHTXX_SHT_CRC8(&out[3], out[5]))
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "%s: CRC mismatch (SDA=%i addr=0x%02X)",func, dev->i2c.pin_data,dev->i2cAddr >> 1);
        return false;
    }
    return true;
}

// =======================================================================
// -----------------------------------------------------------------------
//  SHT3x  family
// -----------------------------------------------------------------------
// T  = -45 + 175 * raw / 65535    → ×10: (1750*raw + 32767)/65535 - 450
// RH = 100 * raw / 65535          → ×10: (1000*raw + 32767)/65535
// Integer error < 0.02°C / 0.05%RH
// -----------------------------------------------------------------------

static void XHTXX_SHT3x_ConvertStore(xhtxx_dev_t *dev, const uint8_t *d)
{
    uint16_t raw_t = ((uint16_t)d[0] << 8) | d[1];
    uint16_t raw_h = ((uint16_t)d[3] << 8) | d[4];
    int16_t t10 = (int16_t)((1750u * raw_t + 32767u) / 65535u) - 450 + dev->calTemp;
    int16_t h10 = (int16_t)((1000u * (uint32_t)raw_h + 32767u) / 65535u) + dev->calHum;
    XHTXX_StoreAndLog(dev, t10, h10);
}

// Probe via serial-number command: faster than a measurement (no conversion
// wait) and distinguishes SHT3x from SHT4x – SHT4x NAKs or fails CRC here.
static bool XHTXX_SHT3x_Probe(xhtxx_dev_t *dev)
{
    uint8_t d[6];
    if(!XHTXX_SHT_CmdRead6(dev, 0x36, 0x82, 2, 2, d,"SHT3x_Probe")) return false;
#ifdef XHTXX_ENABLE_SERIAL_LOG
    dev->serial = ((uint32_t)d[0]<<24)|((uint32_t)d[1]<<16)|((uint32_t)d[3]<<8)|d[4];
#endif
    return true;
}

static void XHTXX_SHT3x_Init(xhtxx_dev_t *dev)
{
#ifdef XHTXX_ENABLE_SERIAL_LOG
    if(!dev->serial) XHTXX_SHT3x_Probe(dev);   // skipped when probe already ran
#endif
    I2C_Write(dev, 0x30, 0xA2, 0, 2);             // soft-reset
    rtos_delay_milliseconds(2);
    uint8_t d[6];
    dev->isWorking = XHTXX_SHT_CmdRead6(dev, 0x24, 0x00, 2, 15, d,"SHT3x_Init");
    if(dev->isWorking) XHTXX_SHT3x_ConvertStore(dev, d);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX SHT3x init %s (SDA=%i addr=0x%02X)",
                dev->isWorking ? "ok" : "FAILED", dev->i2c.pin_data, dev->i2cAddr>>1);
}

static void XHTXX_SHT3x_Measure(xhtxx_dev_t *dev)
{
    uint8_t d[6];
    if(!XHTXX_SHT_CmdRead6(dev, 0x24, 0x00, 2, 15, d,"SHT3x_Measure"))
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX SHT3x measure failed (SDA=%i)", dev->i2c.pin_data);
        return;
    }
    XHTXX_SHT3x_ConvertStore(dev, d);
}

static void XHTXX_SHT3x_Reset(xhtxx_dev_t *dev)
{
    I2C_Write(dev, 0x30, 0xA2, 0, 2);
    rtos_delay_milliseconds(2);
}

// =======================================================================
// -----------------------------------------------------------------------
//  SHT4x  family
// -----------------------------------------------------------------------
// T  = -45 + 175 * raw / 65535   (same as SHT3x)
// RH = -6  + 125 * raw / 65535
//   → ×10: (1250*raw + 32767)/65535 - 60
//   max intermediate: 1250*65535 = 81,918,750  fits uint32_t ✓
// -----------------------------------------------------------------------

static void XHTXX_SHT4x_ConvertStore(xhtxx_dev_t *dev, const uint8_t *d)
{
    uint16_t raw_t = ((uint16_t)d[0] << 8) | d[1];
    uint16_t raw_h = ((uint16_t)d[3] << 8) | d[4];
    int16_t t10 = (int16_t)((1750u * raw_t + 32767u) / 65535u) - 450 + dev->calTemp;
    int16_t h10 = (int16_t)((1250u * (uint32_t)raw_h + 32767u) / 65535u) - 60 + dev->calHum;
    XHTXX_StoreAndLog(dev, t10, h10);
}

// Probe via serial command 0x89: SHT3x will NAK or fail CRC.
static bool XHTXX_SHT4x_Probe(xhtxx_dev_t *dev)
{
    uint8_t d[6];
    if(!XHTXX_SHT_CmdRead6(dev, 0x89, 0x00, 1, 2, d,"SHT4x_Probe")) return false;
#ifdef XHTXX_ENABLE_SERIAL_LOG
    dev->serial = ((uint32_t)d[0]<<24)|((uint32_t)d[1]<<16)|((uint32_t)d[3]<<8)|d[4];
#endif
    return true;
}

static void XHTXX_SHT4x_Init(xhtxx_dev_t *dev)
{
#ifdef XHTXX_ENABLE_SERIAL_LOG
    if(!dev->serial) XHTXX_SHT4x_Probe(dev);
#endif
    I2C_Write(dev, 0x94, 0x00, 0, 1);             // soft-reset
    rtos_delay_milliseconds(1);
    uint8_t d[6];
//    dev->isWorking = XHTXX_SHT_CmdRead6(dev, 0xE0, 0x00, 1, 10, d,"SHT4x_Init");
    dev->isWorking = true;
    XHTXX_SHT_CmdRead6(dev, 0xE0, 0x00, 1, 10, d,"SHT4x_Init");
    if(dev->isWorking) XHTXX_SHT4x_ConvertStore(dev, d);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX SHT4x init %s (SDA=%i)",
                dev->isWorking ? "ok" : "FAILED", dev->i2c.pin_data);
}

static void XHTXX_SHT4x_Measure(xhtxx_dev_t *dev)
{
    uint8_t d[6];
    if(!XHTXX_SHT_CmdRead6(dev, 0xFD, 0x00, 1, 10, d,"SHT4x_Measure"))
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX SHT4x measure failed (SDA=%i)", dev->i2c.pin_data);
//        return;
    }
    XHTXX_SHT4x_ConvertStore(dev, d);
}

static void XHTXX_SHT4x_Reset(xhtxx_dev_t *dev)
{
    I2C_Write(dev, 0x94, 0x00, 0, 1);
    rtos_delay_milliseconds(1);
}

// =======================================================================
// -----------------------------------------------------------------------
//  AHT2x  family
// -----------------------------------------------------------------------
// Protocol: command → busy-poll status byte → 6-byte data read.
// No CRC; the busy bit (bit7) in the status byte is the only handshake.
//
// Conversion (from datasheet, integer):
//   raw_h = 20-bit field from bytes [1..3]
//   raw_t = 20-bit field from bytes [3..5]
//   h10 = (raw_h * 1000 + 524288) / 1048576
//   t10 = (raw_t * 2000 + 524288) / 1048576 - 500
//   max intermediate: 0xFFFFF * 2000 = 2,097,150,000  fits uint32_t ✓
// -----------------------------------------------------------------------
#define AHT2X_CMD_INI   0xBE
#define AHT2X_CMD_TMS   0xAC
#define AHT2X_CMD_RST   0xBA
#define AHT2X_DAT_BUSY  0x80
#define AHT2X_MAX_POLL  20

// Poll the single status byte until busy clears; return it, or 0xFF on timeout.
static uint8_t XHTXX_AHT2x_PollReady(xhtxx_dev_t *dev)
{
    for(uint8_t i = 0; i < AHT2X_MAX_POLL; i++)
    {
        rtos_delay_milliseconds(20);
        uint8_t status;
        I2C_Read(dev, &status, 1);
        if(!(status & AHT2X_DAT_BUSY)) return status;
    }
    return 0xFF;
}

static bool XHTXX_AHT2x_Probe(xhtxx_dev_t *dev)
{
    I2C_Write(dev, AHT2X_CMD_RST, 0, 0, 1);
    rtos_delay_milliseconds(20);
    I2C_Write(dev, AHT2X_CMD_INI, 0x08, 0x00, 3);
    uint8_t status = XHTXX_AHT2x_PollReady(dev);
    // calibrated bit = bit3; bits 6:3 must be 0b0001 → masked value == 0x08
    return (status != 0xFF) && ((status & 0x68) == 0x08);
}

static void XHTXX_AHT2x_Init(xhtxx_dev_t *dev)
{
    dev->isWorking = XHTXX_AHT2x_Probe(dev);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX AHT2x init %s (SDA=%i)",
                dev->isWorking ? "ok" : "FAILED", dev->i2c.pin_data);
}

static void XHTXX_AHT2x_Measure(xhtxx_dev_t *dev)
{
    I2C_Write(dev, AHT2X_CMD_TMS, 0x33, 0x00, 3);
    rtos_delay_milliseconds(80);

    uint8_t data[6] = { 0 };
    bool ready = false;
    for(uint8_t i = 0; i < 10; i++)
    {
        I2C_Read(dev, data, 6);
        if(!(data[0] & AHT2X_DAT_BUSY)) { ready = true; break; }
        ADDLOG_DEBUG(LOG_FEATURE_SENSOR, "XHTXX AHT2x busy (%ims)", i * 20);
        rtos_delay_milliseconds(20);
    }
    if(!ready)
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX AHT2x timed out (SDA=%i)", dev->i2c.pin_data);
        return;
    }
    if(data[1] == 0 && data[2] == 0 && (data[3] >> 4) == 0)
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX AHT2x zero humidity, skipping (SDA=%i)", dev->i2c.pin_data);
        return;
    }

    uint32_t raw_h = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t raw_t = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    int16_t h10 = (int16_t)((raw_h * 1000u + 524288u) / 1048576u) + dev->calHum;
    int16_t t10 = (int16_t)((raw_t * 2000u + 524288u) / 1048576u) - 500 + dev->calTemp;
    XHTXX_StoreAndLog(dev, t10, h10);
}

static void XHTXX_AHT2x_Reset(xhtxx_dev_t *dev)
{
    I2C_Write(dev, AHT2X_CMD_RST, 0, 0, 1);
    rtos_delay_milliseconds(20);
}

// =======================================================================
// -----------------------------------------------------------------------
//  CHT83xx  family  (CHT8305 / CHT8310 / CHT8315)
// -----------------------------------------------------------------------
// Variant detection: device ID word at register 0xFF.
//   0x8215 → CHT8310  (IS_CHT831X)
//   0x8315 → CHT8315  (IS_CHT831X)
//   other  → CHT8305
//
// Manufacturer ID at 0xFE is 0x5959 (rev1.1) or 0x5950 (rev1.0);
// any non-zero response at 0x40 is accepted as a CHT83xx.
//
// CHT8305 conversion (16-bit regs, read together from 0x00):
//   T = raw_t * 165 / 65535 - 40  → ×10: (raw_t*1650+32767)/65535 - 400
//   H = raw_h * 100 / 65535       → ×10: (raw_h*1000+32767)/65535
//
// CHT831x conversion (13-bit T at 0x00, 15-bit H at 0x01):
//   T = (int16_t)(raw >> 3) * 0.03125   → ×10: (s13*50+8)/16
//   H = (raw & 0x7FFF) / 32768 * 100    → ×10: ((raw&0x7FFF)*1000+16384)/32768
// -----------------------------------------------------------------------
#define CHT_REG_TEMP    0x00
#define CHT_REG_HUM     0x01
#define CHT_REG_STATUS  0x02
#define CHT_REG_CFG     0x03
#define CHT_REG_ONESHOT 0x0F
#define CHT_REG_ID      0xFE

#define IS_CHT831X(dev) ((dev)->subtype == 0x8215 || (dev)->subtype == 0x8315)

static bool XHTXX_CHT83xx_Probe(xhtxx_dev_t *dev)
{
    uint8_t buf[2];
    I2C_ReadReg(dev, CHT_REG_ID, buf, 2);               // manufacturer ID
    if(buf[0] == 0xFF && buf[1] == 0xFF) return false;  // no response
    I2C_ReadReg(dev, CHT_REG_ID + 1, buf, 2);           // device ID
    dev->subtype = ((uint16_t)buf[0] << 8) | buf[1];
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX CHT83xx mfr=%02X%02X dev=%04X",
                buf[0], buf[1], dev->subtype);
    return true;
}

static void XHTXX_CHT83xx_Init(xhtxx_dev_t *dev)
{
    if(IS_CHT831X(dev))
    {
        uint8_t status;
        I2C_ReadReg(dev, CHT_REG_STATUS, &status, 1);
        if(status)
            ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX CHT831x wake status: 0x%02X", status);
        // Shutdown / low-power mode; measurements triggered by one-shot register
        I2C_Write(dev, CHT_REG_CFG, 0x48, 0x80, 3);
    }
    const char *variant = IS_CHT831X(dev) ?
                          (dev->subtype == 0x8215 ? "CHT8310" : "CHT8315") :
                          "CHT8305";
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX %s init (SDA=%i)", variant, dev->i2c.pin_data);
    dev->isWorking = true;
}

static void XHTXX_CHT83xx_Measure(xhtxx_dev_t *dev)
{
    if(IS_CHT831X(dev))
    {
        I2C_Write(dev, CHT_REG_ONESHOT, 0x00, 0x00, 3);
    }

    uint8_t buf[4];
    rtos_delay_milliseconds(20);
    I2C_ReadReg(dev, CHT_REG_TEMP, buf, 4);

    int16_t t10, h10;
    if(IS_CHT831X(dev))
    {
        // Humidity has parity issues in the 4-byte read; fetch separately
        I2C_ReadReg(dev, CHT_REG_HUM, buf + 2, 2);
        int16_t s13 = (int16_t)((buf[0] << 8) | buf[1]) >> 3;
        t10 = (int16_t)((s13 * 50 + 8) / 16) + dev->calTemp;
        uint16_t rh = ((uint16_t)buf[2] << 8) | buf[3];
        h10 = (int16_t)(((rh & 0x7FFFu) * 1000u + 16384u) / 32768u) + dev->calHum;
    }
    else
    {
        uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
        uint16_t raw_h = ((uint16_t)buf[2] << 8) | buf[3];
        t10 = (int16_t)((raw_t * 1650u + 32767u) / 65535u) - 400 + dev->calTemp;
        h10 = (int16_t)((raw_h * 1000u + 32767u) / 65535u)        + dev->calHum;
    }
    XHTXX_StoreAndLog(dev, t10, h10);
}

static void XHTXX_CHT83xx_Reset(xhtxx_dev_t *dev)
{
    // No dedicated reset command; restore config register defaults
    if(IS_CHT831X(dev))
    {
        I2C_Write(dev, CHT_REG_CFG, 0x08, 0x80, 3);
    }
}

// =======================================================================
// -----------------------------------------------------------------------
//  SHT3x extended features  (XHTXX_ENABLE_SHT3_EXTENDED_FEATURES)
// -----------------------------------------------------------------------
#ifdef XHTXX_ENABLE_SHT3_EXTENDED_FEATURES

static const char g_onlySht3[] = "XHTXX: SHT3x only.";
#define REQUIRE_SHT3X(dev, code) do {                        \
    if((dev)->familyIdx != XHTXX_FAMILY_SHT3X) {            \
        ADDLOG_ERROR(LOG_FEATURE_SENSOR, g_onlySht3);        \
        return (code);                                       \
    }                                                        \
} while(0)

static void XHTXX_SHT3x_StartPeriodic(xhtxx_dev_t *dev, uint8_t msb, uint8_t lsb)
{
    I2C_Write(dev, msb, lsb, 0, 2);
    dev->periodicActive = true;
}

static void XHTXX_SHT3x_StopPeriodic(xhtxx_dev_t *dev)
{
    if(!dev->periodicActive) return;
    I2C_Write(dev, 0x30, 0x93, 0, 2);
    rtos_delay_milliseconds(1);
    dev->periodicActive = false;
}

static void XHTXX_SHT3x_FetchPeriodic(xhtxx_dev_t *dev)
{
    uint8_t d[6];
    if(!XHTXX_SHT_CmdRead6(dev, 0xE0, 0x00, 2, 0, d))
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX SHT3x periodic fetch failed (SDA=%i)", dev->i2c.pin_data);
        return;
    }
    XHTXX_SHT3x_ConvertStore(dev, d);
}

// Alert registers use a packed 16-bit format + CRC; float unavoidable here.
static void XHTXX_SHT3x_ReadAlertReg(xhtxx_dev_t *dev, uint8_t sub,
                                      float *out_hum, float *out_temp)
{
    uint8_t d[2];
    I2C_Write(dev, 0xE1, sub, 0, 2);
    I2C_Read(dev, d, 2);
    uint16_t w = ((uint16_t)d[0] << 8) | d[1];
    *out_hum  = 100.0f * (w & 0xFE00) / 65535.0f;
    *out_temp = 175.0f * ((uint16_t)(w << 7) / 65535.0f) - 45.0f;
}

static void XHTXX_SHT3x_WriteAlertReg(xhtxx_dev_t *dev, uint8_t sub,
                                       float hum, float temp)
{
    if(hum < 0.0f || hum > 100.0f || temp < -45.0f || temp > 130.0f)
    { ADDLOG_INFO(LOG_FEATURE_CMD, "XHTXX: Alert value out of range."); return; }
    uint16_t rawH = (uint16_t)(hum  / 100.0f * 65535.0f);
    uint16_t rawT = (uint16_t)((temp + 45.0f) / 175.0f * 65535.0f);
    uint16_t w    = (rawH & 0xFE00) | ((rawT >> 7) & 0x01FF);
    uint8_t  d[2] = { (uint8_t)(w >> 8), (uint8_t)(w & 0xFF) };
    uint8_t  crc  = 0xFF ^ d[0];
    for(uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? ((crc<<1)^0x31) : (crc<<1);
    crc ^= d[1];
    for(uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? ((crc<<1)^0x31) : (crc<<1);
    // 5-byte write: cmd(2) + data(2) + crc(1) – no generic helper for this length
    Soft_I2C_Start(&dev->i2c, dev->i2cAddr);
    Soft_I2C_WriteByte(&dev->i2c, 0x61);
    Soft_I2C_WriteByte(&dev->i2c, sub);
    Soft_I2C_WriteByte(&dev->i2c, d[0]);
    Soft_I2C_WriteByte(&dev->i2c, d[1]);
    Soft_I2C_WriteByte(&dev->i2c, crc);
    Soft_I2C_Stop(&dev->i2c);
}

#endif  // XHTXX_ENABLE_SHT3_EXTENDED_FEATURES

// =======================================================================
// -----------------------------------------------------------------------
//  Auto-detect
// -----------------------------------------------------------------------
static uint8_t XHTXX_AutoDetect(xhtxx_dev_t *dev)
{
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX: auto-detect on SDA=%i SCL=%i...",
                dev->i2c.pin_data, dev->i2c.pin_clk);
    for(uint8_t i = 0; i < XHTXX_PROBE_STEPS; i++)
    {
        dev->familyIdx = g_probeOrder[i].family;
        dev->i2cAddr   = g_probeOrder[i].addr;
        if(g_families[dev->familyIdx].probe_fn(dev))
        {
            ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX: found %s at 0x%02X",
                        g_families[dev->familyIdx].name, dev->i2cAddr >> 1);
            return dev->familyIdx;
        }
        rtos_delay_milliseconds(10);
    }
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX: no sensor found, defaulting SHT3x @ 0x44");
    dev->familyIdx = XHTXX_FAMILY_SHT3X;
    dev->i2cAddr   = XHTXX_ADDR_SHT;
    return dev->familyIdx;
}

// =======================================================================
// -----------------------------------------------------------------------
//  Multi-sensor selector
// -----------------------------------------------------------------------
static xhtxx_dev_t *XHTXX_GetSensor(const char *cmd, int argIdx, bool present)
{
    if(!present) return &g_sensors[0];
    int n = Tokenizer_GetArgInteger(argIdx);
    if(n < 1 || n > (int)g_numSensors)
    {
        ADDLOG_ERROR(LOG_FEATURE_SENSOR, "%s: sensor %i out of range (1..%i)",
                     cmd, n, g_numSensors);
        return NULL;
    }
    return &g_sensors[n - 1];
}

// =======================================================================
// -----------------------------------------------------------------------
//  Command handlers
// -----------------------------------------------------------------------

// XHTXX_Calibrate <deltaTemp> [<deltaHum>] [<sensorN>]
commandResult_t XHTXX_CMD_Calibrate(const void *context, const char *cmd,
                                     const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int argc = Tokenizer_GetArgsCount();
    xhtxx_dev_t *dev = XHTXX_GetSensor(cmd, 2, argc >= 3);
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    dev->calTemp = (int16_t)(Tokenizer_GetArgFloat(0) * 10.0f);
    dev->calHum  = (int8_t) (Tokenizer_GetArgFloat(1) * 10.0f);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX calibrate: calT=%d calH=%d (×0.1)",
                dev->calTemp, dev->calHum);
    return CMD_RES_OK;
}

// XHTXX_Cycle <seconds>
commandResult_t XHTXX_CMD_Cycle(const void *context, const char *cmd,
                                 const char *args, int cmdFlags)
{
    xhtxx_dev_t *dev = (xhtxx_dev_t *)context;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int s = Tokenizer_GetArgInteger(0);
    if(s < 1) { ADDLOG_INFO(LOG_FEATURE_CMD, "XHTXX: min 1s."); return CMD_RES_BAD_ARGUMENT; }
    dev->secondsBetween = (uint8_t)s;
    ADDLOG_INFO(LOG_FEATURE_CMD, "XHTXX: measure every %i s", s);
    return CMD_RES_OK;
}

// XHTXX_Measure  (immediate one-shot)
commandResult_t XHTXX_CMD_Force(const void *context, const char *cmd,
                                 const char *args, int cmdFlags)
{
    xhtxx_dev_t *dev = (xhtxx_dev_t *)context;
    dev->secondsUntilNext = dev->secondsBetween;
    g_families[dev->familyIdx].measure_fn(dev);
    return CMD_RES_OK;
}

// XHTXX_Reinit  (soft-reset + re-initialise)
commandResult_t XHTXX_CMD_Reinit(const void *context, const char *cmd,
                                  const char *args, int cmdFlags)
{
    xhtxx_dev_t *dev = (xhtxx_dev_t *)context;
    dev->secondsUntilNext = dev->secondsBetween;
#ifdef XHTXX_ENABLE_SERIAL_LOG
    dev->serial = 0;    // allow init to re-read serial
#endif
    g_families[dev->familyIdx].reset_fn(dev);
    g_families[dev->familyIdx].init_fn(dev);
    return CMD_RES_OK;
}

// XHTXX_AddSensor  [SDA=<pin>] [SCL=<pin>] [family=…] …
commandResult_t XHTXX_CMD_AddSensor(const void *context, const char *cmd,
                                     const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    XHTXX_Init();
    return CMD_RES_OK;
}

// XHTXX_ListSensors
commandResult_t XHTXX_CMD_ListSensors(const void *context, const char *cmd,
                                       const char *args, int cmdFlags)
{
    if(!g_numSensors)
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX: no sensors registered.");
        return CMD_RES_OK;
    }
    for(uint8_t i = 0; i < g_numSensors; i++)
    {
        xhtxx_dev_t *s = &g_sensors[i];
        int16_t tf = s->lastTemp % 10; if(tf < 0) tf = -tf;
#ifdef XHTXX_ENABLE_SERIAL_LOG
        ADDLOG_INFO(LOG_FEATURE_SENSOR,
            "  [%u] %s sn=%08X SDA=%i SCL=%i addr=0x%02X T=%d.%d°C H=%d.%d%% ch=%i/%i",
            i+1, g_families[s->familyIdx].name, s->serial,
            s->i2c.pin_data, s->i2c.pin_clk, s->i2cAddr>>1,
            s->lastTemp/10, tf, s->lastHumid/10, s->lastHumid%10,
            s->channel_temp, s->channel_humid);
#else
        ADDLOG_INFO(LOG_FEATURE_SENSOR,
            "  [%u] %s SDA=%i SCL=%i addr=0x%02X T=%d.%d°C H=%d.%d%% ch=%i/%i",
            i+1, g_families[s->familyIdx].name,
            s->i2c.pin_data, s->i2c.pin_clk, s->i2cAddr>>1,
            s->lastTemp/10, tf, s->lastHumid/10, s->lastHumid%10,
            s->channel_temp, s->channel_humid);
#endif
    }
    return CMD_RES_OK;
}

// -----------------------------------------------------------------------
//  SHT3x extended commands  (XHTXX_ENABLE_SHT3_EXTENDED_FEATURES)
// -----------------------------------------------------------------------
#ifdef XHTXX_ENABLE_SHT3_EXTENDED_FEATURES

commandResult_t XHTXX_CMD_LaunchPer(const void *context, const char *cmd,
                                     const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    int argc = Tokenizer_GetArgsCount();
    uint8_t msb = 0x23, lsb = 0x22;
    xhtxx_dev_t *dev;
    if(argc >= 2) {
        msb = (uint8_t)Tokenizer_GetArgInteger(0);
        lsb = (uint8_t)Tokenizer_GetArgInteger(1);
        dev = XHTXX_GetSensor(cmd, 2, argc >= 3);
    } else {
        dev = XHTXX_GetSensor(cmd, 0, argc >= 1);
    }
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    REQUIRE_SHT3X(dev, CMD_RES_ERROR);
    XHTXX_SHT3x_StopPeriodic(dev);
    rtos_delay_milliseconds(25);
    XHTXX_SHT3x_StartPeriodic(dev, msb, lsb);
    return CMD_RES_OK;
}

commandResult_t XHTXX_CMD_FetchPer(const void *context, const char *cmd,
                                    const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    xhtxx_dev_t *dev = XHTXX_GetSensor(cmd, 0, Tokenizer_GetArgsCount() >= 1);
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    REQUIRE_SHT3X(dev, CMD_RES_ERROR);
    if(!dev->periodicActive) {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX: periodic not running."); return CMD_RES_ERROR;
    }
    XHTXX_SHT3x_FetchPeriodic(dev);
    return CMD_RES_OK;
}

commandResult_t XHTXX_CMD_StopPer(const void *context, const char *cmd,
                                   const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    xhtxx_dev_t *dev = XHTXX_GetSensor(cmd, 0, Tokenizer_GetArgsCount() >= 1);
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    REQUIRE_SHT3X(dev, CMD_RES_ERROR);
    XHTXX_SHT3x_StopPeriodic(dev);
    return CMD_RES_OK;
}

commandResult_t XHTXX_CMD_Heater(const void *context, const char *cmd,
                                  const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int on = Tokenizer_GetArgInteger(0);
    xhtxx_dev_t *dev = XHTXX_GetSensor(cmd, 1, Tokenizer_GetArgsCount() >= 2);
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    REQUIRE_SHT3X(dev, CMD_RES_ERROR);
    I2C_Write(dev, 0x30, on ? 0x6D : 0x66, 0, 2);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX SHT3x heater %s (SDA=%i)", on?"on":"off", dev->i2c.pin_data);
    return CMD_RES_OK;
}

commandResult_t XHTXX_CMD_GetStatus(const void *context, const char *cmd,
                                     const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    xhtxx_dev_t *dev = XHTXX_GetSensor(cmd, 0, Tokenizer_GetArgsCount() >= 1);
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    REQUIRE_SHT3X(dev, CMD_RES_ERROR);
    uint8_t buf[3];
    I2C_Write(dev, 0xF3, 0x2D, 0, 2);
    I2C_Read(dev, buf, 3);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX SHT3x status: %02X%02X (SDA=%i)",
                buf[0], buf[1], dev->i2c.pin_data);
    return CMD_RES_OK;
}

commandResult_t XHTXX_CMD_ClearStatus(const void *context, const char *cmd,
                                       const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    xhtxx_dev_t *dev = XHTXX_GetSensor(cmd, 0, Tokenizer_GetArgsCount() >= 1);
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    REQUIRE_SHT3X(dev, CMD_RES_ERROR);
    I2C_Write(dev, 0x30, 0x41, 0, 2);
    return CMD_RES_OK;
}

commandResult_t XHTXX_CMD_ReadAlert(const void *context, const char *cmd,
                                     const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    xhtxx_dev_t *dev = XHTXX_GetSensor(cmd, 0, Tokenizer_GetArgsCount() >= 1);
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    REQUIRE_SHT3X(dev, CMD_RES_ERROR);
    float tLS, tLC, tHC, tHS, hLS, hLC, hHC, hHS;
    XHTXX_SHT3x_ReadAlertReg(dev, 0x1F, &hHS, &tHS);
    XHTXX_SHT3x_ReadAlertReg(dev, 0x14, &hHC, &tHC);
    XHTXX_SHT3x_ReadAlertReg(dev, 0x09, &hLC, &tLC);
    XHTXX_SHT3x_ReadAlertReg(dev, 0x02, &hLS, &tLS);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "Alert T (LS/LC/HC/HS): %.1f/%.1f/%.1f/%.1f", tLS, tLC, tHC, tHS);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "Alert H (LS/LC/HC/HS): %.1f/%.1f/%.1f/%.1f", hLS, hLC, hHC, hHS);
    return CMD_RES_OK;
}

// XHTXX_SetAlert <tHS> <tLS> <hHS> <hLS> [sensor]
commandResult_t XHTXX_CMD_SetAlert(const void *context, const char *cmd,
                                    const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 4)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    xhtxx_dev_t *dev = XHTXX_GetSensor(cmd, 4, Tokenizer_GetArgsCount() >= 5);
    if(!dev) return CMD_RES_BAD_ARGUMENT;
    REQUIRE_SHT3X(dev, CMD_RES_ERROR);
    float tHS = Tokenizer_GetArgFloat(0), tLS = Tokenizer_GetArgFloat(1);
    float hHS = Tokenizer_GetArgFloat(2), hLS = Tokenizer_GetArgFloat(3);
    XHTXX_SHT3x_WriteAlertReg(dev, 0x1D, hHS,        tHS);
    XHTXX_SHT3x_WriteAlertReg(dev, 0x16, hHS - 0.5f, tHS - 0.5f);
    XHTXX_SHT3x_WriteAlertReg(dev, 0x0B, hLS + 0.5f, tLS + 0.5f);
    XHTXX_SHT3x_WriteAlertReg(dev, 0x00, hLS,        tLS);
    return CMD_RES_OK;
}

#endif  // XHTXX_ENABLE_SHT3_EXTENDED_FEATURES

// =======================================================================
// -----------------------------------------------------------------------
//  Public driver entry points
// -----------------------------------------------------------------------

// startDriver XHTXX [SDA=<pin>] [SCL=<pin>]
//                   [family=sht3|sht4|aht2|cht]   (omit → auto-detect)
//                   [address=<hex>]               (override I²C address)
//                   [chan_t=<ch>] [chan_h=<ch>]
void XHTXX_Init(void)
{
    if(g_numSensors >= XHTXX_MAX_SENSORS)
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "XHTXX: sensor array full (%i).", XHTXX_MAX_SENSORS);
        return;
    }
    xhtxx_dev_t *dev = &g_sensors[g_numSensors];

    // --- Default pin / channel assignment ---
    dev->i2c.pin_clk  = 9;
    dev->i2c.pin_data = 17;
    dev->channel_temp  = -1;
    dev->channel_humid = -1;

    if(g_numSensors == 0)
    {
        dev->i2c.pin_clk   = PIN_FindPinIndexForRole(IOR_SHT3X_CLK, dev->i2c.pin_clk);
        dev->i2c.pin_data  = PIN_FindPinIndexForRole(IOR_SHT3X_DAT, dev->i2c.pin_data);
        dev->channel_temp  = g_cfg.pins.channels [dev->i2c.pin_data];
        dev->channel_humid = g_cfg.pins.channels2[dev->i2c.pin_data];
    }

    // --- Parse keyword args ---
    dev->i2c.pin_clk   = Tokenizer_GetPinEqual("SCL",   dev->i2c.pin_clk);
    dev->i2c.pin_data  = Tokenizer_GetPinEqual("SDA",   dev->i2c.pin_data);
    dev->channel_temp  = Tokenizer_GetArgEqualInteger("chan_t", dev->channel_temp);
    dev->channel_humid = Tokenizer_GetArgEqualInteger("chan_h", dev->channel_humid);

    dev->secondsBetween   = 10;
    dev->secondsUntilNext = 1;
    dev->calTemp = 0;
    dev->calHum  = 0;
#ifdef XHTXX_ENABLE_SERIAL_LOG
    dev->serial  = 0;
#endif
#ifdef XHTXX_ENABLE_SHT3_EXTENDED_FEATURES
    dev->periodicActive = false;
#endif

    // --- family= : first char  s→SHT (digit 3/4), a→AHT2x, c→CHT83xx ---
    // Use "default" as fallback string to avoid matching 'a' for "auto"
    const char *fam = Tokenizer_GetArgEqualDefault("family", "default");
    uint8_t requestedFamily = XHTXX_FAMILY_AUTO;
    if     (fam[0] == 's' || fam[0] == 'S')
        requestedFamily = (fam[3] == '4') ? XHTXX_FAMILY_SHT4X : XHTXX_FAMILY_SHT3X;
    else if(fam[0] == 'a' || fam[0] == 'A')
        requestedFamily = XHTXX_FAMILY_AHT2X;
    else if(fam[0] == 'c' || fam[0] == 'C')
        requestedFamily = XHTXX_FAMILY_CHT83XX;

    // --- address= resolved before any I²C traffic ---
    uint8_t addrArg = (uint8_t)Tokenizer_GetArgEqualInteger("address", 0);

    Soft_I2C_PreInit(&dev->i2c);
    rtos_delay_milliseconds(50);
    setPinUsedString(dev->i2c.pin_clk,  "XHTXX SCL");
    setPinUsedString(dev->i2c.pin_data, "XHTXX SDA");

    // --- Family resolution ---
    if(requestedFamily == XHTXX_FAMILY_AUTO)
    {
        XHTXX_AutoDetect(dev);
        if(addrArg) dev->i2cAddr = addrArg << 1;   // rare override after detect
    }
    else
    {
        dev->familyIdx = requestedFamily;
        dev->i2cAddr   = addrArg ? (addrArg << 1)
                                 : g_families[requestedFamily].defaultAddr;
    }

    // --- Initialise ---
    g_families[dev->familyIdx].init_fn(dev);

    // --- Register commands on first sensor only ---
    if(g_numSensors == 0)
    {
        //cmddetail:{"name":"XHTXX_Calibrate","args":"[DeltaTemp] [DeltaHum] [sensor]",
        //cmddetail:"descr":"Offset calibration in °C and %RH. Sensor index optional (1-based).",
        //cmddetail:"fn":"XHTXX_CMD_Calibrate","file":"driver/drv_xhtxx.c","requires":"",
        //cmddetail:"examples":"XHTXX_Calibrate -1.5 3"}
        CMD_RegisterCommand("XHTXX_Calibrate",   XHTXX_CMD_Calibrate,   dev);

        //cmddetail:{"name":"XHTXX_Cycle","args":"[Seconds]",
        //cmddetail:"descr":"Measurement interval in seconds (min 1, max 255).",
        //cmddetail:"fn":"XHTXX_CMD_Cycle","file":"driver/drv_xhtxx.c","requires":"",
        //cmddetail:"examples":"XHTXX_Cycle 30"}
        CMD_RegisterCommand("XHTXX_Cycle",       XHTXX_CMD_Cycle,       dev);

        //cmddetail:{"name":"XHTXX_Measure","args":"",
        //cmddetail:"descr":"Immediate one-shot measurement.",
        //cmddetail:"fn":"XHTXX_CMD_Force","file":"driver/drv_xhtxx.c","requires":"",
        //cmddetail:"examples":"XHTXX_Measure"}
        CMD_RegisterCommand("XHTXX_Measure",     XHTXX_CMD_Force,       dev);

        //cmddetail:{"name":"XHTXX_Reinit","args":"",
        //cmddetail:"descr":"Soft-reset and re-initialise sensor.",
        //cmddetail:"fn":"XHTXX_CMD_Reinit","file":"driver/drv_xhtxx.c","requires":"",
        //cmddetail:"examples":"XHTXX_Reinit"}
        CMD_RegisterCommand("XHTXX_Reinit",      XHTXX_CMD_Reinit,      dev);

        //cmddetail:{"name":"XHTXX_AddSensor","args":"[SDA=pin] [SCL=pin] [family=…] [chan_t=ch] [chan_h=ch]",
        //cmddetail:"descr":"Register an additional sensor on different pins or with a different family.",
        //cmddetail:"fn":"XHTXX_CMD_AddSensor","file":"driver/drv_xhtxx.c","requires":"",
        //cmddetail:"examples":"XHTXX_AddSensor SDA=4 SCL=5 family=aht2"}
        CMD_RegisterCommand("XHTXX_AddSensor",   XHTXX_CMD_AddSensor,   dev);

        //cmddetail:{"name":"XHTXX_ListSensors","args":"",
        //cmddetail:"descr":"List all registered sensors and their last readings.",
        //cmddetail:"fn":"XHTXX_CMD_ListSensors","file":"driver/drv_xhtxx.c","requires":"",
        //cmddetail:"examples":"XHTXX_ListSensors"}
        CMD_RegisterCommand("XHTXX_ListSensors", XHTXX_CMD_ListSensors, dev);

#ifdef XHTXX_ENABLE_SHT3_EXTENDED_FEATURES
        CMD_RegisterCommand("XHTXX_LaunchPer",   XHTXX_CMD_LaunchPer,   dev);
        CMD_RegisterCommand("XHTXX_FetchPer",    XHTXX_CMD_FetchPer,    dev);
        CMD_RegisterCommand("XHTXX_StopPer",     XHTXX_CMD_StopPer,     dev);
        CMD_RegisterCommand("XHTXX_Heater",      XHTXX_CMD_Heater,      dev);
        CMD_RegisterCommand("XHTXX_GetStatus",   XHTXX_CMD_GetStatus,   dev);
        CMD_RegisterCommand("XHTXX_ClearStatus", XHTXX_CMD_ClearStatus, dev);
        CMD_RegisterCommand("XHTXX_ReadAlert",   XHTXX_CMD_ReadAlert,   dev);
        CMD_RegisterCommand("XHTXX_SetAlert",    XHTXX_CMD_SetAlert,    dev);
#endif
    }

    g_numSensors++;
}

void XHTXX_StopDriver(void)
{
    for(uint8_t i = 0; i < g_numSensors; i++)
    {
        xhtxx_dev_t *dev = &g_sensors[i];
#ifdef XHTXX_ENABLE_SHT3_EXTENDED_FEATURES
        if(dev->familyIdx == XHTXX_FAMILY_SHT3X && dev->periodicActive)
            XHTXX_SHT3x_StopPeriodic(dev);
#endif
        g_families[dev->familyIdx].reset_fn(dev);
    }
}

void XHTXX_OnEverySecond(void)
{
    for(uint8_t i = 0; i < g_numSensors; i++)
    {
        xhtxx_dev_t *dev = &g_sensors[i];
        if(dev->secondsUntilNext == 0)
        {
            if(dev->isWorking)
            {
#ifdef XHTXX_ENABLE_SHT3_EXTENDED_FEATURES
                if(dev->periodicActive)
                    XHTXX_SHT3x_FetchPeriodic(dev);
                else
#endif
                g_families[dev->familyIdx].measure_fn(dev);
            }
            dev->secondsUntilNext = dev->secondsBetween;
        }
        else
        {
            dev->secondsUntilNext--;
        }
    }
}

void XHTXX_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState)
{
    if(bPreState) return;
    for(uint8_t i = 0; i < g_numSensors; i++)
    {
        xhtxx_dev_t *dev = &g_sensors[i];
        int16_t tf = dev->lastTemp % 10; if(tf < 0) tf = -tf;
        hprintf255(request,
                   "<h2>%s[%u] Temp=%d.%d°C Humid=%d.%d%%</h2>",
                   g_families[dev->familyIdx].name, i+1,
                   dev->lastTemp/10, tf,
                   dev->lastHumid/10, dev->lastHumid%10);
        if(!dev->isWorking)
            hprintf255(request, "WARNING: %s[%u] init failed – check pins!",
                       g_families[dev->familyIdx].name, i+1);
        if(dev->channel_humid >= 0 && dev->channel_humid == dev->channel_temp)
            hprintf255(request, "WARNING: %s[%u] temp/humid share a channel!",
                       g_families[dev->familyIdx].name, i+1);
    }
}

#endif  // ENABLE_DRIVER_XHTXX
