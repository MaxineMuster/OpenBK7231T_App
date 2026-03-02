// drv_soft_i2c_sim_sensors.c
// -------------------------------------------------------
// Sensor plugins for the soft-I2C simulator.
//
// Each sensor family provides:
//   - init()             – seed default value ranges
//   - encode_response()  – turn cmd bytes + dynamic values
//                          into the exact byte sequence the
//                          real hardware would return
//
// To add a new sensor:
//   1. Write its three callbacks (init, encode_response,
//      optionally on_read_complete).
//   2. Define a static const sim_sensor_ops_t for it.
//   3. Add a SoftI2C_Sim_AddXxx() convenience function.
//   4. Declare it in drv_soft_i2c_sim.h.
//
// No changes to drv_soft_i2c_sim.c are ever needed.
// -------------------------------------------------------
#ifdef WIN32

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>   // abs()
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"

#include "drv_soft_i2c_sim.h"

// ===================================================================
// Shared helpers
// ===================================================================

// CRC-8  poly=0x31, init=0xFF  (Sensirion convention)
static uint8_t crc8_sensirion(uint8_t a, uint8_t b) {
    uint8_t crc = 0xFF ^ a;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    crc ^= b;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    return crc;
}

// Build a 3-byte Sensirion word (MSB, LSB, CRC) into buf[]
static void pack_sensirion_word(uint8_t *buf, uint16_t raw) {
    buf[0] = (uint8_t)(raw >> 8);
    buf[1] = (uint8_t)(raw & 0xFF);
    buf[2] = crc8_sensirion(buf[0], buf[1]);
}

// Build a 6-byte Sensirion measurement response (T word + H word)
static void pack_sensirion_meas(uint8_t *resp, uint16_t raw_t, uint16_t raw_h) {
    pack_sensirion_word(resp,     raw_t);
    pack_sensirion_word(resp + 3, raw_h);
}

// ===================================================================
// SHT3x plugin
// ===================================================================
// Protocol used by drv_shtxx.c:
//   Write 0x24 0x00  -> trigger single-shot (high rep., no stretch)
//   Write 0x30 0xA2  -> soft reset
//   Write 0x36 0x82  -> read serial number
//   Write 0x23 0x22  -> start periodic (ignored here)
//   Write 0xE0 0x00  -> fetch periodic result
//   Read  6 bytes    -> [T_MSB T_LSB T_CRC H_MSB H_LSB H_CRC]
//
// Physical encoding (from Sensirion datasheet / drv_shtxx.c):
//   raw_T = (t_x10 + 450) * 65535 / 1750
//   raw_H = h_x10 * 65535 / 1000          (hum_scale=100 path)
// ===================================================================

static void sht3x_init(sim_ctx_t *ctx) {
    // Temperature 20.0-25.0 C, start 22.0, drift +-0.3/read
    SoftI2C_Sim_SetValue(ctx, SIM_Q_TEMPERATURE, 220, 200, 250, 3);
    // Humidity 40-60 %RH, start 50.0, drift +-0.5/read
    SoftI2C_Sim_SetValue(ctx, SIM_Q_HUMIDITY,    500, 400, 600, 5);
}

static void sht3x_build_meas(sim_ctx_t *ctx) {
    int32_t t10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_TEMPERATURE);
    int32_t h10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_HUMIDITY);
    uint16_t raw_t = (uint16_t)(((t10 + 450) * 65535) / 1750);
    uint16_t raw_h = (uint16_t)((h10 * 65535) / 1000);
    if (raw_h > 65535) raw_h = 65535;
    pack_sensirion_meas(ctx->resp, raw_t, raw_h);
    ctx->resp_len = 6;
    printf("[SIM][SHT3x] T=%d.%d C  H=%d.%d%%  raw_T=0x%04X raw_H=0x%04X\n",
           (int)(t10/10), (int)abs(t10%10),
           (int)(h10/10), (int)(h10%10), raw_t, raw_h);
}

static bool sht3x_encode(sim_ctx_t *ctx) {
    if (ctx->cmd_len < 1) return false;
    uint8_t c0 = ctx->cmd[0];

    // Single-shot measurement
    if (c0 == 0x24) {
        sht3x_build_meas(ctx);
        return true;
    }
    // Periodic fetch
    if (c0 == 0xE0 && ctx->cmd_len >= 2 && ctx->cmd[1] == 0x00) {
        sht3x_build_meas(ctx);
        return true;
    }
    // Periodic start commands (0x20-0x27) – just ACK, no data
    if (c0 >= 0x20 && c0 <= 0x27) {
        ctx->resp_len = 0;
        return true;
    }
    // Soft reset
    if (c0 == 0x30) {
        ctx->resp_len = 0;
        return true;
    }
    // Serial number  (cmd 0x36 0x82)
    if (c0 == 0x36) {
        // Return a plausible CRC-valid fake serial: 0xDEAD / 0xBEEF
        pack_sensirion_word(ctx->resp,     0xDEAD);
        pack_sensirion_word(ctx->resp + 3, 0xBEEF);
        ctx->resp_len = 6;
        return true;
    }
    // Status register clear / heater commands – ACK only
    ctx->resp_len = 0;
    return true;
}

static const sim_sensor_ops_t g_sht3x_ops = {
    .name             = "SHT3x",
    .init             = sht3x_init,
    .encode_response  = sht3x_encode,
    .on_read_complete = NULL,
};

// ===================================================================
// SHT4x plugin
// ===================================================================
// Protocol used by drv_shtxx.c:
//   Write 0xFD        -> high-repeatability single-shot
//   Write 0x94        -> soft reset
//   Write 0x89        -> read serial number
//   Read  6 bytes     -> [T_MSB T_LSB T_CRC H_MSB H_LSB H_CRC]
//
// SHT4x hum_scale=125, offset=-6 (from SHT_g_cfg).
// To keep the encode simple we use the SHT3x raw encoding and let
// the driver's hum_scale/offset correction produce the right output.
// For a truly faithful SHT4x encoding:
//   raw_H = (h_x10 + 60) * 65535 / 1250
// ===================================================================

static void sht4x_init(sim_ctx_t *ctx) {
    SoftI2C_Sim_SetValue(ctx, SIM_Q_TEMPERATURE, 220, 200, 250, 3);
    SoftI2C_Sim_SetValue(ctx, SIM_Q_HUMIDITY,    500, 400, 600, 5);
}

static bool sht4x_encode(sim_ctx_t *ctx) {
    if (ctx->cmd_len < 1) return false;
    uint8_t c0 = ctx->cmd[0];

    if (c0 == 0xFD) {
        int32_t t10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_TEMPERATURE);
        int32_t h10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_HUMIDITY);
        uint16_t raw_t = (uint16_t)(((t10 + 450) * 65535) / 1750);
        // SHT4x: raw_H = (h_x10 + 60) * 65535 / 1250
        int32_t h_adj = h10 + 60;  // compensate the -6 offset the driver applies
        uint16_t raw_h = (uint16_t)((h_adj * 65535) / 1250);
        if (raw_h > 65535) raw_h = 65535;
        pack_sensirion_meas(ctx->resp, raw_t, raw_h);
        ctx->resp_len = 6;
        printf("[SIM][SHT4x] T=%d.%d C  H=%d.%d%%  raw_T=0x%04X raw_H=0x%04X\n",
               (int)(t10/10), (int)abs(t10%10),
               (int)(h10/10), (int)(h10%10), raw_t, raw_h);
        return true;
    }
    if (c0 == 0x94) {  // reset
        ctx->resp_len = 0;
        return true;
    }
    if (c0 == 0x89) {  // serial number
        pack_sensirion_word(ctx->resp,     0xDEAD);
        pack_sensirion_word(ctx->resp + 3, 0xBEEF);
        ctx->resp_len = 6;
        return true;
    }
    ctx->resp_len = 0;
    return true;
}

static const sim_sensor_ops_t g_sht4x_ops = {
    .name             = "SHT4x",
    .init             = sht4x_init,
    .encode_response  = sht4x_encode,
    .on_read_complete = NULL,
};

// ===================================================================
// AHT2x plugin
// ===================================================================
// Protocol (from drv_aht2x.c):
//   Write 0xBA              -> soft reset (AHT2X_CMD_RST)
//   Write 0xBE 0x08 0x00    -> initialise (AHT2X_CMD_INI + params)
//   Read  1 byte            -> status (bit7=busy, bits[6:3]=0001 when calibrated)
//   Write 0xAC 0x33 0x00    -> trigger measurement (AHT2X_CMD_TMS)
//   Read  6 bytes           -> [status, H[19:12], H[11:4], H[3:0]|T[19:16],
//                               T[15:8], T[7:0]]
//
// Physical encoding:
//   raw_H = (humid/100) * 2^20   = humid_x10 * 104857.6 / 1000
//   raw_T = ((temp+50)/200) * 2^20 = (temp_x10+500) * 104857.6 / 2000
// ===================================================================

// Per-sensor state: track calibration / busy phase
typedef struct { bool calibrated; uint8_t busy_countdown; } aht2x_state_t;

static void aht2x_init(sim_ctx_t *ctx) {
    SoftI2C_Sim_SetValue(ctx, SIM_Q_TEMPERATURE, 220, 150, 350,  3);
    SoftI2C_Sim_SetValue(ctx, SIM_Q_HUMIDITY,    500, 200, 900,  5);
    aht2x_state_t *s = (aht2x_state_t *)malloc(sizeof(aht2x_state_t));
    s->calibrated      = false;
    s->busy_countdown  = 0;
    ctx->user = s;
}

static bool aht2x_encode(sim_ctx_t *ctx) {
    aht2x_state_t *s = (aht2x_state_t *)ctx->user;
    if (!s || ctx->cmd_len < 1) return false;
    uint8_t c0 = ctx->cmd[0];

    // Soft reset
    if (c0 == 0xBA) {
        s->calibrated     = false;
        s->busy_countdown = 0;
        ctx->resp_len = 0;
        return true;
    }

    // Initialise
    if (c0 == 0xBE) {
        s->calibrated = true;
        ctx->resp_len = 0;
        return true;
    }

    // Status read is triggered by a Read Start (no write command), but
    // the driver polls via a bare Read Start after the init command.
    // We handle it as a zero-length command producing 1 byte:
    //   bit7=0 (not busy), bits[3]=1, bits[6:4]=000 -> 0x08 (calibrated, ready)
    if (ctx->cmd_len == 0) {
        ctx->resp[0]  = s->calibrated ? 0x08 : 0x00;
        ctx->resp_len = 1;
        return true;
    }

    // Trigger measurement
    if (c0 == 0xAC) {
        int32_t t10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_TEMPERATURE);
        int32_t h10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_HUMIDITY);

        // raw values in Q20 (2^20 = 1048576)
        uint32_t raw_h = (uint32_t)((int64_t)h10 * 1048576 / 1000);
        uint32_t raw_t = (uint32_t)((int64_t)(t10 + 500) * 1048576 / 2000);
        if (raw_h > 0xFFFFF) raw_h = 0xFFFFF;
        if (raw_t > 0xFFFFF) raw_t = 0xFFFFF;

        // byte[0]: status – not busy, calibrated
        ctx->resp[0] = s->calibrated ? 0x08 : 0x00;
        // bytes[1-3]: humidity (20 bits, MSB first, upper nibble of byte[3])
        ctx->resp[1] = (uint8_t)((raw_h >> 12) & 0xFF);
        ctx->resp[2] = (uint8_t)((raw_h >>  4) & 0xFF);
        ctx->resp[3] = (uint8_t)(((raw_h & 0x0F) << 4) | ((raw_t >> 16) & 0x0F));
        // bytes[4-5]: temperature low 16 bits
        ctx->resp[4] = (uint8_t)((raw_t >>  8) & 0xFF);
        ctx->resp[5] = (uint8_t)( raw_t        & 0xFF);
        ctx->resp_len = 6;

        printf("[SIM][AHT2x] T=%d.%d C  H=%d.%d%%  raw_T=0x%05X raw_H=0x%05X\n",
               (int)(t10/10), (int)abs(t10%10),
               (int)(h10/10), (int)(h10%10), raw_t, raw_h);
        return true;
    }

    ctx->resp_len = 0;
    return true;
}

static const sim_sensor_ops_t g_aht2x_ops = {
    .name             = "AHT2x",
    .init             = aht2x_init,
    .encode_response  = aht2x_encode,
    .on_read_complete = NULL,
};

// ===================================================================
// BMP280 / BME280 plugin
// ===================================================================
// The BMP280 driver (BMP280.h included by drv_bmp280.c) reads via
// raw register accesses using BMP280_Start/Write/Read/Stop wrappers,
// which internally map to Soft_I2C_Start_Internal / WriteByte /
// ReadByte / Stop.
//
// Key registers:
//   0xD0  -> chip ID  (BMP280=0x60, BME280=0x60 for most clones)
//   0x88-0x9F -> calibration data (trim coefficients, 24 bytes)
//   0xA1  -> dig_H1 (BME280 only, 1 byte)
//   0xE1-0xE7 -> dig_H2..H6 (BME280, 7 bytes)
//   0xF3  -> status
//   0xF4  -> ctrl_meas (osrs_t, osrs_p, mode)
//   0xF7-0xFC -> raw ADC data (press[19:0], temp[19:0], hum[15:0])
//
// Physical encoding (from BMP280 datasheet compensated formulas):
//   We use the "simpler" integer-only forward path.
//   For simulation we just pre-compute the ADC values that would
//   produce the desired temperature/pressure, using fixed
//   calibration constants that are also returned from 0x88.
// ===================================================================

// Fixed calibration constants chosen so the compensation formulas
// produce human-readable results without overflow.
// These match the defaults used in many BMP280 evaluation boards.
#define BMP_DIG_T1 27504u
#define BMP_DIG_T2 26435
#define BMP_DIG_T3 -1000
#define BMP_DIG_P1 36477u
#define BMP_DIG_P2 -10685
#define BMP_DIG_P3 3024
#define BMP_DIG_P4 2855
#define BMP_DIG_P5 140
#define BMP_DIG_P6 -7
#define BMP_DIG_P7 15500
#define BMP_DIG_P8 -14600
#define BMP_DIG_P9 6000

typedef struct {
    uint8_t reg;          // register pointer set by last write
    bool    is_bme280;    // if true, also simulate humidity
} bmp280_state_t;

// Pack calibration bytes into buf (little-endian, as the chip stores them)
static void bmp280_pack_calib(uint8_t *buf) {
    // T1 (uint16), T2 (int16), T3 (int16)
    uint16_t t1 = BMP_DIG_T1;
    int16_t  t2 = BMP_DIG_T2, t3 = BMP_DIG_T3;
    buf[0]  = t1 & 0xFF; buf[1]  = t1 >> 8;
    buf[2]  = t2 & 0xFF; buf[3]  = t2 >> 8;
    buf[4]  = t3 & 0xFF; buf[5]  = t3 >> 8;
    // P1-P9
    uint16_t p1 = BMP_DIG_P1;
    int16_t  pv[8] = {BMP_DIG_P2, BMP_DIG_P3, BMP_DIG_P4,
                      BMP_DIG_P5, BMP_DIG_P6, BMP_DIG_P7,
                      BMP_DIG_P8, BMP_DIG_P9};
    buf[6]  = p1 & 0xFF; buf[7]  = p1 >> 8;
    for (int i = 0; i < 8; i++) {
        buf[8  + i*2] = pv[i] & 0xFF;
        buf[9  + i*2] = (pv[i] >> 8) & 0xFF;
    }
    // Total: 6 + 2 + 16 = 24 bytes (0x88-0x9F)
}

// Compute the raw ADC temperature value that produces t_x10 degrees
// using the BMP280 compensation algorithm (inverted).
// The compensation forward path is:
//   var1 = (adc_T/8 - DIG_T1*2) * DIG_T2 / 2048
//   var2 = (adc_T/16 - DIG_T1)^2 * DIG_T3 / 67108864 / 32
//   t_fine = var1 + var2
//   T = t_fine * 5 / 320   (in 0.01 C units)
// We use a linear approximation (DIG_T3 is tiny) to invert:
//   adc_T ~= (T_hundredths * 320/5 * 2048 / DIG_T2 + DIG_T1*2) * 8
static uint32_t bmp280_temp_to_adc(int32_t t10) {
    // t_fine target: T_hundredths * 320 / 5  where T_hundredths = t10 * 10
    int32_t T_h     = t10 * 10;          // 0.01 C units
    int32_t t_fine  = (T_h * 320) / 5;   // ~6400 * T_h / 100
    // var1 = t_fine (ignoring var2)
    // adc_T = (var1 * 2048 / DIG_T2 + DIG_T1*2) * 8
    int32_t adc_T = ((t_fine * 2048 / BMP_DIG_T2) + (int32_t)BMP_DIG_T1 * 2) * 8;
    if (adc_T < 0) adc_T = 0;
    if (adc_T > 0xFFFFF) adc_T = 0xFFFFF;  // 20-bit
    return (uint32_t)adc_T;
}

// Pressure: use a fixed nominal ADC value near sea level for now,
// shifted slightly with the dynamic pressure value.
// Full inversion of the BMP280 pressure compensation is complex;
// we use an empirical linear mapping calibrated at ~1013 hPa.
static uint32_t bmp280_press_to_adc(int32_t p10, int32_t t10) {
    // With the fixed calibration constants above, adc_P near 415000
    // corresponds to ~1013 hPa when T=25 C.
    // We scale linearly: delta_p10 -> delta_adc ~= 38
    int32_t base_p10 = 10132;  // 1013.2 hPa
    int32_t base_adc = 415000;
    int32_t adc_P    = base_adc + (int32_t)((p10 - base_p10) * 38);
    (void)t10;
    if (adc_P < 0) adc_P = 0;
    if (adc_P > 0xFFFFF) adc_P = 0xFFFFF;
    return (uint32_t)adc_P;
}

static void bmp280_init(sim_ctx_t *ctx) {
    SoftI2C_Sim_SetValue(ctx, SIM_Q_TEMPERATURE, 250, 180, 450,  3);
    SoftI2C_Sim_SetValue(ctx, SIM_Q_PRESSURE,  10132, 9800, 10400, 5);
    SoftI2C_Sim_SetValue(ctx, SIM_Q_HUMIDITY,    500, 200,  900,  4);

    bmp280_state_t *s = (bmp280_state_t *)malloc(sizeof(bmp280_state_t));
    s->reg        = 0;
    s->is_bme280  = false;   // change to true for BME280 simulation
    ctx->user     = s;
}

static bool bmp280_encode(sim_ctx_t *ctx) {
    bmp280_state_t *s = (bmp280_state_t *)ctx->user;
    if (!s || ctx->cmd_len < 1) return false;

    uint8_t reg = ctx->cmd[0];

    // Register write (2 bytes: reg + value) – just acknowledge
    if (ctx->cmd_len >= 2) {
        s->reg = reg;
        ctx->resp_len = 0;
        return true;
    }

    // Register read: remember the register, response comes on the
    // following read Start.
    s->reg = reg;

    if (reg == 0xD0) {
        // Chip ID
        ctx->resp[0]  = s->is_bme280 ? 0x60 : 0x60;  // both are 0x60 in practice
        ctx->resp_len = 1;
        return true;
    }

    if (reg == 0x88) {
        // Calibration data (24 bytes)
        bmp280_pack_calib(ctx->resp);
        ctx->resp_len = 24;
        return true;
    }

    if (reg == 0xF3) {
        // Status: measuring=0, im_update=0 -> ready
        ctx->resp[0]  = 0x00;
        ctx->resp_len = 1;
        return true;
    }

    if (reg == 0xF4) {
        // ctrl_meas: mode=11 (normal), osrs=001 (x1)
        ctx->resp[0]  = 0x27;
        ctx->resp_len = 1;
        return true;
    }

    if (reg == 0xF7) {
        // Raw ADC data: press[19:0] @ F7-F9, temp[19:0] @ FA-FC
        int32_t t10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_TEMPERATURE);
        int32_t p10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_PRESSURE);
        uint32_t adc_P = bmp280_press_to_adc(p10, t10);
        uint32_t adc_T = bmp280_temp_to_adc(t10);

        // Pressure: [F7]=MSB [F8]=LSB [F9]=XLSB (bits 7:4 significant)
        ctx->resp[0] = (uint8_t)((adc_P >> 12) & 0xFF);
        ctx->resp[1] = (uint8_t)((adc_P >>  4) & 0xFF);
        ctx->resp[2] = (uint8_t)((adc_P <<  4) & 0xF0);
        // Temperature: [FA]=MSB [FB]=LSB [FC]=XLSB
        ctx->resp[3] = (uint8_t)((adc_T >> 12) & 0xFF);
        ctx->resp[4] = (uint8_t)((adc_T >>  4) & 0xFF);
        ctx->resp[5] = (uint8_t)((adc_T <<  4) & 0xF0);

        if (s->is_bme280) {
            int32_t h10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_HUMIDITY);
            // raw_H = h_percent * 2^16 / 100  = h10 * 65536 / 1000
            uint32_t raw_H = (uint32_t)((int64_t)h10 * 65536 / 1000);
            if (raw_H > 0xFFFF) raw_H = 0xFFFF;
            ctx->resp[6] = (uint8_t)(raw_H >> 8);
            ctx->resp[7] = (uint8_t)(raw_H & 0xFF);
            ctx->resp_len = 8;
        } else {
            ctx->resp_len = 6;
        }

        printf("[SIM][BMP280] T=%d.%d C  P=%d.%d hPa  adc_T=0x%05X adc_P=0x%05X\n",
               (int)(t10/10), (int)abs(t10%10),
               (int)(p10/10), (int)(p10%10), adc_T, adc_P);
        return true;
    }

    // Unknown register – return 0x00
    ctx->resp[0]  = 0x00;
    ctx->resp_len = 1;
    return true;
}

static const sim_sensor_ops_t g_bmp280_ops = {
    .name             = "BMP280",
    .init             = bmp280_init,
    .encode_response  = bmp280_encode,
    .on_read_complete = NULL,
};

// ===================================================================
// CHT83xx plugin  (CHT8305, CHT8310, CHT8315)
// ===================================================================
// Protocol (from drv_cht8305.c + drv_cht83xx.h):
//   Write 0x00          -> set register pointer to temp (CHT831X_REG_TEMP)
//   Write 0x01          -> set register pointer to hum  (CHT831X_REG_HUM)
//   Write 0x07 + 2B     -> write config register
//   Write 0x0F          -> read manufacturer ID
//   Write 0x11 + 2B     -> write one-shot register
//   Read  4 bytes       -> [T_MSB T_LSB H_MSB H_LSB]  (from reg 0x00)
//   Read  2 bytes       -> [H_MSB H_LSB]               (from reg 0x01)
//   Read  2 bytes       -> [ID_MSB ID_LSB]             (from reg 0x0F)
//
// Physical encoding:
//   CHT8305 (default):
//     raw_T = (temp + 40) * 65535 / 165    (float path)
//     raw_H = humid * 65535 / 100
//   CHT831X (8310/8315):
//     raw_T = temp / 0.03125               (13-bit, signed, >> 3)
//     raw_H = humid * 32768 / 100          (15-bit, sign bit = parity)
// ===================================================================

typedef struct {
    uint8_t  reg;        // current register pointer
    uint16_t sensor_id;  // 0x0000=CHT8305, 0x8215=CHT8310, 0x8315=CHT8315
} cht83xx_state_t;

static void cht83xx_init(sim_ctx_t *ctx) {
    SoftI2C_Sim_SetValue(ctx, SIM_Q_TEMPERATURE, 220, 150, 400,  3);
    SoftI2C_Sim_SetValue(ctx, SIM_Q_HUMIDITY,    500, 200, 900,  5);

    cht83xx_state_t *s = (cht83xx_state_t *)malloc(sizeof(cht83xx_state_t));
    s->reg       = 0x00;
    s->sensor_id = 0x8215;  // default: CHT8310; change to 0 for CHT8305
    ctx->user    = s;
}

static bool cht83xx_encode(sim_ctx_t *ctx) {
    cht83xx_state_t *s = (cht83xx_state_t *)ctx->user;
    if (!s) return false;

    // Zero-length write = just a read Start, serve from current reg
    if (ctx->cmd_len == 0) {
        goto serve_reg;
    }

    uint8_t reg = ctx->cmd[0];
    s->reg = reg;

    // Multi-byte write (register write, e.g. config, one-shot)
    if (ctx->cmd_len > 1) {
        ctx->resp_len = 0;  // just ACK the write
        return true;
    }

    // Single-byte write = register pointer set; data comes on next read
    // fall through to serve_reg

serve_reg:
    {
        int32_t t10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_TEMPERATURE);
        int32_t h10 = SoftI2C_Sim_NextValue(ctx, SIM_Q_HUMIDITY);
        bool is_831x = (s->sensor_id == 0x8215 || s->sensor_id == 0x8315);

        if (s->reg == 0x00) {
            // Temperature (+ humidity for CHT8305 combined read)
            uint16_t raw_t, raw_h;
            if (is_831x) {
                // 13-bit signed, >>3 -> multiply back
                int16_t t_13 = (int16_t)((float)(t10/10.0f) / 0.03125f);
                raw_t = (uint16_t)(t_13 << 3);
                // 15-bit humidity with parity in bit15
                uint16_t h_15 = (uint16_t)((int64_t)h10 * 32768 / 1000);
                // compute parity of bits[14:0]
                uint8_t par = 0;
                for (int b = 0; b < 15; b++) par ^= (h_15 >> b) & 1;
                raw_h = (h_15 & 0x7FFF) | (par ? 0x8000 : 0);
            } else {
                raw_t = (uint16_t)(((float)(t10/10.0f) + 40.0f) * 65535.0f / 165.0f);
                raw_h = (uint16_t)((float)(h10/10.0f) * 65535.0f / 100.0f);
            }
            ctx->resp[0] = (uint8_t)(raw_t >> 8);
            ctx->resp[1] = (uint8_t)(raw_t & 0xFF);
            ctx->resp[2] = (uint8_t)(raw_h >> 8);
            ctx->resp[3] = (uint8_t)(raw_h & 0xFF);
            ctx->resp_len = 4;
            printf("[SIM][CHT83xx] T=%d.%d C  H=%d.%d%%  raw_T=0x%04X raw_H=0x%04X\n",
                   (int)(t10/10), (int)abs(t10%10),
                   (int)(h10/10), (int)(h10%10), raw_t, raw_h);
            return true;
        }

        if (s->reg == 0x01) {
            // Humidity only (CHT831X separate read)
            int32_t h10b = SoftI2C_Sim_PeekValue(ctx, SIM_Q_HUMIDITY); // don't advance again
            uint16_t h_15 = (uint16_t)((int64_t)h10b * 32768 / 1000);
            uint8_t par = 0;
            for (int b = 0; b < 15; b++) par ^= (h_15 >> b) & 1;
            uint16_t raw_h = (h_15 & 0x7FFF) | (par ? 0x8000 : 0);
            ctx->resp[0] = (uint8_t)(raw_h >> 8);
            ctx->resp[1] = (uint8_t)(raw_h & 0xFF);
            ctx->resp_len = 2;
            return true;
        }

        if (s->reg == 0x0F || s->reg == 0x10) {
            // Manufacturer / sensor ID
            // drv reads 0x0F for 2 bytes (mfr ID) then 0x10 for 2 bytes (sensor ID)
            uint16_t id = (s->reg == 0x0F) ? 0x5453 : s->sensor_id;
            ctx->resp[0] = (uint8_t)(id >> 8);
            ctx->resp[1] = (uint8_t)(id & 0xFF);
            ctx->resp_len = 2;
            return true;
        }

        if (s->reg == 0x02) {
            // Status register (CHT831X) – no alerts active
            ctx->resp[0]  = 0x00;
            ctx->resp_len = 1;
            return true;
        }

        // Unknown register
        ctx->resp[0]  = 0x00;
        ctx->resp[1]  = 0x00;
        ctx->resp_len = 2;
        return true;
    }
}

static const sim_sensor_ops_t g_cht83xx_ops = {
    .name             = "CHT83xx",
    .init             = cht83xx_init,
    .encode_response  = cht83xx_encode,
    .on_read_complete = NULL,
};

// ===================================================================
// Convenience registration functions
// ===================================================================


commandResult_t CMD_SoftI2C_simAddSensor(const void* context, const char* cmd, const char* args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	
	
	uint8_t pin_data=9, pin_clk=17;
	pin_clk   = (uint8_t)Tokenizer_GetPinEqual("SCL=", pin_clk);
	pin_data  = (uint8_t)Tokenizer_GetPinEqual("SDA=", pin_data);
	const char *type = Tokenizer_GetArgEqualDefault("type=","NO");
	uint8_t def_addr;
	sim_sensor_ops_t *sens_ops;
	if (!strcmp(type,"NO")){
		ADDLOG_ERROR(LOG_FEATURE_SENSOR, "No sensor type given!");
		return CMD_RES_BAD_ARGUMENT;
	}else {
        	if (strcasecmp(type, "SHT3x") == 0) {
			printf("Detected: SHT3x\n");
			sens_ops = &g_sht3x_ops;
			def_addr = 0x44 << 1;
		} else if (strcasecmp(type, "SHT4x") == 0) {
			printf("Detected: SHT4x\n");
			sens_ops = &g_sht4x_ops;
			def_addr = 0x44 << 1;
		} else if (strcasecmp(input, "AHT2x") == 0) {
			printf("Detected: AHT2x\n");
			sens_ops = &g_aht2x_ops;
			def_addr = 0x38 << 1;
		} else if (strcasecmp(input, "CHT83xx") == 0) {
			printf("Detected: CHT83xx\n");
			sens_ops = &g_cht83xx_ops;
			def_addr = 0x40 << 1;
		} else if (strcasecmp(input, "BMP280") == 0) {
			printf("Detected: BMP280\n");
			sens_ops = &g_bmp280_ops;
			def_addr = 0x58 << 1;		// to be dicussed, what is "default"
		} else {
			printf("Unknown sensor type %s.\n",type);
		}
        }
        uint8_t A = (int8_t)(Tokenizer_GetArgEqualInteger("adress=", 0));
        if (A != 0){
        	dev->i2cAddr = A << 1;
        } else {
        	dev->i2cAddr = def_addr; 
        }
	SoftI2C_Sim_Register(pin_data, pin_clk, addr, sens_ops);
	return CMD_RES_OK;
}

int SoftI2C_Sim_AddSHT3x(uint8_t pin_data, uint8_t pin_clk, uint8_t addr) {
    return SoftI2C_Sim_Register(pin_data, pin_clk, addr, &g_sht3x_ops);
}

int SoftI2C_Sim_AddSHT4x(uint8_t pin_data, uint8_t pin_clk, uint8_t addr) {
    return SoftI2C_Sim_Register(pin_data, pin_clk, addr, &g_sht4x_ops);
}

int SoftI2C_Sim_AddAHT2x(uint8_t pin_data, uint8_t pin_clk) {
    return SoftI2C_Sim_Register(pin_data, pin_clk, 0x38, &g_aht2x_ops);
}

int SoftI2C_Sim_AddBMP280(uint8_t pin_data, uint8_t pin_clk, uint8_t addr) {
    return SoftI2C_Sim_Register(pin_data, pin_clk, addr, &g_bmp280_ops);
}

int SoftI2C_Sim_AddCHT83xx(uint8_t pin_data, uint8_t pin_clk, uint8_t addr) {
    return SoftI2C_Sim_Register(pin_data, pin_clk, addr, &g_cht83xx_ops);
}

#endif // WIN32
