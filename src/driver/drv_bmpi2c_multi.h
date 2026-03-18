// drv_bmpi2c.h  –  BMP085/180/280, BME280/680/688 chip logic
//
// All functions take an explicit bmp_ctx_t* — no translation-unit globals,
// no bus-swapping ceremony.  Multiple sensors on different buses work by
// simply using different context pointers.
//
// This header is included by exactly one .c file (drv_bmpi2c.c).
// All functions are static inline to avoid multiple-definition errors if
// the header is ever included elsewhere by accident.
//
// Fixed vs previous version:
//  [1]  #pragma once → portable include guard
//  [2]  Added _bmp_r16be() for BMP180 big-endian calibration and ADC reads
//  [3]  BMP180 calibration now uses _bmp_r16be (was _bmp_r16le — byte-swapped)
//  [4]  BMP180 ADC temperature read now uses _bmp_r16be (was _bmp_r16le)
//  [5]  BME280 H4/H5 sign-extended correctly from 12-bit two's complement
//  [6]  BME280 H5 register source corrected: 0xE5[7:4] + 0xE6[7:0]
//  [7]  BMX280 pressure output: removed erroneous /256 (result was already Pa)
//  [8]  Pressure unit throughout: Pa×100 stored; HTTP display uses ×0.0001 for hPa
//  [9]  BME68X range_sw_err: cast moved after shift (was UB on signed shift)
// [10]  BMP_ReadRaw BMX280: reads 8 bytes always, extracts H from d[6..7]
//       (was reading 6 or 8 but always extracting from d[6] which is in range)
// [11]  BMX280 forced mode: BMP_ReadRaw re-triggers measurement before reading
// [12]  BMP_Humidity BME280: integer rounding added to /1024 step
// [13]  BMP_Configure: comment added to explain ctrl_hum-before-ctrl_meas ordering

#ifndef DRV_BMPI2C_MULTI_H
#define DRV_BMPI2C_MULTI_H

#include "../new_pins.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include <stdint.h>
#include <limits.h>

// ---------------------------------------------------------------------------
// Chip IDs
// ---------------------------------------------------------------------------
#define BMP_ID_180      0x55
#define BMP_ID_280_S1   0x56
#define BMP_ID_280_S2   0x57
#define BMP_ID_280      0x58
#define BME_ID_280      0x60
#define BME_ID_68X      0x61

// ---------------------------------------------------------------------------
// I²C addresses (8-bit form, LSB = R/W bit)
// ---------------------------------------------------------------------------
#define BMP_ADDR_MAIN   (0x77 << 1)
#define BMP_ADDR_ALT    (0x76 << 1)

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------
#define REG_CHIP_ID     0xD0
#define REG_RESET       0xE0
#define REG_CTRL        0xF4        // ctrl_meas on BMX280, ctrl_meas on BMP180
#define SOFT_RESET_VAL  0xB6

// BMX280
#define REG_BMX_STATUS  0xF3
#define REG_BMX_CONFIG  0xF5
#define REG_BMX_PDATA   0xF7        // burst: press[2] temp[2] hum[2] = 8 bytes for BME280
#define REG_BME_CTRLHUM 0xF2        // must write before ctrl_meas (BME280 datasheet §5.4.3)

// BMP180
#define REG_180_CALIB   0xAA        // first of 22 big-endian calibration bytes
#define REG_180_ADC_MSB 0xF6
#define BMP180_TEMP_CMD 0x2E

// BME68X
#define REG_68X_COEFF1  0x89
#define REG_68X_COEFF2  0xE1
#define REG_68X_CONFIG  0x75
#define REG_68X_CTRLM   0x74
#define REG_68X_CTRLH   0x72
#define REG_68X_CTRLG1  0x71
#define REG_68X_CTRLG0  0x70
#define REG_68X_FIELD0  0x1D

// ---------------------------------------------------------------------------
// Enumerations  (values match original driver for config compatibility)
// ---------------------------------------------------------------------------
typedef enum { MODE_SLEEP=0, MODE_FORCED=1, MODE_NORMAL=3 } BMP_mode;
typedef enum { SAMPLING_SKIPPED=0, SAMPLING_X1, SAMPLING_X2,
               SAMPLING_X4, SAMPLING_X8, SAMPLING_X16 } BMP_sampling;
typedef enum { FILTER_OFF=0, FILTER_2X, FILTER_4X, FILTER_8X, FILTER_16X,
               FILTER_32X, FILTER_64X, FILTER_128X } BMP_filter;
typedef enum { STANDBY_0_5=0, STANDBY_62_5, STANDBY_125, STANDBY_250,
               STANDBY_500, STANDBY_1000, STANDBY_2000, STANDBY_4000 } BMP_standby;
typedef enum { BMP180_ULP=0, BMP180_STD, BMP180_HI, BMP180_UHR } BMP180_res;

// ---------------------------------------------------------------------------
// Calibration storage  (all per-context; no global structs)
// ---------------------------------------------------------------------------
typedef struct {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2,P3,P4,P5,P6,P7,P8,P9;
    // H1,H3 are unsigned; H2,H4,H5 signed; H6 signed
    uint8_t H1, H3; int16_t H2, H4, H5; int8_t H6;
} bmx280_calib_t;

typedef struct {
    // All BMP180 calibration words are big-endian on the wire.
    // Signed/unsigned per BMP180 datasheet Table 4.
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1, B2, MB, MC, MD;
} bmp180_calib_t;

typedef struct {
    uint16_t T1, T2; uint8_t T3;
    uint16_t P1; int16_t P2; int8_t P3; int16_t P4, P5;
    int8_t   P6, P7; int16_t P8, P9; int8_t P10;
    uint16_t H1, H2; int8_t H3, H4, H5; uint8_t H6; int8_t H7;
    int8_t   GH1; int16_t GH2; int8_t GH3;
    BMP_sampling t_samp, p_samp, h_samp;
    uint8_t  res_heat_range; int8_t res_heat_val, range_sw_err;
} bme68x_calib_t;

// ---------------------------------------------------------------------------
// Chip context  –  one instance per physical sensor
// ---------------------------------------------------------------------------
typedef struct {
    softI2C_t      i2c;
    uint8_t        chip_id;
    bool           has_humidity;
    BMP180_res     bmp180_res;
    BMP_mode       mode;           // remembered to re-trigger forced-mode reads
    int32_t        adc_T, adc_P, adc_H, t_fine;
    bmx280_calib_t bmx280;
    bmp180_calib_t bmp180;
    bme68x_calib_t bme68x;
} bmp_ctx_t;

// Convenience predicates on the context's chip_id
#define IS_BMP180(c)  ((c)->chip_id == BMP_ID_180)
#define IS_BMX280(c)  ((c)->chip_id == BMP_ID_280_S1 || \
                       (c)->chip_id == BMP_ID_280_S2 || \
                       (c)->chip_id == BMP_ID_280    || \
                       (c)->chip_id == BME_ID_280)
#define IS_BME280(c)  ((c)->chip_id == BME_ID_280)
#define IS_BME68X(c)  ((c)->chip_id == BME_ID_68X)

// ---------------------------------------------------------------------------
// I²C primitives
//
// BMX280 / BME68X use little-endian 16-bit words.
// BMP180 uses big-endian 16-bit words (calibration + ADC).
// All use auto-increment burst reads.
// ---------------------------------------------------------------------------
static inline void _bmp_w8(bmp_ctx_t *c, uint8_t reg, uint8_t val)
{
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit);
    Soft_I2C_WriteByte(&c->i2c, reg);
    Soft_I2C_WriteByte(&c->i2c, val);
    Soft_I2C_Stop(&c->i2c);
}

static inline uint8_t _bmp_r8(bmp_ctx_t *c, uint8_t reg)
{
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit);
    Soft_I2C_WriteByte(&c->i2c, reg);
    Soft_I2C_Stop(&c->i2c);
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit | 1);
    uint8_t v = Soft_I2C_ReadByte(&c->i2c, 1);  // NACK — last byte
    Soft_I2C_Stop(&c->i2c);
    return v;
}

// FIX [2]: little-endian 16-bit read (BMX280 / BME68X calibration)
static inline uint16_t _bmp_r16le(bmp_ctx_t *c, uint8_t reg)
{
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit);
    Soft_I2C_WriteByte(&c->i2c, reg);
    Soft_I2C_Stop(&c->i2c);
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit | 1);
    uint8_t lo = Soft_I2C_ReadByte(&c->i2c, 0);  // ACK
    uint8_t hi = Soft_I2C_ReadByte(&c->i2c, 1);  // NACK
    Soft_I2C_Stop(&c->i2c);
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

// FIX [2]: big-endian 16-bit read (BMP180 calibration and ADC)
static inline uint16_t _bmp_r16be(bmp_ctx_t *c, uint8_t reg)
{
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit);
    Soft_I2C_WriteByte(&c->i2c, reg);
    Soft_I2C_Stop(&c->i2c);
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit | 1);
    uint8_t hi = Soft_I2C_ReadByte(&c->i2c, 0);  // ACK
    uint8_t lo = Soft_I2C_ReadByte(&c->i2c, 1);  // NACK
    Soft_I2C_Stop(&c->i2c);
    return ((uint16_t)hi << 8) | (uint16_t)lo;
}

// Burst read: n bytes starting at reg into buf (auto-increment)
static inline void _bmp_rn(bmp_ctx_t *c, uint8_t reg, uint8_t *buf, uint8_t n)
{
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit);
    Soft_I2C_WriteByte(&c->i2c, reg);
    Soft_I2C_Stop(&c->i2c);
    Soft_I2C_Start(&c->i2c, c->i2c.address8bit | 1);
    Soft_I2C_ReadBytes(&c->i2c, buf, n);
    Soft_I2C_Stop(&c->i2c);
}

// ---------------------------------------------------------------------------
// User-value → enum helpers
// ---------------------------------------------------------------------------
static inline BMP_mode BMP_GetMode(int v)
{
    return (v == 1) ? MODE_FORCED : (v == 2) ? MODE_SLEEP : MODE_NORMAL;
}

#ifndef WINDOWS
static inline BMP_sampling BMP_GetSampling(int v)
{
    switch(v) {
    case -1:             return SAMPLING_SKIPPED;
    case  2:             return SAMPLING_X2;
    case  3 ...  5:      return SAMPLING_X4;
    case  6 ... 11:      return SAMPLING_X8;
    case 12 ... INT_MAX: return SAMPLING_X16;
    default:             return SAMPLING_X1;
    }
}
static inline BMP_filter BMP_GetFilter(int v)
{
    switch(v) {
    case  1 ...  2:      return FILTER_2X;
    case  3 ...  5:      return FILTER_4X;
    case  6 ... 11:      return FILTER_8X;
    case 12 ... 23:      return FILTER_16X;
    case 24 ... 47:      return FILTER_32X;
    case 48 ... 95:      return FILTER_64X;
    case 96 ... INT_MAX: return FILTER_128X;
    default:             return FILTER_OFF;
    }
}
static inline BMP_standby BMP_GetStandby(int v)
{
    switch(v) {
    case  32 ...   94:   return STANDBY_62_5;
    case  95 ...  174:   return STANDBY_125;
    case 175 ...  374:   return STANDBY_250;
    case 375 ...  750:   return STANDBY_500;
    case 751 ...  1500:  return STANDBY_1000;
    case 1501 ... 3000:  return STANDBY_2000;
    case 3001 ... INT_MAX: return STANDBY_4000;
    default:             return STANDBY_0_5;
    }
}
#else
static inline BMP_sampling BMP_GetSampling(int v)
{
    if(v==-1) return SAMPLING_SKIPPED;
    if(v== 2) return SAMPLING_X2;
    if(v>= 3 && v<= 5) return SAMPLING_X4;
    if(v>= 6 && v<=11) return SAMPLING_X8;
    if(v>=12)          return SAMPLING_X16;
    return SAMPLING_X1;
}
static inline BMP_filter BMP_GetFilter(int v)
{
    if(v<= 0) return FILTER_OFF;
    if(v<= 2) return FILTER_2X;
    if(v<= 5) return FILTER_4X;
    if(v<=11) return FILTER_8X;
    if(v<=23) return FILTER_16X;
    if(v<=47) return FILTER_32X;
    if(v<=95) return FILTER_64X;
    return FILTER_128X;
}
static inline BMP_standby BMP_GetStandby(int v)
{
    if(v<= 31)  return STANDBY_0_5;
    if(v<= 94)  return STANDBY_62_5;
    if(v<=174)  return STANDBY_125;
    if(v<=374)  return STANDBY_250;
    if(v<=750)  return STANDBY_500;
    if(v<=1500) return STANDBY_1000;
    if(v<=3000) return STANDBY_2000;
    return STANDBY_4000;
}
#endif  // WINDOWS

// ---------------------------------------------------------------------------
// Calibration readers
// ---------------------------------------------------------------------------
static inline void _calib_bmx280(bmp_ctx_t *c)
{
    bmx280_calib_t *k = &c->bmx280;
    // Pressure and temperature calib: all little-endian, contiguous from 0x88
    k->T1 = _bmp_r16le(c, 0x88); k->T2 = _bmp_r16le(c, 0x8A); k->T3 = _bmp_r16le(c, 0x8C);
    k->P1 = _bmp_r16le(c, 0x8E); k->P2 = _bmp_r16le(c, 0x90); k->P3 = _bmp_r16le(c, 0x92);
    k->P4 = _bmp_r16le(c, 0x94); k->P5 = _bmp_r16le(c, 0x96); k->P6 = _bmp_r16le(c, 0x98);
    k->P7 = _bmp_r16le(c, 0x9A); k->P8 = _bmp_r16le(c, 0x9C); k->P9 = _bmp_r16le(c, 0x9E);

    if(IS_BME280(c)) {
        k->H1 = _bmp_r8(c, 0xA1);
        k->H2 = (int16_t)_bmp_r16le(c, 0xE1);  // signed
        k->H3 = _bmp_r8(c, 0xE3);

        // FIX [5,6]: H4 = 0xE4[7:0] << 4 | 0xE5[3:0]   (signed 12-bit)
        //            H5 = 0xE5[7:4]       | 0xE6[7:0]<<4 (signed 12-bit)
        // Must sign-extend from 12 bits into int16.
        uint8_t e4 = _bmp_r8(c, 0xE4);
        uint8_t e5 = _bmp_r8(c, 0xE5);
        uint8_t e6 = _bmp_r8(c, 0xE6);
        // Assemble as unsigned 12-bit, then sign-extend
        uint16_t raw_h4 = ((uint16_t)e4 << 4) | (e5 & 0x0F);
        uint16_t raw_h5 = ((uint16_t)e6 << 4) | (e5 >> 4);
        k->H4 = (raw_h4 & 0x0800) ? (int16_t)(raw_h4 | 0xF000) : (int16_t)raw_h4;
        k->H5 = (raw_h5 & 0x0800) ? (int16_t)(raw_h5 | 0xF000) : (int16_t)raw_h5;

        k->H6 = (int8_t)_bmp_r8(c, 0xE7);
    }
}

static inline void _calib_bmp180(bmp_ctx_t *c)
{
    // FIX [3]: BMP180 calibration is big-endian — must use _bmp_r16be.
    // Read all 22 bytes in one burst to minimise I²C transactions.
    uint8_t b[22];
    _bmp_rn(c, REG_180_CALIB, b, 22);
    bmp180_calib_t *k = &c->bmp180;
    // Datasheet Table 4: AC1..AC6 signed/unsigned, B1..MD signed
    k->AC1 = (int16_t) ((uint16_t)b[ 0]<<8 | b[ 1]);
    k->AC2 = (int16_t) ((uint16_t)b[ 2]<<8 | b[ 3]);
    k->AC3 = (int16_t) ((uint16_t)b[ 4]<<8 | b[ 5]);
    k->AC4 = (uint16_t)((uint16_t)b[ 6]<<8 | b[ 7]);
    k->AC5 = (uint16_t)((uint16_t)b[ 8]<<8 | b[ 9]);
    k->AC6 = (uint16_t)((uint16_t)b[10]<<8 | b[11]);
    k->B1  = (int16_t) ((uint16_t)b[12]<<8 | b[13]);
    k->B2  = (int16_t) ((uint16_t)b[14]<<8 | b[15]);
    k->MB  = (int16_t) ((uint16_t)b[16]<<8 | b[17]);
    k->MC  = (int16_t) ((uint16_t)b[18]<<8 | b[19]);
    k->MD  = (int16_t) ((uint16_t)b[20]<<8 | b[21]);
}

static inline void _calib_bme68x(bmp_ctx_t *c)
{
    uint8_t b1[24], b2[16];
    _bmp_rn(c, REG_68X_COEFF1, b1, 24);
    _bmp_rn(c, REG_68X_COEFF2, b2, 16);
    bme68x_calib_t *k = &c->bme68x;

    k->T1 = (uint16_t)b2[9]<<8  | b2[8];
    k->T2 = (int16_t)((uint16_t)b1[2]<<8 | b1[1]);
    k->T3 = (int8_t)b1[3];

    k->P1  = (uint16_t)b1[6]<<8  | b1[5];
    k->P2  = (int16_t)((uint16_t)b1[8]<<8  | b1[7]);
    k->P3  = (int8_t)b1[9];
    k->P4  = (int16_t)((uint16_t)b1[12]<<8 | b1[11]);
    k->P5  = (int16_t)((uint16_t)b1[14]<<8 | b1[13]);
    k->P6  = (int8_t)b1[16];
    k->P7  = (int8_t)b1[15];
    k->P8  = (int16_t)((uint16_t)b1[20]<<8 | b1[19]);
    k->P9  = (int16_t)((uint16_t)b1[22]<<8 | b1[21]);
    k->P10 = (int8_t)b1[23];

    k->H1 = (uint16_t)b2[2]<<4 | (b2[1] & 0x0F);
    k->H2 = (uint16_t)b2[0]<<4 | (b2[1] >> 4);
    k->H3 = (int8_t)b2[3];
    k->H4 = (int8_t)b2[4];
    k->H5 = (int8_t)b2[5];
    k->H6 = b2[6];
    k->H7 = (int8_t)b2[7];

    k->GH1 = (int8_t)b2[14];
    k->GH2 = (int16_t)((uint16_t)b2[12]<<8 | b2[13]);
    k->GH3 = (int8_t)b2[15];

    k->res_heat_range = (_bmp_r8(c, 0x02) & 0x30) >> 4;
    k->res_heat_val   = (int8_t)_bmp_r8(c, 0x00);
    // FIX [9]: cast after shift so the shift is on an unsigned value (no UB)
    k->range_sw_err   = (int8_t)((_bmp_r8(c, 0x04) & 0xF0) >> 4);
}

// ---------------------------------------------------------------------------
// Probe + init  —  returns chip name string on success, NULL on failure
// ---------------------------------------------------------------------------
static inline const char *BMP_Init(bmp_ctx_t *c)
{
    c->chip_id      = _bmp_r8(c, REG_CHIP_ID);
    c->has_humidity = false;
    c->bmp180_res   = BMP180_UHR;
    c->mode         = MODE_NORMAL;

    const char *name = NULL;
    switch(c->chip_id) {
    case BMP_ID_180:                          name = "BMP180"; break;
    case BMP_ID_280_S1: case BMP_ID_280_S2:
    case BMP_ID_280:                          name = "BMP280"; break;
    case BME_ID_280:  c->has_humidity = true; name = "BME280"; break;
    case BME_ID_68X:  c->has_humidity = true; name = "BME68X"; break;
    default:
        ADDLOG_WARN(LOG_FEATURE_SENSOR, "BMP: unknown chip 0x%02X", c->chip_id);
        // Courtesy probe of the alternate address to help with misconfiguration
        uint8_t alt = (c->i2c.address8bit == BMP_ADDR_MAIN) ? BMP_ADDR_ALT : BMP_ADDR_MAIN;
        Soft_I2C_Start(&c->i2c, alt);
        Soft_I2C_WriteByte(&c->i2c, REG_CHIP_ID);
        Soft_I2C_Stop(&c->i2c);
        Soft_I2C_Start(&c->i2c, alt | 1);
        uint8_t alt_id = Soft_I2C_ReadByte(&c->i2c, 1);
        Soft_I2C_Stop(&c->i2c);
        if(alt_id != 0xFF)
            ADDLOG_WARN(LOG_FEATURE_SENSOR,
                        "BMP: chip found at alt addr 0x%02X (id=0x%02X) — fix addr= in config",
                        alt >> 1, alt_id);
        return NULL;
    }

    _bmp_w8(c, REG_RESET, SOFT_RESET_VAL);
    delay_ms(5);

    if(IS_BMX280(c)) {
        delay_ms(100);
        while(_bmp_r8(c, REG_BMX_STATUS) & 0x01) delay_ms(10);
        _calib_bmx280(c);
    } else if(IS_BMP180(c)) {
        _calib_bmp180(c);
    } else if(IS_BME68X(c)) {
        _calib_bme68x(c);
        delay_ms(5);
    }
    return name;
}

// ---------------------------------------------------------------------------
// Configure sampling / filter / standby
// ---------------------------------------------------------------------------
static inline void BMP_Configure(bmp_ctx_t *c,
                                  BMP_mode mode, BMP_sampling ts, BMP_sampling ps,
                                  BMP_sampling hs, BMP_filter f, BMP_standby sb)
{
    c->mode = mode;  // remember for forced-mode re-triggering in BMP_ReadRaw

    if(IS_BMX280(c)) {
        // FIX [13]: ctrl_hum MUST be written before ctrl_meas — BME280 DS §5.4.3:
        // "Changes to osrs_h[2:0] only become effective after a write operation
        //  to ctrl_meas."
        if(IS_BME280(c))
            _bmp_w8(c, REG_BME_CTRLHUM,
                    (_bmp_r8(c, REG_BME_CTRLHUM) & ~0x07u) | (hs & 0x07u));
        _bmp_w8(c, REG_BMX_CONFIG, (uint8_t)(((uint8_t)sb << 5) | ((uint8_t)f << 2)) & 0xFC);
        _bmp_w8(c, REG_CTRL,       (uint8_t)(((uint8_t)ts << 5) | ((uint8_t)ps << 2) | mode));
    } else if(IS_BME68X(c)) {
        c->bme68x.t_samp = ts;
        c->bme68x.p_samp = ps;
        c->bme68x.h_samp = hs;
        _bmp_w8(c, REG_68X_CONFIG,
                (_bmp_r8(c, REG_68X_CONFIG) & ~0x1Cu) | ((uint8_t)(f & 0x07u) << 2));
        _bmp_w8(c, REG_68X_CTRLH,
                (_bmp_r8(c, REG_68X_CTRLH) & ~0x07u) | (hs & 0x07u));
        _bmp_w8(c, REG_68X_CTRLG1,
                (_bmp_r8(c, REG_68X_CTRLG1) & ~0x1Fu) | (1u << 4));
        _bmp_w8(c, REG_68X_CTRLG0,
                (_bmp_r8(c, REG_68X_CTRLG0) & ~0x08u) | (1u << 3));
    }
    // BMP180: no config register — resolution lives in c->bmp180_res
}

// ---------------------------------------------------------------------------
// Read raw ADC values
//
// FIX [10,11]: BMX280 always reads 8 bytes from 0xF7 (covers P+T+H in one
//   burst regardless of chip variant — unused bytes are harmless).
//   In forced mode the measurement must be re-triggered before reading;
//   the sensor returns to sleep automatically after one conversion.
// ---------------------------------------------------------------------------
static inline void BMP_ReadRaw(bmp_ctx_t *c)
{
    if(IS_BMX280(c)) {
        // FIX [11]: re-trigger forced-mode measurement before reading
        if(c->mode == MODE_FORCED)
            _bmp_w8(c, REG_CTRL,
                    _bmp_r8(c, REG_CTRL) | 0x01);  // set mode bits to forced (01)

        // FIX [10]: always read 8 bytes; P=d[0..2], T=d[3..5], H=d[6..7]
        uint8_t d[8];
        _bmp_rn(c, REG_BMX_PDATA, d, 8);
        c->adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
        c->adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
        if(IS_BME280(c))
            c->adc_H = ((int32_t)d[6] << 8) | d[7];

    } else if(IS_BME68X(c)) {
        // BME68X is always forced-mode: write ctrl_meas to trigger, then wait
        bme68x_calib_t *k = &c->bme68x;
        uint8_t mc = ((uint8_t)(k->t_samp & 7) << 5) |
                     ((uint8_t)(k->p_samp & 7) << 2) | 0x01u;
        _bmp_w8(c, REG_68X_CTRLM, mc);

        // Measurement duration per BME68X datasheet §3.2.1 (µs, rounded up to ms)
        const uint8_t os_cyc[6] = {0, 1, 2, 4, 8, 16};
        uint32_t dur_us = ((uint32_t)os_cyc[k->t_samp] +
                           (uint32_t)os_cyc[k->p_samp] +
                           (uint32_t)os_cyc[k->h_samp]) * 1963u
                          + 477u * 4u + 477u * 5u + 500u;
        delay_ms(dur_us / 1000u + 1u);

        uint8_t d[14];
        _bmp_rn(c, REG_68X_FIELD0, d, 14);
        c->adc_T = ((int32_t)d[5] << 12) | ((int32_t)d[6] << 4) | (d[7] >> 4);
        c->adc_P = ((int32_t)d[2] << 12) | ((int32_t)d[3] << 4) | (d[4] >> 4);
        c->adc_H = ((int32_t)d[8] << 8)  |  (int32_t)d[9];
    }
    // BMP180: combined T+P read handled separately in BMP_ReadTP_180()
}

// ---------------------------------------------------------------------------
// Compensation math
//
// Output units: T = °C × 100   P = Pa × 100   H = %RH × 10
// [8] Pa×100 means 101325 Pa = 10132500.  Display as hPa: value * 0.0001f
//
// Always call BMP_Temperature() before BMP_Pressure() / BMP_Humidity()
// because it sets c->t_fine which the others depend on.
// ---------------------------------------------------------------------------
static inline int32_t BMP_Temperature(bmp_ctx_t *c)
{
    if(IS_BMX280(c)) {
        // Bosch BME280 datasheet §4.2.3 integer formula
        int32_t t1 = c->bmx280.T1, t2 = c->bmx280.T2, t3 = c->bmx280.T3;
        int32_t v1 = (((c->adc_T >> 3) - (t1 << 1)) * t2) >> 11;
        int32_t v2 = ((((c->adc_T >> 4) - t1) * ((c->adc_T >> 4) - t1)) >> 12) * t3 >> 14;
        c->t_fine = v1 + v2;
        // Datasheet: T_celsius = (t_fine * 5 + 128) >> 8  gives °C × 100
        return (c->t_fine * 5 + 128) >> 8;
    }
    if(IS_BME68X(c)) {
        // BME68X datasheet §4.2 (same polynomial form as BME280)
        int64_t v1 = ((int64_t)c->adc_T >> 3) - ((int64_t)c->bme68x.T1 << 1);
        v1 = (v1 * (int64_t)c->bme68x.T2) >> 11;
        int64_t v2 = (((int64_t)c->adc_T >> 4) - (int64_t)c->bme68x.T1);
        v2 = ((v2 * v2) >> 12) * (int64_t)c->bme68x.T3 >> 14;
        c->t_fine = (int32_t)(v1 + v2);
        return (c->t_fine * 5 + 128) >> 8;
    }
    return 0;  // BMP180: temperature computed in BMP_ReadTP_180()
}

static inline uint32_t BMP_Pressure(bmp_ctx_t *c)
{
    if(IS_BMX280(c)) {
        // Bosch BME280 datasheet §4.2.3 — 64-bit integer formula
        int64_t p1 = c->bmx280.P1, p2 = c->bmx280.P2, p3 = c->bmx280.P3,
                p4 = c->bmx280.P4, p5 = c->bmx280.P5, p6 = c->bmx280.P6,
                p7 = c->bmx280.P7, p8 = c->bmx280.P8, p9 = c->bmx280.P9;
        int64_t v1 = (int64_t)c->t_fine - 128000;
        int64_t v2 = v1 * v1 * p6 + ((v1 * p5) << 17) + (p4 << 35);
        v1 = ((v1 * v1 * p3) >> 8) + ((v1 * p2) << 12);
        v1 = (((int64_t)1 << 47) + v1) * p1 >> 33;
        if(!v1) return 0;
        int64_t p = 1048576 - (int64_t)c->adc_P;
        p = (((p << 31) - v2) * 3125) / v1;
        v1 = (p9 * (p >> 13) * (p >> 13)) >> 25;
        v2 = (p8 * p) >> 19;
        // FIX [7]: datasheet result p = ((p + v1 + v2) >> 8) + (p7 << 4) is in Pa (integer).
        // Was wrongly multiplied by 100/256; correct is *100 only to reach Pa×100.
        p = ((p + v1 + v2) >> 8) + (p7 << 4);  // Pa (integer)
        return (uint32_t)(p * 100);             // Pa×100
    }
    if(IS_BME68X(c)) {
        bme68x_calib_t *k = &c->bme68x;
        int32_t v1, v2, v3, v4, p;
        v1 = (c->t_fine >> 1) - 64000;
        v2 = (((v1 >> 2) * (v1 >> 2)) >> 11) * (int32_t)k->P6 >> 2;
        v2 = v2 + ((v1 * (int32_t)k->P5) << 1);
        v2 = (v2 >> 2) + ((int32_t)k->P4 << 16);
        v1 = (((((v1 >> 2) * (v1 >> 2)) >> 13) * ((int32_t)k->P3 << 5)) >> 3)
             + (((int32_t)k->P2 * v1) >> 1);
        v1 = v1 >> 18;
        v1 = ((32768 + v1) * (int32_t)k->P1) >> 15;
        if(!v1) return 0;
        p = 1048576 - c->adc_P;
        p = (int32_t)((uint32_t)(p - (v2 >> 12)) * 3125u);
        v4 = (int32_t)(1u << 31);
        p = (p >= v4) ? (int32_t)((uint32_t)p / (uint32_t)v1 << 1)
                      : (int32_t)((uint32_t)(p << 1) / (uint32_t)v1);
        v1 = ((int32_t)k->P9 * (int32_t)(((uint32_t)p >> 3) * ((uint32_t)p >> 3) >> 13)) >> 12;
        v2 = ((int32_t)(p >> 2) * (int32_t)k->P8) >> 13;
        v3 = ((int32_t)(p >> 8) * (int32_t)(p >> 8) * (int32_t)(p >> 8)
              * (int32_t)k->P10) >> 17;
        p = p + ((v1 + v2 + v3 + ((int32_t)k->P7 << 7)) >> 4);
        return (uint32_t)p * 100u;  // Pa×100
    }
    return 0;
}

static inline uint32_t BMP_Humidity(bmp_ctx_t *c)
{
    if(IS_BME280(c)) {
        if(c->adc_H == 0x8000) return 0;  // sensor skipped humidity
        bmx280_calib_t *k = &c->bmx280;
        int32_t v = c->t_fine - 76800;
        v = ((((c->adc_H << 14) - ((int32_t)k->H4 << 20) - ((int32_t)k->H5 * v))
              + 16384) >> 15) *
            ((((((v * (int32_t)k->H6) >> 10) *
                (((v * (int32_t)k->H3) >> 11) + 32768)) >> 10) + 2097152) *
             (int32_t)k->H2 + 8192) >> 14;
        v -= (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)k->H1) >> 4);
        v = (v < 0) ? 0 : (v > 419430400) ? 419430400 : v;
        // v is now %RH in Q22.10 format (i.e. v/1024 = %RH).
        // FIX [12]: add 512 before /1024 to round rather than truncate,
        // then *10 to reach %RH×10.
        return (uint32_t)(((v >> 12) * 10u + 512u) / 1024u);
    }
    if(IS_BME68X(c)) {
        bme68x_calib_t *k = &c->bme68x;
        int32_t ts = (c->t_fine * 5 + 128) >> 8;
        int32_t v1 = c->adc_H
                     - ((int32_t)((int32_t)k->H1 << 4))
                     - (((ts * (int32_t)k->H3) / 100) >> 1);
        int32_t v2 = ((int32_t)k->H2 *
                      (((ts * (int32_t)k->H4) / 100)
                       + (((ts * ((ts * (int32_t)k->H5) / 100)) >> 6) / 100)
                       + (int32_t)(1 << 14))) >> 10;
        int32_t v3 = v1 * v2;
        int32_t v4 = ((int32_t)k->H6 << 7) + ((ts * (int32_t)k->H7) / 100);
        v4 = (v4 >> 4);
        int32_t v5 = ((v3 >> 14) * (v3 >> 14)) >> 10;
        int32_t h  = (((v3 + ((v4 * v5) >> 1)) >> 10) * 1000) >> 12;
        h = (h > 100000) ? 100000 : (h < 0) ? 0 : h;
        // h is %RH × 1000; divide by 100 to get %RH × 10
        return (uint32_t)((h + 50) / 100);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// BMP180: combined temperature + pressure read
// (temperature compensation is required to compute pressure, so they're
//  inseparable for BMP180 — unlike BMX280 where they are separate ADC reads)
//
// FIX [2,4]: uses _bmp_rn for the burst ADC read (big-endian, MSB first);
//   temperature ADC also read via big-endian burst.
// Output: *out_t = °C×100,  *out_p = Pa×100
// ---------------------------------------------------------------------------
static inline void BMP_ReadTP_180(bmp_ctx_t *c, int32_t *out_t, uint32_t *out_p)
{
    bmp180_calib_t *k = &c->bmp180;

    // --- Temperature ---
    _bmp_w8(c, REG_CTRL, BMP180_TEMP_CMD);
    delay_ms(5);
    // FIX [4]: read temperature ADC big-endian (MSB at 0xF6, LSB at 0xF7)
    uint8_t tb[2];
    _bmp_rn(c, REG_180_ADC_MSB, tb, 2);
    int32_t UT = ((int32_t)tb[0] << 8) | tb[1];

    // --- Pressure ---
    static const uint8_t res_cmd[] = {0x34, 0x74, 0xB4, 0xF4};
    static const uint8_t res_dly[] = {5, 8, 14, 26};
    uint8_t res = (uint8_t)c->bmp180_res & 3u;
    _bmp_w8(c, REG_CTRL, res_cmd[res]);
    delay_ms(res_dly[res]);
    // Read 3 bytes big-endian: MSB, LSB, XLSB
    uint8_t pb[3];
    _bmp_rn(c, REG_180_ADC_MSB, pb, 3);
    int32_t UP = (((int32_t)pb[0] << 16) | ((int32_t)pb[1] << 8) | pb[2]) >> (8 - res);

    // --- Compensation — BMP085/180 datasheet §3.5 ---
    int32_t X1 = ((UT - (int32_t)k->AC6) * (int32_t)k->AC5) >> 15;
    int32_t X2 = ((int32_t)k->MC << 11) / (X1 + (int32_t)k->MD);
    int32_t B5 = X1 + X2;
    // Datasheet: T = (B5 + 8) >> 4  in units of 0.1 °C.  ×10 → °C×100.
    *out_t = ((B5 + 8) >> 4) * 10;

    int32_t B6 = B5 - 4000;
    X1 = ((int32_t)k->B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)k->AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)k->AC1 * 4 + X3) << res) + 2) >> 2;
    X1 = ((int32_t)k->AC3 * B6) >> 13;
    X2 = ((int32_t)k->B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = ((uint32_t)k->AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - (uint32_t)B3) * (uint32_t)(50000u >> res);
    if(!B4) { *out_p = 0; return; }
    int32_t p = (B7 < 0x80000000u) ? (int32_t)((B7 * 2u) / B4)
                                    : (int32_t)((B7 / B4) * 2u);
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);  // Pa (integer)
    *out_p = (uint32_t)p * 100u;       // Pa×100
}

#endif  // DRV_BMPI2C_MULTI_H
