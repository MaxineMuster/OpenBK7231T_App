// drv_bmpi2c.c  –  BMP085/180/280, BME280/680/688 multi-sensor driver
//
// startDriver BMPI2C [SCL=pin] [SDA=pin] [chan_t=ch] [chan_p=ch] [chan_h=ch] [addr=7bit]
// Legacy positional form: BMPI2C [clk] [data] [chan_t] [chan_p] [chan_h] [0|1]
// Add more sensors at runtime: BMPI2C_AddSensor SCL=pin SDA=pin ...

#include "../new_pins.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_bmpi2c_multi.h"
#include "../obk_config.h"

#if ENABLE_DRIVER_BMPI2C_MULTI

#ifndef BMPI2C_MAX_SENSORS
#  define BMPI2C_MAX_SENSORS 4
#endif

// ---------------------------------------------------------------------------
// Per-sensor driver state  (chip context + driver metadata)
// ---------------------------------------------------------------------------
typedef struct {
    bmp_ctx_t   chip;               // all chip state; no shared globals
    const char *name;               // e.g. "BME280"
    int32_t     calTemp;            // °C  × 100
    int32_t     calPres;            // Pa  × 100
    int32_t     calHum;             // %RH × 10
    int32_t     temperature;        // last reading, same units
    uint32_t    pressure;
    uint32_t    humidity;
    int8_t      chan_t, chan_p, chan_h;
    uint8_t     secondsBetween, secondsUntilNext;
    bool        ok;
} sensor_t;

static sensor_t g_dev[BMPI2C_MAX_SENSORS];
static uint8_t  g_n = 0;

#define CMD_UNUSED  (void)context; (void)cmdFlags

// ---------------------------------------------------------------------------
// Internal: sensor selector for commands with optional [sensorN] arg
// ---------------------------------------------------------------------------
static sensor_t *pick(const char *cmd, int idx, bool present)
{
    if(!present) return &g_dev[0];
    int n = Tokenizer_GetArgInteger(idx);
    if(n < 1 || n > (int)g_n) {
        ADDLOG_ERROR(LOG_FEATURE_SENSOR, "%s: sensor %i out of range (1..%i)", cmd, n, g_n);
        return NULL;
    }
    return &g_dev[n-1];
}

// ---------------------------------------------------------------------------
// Measure one sensor and publish to channels
// ---------------------------------------------------------------------------
static void Measure(sensor_t *s)
{
    bmp_ctx_t *c = &s->chip;

    if(IS_BMP180(c)) {
        BMP_ReadTP_180(c, &s->temperature, &s->pressure);
    } else {
        BMP_ReadRaw(c);
        s->temperature = BMP_Temperature(c);
        s->pressure    = BMP_Pressure(c);
        s->humidity    = c->has_humidity ? BMP_Humidity(c) : 0;
    }

    s->temperature += s->calTemp;
    s->pressure    += s->calPres;

    if(s->chan_t >= 0) CHANNEL_Set(s->chan_t, s->temperature, CHANNEL_SET_FLAG_SILENT);
    if(s->chan_p >= 0) CHANNEL_Set(s->chan_p, s->pressure,    CHANNEL_SET_FLAG_SILENT);

    if(c->has_humidity) {
        s->humidity += s->calHum;
        if(s->chan_h >= 0) CHANNEL_Set(s->chan_h, s->humidity, CHANNEL_SET_FLAG_SILENT);
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "%s[%u] T=%.2f°C P=%.2f hPa H=%.1f%%",
                    s->name, (uint8_t)(s-g_dev)+1,
                    s->temperature*0.01f, s->pressure*0.0001f, s->humidity*0.1f);
    } else {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "%s[%u] T=%.2f°C P=%.2f hPa",
                    s->name, (uint8_t)(s-g_dev)+1,
                    s->temperature*0.01f, s->pressure*0.0001f);
    }
}

// ---------------------------------------------------------------------------
// Parse pin/channel/address args (shared by Init and AddSensor)
// ---------------------------------------------------------------------------
static void ParseArgs(sensor_t *s)
{
    bmp_ctx_t *c = &s->chip;
    c->i2c.pin_clk    = Tokenizer_GetPin(1, 8);
    c->i2c.pin_data   = Tokenizer_GetPin(2, 14);
    s->chan_t          = Tokenizer_GetArgIntegerDefault(3, -1);
    s->chan_p          = Tokenizer_GetArgIntegerDefault(4, -1);
    s->chan_h          = Tokenizer_GetArgIntegerDefault(5, -1);
    c->i2c.address8bit = Tokenizer_GetArgIntegerDefault(6, 0) ? BMP_ADDR_ALT : BMP_ADDR_MAIN;

    c->i2c.pin_clk    = Tokenizer_GetPinEqual("SCL",   c->i2c.pin_clk);
    c->i2c.pin_data   = Tokenizer_GetPinEqual("SDA",   c->i2c.pin_data);
    s->chan_t          = Tokenizer_GetArgEqualInteger("chan_t", s->chan_t);
    s->chan_p          = Tokenizer_GetArgEqualInteger("chan_p", s->chan_p);
    s->chan_h          = Tokenizer_GetArgEqualInteger("chan_h", s->chan_h);
    c->i2c.address8bit = Tokenizer_GetArgEqualInteger("addr",
                          c->i2c.address8bit >> 1) << 1;
}

// ---------------------------------------------------------------------------
// Init one sensor slot  (shared by Init and AddSensor)
// ---------------------------------------------------------------------------
static bool InitSensor(sensor_t *s)
{
    s->ok = false;
    s->calTemp = s->calPres = s->calHum = 0;
    s->secondsBetween = s->secondsUntilNext = 1;

    setPinUsedString(s->chip.i2c.pin_clk,  "BMPI2C SCL");
    setPinUsedString(s->chip.i2c.pin_data, "BMPI2C SDA");
    Soft_I2C_PreInit(&s->chip.i2c);
    delay_ms(1);

    s->name = BMP_Init(&s->chip);
    if(!s->name) {
        ADDLOG_ERROR(LOG_FEATURE_SENSOR, "BMPI2C[%u] (SDA=%i) not found",
                     g_n+1, s->chip.i2c.pin_data);
        return false;
    }

    // Default config — user can refine with BMPI2C_Configure
    if(!IS_BMP180(&s->chip))
        BMP_Configure(&s->chip, MODE_NORMAL,
                      SAMPLING_X1, SAMPLING_X1, SAMPLING_X1,
                      FILTER_OFF,  STANDBY_0_5);

    ADDLOG_INFO(LOG_FEATURE_SENSOR, "%s[%u] (SDA=%i) ready",
                s->name, g_n+1, s->chip.i2c.pin_data);
    s->ok = true;
    return true;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

//cmddetail:{"name":"BMPI2C_Configure","args":"[Mode][TSamp][PSamp][HSamp][IIR][Standby] [sensorN]",
//cmddetail:"descr":"Configure sensor. Mode: 0=normal 1=forced 2=sleep. Sampling: -1=skip 1 2 4 8 16. IIR filter: 0=off 2..128. Standby ms: 1 63 125 250 500 1000 2000 4000. Optional sensorN is 1-based.",
//cmddetail:"fn":"BMPI2C_Configure","file":"driver/drv_bmpi2c.c","requires":"",
//cmddetail:"examples":"BMPI2C_Configure 0 8 2 4 16 125 <br /> BMPI2C_Configure 0 1 1 1 0 1000 2"}
commandResult_t BMPI2C_Configure(const void *context, const char *cmd,
                                  const char *args, int cmdFlags)
{
    CMD_UNUSED;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    sensor_t *s = pick(cmd, 6, Tokenizer_GetArgsCount() >= 7);
    if(!s) return CMD_RES_BAD_ARGUMENT;
    BMP_Configure(&s->chip,
        BMP_GetMode    (Tokenizer_GetArgIntegerDefault(0, 0)),
        BMP_GetSampling(Tokenizer_GetArgIntegerDefault(1, 1)),
        BMP_GetSampling(Tokenizer_GetArgIntegerDefault(2, 1)),
        BMP_GetSampling(Tokenizer_GetArgIntegerDefault(3, 1)),
        BMP_GetFilter  (Tokenizer_GetArgIntegerDefault(4, 0)),
        BMP_GetStandby (Tokenizer_GetArgIntegerDefault(5, 1000)));
    return CMD_RES_OK;
}

//cmddetail:{"name":"BMPI2C_Calibrate","args":"[DeltaTemp][DeltaPres][DeltaHum] [sensorN]",
//cmddetail:"descr":"Offset calibration in °C, hPa, %%RH. Optional sensorN is 1-based.",
//cmddetail:"fn":"BMPI2C_Calibrate","file":"driver/drv_bmpi2c.c","requires":"",
//cmddetail:"examples":"BMPI2C_Calibrate -4 0 10 <br /> BMPI2C_Calibrate -1.5 0 5 2"}
commandResult_t BMPI2C_Calibrate(const void *context, const char *cmd,
                                  const char *args, int cmdFlags)
{
    CMD_UNUSED;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    sensor_t *s = pick(cmd, 3, Tokenizer_GetArgsCount() >= 4);
    if(!s) return CMD_RES_BAD_ARGUMENT;
    s->calTemp = (int32_t)(Tokenizer_GetArgFloatDefault(0, 0.0f) * 100);
    s->calPres = (int32_t)(Tokenizer_GetArgFloatDefault(1, 0.0f) * 100);
    s->calHum  = (int32_t)(Tokenizer_GetArgFloatDefault(2, 0.0f) * 10);
    ADDLOG_INFO(LOG_FEATURE_SENSOR, "%s[%u] cal T%+.2f°C P%+.2f hPa H%+.1f%%",
                s->name, (uint8_t)(s-g_dev)+1,
                s->calTemp*0.01f, s->calPres*0.0001f, s->calHum*0.1f);
    return CMD_RES_OK;
}

//cmddetail:{"name":"BMPI2C_Cycle","args":"[Seconds] [sensorN]",
//cmddetail:"descr":"Measurement interval in seconds (1–255). Optional sensorN is 1-based.",
//cmddetail:"fn":"BMPI2C_Cycle","file":"driver/drv_bmpi2c.c","requires":"",
//cmddetail:"examples":"BMPI2C_Cycle 60 <br /> BMPI2C_Cycle 30 2"}
commandResult_t BMPI2C_Cycle(const void *context, const char *cmd,
                              const char *args, int cmdFlags)
{
    CMD_UNUSED;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    sensor_t *s = pick(cmd, 1, Tokenizer_GetArgsCount() >= 2);
    if(!s) return CMD_RES_BAD_ARGUMENT;
    int sec = Tokenizer_GetArgInteger(0);
    if(sec < 1) { ADDLOG_INFO(LOG_FEATURE_CMD, "BMPI2C: min 1s"); return CMD_RES_BAD_ARGUMENT; }
    s->secondsBetween = (uint8_t)sec;
    return CMD_RES_OK;
}

//cmddetail:{"name":"BMPI2C_Measure","args":"[sensorN]",
//cmddetail:"descr":"Trigger an immediate measurement. Optional sensorN is 1-based.",
//cmddetail:"fn":"BMPI2C_Measure","file":"driver/drv_bmpi2c.c","requires":"",
//cmddetail:"examples":"BMPI2C_Measure <br /> BMPI2C_Measure 2"}
commandResult_t BMPI2C_CMD_Force(const void *context, const char *cmd,
                                  const char *args, int cmdFlags)
{
    CMD_UNUSED;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    sensor_t *s = pick(cmd, 0, Tokenizer_GetArgsCount() >= 1);
    if(!s || !s->ok) return CMD_RES_BAD_ARGUMENT;
    Measure(s);
    return CMD_RES_OK;
}

//cmddetail:{"name":"BMPI2C_AddSensor","args":"[SCL=pin] [SDA=pin] [chan_t=ch] [chan_p=ch] [chan_h=ch] [addr=7bit]",
//cmddetail:"descr":"Register an additional BMP/BME sensor at runtime.",
//cmddetail:"fn":"BMPI2C_AddSensor","file":"driver/drv_bmpi2c.c","requires":"",
//cmddetail:"examples":"BMPI2C_AddSensor SCL=5 SDA=4 chan_t=3 chan_p=4"}
commandResult_t BMPI2C_CMD_AddSensor(const void *context, const char *cmd,
                                      const char *args, int cmdFlags)
{
    CMD_UNUSED; (void)cmd;
    if(g_n >= BMPI2C_MAX_SENSORS) {
        ADDLOG_ERROR(LOG_FEATURE_SENSOR, "BMPI2C: sensor array full (%i)", BMPI2C_MAX_SENSORS);
        return CMD_RES_ERROR;
    }
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    sensor_t *s = &g_dev[g_n];
    // Inherit first sensor's bus as default; keyword args will override.
    s->chip.i2c = g_dev[0].chip.i2c;
    s->chan_t = s->chan_p = s->chan_h = -1;
    ParseArgs(s);
    if(InitSensor(s)) g_n++;
    return CMD_RES_OK;
}

//cmddetail:{"name":"BMPI2C_ListSensors","args":"",
//cmddetail:"descr":"List all registered BMPI2C sensors and their last readings.",
//cmddetail:"fn":"BMPI2C_ListSensors","file":"driver/drv_bmpi2c.c","requires":"",
//cmddetail:"examples":"BMPI2C_ListSensors"}
commandResult_t BMPI2C_CMD_ListSensors(const void *context, const char *cmd,
                                        const char *args, int cmdFlags)
{
    CMD_UNUSED; (void)cmd; (void)args;
    for(uint8_t i = 0; i < g_n; i++) {
        sensor_t *s = &g_dev[i];
        ADDLOG_INFO(LOG_FEATURE_SENSOR,
                    "  [%u] %s SDA=%i SCL=%i addr=0x%02X "
                    "T=%.2f°C P=%.2f hPa H=%.1f%% ch=%i/%i/%i%s",
                    i+1, s->name,
                    s->chip.i2c.pin_data, s->chip.i2c.pin_clk,
                    s->chip.i2c.address8bit>>1,
                    s->temperature*0.01f, s->pressure*0.0001f, s->humidity*0.1f,
                    s->chan_t, s->chan_p, s->chan_h,
                    s->ok ? "" : " [FAILED]");
    }
    return CMD_RES_OK;
}

// ---------------------------------------------------------------------------
// Driver entry points
// ---------------------------------------------------------------------------

void BMPI2C_Init()
{
    if(g_n >= BMPI2C_MAX_SENSORS) {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "BMPI2C: sensor array full (%i)", BMPI2C_MAX_SENSORS);
        return;
    }
    sensor_t *s = &g_dev[g_n];
    s->chan_t = s->chan_p = s->chan_h = -1;
    ParseArgs(s);
    if(!InitSensor(s)) return;
    g_n++;

    if(g_n == 1) {  // register commands on first successful sensor only
        CMD_RegisterCommand("BMPI2C_Configure",  BMPI2C_Configure,       NULL);
        CMD_RegisterCommand("BMPI2C_Calibrate",  BMPI2C_Calibrate,       NULL);
        CMD_RegisterCommand("BMPI2C_Cycle",      BMPI2C_Cycle,           NULL);
        CMD_RegisterCommand("BMPI2C_Measure",    BMPI2C_CMD_Force,       NULL);
        CMD_RegisterCommand("BMPI2C_AddSensor",  BMPI2C_CMD_AddSensor,   NULL);
        CMD_RegisterCommand("BMPI2C_ListSensors",BMPI2C_CMD_ListSensors, NULL);
    }
}

void BMPI2C_OnEverySecond()
{
    for(uint8_t i = 0; i < g_n; i++) {
        sensor_t *s = &g_dev[i];
        if(s->secondsUntilNext == 0) {
            if(s->ok) Measure(s);
            s->secondsUntilNext = s->secondsBetween;
        } else {
            s->secondsUntilNext--;
        }
    }
}

void BMPI2C_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState)
{
    if(bPreState) return;
    for(uint8_t i = 0; i < g_n; i++) {
        sensor_t *s = &g_dev[i];
        // T is °C×100 → ×0.01 = °C.  P is Pa×100 → ×0.0001 = hPa.
        hprintf255(request, "<h2>%s[%u] T=%.2f°C P=%.2f hPa",
                   s->name, i+1, s->temperature*0.01f, s->pressure*0.0001f);
        if(s->chip.has_humidity)
            hprintf255(request, " H=%.1f%%", s->humidity*0.1f);
        hprintf255(request, "</h2>");
    }
}
#endif // ENABLE_DRIVER_BMPI2C_MULTI

