// drv_bmpi2c.c  –  multi-sensor BMP/BME I²C driver
//
// Supports BMP085, BMP180, BMP280, BME280, BME680, BME688.
// Multiple independent sensors on different pin pairs or I²C addresses
// are handled through a per-sensor state array (g_sensors[]).
// Use:
//   startDriver BMPI2C [SCL=<pin>] [SDA=<pin>] [chan_t=<ch>] [chan_p=<ch>]
//                      [chan_h=<ch>] [addr=<7bit>]
// Additional sensors at runtime:
//   BMPI2C_AddSensor SDA=<pin> SCL=<pin> chan_t=<ch> chan_p=<ch> chan_h=<ch>
//
// All measurement arithmetic on the hot path uses integer math only.
// Float is used only in the HTTP page render and calibrate log (user path).

#include "../new_pins.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include <limits.h>
#include "../obk_config.h"

#if ENABLE_DRIVER_BMPI2C

// -----------------------------------------------------------------------
// Tuneable limits
// -----------------------------------------------------------------------
#ifndef BMPI2C_MAX_SENSORS
#  define BMPI2C_MAX_SENSORS  4
#endif

// -----------------------------------------------------------------------
// drv_bmpi2c.h is an implementation header: it defines functions that
// reference g_softI2C and g_chipName as bare globals.  Those must be
// declared before the header is included so the inline functions compile.
// -----------------------------------------------------------------------
static softI2C_t  g_softI2C;
static char      *g_chipName = "BMPI2C";

#include "drv_bmpi2c.h"

// -----------------------------------------------------------------------
// Per-sensor state
// -----------------------------------------------------------------------
typedef struct
{
    softI2C_t   i2c;

    // Snapshot of chip-level flags captured right after BMP_BasicInit()
    // so each sensor remembers its own type independently.
    uint8_t     chipId;           // raw chip_id byte from the header
    bool        hasHumidity;      // copy of isHumidityAvail at init time
    const char *chipName;         // pointer into a string literal

    // Measured values (×100 for T and P, ×10 for H)
    int32_t     temperature;      // °C × 100
    uint32_t    pressure;         // hPa × 100
    uint32_t    humidity;         // %RH × 10

    // User calibration offsets (same units as above)
    int32_t     calTemp;
    int32_t     calPres;
    int32_t     calHum;

    // Target channels (-1 = not assigned)
    int8_t      chan_t;
    int8_t      chan_p;
    int8_t      chan_h;

    // Timing
    uint8_t     secondsBetween;
    uint8_t     secondsUntilNext;

    bool        isConfigured;
    bool        isWorking;
} bmpi2c_dev_t;

// -----------------------------------------------------------------------
// Module-level sensor registry
// -----------------------------------------------------------------------
static bmpi2c_dev_t g_sensors[BMPI2C_MAX_SENSORS];
static uint8_t      g_numSensors = 0;

// -----------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------

// Resolve sensor by optional 1-based index argument at argIdx.
// Returns NULL and logs an error when out of range.
static bmpi2c_dev_t *BMPI2C_GetSensor(const char *cmd, int argIdx, bool present)
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

static void BMPI2C_StoreAndPublish(bmpi2c_dev_t *dev)
{
    dev->temperature += dev->calTemp;
    dev->pressure    += dev->calPres;

    if(dev->chan_t >= 0)
        CHANNEL_Set(dev->chan_t, dev->temperature, CHANNEL_SET_FLAG_SILENT);
    if(dev->chan_p >= 0)
        CHANNEL_Set(dev->chan_p, dev->pressure,    CHANNEL_SET_FLAG_SILENT);

    if(dev->hasHumidity)
    {
        dev->humidity += dev->calHum;
        if(dev->chan_h >= 0)
            CHANNEL_Set(dev->chan_h, dev->humidity, CHANNEL_SET_FLAG_SILENT);
        ADDLOG_INFO(LOG_FEATURE_SENSOR,
                    "%s[%u] (SDA=%i): T=%i P=%i H=%i",
                    dev->chipName,
                    (uint8_t)(dev - g_sensors) + 1,
                    dev->i2c.pin_data,
                    dev->temperature, dev->pressure, dev->humidity);
    }
    else
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR,
                    "%s[%u] (SDA=%i): T=%i P=%i",
                    dev->chipName,
                    (uint8_t)(dev - g_sensors) + 1,
                    dev->i2c.pin_data,
                    dev->temperature, dev->pressure);
    }
}

static void BMPI2C_Measure(bmpi2c_dev_t *dev)
{
    // Reset so calibration offset is applied exactly once per cycle
    dev->temperature = 0;
    dev->pressure    = 0;
    dev->humidity    = 0;

    // The BMP library functions in drv_bmpi2c.h all use g_softI2C and
    // chip_id as module-level globals.  Point them at this sensor's bus
    // and restore the chip_id that was recorded at init time.
    g_softI2C = dev->i2c;
    chip_id   = dev->chipId;

    if(IsBMX280)
    {
        BMX280_Update();
        delay_ms(125);
        dev->temperature = BMX280_ReadTemperature();
        dev->pressure    = BMX280_ReadPressure();
        if(IsBME280)
            dev->humidity = BME280_ReadHumidity();
    }
    else if(IsBMP180)
    {
        dev->pressure = BMP180_ReadData(&dev->temperature);
    }
    else if(IsBME68X)
    {
        BME68X_Update();
        dev->temperature = BME68X_ReadTemperature();
        dev->pressure    = BME68X_ReadPressure();
        dev->humidity    = BME68X_ReadHumidity();
    }

    BMPI2C_StoreAndPublish(dev);
}

// -----------------------------------------------------------------------
// Shared sensor-init logic (called from BMPI2C_Init and BMPI2C_AddSensor)
// -----------------------------------------------------------------------
static void BMPI2C_InitSensor(bmpi2c_dev_t *dev)
{
    dev->isConfigured     = false;
    dev->isWorking        = false;
    dev->secondsBetween   = 1;
    dev->secondsUntilNext = 1;
    dev->calTemp          = 0;
    dev->calPres          = 0;
    dev->calHum           = 0;
    dev->hasHumidity      = false;
    dev->chipId           = 0;
    dev->chipName         = "BMPI2C";

    setPinUsedString(dev->i2c.pin_clk,  "BMPI2C SCL");
    setPinUsedString(dev->i2c.pin_data, "BMPI2C SDA");

    Soft_I2C_PreInit(&dev->i2c);
    delay_ms(1);

    // Point the header's globals at this sensor's bus before probing
    g_softI2C = dev->i2c;

    if(BMP_BasicInit())
    {
        // Capture chip-level state into the per-sensor struct so that
        // subsequent measurements can restore it without re-probing.
        dev->chipId      = chip_id;
        dev->hasHumidity = isHumidityAvail;
        dev->chipName    = g_chipName;   // points to a string literal; safe
        dev->isWorking   = true;

        ADDLOG_INFO(LOG_FEATURE_SENSOR, "%s[%u] (SDA=%i) initialized!",
                    dev->chipName, g_numSensors + 1, dev->i2c.pin_data);

        if(IsBMX280 || IsBME68X)
        {
            BMXI2C_Configure(MODE_NORMAL, SAMPLING_X1, SAMPLING_X1,
                             SAMPLING_X1, FILTER_OFF, STANDBY_0_5);
            dev->isConfigured = true;
        }
        else if(IsBMP180)
        {
            dev->isConfigured = true;
        }
    }
    else
    {
        ADDLOG_ERROR(LOG_FEATURE_SENSOR, "BMPI2C[%u] (SDA=%i) probe failed!",
                     g_numSensors + 1, dev->i2c.pin_data);
    }
}

// -----------------------------------------------------------------------
// Command handlers
// -----------------------------------------------------------------------

// BMPI2C_Configure [Mode][TempSampling][PressureSampling][HumSampling]
//                  [IIRFilter][StandbyTime] [sensorN]
commandResult_t BMPI2C_Configure(const void *context, const char *cmd,
                                  const char *args, int cmdFlags)
{
    (void)context; (void)cmdFlags;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    int argc = Tokenizer_GetArgsCount();

    bmpi2c_dev_t *dev = BMPI2C_GetSensor(cmd, 6, argc >= 7);
    if(!dev) return CMD_RES_BAD_ARGUMENT;

    BMP_mode       mode       = GetMode      (Tokenizer_GetArgIntegerDefault(0, 0));
    BMP_sampling   t_sampling = GetSampling  (Tokenizer_GetArgIntegerDefault(1, 1));
    BMP_sampling   p_sampling = GetSampling  (Tokenizer_GetArgIntegerDefault(2, 1));
    BMP_sampling   h_sampling = GetSampling  (Tokenizer_GetArgIntegerDefault(3, 1));
    BMP_filter     filter     = GetFilter    (Tokenizer_GetArgIntegerDefault(4, 0));
    standby_time   time       = GetStandbyTime(Tokenizer_GetArgIntegerDefault(5, 1000));

    g_softI2C = dev->i2c;
    chip_id   = dev->chipId;
    BMXI2C_Configure(mode, t_sampling, p_sampling, h_sampling, filter, time);
    dev->isConfigured = true;

    ADDLOG_INFO(LOG_FEATURE_SENSOR, "%s[%u] (SDA=%i): configured",
                dev->chipName,
                (uint8_t)(dev - g_sensors) + 1, dev->i2c.pin_data);
    return CMD_RES_OK;
}

// BMPI2C_Cycle <seconds> [sensorN]
commandResult_t BMPI2C_Cycle(const void *context, const char *cmd,
                              const char *args, int cmdFlags)
{
    (void)context; (void)cmdFlags;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    if(Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;

    int argc         = Tokenizer_GetArgsCount();
    bmpi2c_dev_t *dev = BMPI2C_GetSensor(cmd, 1, argc >= 2);
    if(!dev) return CMD_RES_BAD_ARGUMENT;

    int seconds = Tokenizer_GetArgInteger(0);
    if(seconds < 1)
    {
        ADDLOG_INFO(LOG_FEATURE_CMD, "BMPI2C_Cycle: minimum cycle is 1 second.");
        return CMD_RES_BAD_ARGUMENT;
    }

    dev->secondsBetween = (uint8_t)seconds;
    ADDLOG_INFO(LOG_FEATURE_CMD, "BMPI2C[%u] (SDA=%i): measure every %i s",
                (uint8_t)(dev - g_sensors) + 1, dev->i2c.pin_data, seconds);
    return CMD_RES_OK;
}

// BMPI2C_Calibrate <deltaTemp> <deltaPressure> <deltaHumidity> [sensorN]
commandResult_t BMPI2C_Calibrate(const void *context, const char *cmd,
                                  const char *args, int cmdFlags)
{
    (void)context; (void)cmdFlags;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    int argc         = Tokenizer_GetArgsCount();
    bmpi2c_dev_t *dev = BMPI2C_GetSensor(cmd, 3, argc >= 4);
    if(!dev) return CMD_RES_BAD_ARGUMENT;

    dev->calTemp = (int32_t)(Tokenizer_GetArgFloatDefault(0, 0.0f) * 100);
    dev->calPres = (int32_t)(Tokenizer_GetArgFloatDefault(1, 0.0f) * 100);
    dev->calHum  = (int32_t)(Tokenizer_GetArgFloatDefault(2, 0.0f) * 10);

    ADDLOG_INFO(LOG_FEATURE_SENSOR,
                "BMPI2C[%u] (SDA=%i): calibrate T%+.2f P%+.2f H%+.1f",
                (uint8_t)(dev - g_sensors) + 1, dev->i2c.pin_data,
                dev->calTemp * 0.01f,
                dev->calPres * 0.01f,
                dev->calHum  * 0.1f);
    return CMD_RES_OK;
}

// BMPI2C_Measure [sensorN]  –  immediate one-shot
commandResult_t BMPI2C_CMD_Force(const void *context, const char *cmd,
                                  const char *args, int cmdFlags)
{
    (void)context; (void)cmdFlags;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    bmpi2c_dev_t *dev = BMPI2C_GetSensor(cmd, 0, Tokenizer_GetArgsCount() >= 1);
    if(!dev || !dev->isConfigured) return CMD_RES_BAD_ARGUMENT;
    dev->secondsUntilNext = dev->secondsBetween;
    BMPI2C_Measure(dev);
    return CMD_RES_OK;
}

// BMPI2C_AddSensor [SCL=pin] [SDA=pin] [chan_t=ch] [chan_p=ch] [chan_h=ch] [addr=7bit]
commandResult_t BMPI2C_CMD_AddSensor(const void *context, const char *cmd,
                                      const char *args, int cmdFlags)
{
    (void)context; (void)cmd; (void)cmdFlags;
    if(g_numSensors >= BMPI2C_MAX_SENSORS)
    {
        ADDLOG_ERROR(LOG_FEATURE_SENSOR, "BMPI2C_AddSensor: sensor array full (%i).",
                     BMPI2C_MAX_SENSORS);
        return CMD_RES_ERROR;
    }

    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    bmpi2c_dev_t *dev = &g_sensors[g_numSensors];

    // Default pins from first sensor
    dev->i2c.pin_clk    = g_sensors[0].i2c.pin_clk;
    dev->i2c.pin_data   = g_sensors[0].i2c.pin_data;
    dev->i2c.address8bit = g_sensors[0].i2c.address8bit;
    dev->chan_t = -1;
    dev->chan_p = -1;
    dev->chan_h = -1;

    dev->i2c.pin_clk    = Tokenizer_GetPinEqual("SCL",  dev->i2c.pin_clk);
    dev->i2c.pin_data   = Tokenizer_GetPinEqual("SDA",  dev->i2c.pin_data);
    dev->chan_t          = Tokenizer_GetArgEqualInteger("chan_t", dev->chan_t);
    dev->chan_p          = Tokenizer_GetArgEqualInteger("chan_p", dev->chan_p);
    dev->chan_h          = Tokenizer_GetArgEqualInteger("chan_h", dev->chan_h);
    dev->i2c.address8bit = (Tokenizer_GetArgEqualInteger("addr",
                             dev->i2c.address8bit >> 1) << 1);

    BMPI2C_InitSensor(dev);
    if(dev->isWorking)
        g_numSensors++;

    return CMD_RES_OK;
}

// BMPI2C_ListSensors
commandResult_t BMPI2C_CMD_ListSensors(const void *context, const char *cmd,
                                        const char *args, int cmdFlags)
{
    (void)context; (void)cmd; (void)args; (void)cmdFlags;
    if(!g_numSensors)
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR, "BMPI2C: no sensors registered.");
        return CMD_RES_OK;
    }
    for(uint8_t i = 0; i < g_numSensors; i++)
    {
        bmpi2c_dev_t *s = &g_sensors[i];
        ADDLOG_INFO(LOG_FEATURE_SENSOR,
                    "  [%u] SDA=%i SCL=%i addr=0x%02X "
                    "T=%.2f P=%.2f H=%.1f ch_t=%i ch_p=%i ch_h=%i %s",
                    i + 1,
                    s->i2c.pin_data, s->i2c.pin_clk,
                    s->i2c.address8bit >> 1,
                    s->temperature * 0.01f,
                    s->pressure    * 0.01f,
                    s->humidity    * 0.1f,
                    s->chan_t, s->chan_p, s->chan_h,
                    s->isWorking ? "" : "[FAILED]");
    }
    return CMD_RES_OK;
}

// -----------------------------------------------------------------------
// Driver entry points
// -----------------------------------------------------------------------

// startDriver BMPI2C [SCL=pin] [SDA=pin] [chan_t=ch] [chan_p=ch] [chan_h=ch]
//                    [addr=7bit]
// Legacy positional form also accepted:
//   startDriver BMPI2C [CLK] [DATA] [chan_t] [chan_p] [chan_h] [0|1]
void BMPI2C_Init()
{
    if(g_numSensors >= BMPI2C_MAX_SENSORS)
    {
        ADDLOG_INFO(LOG_FEATURE_SENSOR,
                    "BMPI2C: sensor array full (%i).", BMPI2C_MAX_SENSORS);
        return;
    }

    bmpi2c_dev_t *dev = &g_sensors[g_numSensors];

    // --- Legacy positional defaults ---
    dev->i2c.pin_clk  = Tokenizer_GetPin(1, 8);
    dev->i2c.pin_data = Tokenizer_GetPin(2, 14);
    dev->chan_t        = Tokenizer_GetArgIntegerDefault(3, -1);
    dev->chan_p        = Tokenizer_GetArgIntegerDefault(4, -1);
    dev->chan_h        = Tokenizer_GetArgIntegerDefault(5, -1);
    dev->i2c.address8bit = (Tokenizer_GetArgIntegerDefault(6, 0) == 1)
                           ? I2C_ALT_ADDR : I2C_MAIN_ADDR;

    // --- Keyword overrides ---
    dev->i2c.pin_clk     = Tokenizer_GetPinEqual("SCL",    dev->i2c.pin_clk);
    dev->i2c.pin_data    = Tokenizer_GetPinEqual("SDA",    dev->i2c.pin_data);
    dev->chan_t           = Tokenizer_GetArgEqualInteger("chan_t", dev->chan_t);
    dev->chan_p           = Tokenizer_GetArgEqualInteger("chan_p", dev->chan_p);
    dev->chan_h           = Tokenizer_GetArgEqualInteger("chan_h", dev->chan_h);
    dev->i2c.address8bit  = (Tokenizer_GetArgEqualInteger("addr",
                              dev->i2c.address8bit >> 1) << 1);

    BMPI2C_InitSensor(dev);

    if(!dev->isWorking)
        return;   // don't register commands if no sensor found

    g_numSensors++;

    // Commands are registered once on the first sensor load
    if(g_numSensors == 1)
    {
        //cmddetail:{"name":"BMPI2C_Configure","args":"[Mode][TempSampling][PressureSampling][HumSampling][IIRFilter][StandbyTime] [sensorN]",
        //cmddetail:"descr":"Manual sensor configuration. Modes: 0=normal 1=forced 2=sleep. Oversampling range: -1=skipped 2^0..2^4. IIRFilter: 0=off 2^1..2^4 (up to 2^7 for BME68X). StandbyTime: 1=0.5ms 63=62.5ms 125 250 500 1000 2000 4000. Optional sensorN selects sensor (1-based, default=1).",
        //cmddetail:"fn":"BMPI2C_Configure","file":"driver/drv_bmpi2c.c","requires":"",
        //cmddetail:"examples":"BMPI2C_Configure 0 8 2 4 16 125 <br /> BMPI2C_Configure 0 1 1 1 0 1000 2"}
        CMD_RegisterCommand("BMPI2C_Configure",   BMPI2C_Configure,        NULL);

        //cmddetail:{"name":"BMPI2C_Calibrate","args":"[DeltaTemp][DeltaPressure][DeltaHumidity] [sensorN]",
        //cmddetail:"descr":"Calibrate the BMPI2C sensor. Optional sensorN selects sensor (1-based, default=1).",
        //cmddetail:"fn":"BMPI2C_Calibrate","file":"driver/drv_bmpi2c.c","requires":"",
        //cmddetail:"examples":"BMPI2C_Calibrate -4 0 10 <br /> BMPI2C_Calibrate -4 0 10 2"}
        CMD_RegisterCommand("BMPI2C_Calibrate",   BMPI2C_Calibrate,        NULL);

        //cmddetail:{"name":"BMPI2C_Cycle","args":"[IntervalSeconds] [sensorN]",
        //cmddetail:"descr":"Measurement interval in seconds (min 1, max 255). Optional sensorN selects sensor (1-based, default=1).",
        //cmddetail:"fn":"BMPI2C_Cycle","file":"driver/drv_bmpi2c.c","requires":"",
        //cmddetail:"examples":"BMPI2C_Cycle 60 <br /> BMPI2C_Cycle 30 2"}
        CMD_RegisterCommand("BMPI2C_Cycle",       BMPI2C_Cycle,            NULL);

        //cmddetail:{"name":"BMPI2C_Measure","args":"[sensorN]",
        //cmddetail:"descr":"Immediate one-shot measurement. Optional sensorN selects sensor (1-based, default=1).",
        //cmddetail:"fn":"BMPI2C_CMD_Force","file":"driver/drv_bmpi2c.c","requires":"",
        //cmddetail:"examples":"BMPI2C_Measure <br /> BMPI2C_Measure 2"}
        CMD_RegisterCommand("BMPI2C_Measure",     BMPI2C_CMD_Force,        NULL);

        //cmddetail:{"name":"BMPI2C_AddSensor","args":"[SCL=pin] [SDA=pin] [chan_t=ch] [chan_p=ch] [chan_h=ch] [addr=7bit]",
        //cmddetail:"descr":"Register an additional BMP/BME sensor on different pins or address.",
        //cmddetail:"fn":"BMPI2C_CMD_AddSensor","file":"driver/drv_bmpi2c.c","requires":"",
        //cmddetail:"examples":"BMPI2C_AddSensor SCL=5 SDA=4 chan_t=3 chan_p=4"}
        CMD_RegisterCommand("BMPI2C_AddSensor",   BMPI2C_CMD_AddSensor,    NULL);

        //cmddetail:{"name":"BMPI2C_ListSensors","args":"",
        //cmddetail:"descr":"List all registered BMPI2C sensors and their last readings.",
        //cmddetail:"fn":"BMPI2C_CMD_ListSensors","file":"driver/drv_bmpi2c.c","requires":"",
        //cmddetail:"examples":"BMPI2C_ListSensors"}
        CMD_RegisterCommand("BMPI2C_ListSensors", BMPI2C_CMD_ListSensors,  NULL);
    }
}

void BMPI2C_OnEverySecond()
{
    for(uint8_t i = 0; i < g_numSensors; i++)
    {
        bmpi2c_dev_t *dev = &g_sensors[i];
        if(dev->secondsUntilNext == 0)
        {
            if(dev->isConfigured)
                BMPI2C_Measure(dev);
            dev->secondsUntilNext = dev->secondsBetween;
        }
        else
        {
            dev->secondsUntilNext--;
        }
    }
}

void BMPI2C_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState)
{
    if(bPreState) return;
    for(uint8_t i = 0; i < g_numSensors; i++)
    {
        bmpi2c_dev_t *dev = &g_sensors[i];
        hprintf255(request,
                   "<h2>%s[%u] (SDA=%i) T=%.2f°C P=%.2f hPa",
                   dev->chipName, i + 1, dev->i2c.pin_data,
                   dev->temperature * 0.01f,
                   dev->pressure    * 0.01f);
        if(dev->hasHumidity)
            hprintf255(request, " H=%.1f%%", dev->humidity * 0.1f);
        hprintf255(request, "</h2>");
        if(!dev->isWorking)
            hprintf255(request,
                       "<p style='color:red'>WARNING: %s[%u] init failed – check pins!</p>",
                       dev->chipName, i + 1);
    }
}

#endif // ENABLE_DRIVER_BMPI2C
