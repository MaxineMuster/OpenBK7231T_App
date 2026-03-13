// drv_i2c_slave.c
// -------------------------------------------------------
// Generic I2C slave driver.
// Turns the MCU into a fake sensor using interrupt-driven
// bit-banging on any two GPIO pins.
//
// ISR design
// ----------
// Two interrupts are active at all times:
//
//   SDA CHANGE  – detects START (SDA↓ while SCL↑) and
//                 STOP  (SDA↑ while SCL↑).
//
//   SCL RISING  – sample SDA to receive a bit, or no-op
//                 during transmit (data was already placed
//                 on SDA at the preceding falling edge).
//
//   SCL FALLING – place the next transmit bit on SDA, or
//                 release SDA (set high) during receive.
//
// Bit ordering: MSB first, same as I2C spec.
//
// Output model
// ------------
// To drive SDA low  : configure pin as output, write 0.
// To release SDA    : configure pin as input (open-drain
//                     pull-up takes it high).  We never
//                     actively drive SDA high – that would
//                     fight the master on a real bus.
//
// encode_response() is called OUTSIDE the ISR (in
// I2CSlave_Process()) after the write phase completes.
// The ISR only sets pending_encode = true and returns.
// -------------------------------------------------------

#include "../obk_config.h"

#if ENABLE_DRIVER_I2C_SLAVE

#include <string.h>
#include "../new_pins.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "../hal/hal_pins.h"
#include "drv_i2c_slave.h"
#include "../cmnds/cmd_public.h"

// -------------------------------------------------------
// GPIO helpers
// -------------------------------------------------------

// Open-drain SDA emulation WITHOUT mode switching in ISR.
//
// SDA is configured as OUTPUT once at Init and stays that way.
// "Release" (logical 1) = drive output HIGH and let pull-up confirm.
// "Low"     (logical 0) = drive output LOW (pulls bus low).
//
// This is safe on I2C buses because:
//   - We only drive LOW actively (never fight the master driving high)
//   - When we drive HIGH the pull-up and our output agree
//   - HAL_PIN_SetOutputValue is a single register write – ISR safe
//
// sda_read() still works while pin is output on most MCUs
// (GPIO input register reflects the actual bus voltage).
static inline void sda_release(i2cs_dev_t *dev) {
//    HAL_PIN_SetOutputValue(dev->pin_sda, 1);  // let pull-up take it high
    HAL_PIN_Setup_Input_Pullup(dev->pin_sda);
}

static inline void sda_low(i2cs_dev_t *dev) {
    HAL_PIN_SetOutputValue(dev->pin_sda, 0);  // pull bus low
}

static inline int sda_read(i2cs_dev_t *dev) {
    return HAL_PIN_ReadDigitalInput(dev->pin_sda);
}

static inline int scl_read(i2cs_dev_t *dev) {
    return HAL_PIN_ReadDigitalInput(dev->pin_scl);
}

// -------------------------------------------------------
// Single global instance (one slave device per MCU is
// the common case; addr is fixed, ops is set at Init).
// -------------------------------------------------------
static i2cs_dev_t g_slave;

// -------------------------------------------------------
// ISR-safe diagnostic counters.
// Incremented in ISR (no ADDLOG there!).
// Read and logged in I2CSlave_Process() – main loop only.
// -------------------------------------------------------
static volatile uint32_t g_dbg_starts     = 0; // START conditions seen
static volatile uint32_t g_dbg_addr_match = 0; // address matched ours
static volatile uint32_t g_dbg_addr_miss  = 0; // address was for someone else
static volatile uint32_t g_dbg_reads      = 0; // master read transactions
static volatile uint32_t g_dbg_writes     = 0; // master write transactions
static volatile uint32_t g_dbg_bytes_sent = 0; // bytes clocked out to master
static volatile uint8_t  g_dbg_last_addr  = 0; // last raw address byte seen
static volatile uint8_t  g_dbg_last_cmd   = 0; // last command/register byte received

// -------------------------------------------------------
// Forward declarations
// -------------------------------------------------------
static void i2cs_isr_scl(int gpio);
static void i2cs_isr_sda(int gpio);

// -------------------------------------------------------
// State machine helpers
// -------------------------------------------------------

// Called from ISR after a full byte has been received into shift_reg.
// Handles address match, data accumulation, and ACK/NACK.
static void i2cs_byte_received(i2cs_dev_t *dev) {
    uint8_t byte = dev->shift_reg;

    switch (dev->state) {

    case I2CS_RECV_ADDR: {
        uint8_t addr_wire = byte & 0xFE;   // strip R/W bit
        bool    is_read   = (byte & 0x01) != 0;

        g_dbg_last_addr = byte;
        if (addr_wire != dev->addr) {
            // Not our address – stay silent, return to IDLE
            g_dbg_addr_miss++;
            sda_release(dev);
            dev->state = I2CS_IDLE;
            return;
        }

        g_dbg_addr_match++;
        // Our address – ACK it
        sda_low(dev);
        dev->state = I2CS_ACK_ADDR;

        if (is_read) {
            // Master wants to read – serve the pre-built response
            g_dbg_reads++;
            dev->ctx.resp_pos = 0;
            dev->send_pos     = 0;
            dev->send_bit     = 7;
            dev->state        = I2CS_SEND_DATA;
        } else {
            // Master wants to write – clear cmd buffer, start receiving
            g_dbg_writes++;
            dev->ctx.cmd_len = 0;
            dev->state       = I2CS_RECV_DATA;
        }
        break;
    }

    case I2CS_RECV_DATA:
        // Accumulate command / data bytes
        if (dev->ctx.cmd_len == 0)
            g_dbg_last_cmd = byte;   // first byte = register/command
        if (dev->ctx.cmd_len < (uint8_t)sizeof(dev->ctx.cmd))
            dev->ctx.cmd[dev->ctx.cmd_len++] = byte;
        // ACK the byte
        sda_low(dev);
        dev->state = I2CS_ACK_DATA;
        break;

    default:
        break;
    }
}

// Called from ISR after master ACKs/NACKs one of our sent bytes.
// bit=0 → master ACK (wants more), bit=1 → master NACK (done).
static void i2cs_master_ack_received(i2cs_dev_t *dev, uint8_t bit) {
    if (bit == 0) {
        // Master ACK – send next byte
        dev->send_pos++;
        dev->send_bit = 7;
        dev->state    = I2CS_SEND_DATA;
    } else {
        // Master NACK – read transaction complete
        sda_release(dev);
        dev->state = I2CS_IDLE;
        if (dev->ops->on_read_complete)
            dev->ops->on_read_complete(&dev->ctx);
    }
}


static void i2cs_isr_scl_fall(int gpio);
// -------------------------------------------------------
// SCL rising edge ISR  – sample SDA (receive) or no-op (send)
// -------------------------------------------------------
static void i2cs_isr_scl(int gpio) {
    (void)gpio;
    i2cs_dev_t *dev = &g_slave;

    switch (dev->state) {

    case I2CS_RECV_ADDR:
    case I2CS_RECV_DATA: {
        // Sample SDA into shift register, MSB first
        uint8_t bit = (uint8_t)sda_read(dev);
        dev->shift_reg = (uint8_t)((dev->shift_reg << 1) | bit);
        dev->bit_count++;
        if (dev->bit_count == 8) {
            dev->bit_count = 0;
            i2cs_byte_received(dev);
        }
        break;
    }

    case I2CS_RECV_MACK: {
        // Sample master ACK/NACK bit
        uint8_t bit = (uint8_t)sda_read(dev);
        i2cs_master_ack_received(dev, bit);
        break;
    }

    // SEND_DATA, ACK_ADDR, ACK_DATA: we placed the bit at SCL falling,
    // master samples it now – nothing for us to do on rising edge.
    default:
        break;
    }
    HAL_AttachInterrupt(dev->pin_scl, INTERRUPT_FALLING, i2cs_isr_scl_fall);

}

// -------------------------------------------------------
// SCL falling edge ISR  – place next transmit bit on SDA
// -------------------------------------------------------
static void i2cs_isr_scl_fall(int gpio) {
    (void)gpio;
    i2cs_dev_t *dev = &g_slave;

    switch (dev->state) {

    case I2CS_ACK_ADDR:
    case I2CS_ACK_DATA:
        // ACK was driven low at byte_received(); release after SCL falls
        sda_release(dev);
        // state was already advanced in byte_received()
        break;

    case I2CS_SEND_DATA: {
        // Place next bit of current response byte on SDA
        uint8_t pos = dev->send_pos;
        if (pos >= dev->ctx.resp_len) {
            // Ran out of response bytes – release SDA (master will NACK)
            sda_release(dev);
            dev->state = I2CS_RECV_MACK;
            break;
        }
        uint8_t byte = dev->ctx.resp[pos];
        if (dev->send_bit == 7) g_dbg_bytes_sent++;  // count once per byte
        uint8_t bit  = (byte >> dev->send_bit) & 0x01;
        if (bit)
            sda_release(dev);
        else
            sda_low(dev);

        if (dev->send_bit == 0) {
            // Byte fully sent – next rising edge will be master ACK/NACK
            dev->state = I2CS_RECV_MACK;
        } else {
            dev->send_bit--;
        }
        break;
    }

    case I2CS_RECV_ADDR:
    case I2CS_RECV_DATA:
        // During receive we keep SDA released (input)
        sda_release(dev);
        break;

    default:
        break;
    }
    HAL_AttachInterrupt(dev->pin_scl, INTERRUPT_RISING, i2cs_isr_scl);
}

// -------------------------------------------------------
// SDA change ISR  – START and STOP detection
// SCL must be high for both conditions.
// -------------------------------------------------------
static void i2cs_isr_sda(int gpio) {
    (void)gpio;
    i2cs_dev_t *dev = &g_slave;

    if (!scl_read(dev)) return;   // SCL low → data change, not START/STOP

    if (!sda_read(dev)) {
        // SDA just went LOW while SCL is HIGH → START condition
        g_dbg_starts++;
        dev->state     = I2CS_RECV_ADDR;
        dev->bit_count = 0;
        dev->shift_reg = 0;
        dev->busy      = true;
        sda_release(dev);   // ensure we're in input mode while receiving addr
    } else {
        // SDA just went HIGH while SCL is HIGH → STOP condition
        sda_release(dev);
        if (dev->state == I2CS_RECV_DATA || dev->state == I2CS_ACK_DATA) {
            // Write transaction just finished – flag for encode_response()
            dev->pending_encode = true;
        }
        dev->state = I2CS_IDLE;
        dev->busy  = false;
    }
}

// -------------------------------------------------------
// HAL_AttachInterrupt only supports one mode per pin.
// We need both rising and falling edges on SCL.
// Solution: attach CHANGE on SCL, branch inside the ISR.
// -------------------------------------------------------
static void i2cs_isr_scl_change(int gpio) {
    if (scl_read(&g_slave))
        i2cs_isr_scl(gpio);        // rising  → sample
    else
        i2cs_isr_scl_fall(gpio);   // falling → output
}

// -------------------------------------------------------
// Public API
// -------------------------------------------------------

void DRV_I2CSlave_Init_Sensor(uint8_t pin_sda, uint8_t pin_scl,
                   uint8_t addr, const sim_sensor_ops_t *ops)
{
    i2cs_dev_t *dev = &g_slave;

    ADDLOG_INFO(LOG_FEATURE_SENSOR, "I2CSlave Init: start SDA=%i SCL=%i addr=0x%02X", pin_sda, pin_scl, addr);

    memset(dev, 0, sizeof(i2cs_dev_t));

    dev->pin_sda  = pin_sda;
    dev->pin_scl  = pin_scl;
    dev->addr     = addr & 0xFE;   // ensure R/W bit is clear
    dev->ops      = ops;
    dev->state    = I2CS_IDLE;

    dev->ctx.pin_data  = pin_sda;
    dev->ctx.pin_clk   = pin_scl;
    dev->ctx.i2c_addr  = addr & 0xFE;

    ADDLOG_INFO(LOG_FEATURE_SENSOR, "I2CSlave Init: calling ops->init (plugin=%s)",
                ops->name ? ops->name : "?");

    // Let the plugin seed its default value ranges
    // NOTE: ops->init() calls SoftI2C_Sim_SetValue() and malloc().
    // If this crashes, drv_soft_i2c_sim_sensors.c needs the WIN32
    // guard removed so sensor plugins compile on MCU.
    if (ops->init)
        ops->init(&dev->ctx);

    ADDLOG_INFO(LOG_FEATURE_SENSOR, "I2CSlave Init: ops->init done, calling encode_response");

    // Pre-build a first response so the very first read is not empty
    if (ops->encode_response)
        ops->encode_response(&dev->ctx);

    ADDLOG_INFO(LOG_FEATURE_SENSOR, "I2CSlave Init: encode_response done, resp_len=%i", dev->ctx.resp_len);

    // SCL: input only – we never drive the clock
    HAL_PIN_Setup_Input_Pullup(pin_scl);

    // SDA: configure as OUTPUT now and keep it that way permanently.
    // ISR only toggles the output value (0/1), never switches mode.
    // Start released (high) – idle bus.
    HAL_PIN_Setup_Input_Pullup(pin_sda);
//    HAL_PIN_SetOutputValue(pin_sda, 1);

    ADDLOG_INFO(LOG_FEATURE_SENSOR,
                "I2CSlave: pins configured SDA=%i(out) SCL=%i(in)", pin_sda, pin_scl);

    // Attach interrupts AFTER pin setup
//    HAL_AttachInterrupt(pin_scl, INTERRUPT_CHANGE, i2cs_isr_scl_change);
    HAL_AttachInterrupt(pin_scl, INTERRUPT_FALLING, i2cs_isr_scl_fall);
    HAL_AttachInterrupt(pin_sda, INTERRUPT_CHANGE, i2cs_isr_sda);

    ADDLOG_INFO(LOG_FEATURE_SENSOR,
                "I2CSlave: '%s' addr=0x%02X SDA=%i SCL=%i - ready",
                ops->name ? ops->name : "?", addr, pin_sda, pin_scl);
}

void I2CSlave_Process(void)
{
    i2cs_dev_t *dev = &g_slave;

    // Log activity snapshot whenever any counter has changed.
    // Snapshot+reset is not atomic but these are debug counters – close enough.
    static uint32_t s_last_starts = 0;
    if (g_dbg_starts != s_last_starts) {
        s_last_starts = g_dbg_starts;
        ADDLOG_INFO(LOG_FEATURE_SENSOR,
            "I2CSlave: starts=%u match=%u miss=%u reads=%u writes=%u sent=%u lastAddr=0x%02X lastCmd=0x%02X",
            (unsigned)g_dbg_starts,
            (unsigned)g_dbg_addr_match,
            (unsigned)g_dbg_addr_miss,
            (unsigned)g_dbg_reads,
            (unsigned)g_dbg_writes,
            (unsigned)g_dbg_bytes_sent,
            (unsigned)g_dbg_last_addr,
            (unsigned)g_dbg_last_cmd);
    }

    if (!dev->pending_encode) return;
    dev->pending_encode = false;

    // encode_response() reads ctx.cmd[] and fills ctx.resp[].
    // Called here, outside ISR, so it can do float math, printf, etc.
    dev->ctx.resp_len = 0;
    dev->ctx.resp_pos = 0;
    if (dev->ops && dev->ops->encode_response)
        dev->ops->encode_response(&dev->ctx);
    ADDLOG_INFO(LOG_FEATURE_SENSOR,
        "I2CSlave: encoded resp_len=%i cmd[0]=0x%02X",
        dev->ctx.resp_len,
        dev->ctx.cmd_len ? dev->ctx.cmd[0] : 0xFF);
    // resp[] is now ready; the next read-addressed Start will clock it out.
}

void DRV_I2CSlave_OnEverySecond(void)
{
    // Nothing needed every second by default – values only advance when
    // encode_response() calls SoftI2C_Sim_NextValue().
    // Override point for future use (e.g. forced drift).
    (void)&g_slave;
    I2CSlave_Process();
}

void DRV_I2CSlave_Stop(void)
{
    i2cs_dev_t *dev = &g_slave;
    HAL_DetachInterrupt(dev->pin_scl);
    HAL_DetachInterrupt(dev->pin_sda);
    // Release SDA and return to input so we stop driving the bus
    HAL_PIN_SetOutputValue(dev->pin_sda, 1);
    HAL_PIN_Setup_Input_Pullup(dev->pin_sda);
    dev->state = I2CS_IDLE;
    dev->busy  = false;
}

void DRV_I2CSlave_SetValue(sim_quantity_t q, int32_t value)
{
    SoftI2C_Sim_ForceValue(&g_slave.ctx, q, value);
}


// -------------------------------------------------------
// startDriver / command integration
// -------------------------------------------------------
// This section is compiled separately so the core slave
// logic above stays clean and testable without the OBK
// command infrastructure.
// -------------------------------------------------------


#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../httpserver/new_http.h"

// -------------------------------------------------------
// Sim value helpers for non-WIN32 builds.
// On WIN32 these live in drv_soft_i2c_sim.c.
// On MCU they are compiled here so sensor plugin init()
// functions can call SoftI2C_Sim_SetValue etc. normally.
// -------------------------------------------------------
#ifndef WINDOWN

#define SIM_DEFAULT_STEP  10

static uint32_t g_i2cs_rng = 12345678u;

static int32_t i2cs_sim_rand(int32_t lo, int32_t hi) {
    g_i2cs_rng = g_i2cs_rng * 1664525u + 1013904223u;
    int32_t range = hi - lo + 1;
    if (range <= 0) return lo;
    return lo + (int32_t)(g_i2cs_rng % (uint32_t)range);
}

void SoftI2C_Sim_SetValue(sim_ctx_t *ctx, sim_quantity_t q,
                           int32_t initial, int32_t min, int32_t max,
                           int32_t step) {
    if (!ctx || q >= SIM_Q_COUNT) return;
    sim_value_t *v = &ctx->values[q];
    v->active  = true;
    v->current = initial;
    v->min     = min;
    v->max     = max;
    v->step    = step > 0 ? step : SIM_DEFAULT_STEP;
}

int32_t SoftI2C_Sim_NextValue(sim_ctx_t *ctx, sim_quantity_t q) {
    if (!ctx || q >= SIM_Q_COUNT) return 0;
    sim_value_t *v = &ctx->values[q];
    if (!v->active) return 0;
    v->current += i2cs_sim_rand(-v->step, v->step);
    if (v->current < v->min) v->current = v->min;
    if (v->current > v->max) v->current = v->max;
    return v->current;
}

int32_t SoftI2C_Sim_PeekValue(sim_ctx_t *ctx, sim_quantity_t q) {
    if (!ctx || q >= SIM_Q_COUNT) return 0;
    return ctx->values[q].current;
}

void SoftI2C_Sim_ForceValue(sim_ctx_t *ctx, sim_quantity_t q, int32_t value) {
    if (!ctx || q >= SIM_Q_COUNT) return;
    ctx->values[q].current = value;
    ctx->values[q].active  = true;
}

#endif // !WINDOWS

// Forward declarations of all sensor plugin ops tables
// defined in drv_soft_i2c_sim_sensors.c
extern const sim_sensor_ops_t g_sht3x_ops;
extern const sim_sensor_ops_t g_sht4x_ops;
extern const sim_sensor_ops_t g_aht2x_ops;
extern const sim_sensor_ops_t g_bmp280_ops;
extern const sim_sensor_ops_t g_cht83xx_ops;
extern const sim_sensor_ops_t g_veml7700_ops;

typedef struct {
    const char           *name;
    const sim_sensor_ops_t *ops;
    uint8_t               default_addr; // 8-bit wire address with R/W=0
} i2cs_sensor_entry_t;

static const i2cs_sensor_entry_t c_sensors[] = {
    { "SHT3x",    &g_sht3x_ops,    0x88 },  // 0x44 << 1
    { "SHT4x",    &g_sht4x_ops,    0x88 },  // 0x44 << 1
    { "AHT2x",    &g_aht2x_ops,    0x70 },  // 0x38 << 1
    { "BMP280",   &g_bmp280_ops,   0xEE },  // 0x77 << 1
    { "BME280",   &g_bmp280_ops,   0xEE },  // same plugin, BME flag set below
    { "CHT8305",  &g_cht83xx_ops,  0x80 },  // 0x40 << 1
    { "VEML7700", &g_veml7700_ops, 0x20 },  // 0x10 << 1
};
#define NUM_SENSORS ((int)(sizeof(c_sensors)/sizeof(c_sensors[0])))

// I2CSlave_Measure – force an immediate encode cycle
//cmddetail:{"name":"I2CSlave_Measure","args":"",
//cmddetail:"descr":"Force immediate re-encode of the slave response buffer.",
//cmddetail:"fn":"I2CSlave_CMD_Measure","file":"driver/drv_i2c_slave.c","requires":"",
//cmddetail:"examples":"I2CSlave_Measure"}
commandResult_t I2CSlave_CMD_Measure(const void *ctx, const char *cmd,
                                      const char *args, int flags)
{
    (void)ctx; (void)cmd; (void)args; (void)flags;
    i2cs_dev_t *dev = &g_slave;
    if (!dev->ops) return CMD_RES_ERROR;
    dev->ctx.resp_len = 0;
    dev->ctx.resp_pos = 0;
    dev->ops->encode_response(&dev->ctx);
    return CMD_RES_OK;
}

// I2CSlave_Set <quantity> <value>
// quantity: 0=temperature 1=humidity 2=pressure 3=co2 4=altitude 5=light
// value: x10 units (224 = 22.4 C, 500 = 50.0 %RH, 10132 = 1013.2 hPa)
//cmddetail:{"name":"I2CSlave_Set","args":"[quantity] [value]",
//cmddetail:"descr":"Force a simulated sensor quantity. quantity: 0=temp 1=hum 2=pressure 3=co2 4=altitude 5=light. value in x10 units.",
//cmddetail:"fn":"I2CSlave_CMD_Set","file":"driver/drv_i2c_slave.c","requires":"",
//cmddetail:"examples":"I2CSlave_Set 0 245 <br /> set temperature to 24.5 C"}
commandResult_t I2CSlave_CMD_Set(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{
    (void)ctx;
    Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2))
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int q   = Tokenizer_GetArgInteger(0);
    int val = Tokenizer_GetArgInteger(1);
    if (q < 0 || q >= SIM_Q_COUNT) {
        ADDLOG_ERROR(LOG_FEATURE_CMD, "I2CSlave_Set: quantity %i out of range (0..%i)", q, SIM_Q_COUNT-1);
        return CMD_RES_BAD_ARGUMENT;
    }
    DRV_I2CSlave_SetValue((sim_quantity_t)q, val);
    return CMD_RES_OK;
}

// startDriver I2CSlave [SDA=pin] [SCL=pin] [type=SHT3x] [addr=0xEC]
void DRV_I2CSlave_Init(void)
{
    // Pin defaults
    uint8_t pin_sda = (uint8_t)Tokenizer_GetPinEqual("SDA", 14);
    uint8_t pin_scl = (uint8_t)Tokenizer_GetPinEqual("SCL",  9);

printf("DRV_I2CSlave_StartDriver SDA=%i SCL=%i \r\n",pin_sda,pin_scl);
    // Sensor type – default to CHT8305
    const char *type_str = Tokenizer_GetArgEqualDefault("type","CHT8305");
    const i2cs_sensor_entry_t *entry = &c_sensors[5]; // CHT8305 default
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (type_str && strcasecmp(type_str, c_sensors[i].name) == 0) {
            entry = &c_sensors[i];
            break;
        }
    }

    // Address – default from table, override with addr= argument
    uint8_t addr = (uint8_t)Tokenizer_GetArgEqualInteger("addr",
                                                                     entry->default_addr);

    DRV_I2CSlave_Init_Sensor(pin_sda, pin_scl, addr, entry->ops);

    // Special case: BME280 uses the BMP280 plugin but with is_bme280=true
    if (type_str && strcasecmp(type_str, "BME280") == 0) {
        // The bmp280_state_t is allocated by the plugin's init().
        // Cast through user pointer to set the flag (same trick as simulator).
        typedef struct { uint8_t reg; bool is_bme280; } bmp280_state_t;
        bmp280_state_t *s = (bmp280_state_t *)g_slave.ctx.user;
        if (s) s->is_bme280 = true;
    }

    CMD_RegisterCommand("I2CSlave_Measure", I2CSlave_CMD_Measure, NULL);
    CMD_RegisterCommand("I2CSlave_Set",     I2CSlave_CMD_Set,     NULL);
}

void DRV_I2CSlave_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState)
{
    if (bPreState) return;
    i2cs_dev_t *dev = &g_slave;
    if (!dev->ops)
        hprintf255(request, "<b>I2CSlave: not running</b>");
    else
        hprintf255(request,
                   "<h2>I2CSlave: %s addr=0x%02X SDA=%i SCL=%i</h2>",
                   dev->ops->name ? dev->ops->name : "?",
                   dev->addr, dev->pin_sda, dev->pin_scl);
}

#endif // ENABLE_DRIVER_I2C_SLAVE
