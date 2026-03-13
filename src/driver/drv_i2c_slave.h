// drv_i2c_slave.h
// -------------------------------------------------------
// Generic I2C slave driver – turns the MCU into a fake
// sensor that answers I2C transactions like a real chip.
//
// Architecture
// ------------
//   HAL layer : HAL_AttachInterrupt() on SCL + SDA pins.
//   ISR       : bit-level state machine (START/STOP detection,
//               address matching, byte RX/TX, ACK/NACK).
//   Main loop : encode_response() called outside ISR after
//               each write phase completes (flag-based).
//
// The sensor protocol (register maps, byte encoding) is
// provided by the same sim_sensor_ops_t plugins used by
// the Windows simulator.  No duplication needed.
//
// Usage
// -----
//   // Impersonate a BMP280 on SDA=14, SCL=9, address 0x76
//   I2CSlave_Init(14, 9, 0xEC, &g_bmp280_ops);
//
//   // In main loop or task:
//   I2CSlave_Process();   // encodes response when flagged
//
//   // In periodic tick:
//   I2CSlave_OnEverySecond();  // drifts simulated values
// -------------------------------------------------------
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "drv_soft_i2c_sim.h"   // sim_ctx_t, sim_sensor_ops_t, sim_value_t

// -------------------------------------------------------
// I2C slave state machine states
// -------------------------------------------------------
typedef enum {
    I2CS_IDLE = 0,       // waiting for START
    I2CS_RECV_ADDR,      // receiving 7-bit address + R/W bit
    I2CS_ACK_ADDR,       // driving SDA low for address ACK
    I2CS_RECV_DATA,      // receiving command/data bytes from master
    I2CS_ACK_DATA,       // driving SDA low for data byte ACK
    I2CS_SEND_DATA,      // clocking out response bytes to master
    I2CS_RECV_MACK,      // receiving master ACK/NACK after each sent byte
} i2cs_state_t;

// -------------------------------------------------------
// Slave device context
// -------------------------------------------------------
typedef struct {
    // Pins
    uint8_t          pin_sda;
    uint8_t          pin_scl;

    // Our address on the bus (8-bit wire, R/W bit = 0)
    uint8_t          addr;

    // Sensor plugin and its context (same types as Windows sim)
    const sim_sensor_ops_t *ops;
    sim_ctx_t        ctx;

    // State machine
    volatile i2cs_state_t state;
    volatile uint8_t  shift_reg;     // bits being assembled / sent
    volatile uint8_t  bit_count;     // bit position within current byte (0-7)
    volatile bool     pending_encode;// set by ISR, cleared by I2CSlave_Process()
    volatile bool     busy;          // true while ISR is mid-transaction

    // Current byte being sent (index into ctx.resp[])
    // managed only by ISR so no volatile needed beyond the ctx fields
    uint8_t          send_pos;
    uint8_t          send_bit;       // bit index within current send byte (7..0)

} i2cs_dev_t;

// -------------------------------------------------------
// Public API
// -------------------------------------------------------

// Initi - called by StartDriver
void DRV_I2CSlave_Init(void);

// Initialise and attach interrupts.
// addr   : 8-bit wire address with R/W=0 (e.g. 0xEC for BMP280@0x76)
// ops    : sensor plugin (same pointer used with SoftI2C_Sim_Register)
void DRV_I2CSlave_Init_Sensor(uint8_t pin_sda, uint8_t pin_scl,
                   uint8_t addr, const sim_sensor_ops_t *ops);

// Call from main loop / task as often as possible.
// Runs encode_response() when the ISR flags pending_encode.
// NOT safe to call from ISR context.
void DRV_I2CSlave_Process(void);

// Call once per second to advance simulated sensor values (drift).
void DRV_I2CSlave_OnEverySecond(void);

// Release interrupts and tri-state pins.
void DRV_I2CSlave_Stop(void);

// Force-set a simulated quantity (e.g. from a channel value).
// q      : SIM_Q_TEMPERATURE, SIM_Q_HUMIDITY, etc.
// value  : in x10 units (224 = 22.4 C)
void DRV_I2CSlave_SetValue(sim_quantity_t q, int32_t value);

