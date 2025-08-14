#include "drv_DCF77.h"
#include "../obk_config.h"

#if ENABLE_DRIVER_DCF77

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"


#if PLATFORM_BEKEN
#include "BkDriverTimer.h"
#include "BkDriverGpio.h"
#include "sys_timer.h"
#include "gw_intf.h"
#elif PLATFORM_W600 || PLATFORM_W800
#include "wm_gpio.h"
#define IRQ_raise_and_fall 1
#elif PLATFORM_BL602
#include "hal_gpio.h"
#include "bl_gpio.h"
#elif PLATFORM_LN882H
#include "../../sdk/OpenLN882H/mcu/driver_ln882h/hal/hal_common.h"
#include "../../sdk/OpenLN882H/mcu/driver_ln882h/hal/hal_gpio.h"
#define IRQ_raise_and_fall 1
#elif PLATFORM_REALTEK
#include "gpio_irq_api.h"
#include "../hal/realtek/hal_pinmap_realtek.h"
rtlPinMapping_t* rtl_dcf77;
#elif PLATFORM_ECR6600
#include "gpio.h"
#elif PLATFORM_XRADIO
#include "../hal/xradio/hal_pinmap_xradio.h"
extern void HAL_XR_ConfigurePin(GPIO_Port port, GPIO_Pin pin, GPIO_WorkMode mode, GPIO_PullType pull);
xrpin_t* xr_dcf77;
#elif PLATFORM_ESP8266 || PLATFORM_ESPIDF
#include "../hal/espidf/hal_pinmap_espidf.h"
espPinMapping_t* esp_dcf77;
#define IRQ_raise_and_fall 1
#endif

#define DCF77_PIN_DEFAULT 25
int GPIO_DCF77 = DCF77_PIN_DEFAULT;

#if PLATFORM_W600 || PLATFORM_W800
unsigned int GPIO_DCF77_pin;
#endif

#define DCF77_SYNC_THRESHOLD_MS 1500 // missing pulse threshold (between bits ~1000ms, missing pulse ~2000ms)
#define DCF77_MIN_PULSE_MS 20
#define DCF77_MAX_PULSE_MS 300

static volatile uint32_t dcf77_last_edge_tick = 0;
static volatile uint32_t dcf77_pulse_length_ms = 0;
static volatile uint8_t dcf77_minute_bits[59];
static volatile uint8_t dcf77_pulse_count = 0;
static volatile bool dcf77_synced = false;
static volatile time_t dcf77_next_minute_epoch = 0;
static volatile bool dcf77_has_valid_epoch = false;

static uint32_t get_ms_tick() {
#ifdef PLATFORM_W600
	typedef portTickType TickType_t;		// W600/W800: xTaskGetTickCount() is of type "portTickType", all others "TickType_t" , W600 has no definition for TickType_t
#endif
#if PLATFORM_BEKEN || PLATFORM_LN882H || PLATFORM_REALTEK || PLATFORM_XRADIO || PLATFORM_ESP8266 || PLATFORM_ESPIDF || PLATFORM_W600 || PLATFORM_W800
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
#elif 0
    portTICK_RATE_MS * xTaskGetTickCount();
#elif PLATFORM_BL602 || PLATFORM_ECR6600
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
#else
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
#endif
}

// Helper: convert BCD bits to int
static int bits_to_bcd(const uint8_t* bits, int count) {
    int val = 0;
    int mul = 1;
    // factors are: 1,2,4,8,10,20,40,80
    // so instead of 16 use 10 
    for (int i = 0; i < count; i++) {
        val += bits[i] * mul;
        mul *= 2;
        if (mul == 16) mul=10;
    }
    return val;
}

// Helper: parity check - even parity
static int bits_parity(const uint8_t* bits, int count) {
    int p = 0;
    for (int i = 0; i < count; i++) {
        p ^= bits[i];
    }
    return p;
}

// DCF77 epoch calculation
static int dcf77_decode(const uint8_t* bits, time_t* epoch_out) {
    // Minute: bits 21-27 (7), parity 28
    int minute = bits_to_bcd(bits+21, 7);
    int minute_parity = bits[28];
    // Hour: bits 29-34 (6), parity 35
    int hour = bits_to_bcd(bits+29, 6);
    int hour_parity = bits[35];
    // Day: bits 36-41 (6)
    int day = bits_to_bcd(bits+36, 6);
    // Weekday: bits 42-44 (3)
    int weekday = bits_to_bcd(bits+42, 3);
    // Month: bits 45-49 (5)
    int month = bits_to_bcd(bits+45, 5);
    // Year: bits 50-57 (8)
    int year = bits_to_bcd(bits+50, 8);
    int date_parity = bits[58];
    
    // Parity checks  
    if (bits_parity(bits+21,7) != minute_parity){ 
    	addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: Parity Error minute - wanted %d got %d",bits_parity(bits+21,7), minute_parity);
    	return 0;
    }
    if (bits_parity(bits+29,6) != hour_parity){
    	addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: Parity Error hour - wanted %d got %d",bits_parity(bits+29,6), hour_parity);
    	return 0;
    }
    if (bits_parity(bits+36,22) != date_parity){
    	addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: Parity Error minute - wanted %d got %d",bits_parity(bits+36,22), date_parity);
    	return 0;
    }

    // just to be sure: a range check
    if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12 || year > 99) return 0;

    struct tm t;
    t.tm_sec = 0;
    t.tm_min = minute;
    t.tm_hour = hour;
    t.tm_mday = day;
    t.tm_mon = month-1;
    t.tm_year = year+100; // DCF77 gives 2-digit year; assume 2000+
    t.tm_isdst = -1;
    time_t epoch = mktime(&t);
//    const char *wdays[] = {"Sun","Mon", "Tue", "Wed", "Thu", "Fri", "Sat" }; 
//    addLogAdv(LOG_DEBUG, LOG_FEATURE_RAW, "DCF77: dcf77_decode %s %d.%d.%d %d:%d:00",wdays[weekday],day,month,year+2000,hour,minute);
    addLogAdv(LOG_DEBUG, LOG_FEATURE_RAW, "DCF77: dcf77_decode %d.%d.%d %d:%d:00",day,month,year+2000,hour,minute);

    if (epoch_out) *epoch_out = epoch;
    return 1;
}

static void DCF77_Reset() {
    dcf77_pulse_count = 0;
    dcf77_synced = false;
    memset((void*)dcf77_minute_bits, 0, sizeof(dcf77_minute_bits));
}

// DCF77 interrupt handler for all platforms
static void DCF77_ISR_Common() {
    uint32_t now = get_ms_tick();
    if (dcf77_last_edge_tick == 0) dcf77_last_edge_tick = now;		// start is no detected sync ;-)
    
    addLogAdv(LOG_EXTRADEBUG, LOG_FEATURE_RAW, "DCF77: DEBUG: gap to prev. interrupt  %i", now - dcf77_last_edge_tick );

    // handle detection of "long gap" == start of sync
    if (now - dcf77_last_edge_tick > DCF77_SYNC_THRESHOLD_MS){
           addLogAdv(LOG_DEBUG, LOG_FEATURE_RAW, "DCF77: DEBUG: detected sync pulse > %i", now - dcf77_last_edge_tick );
           dcf77_pulse_count = 0;
            dcf77_synced = true;
            // If previous buffer is valid, set time here
            if (dcf77_has_valid_epoch) {
                CLOCK_setDeviceTime((uint32_t)dcf77_next_minute_epoch);
                addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: setDeviceTime %lu", (unsigned long)dcf77_next_minute_epoch);
                dcf77_has_valid_epoch = false;
            }
            else{
                addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: no setDeviceTime (invalid) %lu", (unsigned long)dcf77_next_minute_epoch);

            }
            DCF77_Reset();
            dcf77_synced = true;
        }
     
#ifdef IRQ_raise_and_fall
	 // this is for both rising and falling
    if (HAL_PIN_ReadDigitalInput(GPIO_DCF77) ==1){		// start of high pulse = start of "bit"
    	dcf77_last_edge_tick = now;
    	return;
    }
#else 
// not working -- WDT
#if 0
    dcf77_last_edge_tick = now;
    int count=0;
    vTaskDelay(50);
    addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: HALInput_Level= %i", HAL_PIN_ReadDigitalInput(GPIO_DCF77));
    while(HAL_PIN_ReadDigitalInput(GPIO_DCF77)==1 && count++ < 30) vTaskDelay(10);
    if (count > 29) return;	// bailing out, we didn't find end of signal in 300 ms !
    addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: End of pulse - count= %i", count);
#endif
#endif
    // so we knwo we are after falling edge now
    uint32_t pulse_length = now - dcf77_last_edge_tick;
    addLogAdv(LOG_DEBUG, LOG_FEATURE_RAW, "DCF77: DEBUG: actual pulse length %i (count=%i)", pulse_length, dcf77_pulse_count);
    // Synchronization: detect missing pulse (long gap)
    if (!dcf77_synced) {
        // Not synced yet, do nothing
//        addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: Not synced!");
        return;
    }

    // If synced, collect bits
    if (dcf77_pulse_count < 59) {
        if (pulse_length < DCF77_MIN_PULSE_MS || pulse_length > DCF77_MAX_PULSE_MS) {
            // Invalid pulse, lose sync
            dcf77_synced = false;
            dcf77_pulse_count = 0;
            addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: bit illegal pulse length %i", pulse_length );
            return;
        }
        dcf77_minute_bits[dcf77_pulse_count] = (pulse_length < 150) ? 0 : 1;
        addLogAdv(LOG_EXTRADEBUG, LOG_FEATURE_RAW, "DCF77: bit %i is %i", dcf77_pulse_count, dcf77_minute_bits[dcf77_pulse_count] );
        dcf77_pulse_count++;
    } if (dcf77_pulse_count == 59) {
        // Buffer full, decode for next minute
        addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: got 59 bits!");
        time_t epoch = 0;
        int valid = dcf77_decode(dcf77_minute_bits, &epoch);
        if (valid && epoch > 1609459200) { // sanity check for 2021+
            dcf77_next_minute_epoch = epoch;
            dcf77_has_valid_epoch = true;
        } else {
            dcf77_has_valid_epoch = false;
        }
        dcf77_pulse_count++; // Don't record more until re-synced
    }
}

#if PLATFORM_W600 || PLATFORM_W800
static void DCF77_Interrupt(void* context) {
    tls_clr_gpio_irq_status(GPIO_DCF77_pin);
    DCF77_ISR_Common();
}
#define IRQ_raise_and_fall 1
#elif PLATFORM_BL602
static void DCF77_Interrupt(void* arg) {
    DCF77_ISR_Common();
    bl_gpio_intmask(GPIO_DCF77, 0);
}
#elif PLATFORM_LN882H
// handler are defined in BL0937
#if ! (ENABLE_DRIVER_BL0937)
void GPIOA_IRQHandler() {
    uint32_t base = GPIO_DCF77 < 16 ? GPIOA_BASE : GPIOB_BASE;
    uint16_t gpio_pin = (uint16_t)1 << (uint16_t)(GPIO_DCF77 % 16);
    if(hal_gpio_pin_get_it_flag(base, gpio_pin) == HAL_SET) {
        hal_gpio_pin_clr_it_flag(base, gpio_pin);
        DCF77_ISR_Common();
    }
}
void GPIOB_IRQHandler() {
    GPIOA_IRQHandler(); // Same logic for simplicity
}
#endif
#elif PLATFORM_BEKEN
static void DCF77_Interrupt(void* arg) {
    DCF77_ISR_Common();
}
#elif PLATFORM_REALTEK
void dcf77_irq_handler(uint32_t id, gpio_irq_event event) {
    DCF77_ISR_Common();
}
#elif PLATFORM_ECR6600
void DCF77_Interrupt(unsigned char pinNum) {
    DCF77_ISR_Common();
}
#elif PLATFORM_XRADIO
static void DCF77_Interrupt(void* arg) {
    DCF77_ISR_Common();
}
#elif PLATFORM_ESP8266 || PLATFORM_ESPIDF
static void DCF77_Interrupt(void* arg) {
    DCF77_ISR_Common();
}
#else
void DCF77_Interrupt(unsigned char pinNum) {
    DCF77_ISR_Common();
}
#endif

void DCF77_Init_Pins() {
    GPIO_DCF77 = PIN_FindPinIndexForRole(IOR_DCF77, DCF77_PIN_DEFAULT);
    HAL_PIN_Setup_Input_Pulldown(GPIO_DCF77);

#if PLATFORM_W600 || PLATFORM_W800
    GPIO_DCF77_pin = HAL_GetGPIOPin(GPIO_DCF77);
    tls_gpio_cfg(GPIO_DCF77_pin, WM_GPIO_DIR_INPUT, WM_GPIO_ATTR_PULLLOW);
    tls_gpio_isr_register(GPIO_DCF77_pin, DCF77_Interrupt, NULL);
    tls_gpio_irq_enable(GPIO_DCF77_pin, WM_GPIO_IRQ_TRIG_DOUBLE_EDGE);
#elif PLATFORM_BL602
    hal_gpio_register_handler(DCF77_Interrupt, GPIO_DCF77, GPIO_INT_CONTROL_ASYNC, GPIO_INT_TRIG_POS_PULSE, (void*)NULL);
#elif PLATFORM_LN882H
    hal_gpio_pin_it_cfg(GPIO_DCF77 < 16 ? GPIOA_BASE : GPIOB_BASE, (uint16_t)1 << (GPIO_DCF77 % 16), GPIO_INT_FALLING);
    hal_gpio_pin_it_en(GPIO_DCF77 < 16 ? GPIOA_BASE : GPIOB_BASE, (uint16_t)1 << (GPIO_DCF77 % 16), HAL_ENABLE);
    NVIC_SetPriority(GPIO_DCF77 < 16 ? GPIOA_IRQn : GPIOB_IRQn, 1);
    NVIC_EnableIRQ(GPIO_DCF77 < 16 ? GPIOA_IRQn : GPIOB_IRQn);
#elif PLATFORM_BEKEN
    gpio_int_enable(GPIO_DCF77, IRQ_TRIGGER_FALLING_EDGE, DCF77_Interrupt);
#elif PLATFORM_REALTEK
    rtl_dcf77 = g_pins + GPIO_DCF77;
    rtl_dcf77->irq = os_malloc(sizeof(gpio_irq_t));
    memset(rtl_dcf77->irq, 0, sizeof(gpio_irq_t));
    gpio_irq_init(rtl_dcf77->irq, rtl_dcf77->pin, dcf77_irq_handler, NULL);
    gpio_irq_set(rtl_dcf77->irq, IRQ_FALL, 1);
    gpio_irq_enable(rtl_dcf77->irq);
#elif PLATFORM_ECR6600
    T_GPIO_ISR_CALLBACK dcf77isr;
    dcf77isr.gpio_callback = (&DCF77_Interrupt);
    dcf77isr.gpio_data = 0;
    drv_gpio_ioctrl(GPIO_DCF77, DRV_GPIO_CTRL_INTR_MODE, DRV_GPIO_ARG_INTR_MODE_N_EDGE);
    drv_gpio_ioctrl(GPIO_DCF77, DRV_GPIO_CTRL_REGISTER_ISR, (int)&dcf77isr);
    drv_gpio_ioctrl(GPIO_DCF77, DRV_GPIO_CTRL_INTR_ENABLE, 0);
#elif PLATFORM_XRADIO
    xr_dcf77 = g_pins + GPIO_DCF77;
    HAL_XR_ConfigurePin(xr_dcf77->port, xr_dcf77->pin, GPIOx_Pn_F6_EINT, GPIO_PULL_UP);
    GPIO_IrqParam dcf77param;
    dcf77param.event = GPIO_IRQ_EVT_FALLING_EDGE;
    dcf77param.callback = DCF77_Interrupt;
    dcf77param.arg = (void*)0;
    HAL_GPIO_EnableIRQ(xr_dcf77->port, xr_dcf77->pin, &dcf77param);
#elif PLATFORM_ESP8266 || PLATFORM_ESPIDF
#if PLATFORM_ESPIDF
//    from esp-idf/components/hal/include/hal/gpio_types.h
typedef enum {
    GPIO_INTR_DISABLE = 0,     /*!< Disable GPIO interrupt                             */
    GPIO_INTR_POSEDGE = 1,     /*!< GPIO interrupt type : rising edge                  */
    GPIO_INTR_NEGEDGE = 2,     /*!< GPIO interrupt type : falling edge                 */
    GPIO_INTR_ANYEDGE = 3,     /*!< GPIO interrupt type : both rising and falling edge */
    GPIO_INTR_LOW_LEVEL = 4,   /*!< GPIO interrupt type : input low level trigger      */
    GPIO_INTR_HIGH_LEVEL = 5,  /*!< GPIO interrupt type : input high level trigger     */
    GPIO_INTR_MAX,
} gpio_int_type_t;
#endif
    esp_dcf77 = g_pins + GPIO_DCF77;
    gpio_install_isr_service(0);
    ESP_ConfigurePin(esp_dcf77->pin, GPIO_MODE_INPUT, true, false, GPIO_INTR_ANYEGEDGE);
    gpio_isr_handler_add(esp_dcf77->pin, DCF77_Interrupt, NULL);
#else
    HAL_PIN_Setup_Input_Pullup(GPIO_DCF77);
#endif
}

void DCF77_Shutdown_Pins() {
#if PLATFORM_W600 || PLATFORM_W800
    tls_gpio_irq_disable(GPIO_DCF77_pin);
#elif PLATFORM_BEKEN
    gpio_int_disable(GPIO_DCF77);
#elif PLATFORM_REALTEK
    gpio_irq_free(rtl_dcf77->irq);
    os_free(rtl_dcf77->irq);
    rtl_dcf77->irq = NULL;
#elif PLATFORM_ECR6600
    drv_gpio_ioctrl(GPIO_DCF77, DRV_GPIO_CTRL_INTR_DISABLE, 0);
#elif PLATFORM_XRADIO
    HAL_GPIO_DeInit(xr_dcf77->port, xr_dcf77->pin);
    HAL_GPIO_DisableIRQ(xr_dcf77->port, xr_dcf77->pin);
#elif PLATFORM_ESP8266 || PLATFORM_ESPIDF
    gpio_isr_handler_remove(esp_dcf77->pin);
    gpio_uninstall_isr_service();
#endif
}

// Optionally, can be called once per second in your main loop
void DCF77_OnEverySecond() {
    // Nothing needed, all logic is interrupt-driven and sync-based
    char str[60]={0};
    int i;
    if (dcf77_pulse_count > 57){ 
    	for (i=0; i<dcf77_pulse_count && i<59; i++){ str[i]=dcf77_minute_bits[i] ==0? 'L':'H';} 
    	addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: got %02i bits: %s\n", i,str);
    }
}

void DCF77_Init(void) {
    addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: DCF77_Init_Pins()\n");
    DCF77_Init_Pins();
    addLogAdv(LOG_INFO, LOG_FEATURE_RAW, "DCF77: DCF77_Init_Reset()\n");
    DCF77_Reset();
}
void DCF77_Stop(){
};
#endif // ENABLE_DRIVER_DCF77
