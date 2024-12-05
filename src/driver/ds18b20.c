/*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_uart.h"
#include "../httpserver/new_http.h"
#include "../hal/hal_pins.h"

#include "ds18b20.h"
#include "OneWire_common.h"

static int ds18_conversionPeriod = 15;	// time between refreshs of temperature
static int ds18_count = 0;		// detected number of devices 
static int errcount = 0;
static int lastconv; 			// secondsElapsed on last successfull reading
static uint8_t dsread = 0;
static int Pin;
static devicesArray ds18b20devices;

/*
// dynamic Array for the devices
typedef struct {
  DeviceAddress[];
  float *lasttemp;
  size_t used;
  size_t size;
} devicesArray;



void initArray(devicesArray *a, size_t initialSize) {
  a->array = malloc(initialSize * sizeof(DeviceAddress));
  a->lasttemp = malloc(initialSize * sizeof(float));
  
  if (! a->array || ! a->lasttemp ){
  	 bk_printf ("initArray: malloc failed!\n");
  	 return;
  }
  a->used = 0;
  a->size = initialSize;
}

void insertArray(devicesArray *a, DeviceAddress dev) {
  // a->used is the number of used entries, because a->array[a->used++] updates a->used only *after* the array has been accessed.
  // Therefore a->used can go up to a->size 
  if (a->used == a->size) {
    a->size++;
    a->array = realloc(a->array, a->size * sizeof(DeviceAddress));
    if (! a->array ){
    	bk_printf ("insertArray: realloc of array failed!\n");
    }
    a->lasttemp = realloc(a->lasttemp, a->size * sizeof(float));
    if (! a->lasttemp ){
    	bk_printf ("insertArray: realloc of lasttemp failed!\n");
    }
  }
  memcpy(a->array[a->used++], dev,sizeof(DeviceAddress));
}

void freeArray(devicesArray *a) {
  free(a->array);
  free(a->lasttemp);
  a->array = NULL;
  a->lasttemp = NULL;
  a->used = a->size = 0;
}
*/


// OneWire commands
#define GETTEMP			0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define SKIPROM			0xCC  // Command to address all devices on the bus
#define SELECTDEVICE	0x55  // Command to address all devices on the bus
#define COPYSCRATCH     0x48  // Copy scratchpad to EEPROM
#define READSCRATCH     0xBE  // Read from scratchpad
#define WRITESCRATCH    0x4E  // Write to scratchpad
#define RECALLSCRATCH   0xB8  // Recall from EEPROM to scratchpad
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition
// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8
// DSROM FIELDS
#define DSROM_FAMILY    0
#define DSROM_CRC       7
// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

uint8_t DS_GPIO;
uint8_t init = 0;
uint8_t bitResolution = 12;
uint8_t devices = 0;

DeviceAddress ROM_NO;
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
bool LastDeviceFlag;

/// Sends one bit to bus
void ds18b20_write(char bit) {
	if (bit & 1) {
		HAL_PIN_Setup_Output(DS_GPIO);
		noInterrupts();
		HAL_PIN_SetOutputValue(DS_GPIO, 0);
		OWusleepshort(6); 	// was usleep(6);
		HAL_PIN_Setup_Input(DS_GPIO);	// release bus
		OWusleepmed(64);		// was usleep(64);
		interrupts();
	}
	else {
		HAL_PIN_Setup_Output(DS_GPIO);
		noInterrupts();
		HAL_PIN_SetOutputValue(DS_GPIO, 0);
		OWusleepmed(60);		// was usleep(60);
		HAL_PIN_Setup_Input(DS_GPIO);	// release bus
		OWusleepshort(10);		// was usleep(10);
		interrupts();
	}
}

// Reads one bit from bus
unsigned char ds18b20_read(void) {
	unsigned char value = 0;
	HAL_PIN_Setup_Output(DS_GPIO);
	noInterrupts();
	HAL_PIN_SetOutputValue(DS_GPIO, 0);
	OWusleepshort(6);		// was usleep(6);
	HAL_PIN_Setup_Input(DS_GPIO);
	OWusleepshort(9);		// was usleep(9);
	value = HAL_PIN_ReadDigitalInput(DS_GPIO);
	OWusleepmed(55);		// was usleep(55);
	interrupts();
	return (value);
}
// Sends one byte to bus
void ds18b20_write_byte(char data) {
	unsigned char i;
	unsigned char x;
	for (i = 0; i < 8; i++) {
		x = data >> i;
		x &= 0x01;
		ds18b20_write(x);
	}
	OWusleepmed(100);		// was usleep(100);
}
// Reads one byte from bus
unsigned char ds18b20_read_byte(void) {
	unsigned char i;
	unsigned char data = 0;
	for (i = 0; i < 8; i++)
	{
		if (ds18b20_read()) data |= 0x01 << i;
		OWusleepshort(15);		// was usleep(15);
	}
	return(data);
}
// Sends reset pulse
unsigned char ds18b20_reset(void) {
	unsigned char presence;
	HAL_PIN_Setup_Output(DS_GPIO);
	noInterrupts();
	HAL_PIN_SetOutputValue(DS_GPIO, 0);
	OWusleeplong(480);		// was usleep(480);
	HAL_PIN_SetOutputValue(DS_GPIO, 1);
	HAL_PIN_Setup_Input(DS_GPIO);
	OWusleepmed(70);		// was usleep(70);
	presence = (HAL_PIN_ReadDigitalInput(DS_GPIO) == 0);
	OWusleeplong(410);		// was usleep(410);
	interrupts();
	return presence;
}

bool ds18b20_setResolution(const DeviceAddress tempSensorAddresses[], int numAddresses, uint8_t newResolution) {
	bool success = false;
	// handle the sensors with configuration register
	newResolution = constrain(newResolution, 9, 12);
	uint8_t newValue = 0;
	ScratchPad scratchPad;
	// loop through each address
	for (int i = 0; i < numAddresses; i++) {
		// we can only update the sensor if it is connected
		if (ds18b20_isConnected((DeviceAddress*)tempSensorAddresses[i], scratchPad)) {
			switch (newResolution) {
			case 12:
				newValue = TEMP_12_BIT;
				break;
			case 11:
				newValue = TEMP_11_BIT;
				break;
			case 10:
				newValue = TEMP_10_BIT;
				break;
			case 9:
			default:
				newValue = TEMP_9_BIT;
				break;
			}
			// if it needs to be updated we write the new value
			if (scratchPad[CONFIGURATION] != newValue) {
				scratchPad[CONFIGURATION] = newValue;
				ds18b20_writeScratchPad((DeviceAddress*)tempSensorAddresses[i], scratchPad);
			}
			// done
			success = true;
		}
	}
	return success;
}

void ds18b20_writeScratchPad(const DeviceAddress *deviceAddress, const uint8_t *scratchPad) {
	ds18b20_reset();
	ds18b20_select(deviceAddress);
	ds18b20_write_byte(WRITESCRATCH);
	ds18b20_write_byte(scratchPad[HIGH_ALARM_TEMP]); // high alarm temp
	ds18b20_write_byte(scratchPad[LOW_ALARM_TEMP]); // low alarm temp
	ds18b20_write_byte(scratchPad[CONFIGURATION]);
	ds18b20_reset();
}

bool ds18b20_readScratchPad(const DeviceAddress *deviceAddress, uint8_t* scratchPad) {
	// send the reset command and fail fast
	int b = ds18b20_reset();
	if (b == 0) return false;
	ds18b20_select(deviceAddress);
	ds18b20_write_byte(READSCRATCH);
	// Read all registers in a simple loop
	// byte 0: temperature LSB
	// byte 1: temperature MSB
	// byte 2: high alarm temp
	// byte 3: low alarm temp
	// byte 4: DS18B20 & DS1822: configuration register
	// byte 5: internal use & crc
	// byte 6: DS18B20 & DS1822: store for crc
	// byte 7: DS18B20 & DS1822: store for crc
	// byte 8: SCRATCHPAD_CRC
	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = ds18b20_read_byte();
	}
	b = ds18b20_reset();
	return (b == 1);
}

void ds18b20_select(const DeviceAddress *address) {
	uint8_t i;
	ds18b20_write_byte(SELECTDEVICE);           // Choose ROM
	for (i = 0; i < 8; i++) ds18b20_write_byte(((uint8_t *)address)[i]);
}

void ds18b20_requestTemperatures() {
	ds18b20_reset();
	ds18b20_write_byte(SKIPROM);
	ds18b20_write_byte(GETTEMP);
	//unsigned long start = esp_timer_get_time() / 1000ULL;
	//while (!isConversionComplete() && ((esp_timer_get_time() / 1000ULL) - start < millisToWaitForConversion())) vPortYield();
}

bool isConversionComplete() {
	uint8_t b = ds18b20_read();
	return (b == 1);
}

uint16_t millisToWaitForConversion() {
	switch (bitResolution) {
	case 9:
		return 94;
	case 10:
		return 188;
	case 11:
		return 375;
	default:
		return 750;
	}
}

bool ds18b20_isConnected(const DeviceAddress *deviceAddress, uint8_t *scratchPad) {
	bool b = ds18b20_readScratchPad(deviceAddress, scratchPad);
	return b && !ds18b20_isAllZeros(scratchPad) && (ds18b20_crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}

uint8_t ds18b20_crc8(const uint8_t *addr, uint8_t len) {
	uint8_t crc = 0;
	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
			pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}
	return crc;
}

bool ds18b20_isAllZeros(const uint8_t * const scratchPad) {
	for (size_t i = 0; i < 9; i++) {
		if (scratchPad[i] != 0) {
			return false;
		}
	}
	return true;
}

float ds18b20_getTempF(const DeviceAddress *deviceAddress) {
	ScratchPad scratchPad;
	if (ds18b20_isConnected(deviceAddress, scratchPad)) {
		int16_t rawTemp = calculateTemperature(deviceAddress, scratchPad);
		if (rawTemp <= DEVICE_DISCONNECTED_RAW)
			return DEVICE_DISCONNECTED_F;
		// C = RAW/128
		// F = (C*1.8)+32 = (RAW/128*1.8)+32 = (RAW*0.0140625)+32
		return ((float)rawTemp * 0.0140625f) + 32.0f;
	}
	return DEVICE_DISCONNECTED_F;
}

float ds18b20_getTempC(const DeviceAddress *deviceAddress) {
	ScratchPad scratchPad;
	if (ds18b20_isConnected(deviceAddress, scratchPad)) {
		int16_t rawTemp = calculateTemperature(deviceAddress, scratchPad);
		if (rawTemp <= DEVICE_DISCONNECTED_RAW)
			return DEVICE_DISCONNECTED_C;
		// C = RAW/128
		// F = (C*1.8)+32 = (RAW/128*1.8)+32 = (RAW*0.0140625)+32
		return (float)rawTemp / 128.0f;
	}
	return DEVICE_DISCONNECTED_C;
}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int16_t calculateTemperature(const DeviceAddress *deviceAddress, uint8_t* scratchPad) {
	int16_t fpTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 11) | (((int16_t)scratchPad[TEMP_LSB]) << 3);
	return fpTemperature;
}

// Returns temperature from sensor
float ds18b20_get_temp(void) {
	if (init == 1) {
		unsigned char check;
		char temp1 = 0, temp2 = 0;
		check = ds18b20_RST_PULSE();
		if (check == 1)
		{
			ds18b20_send_byte(0xCC);
			ds18b20_send_byte(0x44);
			vTaskDelay(750 / portTICK_RATE_MS);
			check = ds18b20_RST_PULSE();
			ds18b20_send_byte(0xCC);
			ds18b20_send_byte(0xBE);
			temp1 = ds18b20_read_byte();
			temp2 = ds18b20_read_byte();
			check = ds18b20_RST_PULSE();
			float temp = 0;
			temp = (float)(temp1 + (temp2 * 256)) / 16;
			return temp;
		}
		else { return 0; }

	}
	else { return 0; }
}

void ds18b20_init(int GPIO) {
	DS_GPIO = GPIO;
	//gpio_pad_select_gpio(DS_GPIO);
	init = 1;
}

//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void reset_search() {
	devices = 0;
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = false;
	LastFamilyDiscrepancy = 0;
	for (int i = 7; i >= 0; i--) {
		ROM_NO[i] = 0;
	}
}
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search

bool search(uint8_t *newAddr, bool search_mode) {
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number;
	bool search_result;
	uint8_t id_bit, cmp_id_bit;

	unsigned char rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = false;

	// if the last call was not the last one
	if (!LastDeviceFlag) {
		// 1-Wire reset
		if (!ds18b20_reset()) {
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = false;
			LastFamilyDiscrepancy = 0;
			return false;
		}

		// issue the search command
		if (search_mode == true) {
			ds18b20_write_byte(0xF0);   // NORMAL SEARCH
		}
		else {
			ds18b20_write_byte(0xEC);   // CONDITIONAL SEARCH
		}

		// loop to do the search
		do {
			// read a bit and its complement
			id_bit = ds18b20_read();
			cmp_id_bit = ds18b20_read();

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1)) {
				break;
			}
			else {
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit) {
					search_direction = id_bit;  // bit write value for search
				}
				else {
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy) {
						search_direction = ((ROM_NO[rom_byte_number]
							& rom_byte_mask) > 0);
					}
					else {
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);
					}
					// if 0 was picked then record its position in LastZero
					if (search_direction == 0) {
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
							LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;

				// serial number search direction write bit
				ds18b20_write(search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0) {
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65)) {
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0) {
				LastDeviceFlag = true;
			}
			search_result = true;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !ROM_NO[0]) {
		devices = 0;
		LastDiscrepancy = 0;
		LastDeviceFlag = false;
		LastFamilyDiscrepancy = 0;
		search_result = false;
	}
	else {
		for (int i = 0; i < 8; i++) {
			newAddr[i] = ROM_NO[i];
		}
		devices++;
	}
	return search_result;
}


void insertArray(devicesArray *a, DeviceAddress devaddr) {
	if (ds18_count >= DS18B20MAX){
		bk_printf("insertArray ERROR:Arry is full, can't add device 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X ",
			devaddr[0],devaddr[1],devaddr[2],devaddr[3],devaddr[4],devaddr[5],devaddr[6],devaddr[7]);
	  	return;
	}
	bk_printf("insertArray - ds18_count=%i  -- adding device 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X ",ds18_count,
		devaddr[0],devaddr[1],devaddr[2],devaddr[3],devaddr[4],devaddr[5],devaddr[6],devaddr[7]);
	for (int i = 0; i < 8; i++) {
		a->array[ds18_count][i] = devaddr[i];
	}
	a->lasttemp[ds18_count] = -127;
	a->last_read[ds18_count] = 0;
	ds18_count++;
}


int DS18B20_fill_devicelist()
{
	DeviceAddress devaddr;
	int ret=0;
	while (search(devaddr,1) && ds18_count < DS18B20MAX ){
		ret++;
		bk_printf("found device 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X ",
			devaddr[0],devaddr[1],devaddr[2],devaddr[3],devaddr[4],devaddr[5],devaddr[6],devaddr[7]);
		insertArray(&ds18b20devices,devaddr);
		insertArray(&ds18b20devices,devaddr);
		ret++;
	}
	return ret;
};

// startDriver DS18B20 [conversionPeriod (seconds) - default 15]
void DS18B20_driver_Init()
{
	int Pin = PIN_FindPinIndexForRole(IOR_DS1820_IO, 99);
	if(Pin != 99)
	{
		ds18b20_init(Pin);
		DS18B20_fill_devicelist();
	}
	ds18_conversionPeriod = Tokenizer_GetArgIntegerDefault(1, 15);
	lastconv = 0;
};

void DS18B20_AppendInformationToHTTPIndexPage(http_request_t* request)
{
	hprintf255(request, "<h5>DS18B20 devices detected: %i</h5>",ds18_count);
	for (int i=0; i < ds18_count; i++) {
		hprintf255(request, "<h5>Device %i (0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X) temperature:",i,
		ds18b20devices.array[i][0],ds18b20devices.array[i][1],ds18b20devices.array[i][2],ds18b20devices.array[i][3],
		ds18b20devices.array[i][4],ds18b20devices.array[i][5],ds18b20devices.array[i][6],ds18b20devices.array[i][7]);
		if (ds18b20devices.lasttemp[i] > -127){
			hprintf255(request, " %0.2f (%i seconds ago)</h5>", ds18b20devices.lasttemp[i],ds18b20devices.last_read[i]);
		}		
		else 
		hprintf255(request, " -- </h5>");
	}
}


void DS18B20_OnEverySecond()
{
	// for now just find the pin used
	Pin = PIN_FindPinIndexForRole(IOR_DS1820_IO, 99);
	uint8_t scratchpad[9], crc;
	if(Pin != 99)
	{
		if (ds18_count == 0) DS18B20_fill_devicelist();
		// only if pin is set
		// request temp if conversion was requested two seconds after request
		// if (dsread == 1 && g_secondsElapsed % 5 == 2) {
		// better if we don't use parasitic power, we can check if conversion is ready		
		if(dsread == 1 && isConversionComplete())
		{
			float t = -127;
			bk_printf("Reading temperature from DS18B20 sensor(s)\r\n");
			for (int i=0; i < ds18_count; i++) {
				ds18b20devices.last_read[i] += 1 ;
				errcount = 0;
				t = -127;
				while ( t == -127 && errcount++ < 5){ 
					t = ds18b20_getTempC(ds18b20devices.array[i]);
					bk_printf("Device %i (0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X) reported %0.2f\r\n",i,
						ds18b20devices.array[i][0],ds18b20devices.array[i][1],ds18b20devices.array[i][2],ds18b20devices.array[i][3],
						ds18b20devices.array[i][4],ds18b20devices.array[i][5],ds18b20devices.array[i][6],ds18b20devices.array[i][7],t);
				}
				if (t != -127){
					ds18b20devices.lasttemp[i] = t;
					ds18b20devices.last_read[i] = 0;
					lastconv = g_secondsElapsed;
				} else{
					if (ds18b20devices.last_read[i] > 60) {
						bk_printf("No temperature read for over 60 seconds for"
							" device %i (0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X)! Setting to -127°C!\r\n",i,
							ds18b20devices.array[i][0],ds18b20devices.array[i][1],ds18b20devices.array[i][2],
							ds18b20devices.array[i][3],ds18b20devices.array[i][4],ds18b20devices.array[i][5],
							ds18b20devices.array[i][6],ds18b20devices.array[i][7]);
						ds18b20devices.lasttemp[i] = -127;	
					}
				}
				
			}
			dsread=0;
			
		}	
		else{
			for (int i=0; i < ds18_count; i++) {
				ds18b20devices.last_read[i] += 1 ;
			}
			if(dsread == 0 && (g_secondsElapsed % ds18_conversionPeriod == 0 || lastconv == 0))
			{
				ds18b20_requestTemperatures();
				dsread = 1;
				errcount = 0;
			}
		}
	}
}
