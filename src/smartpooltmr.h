/*
 * system.h
 *
 *  Created on: Jun 13, 2017
 *      Author: chiz
 */

#ifndef SRC_SMARTPOOLTMR_H_
#define SRC_SMARTPOOLTMR_H_

#include "common/cs_dbg.h"
#include "common/mbuf.h"
#include "common/json_utils.h"
#include "common/platform.h"
#include "frozen/frozen.h"
#include "fw/src/mgos_app.h"
#include "fw/src/mgos_gpio.h"
#include "fw/src/mgos_mongoose.h"
#include "fw/src/mgos_sys_config.h"
#include "fw/src/mgos_hal.h"
#include "fw/src/mgos_timers.h"
#include "fw/src/mgos_app.h"
#include "fw/src/mgos_wifi.h"
#include "fw/src/mgos_adc.h"
// #include "fw/src/mgos_i2c.h"
//#include "fw/src/mgos_spi.h"

// include from libraries
#include "mgos_i2c.h"
#include "mgos_spi.h"

// include from esp-idf
#include "driver/adc.h"

// include from src
#define BMP180_API			// This definition needed by bmp180.h
#include "bmp180.h"
#include "ade7912.h"


// Definitions
#define YEAR_NOW				2017
#define YEAR_1900				1900
#define TIME_ZONE_EST			(int)-4
#define ISTIMESYNCED(x)			((x+YEAR_1900) >= YEAR_NOW) ? 1 : 0
#define SSID_SIZE 				32 					// Maximum SSID size
#define PASSWORD_SIZE 			64 					// Maximum password size

#define RELAY_DRV_GPIO			21					// output
#define RELAY_FDBK_GPIO			4					// input
#define ADC_DRDY_GPIO			22					// input, interrupt enabled
#define ADC_CH5_GPIO			33					// Analog Input
#define MANUAL_OVRD_GPIO		34					// input
#define GPIO_LED_YEL_OUT		27
#define GPIO_LED_GRN_OUT		12

#define CALLBACK_PERIOD 		2000 				// Frequency a timer call back is repeated

// Define JSON formats for different data outputs
#define STATUS_FMT "{cpu: %d, ssid: %Q, ip: %Q}"
#define JSON_EPOOLTIMER_FMT "{volt: %.2f, cur: %.2f, pow: %.2f, temp: %.1f, press: %.1f, auto: %B, relay: %d}"

#define ANALOG_VDIV 				(1.0/0.3125)
#define NODEMCU_VREF 				3.3
#define ADC_ATTEN					2
#define ADC_MAX_RES					(float)4095

#define HDWR_VERSION				0x3033
#define SFWR_VERSION				0x3032

#define NUM_WEEKDAYS				7
#define OUTPUTBUFSIZE				16

struct device_settings {
  char ssid[SSID_SIZE];
  char pswd[PASSWORD_SIZE];
};

struct bmp180_data_t {
	float temperature;
	float pressure;
};

//
// Create dpt_system types
//

typedef enum
{
	eRELAY_OFF = 0,
	eRELAY_ON = 1,
	eRELAY_FAIL = 2
} relay_status_t;

typedef enum
{
	ePUMP_OFF = 0,
	ePUMP_ON = 1
} pump_status_t;

typedef enum
{
	eSYS_NORMAL = 0,
	eMANUAL_OVRD = 1,
	eSYS_FAULT = 2
} system_state_t;

// System Data Structure
typedef struct
{
    uint8_t 	pumpCmd;
    float  		box_temperature;
    float	 	box_pressure;
    float 		pump_voltage;
    float 		pump_current;
    float	    pump_power;
    float 		dc_input_voltage;
    float 		filter_pressure;
    double 		upTime;
    int32_t		pump_offset_v;
    int32_t		pump_offset_i;
    uint8_t 	rly1Status;
    uint8_t 	rly2Status;
    uint8_t 	pumpFdbkState;
    uint8_t 	manOverride;
    uint8_t 	pumpStatus;
    uint8_t		useSchedule;
    uint8_t 	sysState;
    uint8_t		swVersion[2];
    uint8_t		hwVersion[2];
} dpt_system_t;

// structure for schedule
typedef struct tmSchedule {
	int tm_min_start;
	int tm_hour_start;
	int tm_min_end;
	int tm_hour_end;
	int tm_wday;
} tmSchedule_t;

// Function declarations
s32 bmp180_sensor_initialize(void);
void bmp180_sensor_data(struct bmp180_data_t *bmp180_sensor_data);
void delay_ms(u32 msec);
void deviceInit(void);
uint8_t cmdPumpOnOff(relay_status_t cmd);
void user_init(void);
void checkPumpSchedule(void);
void getDefaultSchedule(tmSchedule_t *schedule);

void becomeStation(struct sys_config_wifi_sta *device_cfg_sta);
void report_state(void);
void update_state(void);
bool savePumpSchedule(void);
bool getPumpSchedule(void);

#endif /* SRC_SMARTPOOLTMR_H_ */
