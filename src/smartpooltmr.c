/*
 * Copyright 2016-2021 Chiz Chikwendu
 *
 */
/**
 * @file smartpooltmr.c
 * @brief glue code for interface with spi, i2c and gpio
 *
  */
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "smartpooltmr.h"

/* Externally Defined Variables	*/
extern ade7912_capture_mode_t adc_mode;
extern dpt_system_t system_data;
extern tmSchedule_t pump_schedule[NUM_WEEKDAYS];

#define	  C_BMP180_ONE_U8X			((u8)1)

/*----------------------------------------------------------------------------
  * The following functions are used for reading and writing of
  * sensor data using I2C or SPI communication
----------------------------------------------------------------------------*/
#ifdef BMP180_API
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP180_delay_msek(u32 msek);
/*
 * \Brief: I2C init routine
*/
s8 I2C_routine(void);
#endif
/********************End of I2C function declarations***********************/


/**
 * @brief i2c master initialization
 */
void i2c_master_init(void)
{
	// By default I2C is configured for 100kHz
	// mos.yml defines pinout for I2C
	// I2C lines configured with pull-ups
    struct mgos_i2c *i2c = mgos_i2c_get_global();		// gets pointer to the I2C master structure

    if (i2c == NULL) {
    	LOG(LL_DEBUG, ("I2C not configured \n"));
    }
}

/**
 * @brief i2c write code
 *        Master device write data to slave
 */
bool i2c_master_write_slave(uint8_t dev_addr, uint8_t* data_wr, size_t size)
{
	uint8_t reg = *data_wr;
    struct mgos_i2c *i2c = mgos_i2c_get_global();		// gets pointer to the I2C master structure
	return mgos_i2c_write_reg_n(i2c, (uint16_t) dev_addr, reg, size-C_BMP180_ONE_U8X, ++data_wr);
}

/**
 * @brief code to master to read from slave
 *
 */
bool i2c_master_read_slave(uint8_t dev_addr, uint8_t* data_rd, size_t size)
{
	uint8_t reg = *data_rd;
	struct mgos_i2c *i2c = mgos_i2c_get_global();		// gets pointer to the I2C master structure
	return mgos_i2c_read_reg_n(i2c, (uint16_t) dev_addr, reg, size-C_BMP180_ONE_U8X, data_rd);
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
//void BMP180_delay_msek(u32 msek);


/*----------------------------------------------------------------------------
 struct bmp180_t parameters can be accessed by using bmp180
 *	bmp180_t having the following parameters
 *	Bus write function pointer: BMP180_WR_FUNC_PTR
 *	Bus read function pointer: BMP180_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *	Calibration parameters
 ---------------------------------------------------------------------------*/
struct bmp180_t bmp180;

s32 bmp180_sensor_initialize(void)
{
	/* communication results/flags	*/
	s32 ret = E_BMP_COMM_RES;

	/*--------------------- START INITIALIZATION ---------------*/
	i2c_master_init();
	#ifdef BMP180_API
	I2C_routine();
	#endif

	/*
	 *  This function used to assign the value/reference of
	 *	the following parameters
	 *	I2C address
	 *	Bus Write
	 *	Bus read
	 *	Chip id
	 *	Calibration values
	*/
	ret = bmp180_init(&bmp180);

	/*
	 *--------------------START CALIPRATION ------------------------------
	 *  This function used to read the calibration values of following
	 *	these values are used to calculate the true pressure and temperature
	 */
	ret += bmp180_get_calib_param();

	return ret;
}

void bmp180_sensor_data(struct bmp180_data_t *bmp180_sensor_data)
{
	u16 v_uncomp_temp_u16 = BMP180_INIT_VALUE;
	u32 v_uncomp_press_u32 = BMP180_INIT_VALUE;

	/*	This API is used to read the
	*	uncompensated temperature(ut) value
	*/
	v_uncomp_temp_u16 = bmp180_get_uncomp_temperature();

	/*	This API is used to read the
	*	uncompensated pressure(ut) value
	*/
	v_uncomp_press_u32 = bmp180_get_uncomp_pressure();

/****************************************************************************
 *	This API is used to read the
 *	true temperature(t) value input
 *	parameter as uncompensated temperature(ut)
 *
 ****************************************ret***********************************/
	bmp180_sensor_data->temperature = (float)bmp180_get_temperature(v_uncomp_temp_u16) * 0.1;

/****************************************************************************
 *	This API is used to read the
 *	true pressure(p) value
 *	input parameter as uncompensated pressure(up)
 *	Convert from hPA to PSI
 *
 ***************************************************************************/
	bmp180_sensor_data->pressure = ((float)bmp180_get_pressure(v_uncomp_press_u32) * 0.01) * 0.014503773773022;
}


#ifdef BMP180_API

/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmp180_t
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp180 the following structure parameter can be accessed
 *	Bus write function pointer: BMP180_WR_FUNC_PTR
 *	Bus read function pointer: BMP180_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bmp180.bus_write = BMP180_I2C_bus_write;

	bmp180.bus_read = BMP180_I2C_bus_read;
	bmp180.dev_addr = BMP180_I2C_ADDR;
	bmp180.delay_msec = BMP180_delay_msek;

	return BMP180_INIT_VALUE;
}

/************** I2C buffer length ******/

#define	I2C_BUFFER_LEN 8
#define I2C0 5

/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP180_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMP180_INIT_VALUE;
	array[BMP180_INIT_VALUE] = reg_addr;

	for (stringpos = BMP180_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos + C_BMP180_ONE_U8X] = *(reg_data + stringpos);
	}

	// write to slave device
	iError = i2c_master_write_slave(dev_addr, array, cnt+C_BMP180_ONE_U8X);

	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\paramICACHE_FLASH_ATTR dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP180_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BMP180_INIT_VALUE};
	u8 stringpos = BMP180_INIT_VALUE;
	array[BMP180_INIT_VALUE] = reg_addr;

	// Read from slave device
	iError = i2c_master_read_slave(dev_addr, array, cnt+C_BMP180_ONE_U8X);

	for (stringpos = BMP180_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}
	return (s8)iError;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP180_delay_msek(u32 msek)
{
	delay_ms(msek);
}

#endif

// Delay function
void delay_ms(u32 msec)
{
	/*Here you can write your own delay routine*/
	for (u32 ms_delay=0; ms_delay < msec; ms_delay++)
		mgos_usleep(1000);
}


/*
 * getDefaultSchedule: Initialize structure with pump default
 * schedule
 *
 */
void getDefaultSchedule(tmSchedule_t *schedule){

	schedule->tm_hour_start = 5;	// 6am
	schedule->tm_min_start = 0;
	schedule->tm_hour_end = 8;		// 8am
	schedule->tm_min_end = 0;
}


/*
 * deviceInit: Configures peripherals used
 *
 */
void deviceInit(void)
{
	LOG(LL_DEBUG, ("Initialize GPIO ports \n"));

	mgos_gpio_set_mode(MANUAL_OVRD_GPIO, MGOS_GPIO_MODE_INPUT);
	mgos_gpio_set_pull(MANUAL_OVRD_GPIO, MGOS_GPIO_PULL_NONE);

	mgos_gpio_set_mode(RELAY_DRV_GPIO, MGOS_GPIO_MODE_OUTPUT);
	mgos_gpio_set_pull(RELAY_DRV_GPIO, MGOS_GPIO_PULL_NONE);
	mgos_gpio_write(RELAY_DRV_GPIO, eRELAY_OFF);

	system_data.hwVersion[0] = HDWR_VERSION >> 8;
	system_data.hwVersion[1] = HDWR_VERSION & 0xFF;
	system_data.swVersion[0] = SFWR_VERSION >> 8;
	system_data.swVersion[1] = SFWR_VERSION & 0xFF;

	// Configure ADC
	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_6db);

	// Initialize Pump Schedule
	if (!getPumpSchedule()) {
		// This is a virgin board with no pump schedule
		LOG(LL_INFO, ("Creating default schedule..."));
		for (uint8_t i=0; i<NUM_WEEKDAYS; i++) {
			pump_schedule[i].tm_wday = i;
			getDefaultSchedule(&pump_schedule[i]);
		}
		// Save default schedule file
		savePumpSchedule();
		LOG(LL_INFO, ("Saved Schedule file %s\n", "pump_schedule.txt"));
	}
}

/*
 * cmdPumpOnOff: Turn pump on or off
 *
 */
uint8_t cmdPumpOnOff(relay_status_t cmd)
{
	if (adc_mode == eMEASURE) {
		if (cmd)
			mgos_gpio_write(RELAY_DRV_GPIO, eRELAY_ON);
		else
			mgos_gpio_write(RELAY_DRV_GPIO, eRELAY_OFF);
		// Give relay sometime to switch
		mgos_usleep(5000);
	}

    return mgos_gpio_read(RELAY_DRV_GPIO);
}

/*
 * savePumpSchedule: Save Pump Schedule
 *
 */
bool savePumpSchedule(void)
{
	FILE *fp;
	size_t size_of_elements = sizeof(pump_schedule[0]);
	size_t number_of_elements = sizeof(pump_schedule)/size_of_elements;

	fp = fopen("pump_schedule.txt", "wb");

	if (fp == NULL)
		return false;

	fwrite(&pump_schedule, size_of_elements, number_of_elements, fp);
	LOG(LL_DEBUG, ("Wrote %d bytes\n", number_of_elements*size_of_elements));

    return true;
}


/*
 * getPumpSchedule: Save Pump Schedule
 *
 */
bool getPumpSchedule(void)
{
	FILE *fp;
	size_t size_of_elements = sizeof(pump_schedule[0]);
	size_t number_of_elements = sizeof(pump_schedule)/size_of_elements;

	fp = fopen("pump_schedule.txt", "rb");

	if (fp == NULL)
		return false;

	fread(&pump_schedule, size_of_elements, number_of_elements, fp);
	LOG(LL_DEBUG, ("Read %d bytes\n", number_of_elements*size_of_elements));

    return true;
}
