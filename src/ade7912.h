/*
 * ade7912.h
 *
 *  Created on: 08/03/2016
 *      Author: Chiz Chikwendu
 */

#ifndef DRIVER_ADE7912_H_
#define DRIVER_ADE7912_H_

#include "stdint.h"
#include "stdbool.h"

#define ADE7912_CLKIN				4096000

// ADC gains and constants
#define ADC_MIN		0x800000								// -8388608
#define ADC_MAX		(float)8388607							// 0x7FFFFF
#define ADC_VREF	1.2										// Volts
#define ADC_REFV	0.788
#define ADC_REFI	0.04927
#define V_GAIN		(ADC_REFV/ADC_MAX)*(float)((330*3)/1)
#define I_GAIN		(ADC_REFI/ADC_MAX)*(float)1000			// 1/R ==> 1/0.001
//#define V_OFFSET	(0.035/ADC_REFV) * ADC_MAX				// 35mV typical offset per datasheet
//#define I_OFFSET	(0.002/ADC_REFI) * ADC_MAX				// 2mV typical offset per datasheet

// Define SPI Registers and Contents
// SPI Register Addresses
typedef enum {
	eIWV = 0,
	eV1WV = 1,
	eV2WV = 2,
	eRSVD3 = 3,
	eADC_CRC = 4,
	eCTRL_CRC = 5,
	eRSVD6 = 6,
	eCNT_SNAPSHOT = 7,
	eCONFIG = 8,
	eSTATUS0 = 9,
	eLOCK = 10,
	eSYNC_SNAP = 11,
	eCOUNTER0 = 12,
	eCOUNTER1 = 13,
	eEMI_CTRL = 14,
	eSTATUS1 = 15,
	eTEMPOS = 24,
} ade7912_reg_addr_t;

typedef struct {
	uint32_t iwv;
	uint32_t v1wv;
	uint32_t v2wv;
	uint32_t rsvd3;
	uint16_t adc_crc;
	uint16_t ctrl_crc;
	uint16_t rsv6;
	uint16_t cnt_snapshot;
	uint8_t  config;
	uint8_t  status0;
	uint8_t  lock;
	uint8_t  sync_snap;
	uint8_t  counter0;
	uint8_t  counter1;
	uint8_t  emi_ctrl;
	uint8_t  status1;
	uint8_t  rsvd16[8];
	uint8_t  tempos;
} ade7912_register_t;

typedef union {
	uint8_t reg8[2];
	uint16_t reg16;
} reg16_t;

typedef union {
	uint8_t reg8[4];
	uint16_t reg16[2];
	uint16_t reg32;
} reg32_t;

// Command & Address Register
#define ADE7912_SPI_READ	(1 << 2)
#define ADE7912_SPI_WRITE	0x00
#define ADE7912_ADDR_MASK	0xF8
#define ADE7912_ADDR_SHIFT	3
#define ADE7912_ADDR_SET(x)	((x << 3) & ADE7912_ADDR_MASK)

// Config Register Parameters
#define CLKOUT_EN_SHIFT		0
#define CLKOUT_EN_MASK		(1 << CLKOUT_EN_SHIFT)
#define PWRDN_EN_SHIFT		2
#define PWRDN_EN_MASK		(1 << PWRDN_EN_SHIFT)
#define TEMP_EN_SHIFT		3
#define TEMP_EN_MASK		(1 << TEMP_EN_SHIFT)
#define ADC_FREQ_SHIFT		4
#define ADC_FREQ_MASK 		(3 << ADC_FREQ_SHIFT)
#define ADC_FREQ_SET(x) 	((x << ADC_FREQ_SHIFT) & ADC_FREQ_MASK)
#define SWRST_SHIFT			6
#define SWRST_MASK			(1 << SWRST_SHIFT)
#define BW_SHIFT			7
#define BW_MASK				(1 << BW_SHIFT)

#define CONFIG_PROT_ENABLE	0xCA
#define CONFIG_PROT_DISABLE	0x9C

typedef enum ade7912_adc_freq {
	eFRQ8khz = 0,
	eFRQ4khz = 1,
	eFRQ2khz = 2,
	eFRQ1khz = 3,
} ade7912_adc_freq_t;


typedef enum ade7912_capture_mode {
	eSTARTUP = 0,
	eMEASURE,
	eTARE,
} ade7912_capture_mode_t;

// Status0 Register Parameters
#define RESET_ON_MASK		1
#define CRC_STAT_MASK		(1 << 1)
#define IC_PROT_MASK		(1 << 2)

// Sync Snap Register Parameters
#define SYNC_MASK			1
#define SNAP_MASK			(1 << 1)


// Status1 Register Parameters
#define VERSION_MASK		7
#define ADC_NA_MASK			(1 << 3)

// Create ade7912_types
//

// voltage in Vrms
typedef float ade7912_voltage_t;
// current in amps
typedef float ade7912_current_t;

// ADE7912 result structure
typedef struct
{
    uint16_t sample;
    uint32_t vrms_counts;
    uint32_t irms_counts;
} ade7912_result_t;

// initialize SPI master
bool spi_master_init(void);

// Init ade7912 driver ...
bool ade7912_init(uint8_t adc_freq);

// Function allows SPI register access to the ADC
uint8_t ade7912_reg_access(uint8_t cmd, uint8_t address, uint8_t data);

// Reads ADC results via ADC
void ade7912_get_data(ade7912_result_t *dReg);

// Triggers and takes samples of ADC
bool ade7912_trigger_capture(void);

#endif /* DRIVER_ADE7912_H_ */
