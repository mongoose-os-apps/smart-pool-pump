//#include "bmp180.h"

#include <math.h>
#include <string.h>
#include "smartpooltmr.h"
#include "ade7912.h"

#define HIGH true
#define LOW	false

// Extern Variable Declaration
extern dpt_system_t system_data;
struct mgos_spi_txn gSPIHandle;

/*
 * if the measurement voltage is 60Hz, then number of samples required for
 * proper AC measurement depends on the ADC frequency.
 * If ADC frequency is 8KHz, the ADE7912 captures 512 samples. If the frequency
 * is 4kHZ, then ADE7912 captures 1024 samples.
 * The sample size = 4096000/ADC Frequency
 * These samples are sufficient to reproduce the waveform, being greater than twice
 * the Nyquist sampling rate
 *
 */

// Global Initialized Variables
uint16_t ADE7912_RX_QSIZE			= 0;
uint16_t ADE7912_LAST_SAMPLE		= 0;
uint8_t adc_int_num 				= 0;
bool gAdcCaptureDone 				= false;
ade7912_result_t *adc_sample_buffer = NULL;
ade7912_capture_mode_t adc_mode 	= eSTARTUP;

/*
 * ADC Interrupt Handler. ISR is called when DRDY_GPIO transitions low
 * Interrupt Handler
 *
 */
void my_adc_int_handler(int pin, void *arg)
{
	ade7912_result_t result;
	adc_int_num = (uint32_t) arg;

	ade7912_get_data(&result);
	result.sample = ADE7912_LAST_SAMPLE;
	adc_sample_buffer[ADE7912_LAST_SAMPLE] = result;

	if (++ADE7912_LAST_SAMPLE == ADE7912_RX_QSIZE) {
		mgos_gpio_disable_int(ADC_DRDY_GPIO);
		gAdcCaptureDone = true;
		ADE7912_LAST_SAMPLE = 0;
	}

	(void)pin;
}


/**
 * @brief spi master initialization
 */
bool spi_master_init(void)
{
	struct mgos_spi *spi = mgos_spi_get_global();

	/* SPI is configured by mongoose
	 * Frequency is not set by mgos_spi_configure
	 */
    if (spi == NULL) {
    	LOG(LL_DEBUG, ("SPI not configured \n"));
    	return false;
    }

    gSPIHandle.cs = 0;
    gSPIHandle.freq = 1000000;
    gSPIHandle.mode = 3;

	mgos_gpio_set_mode(ADC_DRDY_GPIO, MGOS_GPIO_MODE_INPUT);
	mgos_gpio_set_pull(ADC_DRDY_GPIO, MGOS_GPIO_PULL_UP);
	/*
     * Install a GPIO interrupt handler.
     *
     * Note that this will not enable the interrupt, this must be done explicitly
     * with mgos_gpio_enable_int.
     */
    mgos_gpio_set_int_handler(ADC_DRDY_GPIO, MGOS_GPIO_INT_EDGE_NEG, my_adc_int_handler, NULL);
    return true;
}


/* Initialize ADC
 * Enable DRDY GPIO Interrupt
 * Create ADC Measurement Task
 	printf ("in mgos_app_init \r\n");
 */
bool ade7912_init(uint8_t adc_freq)
{
	ade7912_register_t adc_register;

	// configure SPI
	if (!spi_master_init()) {
		LOG(LL_DEBUG, ("SPI not configured \n"));
		return false;
	}

	// Determine that ADE7912 is ready to accept commands
	delay_ms(200);
	if (ade7912_reg_access(ADE7912_SPI_READ, eSTATUS0, 0) & RESET_ON_MASK) {
		LOG(LL_DEBUG, ("ADC NOT ready to accept commands \n"));
		return false;
	}

	LOG(LL_DEBUG, ("ADE7912 ready. Configuring...\n"));

	// Write ADE7912 Configure ADC frequency, bandwidth enabled
	ade7912_reg_access(ADE7912_SPI_WRITE, eCONFIG, ADC_FREQ_SET(adc_freq) | BW_MASK);

	// Get value from ADC Config Register
	adc_register.config = ade7912_reg_access(ADE7912_SPI_READ, eCONFIG, 0);

	if (adc_register.config != (ADC_FREQ_SET(adc_freq) | BW_MASK)) {
		LOG(LL_DEBUG, ("ADE7912 CONFIG. Wrote: 0x%x, Read:0x%x\n", (ADC_FREQ_SET(adc_freq) | BW_MASK), adc_register.config));
		return false;
	}

	// Set ADE7912 EMI Control
	ade7912_reg_access(ADE7912_SPI_WRITE, eEMI_CTRL, 0x55);

	// Set Lock Config Register
	ade7912_reg_access(ADE7912_SPI_WRITE, eLOCK, CONFIG_PROT_ENABLE);

	switch (adc_freq) {
		case eFRQ8khz:
			ADE7912_RX_QSIZE = ADE7912_CLKIN/8000;
			break;

		case eFRQ4khz:
			ADE7912_RX_QSIZE = ADE7912_CLKIN/4000;
			break;

		case eFRQ2khz:
			ADE7912_RX_QSIZE = ADE7912_CLKIN/2000;
			break;

		case eFRQ1khz: default:
			ADE7912_RX_QSIZE = ADE7912_CLKIN/1000;
			break;
	}

	/* if init was called before, free memory location before
	 * allocating new memory block
	 */
	if (adc_sample_buffer != NULL) {
		free(adc_sample_buffer);
	}
	adc_sample_buffer = malloc(ADE7912_RX_QSIZE*sizeof(ade7912_result_t));

	LOG(LL_DEBUG, ("ADE7912 Config Complete! Enabling Interrupt\n"));

	/* enable interrupt to perform conversions	*/
	gAdcCaptureDone = false;
	mgos_gpio_enable_int(ADC_DRDY_GPIO);

	return true;
}

/*
 * Write/Read from ADC register
 */
uint8_t ade7912_reg_access(uint8_t cmd, uint8_t address, uint8_t data)
{
	/* The NodeMCU is little ENDIAN	*/

	uint8_t obuf[2] = {(ADE7912_ADDR_SET(address) | cmd), data};
	uint8_t ibuf[2] = {'\0'};

//	LOG(LL_DEBUG, ("ADE7912 SPI data out: 0x%x\n", obuf[0]));
	gSPIHandle.fd.tx_data =  obuf;
	gSPIHandle.fd.rx_data = ibuf;
	gSPIHandle.fd.len = sizeof(obuf)/sizeof(uint8_t);

	mgos_spi_run_txn(mgos_spi_get_global(), true, &gSPIHandle);
//	LOG(LL_DEBUG, ("ADE7912 SPI data in: 0x%x\n", ibuf[1]));
	return ibuf[1];
}


/*
 * Read conversion data results
 */
void ade7912_get_data(ade7912_result_t *dReg)
{
	uint8_t obuf[4] = {0};
	uint8_t ibuf[4] = {0};

	// Read ADC Samples
	obuf[0] = (ADE7912_ADDR_SET(eIWV) | ADE7912_SPI_READ);
	gSPIHandle.fd.tx_data = obuf;
	gSPIHandle.fd.rx_data = ibuf;
	gSPIHandle.fd.len = sizeof(obuf)/sizeof(uint8_t);
	mgos_spi_run_txn(mgos_spi_get_global(), true, &gSPIHandle);

	// Remove high order 8 bits
	dReg->irms_counts = (ibuf[1] << 16) | (ibuf[2] << 8) | ibuf[3];

	// Read ADC Samples
	obuf[0] = (ADE7912_ADDR_SET(eV1WV) | ADE7912_SPI_READ);
	gSPIHandle.fd.tx_data = obuf;
	gSPIHandle.fd.rx_data = ibuf;
	gSPIHandle.fd.len = sizeof(obuf)/sizeof(uint8_t);
	mgos_spi_run_txn(mgos_spi_get_global(), true, &gSPIHandle);

	// Remove high order 8 bits
	dReg->vrms_counts = (ibuf[1] << 16) | (ibuf[2] << 8) | ibuf[3];
}



/*
 * ade7912_trigger_capture: Initiates ADC measurement
 */
bool ade7912_trigger_capture(void)
{
    // Data to be received from user
	ade7912_result_t *rxResult;

	uint16_t sample_index = 0;
	ade7912_voltage_t calcPvoltage = 0.0;
	ade7912_current_t calcIcurrent	= 0.0;
	ade7912_voltage_t instantVoltage = 0.0;
	ade7912_current_t instantCurrent = 0.0;
	int32_t vOffset = 0;
	int32_t iOffset = 0;

    if (!gAdcCaptureDone) {
     	LOG(LL_DEBUG, ("ADC capture NOT complete [gAdcCaptureDone = %d] \n", gAdcCaptureDone));
    	return false;		// Wait for conversion to finish
    }

	rxResult = adc_sample_buffer;

	for (sample_index = 0; sample_index < ADE7912_RX_QSIZE; sample_index++) {
		if (rxResult->vrms_counts & ADC_MIN)	// negative measurement
			instantVoltage = V_GAIN * ((int32_t)((0xFF << 24) | rxResult->vrms_counts) + system_data.pump_offset_v);
		else
			instantVoltage = V_GAIN * (int32_t)(rxResult->vrms_counts - system_data.pump_offset_v);

		if (rxResult->irms_counts & ADC_MIN)	// negative measurement
			instantCurrent = I_GAIN *  ((int32_t)((0xFF << 24) | rxResult->irms_counts) + system_data.pump_offset_i);
		else
			instantCurrent = I_GAIN * (int32_t)(rxResult->irms_counts - system_data.pump_offset_i);

		calcPvoltage += pow(instantVoltage, 2);
		calcIcurrent += pow(instantCurrent, 2);

		/* Capture offsets, if in STARTUP or TARE mode	*/
		vOffset += rxResult->vrms_counts;
		iOffset += rxResult->irms_counts;

		rxResult++;
	}

	/* At this point sample_index is equal to ADE7912_RX_QSIZE	*/
	if (sample_index > 0) {
		if ((adc_mode == eSTARTUP) || (adc_mode == eTARE))
		{
			system_data.pump_offset_v = vOffset/sample_index;
			system_data.pump_offset_i = iOffset/sample_index;
			if (adc_mode == eSTARTUP)
				adc_mode = eMEASURE;
		}
		/* Calculate actual pump voltage and pump current	*/
		system_data.pump_voltage = sqrt((double)calcPvoltage/sample_index);
		system_data.pump_current = sqrt((double)calcIcurrent/sample_index);
	}

 	LOG(LL_DEBUG, ("ADC capture COMPLETE! [gAdcCaptureDone = %d]\n", gAdcCaptureDone));

	/* enable interrupt to perform conversions	*/
	gAdcCaptureDone = false;
	mgos_gpio_enable_int(ADC_DRDY_GPIO);

	return true;
}

