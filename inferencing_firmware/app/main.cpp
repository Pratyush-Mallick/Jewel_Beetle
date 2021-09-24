/**
 * Jewel Bettle: Forest Fire detection System
 * 
 * Author: Pratyush Mallick
 * Date: Augutst 15, 2021
 * 
 * Pin Map Used: 
 * MCU (EV-COG-4050LZ) <--> Peripheral
 * D10 <-----> CS (BME688)
 * D11 <-----> SDO (BME688)
 * D12 <-----> SDI (BME688)
 * D13 <-----> SCK (BME688)
 * 
 * Change the #define STDIO_UART_TX/Rx to USBTX1/RX1 in PeripheralName.h file
 * P1_15 (USBTX1) <-----> D3 (Xbee DataIn)
 * P2_00 (USBRX1) <-----> D2 (Xbee DataOut)
 * D7 <-----> D9 (Xbee sleep pin)
 *  
 * P0_12 <-----> DONE pin (nano power timer)
 * 
 *********************************************************************************
 * Copyright (c) 2021 Pratyush Mallick
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "mbed.h"
#include "ei_run_classifier.h"
#include "numpy.hpp"
#include "bme_688/bme68x.h"
#include <cstdio>
#include <cstdlib>
#include <stdint.h>

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/
#define PIN_SPI_SS SPI0_CS2
 
/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/
/* Keep track of the time is seconds to put system to shutdown */
static uint64_t time_elapse = 0;

/**
  * @enum fire_status
  * @brief Possible status of wildfire
  */
enum fire_status {
	FIRE,
	NORMAL
};

/* To store the enviromental data features to feed into the model */
static float features[15];

/* Bmee688 specific declarations */
uint8_t n_fields;
struct bme68x_dev bme_dev;
struct bme68x_conf gas_conf;
struct bme68x_heatr_conf gas_heatr_conf;
struct bme68x_data data;

/* Declaration of mbed objects of the class */
DigitalOut pc_activity(LED1);
SPI spi_dev(D11, D12, D13);
DigitalOut slaveSelect_Pin(PIN_SPI_SS);  
DigitalOut done_pin(P0_12);   
DigitalOut xbee(D7);

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
static void spi_dev_config(void);
static bool bme_init(void);
static int8_t spi_bme_read(uint8_t reg_addr,
	uint8_t *reg_data,
	uint32_t len,
	void *intf_ptr);
static int8_t spi_bme_write(uint8_t reg_addr,
	const uint8_t *reg_data,
	uint32_t len,
	void *intf_ptr); 
static void udelay(uint32_t us, void *intf_ptr);
 
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
/*!
 * @brief	Transfer the data from feature into th memory buffer
 * @param	offset[in] - User buffer offset
 * @param	length[in] - Length of the buffer to copy
 * @param	out_ptr[out] - Pointer to the buffer
 * @return	Menu status constant
 */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
	memcpy(out_ptr, features + offset, length * sizeof(float));
	return 0;
}

/**
 * @brief	Main entry point to application
 * @return	none
 */
int main() {
	
	int8_t rslt;
	
	/* Putting xbee into sleep mode */
	xbee = 1;
	wait_us(100);   
	
	/* SPI configuration */
	spi_dev_config();
	
	/* SPI initialization */
	if (!bme_init()) {
		printf("\r\nBME init Error");
		while (1) ;
	}
	
	printf("Starting Forest Fire inferencing (Mbed)\n");
	if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
		printf("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
			EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
			sizeof(features) / sizeof(float));
		return 1;
	}
	ei_impulse_result_t result = { 0 };

	while (1) {

		memset(features, 0, sizeof(features));
		for (uint8_t i = 0; i < 5; i++) {
			
			/* Led blinking to indicate start of operation */
			pc_activity = ~pc_activity;
			
			/* Fetching data from bme688 */
			rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme_dev);
			uint32_t delayus_period = (uint32_t)bme68x_get_meas_dur(
						  BME68X_FORCED_MODE,
				&gas_conf,
				&bme_dev) + 
				((uint32_t)gas_heatr_conf.heatr_dur * 1000);
			bme_dev.delay_us(delayus_period, bme_dev.intf_ptr);
			bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme_dev);
			
			data.humidity = round(data.humidity * 100) / 100;
			data.temperature = round(data.temperature * 100) / 100;
			data.gas_resistance = data.gas_resistance / 100;
			features[(i * 3) + 0] = data.humidity;
			features[(i * 3) + 1] = data.temperature;
			features[(i * 3) + 2] = data.gas_resistance;
			time_elapse = time_elapse + (delayus_period / 1000);
		}
		
		// the features are stored into flash, and we don't want to load everything into RAM
		signal_t features_signal;
		features_signal.total_length = sizeof(features) / sizeof(features[0]);
		features_signal.get_data = &raw_feature_get_data;

		// invoke the impulse
		EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

		if (res != 0) return 1;
		
		/* Start sending data only if fire event is detected */ 
		if (result.classification[FIRE].value > 0.8) {
			/* Putting xbee into active mode */ 
			xbee = 0;
			wait_us(100);
			
			printf("\r\nH:%0.2f T:%0.2f G:%0.2f Result:%0.2f", data.humidity, 
				data.temperature, data.gas_resistance, result.classification[FIRE].value); 
			
			/* Putting xbee into sleep mode */
			xbee = 1;
			wait_us(100);  
		}
		
		/* If everything is normal put system into shutdown mode */ 
		else if (time_elapse / 1000 > 20) {
			done_pin = 0;
			wait_ns(500);
			done_pin = 1;
			wait_us(10000);
		}
	}
}

/*!
 * @brief	Configures the SPI settings
 * @param	none
 * @return	none
 */
static void spi_dev_config(void)
{
	spi_dev.format(8, 0);
	spi_dev.frequency(1000000);
	spi_dev.set_default_write_value(0x00);
}

/*!
 * @brief	Initializes the bme688 interface and device
 * @param	none
 * @return	true if successfull, otherwise false
 */
static bool bme_init(void)
{
	bme_dev.chip_id = BME68X_CHIP_ID;
	bme_dev.variant_id = BME68X_REG_VARIANT_ID;
	bme_dev.intf_ptr = (SPI *)&spi_dev;
	bme_dev.intf = BME68X_SPI_INTF;
	bme_dev.read = &spi_bme_read;
	bme_dev.write = &spi_bme_write;

	bme_dev.amb_temp = 25; 
	bme_dev.delay_us = udelay;

	int8_t ret = bme68x_init(&bme_dev);
	if (ret != BME68X_OK) {
		printf("\r\n Cannot find device");
		while (1) ;
	}
	
	gas_conf.filter = BME68X_FILTER_SIZE_3;
	gas_conf.odr = BME68X_ODR_NONE;
	gas_conf.os_hum = BME68X_OS_2X;
	gas_conf.os_pres = BME68X_OS_4X;
	gas_conf.os_temp = BME68X_OS_2X;
	gas_heatr_conf.enable = BME68X_ENABLE;
	gas_heatr_conf.heatr_temp = 320;
	gas_heatr_conf.heatr_dur = 150;

	ret = bme68x_set_conf(&gas_conf, &bme_dev);
	
	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	gas_heatr_conf.enable = BME68X_ENABLE;
	//gas_heatr_conf.heatr_temp = 300;
	//gas_heatr_conf.heatr_dur = 100;
	ret = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &bme_dev);
	
	return true;
}

/*!
 * @brief	Mbed spcific spi read function for bme688
 * @param   reg_addr[in] - Register address to read from
 * @param   reg_data[out] - Pointer to buffer store the register content
 * @param   len[in] - Number of bytes to read
 * @param   intf_ptr[i] - not used here
 * @return	zero
 */
static int8_t spi_bme_read(uint8_t reg_addr,
	uint8_t *reg_data,
	uint32_t len,
	void *intf_ptr)
{
	SPI *dev = (mbed::SPI *)intf_ptr;
	
	slaveSelect_Pin = 0;
	
	dev->write(reg_addr);
	
	for (uint32_t i = 0;i < len;i++)
	{
		reg_data[i]  = dev->write(0x00);
	}
	//dev->write((char *)&reg_addr, 1, (char *)&reg_addr, 1);
	//dev->write((char *)reg_data, len, (char *)reg_data, len);
	
	slaveSelect_Pin = 1;

	return 0;
}

/*!
 * @brief	Mbed spcific spi write function for bme688
 * @param   reg_addr[in] - Register address to write to
 * @param   reg_data[out] - Pointer to buffer storing data to write
 * @param   len[in] - Number of bytes to write
 * @param   intf_ptr[i] - not used here
 * @return	zero
 */
static int8_t spi_bme_write(uint8_t reg_addr,
	const uint8_t *reg_data,
	uint32_t len,
	void *intf_ptr) 
{
	SPI *dev = (SPI *)intf_ptr;
	
	slaveSelect_Pin = 0;
	
	dev->write((char *)&reg_addr, 1, (char *)&reg_addr, 1);
	
	dev->write((char *)reg_data, len, (char *)reg_data, len);
	
	slaveSelect_Pin = 1;

	return 0;          
}

/*!
 * @brief	Mbed specific microsecond delay function for bme688
 * @param	usin] - delay in microseconds
 * @return	none
 */
static void udelay(uint32_t us, void *intf_ptr) {
	wait_us(us);
}