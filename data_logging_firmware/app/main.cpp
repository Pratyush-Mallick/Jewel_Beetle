/**
 * Jewel Bettle: Data Logging firmware
 * 
 * Author: Pratyush Mallick
 * Date: Augutst 15, 2021
 * 
 * Pin Map Used: 
 * MCU (EV-COG-4050LZ) <--> Peripheral
 * SPI1_CS0 (P1_09) <-----> CS (BME688)
 * SPI1_MISO (P1_08) <-----> SDO (BME688)
 * SPI1_MOSI (P1_07) <-----> SDI (BME688)
 * SPI1_SCLK (P1_06) <-----> SCK (BME688)
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
#include "bme_688/bme68x.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"
#include <errno.h>
#include <cstdio>
#include <cstdlib>
#include <stdint.h>

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/
#define MAX_SPI 1000000
#define PIN_SPI_SS SPI1_CS0
#define timer_read_f(x)     chrono::duration<float>((x).elapsed_time()).count()
#define timer_read_s(x)     chrono::duration_cast<chrono::seconds>((x).elapsed_time()).count();
#define timer_read_ms(x)    chrono::duration_cast<chrono::milliseconds>((x).elapsed_time()).count()

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/
Timer t;
InterruptIn button_1(BUTTON1);  // Data logging start button 
InterruptIn button_2(BUTTON2);  // Data logging stop button
DigitalOut pc_activity(LED1);
DigitalOut slaveSelect_Pin(PIN_SPI_SS);
SPI spi_dev(SPI1_MOSI, SPI1_MISO, SPI1_SCLK);

// Instantiate the SDBlockDevice by specifying the SPI pins connected to the SDCard
// socket. The PINS are:
//     MOSI (Master Out Slave In)
//     MISO (Master In Slave Out)
//     SCLK (Serial Clock)
//     CS (Chip Select)
SDBlockDevice blockDevice(SPI0_MOSI, SPI0_MISO, SPI0_SCLK, SPI0_CS2, 4000000);
FATFileSystem fileSystem("fs");
typedef struct 
{	
	FILE* file_p;
}file_handle;

static bool spi_dev_init(void);
static bool bme_init(void);
static int8_t spi_bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);
static int8_t spi_bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr); 
void start_data_logging(void);
void stop_data_logging(void);
void open_text_file(file_handle* file_open);
void close_text_file(file_handle* file_close);
static void udelay(uint32_t us, void *intf_ptr);

struct bme68x_dev bme_dev;
struct bme68x_conf gas_conf;
struct bme68x_heatr_conf gas_heatr_conf;
struct bme68x_data data;
uint8_t n_fields;

static int64_t start_time;
volatile bool start_logging_flag = false; 
volatile bool open_file_flag = false;
volatile bool close_file_flag = false;
static uint64_t time_elapse = 0;
char file_name[30];
char block[50];
int err;

// main() runs in its own thread in the OS
int main()
{
	int8_t rslt;
	file_handle *file_ptr = (file_handle*)malloc(sizeof(file_ptr));
	
	pc_activity = 0;
    if(!spi_dev_init()) {
        printf("\r\nSPI Error");
    }

    if(!bme_init()) {
        printf("\r\nSPI Error");
    }
	
	button_1.rise(&start_data_logging);  // attach the address of the flip function to the rising edge
	button_2.rise(&stop_data_logging);
	
	// Try to mount the filesystem
	printf("Mounting the filesystem... ");
	fflush(stdout);
	err = fileSystem.mount(&blockDevice);
	wait_us(10);
	printf("%s\n", (err ? "Fail :(" : "OK"));
	if (err) {
		// Reformat if we can't mount the filesystem
		// this should only happen on the first boot
		printf("No filesystem found, formatting... ");
		fflush(stdout);
		err = fileSystem.reformat(&blockDevice);
		printf("%s\n", (err ? "Fail :(" : "OK"));
		if (err) {
			error("error: %s (%d)\n", strerror(-err), err);
		}
	}

	pc_activity = 1;
	while (true) {
		if (start_logging_flag) {
			if (open_file_flag) {
				open_text_file(file_ptr);
			}
			memset(block, 0, sizeof(block));
			rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme_dev);
			uint32_t delayus_period = (uint32_t)bme68x_get_meas_dur(
				BME68X_FORCED_MODE,
				&gas_conf,
				&bme_dev) + 
				((uint32_t)gas_heatr_conf.heatr_dur * 1000);
	  
			bme_dev.delay_us(delayus_period, bme_dev.intf_ptr);
			//wait_us(delayus_period);
			bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme_dev);
			time_elapse = time_elapse + (delayus_period / 1000);
			sprintf(block, "%llu,%0.2f,%0.2f,%0.2f", time_elapse, data.humidity, data.temperature, data.gas_resistance);
			fflush(stdout);
			err = fprintf(file_ptr->file_p , "\r\n%s", block);
			printf("%s\n", (err < 0 ? "Fail writing data to file:(" : "OK"));
		}
		else {
			if (close_file_flag) {
				close_text_file(file_ptr);
			}
		}
	}
}

static bool spi_dev_init(void)
{
	spi_dev.format(8, 0);
	spi_dev.frequency(1000000);
	spi_dev.set_default_write_value(0x00);
	
    return true;
}

static bool bme_init()
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
        while(1);
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

static int8_t spi_bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
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

static int8_t spi_bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr) 
{
	SPI *dev = (SPI *)intf_ptr;
	
	slaveSelect_Pin = 0;
	
	dev->write((char *)&reg_addr, 1, (char *)&reg_addr, 1);
	
	dev->write((char *)reg_data, len, (char *)reg_data, len);
	
	slaveSelect_Pin = 1;

  return 0;          
}

void start_data_logging(void)
{
	if (start_logging_flag == false)
	{
		start_logging_flag = true;
		open_file_flag = true;
		time_elapse = 0;
	}
}

void stop_data_logging(void)
{
	if (start_logging_flag == true)
	{
		start_logging_flag = false;
		close_file_flag = true;
	}
}

void open_text_file(file_handle* file_open)
{
	t.start();
	auto start_time = timer_read_s(t)
	sprintf(file_name, "/fs/test_%llu.txt", start_time);
	// initialize to open function, ony declaring won't help
	file_open->file_p= fopen(file_name, "r+");
	printf("%s\n", (!file_open->file_p ? "Fail :(" : "OK"));
	if (!file_open->file_p) {
		// Create the numbers file if it doesn't exist
		printf("No file found, creating a new file... ");
		fflush(stdout);
		file_open->file_p = fopen(file_name, "w+");
		printf("%s\n", (!file_open->file_p ? "Fail :(" : "OK"));
	}
		
	printf("\r\n Writing headers to csv.txt");
	// Write txt headers to file:
	if(file_open->file_p) // it opened OK
	{
		err = fprintf(file_open->file_p, "\r\nTemperature,Humidity,Gas");
		printf("%s\n", (err < 0 ? "Fail :(" : "OK"));
		pc_activity = 0;
	}
	else {
		printf("\r\n Couldn't create the file");
	}
	open_file_flag = false;
}

void close_text_file(file_handle* file_close)
{
	// Close the file which also flushes any cached writes
	t.stop();
	printf("Closing %s", file_name);
	fflush(stdout);
	err = fclose(file_close->file_p);
	printf("%s\n", (err < 0 ? "Fail :(" : "OK"));
	printf("\r\nPress Button 1 to Start");
	pc_activity = 1;
	close_file_flag = false;
}

static void udelay(uint32_t us, void *intf_ptr) {
  wait_us(us);
}