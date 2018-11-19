/**
 * @author Aaron Berk
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of uint8_tge, to any person obtaining a copy
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * ADXL345, triple axis, digital interface, accelerometer.
 *
 * Datasheet:
 *
 * http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf
 */

/**
 * Includes
 */
#include <stdint.h>
#include <cstring>
#include <math.h>
#include "spi.h"
#include "gpio.h"
#include "stm32f7xx_hal.h"
#include "ADXL345.h"

ADXL345::ADXL345(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,
		SPI_HandleTypeDef* bmaspi) :
		_SPICSTypeDef(SPICSTypeDef), _SPICSPin(SPICSPin), _bmaspi(bmaspi) {
	;

}

int ADXL345::getDevId(void) {

	return oneByteRead(ADXL345_DEVID_REG);

}

int ADXL345::getTapThreshold(void) {

	return oneByteRead(ADXL345_THRESH_TAP_REG);

}

void ADXL345::setTapThreshold(int threshold) {

	oneByteWrite(ADXL345_THRESH_TAP_REG, threshold);

}

int ADXL345::getOffset(int axis) {

	int address = 0;

	if (axis == ADXL345_X) {
		address = ADXL345_OFSX_REG;
	} else if (axis == ADXL345_Y) {
		address = ADXL345_OFSY_REG;
	} else if (axis == ADXL345_Z) {
		address = ADXL345_OFSZ_REG;
	}

	return oneByteRead(address);

}

void ADXL345::setOffset(int axis, uint8_t offset) {

	int address = 0;

	if (axis == ADXL345_X) {
		address = ADXL345_OFSX_REG;
	} else if (axis == ADXL345_Y) {
		address = ADXL345_OFSY_REG;
	} else if (axis == ADXL345_Z) {
		address = ADXL345_OFSZ_REG;
	}
	oneByteWrite(address, offset);
	return;

}

int ADXL345::getTapDuration(void) {

	return oneByteRead(ADXL345_DUR_REG) * 625;

}

void ADXL345::setTapDuration(int duration_us) {

	int tapDuration = duration_us / 625;

	oneByteWrite(ADXL345_DUR_REG, tapDuration);

}

float ADXL345::getTapLatency(void) {

	return oneByteRead(ADXL345_LATENT_REG) * 1.25;

}

void ADXL345::setTapLatency(int latency_ms) {

	int tapLatency = latency_ms / 1.25;

	oneByteWrite(ADXL345_LATENT_REG, tapLatency);

}

float ADXL345::getWindowTime(void) {

	return oneByteRead(ADXL345_WINDOW_REG) * 1.25;

}

void ADXL345::setWindowTime(int window_ms) {

	int windowTime = window_ms / 1.25;

	oneByteWrite(ADXL345_WINDOW_REG, windowTime);

}

int ADXL345::getActivityThreshold(void) {

	return oneByteRead(ADXL345_THRESH_ACT_REG);

}

void ADXL345::setActivityThreshold(int threshold) {

	oneByteWrite(ADXL345_THRESH_ACT_REG, threshold);

}

int ADXL345::getInactivityThreshold(void) {

	return oneByteRead(ADXL345_THRESH_INACT_REG);

}

void ADXL345::setInactivityThreshold(int threshold) {

	oneByteWrite(ADXL345_THRESH_INACT_REG, threshold);
	return;

}

int ADXL345::getTimeInactivity(void) {

	return oneByteRead(ADXL345_TIME_INACT_REG);

}

void ADXL345::setTimeInactivity(int timeInactivity) {

	oneByteWrite(ADXL345_TIME_INACT_REG, timeInactivity);

}

int ADXL345::getActivityInactivityControl(void) {

	return oneByteRead(ADXL345_ACT_INACT_CTL_REG);

}

void ADXL345::setActivityInactivityControl(int settings) {

	oneByteWrite(ADXL345_ACT_INACT_CTL_REG, settings);

}

int ADXL345::getFreefallThreshold(void) {

	return oneByteRead(ADXL345_THRESH_FF_REG);

}

void ADXL345::setFreefallThreshold(int threshold) {

	oneByteWrite(ADXL345_THRESH_FF_REG, threshold);

}

int ADXL345::getFreefallTime(void) {

	return oneByteRead(ADXL345_TIME_FF_REG) * 5;

}

void ADXL345::setFreefallTime(int freefallTime_ms) {

	int freefallTime = freefallTime_ms / 5;

	oneByteWrite(ADXL345_TIME_FF_REG, freefallTime);

}

int ADXL345::getTapAxisControl(void) {

	return oneByteRead(ADXL345_TAP_AXES_REG);

}

void ADXL345::setTapAxisControl(int settings) {

	oneByteWrite(ADXL345_TAP_AXES_REG, settings);

}

int ADXL345::getTapSource(void) {

	return oneByteRead(ADXL345_ACT_TAP_STATUS_REG);

}

void ADXL345::setPowerMode(uint8_t mode) {

	//Get the current register contents, so we don't clobber the rate value.
	uint8_t registerContents = oneByteRead(ADXL345_BW_RATE_REG);

	registerContents = (mode << 4) | registerContents;

	oneByteWrite(ADXL345_BW_RATE_REG, registerContents);

}

int ADXL345::getPowerControl(void) {

	return oneByteRead(ADXL345_POWER_CTL_REG);

}

void ADXL345::setPowerControl(int settings) {

	oneByteWrite(ADXL345_POWER_CTL_REG, settings);

}

int ADXL345::getInterruptEnableControl(void) {

	return oneByteRead(ADXL345_INT_ENABLE_REG);

}

void ADXL345::setInterruptEnableControl(int settings) {

	oneByteWrite(ADXL345_INT_ENABLE_REG, settings);

}

int ADXL345::getInterruptMappingControl(void) {

	return oneByteRead(ADXL345_INT_MAP_REG);

}

void ADXL345::setInterruptMappingControl(int settings) {

	oneByteWrite(ADXL345_INT_MAP_REG, settings);

}

int ADXL345::getInterruptSource(void) {

	return oneByteRead(ADXL345_INT_SOURCE_REG);

}

int ADXL345::getDataFormatControl(void) {

	return oneByteRead(ADXL345_DATA_FORMAT_REG);

}

void ADXL345::setDataFormatControl(int settings) {

	oneByteWrite(ADXL345_DATA_FORMAT_REG, settings);

}

bool ADXL345::setResolution(int settings){
	bool retVal=false;
	if(settings<=0x04||ADXL345_AFS_FULL_RANGE){
		uint8_t oldState=oneByteRead(ADXL345_DATA_FORMAT_REG);
		retVal=oneByteWrite(ADXL345_DATA_FORMAT_REG, ((oldState&0xF4)|settings));
	}
	if(retVal==true){
	if(settings==ADXL345_AFS_2G)_ares=ADXL345_AFS_2G_CONVERSIONFACTOR;
	if(settings==ADXL345_AFS_4G)_ares=ADXL345_AFS_4G_CONVERSIONFACTOR;
	if(settings==ADXL345_AFS_8G)_ares=ADXL345_AFS_8G_CONVERSIONFACTOR;
	if(settings==ADXL345_AFS_16G)_ares=ADXL345_AFS_16G_CONVERSIONFACTOR;
	if(settings==ADXL345_AFS_FULL_RANGE)_ares=ADXL345_AFS_FULL_RANGE_CONVERSIONFACTOR;
	}
	return retVal;
}

void ADXL345::setDataRate(int rate) {

	//Get the current register contents, so we don't clobber the power bit.
	uint8_t registerContents = oneByteRead(ADXL345_BW_RATE_REG);

	registerContents &= 0x10;
	registerContents |= rate;

	oneByteWrite(ADXL345_BW_RATE_REG, registerContents);

}

void ADXL345::getOutput(int16_t* readings) {

	uint8_t buffer[6];
	if(multiByteRead(ADXL345_DATAX0_REG, buffer, 6)==true){
	readings[0] = (int16_t) (buffer[0] << 8 | (int8_t) buffer[1]);
	readings[1] = (int16_t) (buffer[2] << 8 | (int8_t) buffer[3]);
	readings[2] = (int16_t) (buffer[4] << 8 | (int8_t) buffer[5]);
	//memcpy(&readings,&temp,sizeof(temp));
	}
	else
	{
		readings[0]=0;
		readings[1]=0;
		readings[2]=0;
	}
}

AccelData ADXL345::GetData(){
	AccelData retVal;
	int16_t readings[3];
	getOutput(readings);
	retVal.x=readings[0]*_ares;
	retVal.y=readings[1]*_ares;
	retVal.z=readings[2]*_ares;
	retVal.temperature=NAN;
	return retVal;
}

AccelDataStamped ADXL345::GetStampedData(uint32_t UnixSecs,uint32_t RawTimerCount,uint32_t CaptureCount){
	AccelDataStamped returnVal{0,0,0,0};
	returnVal.Data=ADXL345::GetData();
	returnVal.UnixSecs=UnixSecs;
	returnVal.RawTimerCount=RawTimerCount;
	returnVal.CaptureCount=CaptureCount;
	return returnVal;

}

int ADXL345::getFifoControl(void) {

	return oneByteRead(ADXL345_FIFO_CTL);

}

void ADXL345::setFifoControl(int settings) {

	oneByteWrite(ADXL345_FIFO_STATUS, settings);

}

int ADXL345::getFifoStatus(void) {

	return oneByteRead(ADXL345_FIFO_STATUS);

}

int ADXL345::oneByteRead(int address) {

	uint8_t tx[2] = { (ADXL345_SPI_READ | (address & 0x3F)), 0 };
	uint8_t rx[2] = { 0, 0 };
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(_bmaspi, tx, rx, 2, SPI_TIMEOUT);
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
	return rx[2];

}

bool ADXL345::oneByteWrite(int address, uint8_t data) {
	bool retVal = false;
	uint8_t tx[2] = { (ADXL345_SPI_WRITE | (address & 0x3F)), (uint8_t) data };
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(_bmaspi, tx, 2, SPI_TIMEOUT) == HAL_OK) {
		retVal = true;
	}
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
	return retVal;

}

bool ADXL345::multiByteRead(int startAddress, uint8_t* buffer, int size) {
	bool retVal = false;
	uint8_t tx[size + 1] = { 0 };
	uint8_t rx[size + 1] = { 0 };
	tx[0] = (ADXL345_SPI_READ | ADXL345_MULTI_BYTE | (startAddress & 0x3F));
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive(_bmaspi, tx, rx, size + 1, SPI_TIMEOUT)
			== HAL_OK) {
		retVal = true;
	}
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
	memcpy(buffer, &rx[1], size);
	return retVal;

}

bool ADXL345::multiByteWrite(int startAddress, uint8_t* buffer, int size) {
	bool retVal = false;
	uint8_t tx[size + 1] = { 0 };
	uint8_t rx[size + 1] = { 0 };
	tx[0] = (ADXL345_SPI_READ | ADXL345_MULTI_BYTE | (startAddress & 0x3F));
	memcpy(buffer, &tx[1], size);
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive(_bmaspi, tx, rx, size + 1, SPI_TIMEOUT)
			== HAL_OK) {
		retVal = true;
	}
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
	return retVal;
}
