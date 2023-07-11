/*
 *
 * https://github.com/bolderflight/ADXL355
ADXL355.cpp
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems
changed by B. Seeger to be usable at stm32 with HAl 2019

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ADXL355.h"

/* ADXL355 object, input the SPI bus and chip select pin */
ADXL355::ADXL355(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,SPI_HandleTypeDef* ADXL355spi,uint32_t BaseID):
	Met4FoFSensor::Met4FoFSensor(BaseID){
	_SPICSTypeDef=SPICSTypeDef;
    _SPICSPin=SPICSPin;
    _ADXL355spi=ADXL355spi;
	Met4FoFSensors::listMet4FoFSensors.push_back((Met4FoFSensors::Met4FoFSensor *)this);
}

ADXL355::~ADXL355()
{
	Met4FoFSensors::listMet4FoFSensors.remove((Met4FoFSensors::Met4FoFSensor *)this);
}

/* starts communication with the MPU-9250 */
int ADXL355::begin(){
	getIDs();
	setOpMode(ADXL355_STDBY_TEMP_OFF_DRDY_OFF);
	setRange(accRange);
	setLPFCorner(lpfSeting);
	setHPFCorner(hpfSeting);
	setOpMode(ADXL355_MEAS_TEMP_ON_DRDY_ON);
	  return 0;
	}




/* reads the most current data from ADXL355 and stores in buffer */
int ADXL355::readSensor() {

  //readStatus();
  // grab the data from the ADXL355
  if (readRegisters(TEMP2, 11, buffer) < 0) {
    return 0;
  }
  // combine into 16 bit values
  tempRaw =  (((uint16_t)buffer[0]&0x0F) << 8 ) |  (uint16_t)buffer[1];
  accRawX =  (((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 8) | (uint32_t)buffer[4])>>4;
  accRawY =  (((uint32_t)buffer[5] << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7])>>4;
  accRawZ =  (((uint32_t)buffer[8] << 16) | ((uint32_t)buffer[9] << 8) | (uint32_t)buffer[10])>>4;
  // transform and convert to float values
  accX=convertACCReading(accRawX);
  accY=convertACCReading(accRawY);
  accZ=convertACCReading(accRawZ);
  temp=convertTempReading(tempRaw);
  return 1;
}

int ADXL355::getIDs(){
	uint8_t buffer[4]={0};
	int ret=readRegisters(DEVID_AD,4,buffer);
    vendorID=buffer[0];
    memsID=buffer[1];
    deviceID=buffer[2];
    revisionID=buffer[3];
	return ret;
}

int ADXL355::readStatus(){
	int ret=readRegisters(STATUS,1,&status.value);
	return ret;
}


float ADXL355::convertACCReading(uint32_t reading){
	int32_t accelData=0;
	if ((reading & 0x00080000) == 0x00080000)
		accelData = reading | ADXL355_NEG_ACC_MSK;
	else
		accelData = reading;

	return float(accelData)*accScaleFactor;
}

float ADXL355::convertTempReading(uint16_t reading){
	int16_t tempData=0;
	tempData=reading-1825;
	return (float)tempData/-9.05+25.0;
}

/* writes a byte to ADXL355 register given a register address and data */
int ADXL355::writeRegister(uint8_t address, uint8_t data){
  /* write data to device */
  uint8_t buffer[2] = {(address<<1), data };
  HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_ADXL355spi, buffer, 2, SPI_TIMEOUT);
  HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
  /* read back the register */
  readRegisters(address,1,buffer);
  /* check the read back register against the written register */
  if(buffer[0] == data) {
    return 0;
  }
  else{
    return -1;
  }
}

/* reads registers from ADXL355 given a starting register address, number of bytes, and a pointer to store data */
int ADXL355::readRegisters(uint8_t Address, uint8_t count, uint8_t* dest){
    // begin the transaction
  	int retVal=-1;
  	uint8_t tx[count+1]={0};
  	uint8_t rx[count+1]={0};
  	tx[0] = (Address<<1)|0x01;
  	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
  	if(HAL_SPI_TransmitReceive(_ADXL355spi,tx, rx, count+1, SPI_TIMEOUT)==HAL_OK)
  	{
  		retVal=0;
  	}
  	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
  	memcpy(dest, &rx[1], count);
  return retVal;
}

int ADXL355::setOpMode(adxl355_op_mode opMode){
	uint8_t mode=0;
	readRegisters(POWER_CTL,1,&mode);
	int ret=-1;
	uint8_t powerCTLReg;
	while (ret==-1){
	ret = writeRegister(POWER_CTL, opMode);
	}
	return ret;
}

int ADXL355::setRange(adxl355_range range){
	accScaleFactor=0.00003824593*range;
	accRange=range;
	int ret;
	uint8_t rangeReg;
	uint8_t rangeRegOld;
	ret = readRegisters(RANGE,1, &rangeReg);
	rangeRegOld=rangeReg;
	rangeReg &= ~ADXL355_RANGE_FIELD_MSK;
	rangeReg |= rangeRegOld &ADXL355_RANGE_FIELD_MSK;
	ret=writeRegister(RANGE,rangeReg);
	return ret;
}


int ADXL355::setHPFCorner(adxl355_hpf_corner hpfCorner){
int ret;
uint8_t regValue;
enum adxl355_op_mode oldMode=opMode;
switch(oldMode) {
case ADXL355_MEAS_TEMP_ON_DRDY_ON:
case ADXL355_MEAS_TEMP_OFF_DRDY_ON:
case ADXL355_MEAS_TEMP_ON_DRDY_OFF:
case ADXL355_MEAS_TEMP_OFF_DRDY_OFF:
	ret = setOpMode(ADXL355_STDBY_TEMP_ON_DRDY_ON);
	if (ret)
		return ret;
	break;
default:
	break;
}
ret = readRegisters(FILTER,1,&regValue);
if (ret){
	return ret;
}
regValue &= ~(ADXL355_HPF_FIELD_MSK);
regValue |= (hpfCorner<<4) & ADXL355_HPF_FIELD_MSK;
writeRegister(FILTER,regValue);
if (ret)
	return ret;
hpfSeting=hpfCorner;
return setOpMode(oldMode);
}


int ADXL355::setLPFCorner(adxl355_odr_lpf lpfFreq){
	int ret;
	uint8_t regValue;
	enum adxl355_op_mode oldMode=opMode;
	switch(oldMode) {
	case ADXL355_MEAS_TEMP_ON_DRDY_ON:
	case ADXL355_MEAS_TEMP_OFF_DRDY_ON:
	case ADXL355_MEAS_TEMP_ON_DRDY_OFF:
	case ADXL355_MEAS_TEMP_OFF_DRDY_OFF:
		ret = setOpMode(ADXL355_STDBY_TEMP_ON_DRDY_ON);
		if (ret)
			return ret;
		break;
	default:
		break;
	}
	ret = readRegisters(FILTER,1,&regValue);
	if (ret){
		return ret;
	}
	regValue &= ~(ADXL355_ODR_LPF_FIELD_MSK);
	regValue |= lpfFreq & ADXL355_ODR_LPF_FIELD_MSK;
	writeRegister(FILTER,regValue);
	if (ret)
		return ret;
	lpfSeting=lpfFreq;
	return setOpMode(oldMode);
}


int ADXL355::getData(DataMessage * Message,uint64_t RawTimeStamp){
	int result=0;
	_SampleCount++;
	if (Message==0){
		return result;
	}
	int readresult=-1;
	memcpy(Message,&empty_DataMessage,sizeof(DataMessage));//Copy default values into array
	Message->id=_ID;
	Message->unix_time=0XFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=	_SampleCount;
	readresult=ADXL355::readSensor();
	Message->Data_01=accX;
	Message->has_Data_02=true;
	Message->Data_02=accY;
	Message->has_Data_03=true;
	Message->Data_03=accZ;
	Message->has_Data_04=true;
	Message->Data_04=temp;
	return readresult;
}

int ADXL355::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	strncpy(Message->Sensor_name,"ADXL_355\0",sizeof(Message->Sensor_name));
	Message->id=_ID;
	Message->Description_Type=DESCRIPTION_TYPE;
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_01,"X Acceleration\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Y Acceleration\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Z Acceleration\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"Temperature\0",sizeof(Message->str_Data_04));
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_UNIT)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_01,"\\metre\\second\\tothe{-2}\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"\\metre\\second\\tothe{-2}\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"\\metre\\second\\tothe{-2}\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"\\degreecelsius\0",sizeof(Message->str_Data_04));
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->f_Data_01=1048576;
		Message->f_Data_02=1048576;
		Message->f_Data_03=1048576;
		Message->f_Data_04=4096;
	}
	//TODO add min and max scale values as calls member vars so they have not to be calculated all the time
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{

		float accMIN=convertACCReading(-524288);
		float tempMIN = convertTempReading(4096);
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->f_Data_01=accMIN;
		Message->f_Data_02=accMIN;
		Message->f_Data_03=accMIN;
		Message->f_Data_04=tempMIN;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{
		float accMAX=convertACCReading(524287);
		float tempMAX=convertTempReading(0);
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->f_Data_01=accMAX;
		Message->f_Data_02=accMAX;
		Message->f_Data_03=accMAX;
		Message->f_Data_04=tempMAX;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_01,"Acceleration/0\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Acceleration/1\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Acceleration/2\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"Temperature/0\0",sizeof(Message->str_Data_04));
	}
	return retVal;
}
