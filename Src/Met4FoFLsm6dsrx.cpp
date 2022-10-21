/*
 * Met4FoFLsm6dsrx.cpp
 *
 *  Created on: 21.10.2022
 *      Author: seeger01
 */




#include "Met4FoFLsm6dsrx.h"

/* MPU9250 object, input the SPI bus and chip select pin */
Met4FoFLsm6dsrx::Met4FoFLsm6dsrx(GPIO_TypeDef* SPICSPort, uint16_t SPICSPin,SPI_HandleTypeDef* spiIfaceHandle,uint32_t BaseID){
	_SPICSPort=SPICSPort;
    _SPICSPin=SPICSPin;
    _spi=spiIfaceHandle;
	_BaseID=BaseID;
	_SetingsID=0;
	_ID=_BaseID+(uint32_t)_SetingsID;
	_dev_ctx.write_reg =(stmdev_write_ptr) &Met4FoFLsm6dsrx::platform_write;
	_dev_ctx.read_reg =(stmdev_read_ptr) &Met4FoFLsm6dsrx::platform_read;
	_dev_ctx.handle = &_spi;
}

int Met4FoFLsm6dsrx::setBaseID(uint32_t BaseID)
{
	_BaseID=BaseID;
	_SetingsID=0;
	_ID=_BaseID+(uint32_t)_SetingsID;
	return 0;
}

uint32_t Met4FoFLsm6dsrx::getSampleCount(){
	return _SampleCount;
}


int Met4FoFLsm6dsrx::setUp(){
	lsm6dsrx_device_id_get(&_dev_ctx, &_whoamI);
	if (_whoamI != LSM6DSRX_ID){
		return -1;
	}
	else
		return 0;
}



int Met4FoFLsm6dsrx::getData(DataMessage * Message,uint64_t RawTimeStamp){
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
	//TODO Add read sensor code
	//readresult=Met4FoFLsm6dsrx::readSensor();

	/*
	Message->Data_01=_ax;
	Message->has_Data_02=true;
	Message->Data_02=_ay;
	Message->has_Data_03=true;
	Message->Data_03=_az;
	Message->has_Data_04=true;
	Message->Data_04=_gx;
	Message->has_Data_05=true;
	Message->Data_05=_gy;
	Message->has_Data_06=true;
	Message->Data_06=_gz;
	Message->has_Data_07=true;
	Message->Data_07=_hx;
	*/
	return readresult;
}

int32_t Met4FoFLsm6dsrx::platform_write(void* handle, uint8_t reg, const uint8_t *bufp,uint16_t len)
{

  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, &reg, 1, 1000);
  HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_SET);
  return 0;
}

int32_t Met4FoFLsm6dsrx::platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
  reg |= 0x80;
  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, &reg, 1, 1000);
  HAL_SPI_Receive((SPI_HandleTypeDef*)handle, bufp, len, 1000);
  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_SET);
  return 0;
}



int Met4FoFLsm6dsrx::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	strncpy(Message->Sensor_name,"LSM6DSRX\0",sizeof(Message->Sensor_name));
	Message->id=_ID;
	Message->Description_Type=DESCRIPTION_TYPE;
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{

		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		Message->has_str_Data_04=true;
		Message->has_str_Data_05=true;
		Message->has_str_Data_06=true;
		Message->has_str_Data_07=true;
		strncpy(Message->str_Data_01,"X Acceleration\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Y Acceleration\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Z Acceleration\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"X Angular velocity\0",sizeof(Message->str_Data_04));
		strncpy(Message->str_Data_05,"Y Angular velocity\0",sizeof(Message->str_Data_05));
		strncpy(Message->str_Data_06,"Z Angular velocity\0",sizeof(Message->str_Data_06));
		strncpy(Message->str_Data_07,"Temperature\0",sizeof(Message->str_Data_10));

	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_UNIT)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		Message->has_str_Data_04=true;
		Message->has_str_Data_05=true;
		Message->has_str_Data_06=true;
		Message->has_str_Data_07=true;
		strncpy(Message->str_Data_01,"\\metre\\second\\tothe{-2}\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"\\metre\\second\\tothe{-2}\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"\\metre\\second\\tothe{-2}\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"\\radian\\second\\tothe{-1}\0",sizeof(Message->str_Data_04));
		strncpy(Message->str_Data_05,"\\radian\\second\\tothe{-1}\0",sizeof(Message->str_Data_05));
		strncpy(Message->str_Data_06,"\\radian\\second\\tothe{-1}\0",sizeof(Message->str_Data_06));
		strncpy(Message->str_Data_07,"\\degreecelsius\0",sizeof(Message->str_Data_10));
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION)
	{
		/*
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->has_f_Data_05=true;
		Message->has_f_Data_06=true;
		Message->has_f_Data_07=true;
		Message->f_Data_01=65536;
		Message->f_Data_02=65536;
		Message->f_Data_03=65536;
		Message->f_Data_04=65536;
		Message->f_Data_05=65536;
		Message->f_Data_06=65536;
		Message->f_Data_07=65520;
		*/
	}
	//TODO add min and max scale values as calls member vars so they have not to be calculated all the time
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{
		/*
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->has_f_Data_05=true;
		Message->has_f_Data_06=true;
		Message->has_f_Data_07=true;
		Message->f_Data_01=accMIN;
		Message->f_Data_02=accMIN;
		Message->f_Data_03=accMIN;
		Message->f_Data_04=gyrMIN;
		Message->f_Data_05=gyrMIN;
		Message->f_Data_06=gyrMIN;
		Message->f_Data_07=tempMIN;
		*/
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{
		/*
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->has_f_Data_05=true;
		Message->has_f_Data_06=true;
		Message->has_f_Data_07=true;
		Message->f_Data_01=accMAX;
		Message->f_Data_02=accMAX;
		Message->f_Data_03=accMAX;
		Message->f_Data_04=gyrMAX;
		Message->f_Data_05=gyrMAX;
		Message->f_Data_06=gyrMAX;
		Message->f_Data_7=tempMAX;
		*/
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		Message->has_str_Data_04=true;
		Message->has_str_Data_05=true;
		Message->has_str_Data_06=true;
		Message->has_str_Data_07=true;
		strncpy(Message->str_Data_01,"Acceleration/0\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Acceleration/1\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Acceleration/2\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"Angular_velocity/0\0",sizeof(Message->str_Data_04));
		strncpy(Message->str_Data_05,"Angular_velocity/1\0",sizeof(Message->str_Data_05));
		strncpy(Message->str_Data_06,"Angular_velocity/2\0",sizeof(Message->str_Data_06));
		strncpy(Message->str_Data_07,"Temperature/0\0",sizeof(Message->str_Data_10));
	}
	return retVal;
}
