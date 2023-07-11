/*
 * Met4FoFICM42866P.cpp
 *
 *  Created on: 21.10.2022
 *      Author: seeger01
 */




#include "Met4FoFICM42866P.h"


Met4FoFICM42866P::Met4FoFICM42866P(GPIO_TypeDef* SPICSPort, uint16_t SPICSPin,SPI_HandleTypeDef* spiIfaceHandle,uint32_t BaseID):
	Met4FoFSensor::Met4FoFSensor(BaseID){
	_SPICSPort=SPICSPort;
    _SPICSPin=SPICSPin;
    _spi=spiIfaceHandle;
    Met4FoFSensors::listMet4FoFSensors.push_back((Met4FoFSensors::Met4FoFSensor *)this);
}



int Met4FoFICM42866P::setODR(ICM426XX_GYRO_CONFIG0_ODR_t odr) {
	int ret= inv_icm426xx_set_gyro_frequency(&this->_Instance,odr);
	ret |= inv_icm426xx_set_accel_frequency(&this->_Instance,(ICM426XX_ACCEL_CONFIG0_ODR_t)odr);
	if (ret==0){
		switch(odr){
		case ICM426XX_GYRO_CONFIG0_ODR_32_KHZ:
			_nominalODR=32000;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_16_KHZ:
			_nominalODR=16000;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_8_KHZ:
			_nominalODR=320000;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_4_KHZ:
			_nominalODR=4000;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_2_KHZ:
			_nominalODR=2000;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_1_KHZ:
			_nominalODR=1000;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_500_HZ:
			_nominalODR=500;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_200_HZ:
			_nominalODR=200;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_100_HZ:
			_nominalODR=100;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_50_HZ:
			_nominalODR=200;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_25_HZ:
			_nominalODR=25;
			break;
		case ICM426XX_GYRO_CONFIG0_ODR_12_5_HZ:
			_nominalODR=12.5;
			break;
		}

	}
	return ret;

}

int Met4FoFICM42866P::setAccFS(ICM426XX_ACCEL_CONFIG0_FS_SEL_t accFullScale){
	switch(accFullScale){
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_2g:
		_ACCFSScaleFactor=16384.0/9.81;
		break;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_4g:
		_ACCFSScaleFactor=8192.0/9.81;
		break;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_8g:
		_ACCFSScaleFactor=4096.0/9.81;
		break;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_16g:
		_ACCFSScaleFactor=2048.0/9.81;
		break;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_32g:
		_ACCFSScaleFactor=4096.0/9.81;
		break;
	}
	int ret= inv_icm426xx_set_accel_fsr(&this->_Instance,accFullScale);
	if (ret==0){
		_ACCFullScaleCOnfig=accFullScale;
	}
	return ret;
}

int Met4FoFICM42866P::setGyroFS(ICM426XX_GYRO_CONFIG0_FS_SEL_t gyroFullScale){
	switch(gyroFullScale){
	case ICM426XX_GYRO_CONFIG0_FS_SEL_4000dps:
		_GyroFSScaleFactor=16.4;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps:
		_GyroFSScaleFactor=32.8;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps:
		_GyroFSScaleFactor=65.5;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_500dps:
		_GyroFSScaleFactor=131;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_250dps:
		_GyroFSScaleFactor=262;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_125dps:
		_GyroFSScaleFactor=524.3;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_62dps:
		_GyroFSScaleFactor=1048.6;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_31dps:
		_GyroFSScaleFactor=2097.2;
		break;
	}
	int ret= inv_icm426xx_set_gyro_fsr(&this->_Instance,gyroFullScale);
	if (ret==0){
		_GyroFullScaleCOnfig=gyroFullScale;
	}
	return ret;
	}


int Met4FoFICM42866P::setUp(){
	int ret=0;
	ret=inv_icm426xx_init(&_Instance,&_serif,(void (*)(inv_icm426xx_sensor_event_t*))&Met4FoFICM42866P::evntCB);
	setAccFS(_ACCFullScaleCOnfig);
	setGyroFS(_GyroFullScaleCOnfig);
	setODR(_ODRCOnfig);
	return ret;
}



int Met4FoFICM42866P::getData(DataMessage * Message,uint64_t RawTimeStamp){
	int result=-1;
	_SampleCount++;
	if (Message==0){
		return result;
	}
	int readresult=0;
	memcpy(Message,&empty_DataMessage,sizeof(DataMessage));//Copy default values into array
	Message->id=_ID;
	Message->unix_time=0XFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=	_SampleCount;

	return readresult;
}


int Met4FoFICM42866P::write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, const uint8_t *buf,uint32_t len)
{

  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_spi, &reg, 1, 1000);
  HAL_SPI_Transmit(_spi, (uint8_t*) buf, len, 1000);
  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_SET);
  return 0;
}

int Met4FoFICM42866P::read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
  reg |= 0x80;
  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_spi, &reg, 1, 1000);
  HAL_SPI_Receive(_spi,(uint8_t*) buf, len, 1000);
  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_SET);
  return 0;
}



int Met4FoFICM42866P::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	strncpy(Message->Sensor_name,"ICM42688P\0",sizeof(Message->Sensor_name));
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

		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->has_f_Data_05=true;
		Message->has_f_Data_06=true;
		Message->has_f_Data_07=true;
		Message->f_Data_01=65536;//All Channels 16 Bit
		Message->f_Data_02=65536;
		Message->f_Data_03=65536;
		Message->f_Data_04=65536;
		Message->f_Data_05=65536;
		Message->f_Data_06=65536;
		Message->f_Data_07=65536;

	}
	//TODO add min and max scale values as calls member vars so they have not to be calculated all the time
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{

		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->has_f_Data_05=true;
		Message->has_f_Data_06=true;
		Message->has_f_Data_07=true;
		Message->f_Data_01=-32768.0f*_ACCFSScaleFactor;
		Message->f_Data_02=-32768.0f*_ACCFSScaleFactor;
		Message->f_Data_03=-32768.0f*_ACCFSScaleFactor;
		Message->f_Data_04=-32768.0f*_GyroFSScaleFactor;
		Message->f_Data_05=-32768.0f*_GyroFSScaleFactor;
		Message->f_Data_06=-32768.0f*_GyroFSScaleFactor;
		Message->f_Data_07=-32768.0f/25.0f+25.0f;

	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{

		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->has_f_Data_04=true;
		Message->has_f_Data_05=true;
		Message->has_f_Data_06=true;
		Message->has_f_Data_07=true;
		Message->f_Data_01=32767.0f*_ACCFSScaleFactor;
		Message->f_Data_02=32767.0f*_ACCFSScaleFactor;
		Message->f_Data_03=32767.0f*_ACCFSScaleFactor;
		Message->f_Data_04=32767.0f*_GyroFSScaleFactor;
		Message->f_Data_05=32767.0f*_GyroFSScaleFactor;
		Message->f_Data_06=32767.0f*_GyroFSScaleFactor;
		Message->f_Data_07=32767.0f/25.0f+25.0f;

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
