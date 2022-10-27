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
	/*
	_dev_ctx.write_reg =(stmdev_write_ptr) &Met4FoFLsm6dsrx::platform_write;
	_dev_ctx.read_reg =(stmdev_read_ptr) &Met4FoFLsm6dsrx::platform_read;
	_dev_ctx.handle = this;
	*/
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

int Met4FoFLsm6dsrx::setODR(Met4FoFLsm6dsrx::outPutDatarate odr) {

	int32_t retSetAccODR=lsm6dsrx_xl_data_rate_set(&_dev_ctx,(lsm6dsrx_odr_xl_t) odr);//ACC
	int32_t retSetGyroODR=lsm6dsrx_gy_data_rate_set(&_dev_ctx,(lsm6dsrx_odr_g_t) odr);//Gyro
	if(retSetAccODR==0 and  retSetGyroODR==0)
	{

	  switch(odr){
	  case ODR_OFF:
		  _NominalSamplingFreq=0.0f;
		  break;
	  case ODR_12Hz5:
		  _NominalSamplingFreq=12.5f;
		  break;
	  case ODR_26Hz:
		  _NominalSamplingFreq=26.0f;
		  break;
	  case ODR_52Hz:
		  _NominalSamplingFreq=52.0f;
		  break;
	  case ODR_104Hz:
		  _NominalSamplingFreq=104.0f;
		  break;
	  case ODR_208Hz:
		  _NominalSamplingFreq=208.0f;
		  break;
	  case ODR_416Hz:
		  _NominalSamplingFreq=416.0f;
		  break;
	  case ODR_833Hz:
		  _NominalSamplingFreq=833.0f;
		  break;
	  case ODR_1666Hz:
		  _NominalSamplingFreq=1666.0f;
		  break;
	  case ODR_332Hz:
		  _NominalSamplingFreq=332.0f;
		  break;
	  case ODR_6667Hz:
		  _NominalSamplingFreq=6667.0f;
		  break;
	  default:
		  _NominalSamplingFreq=NAN;
		  break;
	  }
	  return 0;
	}
	else
	{
		return retSetAccODR+retSetGyroODR*256;
	}
}

int Met4FoFLsm6dsrx::setAccFS(lsm6dsrx_fs_xl_t accFullScale){
	 int ret=lsm6dsrx_xl_full_scale_set(&_dev_ctx, accFullScale);//Acc
	 if (ret==0)
	 {
		  _accFullScaleSet=accFullScale;
		  switch(accFullScale){
		  case LSM6DSRX_2g:
			  _ACCFSScaleFactor=0.061f/1000.0*9.81;
			  break;
		  case LSM6DSRX_4g:
			  _ACCFSScaleFactor=0.122f/1000.0*9.81;
			  break;
		  case LSM6DSRX_8g:
			  _ACCFSScaleFactor=0.244f/1000.0*9.81;
			  break;
		  case LSM6DSRX_16g:
			  _ACCFSScaleFactor=0.488f/1000.0*9.81;
			  break;
		  default:
			  _ACCFSScaleFactor=NAN;
			  break;
		  }
	 }
return ret;
}

int Met4FoFLsm6dsrx::setGyroFS(lsm6dsrx_fs_g_t gyroFullScale){
		 int ret=lsm6dsrx_gy_full_scale_set(&_dev_ctx, gyroFullScale);//Acc
		 if (ret==0)
		 {
			  _gyroFullScaleSet=gyroFullScale;
			  switch(gyroFullScale){
			  case LSM6DSRX_125dps:
				  _GyroFSScaleFactor=4.375f/180000.0*M_PI;
				  break;
			  case LSM6DSRX_250dps:
				  _GyroFSScaleFactor=8.75f/180000.0*M_PI;
				  break;
			  case LSM6DSRX_500dps:
				  _GyroFSScaleFactor=17.50f/180000.0*M_PI;
				  break;
			  case LSM6DSRX_1000dps:
				  _GyroFSScaleFactor=35.0f/180000.0*M_PI;
				  break;
			  case LSM6DSRX_2000dps:
				  _GyroFSScaleFactor=70.0f/180000.0*M_PI;
				  break;
			  case LSM6DSRX_4000dps:
				  _GyroFSScaleFactor=140.0f/180000.0*M_PI;
				  break;
			  default:
				  _GyroFSScaleFactor=NAN;
				  break;
			  }
		 }
	return ret;
	}


int Met4FoFLsm6dsrx::setUp(){
	bool connectionSuccsesfull=false;
	unsigned int retrysSoFar=0;
	while (connectionSuccsesfull==false and retrysSoFar<_connectionInitRetys){
		lsm6dsrx_device_id_get(&_dev_ctx, &_whoamI);
		if (_whoamI == LSM6DSRX_ID){
			connectionSuccsesfull=true;
		}
		else
		{
			osDelay(_connectionInitDelayMs);
		}
		retrysSoFar++;
	}
	if (connectionSuccsesfull==false)
	{
		return -1;
	}
	  /* Restore default configuration. */
	  lsm6dsrx_reset_set(&_dev_ctx, PROPERTY_ENABLE);
	  do {
	    lsm6dsrx_reset_get(&_dev_ctx, &_rst);
	  } while (_rst);

	  /* Disable I3C interface. */
	  lsm6dsrx_i3c_disable_set(&_dev_ctx, LSM6DSRX_I3C_DISABLE);
	  /* Enable Block Data Update. */
	  /* Enable Block Data Update. */
	  lsm6dsrx_block_data_update_set(&_dev_ctx, PROPERTY_ENABLE);
	  /* Set Output Data Rate. */
	  setODR(_odr);
	  setAccFS(_accFullScaleSet);
	  setGyroFS(_gyroFullScaleSet);
	  /* Set full scale. */

	  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed. */
	  lsm6dsrx_data_ready_mode_set(&_dev_ctx, LSM6DSRX_DRDY_PULSED);

	  lsm6dsrx_pin_int1_route_get(&_dev_ctx, &_int1_route);
	  //TODO Debug this registers with actual sensors
	  _int1_route.int1_ctrl.den_drdy_flag = PROPERTY_ENABLE;
	  _int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
	  _int1_route.int1_ctrl.int1_drdy_g = PROPERTY_ENABLE;
	  lsm6dsrx_pin_int1_route_set(&_dev_ctx, &_int1_route);
	  lsm6dsrx_pin_int1_route_get(&_dev_ctx, &_int1_route);

	  lsm6dsrx_pin_int2_route_get(&_dev_ctx, &_int2_route);
	  //TODO Debug this registers with actual sensors
	  _int2_route.int2_ctrl.int2_drdy_xl = PROPERTY_ENABLE;
	  _int2_route.int2_ctrl.int2_drdy_g = PROPERTY_ENABLE;
	  lsm6dsrx_pin_int2_route_set(&_dev_ctx, &_int2_route);
	  lsm6dsrx_pin_int2_route_get(&_dev_ctx, &_int2_route);
	  dummyRead();
	  return 0;

}



int Met4FoFLsm6dsrx::getData(DataMessage * Message,uint64_t RawTimeStamp){
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

	int16_t data_raw_acceleration[3]={0};
	int16_t data_raw_angular_rate[3]={0};
	int16_t data_raw_temp=0;

	lsm6dsrx_acceleration_raw_get(&_dev_ctx, data_raw_acceleration);
	lsm6dsrx_angular_rate_raw_get(&_dev_ctx, data_raw_angular_rate);
	lsm6dsrx_temperature_raw_get(&_dev_ctx, &data_raw_temp);
	Message->Data_01=(float)data_raw_acceleration[0]*_ACCFSScaleFactor;
	Message->has_Data_02=true;
	Message->Data_02=(float)data_raw_acceleration[1]*_ACCFSScaleFactor;
	Message->has_Data_03=true;
	Message->Data_03=(float)data_raw_acceleration[2]*_ACCFSScaleFactor;
	Message->has_Data_04=true;
	Message->Data_04=(float)data_raw_angular_rate[0]*_GyroFSScaleFactor;
	Message->has_Data_05=true;
	Message->Data_05=(float)data_raw_angular_rate[1]*_GyroFSScaleFactor;
	Message->has_Data_06=true;
	Message->Data_06=(float)data_raw_angular_rate[2]*_GyroFSScaleFactor;
	Message->has_Data_07=true;
	Message->Data_07=(float)data_raw_temp*_TempFSScaleFactor+_TempScaleOffset;
	return readresult;
}

void Met4FoFLsm6dsrx::dummyRead(){
	int16_t data_raw_acceleration[3]={0};
	int16_t data_raw_angular_rate[3]={0};
	int16_t data_raw_temp=0;

	lsm6dsrx_acceleration_raw_get(&_dev_ctx, data_raw_acceleration);
	lsm6dsrx_angular_rate_raw_get(&_dev_ctx, data_raw_angular_rate);
	lsm6dsrx_temperature_raw_get(&_dev_ctx, &data_raw_temp);
}
int32_t Met4FoFLsm6dsrx::platform_write(uint8_t reg, const uint8_t *bufp,uint16_t len)
{

  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_spi, &reg, 1, 1000);
  HAL_SPI_Transmit(_spi, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_SET);
  return 0;
}

int32_t Met4FoFLsm6dsrx::platform_read(uint8_t reg, uint8_t *bufp,uint16_t len)
{
  reg |= 0x80;
  HAL_GPIO_WritePin(_SPICSPort,  _SPICSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_spi, &reg, 1, 1000);
  HAL_SPI_Receive(_spi,(uint8_t*) bufp, len, 1000);
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
