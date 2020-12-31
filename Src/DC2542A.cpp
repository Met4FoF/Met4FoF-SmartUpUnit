#include "DC2542A.h"

/* MPU9250 object, input the SPI bus and chip select pin */
DC2542A::DC2542A(GPIO_TypeDef* CSPort, uint16_t CSPin,GPIO_TypeDef* ConfCSPort, uint16_t ConfCSPin,SPI_HandleTypeDef* MasterSpi,uint32_t BaseID,uint8_t cnv_trig,bool edge){
    // spi
   _CSPort=CSPort;
   _CSPin=CSPin;
   _ConfCSPort=ConfCSPort;
   _ConfCSPin=ConfCSPin;
   _MasterSPI=MasterSpi;
   _BaseID=BaseID;
   _SetingsID=0;
   _ID=_BaseID+(uint32_t)_SetingsID;
  _CNV_TRIG=cnv_trig;
   _CNV_EDGE=edge;//true=rising false =falling
}


/* starts and configutes LTC2358-18 and LTM2893 */
int DC2542A::begin(){
	configLTM2893();
	return 0;
	}


/* starts communication with the MPU-9250 */
int DC2542A::configLTM2893(){
	// UserConfig 0 Register
    _CFGREG0=0;
    _CFGREG1=0;
    if(_SADir){
    	_CFGREG0+=(1 << 0);
    }
    if(_SBDir){
    	_CFGREG0+=(1 << 1);
    }
    if(_SCDir){
    	_CFGREG0+=(1 << 2);
    }
    if(_CRC){
    	_CFGREG0+=(1 << 3) ;
    }
    _CFGREG0+=_OSCDIV;
    _CFGREG1+=_UC1_WORDL;
    _CFGREG1+=_OSCDIV;
    uint8_t buffer[2] = {_CFGREG0, _CFGREG1 };
  	HAL_GPIO_WritePin(_ConfCSPort, _ConfCSPin, GPIO_PIN_RESET);
  	HAL_SPI_Transmit(_MasterSPI, buffer, 2, SPI_TIMEOUT);
  	HAL_GPIO_WritePin(_ConfCSPort, _ConfCSPin, GPIO_PIN_SET);
  	if(_CNV_TRIG==TIM2_CH1){
  		HAL_GPIO_WritePin(_S0Port, _S0Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(_S1Port, _S1Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(_S2Port, _S2Pin, GPIO_PIN_RESET);
  	}
  	if(_CNV_TRIG==TIM2_CH3){
  		HAL_GPIO_WritePin(_S0Port, _S0Pin, GPIO_PIN_SET);
  		HAL_GPIO_WritePin(_S1Port, _S1Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(_S2Port, _S2Pin, GPIO_PIN_RESET);
  	}
  	if(_CNV_TRIG==TIM1_CH1){
  		HAL_GPIO_WritePin(_S0Port, _S0Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(_S1Port, _S1Pin, GPIO_PIN_SET);
  		HAL_GPIO_WritePin(_S2Port, _S2Pin, GPIO_PIN_RESET);
  	}
  	if(_CNV_TRIG==TIM1_CH2){
  		HAL_GPIO_WritePin(_S0Port, _S0Pin, GPIO_PIN_SET);
  		HAL_GPIO_WritePin(_S1Port, _S1Pin, GPIO_PIN_SET);
  		HAL_GPIO_WritePin(_S2Port, _S2Pin, GPIO_PIN_RESET);
  	}
  	if(_CNV_TRIG==TIM3_CH2){
  		HAL_GPIO_WritePin(_S0Port, _S0Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(_S1Port, _S1Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(_S2Port, _S2Pin, GPIO_PIN_SET);
  	}
  	if(_CNV_EDGE){
  		HAL_GPIO_WritePin(_INVPort, _INVPin, GPIO_PIN_RESET);
  	}
  	else{
  		HAL_GPIO_WritePin(_INVPort, _INVPin, GPIO_PIN_RESET);
  	}
	return 0;
	}

uint32_t DC2542A::getSampleCount(){
	return _SampleCount;
}


void DC2542A::tiggerCNVSOftware(){
	HAL_GPIO_WritePin(EXT_CNV_GPIO_Port, EXT_CNV_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EXT_CNV_GPIO_Port, EXT_CNV_Pin, GPIO_PIN_RESET);
	return;
}

float DC2542A::getNominalSamplingFreq(){
	return _NominalSamplingFreq;
}

int DC2542A::getData(DataMessage * Message,uint64_t RawTimeStamp){
	_SampleCount++;

	  int i;
	  uint8_t tx_array[24];
	  uint8_t rx_array[24];
	  tx_array[23] = (uint8_t)(cfgWORD >> 16);
	  tx_array[22] = (uint8_t)(cfgWORD >> 8);
	  tx_array[21] = (uint8_t)(cfgWORD);
	  for (i = 20; i >= 0; --i)
	  {
	    tx_array[i] = 0;
	  }
	  HAL_GPIO_WritePin(_CSPort, _CSPin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(_MasterSPI, tx_array,rx_array, 24, SPI_TIMEOUT);
	  HAL_GPIO_WritePin(_CSPort, _CSPin, GPIO_PIN_RESET);

	if (Message!=NULL){
		int readresult=-1;
	memcpy(Message,&empty_DataMessage,sizeof(DataMessage));//Copy default values into array
	Message->id=_ID;
	Message->unix_time=0XFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=	_SampleCount;



	return readresult;
	}
	else
	{
		return -2;
	}
}

void DC2542A::setSoftSPanConf(uint8_t channel,enum SOFTSPAN softSPanCode){
	if(channel<=7){
		_SoftSpanConf[channel]=softSPanCode;
		cfgWORD = cfgWORD | (uint32_t(_SoftSpanConf[channel] & 0x07) << (channel * 3));
	}
}

void DC2542A::generateCFGWord(){
	int i;
	for (i=0;i<8;i++)
	{
		cfgWORD = cfgWORD | (uint32_t(_SoftSpanConf[i] & 0x07) << (i * 3));
	}

}

int32_t DC2542A::sign_extend_17(uint32_t data)
{
  uint8_t sign;
  uint32_t mask = 0x20000;
  int32_t data_signed = data;
  sign = (data & mask) >> 17;
  if (sign)
    data_signed = data_signed | 0xFFFC0000;
  return data_signed;
}

// Calculates the voltage from ADC output data depending on the channel configuration
float DC2542A::calculateVoltage(uint32_t data, enum SOFTSPAN channel_configuration)
{
  float voltage;
  int32_t data_signed;
  switch (channel_configuration)
  {
    case DISABLED:
      voltage = 0;
      break;   // Disable Channel
    case ZEROTO5V12:
      voltage = (float)data * (1.25 * _vref / 1.000) / POW2_18;
      break;
    case PM5:
      data_signed = sign_extend_17(data);
      voltage = (float)data_signed * (1.25 * _vref  / 1.024) / POW2_17;
      break;
    case PM5V12:
      data_signed = sign_extend_17(data);
      voltage = (float)data_signed * (1.25 * _vref  / 1.000) / POW2_17;
      break;
    case ZEROTO10:
      voltage = (float)data * (2.50 * _vref  / 1.024) / POW2_18;
      break;
    case ZEROTO10V24:
      voltage = (float)data * (2.50 * _vref  / 1.000) / POW2_18;
      break;
    case PM10:
      data_signed = sign_extend_17(data);
      voltage = (float)data_signed * (2.50 * _vref  / 1.024) / POW2_17;
      break;
    case PM10V24:
      data_signed = sign_extend_17(data);
      voltage = (float)data_signed * (2.50 * _vref  ) / POW2_17;
      break;
  }
  return voltage;
}

float DC2542A::getMaxVal(uint8_t channel){
	enum SOFTSPAN channel_configuration=_SoftSpanConf[channel];
	float val=NAN;
	  switch (channel_configuration)
	  {
	    case DISABLED:
	      val = NAN;
	      break;   // Disable Channel
	    case ZEROTO5V12:
	      val = 5.12;
	      break;
	    case PM5:
	    	val=5.0;
	      break;
	    case PM5V12:
	    	val=5.12;
	      break;
	    case ZEROTO10:
	    	val=10.0;
	      break;
	    case ZEROTO10V24:
	      val=10.24;
	      break;
	    case PM10:
	      val=10;
	      break;
	    case PM10V24:
	      val=10.24;
	      break;
	  }
	  return val;
}

float DC2542A::getMinVal(uint8_t channel){
	enum SOFTSPAN channel_configuration=_SoftSpanConf[channel];
	float val=NAN;
	  switch (channel_configuration)
	  {
	    case DISABLED:
	      val = NAN;
	      break;   // Disable Channel
	    case ZEROTO5V12:
	      val = 0;
	      break;
	    case PM5:
	    	val=-5.0;
	      break;
	    case PM5V12:
	    	val=-5.12;
	      break;
	    case ZEROTO10:
	    	val=0.0;
	      break;
	    case ZEROTO10V24:
	      val=0.0;
	      break;
	    case PM10:
	      val=-10;
	      break;
	    case PM10V24:
	      val=-10.24;
	      break;
	  }
	  return val;
}

int DC2542A::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	strncpy(Message->Sensor_name,"MPU_9250\0",sizeof(Message->Sensor_name));
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
		Message->has_str_Data_08=true;
		strncpy(Message->str_Data_01,"Voltage\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Voltage\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Voltage\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"Voltage\0",sizeof(Message->str_Data_04));
		strncpy(Message->str_Data_05,"Voltage\0",sizeof(Message->str_Data_05));
		strncpy(Message->str_Data_06,"Voltage\0",sizeof(Message->str_Data_06));
		strncpy(Message->str_Data_07,"Voltage\0",sizeof(Message->str_Data_07));
		strncpy(Message->str_Data_08,"Voltage\0",sizeof(Message->str_Data_08));
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
		Message->has_str_Data_08=true;
		strncpy(Message->str_Data_01,"\\volt\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"\\volt\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"\\volt\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"\\volt\0",sizeof(Message->str_Data_04));
		strncpy(Message->str_Data_05,"\\volt\0",sizeof(Message->str_Data_05));
		strncpy(Message->str_Data_06,"\\volt\0",sizeof(Message->str_Data_06));
		strncpy(Message->str_Data_07,"\\volt\0",sizeof(Message->str_Data_07));
		strncpy(Message->str_Data_08,"\\volt\0",sizeof(Message->str_Data_08));
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
		Message->has_f_Data_08=true;
		Message->f_Data_01=262144;
		Message->f_Data_02=262144;
		Message->f_Data_03=262144;
		Message->f_Data_04=262144;
		Message->f_Data_05=262144;
		Message->f_Data_06=262144;
		Message->f_Data_07=262144;
		Message->f_Data_08=262144;
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
		Message->has_f_Data_08=true;
		Message->f_Data_01=getMinVal(0);
		Message->f_Data_02=getMinVal(1);
		Message->f_Data_03=getMinVal(2);
		Message->f_Data_04=getMinVal(3);
		Message->f_Data_05=getMinVal(4);
		Message->f_Data_06=getMinVal(5);
		Message->f_Data_07=getMinVal(6);
		Message->f_Data_08=getMinVal(7);
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
		Message->has_f_Data_08=true;
		Message->f_Data_01=getMaxVal(0);
		Message->f_Data_02=getMaxVal(1);
		Message->f_Data_03=getMaxVal(2);
		Message->f_Data_04=getMaxVal(3);
		Message->f_Data_05=getMaxVal(4);
		Message->f_Data_06=getMaxVal(5);
		Message->f_Data_07=getMaxVal(6);
		Message->f_Data_08=getMaxVal(7);
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
		Message->has_str_Data_08=true;
		strncpy(Message->str_Data_01,"Voltage/0\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Voltage/1\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Voltage/2\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"Voltage/3\0",sizeof(Message->str_Data_04));
		strncpy(Message->str_Data_05,"Voltage/4\0",sizeof(Message->str_Data_05));
		strncpy(Message->str_Data_06,"Voltage/5\0",sizeof(Message->str_Data_06));
		strncpy(Message->str_Data_07,"Voltage/6\0",sizeof(Message->str_Data_07));
		strncpy(Message->str_Data_08,"Voltage/7\0",sizeof(Message->str_Data_08));

	}
	return retVal;
}
