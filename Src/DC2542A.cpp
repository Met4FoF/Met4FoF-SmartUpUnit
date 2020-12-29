#include "DC2542A.h"

/* MPU9250 object, input the SPI bus and chip select pin */
DC2542A::DC2542A(GPIO_TypeDef* CSPort, uint16_t CSPin,GPIO_TypeDef* ConfCSPort, uint16_t ConfCSPin,SPI_HandleTypeDef* MasterSpi,uint32_t BaseID){
    // spi
   _CSPort=CSPort;
   _CSPin=CSPin;
   _ConfCSPort=ConfCSPort;
   _ConfCSPin=ConfCSPin;
   _MasterSPI=MasterSpi;
   _BaseID=BaseID;
   _SetingsID=0;
   _ID=_BaseID+(uint32_t)_SetingsID;
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

float DC2542A::getNominalSamplingFreq(){
	return _NominalSamplingFreq;
}

int DC2542A::getData(DataMessage * Message,uint64_t RawTimeStamp){
	_SampleCount++;
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
		Message->f_Data_01=-10.24;
		Message->f_Data_02=-10.24;
		Message->f_Data_03=-10.24;
		Message->f_Data_04=-10.24;
		Message->f_Data_05=-10.24;
		Message->f_Data_06=-10.24;
		Message->f_Data_07=-10.24;
		Message->f_Data_08=-10.24;
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
		Message->f_Data_01=10.24;
		Message->f_Data_02=10.24;
		Message->f_Data_03=10.24;
		Message->f_Data_04=10.24;
		Message->f_Data_05=10.24;
		Message->f_Data_06=10.24;
		Message->f_Data_07=10.24;
		Message->f_Data_08=10.24;
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
