#include "MS5837.h"

#define MS5837_ADDR               0xEC//=0x76<<1
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;

MS5837::MS5837(I2C_HandleTypeDef* I2C,uint8_t model) {
	fluidDensity = 1029;
	_model=model;
	_I2C=I2C;
	C[8]={0};
	D1=0;
	D2=0;
	TEMP=0;
	P=0;
	_ID=0;
}


bool MS5837::init(uint32_t BaseID) {
	_ID=BaseID;
	// Reset the MS5837, per datasheet
	uint8_t CMD=MS5837_RESET;
	HAL_I2C_Master_Transmit(_I2C,MS5837_ADDR,&CMD,1,100);

	// Wait for reset to complete
	osDelay(10);
	//Write Adress the read calldata in one block

	// Read calibration values and CRC


	HAL_StatusTypeDef result;
	for (uint8_t i=0;i<8;i++){
		CMD=MS5837_PROM_READ+(2*i);
		uint8_t Data[2]={0};
		HAL_I2C_Master_Transmit(_I2C,MS5837_ADDR,&CMD,1,100);
		result=HAL_I2C_Master_Receive(_I2C,MS5837_ADDR,(uint8_t*)&Data[0],2, 100);
		C[i]=(Data[0]<<8)|Data[1];
	}
	/* TODO REMOVE THIS DEBUGIG HACK constans read out with arduino
	11:42:53.152 -> 1Int 42162 HEX A4B2RAW Hex A4 B2
	11:42:53.185 -> 2Int 41771 HEX A32BRAW Hex A3 2B
	11:42:53.218 -> 3Int 25569 HEX 63E1RAW Hex 63 E1
	11:42:53.251 -> 4Int 26543 HEX 67AFRAW Hex 67 AF
	11:42:53.284 -> 5Int 31473 HEX 7AF1RAW Hex 7A F1
	11:42:53.317 -> 6Int 26888 HEX 6908RAW Hex 69 8
	11:42:53.350 -> 7Int 0 HEX 0RAW Hex0 0
	*/
	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	if ( crcCalculated == crcRead ) {
		return true; // Initialization success
	}

	return false; // CRC fail
}

void MS5837::setFluidDensity(float density) {
	fluidDensity = density;
}

void MS5837::read() {
	// Request D1 conversion
	uint8_t CMD;
	CMD=MS5837_CONVERT_D1_8192;
	HAL_I2C_Master_Transmit(_I2C,MS5837_ADDR,&CMD,1,100);

	osDelay(20); // Max conversion time per datasheet
	CMD=MS5837_ADC_READ;
	HAL_I2C_Master_Transmit(_I2C,MS5837_ADDR,&CMD,1,100);
	uint8_t Data[3];
	HAL_I2C_Master_Receive(_I2C,MS5837_ADDR,Data,3, 100);
	D1=(Data[0]<<16)|(Data[1]<<8)|Data[2];
	// Request D2 conversion
	CMD=MS5837_CONVERT_D2_8192;
	HAL_I2C_Master_Transmit(_I2C,MS5837_ADDR,&CMD,1,100);

	osDelay(20); // Max conversion time per datasheet
	CMD=MS5837_ADC_READ;
	HAL_I2C_Master_Transmit(_I2C,MS5837_ADDR,&CMD,1,100);
	
	HAL_I2C_Master_Receive(_I2C,MS5837_ADDR,Data ,3, 100);
	D2=(Data[0]<<16)|(Data[1]<<8)|Data[2];
	calculate();
}

void MS5837::calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation
	
	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;  
	int32_t Ti = 0;    
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;
	
	// Terms called
	dT = D2-uint32_t(C[5])*256l;
	if ( _model == MS5837_02BA ) {
		SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
		OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
		P = (D1*SENS/(2097152l)-OFF)/(32768l);
	} else {
		SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
		OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
		P = (D1*SENS/(2097152l)-OFF)/(8192l);
	}
	
	// Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
	
	//Second order compensation
	if ( _model == MS5837_02BA ) {
		if((TEMP/100)<20){         //Low temp
			Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
		}
	} else {
		if((TEMP/100)<20){         //Low temp
			Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15){    //Very low temp
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
		else if((TEMP/100)>=20){    //High temp
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
	}
	
	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	TEMP = (TEMP-Ti);
	
	if ( _model == MS5837_02BA ) {
		P = (((D1*SENS2)/2097152l-OFF2)/32768l); 
	} else {
		P = (((D1*SENS2)/2097152l-OFF2)/8192l);
	}
}

float MS5837::pressure(float conversion) {
    if ( _model == MS5837_02BA ) {
        return P*conversion/100.0f;
    }
    else {
        return P*conversion/10.0f;
    }
}

float MS5837::temperature() {
	return TEMP/100.0f;
}

float MS5837::depth() {
	return (pressure(MS5837::Pa)-101300)/(fluidDensity*9.80665);
}

float MS5837::altitude() {
	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}

int MS5837::getData(DataMessage * Message,uint64_t RawTimeStamp){
	int result=0;
	_SampleCount++;
	if (Message==0){
		return result;
	}
	memcpy(Message,&empty_DataMessage,sizeof(DataMessage));//Copy default values into array
	Message->id=_ID;
	Message->unix_time=0xFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=_SampleCount;
	MS5837::read();
	MS5837::calculate();
	Message->Data_01=MS5837::temperature();
	Message->has_Data_02=true;
	Message->Data_02=MS5837::pressure()*MS5837::Pa;
	return result;
}

int MS5837::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	if(_model==MS5837::MS5837_30BA){
	strncpy(Message->Sensor_name,"MS5837_30BA\0",sizeof(Message->Sensor_name));
	}
	if(_model==MS5837::MS5837_02BA){
	strncpy(Message->Sensor_name,"MS5837_02BA\0",sizeof(Message->Sensor_name));
	}
	Message->id=_ID;
	Message->Description_Type=DESCRIPTION_TYPE;
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_01,"Temperature\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Pressure\0",sizeof(Message->str_Data_02));
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_UNIT)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_01,"\\degreecelsius\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"\\pascal\0",sizeof(Message->str_Data_02));
	}
	if(_model==MS5837::MS5837::MS5837_02BA){
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->f_Data_01=12500;
		Message->f_Data_02=119000;
	}
	//TODO add min and max scale values as calls member vars so they have not to be calculated all the time
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->f_Data_01=-40;
		Message->f_Data_02=1000;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->f_Data_01=85;
		Message->f_Data_02=120000;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_01,"Temperature/0\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Pressure/0\0",sizeof(Message->str_Data_10));
	}
	}
	if(_model==MS5837::MS5837_30BA){
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->f_Data_01=12500;
		Message->f_Data_02=119000;
	}
	//TODO add min and max scale values as calls member vars so they have not to be calculated all the time
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->f_Data_01=-40;
		Message->f_Data_02=0;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->f_Data_01=85;
		Message->f_Data_02=3e6;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_01,"Temperature\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Pressure\0",sizeof(Message->str_Data_02));
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_01,"Temperature/0\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Pressure/0\0",sizeof(Message->str_Data_10));
	}
	}
	return retVal;
}


uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
