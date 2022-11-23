/*
/*
 * Met4FoFGPSPub.cpp
 *
 *  Created on: 08.10.2018
 *      Author: seeger01
 */


#include <Met4FoFGPSPub.h>


Met4FoFGPSPub::Met4FoFGPSPub(struct tref * GPS_ref,uint32_t BaseID):
		Met4FoFSensor::Met4FoFSensor(BaseID){
	    _BaseID=BaseID;
	    _SetingsID=0;
		_ID=_BaseID+(uint32_t)_SetingsID;
		_GPSTimeRef=GPS_ref;
		_publish_time_ticks=true;
}



int Met4FoFGPSPub::getData(DataMessage * Message,uint64_t RawTimeStamp){
	int result=0;
	_SampleCount++;
	if (Message==0){
		return result;
	}
	lgw_gps_get(&_utc_time, &_gps_time, &_coords, NULL);
	memcpy(Message,&empty_DataMessage,sizeof(DataMessage));//Copy default values into array
	Message->id=_ID;
	Message->unix_time=0XFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=	_SampleCount;
	Message->Data_01=_utc_time.tv_sec-1609459200u;
	Message->has_Data_02=true;
	Message->Data_02=_utc_time.tv_nsec;
	Message->has_Data_03=true;
	Message->Data_03= _coords.lat;
	Message->has_Data_04=true;
	Message->Data_04= _coords.lon;
	Message->has_Data_05=true;
	Message->Data_05= _coords.alt;
	Message->has_Data_06=true;
	Message->Data_06= _coords.hdop;
	Message->has_Data_07=true;
	Message->Data_07= _coords.cmg;
	Message->has_Data_08=true;
	Message->Data_08= _coords.sog;
	Message->has_Data_09=true;
	Message->Data_09= _coords.sat;
	Message->has_Data_10=true;
	Message->Data_10= _GPSTimeRef->xtal_err;
	Message->has_Data_11=true;
	Message->Data_11= _GPSTimeRef->xtal_err_deviation;
	if(_publish_time_ticks==true){
	Message->has_time_ticks=true;
	Message->time_ticks=RawTimeStamp;
	}
	return result;
}

int Met4FoFGPSPub::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	Message->id=_ID;
	strncpy(Message->Sensor_name,"uBlox NEO-7 GPS\0",sizeof(Message->Sensor_name));
	Message->Description_Type=DESCRIPTION_TYPE;
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"Sec since (1609459200) 2020.01.01 00:00\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_02,"Time nano Secs\0",sizeof(Message->str_Data_02));
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_03,"Latitude\0",sizeof(Message->str_Data_03));
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_04,"Longitude\0",sizeof(Message->str_Data_04));
		Message->has_str_Data_05=true;
		strncpy(Message->str_Data_05,"Altitude\0",sizeof(Message->str_Data_05));
		Message->has_str_Data_06=true;
		strncpy(Message->str_Data_06,"Horizontal delution of precision\0",sizeof(Message->str_Data_06));
		Message->has_str_Data_07=true;
		strncpy(Message->str_Data_07,"Course Made Good\0",sizeof(Message->str_Data_07));
		Message->has_str_Data_08=true;
		strncpy(Message->str_Data_08,"Speed over ground\0",sizeof(Message->str_Data_08));
		Message->has_str_Data_09=true;
		strncpy(Message->str_Data_09,"Number of satellites\0",sizeof(Message->str_Data_09));
		Message->has_str_Data_10=true;
		strncpy(Message->str_Data_10,"System clock frequency\0",sizeof(Message->str_Data_10));
		Message->has_str_Data_11=true;
		strncpy(Message->str_Data_11,"System clock frequency std\0",sizeof(Message->str_Data_11));

	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_UNIT)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"\\seconds\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_02,"\\nanoseconds\0",sizeof(Message->str_Data_02));
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_03,"\\degree\0",sizeof(Message->str_Data_03));
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_04,"\\degree\0",sizeof(Message->str_Data_04));
		Message->has_str_Data_05=true;
		strncpy(Message->str_Data_05,"\\metre\0",sizeof(Message->str_Data_05));
		Message->has_str_Data_06=true;
		strncpy(Message->str_Data_06,"\\A.U.\0",sizeof(Message->str_Data_06));
		Message->has_str_Data_07=true;
		strncpy(Message->str_Data_07,"\\degree\0",sizeof(Message->str_Data_07));
		Message->has_str_Data_08=true;
		strncpy(Message->str_Data_08,"\\metre\\second\\tothe{-1}\0",sizeof(Message->str_Data_08));
		Message->has_str_Data_09=true;
		strncpy(Message->str_Data_09,"\\one\0",sizeof(Message->str_Data_09));
		Message->has_str_Data_10=true;
		strncpy(Message->str_Data_10,"\\hertz\0",sizeof(Message->str_Data_11));
		Message->has_str_Data_11=true;
		strncpy(Message->str_Data_11,"\\hertz\0",sizeof(Message->str_Data_11));
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=4294967296;
		Message->has_f_Data_02=true;
		Message->f_Data_02=1e9;
		Message->has_f_Data_03=true;
		Message->f_Data_03=1800000000;
		Message->has_f_Data_04=true;
		Message->f_Data_04=3600000000;
		Message->has_f_Data_05=true;
		Message->f_Data_05=NAN;
		Message->has_f_Data_04=true;
		Message->f_Data_04=NAN;
		Message->has_f_Data_05=true;
		Message->f_Data_05=NAN;
		Message->has_f_Data_06=true;
		Message->f_Data_06=NAN;
		Message->has_f_Data_07=true;
		Message->f_Data_07=NAN;
		Message->has_f_Data_08=true;
		Message->f_Data_08=NAN;
		Message->has_f_Data_09=true;
		Message->f_Data_09=NAN;
		Message->has_f_Data_10=true;
		Message->f_Data_10=NAN;
		Message->has_f_Data_11=true;
		Message->f_Data_11=NAN;

	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=0;
		Message->has_f_Data_02=true;
		Message->f_Data_02=0;
		Message->has_f_Data_03=true;
		Message->f_Data_03=-90.0;
		Message->has_f_Data_04=true;
		Message->f_Data_04=-180.0;
		Message->has_f_Data_05=true;
		Message->f_Data_05=NAN;
		Message->has_f_Data_06=true;
		Message->f_Data_06=NAN;
		Message->has_f_Data_07=true;
		Message->f_Data_07=-180.0f;
		Message->has_f_Data_08=true;
		Message->f_Data_08=0.0f;
		Message->has_f_Data_09=true;
		Message->f_Data_09=0.0f;
		Message->has_f_Data_10=true;
		Message->f_Data_10=0.0f;
		Message->has_f_Data_11=true;
		Message->f_Data_11=0.0f;
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=4294967296.0f;
		Message->has_f_Data_02=true;
		Message->f_Data_02=1e9f;
		Message->has_f_Data_03=true;
		Message->f_Data_03=90.0;
		Message->has_f_Data_04=true;
		Message->f_Data_04=180.0;
		Message->has_f_Data_05=true;
		Message->f_Data_05=NAN;
		Message->has_f_Data_06=true;
		Message->f_Data_06=NAN;
		Message->has_f_Data_07=true;
		Message->f_Data_07=180.0f;
		Message->has_f_Data_08=true;
		Message->f_Data_08=6840.0f;
		Message->has_f_Data_09=true;
		Message->f_Data_09=30.0f;
		Message->has_f_Data_10=true;
		Message->f_Data_10=NAN;
		Message->has_f_Data_11=true;
		Message->f_Data_11=NAN;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"Time/0\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_02,"Time/1\0",sizeof(Message->str_Data_02));
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_03,"Coordinates/0\0",sizeof(Message->str_Data_03));
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_04,"Coordinates/1\0",sizeof(Message->str_Data_04));
		Message->has_str_Data_05=true;
		strncpy(Message->str_Data_05,"Coordinates/2\0",sizeof(Message->str_Data_05));
		Message->has_str_Data_06=true;
		strncpy(Message->str_Data_06,"Precision/0\0",sizeof(Message->str_Data_06));
		Message->has_str_Data_07=true;
		strncpy(Message->str_Data_07,"Course/0\0",sizeof(Message->str_Data_07));
		Message->has_str_Data_08=true;
		strncpy(Message->str_Data_08,"Course/1\0",sizeof(Message->str_Data_08));
		Message->has_str_Data_09=true;
		strncpy(Message->str_Data_09,"Precision/1\0",sizeof(Message->str_Data_09));
		Message->has_str_Data_10=true;
		strncpy(Message->str_Data_10,"System_frequency/0\0",sizeof(Message->str_Data_10));
		Message->has_str_Data_11=true;
		strncpy(Message->str_Data_11,"System_frequency/1\0",sizeof(Message->str_Data_11));

	}
	if(_publish_time_ticks==true){
	Message->has_time_ticks=true;
	}
	return retVal;
}






