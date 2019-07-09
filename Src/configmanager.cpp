/*
 * configmanager.cpp
 *
 *  Created on: 08.07.2019
 *      Author: seeger01
 */

#include "configmanager.hpp"



bool ConfigManager::getUseDHCP(){
	return BKPSRAM_Read8(USEDHCPADRESS);
}

bool ConfigManager::setUseDHCP(bool UseDHCP){
	bool retVal=false;
	BKPSRAM_Write8(USEDHCPADRESS, UseDHCP);
	if(BKPSRAM_Read8(USEDHCPADRESS)==UseDHCP){
		retVal=true;
	}
	return retVal;
}
ip_addr_t ConfigManager::getDeviceIP(){
	ip_addr_t retVal;
	retVal.addr=BKPSRAM_Read32(DEVICEIPADRESS);
	return retVal;
}

bool ConfigManager::setDeviceIP(ip_addr_t DeviceIP){
	bool retVal=false;
	BKPSRAM_Write32(DEVICEIPADRESS,DeviceIP.addr);
	if(BKPSRAM_Read32(DEVICEIPADRESS)==DeviceIP.addr){
		retVal=true;
	}
	return retVal;
}
ip_addr_t ConfigManager::get_SubNetMaskDeviceIP(){
	ip_addr_t retVal;
	retVal.addr=BKPSRAM_Read32(SUBNETMASKDEVICEIPADRESS);
	return retVal;
}
bool ConfigManager::set_SubNetMaskDeviceIP(ip_addr_t SubNetMaskDeviceIP){
	bool retVal=false;
	BKPSRAM_Write32(SUBNETMASKDEVICEIPADRESS,SubNetMaskDeviceIP.addr);
	if(BKPSRAM_Read32(SUBNETMASKDEVICEIPADRESS)==SubNetMaskDeviceIP.addr){
		retVal=true;
	}
	return retVal;
}
ip_addr_t ConfigManager::getUDPTargetIP(){
	ip_addr_t retVal;
	retVal.addr=BKPSRAM_Read32(UDPTARGETIPADRESS );
	return retVal;
}
bool ConfigManager::setUDPTargetIP(ip_addr_t UDPTargetIP){
	bool retVal=false;
	BKPSRAM_Write32(UDPTARGETIPADRESS,UDPTargetIP.addr);
	if(BKPSRAM_Read32(UDPTARGETIPADRESS)==UDPTargetIP.addr){
		retVal=true;
	}
	return retVal;
}
ip_addr_t ConfigManager::getUDPSubnetmarsk(){
	ip_addr_t retVal;
	retVal.addr=BKPSRAM_Read32(UDPSUBNETMASKADRESS);
	return retVal;
}
bool ConfigManager::setUDPSubnetmarsk(ip_addr_t UDPSubnetmarsk){
	bool retVal=false;
	BKPSRAM_Write32(UDPSUBNETMASKADRESS, UDPSubnetmarsk.addr);
	if(BKPSRAM_Read32(UDPSUBNETMASKADRESS)==UDPSubnetmarsk.addr){
		retVal=true;
	}
	return retVal;
}
int16_t ConfigManager::getUDPPort(){
	return BKPSRAM_Read16(UDPPORTADRESS);
}

bool ConfigManager::setUDPPort(uint16_t UDPPort){
	bool retVal=false;
	BKPSRAM_Write16(UDPPORTADRESS, UDPPort);
	if(BKPSRAM_Read16(UDPPORTADRESS)==UDPPort){
		retVal=true;
	}
	return retVal;
}

uint32_t ConfigManager::getStartcount(){
	return _Startcounts;
}

bool ConfigManager::setADCCalCoevs(uint8_t ADCNumber,float slope,float xAxisCrossPoint,float RMSNoise){
	if(ADCNumber>2)
	{
		return false;
	}
	bool retVal = false;
_ADCCalCoevs[ADCNumber].slope=slope;
_ADCCalCoevs[ADCNumber].xAxisCrossPoint=xAxisCrossPoint;
_ADCCalCoevs[ADCNumber].RMSNoise=RMSNoise;
BKPSRAM_WriteFloat(ADCCOEVSADRESS+ADCNumber*12,slope);
BKPSRAM_WriteFloat(ADCCOEVSADRESS+ADCNumber*12+4,xAxisCrossPoint);
BKPSRAM_WriteFloat(ADCCOEVSADRESS+ADCNumber*12+8,RMSNoise);
if(BKPSRAM_ReadFloat(ADCCOEVSADRESS+ADCNumber*12) == slope &&
		BKPSRAM_ReadFloat(ADCCOEVSADRESS+ADCNumber*12+4) == xAxisCrossPoint &&
		BKPSRAM_ReadFloat(ADCCOEVSADRESS+ADCNumber*12+8) == RMSNoise)
		{
			retVal=true;
		}
return retVal;
}

float ConfigManager::getADCVoltage(uint8_t ADCNumber,uint32_t ADCVal){
	float retVal = NAN;
	if(ADCNumber>2)
	{
		return retVal;
	}
	retVal=(float)ADCVal*_ADCCalCoevs[ADCNumber].slope+_ADCCalCoevs[ADCNumber].xAxisCrossPoint;
	return retVal;
}

float ConfigManager::getADCRMSNoise(uint8_t ADCNumber){
	float retVal = NAN;
	if(ADCNumber>2)
	{
		return retVal;
	}
	retVal=_ADCCalCoevs[ADCNumber].RMSNoise;
	return retVal;
}

