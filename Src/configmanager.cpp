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

