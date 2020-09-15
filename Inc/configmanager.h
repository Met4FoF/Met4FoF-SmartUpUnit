/*
 * configmanager.hpp
 *
 *  Created on: 08.07.2019
 *      Author: seeger01
 */

#ifndef CONFIGMANAGER_H_
#define CONFIGMANAGER_H_

#include "backupsram.h"
#include "lwip/ip_addr.h"
#include "math.h"



//CRC Function
uint16_t gen_crc16(const uint8_t *data, uint16_t size);

//https://en.wikibooks.org/wiki/C%2B%2B_Programming/Code/Design_Patterns#Singleton

/*
 * Singleton

The Singleton pattern ensures that a class has only one instance and provides a global point of access to that instance. It is named after the singleton set, which is defined to be a set containing one element. This is useful when exactly one object is needed to coordinate actions across the system.

Check list

    Define a private static attribute in the "single instance" class.
    Define a public static accessor function in the class.
    Do "lazy initialization" (creation on first use) in the accessor function.
    Define all constructors to be protected or private.
    Clients may only use the accessor function to manipulate the Singleton.

Let's take a look at how a Singleton differs from other variable types.

Like a global variable, the Singleton exists outside of the scope of any functions.
Traditional implementation uses a static member function of the Singleton class, which will create a single instance of the Singleton class on the first call, and forever return that instance.
 */
class ConfigManager
{
public:
    // The magic function, which allows access to the class from anywhere
    // To get the value of the instance of the class, call:
    //     StringSingleton::Instance().GetString();
	static ConfigManager& instance()
	{
		static ConfigManager _instance;
		return _instance;
	}
	~ConfigManager() {}
	bool getUseDHCP();
	bool setUseDHCP(bool UseDHCP);
	ip_addr_t getDeviceIP();
	bool setDeviceIP(ip_addr_t DeviceIP);
	ip_addr_t get_SubNetMaskDeviceIP();
	bool set_SubNetMaskDeviceIP(ip_addr_t SubNetMaskDeviceIP);
	ip_addr_t getUDPTargetIP();
	bool setUDPTargetIP(ip_addr_t UDPTargetIP);
	ip_addr_t getUDPSubnetmarsk();
	bool setUDPSubnetmarsk(ip_addr_t UDPSubnetmarsk);
	int16_t getUDPPort();
	bool setUDPPort(uint16_t UDPPort);
	uint32_t getStartcount();
	bool setADCCalCoevs(uint8_t ADCNumber,float slope,float xAxisCrossPoint,float RMSNoise);
	float getADCVoltage(uint8_t ADCNumber,uint32_t ADCVal);
	float getADCRMSNoise(uint8_t ADCNumber);
	uint16_t getBaseID();
	uint32_t getSensorBaseID(uint8_t SensorNumber);
	bool getBT1AtStart(void){return _BT1atStart;}
	bool getBT2AtStart(void){return _BT2atStart;}
	void flushData();
	/*
	int pushData(double timeStamp, std::string info, double val);
	int writeDataToCsv(std::string fileName);
	int writeToFileNameWhenBufferIsFull(std::string filename);

	int dumpUcharBufferToConsole(unsigned char *buffer, int bufLen);
	int testbed();
	*/
private:
	//Variables
	uint32_t _Startcounts;
	bool _BT1atStart=false;
	bool _BT2atStart=false;

	typedef struct {
		float slope;
		float xAxisCrossPoint;
		float RMSNoise;
	}ADCCalCoevs;

	ADCCalCoevs _ADCCalCoevs[3];
	ConfigManager()
	{
		/*
		reading GPIO Pins
		#define BT_1_Pin GPIO_PIN_4
		#define BT_1_GPIO_Port GPIOG
		#define BT_2_Pin GPIO_PIN_5
		#define BT_2_GPIO_Port GPIOG
		*/
		bool bt1_state= HAL_GPIO_ReadPin(BT_1_GPIO_Port, BT_1_Pin);
		bool bt2_state= HAL_GPIO_ReadPin(BT_2_GPIO_Port, BT_2_Pin);
		if(not bt1_state) {_BT1atStart=true;}
		if(not bt2_state) {_BT2atStart=true;}

		/*
		 * Initialize backup SRAM peripheral
		 */
			RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
			PWR->CSR1 |= PWR_CSR1_BRE;
			while ((PWR->CSR1 & PWR_CSR1_BRR) == 0)
		_Startcounts=BKPSRAM_Read32(STARTUPCOUNTADRESS);//dummy read
			if(_BT1atStart==true)
			{
				///FLUSH SRAM
				flushData();

			}
		_Startcounts=BKPSRAM_Read32(STARTUPCOUNTADRESS);
		_Startcounts++;
		BKPSRAM_Write32(STARTUPCOUNTADRESS,_Startcounts);
		for (int i=0;i<3;i++)
		{
			_ADCCalCoevs[i].slope=BKPSRAM_ReadFloat(ADCCOEVSADRESS+i*12);
			_ADCCalCoevs[i].xAxisCrossPoint=BKPSRAM_ReadFloat(ADCCOEVSADRESS+i*12+4);
			_ADCCalCoevs[i].RMSNoise=BKPSRAM_ReadFloat(ADCCOEVSADRESS+i*12+8);
		}

	}           // verhindert, dass ein Objekt von au�erhalb von N erzeugt wird.
				// protected, wenn man von der Klasse noch erben m�chte
	ConfigManager(ConfigManager&); /* verhindert, dass eine weitere Instanz via
								   Kopier-Konstruktor erstellt werden kann */
	ConfigManager & operator = (const ConfigManager &); //Verhindert weitere Instanz durch Kopie
};

/* Verwendung:
ConfigManager& s = ConfigManager::instance();
ConfigManager.xyz();
//oder
ConfigManager::instance().xyz(); */



#endif /* CONFIGMANAGER_H_ */
