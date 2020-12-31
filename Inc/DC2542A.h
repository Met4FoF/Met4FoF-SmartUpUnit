/*
 * https://github.com/bolderflight/MPU9250
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems
changed by B. Seeger to be usable at stm32 with HAl 2019

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/



#ifndef MPU9250_h
#define MPU9250_h

#include "spi.h"
#include "gpio.h"
#include "stm32f7xx_hal.h"

// for osDelay instead of HAL_delay
#include "cmsis_os.h"

// memcpy and interger types
#include <stdint.h>
#include <cstring>
#include <math.h>

#include "pb.h"
#include "message.pb.h"
#include "Met4FoFSensor.h"




// UserConfig 1 Register
//- OSCDIV

//- DEVCNT
#define POW2_18 262144
#define POW2_17 131072

#define SPI_TIMEOUT 100U

//extern DescriptionMessage empty_DescriptionMessage;
//extern DataMessage empty_DataMessage;

class DC2542A: public Met4FoFSensor {
  public:
    enum WORDL
    {
WORDL_8   =       ((0 << 0) | (1 << 7)),
WORDL_12  =       ((1 << 0) | (1 << 7)),
WORDL_14   =      ((2 << 0) | (1 << 7)),
WORDL_16    =     ((3 << 0) | (1 << 7)),
WORDL_18     =    ((4 << 0) | (1 << 7)),
WORDL_20      =   ((5 << 0) | (1 << 7)),
WORDL_24       =  ((6 << 0) | (1 << 7)),
WORDL_32        = ((7 << 0) | (1 << 7))
    };
    enum OSCDIV
    {
OSCDIV_100 =      (0 << 4),
OSCDIV_66  =      (1 << 4),
OSCDIV_50   =     (2 << 4),
OSCDIV_40    =    (3 << 4),
OSCDIV_33     =   (4 << 4),
OSCDIV_25      =  (5 << 4),
OSCDIV_12       = (6 << 4),
OSCDIV_6         =(7 << 4)
    };
    enum DEVCNT
    {
DEVCNT_1  =       ((0 << 3) | (1 << 7)),
DEVCNT_2   =      ((1 << 3) | (1 << 7)),
DEVCNT_3    =     ((2 << 3) | (1 << 7)),
DEVCNT_4     =    ((3 << 3) | (1 << 7))
    };
    enum CNV_TRIG
	{
    	TIM2_CH1,
		TIM2_CH3,
		TIM1_CH1,
		TIM1_CH2,
    	TIM3_CH2
	};
    enum SOFTSPAN
	{
    	DISABLED=0x00,
		ZEROTO5V12=0x01,
		PM5=0x02,
		PM5V12=0x03,
    	ZEROTO10=0x04,
		ZEROTO10V24=0x05,
		PM10=0x06,
		PM10V24=0x07
	};
    DC2542A(GPIO_TypeDef* CSPort, uint16_t CSPin,GPIO_TypeDef* ConfCSPort, uint16_t ConfCSPin,SPI_HandleTypeDef* MasterSpi,uint32_t BaseID,uint8_t cnv_trig,bool edge);
    int begin();
    int setBaseID(uint32_t BaseID);
    int getData(DataMessage * Message,uint64_t RawTimeStamp);
    int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
    uint32_t getSampleCount();
    float getNominalSamplingFreq();
    void  tiggerCNVSOftware();
    void setSoftSPanConf(uint8_t channel,enum SOFTSPAN softSPanCode);
  protected:
    uint32_t _ID;
    uint32_t _BaseID;
    uint8_t _SetingsID;
    uint32_t _SampleCount=0;
    double _vref=4.096;
    float _NominalSamplingFreq=-1;
    // spi
    GPIO_TypeDef* _CSPort;
    uint16_t _CSPin;
    GPIO_TypeDef* _ConfCSPort;
    uint16_t _ConfCSPin;
    SPI_HandleTypeDef* _MasterSPI;
    bool _SADir=true;//True = iso-->Logic
    bool _SBDir=true;//True = iso-->Logic
    bool _SCDir=true;//True = iso-->Logic
    bool _CRC=true;
    uint8_t _OSCDIV=OSCDIV_100;
    uint8_t _UC1_WORDL=WORDL_24;
    uint8_t _DEVCNT=DEVCNT_1;
    uint8_t _CFGREG0;
    uint8_t _CFGREG1;
    uint8_t _CNV_TRIG=TIM3_CH2;
    bool _CNV_EDGE=true;//true=rising false =falling
    //conv mux pins
    GPIO_TypeDef* _S0Port=S0_GPIO_Port;
    uint16_t _S0Pin=S0_Pin;

    GPIO_TypeDef* _S1Port=S1_GPIO_Port;
    uint16_t _S1Pin=S1_Pin;

    GPIO_TypeDef* _S2Port=S1_GPIO_Port;
    uint16_t _S2Pin=S1_Pin;

    GPIO_TypeDef* _INVPort=INV_GPIO_Port;
    uint16_t _INVPin=INV_Pin;
    enum SOFTSPAN _SoftSpanConf[8]={PM10V24,PM10V24,PM10V24,PM10V24,PM10V24,PM10V24,PM10V24,PM10V24};
    uint32_t cfgWORD;
    void generateCFGWord();
    int configLTM2893();
    int32_t sign_extend_17(uint32_t data);
    float getMaxVal(uint8_t channel);
    float getMinVal(uint8_t channel);
    float calculateVoltage(uint32_t data, enum SOFTSPAN channel_configuration);
};


#endif
