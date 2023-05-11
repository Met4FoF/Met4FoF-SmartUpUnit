/*
 * https://github.com/bolderflight/ADXL355
ADXL355.h
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



#ifndef ADXL355_h
#define ADXL355_h

#include "spi.h"
#include "gpio.h"
#include "stm32f7xx_hal.h"

// for osDelay instead of HAL_delay
#include "cmsis_os.h"

// memcpy and interger types
#include <stdint.h>
#include <cstring>
#include <math.h>
#include <stdint.h>

#include "pb.h"
#include "message.pb.h"
#include "Met4FoFSensor.h"


/*
 * From https://github.com/analogdevicesinc/no-OS/blob/master/include/no_os_util.h
 *
 */

#define SPI_TIMEOUT 100U
//extern DescriptionMessage empty_DescriptionMessage;
//extern DataMessage empty_DataMessage;
/* SPI commands */

class ADXL355: public Met4FoFSensors::Met4FoFSensor {
  public:
	enum adxl355_hpf_corner {
		ADXL355_HPF_OFF,
		ADXL355_HPF_24_7,
		ADXL355_HPF_6_2084,
		ADXL355_HPF_1_5545,
		ADXL355_HPF_0_3862,
		ADXL355_HPF_0_0954,
		ADXL355_HPF_0_0238
	};

	enum adxl355_odr_lpf {
		ADXL355_ODR_4000HZ,
		ADXL355_ODR_2000HZ,
		ADXL355_ODR_1000HZ,
		ADXL355_ODR_500HZ,
		ADXL355_ODR_250HZ,
		ADXL355_ODR_125HZ,
		ADXL355_ODR_62_5HZ,
		ADXL355_ODR_31_25HZ,
		ADXL355_ODR_15_625HZ,
		ADXL355_ODR_7_813HZ,
		ADXL355_ODR_3_906HZ
	};

	enum adxl355_range {
		ADXL355_RANGE_2G = 1,
		ADXL355_RANGE_4G = 2,
		ADXL355_RANGE_8G = 4,
	};

	enum adxl355_int_pol {
		ADXL355_INT_ACTIVE_LOW = 0,
		ADXL355_INT_ACTIVE_HIGH = 1
	};
	struct _adxl355_int_mask {
		uint8_t RDY_EN1 : 1;
		uint8_t FULL_EN1 : 1;
		uint8_t OVR_EN1 : 1;
		uint8_t ACT_EN1 : 1;
		uint8_t RDY_EN2 : 1;
		uint8_t FULL_EN2 : 1;
		uint8_t OVR_EN2 : 1;
		uint8_t ACT_EN2 : 1;
	};

	union adxl355_int_mask {
		struct _adxl355_int_mask fields;
		uint8_t value;
	};

	struct _adxl355_sts_reg_flags {
		uint8_t DATA_RDY : 1;
		uint8_t FIFO_FULL : 1;
		uint8_t FIFO_OVR : 1;
		uint8_t Activity : 1;
		uint8_t NVM_BUSY : 1;
		uint8_t reserved : 3;
	};

	union adxl355_sts_reg_flags {
		struct _adxl355_sts_reg_flags fields;
		uint8_t value;
	};

	struct _adxl355_act_en_flags {
		uint8_t ACT_X    : 1;
		uint8_t ACT_Y    : 1;
		uint8_t ACT_Z    : 1;
		uint8_t reserved : 4;
	};

	union adxl355_act_en_flags {
		struct _adxl355_act_en_flags fields;
		uint8_t value;
	};

	struct adxl355_frac_repr {
		int64_t integer;
		int32_t fractional;
	} ;

	enum adxl355_op_mode {
		ADXL355_MEAS_TEMP_ON_DRDY_ON = 0,
		ADXL355_STDBY_TEMP_ON_DRDY_ON = 1,
		ADXL355_MEAS_TEMP_OFF_DRDY_ON = 2,
		ADXL355_STDBY_TEMP_OFF_DRDY_ON = 3,
		ADXL355_MEAS_TEMP_ON_DRDY_OFF = 4,
		ADXL355_STDBY_TEMP_ON_DRDY_OFF = 5,
		ADXL355_MEAS_TEMP_OFF_DRDY_OFF = 6,
		ADXL355_STDBY_TEMP_OFF_DRDY_OFF = 7
	};

    ADXL355(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,SPI_HandleTypeDef* ADXL355spi,uint32_t BaseID);
    ~ADXL355();
    int begin();
    int getIDs();
    void setAccSelfTest(uint8_t SelftestStatus);
    void setGyroSelfTest(uint8_t SelftestStatus);
    int setRange(adxl355_range scaleFactor);
    int setHPFCorner(adxl355_hpf_corner hpfCorner);
    int setLPFCorner(adxl355_odr_lpf lpfFreq);
    int setOpMode(adxl355_op_mode opMode);
    int readSensor();
    int getData(DataMessage * Message,uint64_t RawTimeStamp);
    int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
    void increaseCaptureCountWORead(){_SampleCount++;return ;};
    float getNominalSamplingFreq();
    float accX=0.0;
    float accY=0.0;
    float accZ=0.0;
    float temp=0.0;
    uint32_t accRawX=0;
    uint32_t accRawY=0;
    uint32_t accRawZ=0;
    uint16_t tempRaw=0;
    uint32_t _SampleCount=0;
    uint8_t vendorID=0;
    uint8_t memsID=0;
    uint8_t deviceID=0;
    uint8_t revisionID=0;
  protected:
    adxl355_odr_lpf lpfSeting=ADXL355_ODR_4000HZ;
    adxl355_hpf_corner hpfSeting=ADXL355_HPF_OFF;
    adxl355_range scalfactoSetting=ADXL355_RANGE_2G;
    adxl355_sts_reg_flags status;
    GPIO_TypeDef* _SPICSTypeDef;
    uint16_t _SPICSPin;
    SPI_HandleTypeDef* _ADXL355spi;
    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int readStatus();
    float convertTempReading(uint16_t);
    float convertACCReading(uint32_t);
    uint32_t accbytesTounit(uint8_t *raw_array);
    adxl355_range accRange=ADXL355_RANGE_2G;
    adxl355_op_mode opMode=ADXL355_STDBY_TEMP_OFF_DRDY_OFF;
    float accScaleFactor=0.00003824593;
    uint8_t buffer[32];
    /* ADXL355 registers addresses */
    const uint8_t DEVID_AD =                0x00;
    const uint8_t DEVID_MST =               0x01;
    const uint8_t PARTID     =              0x02;
    const uint8_t REVID =                   0x03;
    const uint8_t STATUS =                  0x04;
    const uint8_t FIFO_ENTRIES =            0x05;
    const uint8_t TEMP2 =                   0x06;
    const uint8_t TEMP1  =                  0x07;
    const uint8_t XDATA3 =                  0x08;
    const uint8_t XDATA2 =                  0x09;
    const uint8_t XDATA1 =                  0x0A;
    const uint8_t YDATA3 =                  0x0B;
    const uint8_t YDATA2 =                  0x0C;
    const uint8_t YDATA1 =                  0x0D;
    const uint8_t ZDATA3 =                  0x0E;
    const uint8_t ZDATA2 =                  0x0F;
    const uint8_t ZDATA1 =                  0x10;
    const uint8_t FIFO_DATA =               0x11;
    const uint8_t OFFSET_X_H =              0x1E;
    const uint8_t OFFSET_X_L =              0x1F;
    const uint8_t OFFSET_Y_H =              0x20;
    const uint8_t OFFSET_Y_L =              0x21;
    const uint8_t OFFSET_Z_H =              0x22;
    const uint8_t OFFSET_Z_L =              0x23;
    const uint8_t ACT_EN =                  0x24;
    const uint8_t ACT_THRESH_H =            0x25;
    const uint8_t ACT_THRESH_L =            0x26;
    const uint8_t ACT_COUNT =               0x27;
    const uint8_t FILTER =                  0x28;
    const uint8_t FIFO_SAMPLES =            0x29;
    const uint8_t INT_MAP =                 0x2A;
    const uint8_t SYNC =                    0x2B;
    const uint8_t RANGE =                   0x2C;
    const uint8_t POWER_CTL =               0x2D;
    const uint8_t SELF_TEST =               0x2E;
    const uint8_t RESET =                   0x2F;
    const uint8_t ADXL355_RANGE_FIELD_MSK = 0x03;
    const uint8_t ADXL355_ODR_LPF_FIELD_MSK = 0x0f;
    const uint8_t ADXL355_HPF_FIELD_MSK =  0x80;
    const uint32_t ADXL355_NEG_ACC_MSK = 0xFFF00000;
};



#endif
