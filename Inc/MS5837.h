/* Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
------------------------------------------------------------
 
Title: Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library

Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties MS5837-30BA pressure/temperature 
sensor.

Authors: Rustom Jehangir, Blue Robotics Inc.
         Adam Šimko, Blue Robotics Inc.
		 Benedikt Seeger, PTB germany
-------------------------------
The MIT License (MIT)

Copyright (c) 2015 Blue Robotics Inc + 2019 PTB (changes for STM32).

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/ 

#ifndef MS5837_H_BLUEROBOTICS
#define MS5837_H_BLUEROBOTICS
#include "stm32f7xx_hal.h"
// for osDelay instead of HAL_delay
#include "cmsis_os.h"
#include "pb.h"
#include "message.pb.h"
#include <math.h>
#include "Met4FoFSensor.h"

class MS5837: public Met4FoFSensors::Met4FoFSensor {
public:
	static const float Pa;
	static const float bar;
	static const float mbar;

	static const uint8_t MS5837_30BA;
	static const uint8_t MS5837_02BA;

	MS5837(I2C_HandleTypeDef* I2C,uint8_t model,uint32_t BaseID);

	bool init();
	/** Provide the density of the working fluid in kg/m^3. Default is for 
	 * seawater. Should be 997 for freshwater.
	 */
	void setFluidDensity(float density);

	/** The read from I2C takes up to 40 ms, so use sparingly is possible.
	 */
	void read();

	/** Pressure returned in mbar or mbar*conversion rate.
	 */
	float pressure(float conversion = 1.0f);

	/** Temperature returned in deg C.
	 */
	float temperature();

	/** Depth returned in meters (valid for operation in incompressible
	 *  liquids only. Uses density that is set for fresh or seawater.
	 */
	float depth();

	/** Altitude returned in meters (valid for operation in air only).
	 */
	float altitude();

	int getData(DataMessage * Message,uint64_t RawTimeStamp);

	int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);

	uint32_t getSampleCount(){return _SampleCount;};

	void increaseCaptureCountWORead(){_SampleCount++;return ;};
	int setBaseID(uint32_t BaseID){_ID=BaseID;return 1;};

	float getNominalSamplingFreq(){return 0.0;};

private:
	I2C_HandleTypeDef * _I2C;
	uint16_t C[8];
	uint32_t D1, D2;
	int32_t TEMP;
	int32_t P;
	uint8_t _model;
	uint32_t _ID;
	uint32_t _SampleCount=0;
	float fluidDensity;

	/** Performs calculations per the sensor data sheet for conversion and
	 *  second order compensation.
	 */
	void calculate();

	uint8_t crc4(uint16_t n_prom[]);
};

#endif
