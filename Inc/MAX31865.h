/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865
  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328
  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef ADAFRUIT_MAX31865_H
#define ADAFRUIT_MAX31865_H

#define MAX31856_CONFIG_REG 0x00
#define MAX31856_CONFIG_BIAS 0x80
#define MAX31856_CONFIG_MODEAUTO 0x40
#define MAX31856_CONFIG_MODEOFF 0x00
#define MAX31856_CONFIG_1SHOT 0x20
#define MAX31856_CONFIG_3WIRE 0x10
#define MAX31856_CONFIG_24WIRE 0x00
#define MAX31856_CONFIG_FAULTSTAT 0x02
#define MAX31856_CONFIG_FILT50HZ 0x01
#define MAX31856_CONFIG_FILT60HZ 0x00

#define MAX31856_RTDMSB_REG 0x01
#define MAX31856_RTDLSB_REG 0x02
#define MAX31856_HFAULTMSB_REG 0x03
#define MAX31856_HFAULTLSB_REG 0x04
#define MAX31856_LFAULTMSB_REG 0x05
#define MAX31856_LFAULTLSB_REG 0x06
#define MAX31856_FAULTSTAT_REG 0x07

#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

#define SPI_TIMEOUT 100U


#include <spi.h>
#include <string.h>
#include <math.h>
#include "Met4FoFsensor.h"
typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

/*! Interface class for the MAX31865 RTD Sensor reader */
class MAX31865: public Met4FoFSensors::Met4FoFSensor{
public:
  MAX31865(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,SPI_HandleTypeDef* MAX31865spi,uint32_t BaseID);
  //~MAX31865();
  int getData(DataMessage * Message,uint64_t RawTimeStamp); //data getter function handels sensor communication
  int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);// get the protobuff description
  bool begin(max31865_numwires_t x = MAX31865_2WIRE);
  uint8_t readFault(void);
  void clearFault(void);
  uint16_t readRTD();
  void setWires(max31865_numwires_t wires);
  void autoConvert(bool b);
  void enable50Hz(bool b);
  void enableBias(bool b);
  void setRRef(float rRef){_rRef=rRef;};
  void setRNom(float rNominal){_rNominal=rNominal;};
  float temperature();
private:
  bool readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
  uint8_t readRegister8(uint8_t addr);
  uint16_t readRegister16(uint8_t addr);
  bool writeRegister8(uint8_t addr, uint8_t reg);
  float convertADCReading(uint16_t adcReading);
  GPIO_TypeDef* _SPICSTypeDef;
  uint16_t _SPICSPin;
  SPI_HandleTypeDef* _MAX31865spi;
  uint16_t _ADCReading=0;
  max31865_numwires_t _numWires=MAX31865_2WIRE;
  float _temp=0;
  float _rRef=430.0;
  float _rNominal=100.0;
  float _polyCoeffs[6]={-242.02f , 2.2228f , 2.5859e-3f , 4.8260e-6f , 2.8183e-8f , 1.5243e-10f};
};

#endif
