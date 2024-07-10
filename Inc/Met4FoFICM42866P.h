/*
 * lsm6dsrx.h
 *
 *  Created on: 21.10.2022
 *      Author: seeger01
 */

#ifndef MET4FOFICM42688P_H_
#define MET4FOFICM42688P_H_


#include "stm32f7xx_hal.h"

#include <stdint.h>
#include <cstring>
#include <math.h>

#include <functional>
#include <unordered_map>

#include "pb.h"
#include "message.pb.h"
#include "Met4FoFSensor.h"

#include "Icm426xx/Icm426xxDriver_HL.h"
#include "cmsis_os.h"//for OsDelay in setUp Function



class Met4FoFICM42866P:public Met4FoFSensors::Met4FoFSensor
{
public:
  Met4FoFICM42866P(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,SPI_HandleTypeDef* spiIfaceHandle,uint32_t BaseID);
  int getData(DataMessage * Message,uint64_t RawTimeStamp);
  int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
  void increaseCaptureCountWORead(){_SampleCount++;return ;};
  int setUp();
  float getNominalSamplingFreq();
  int setODR(ICM426XX_GYRO_CONFIG0_ODR_t odr);// gyro and accel will set to the same ODR
  int setAccFS(ICM426XX_ACCEL_CONFIG0_FS_SEL_t accFullScale);
  int setGyroFS(ICM426XX_GYRO_CONFIG0_FS_SEL_t gyroFullScale);
  int activateDRIINT1();
  private:
  ICM426XX_ACCEL_CONFIG0_FS_SEL_t _ACCFullScaleCOnfig=ICM426XX_ACCEL_CONFIG0_FS_SEL_2g;
  ICM426XX_GYRO_CONFIG0_FS_SEL_t _GyroFullScaleCOnfig=ICM426XX_GYRO_CONFIG0_FS_SEL_31dps;
  ICM426XX_GYRO_CONFIG0_ODR_t _ODRCOnfig=ICM426XX_GYRO_CONFIG0_ODR_1_KHZ;
  float _ACCFSScaleFactor=NAN;
  float _GyroFSScaleFactor=NAN;
  float _nominalODR=NAN;

  GPIO_TypeDef* _SPICSPort;
  uint16_t _SPICSPin;
  SPI_HandleTypeDef* _spi;
  static int read_reg(struct inv_icm426xx_serif* serif, uint8_t reg, uint8_t* buf, uint32_t len);
  static int write_reg(struct inv_icm426xx_serif* serif, uint8_t reg, const uint8_t* buf, uint32_t len);

  using CallbackType = std::function<void(inv_icm426xx_sensor_event_t*)>;
  static std::unordered_map<Met4FoFICM42866P*, CallbackType> callbackMap;

  static void evntCBStatic(inv_icm426xx_sensor_event_t* event);

  inv_icm426xx_sensor_event_t _lastEvent;
  void evntCB(inv_icm426xx_sensor_event_t* event);
  inv_icm426xx _Instance;

  inv_icm426xx_serif _serif = {
      this,
      read_reg,
      write_reg,
      nullptr, // configuration function is only needed if I3C is used
      1024 * 32,
      1024 * 32,
      ICM426XX_UI_SPI4
  };

  };



#endif /* Met4FoFICM42866P_H_ */
