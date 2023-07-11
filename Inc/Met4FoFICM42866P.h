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
  private:
  ICM426XX_ACCEL_CONFIG0_FS_SEL_t _ACCFullScaleCOnfig=ICM426XX_ACCEL_CONFIG0_FS_SEL_2g;
  ICM426XX_GYRO_CONFIG0_FS_SEL_t _GyroFullScaleCOnfig=ICM426XX_GYRO_CONFIG0_FS_SEL_31dps;
  ICM426XX_GYRO_CONFIG0_ODR_t _ODRCOnfig=ICM426XX_GYRO_CONFIG0_ODR_1_KHZ;
  float _ACCFSScaleFactor=NAN;
  float _GyroFSScaleFactor=NAN;
  float _nominalODR=NAN;
  int setODR(ICM426XX_GYRO_CONFIG0_ODR_t odr);// gyro and accel will set to the same ODR
  int setAccFS(ICM426XX_ACCEL_CONFIG0_FS_SEL_t accFullScale);
  int setGyroFS(ICM426XX_GYRO_CONFIG0_FS_SEL_t gyroFullScale);
  GPIO_TypeDef* _SPICSPort;
  uint16_t _SPICSPin;
  SPI_HandleTypeDef* _spi;


  int read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len);
  int write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, const uint8_t *buf,uint32_t len);
  /*
   * typedef struct {
	int      sensor_mask;
	uint16_t timestamp_fsync;
	int16_t  accel[3];
	int16_t  gyro[3];
	int16_t  temperature;
	int8_t   accel_high_res[3];
	int8_t   gyro_high_res[3];
   } inv_icm426xx_sensor_event_t;
   */
  inv_icm426xx_sensor_event_t _lastEvent;
  void evntCB(inv_icm426xx_sensor_event_t *event){_lastEvent=*event;};
  inv_icm426xx _Instance;
  /*
  struct inv_icm426xx_serif {
  	void *context;
  	int (*read_reg)(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len);
  	int (*write_reg)(struct inv_icm426xx_serif *serif, uint8_t reg, const uint8_t *buf,
  	                 uint32_t len);
  	int (*configure)(struct inv_icm426xx_serif *serif);
  	uint32_t max_read;
  	uint32_t max_write;
  	uint32_t serif_type;
  };
  */
  inv_icm426xx_serif _serif={NULL,
  (int (*)(inv_icm426xx_serif*, uint8_t, uint8_t*, uint32_t))&Met4FoFICM42866P::read_reg,
  (int (*)(inv_icm426xx_serif*, uint8_t, const uint8_t*, uint32_t))&Met4FoFICM42866P::write_reg,
  NULL,// configuration function is only need if i3C is used
  1024*32,
  1024*32,
  ICM426XX_UI_SPI4
  };

  };



#endif /* Met4FoFICM42866P_H_ */
