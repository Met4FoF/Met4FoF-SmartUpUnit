/*
 * lsm6dsrx.h
 *
 *  Created on: 21.10.2022
 *      Author: seeger01
 */

#ifndef MET4FOFLSM6DSRX_H_
#define MET4FOFLSM6DSRX_H_


#include "stm32f7xx_hal.h"

#include <stdint.h>
#include <cstring>
#include <math.h>

#include "pb.h"
#include "message.pb.h"
#include "Met4FoFSensor.h"

#include "lsm6dsrx_reg.h"
#include "cmsis_os.h"//for OsDelay in setUp Function

class Met4FoFLsm6dsrx: public Met4FoFSensor
{
public:
	enum outPutDatarate
	{
	  ODR_OFF    = 0,
	  ODR_12Hz5  = 1,
	  ODR_26Hz   = 2,
	  ODR_52Hz   = 3,
	  ODR_104Hz  = 4,
	  ODR_208Hz  = 5,
	  ODR_416Hz  = 6,
	  ODR_833Hz  = 7,
	  ODR_1666Hz = 8,
	  ODR_332Hz = 9,
	  ODR_6667Hz = 10
	};
  Met4FoFLsm6dsrx(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,SPI_HandleTypeDef* spiIfaceHandle,uint32_t BaseID);
  int getData(DataMessage * Message,uint64_t RawTimeStamp);
  int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
  uint32_t getSampleCount();
  void increaseCaptureCountWORead(){_SampleCount++;return ;};
  int setBaseID(uint32_t BaseID);
  float getNominalSamplingFreq(){return _NominalSamplingFreq;};
  int setUp();
  void dummyRead();
  private:
  int setODR(Met4FoFLsm6dsrx::outPutDatarate odr);
  int setAccFS(lsm6dsrx_fs_xl_t accFullScale);
  int setGyroFS(lsm6dsrx_fs_g_t gyroFullScale);
  GPIO_TypeDef* _SPICSPort;
  uint16_t _SPICSPin;
  SPI_HandleTypeDef* _spi;

  uint32_t _ID;
  uint32_t _BaseID;
  uint16_t _SetingsID;
  uint32_t _SampleCount=0;
  float _NominalSamplingFreq=-1;

  // PLATFORM functions and pointer for ST Libs
  // this pointer as handle is needed since the meberfunctions Met4FoFLsm6dsrx::platform_write and Met4FoFLsm6dsrx::platform_read expect an pointer to their instances as first argument
  // since the platform Functions are called this way :
  // ctx->read_reg(ctx->handle, reg, data, len);
  // we can directly use ctx->handle for the instance pointer (this)

  int32_t platform_write(uint8_t reg, const uint8_t *bufp,uint16_t len);
  int32_t platform_read(uint8_t reg, uint8_t *bufp,uint16_t len);

  stmdev_ctx_t _dev_ctx={(stmdev_write_ptr) &Met4FoFLsm6dsrx::platform_write,
		  	  	  	  	 (stmdev_read_ptr) &Met4FoFLsm6dsrx::platform_read,
						 (stmdev_mdelay_ptr) NULL,
						 this};

  //_dev_ctx.write_reg =(stmdev_write_ptr) &Met4FoFLsm6dsrx::platform_write;
  //_dev_ctx.read_reg =(stmdev_read_ptr) &Met4FoFLsm6dsrx::platform_read;
  //_dev_ctx.handle = this;

  uint8_t _connectionInitRetys=10;
  static const uint8_t _connectionInitDelayMs=10;
  uint8_t _whoamI,_rst;
  // Internal register configurations
  lsm6dsrx_pin_int1_route_t _int1_route;
  lsm6dsrx_pin_int2_route_t _int2_route;
  Met4FoFLsm6dsrx::outPutDatarate _odr=ODR_6667Hz;
  lsm6dsrx_fs_xl_t _accFullScaleSet=LSM6DSRX_16g;
  float _ACCFSScaleFactor=0.488f;
  lsm6dsrx_fs_g_t _gyroFullScaleSet=LSM6DSRX_4000dps;
  float _GyroFSScaleFactor=140.0f;
  float _TempFSScaleFactor=1.0/256.0f;
  float _TempScaleOffset=25.0f;

};



#endif /* MET4FOFLSM6DSRX_H_ */
