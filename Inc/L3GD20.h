/*
 * L3GD20.h
 *
 *  Created on: 05.02.2019
 *      Author: seeger01
 *
 *BASED ON:
/***************************************************
  This is a library for the L3GD20 GYROSCOPE

  Designed specifically to work with the Adafruit L3GD20 Breakout
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C)
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef L3GD20_L3GD20_H_
#define L3GD20_L3GD20_H_

#include "spi.h"
#include "gpio.h"
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <cstring>
#include <math.h>
#include "main.h"
#include "message.pb.h"

//I2C currently not supportet
/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define L3GD20_ADDRESS           (0x6B)        // 1101011
    #define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
    #define L3GD20_ID                (0xD4)
    #define L3GD20H_ID               (0xD7)
/*=========================================================================
	SPI BITS AND SETTINGS
    -----------------------------------------------------------------------*/
	#define L3GD20_SPI_WRITE  (0x00)
	#define L3GD20_SPI_READ  (0x80)
	#define L3GD20_SPI_MULTIREADINCREMENT (0x40)
	#define SPI_TIMEOUT 100U
/*=========================================================================*/
    // Sesitivity values from the mechanical characteristics in the datasheet.
    #define GYRO_SENSITIVITY_250DPS  (0.00875F)
    #define GYRO_SENSITIVITY_500DPS  (0.0175F)
    #define GYRO_SENSITIVITY_2000DPS (0.070F)

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                             // DEFAULT    TYPE
      GYRO_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
      GYRO_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
      GYRO_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
      GYRO_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
      GYRO_REGISTER_OUT_TEMP            = 0x26,   //            r
      GYRO_REGISTER_STATUS_REG          = 0x27,   //            r
      GYRO_REGISTER_OUT_X_L             = 0x28,   //            r
      GYRO_REGISTER_OUT_X_H             = 0x29,   //            r
      GYRO_REGISTER_OUT_Y_L             = 0x2A,   //            r
      GYRO_REGISTER_OUT_Y_H             = 0x2B,   //            r
      GYRO_REGISTER_OUT_Z_L             = 0x2C,   //            r
      GYRO_REGISTER_OUT_Z_H             = 0x2D,   //            r
      GYRO_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
      GYRO_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
      GYRO_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
      GYRO_REGISTER_INT1_SRC            = 0x31,   //            r
      GYRO_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
      GYRO_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
      GYRO_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
      GYRO_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
      GYRO_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
      GYRO_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
      GYRO_REGISTER_INT1_DURATION       = 0x38,    // 00000000   rw
	  GYRO_REGISTER_LOW_ODR       = 0x39    // 00000000   rw
    } gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      GYRO_RANGE_250DPS  = 250,
      GYRO_RANGE_500DPS  = 500,
      GYRO_RANGE_2000DPS = 2000
    } gyroRange_t;

    typedef enum
    {
      GYRO_UPDATE_12_5HZ  = 0x00<<2|0x00,
	  GYRO_UPDATAE_25_HZ = 0x01<<2|0x00,
	  GYRO_UPDATE_50_HZ_CUTOFF_16_6 = 0x02<<2|0x00,
	  GYRO_UPDATE_100_HZ_CUTOFF_12_5 = 0x00<<2|0x00,
	  GYRO_UPDATE_100_HZ_CUTOFF_25 = 0x00<<2|0x01,
	  GYRO_UPDATE_200_HZ = 0x01<<2|0x01,
	  GYRO_UPDATE_200_HZ_CUTOFF_12_5 = 0x01<<2|0x00,
	  GYRO_UPDATE_200_HZ_CUTOFF_70 = 0x01<<2|0x03,
	  GYRO_UPDATE_400_HZ_CUTOFF_20 = 0x02<<2|0x00,
	  GYRO_UPDATE_400_HZ_CUTOFF_25 = 0x02<<2|0x01,
	  GYRO_UPDATE_400_HZ_CUTOFF_50 = 0x02<<2|0x02,
	  GYRO_UPDATE_400_HZ_CUTOFF_110 = 0x02<<2|0x03,
	  GYRO_UPDATE_800_HZ = 0x03<<2|0x02,
	  GYRO_UPDATE_800_HZ_CUTOFF_30 = 0x03<<2|0x00,
	  GYRO_UPDATE_800_HZ_CUTOFF_35 = 0x03<<2|0x01,
	  GYRO_UPDATE_800_HZ_CUTOFF_100 = 0x03<<2|0x03
    } gyroUpdateFreq_t;

#define DEFAULT_GYRORANGE GYRO_RANGE_2000DPS
/**
 *  L3GD20 triple axis, digital interface, gyroscope.
 */

class  L3GD20
{
  public:
  L3GD20(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,SPI_HandleTypeDef* L3GD20spi);
  bool init(gyroRange_t gyrorange,gyroUpdateFreq_t gyroUpdateFreq);
  ProtoIMUStamped  GetStampedData(uint32_t UnixSecs,uint64_t RawTimerCount,uint32_t CaptureCount,uint16_t ADCVal);
  bool GetData(float * x,float * y,float * z,float * temperature);
  void writeByte(uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t subAddress);
  bool readBytes(uint8_t subAddress, uint8_t count, uint8_t* dest);
  GPIO_TypeDef* _SPICSTypeDef;
  uint16_t _SPICSPin;
  SPI_HandleTypeDef* _L3GD20spi;
  gyroRange_t _gyrorange;
  gyroUpdateFreq_t _gyroUpdateFreq;

};

#endif /*__L3GD20_H */
