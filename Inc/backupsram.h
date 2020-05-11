/*
 * backupsram.h
 *
 *  Created on: 28.06.2019
 *      Author: seeger01
 */

#ifndef BACKUPSRAM_H_
#define BACKUPSRAM_H_
#include "stm32f7xx_hal.h"

#define SIZEOFSRAMSTRINGS 40

#define STARTUPCOUNTADRESS 0
#define SUBNETMASKDEVICEIPADRESS 4
#define DEVICEIPADRESS 8
#define	UDPTARGETIPADRESS 12
#define	UDPSUBNETMASKADRESS 16
#define	UDPPORTADRESS 20
#define USEDHCPADRESS 24
#define DEVICENAMEADRESS 28
#define DEVICEOWNERADRESS 68
#define ADCCOEVSADRESS 108 //3*3*4 bytes


/**
 * Get memory size for internal backup SRAM
 *
 * Returns memory value in Bytes
 */

#define TM_BKPSRAM_GetMemorySize()            (0x00001000)
/**
 * Write 8-bit value to backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address where to save data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 1 is valid, if more, HardFault error can happen.
 * - uint8_t value:
 *         8-bit value which will be stored to backup SRAM
 *
 * No return
 *
 * Defined as macro
 */
#define BKPSRAM_Write8(address, value)    (*(__IO uint8_t *) (BKPSRAM_BASE + (address)) = (value))

/**
 * Read 8-bit value from backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address from where read data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 1 is valid, if more, HardFault error can happen.
 *
 * 8-bit value at specific location is returned
 *
 * Defined as macro
 */
#define BKPSRAM_Read8(address)            (*(__IO uint8_t *) (BKPSRAM_BASE + address))

/**
 * Write 16-bit value to backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address where to save data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 2 is valid, if more, HardFault error can happen.
 * - uint16_t value:
 *         16-bit value which will be stored to backup SRAM
 *
 * No return
 *
 * Defined as macro
 */
#define BKPSRAM_Write16(address, value)    (*(__IO uint16_t *) (BKPSRAM_BASE + (address)) = (value))

/**
 * Read 16-bit value from backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address from where read data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 2 is valid, if more, HardFault error can happen.
 *
 * 16-bit value at specific location is returned
 *
 * Defined as macro
 */
#define BKPSRAM_Read16(address)            (*(__IO uint16_t *) (BKPSRAM_BASE + address))

/**
 * Write 32-bit value to backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address where to save data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 4 is valid, if more, HardFault error can happen.
 * - uint32_t value:
 *         32-bit value which will be stored to backup SRAM
 *
 * No return
 *
 * Defined as macro
 */
#define BKPSRAM_Write32(address, value)    (*(__IO uint32_t *) (BKPSRAM_BASE + (address)) = (value))

/**
 * Read 32-bit value from backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address from where read data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 4 is valid, if more, HardFault error can happen.
 *
 * 32-bit value at specific location is returned
 *
 * Defined as macro
 */
#define BKPSRAM_Read32(address)            (*(__IO uint32_t *) (BKPSRAM_BASE + address))

/**
 * Write 32-bit float value to backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address where to save data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 4 is valid, if more, HardFault error can happen.
 * - float value:
 *         32-bit float value which will be stored to backup SRAM
 *
 * No return
 *
 * Defined as macro
 */
#define BKPSRAM_WriteFloat(address, value)    (*(__IO float *) (BKPSRAM_BASE + (address)) = (value))

/**
 * Read 32-bit float value from backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address from where read data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 4 is valid, if more, HardFault error can happen.
 *
 * 32-bit float value at specific location is returned
 *
 * Defined as macro
 */
#define BKPSRAM_ReadFloat(address)            (*(__IO float *) (BKPSRAM_BASE + address))

/**
 * Write 64-bit double value to backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address where to save data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 4 is valid, if more, HardFault error can happen.
 * - double value:
 *         64-bit double value which will be stored to backup SRAM
 *
 * No return
 *
 * Defined as macro
 */
#define BKPSRAM_WriteDouble(address, value)    (*(__IO double *) (BKPSRAM_BASE + (address)) = (value))

/**
 * Read 64-bit double value from backup SRAM at desired location
 *
 * Parameters:
 *     - uint16_t address:
 *         Address from where read data in SRAM.
 *         Value between 0 and TM_BKPSRAM_GetMemorySize() - 4 is valid, if more, HardFault error can happen.
 *
 * 64-bit double value at specific location is returned
 *
 * Defined as macro
 */
#define BKPSRAM_ReadDouble(address)            (*(__IO double *) (BKPSRAM_BASE + address))
/**
 * Read 8-bit value from Unique device ID register (96 bits)
 *
 * Parameters:
 *     - uint16_t address:
 *         Address from where read data in Unique device ID register (96 bits).
 *         Value between 0 and 11 is valid, if more, HardFault error can happen.
 *
 * 8-bit value at specific location is returned
 *
 * Defined as macro
 */
#define UDID_Read8(address)            (*(__IO uint8_t *) (0x1FF0F420 + address))

#endif /* BACKUPSRAM_H_ */
