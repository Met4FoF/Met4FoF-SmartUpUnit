/*
 * dma_circular.h
 *
 *  Created on: 28.11.2018
 *      Author: seeger01
 */

#ifndef DMA_CIRCULAR_H_
#define DMA_CIRCULAR_H_
#include "stm32f7xx_hal.h"

void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

void DMA_IrqHandler (DMA_HandleTypeDef *hdma);




#endif /* DMA_CIRCULAR_H_ */
