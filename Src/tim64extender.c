/*
 * tim64extender.c
 *
 *  Created on: 30.07.2019
 *      Author: seeger01
 */

/* Includes ------------------------------------------------------------------*/
#include "tim64extender.h"
//Top 32 bit for timer2 inputcapture values
static uint64_t tim2_update_counts=0;
static uint64_t tim2_upper_bits_mask=0;
//Top 48 bit for timer2 inputcapture values
static uint64_t tim1_update_counts=0;
static uint64_t tim1_upper_bits_mask=0;

uint64_t TIM_Get_64Bit_TimeStamp_Base(TIM_HandleTypeDef * htim){
	uint64_t timestamp=0;
	// RACE CONDITION CECKING !!
	// this code occures as well in  void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim) in stm32f7xx_hal_tim.c
	// this function is called every time an Timer generates an interrupt event and adds it do the Nested Vectored Interrupt Controller (NVIC)
	// the timer generates an global interrupt(IRQ#44 TIM2_IRQHandler) witch is always the same regardless the reason for the the interrupt (input capture, output compare, upadte)
	// so the HAL_TIM_IRQHandler() is called to determin the source of the ISR request, there fore the startus registers of the timer are read.
	// in that order:
	// /* Capture compare 1 event */
	// /* Capture compare 2 event */
	// /* Capture compare 3 event */
	// /* Capture compare 4 event */
	// /* TIM Update event */
	// ...
	// if an update and an inputcapure event occures while jumping or processing  in the HAL_TIM_IRQHandler() then the inputcapure event will be
	// procesed before the update event and the tim2_upper_bits_mask is not set right so we have to check the timer values and decive if they are from befor or after the overflow.
	uint64_t tim2_upper_bits_mask_race_condition = 0;
	#define TIM32OLDTIMERVALMIN 0xFF000000 // if an inputcaputure value is biger than this its prppably an old one
	#define TIM16OLDTIMERVALMIN 0xF000 // if an inputcaputure value is biger than this its prppably an old one
	bool tim2_race_condition_up_date = false;
	if (htim->Instance == TIM2) {
		if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
			if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
				__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
				//the flag gets cleared to prevent HAL_TIM_PeriodElapsedCallback() calling and therfore double increasment
				//DEBUG_MSG("WARNING!!! TIMER OVERFLOW DETECTED OUTSIDE OF UPDATEEVENTHANDLER\n START SPECIAL HANDLING CHECK RESULTS OF THIS MESURMENT CYCLE");
				tim2_race_condition_up_date = true;
				tim2_upper_bits_mask_race_condition = tim2_upper_bits_mask;
				tim2_update_counts++;
				tim2_upper_bits_mask = (uint64_t)(tim2_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
			}
		}
	}
	uint64_t tim1_upper_bits_mask_race_condition = 0;
	bool tim1_race_condition_up_date = false;
	if (htim->Instance == TIM1) {
		if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
			if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
				__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
				//the flag gets cleared to prevent HAL_TIM_PeriodElapsedCallback() calling and therfore double increasment
				//DEBUG_MSG("WARNING!!! TIMER OVERFLOW DETECTED OUTSIDE OF UPDATEEVENTHANDLER\n START SPECIAL HANDLING CHECK RESULTS OF THIS MESURMENT CYCLE");
				tim1_race_condition_up_date = true;
				tim1_upper_bits_mask_race_condition = tim1_upper_bits_mask;
				tim1_update_counts++;
				tim1_upper_bits_mask = (uint64_t)(tim1_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
			}
		}
	}
	if (htim->Instance == TIM1) {
				if (tim1_race_condition_up_date == false) {
					//this is the nromal case
					timestamp = tim1_upper_bits_mask
							+ (uint64_t) __HAL_TIM_GetCounter(&htim1);
				} else {
					uint32_t timestamp_raw = __HAL_TIM_GetCounter(&htim1);
					if (timestamp_raw < TIM32OLDTIMERVALMIN)
					//the timer has overflowen tak the updateted bitmask
					{
						timestamp = tim1_upper_bits_mask + (uint64_t) timestamp_raw;
					} else {
						//this is an old value using the old bitmask
						timestamp = tim1_upper_bits_mask_race_condition
								+ (uint64_t) timestamp_raw;
					}
				}

			}

	if (htim->Instance == TIM2) {
				if (tim2_race_condition_up_date == false) {
					//this is the nromal case
					timestamp = tim2_upper_bits_mask
							+ (uint64_t) __HAL_TIM_GetCounter(&htim2);
				} else {
					uint32_t timestamp_raw = __HAL_TIM_GetCounter(&htim2);
					if (timestamp_raw < TIM32OLDTIMERVALMIN)
					//the timer has overflowen tak the updateted bitmask
					{
						timestamp = tim2_upper_bits_mask + (uint64_t) timestamp_raw;
					} else {
						//this is an old value using the old bitmask
						timestamp = tim2_upper_bits_mask_race_condition
								+ (uint64_t) timestamp_raw;
					}
				}

			}
	return timestamp;
}

uint64_t TIM_Get_64Bit_TimeStamp_IC(TIM_HandleTypeDef * htim){

	//TODO add channel mapping
		uint64_t timestamp=0;
		// RACE CONDITION CECKING !!
		// this code occures as well in  void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim) in stm32f7xx_hal_tim.c
		// this function is called every time an Timer generates an interrupt event and adds it do the Nested Vectored Interrupt Controller (NVIC)
		// the timer generates an global interrupt(IRQ#44 TIM2_IRQHandler) witch is always the same regardless the reason for the the interrupt (input capture, output compare, upadte)
		// so the HAL_TIM_IRQHandler() is called to determin the source of the ISR request, there fore the startus registers of the timer are read.
		// in that order:
		// /* Capture compare 1 event */
		// /* Capture compare 2 event */
		// /* Capture compare 3 event */
		// /* Capture compare 4 event */
		// /* TIM Update event */
		// ...
		// if an update and an inputcapure event occures while jumping or processing  in the HAL_TIM_IRQHandler() then the inputcapure event will be
		// procesed before the update event and the tim2_upper_bits_mask is not set right so we have to check the timer values and decive if they are from befor or after the overflow.
		uint64_t tim2_upper_bits_mask_race_condition = 0;
		#define TIM32OLDTIMERVALMIN 0xFF000000 // if an inputcaputure value is biger than this its prppably an old one
		#define TIM16OLDTIMERVALMIN 0xF000 // if an inputcaputure value is biger than this its prppably an old one
		bool tim2_race_condition_up_date = false;
		if (htim->Instance == TIM2) {
			if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
				if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
					__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
					//the flag gets cleared to prevent HAL_TIM_PeriodElapsedCallback() calling and therfore double increasment
					//DEBUG_MSG("WARNING!!! TIMER OVERFLOW DETECTED OUTSIDE OF UPDATEEVENTHANDLER\n START SPECIAL HANDLING CHECK RESULTS OF THIS MESURMENT CYCLE");
					tim2_race_condition_up_date = true;
					tim2_upper_bits_mask_race_condition = tim2_upper_bits_mask;
					tim2_update_counts++;
					tim2_upper_bits_mask = (uint64_t)(tim2_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
				}
			}
		}
		uint64_t tim1_upper_bits_mask_race_condition = 0;
		bool tim1_race_condition_up_date = false;
		if (htim->Instance == TIM1) {
			if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
				if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
					__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
					//the flag gets cleared to prevent HAL_TIM_PeriodElapsedCallback() calling and therfore double increasment
					//DEBUG_MSG("WARNING!!! TIMER OVERFLOW DETECTED OUTSIDE OF UPDATEEVENTHANDLER\n START SPECIAL HANDLING CHECK RESULTS OF THIS MESURMENT CYCLE");
					tim1_race_condition_up_date = true;
					tim1_upper_bits_mask_race_condition = tim1_upper_bits_mask;
					tim1_update_counts++;
					tim1_upper_bits_mask = (uint64_t)(tim1_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
				}
			}
		}
		if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (tim2_race_condition_up_date == false) {
				//this is the nromal case
				timestamp = tim2_upper_bits_mask
						+ (uint64_t) HAL_TIM_ReadCapturedValue(&htim2,
								TIM_CHANNEL_1);
			} else {
				uint32_t timestamp_raw = HAL_TIM_ReadCapturedValue(&htim2,
				TIM_CHANNEL_1);
				if (timestamp_raw < TIM32OLDTIMERVALMIN)
				//the timer has overflowen tak the updateted bitmask
				{
					timestamp = tim2_upper_bits_mask + (uint64_t) timestamp_raw;
				} else {
					//this is an old value using the old bitmask
					timestamp = tim2_upper_bits_mask_race_condition
							+ (uint64_t) timestamp_raw;
				}
			}
		}
		if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (tim2_race_condition_up_date == false) {
				//this is the nromal case
				timestamp = tim2_upper_bits_mask
						+ (uint64_t) HAL_TIM_ReadCapturedValue(&htim2,
								TIM_CHANNEL_3);
			} else {
				uint32_t timestamp_raw = HAL_TIM_ReadCapturedValue(&htim2,
				TIM_CHANNEL_3);
				if (timestamp_raw < TIM32OLDTIMERVALMIN)
				//the timer has overflowen tak the updateted bitmask
				{
					timestamp = tim2_upper_bits_mask + (uint64_t) timestamp_raw;
				} else {
					//this is an old value using the old bitmask
					timestamp = tim2_upper_bits_mask_race_condition
							+ (uint64_t) timestamp_raw;
				}
			}

		}
		if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			if (tim2_race_condition_up_date == false) {
				//this is the nromal case
				timestamp = tim2_upper_bits_mask
						+ (uint64_t) HAL_TIM_ReadCapturedValue(&htim2,
								TIM_CHANNEL_4);
			} else {
				uint32_t timestamp_raw = HAL_TIM_ReadCapturedValue(&htim2,
				TIM_CHANNEL_4);
				if (timestamp_raw < TIM32OLDTIMERVALMIN)
				//the timer has overflowen tak the updateted bitmask
				{
					timestamp = tim2_upper_bits_mask + (uint64_t) timestamp_raw;
				} else {
					//this is an old value using the old bitmask
					timestamp = tim2_upper_bits_mask_race_condition
							+ (uint64_t) timestamp_raw;
				}
			}
		}
		if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (tim1_race_condition_up_date == false) {
				//this is the nromal case
				timestamp = tim1_upper_bits_mask
						+ (uint64_t) HAL_TIM_ReadCapturedValue(&htim1,
								TIM_CHANNEL_1);
			} else {
				uint32_t timestamp_raw = HAL_TIM_ReadCapturedValue(&htim1,
				TIM_CHANNEL_1);
				if (timestamp_raw < TIM32OLDTIMERVALMIN)
				//the timer has overflowen tak the updateted bitmask
				{
					timestamp = tim1_upper_bits_mask + (uint64_t) timestamp_raw;
				} else {
					//this is an old value using the old bitmask
					timestamp = tim1_upper_bits_mask_race_condition
							+ (uint64_t) timestamp_raw;
				}
			}

		}
		return timestamp;
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM14) {
		HAL_IncTick();
	}
	if (htim->Instance == TIM2) {
			tim2_update_counts++;
			tim2_upper_bits_mask = (uint64_t)(tim2_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1

		}
		if (htim->Instance == TIM1) {
			{
				tim1_update_counts++;
				tim1_upper_bits_mask = (uint64_t)(tim1_update_counts - 1) << 16;//timer gets initaled with set upodateflag but we want to start at zero therfore -1

			}
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}
