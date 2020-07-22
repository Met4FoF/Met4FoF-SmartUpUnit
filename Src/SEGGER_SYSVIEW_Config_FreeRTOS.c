/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 2015 - 2017  SEGGER Microcontroller GmbH & Co. KG        *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the RTT protocol and J-Link.                       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* conditions are met:                                                *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this list of conditions and the following disclaimer.    *
*                                                                    *
* o Redistributions in binary form must reproduce the above          *
*   copyright notice, this list of conditions and the following      *
*   disclaimer in the documentation and/or other materials provided  *
*   with the distribution.                                           *
*                                                                    *
* o Neither the name of SEGGER Microcontroller GmbH & Co. KG         *
*   nor the names of its contributors may be used to endorse or      *
*   promote products derived from this software without specific     *
*   prior written permission.                                        *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       SystemView version: V2.52d                                    *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Config_FreeRTOS.c
Purpose : Sample setup configuration of SystemView with FreeRTOS.
Revision: $Rev: 7745 $
*/
#include "FreeRTOS.h"
#include "SEGGER_SYSVIEW.h"

extern const SEGGER_SYSVIEW_OS_API SYSVIEW_X_OS_TraceAPI;

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/
// The application name to be displayed in SystemViewer
#define SYSVIEW_APP_NAME        "Met4FoF-SUU"

// The target device name
#define SYSVIEW_DEVICE_NAME     "Cortex-M7"

// Frequency of the timestamp. Must match SEGGER_SYSVIEW_GET_TIMESTAMP in SEGGER_SYSVIEW_Conf.h
#define SYSVIEW_TIMESTAMP_FREQ  (configCPU_CLOCK_HZ)

// System Frequency. SystemcoreClock is used in most CMSIS compatible projects.
#define SYSVIEW_CPU_FREQ        configCPU_CLOCK_HZ

// The lowest RAM address used for IDs (pointers)
#define SYSVIEW_RAM_BASE        (0x10000000)

/********************************************************************* 
*
*       _cbSendSystemDesc()
*
*  Function description
*    Sends SystemView description strings.
*/
static void _cbSendSystemDesc(void) {
  SEGGER_SYSVIEW_SendSysDesc("N="SYSVIEW_APP_NAME",D="SYSVIEW_DEVICE_NAME",O=FreeRTOS");
  /*
SEGGER_SYSVIEW_SendSysDesc("I#0=_estack");
SEGGER_SYSVIEW_SendSysDesc("I#1=Reset_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#2=NMI_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#3=HardFault_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#4=MemManage_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#5=BusFault_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#6=UsageFault_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#7=0");
SEGGER_SYSVIEW_SendSysDesc("I#8=0");
SEGGER_SYSVIEW_SendSysDesc("I#9=0");
SEGGER_SYSVIEW_SendSysDesc("I#10=0");
SEGGER_SYSVIEW_SendSysDesc("I#11=SVC_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#12=DebugMon_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#13=0");
SEGGER_SYSVIEW_SendSysDesc("I#14=PendSV_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#15=SysTick_Handler");
SEGGER_SYSVIEW_SendSysDesc("I#16=WWDG_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#17=PVD_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#18=TAMP_STAMP_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#19=RTC_WKUP_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#20=FLASH_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#21=RCC_IRQHandler");

SEGGER_SYSVIEW_SendSysDesc("I#22=EXTI0_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#23=EXTI1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#24=EXTI2_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#25=EXTI3_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#26=EXTI4_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#27=DMA1_Stream0_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#28=DMA1_Stream1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#29=DMA1_Stream2_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#30=DMA1_Stream3_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#31=DMA1_Stream4_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#32=DMA1_Stream5_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#33=DMA1_Stream6_IRQHandler");
*/
SEGGER_SYSVIEW_SendSysDesc("I#34=ADC_IRQHandler");
/*
SEGGER_SYSVIEW_SendSysDesc("I#35=CAN1_TX_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#36=CAN1_RX0_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#37=CAN1_RX1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#38=CAN1_SCE_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#39=EXTI9_5_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#40=TIM1_BRK_TIM9_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#41=TIM1_UP_TIM10_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#42=TIM1_TRG_COM_TIM11_IRQHandler");
*/
SEGGER_SYSVIEW_SendSysDesc("I#43=TIM1_CC_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#44=TIM2_IRQHandler");

SEGGER_SYSVIEW_SendSysDesc("I#45=TIM3_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#46=TIM4_IRQHandler");
/*
SEGGER_SYSVIEW_SendSysDesc("I#47=I2C1_EV_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#48=I2C1_ER_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#49=I2C2_EV_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#50=I2C2_ER_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#51=SPI1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#52=SPI2_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#53=USART1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#54=USART2_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#55=USART3_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#56=EXTI15_10_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#57=RTC_Alarm_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#58=OTG_FS_WKUP_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#59=TIM8_BRK_TIM12_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#60=TIM8_UP_TIM13_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#61=TIM8_TRG_COM_TIM14_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#62=TIM8_CC_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#63=DMA1_Stream7_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#64=FMC_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#65=SDMMC1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#66=TIM5_IRQHandler");

SEGGER_SYSVIEW_SendSysDesc("I#67=SPI3_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#68=UART4_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#69=UART5_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#70=TIM6_DAC_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#71=TIM7_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#72=DMA2_Stream0_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#73=DMA2_Stream1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#74=DMA2_Stream2_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#75=DMA2_Stream3_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#76=DMA2_Stream4_IRQHandler");
/*
SEGGER_SYSVIEW_SendSysDesc("I#77=ETH_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#78=ETH_WKUP_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#79=CAN2_TX_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#80=CAN2_RX0_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#81=CAN2_RX1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#82=CAN2_SCE_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#83=OTG_FS_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#84=DMA2_Stream5_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#85=DMA2_Stream6_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#86=DMA2_Stream7_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#87=USART6_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#88=I2C3_EV_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#89=I2C3_ER_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#90=OTG_HS_EP1_OUT_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#91=OTG_HS_EP1_IN_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#92=OTG_HS_WKUP_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#93=OTG_HS_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#94=DCMI_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#95=0");
SEGGER_SYSVIEW_SendSysDesc("I#96=RNG_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#97=FPU_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#98=UART7_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#99=UART8_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#100=SPI4_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#101=SPI5_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#102=SPI6_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#103=SAI1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#104=LTDC_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#105=LTDC_ER_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#106=DMA2D_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#107=SAI2_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#108=QUADSPI_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#109=LPTIM1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#110=CEC_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#111=I2C4_EV_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#112=I2C4_ER_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#113=SPDIF_RX_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#114=0");
SEGGER_SYSVIEW_SendSysDesc("I#115=DFSDM1_FLT0_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#116=DFSDM1_FLT1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#117=DFSDM1_FLT2_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#118=DFSDM1_FLT3_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#119=SDMMC2_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#120=CAN3_TX_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#121=CAN3_RX0_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#122=CAN3_RX1_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#123=CAN3_SCE_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#124=JPEG_IRQHandler");
SEGGER_SYSVIEW_SendSysDesc("I#125=MDIOS_IRQHandler");
*/
}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
void SEGGER_SYSVIEW_Conf(void) {
  SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ, 
                      &SYSVIEW_X_OS_TraceAPI, _cbSendSystemDesc);
  SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);
}

/*************************** End of file ****************************/
