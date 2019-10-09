################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/NMEAPraser.c \
../Src/SEGGER_RTT.c \
../Src/SEGGER_RTT_printf.c \
../Src/SEGGER_SYSVIEW.c \
../Src/SEGGER_SYSVIEW_Config_FreeRTOS.c \
../Src/SEGGER_SYSVIEW_FreeRTOS.c \
../Src/adc.c \
../Src/backupsram.c \
../Src/bsp_driver_sd.c \
../Src/can.c \
../Src/dac.c \
../Src/dma.c \
../Src/ethernetif.c \
../Src/fatfs.c \
../Src/gpio.c \
../Src/httpserver-netconn.c \
../Src/i2c.c \
../Src/lwip.c \
../Src/rng.c \
../Src/rtc.c \
../Src/sd_diskio.c \
../Src/sdmmc.c \
../Src/spi.c \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_hal_timebase_tim.c \
../Src/stm32f7xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f7xx.c \
../Src/tim.c \
../Src/tim64extender.c \
../Src/usart.c \
../Src/usb_otg.c 

CPP_SRCS += \
../Src/GPSTimesyn.cpp \
../Src/MPU9250.cpp \
../Src/configmanager.cpp \
../Src/freertos.cpp \
../Src/main.cpp 

OBJS += \
./Src/GPSTimesyn.o \
./Src/MPU9250.o \
./Src/NMEAPraser.o \
./Src/SEGGER_RTT.o \
./Src/SEGGER_RTT_printf.o \
./Src/SEGGER_SYSVIEW.o \
./Src/SEGGER_SYSVIEW_Config_FreeRTOS.o \
./Src/SEGGER_SYSVIEW_FreeRTOS.o \
./Src/adc.o \
./Src/backupsram.o \
./Src/bsp_driver_sd.o \
./Src/can.o \
./Src/configmanager.o \
./Src/dac.o \
./Src/dma.o \
./Src/ethernetif.o \
./Src/fatfs.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/httpserver-netconn.o \
./Src/i2c.o \
./Src/lwip.o \
./Src/main.o \
./Src/rng.o \
./Src/rtc.o \
./Src/sd_diskio.o \
./Src/sdmmc.o \
./Src/spi.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_hal_timebase_tim.o \
./Src/stm32f7xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f7xx.o \
./Src/tim.o \
./Src/tim64extender.o \
./Src/usart.o \
./Src/usb_otg.o 

C_DEPS += \
./Src/NMEAPraser.d \
./Src/SEGGER_RTT.d \
./Src/SEGGER_RTT_printf.d \
./Src/SEGGER_SYSVIEW.d \
./Src/SEGGER_SYSVIEW_Config_FreeRTOS.d \
./Src/SEGGER_SYSVIEW_FreeRTOS.d \
./Src/adc.d \
./Src/backupsram.d \
./Src/bsp_driver_sd.d \
./Src/can.d \
./Src/dac.d \
./Src/dma.d \
./Src/ethernetif.d \
./Src/fatfs.d \
./Src/gpio.d \
./Src/httpserver-netconn.d \
./Src/i2c.d \
./Src/lwip.d \
./Src/rng.d \
./Src/rtc.d \
./Src/sd_diskio.d \
./Src/sdmmc.d \
./Src/spi.d \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_hal_timebase_tim.d \
./Src/stm32f7xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f7xx.d \
./Src/tim.d \
./Src/tim64extender.d \
./Src/usart.d \
./Src/usb_otg.d 

CPP_DEPS += \
./Src/GPSTimesyn.d \
./Src/MPU9250.d \
./Src/configmanager.d \
./Src/freertos.d \
./Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F767xx -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Inc" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/protobuff_deps" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/system" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FatFs/src" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/netif" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/posix" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/system/arch" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F767xx -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Inc" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/protobuff_deps" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/system" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FatFs/src" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/netif" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/posix" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/system/arch" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


