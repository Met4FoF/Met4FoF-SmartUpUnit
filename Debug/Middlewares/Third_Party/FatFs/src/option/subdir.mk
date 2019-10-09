################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/option/%.o: ../Middlewares/Third_Party/FatFs/src/option/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F767xx -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Inc" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/protobuff_deps" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/system" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FatFs/src" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/netif" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/posix" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Middlewares/Third_Party/LwIP/system/arch" -I"/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


