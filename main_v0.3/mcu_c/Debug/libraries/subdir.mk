################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Andrada\ Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/src/comms.c \
C:/Users/Andrada\ Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/src/eps.c \
C:/Users/Andrada\ Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/src/mram.c \
C:/Users/Andrada\ Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/src/payload.c 

OBJS += \
./libraries/comms.o \
./libraries/eps.o \
./libraries/mram.o \
./libraries/payload.o 

C_DEPS += \
./libraries/comms.d \
./libraries/eps.d \
./libraries/mram.d \
./libraries/payload.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/comms.o: C:/Users/Andrada\ Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/src/comms.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/CMSIS/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

libraries/eps.o: C:/Users/Andrada\ Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/src/eps.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/CMSIS/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

libraries/mram.o: C:/Users/Andrada\ Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/src/mram.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/CMSIS/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

libraries/payload.o: C:/Users/Andrada\ Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/src/payload.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_c/Drivers/CMSIS/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/libraries/inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


