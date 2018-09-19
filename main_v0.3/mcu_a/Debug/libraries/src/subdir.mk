################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/src/comms.c \
../libraries/src/eps.c \
../libraries/src/mram.c \
../libraries/src/payload.c 

OBJS += \
./libraries/src/comms.o \
./libraries/src/eps.o \
./libraries/src/mram.o \
./libraries/src/payload.o 

C_DEPS += \
./libraries/src/comms.d \
./libraries/src/eps.d \
./libraries/src/mram.d \
./libraries/src/payload.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/src/%.o: ../libraries/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_a/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_a/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_a/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_a/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_a/Drivers/CMSIS/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/main_v0.3/mcu_a/libraries/inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


