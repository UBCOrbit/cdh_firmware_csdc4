################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/A_Comparison.c \
../Src/EPS.c \
../Src/comms.c \
../Src/main.c \
../Src/payload.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/A_Comparison.o \
./Src/EPS.o \
./Src/comms.o \
./Src/main.o \
./Src/payload.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/A_Comparison.d \
./Src/EPS.d \
./Src/comms.d \
./Src/main.d \
./Src/payload.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F401xE -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/csdc_4/csdc_final_demo/cdh_firmware/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/csdc_4/csdc_final_demo/cdh_firmware/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/csdc_4/csdc_final_demo/cdh_firmware/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/csdc_4/csdc_final_demo/cdh_firmware/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Andrada Zoltan/Desktop/UBCOrbit/CDH/csdc_4/csdc_final_demo/cdh_firmware/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


