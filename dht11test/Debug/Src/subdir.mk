################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/dwt_stm32_delay.c \
../Src/main.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f3xx.c 

OBJS += \
./Src/dwt_stm32_delay.o \
./Src/main.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f3xx.o 

C_DEPS += \
./Src/dwt_stm32_delay.d \
./Src/main.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F303xE '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/home/wawa6609/Desktop/dht11test/Inc" -I"/home/wawa6609/Desktop/dht11test/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/home/wawa6609/Desktop/dht11test/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/home/wawa6609/Desktop/dht11test/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/home/wawa6609/Desktop/dht11test/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


