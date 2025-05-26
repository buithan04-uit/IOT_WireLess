################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CircularBuffer.c \
../Core/Src/DHT.c \
../Core/Src/MAX30100.c \
../Core/Src/MAX30100_BeatDetector.c \
../Core/Src/MAX30100_PulseOximeter.c \
../Core/Src/MAX30100_SpO2Calculator.c \
../Core/Src/MY_DHT22.c \
../Core/Src/PMS.c \
../Core/Src/delay.c \
../Core/Src/fonts.c \
../Core/Src/gps.c \
../Core/Src/ili9341.c \
../Core/Src/main.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/CircularBuffer.o \
./Core/Src/DHT.o \
./Core/Src/MAX30100.o \
./Core/Src/MAX30100_BeatDetector.o \
./Core/Src/MAX30100_PulseOximeter.o \
./Core/Src/MAX30100_SpO2Calculator.o \
./Core/Src/MY_DHT22.o \
./Core/Src/PMS.o \
./Core/Src/delay.o \
./Core/Src/fonts.o \
./Core/Src/gps.o \
./Core/Src/ili9341.o \
./Core/Src/main.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/CircularBuffer.d \
./Core/Src/DHT.d \
./Core/Src/MAX30100.d \
./Core/Src/MAX30100_BeatDetector.d \
./Core/Src/MAX30100_PulseOximeter.d \
./Core/Src/MAX30100_SpO2Calculator.d \
./Core/Src/MY_DHT22.d \
./Core/Src/PMS.d \
./Core/Src/delay.d \
./Core/Src/fonts.d \
./Core/Src/gps.d \
./Core/Src/ili9341.d \
./Core/Src/main.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CircularBuffer.cyclo ./Core/Src/CircularBuffer.d ./Core/Src/CircularBuffer.o ./Core/Src/CircularBuffer.su ./Core/Src/DHT.cyclo ./Core/Src/DHT.d ./Core/Src/DHT.o ./Core/Src/DHT.su ./Core/Src/MAX30100.cyclo ./Core/Src/MAX30100.d ./Core/Src/MAX30100.o ./Core/Src/MAX30100.su ./Core/Src/MAX30100_BeatDetector.cyclo ./Core/Src/MAX30100_BeatDetector.d ./Core/Src/MAX30100_BeatDetector.o ./Core/Src/MAX30100_BeatDetector.su ./Core/Src/MAX30100_PulseOximeter.cyclo ./Core/Src/MAX30100_PulseOximeter.d ./Core/Src/MAX30100_PulseOximeter.o ./Core/Src/MAX30100_PulseOximeter.su ./Core/Src/MAX30100_SpO2Calculator.cyclo ./Core/Src/MAX30100_SpO2Calculator.d ./Core/Src/MAX30100_SpO2Calculator.o ./Core/Src/MAX30100_SpO2Calculator.su ./Core/Src/MY_DHT22.cyclo ./Core/Src/MY_DHT22.d ./Core/Src/MY_DHT22.o ./Core/Src/MY_DHT22.su ./Core/Src/PMS.cyclo ./Core/Src/PMS.d ./Core/Src/PMS.o ./Core/Src/PMS.su ./Core/Src/delay.cyclo ./Core/Src/delay.d ./Core/Src/delay.o ./Core/Src/delay.su ./Core/Src/fonts.cyclo ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/gps.cyclo ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/ili9341.cyclo ./Core/Src/ili9341.d ./Core/Src/ili9341.o ./Core/Src/ili9341.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src

