################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/subsystem-can-driver/aux-data-module.cpp \
../Core/Src/subsystem-can-driver/mitsuba-driver-data-module.cpp \
../Core/Src/subsystem-can-driver/orion-data-module.cpp \
../Core/Src/subsystem-can-driver/proton1-data-module.cpp \
../Core/Src/subsystem-can-driver/subsystem-data-module.cpp 

OBJS += \
./Core/Src/subsystem-can-driver/aux-data-module.o \
./Core/Src/subsystem-can-driver/mitsuba-driver-data-module.o \
./Core/Src/subsystem-can-driver/orion-data-module.o \
./Core/Src/subsystem-can-driver/proton1-data-module.o \
./Core/Src/subsystem-can-driver/subsystem-data-module.o 

CPP_DEPS += \
./Core/Src/subsystem-can-driver/aux-data-module.d \
./Core/Src/subsystem-can-driver/mitsuba-driver-data-module.d \
./Core/Src/subsystem-can-driver/orion-data-module.d \
./Core/Src/subsystem-can-driver/proton1-data-module.d \
./Core/Src/subsystem-can-driver/subsystem-data-module.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/subsystem-can-driver/aux-data-module.o: ../Core/Src/subsystem-can-driver/aux-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Mark/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev3/Core/Src/subsystem-can-driver" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/subsystem-can-driver/aux-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/subsystem-can-driver/mitsuba-driver-data-module.o: ../Core/Src/subsystem-can-driver/mitsuba-driver-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Mark/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev3/Core/Src/subsystem-can-driver" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/subsystem-can-driver/mitsuba-driver-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/subsystem-can-driver/orion-data-module.o: ../Core/Src/subsystem-can-driver/orion-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Mark/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev3/Core/Src/subsystem-can-driver" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/subsystem-can-driver/orion-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/subsystem-can-driver/proton1-data-module.o: ../Core/Src/subsystem-can-driver/proton1-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Mark/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev3/Core/Src/subsystem-can-driver" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/subsystem-can-driver/proton1-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/subsystem-can-driver/subsystem-data-module.o: ../Core/Src/subsystem-can-driver/subsystem-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Mark/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev3/Core/Src/subsystem-can-driver" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/subsystem-can-driver/subsystem-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

