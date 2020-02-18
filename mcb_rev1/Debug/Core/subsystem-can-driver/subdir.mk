################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/subsystem-can-driver/aux-data-module.cpp \
../Core/subsystem-can-driver/bms-data-module.cpp \
../Core/subsystem-can-driver/mppt-data-module.cpp \
../Core/subsystem-can-driver/subsystem-data-module.cpp 

OBJS += \
./Core/subsystem-can-driver/aux-data-module.o \
./Core/subsystem-can-driver/bms-data-module.o \
./Core/subsystem-can-driver/mppt-data-module.o \
./Core/subsystem-can-driver/subsystem-data-module.o 

CPP_DEPS += \
./Core/subsystem-can-driver/aux-data-module.d \
./Core/subsystem-can-driver/bms-data-module.d \
./Core/subsystem-can-driver/mppt-data-module.d \
./Core/subsystem-can-driver/subsystem-data-module.d 


# Each subdirectory must supply rules for building sources it contributes
Core/subsystem-can-driver/aux-data-module.o: ../Core/subsystem-can-driver/aux-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Drivers/STM32F0xx_HAL_Driver/Inc -I"C:/Users/m37/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev1/Core/subsystem-can-driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/subsystem-can-driver/aux-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/subsystem-can-driver/bms-data-module.o: ../Core/subsystem-can-driver/bms-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Drivers/STM32F0xx_HAL_Driver/Inc -I"C:/Users/m37/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev1/Core/subsystem-can-driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/subsystem-can-driver/bms-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/subsystem-can-driver/mppt-data-module.o: ../Core/subsystem-can-driver/mppt-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Drivers/STM32F0xx_HAL_Driver/Inc -I"C:/Users/m37/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev1/Core/subsystem-can-driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/subsystem-can-driver/mppt-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/subsystem-can-driver/subsystem-data-module.o: ../Core/subsystem-can-driver/subsystem-data-module.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Drivers/STM32F0xx_HAL_Driver/Inc -I"C:/Users/m37/Google Drive/Solar Gators/Auxiliary_2019-2020/mcb_rev1/Core/subsystem-can-driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/subsystem-can-driver/subsystem-data-module.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

