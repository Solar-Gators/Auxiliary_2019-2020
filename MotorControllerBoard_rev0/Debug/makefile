################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subsystem-can-driver/helper-code/subdir.mk
-include Core/Src/subsystem-can-driver/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MotorControllerBoard_rev0.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MotorControllerBoard_rev0.list \

OBJCOPY_BIN += \
MotorControllerBoard_rev0.bin \


# All Target
all: MotorControllerBoard_rev0.elf secondary-outputs

dependents:
	-cd C:/Users/m37/Google\ Drive/Solar\ Gators/Auxiliary_2019-2020/MotorControllerBoard_rev0/Release && $(MAKE) clean all

# Tool invocations
MotorControllerBoard_rev0.elf: $(OBJS) $(USER_OBJS) C:/Users/m37/Google\ Drive/Solar\ Gators/Auxiliary_2019-2020/MotorControllerBoard_rev0/Release/MotorControllerBoard_rev0.elf C:\Users\m37\Google\ Drive\Solar\ Gators\Auxiliary_2019-2020\MotorControllerBoard_rev0\STM32F072RBTX_FLASH.ld
	arm-none-eabi-g++ -o "MotorControllerBoard_rev0.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\m37\Google Drive\Solar Gators\Auxiliary_2019-2020\MotorControllerBoard_rev0\STM32F072RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MotorControllerBoard_rev0.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MotorControllerBoard_rev0.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MotorControllerBoard_rev0.list"
	@echo 'Finished building: $@'
	@echo ' '

MotorControllerBoard_rev0.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MotorControllerBoard_rev0.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:
C:/Users/m37/Google\ Drive/Solar\ Gators/Auxiliary_2019-2020/MotorControllerBoard_rev0/Release/MotorControllerBoard_rev0.elf:

-include ../makefile.targets
