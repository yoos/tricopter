################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/subdir.mk
-include libraries/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Tricopter-eclipse.lss \

FLASH_IMAGE += \
Tricopter-eclipse.hex \

EEPROM_IMAGE += \
Tricopter-eclipse.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Tricopter-eclipse.elf secondary-outputs

dependents:
	-cd /home/ben/workspace/DuemilanoveCore/Release && $(MAKE) all

# Tool invocations
Tricopter-eclipse.elf: $(OBJS) $(USER_OBJS) /home/ben/workspace/DuemilanoveCore/Release/libDuemilanoveCore.a
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ --verbose -Wl,-Map,Tricopter-eclipse.map,--cref -lm -L"/home/ben/workspace/DuemilanoveCore/Release" -mmcu=atmega328p -o"Tricopter-eclipse.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Tricopter-eclipse.lss: Tricopter-eclipse.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Tricopter-eclipse.elf  >"Tricopter-eclipse.lss"
	@echo 'Finished building: $@'
	@echo ' '

Tricopter-eclipse.hex: Tricopter-eclipse.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex Tricopter-eclipse.elf  "Tricopter-eclipse.hex"
	@echo 'Finished building: $@'
	@echo ' '

Tricopter-eclipse.eep: Tricopter-eclipse.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Tricopter-eclipse.elf  "Tricopter-eclipse.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Tricopter-eclipse.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p Tricopter-eclipse.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EEPROM_IMAGE)$(ELFS)$(FLASH_IMAGE)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) Tricopter-eclipse.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
/home/ben/workspace/DuemilanoveCore/Release/libDuemilanoveCore.a:

-include ../makefile.targets
