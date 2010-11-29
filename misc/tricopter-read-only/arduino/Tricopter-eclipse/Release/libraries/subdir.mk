################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../libraries/Messenger.cpp \
../libraries/Servo.cpp \
../libraries/Wire.cpp 

C_SRCS += \
../libraries/twi.c 

OBJS += \
./libraries/Messenger.o \
./libraries/Servo.o \
./libraries/Wire.o \
./libraries/twi.o 

C_DEPS += \
./libraries/twi.d 

CPP_DEPS += \
./libraries/Messenger.d \
./libraries/Servo.d \
./libraries/Wire.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/%.o: ../libraries/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/ben/workspace/Tricopter-eclipse/libraries" -I"/home/ben/workspace/DuemilanoveCore/arduino/include" -Wall -Os -fpack-struct -fshort-enums -std=gnu++98 -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=1600000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

libraries/%.o: ../libraries/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/home/ben/workspace/DuemilanoveCore/arduino/include" -I"/home/ben/workspace/Tricopter-eclipse/libraries" -Wall -Os -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=1600000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


