################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/control.cpp \
../src/input.cpp \
../src/matrix_math.cpp \
../src/motors.cpp \
../src/nav.cpp \
../src/pid.cpp \
../src/sensors.cpp \
../src/telemetry.cpp \
../src/tricopter.cpp \
../src/utils.cpp 

OBJS += \
./src/control.o \
./src/input.o \
./src/matrix_math.o \
./src/motors.o \
./src/nav.o \
./src/pid.o \
./src/sensors.o \
./src/telemetry.o \
./src/tricopter.o \
./src/utils.o 

CPP_DEPS += \
./src/control.d \
./src/input.d \
./src/matrix_math.d \
./src/motors.d \
./src/nav.d \
./src/pid.d \
./src/sensors.d \
./src/telemetry.d \
./src/tricopter.d \
./src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/ben/workspace/Tricopter-eclipse/libraries" -I"/home/ben/workspace/DuemilanoveCore/arduino/include" -Wall -Os -fpack-struct -fshort-enums -std=gnu++98 -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=1600000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


