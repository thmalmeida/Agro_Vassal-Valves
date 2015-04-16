################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../twi.c 

CPP_SRCS += \
../DS1307RTC.cpp \
../MemoryFree.cpp \
../Time.cpp \
../Wire.cpp \
../main.cpp 

C_DEPS += \
./twi.d 

OBJS += \
./DS1307RTC.o \
./MemoryFree.o \
./Time.o \
./Wire.o \
./main.o \
./twi.o 

CPP_DEPS += \
./DS1307RTC.d \
./MemoryFree.d \
./Time.d \
./Wire.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\Users\Thiago\sourceCodes1\arduino_core\src" -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"C:\Users\Thiago\sourceCodes1\arduino_core\src" -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -std=gnu99 -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


