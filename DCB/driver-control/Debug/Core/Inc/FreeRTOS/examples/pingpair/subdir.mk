################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/FreeRTOS/examples/pingpair/app_tasks.cpp \
../Core/Inc/FreeRTOS/examples/pingpair/radio.cpp \
../Core/Inc/FreeRTOS/examples/pingpair/setup.cpp 

OBJS += \
./Core/Inc/FreeRTOS/examples/pingpair/app_tasks.o \
./Core/Inc/FreeRTOS/examples/pingpair/radio.o \
./Core/Inc/FreeRTOS/examples/pingpair/setup.o 

CPP_DEPS += \
./Core/Inc/FreeRTOS/examples/pingpair/app_tasks.d \
./Core/Inc/FreeRTOS/examples/pingpair/radio.d \
./Core/Inc/FreeRTOS/examples/pingpair/setup.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/FreeRTOS/examples/pingpair/%.o: ../Core/Inc/FreeRTOS/examples/pingpair/%.cpp Core/Inc/FreeRTOS/examples/pingpair/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-FreeRTOS-2f-examples-2f-pingpair

clean-Core-2f-Inc-2f-FreeRTOS-2f-examples-2f-pingpair:
	-$(RM) ./Core/Inc/FreeRTOS/examples/pingpair/app_tasks.d ./Core/Inc/FreeRTOS/examples/pingpair/app_tasks.o ./Core/Inc/FreeRTOS/examples/pingpair/radio.d ./Core/Inc/FreeRTOS/examples/pingpair/radio.o ./Core/Inc/FreeRTOS/examples/pingpair/setup.d ./Core/Inc/FreeRTOS/examples/pingpair/setup.o

.PHONY: clean-Core-2f-Inc-2f-FreeRTOS-2f-examples-2f-pingpair

