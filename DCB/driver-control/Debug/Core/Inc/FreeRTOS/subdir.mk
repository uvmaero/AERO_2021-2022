################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/FreeRTOS/heap_1.c \
../Core/Inc/FreeRTOS/list.c \
../Core/Inc/FreeRTOS/port.c \
../Core/Inc/FreeRTOS/queue.c \
../Core/Inc/FreeRTOS/tasks.c \
../Core/Inc/FreeRTOS/timers.c 

C_DEPS += \
./Core/Inc/FreeRTOS/heap_1.d \
./Core/Inc/FreeRTOS/list.d \
./Core/Inc/FreeRTOS/port.d \
./Core/Inc/FreeRTOS/queue.d \
./Core/Inc/FreeRTOS/tasks.d \
./Core/Inc/FreeRTOS/timers.d 

OBJS += \
./Core/Inc/FreeRTOS/heap_1.o \
./Core/Inc/FreeRTOS/list.o \
./Core/Inc/FreeRTOS/port.o \
./Core/Inc/FreeRTOS/queue.o \
./Core/Inc/FreeRTOS/tasks.o \
./Core/Inc/FreeRTOS/timers.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/FreeRTOS/%.o: ../Core/Inc/FreeRTOS/%.c Core/Inc/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-FreeRTOS

clean-Core-2f-Inc-2f-FreeRTOS:
	-$(RM) ./Core/Inc/FreeRTOS/heap_1.d ./Core/Inc/FreeRTOS/heap_1.o ./Core/Inc/FreeRTOS/list.d ./Core/Inc/FreeRTOS/list.o ./Core/Inc/FreeRTOS/port.d ./Core/Inc/FreeRTOS/port.o ./Core/Inc/FreeRTOS/queue.d ./Core/Inc/FreeRTOS/queue.o ./Core/Inc/FreeRTOS/tasks.d ./Core/Inc/FreeRTOS/tasks.o ./Core/Inc/FreeRTOS/timers.d ./Core/Inc/FreeRTOS/timers.o

.PHONY: clean-Core-2f-Inc-2f-FreeRTOS

