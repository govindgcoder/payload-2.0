################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/ICM20948/util/ICM_20948_C.c 

C_DEPS += \
./Core/ICM20948/util/ICM_20948_C.d 

OBJS += \
./Core/ICM20948/util/ICM_20948_C.o 


# Each subdirectory must supply rules for building sources it contributes
Core/ICM20948/util/%.o Core/ICM20948/util/%.su Core/ICM20948/util/%.cyclo: ../Core/ICM20948/util/%.c Core/ICM20948/util/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I"C:/Users/govind/VeloCET/payload2.0/stm/testSTM/Core/ICM20948/util" -I../Core/Inc -I"C:/Users/govind/VeloCET/payload2.0/stm/testSTM/Core/ICM20948" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-ICM20948-2f-util

clean-Core-2f-ICM20948-2f-util:
	-$(RM) ./Core/ICM20948/util/ICM_20948_C.cyclo ./Core/ICM20948/util/ICM_20948_C.d ./Core/ICM20948/util/ICM_20948_C.o ./Core/ICM20948/util/ICM_20948_C.su

.PHONY: clean-Core-2f-ICM20948-2f-util

