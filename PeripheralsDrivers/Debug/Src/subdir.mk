################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc_driver_hal.c \
../Src/cmd_driver.c \
../Src/exti_driver_hal.c \
../Src/gpio_driver_hal.c \
../Src/i2c_driver_hal.c \
../Src/mpu6050_driver.c \
../Src/pll_driver_hal.c \
../Src/pwm_driver_hal.c \
../Src/systick_driver_hal.c \
../Src/timer_driver_hal.c \
../Src/usart_driver_hal.c 

OBJS += \
./Src/adc_driver_hal.o \
./Src/cmd_driver.o \
./Src/exti_driver_hal.o \
./Src/gpio_driver_hal.o \
./Src/i2c_driver_hal.o \
./Src/mpu6050_driver.o \
./Src/pll_driver_hal.o \
./Src/pwm_driver_hal.o \
./Src/systick_driver_hal.o \
./Src/timer_driver_hal.o \
./Src/usart_driver_hal.o 

C_DEPS += \
./Src/adc_driver_hal.d \
./Src/cmd_driver.d \
./Src/exti_driver_hal.d \
./Src/gpio_driver_hal.d \
./Src/i2c_driver_hal.d \
./Src/mpu6050_driver.d \
./Src/pll_driver_hal.d \
./Src/pwm_driver_hal.d \
./Src/systick_driver_hal.d \
./Src/timer_driver_hal.d \
./Src/usart_driver_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/manuel/Escritorio/Taller_v/workspace_brazo/Software_Toolchain/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/manuel/Escritorio/Taller_v/workspace_brazo/Software_Toolchain/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I/home/manuel/Documentos/GitHub/Brazo_rover/PeripheralsDrivers -I"/home/manuel/Documentos/GitHub/EthereaProject/PeripheralDrivers" -I/home/manuel/Documentos/GitHub/Brazo_rover/PeripheralsDrivers/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/adc_driver_hal.d ./Src/adc_driver_hal.o ./Src/adc_driver_hal.su ./Src/cmd_driver.d ./Src/cmd_driver.o ./Src/cmd_driver.su ./Src/exti_driver_hal.d ./Src/exti_driver_hal.o ./Src/exti_driver_hal.su ./Src/gpio_driver_hal.d ./Src/gpio_driver_hal.o ./Src/gpio_driver_hal.su ./Src/i2c_driver_hal.d ./Src/i2c_driver_hal.o ./Src/i2c_driver_hal.su ./Src/mpu6050_driver.d ./Src/mpu6050_driver.o ./Src/mpu6050_driver.su ./Src/pll_driver_hal.d ./Src/pll_driver_hal.o ./Src/pll_driver_hal.su ./Src/pwm_driver_hal.d ./Src/pwm_driver_hal.o ./Src/pwm_driver_hal.su ./Src/systick_driver_hal.d ./Src/systick_driver_hal.o ./Src/systick_driver_hal.su ./Src/timer_driver_hal.d ./Src/timer_driver_hal.o ./Src/timer_driver_hal.su ./Src/usart_driver_hal.d ./Src/usart_driver_hal.o ./Src/usart_driver_hal.su

.PHONY: clean-Src

