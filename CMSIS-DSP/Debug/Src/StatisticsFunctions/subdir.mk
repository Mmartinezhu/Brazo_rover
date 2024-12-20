################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/StatisticsFunctions/StatisticsFunctions.c \
../Src/StatisticsFunctions/StatisticsFunctionsF16.c \
../Src/StatisticsFunctions/arm_absmax_f16.c \
../Src/StatisticsFunctions/arm_absmax_f32.c \
../Src/StatisticsFunctions/arm_absmax_f64.c \
../Src/StatisticsFunctions/arm_absmax_no_idx_f16.c \
../Src/StatisticsFunctions/arm_absmax_no_idx_f32.c \
../Src/StatisticsFunctions/arm_absmax_no_idx_f64.c \
../Src/StatisticsFunctions/arm_absmax_no_idx_q15.c \
../Src/StatisticsFunctions/arm_absmax_no_idx_q31.c \
../Src/StatisticsFunctions/arm_absmax_no_idx_q7.c \
../Src/StatisticsFunctions/arm_absmax_q15.c \
../Src/StatisticsFunctions/arm_absmax_q31.c \
../Src/StatisticsFunctions/arm_absmax_q7.c \
../Src/StatisticsFunctions/arm_absmin_f16.c \
../Src/StatisticsFunctions/arm_absmin_f32.c \
../Src/StatisticsFunctions/arm_absmin_f64.c \
../Src/StatisticsFunctions/arm_absmin_no_idx_f16.c \
../Src/StatisticsFunctions/arm_absmin_no_idx_f32.c \
../Src/StatisticsFunctions/arm_absmin_no_idx_f64.c \
../Src/StatisticsFunctions/arm_absmin_no_idx_q15.c \
../Src/StatisticsFunctions/arm_absmin_no_idx_q31.c \
../Src/StatisticsFunctions/arm_absmin_no_idx_q7.c \
../Src/StatisticsFunctions/arm_absmin_q15.c \
../Src/StatisticsFunctions/arm_absmin_q31.c \
../Src/StatisticsFunctions/arm_absmin_q7.c \
../Src/StatisticsFunctions/arm_entropy_f16.c \
../Src/StatisticsFunctions/arm_entropy_f32.c \
../Src/StatisticsFunctions/arm_entropy_f64.c \
../Src/StatisticsFunctions/arm_kullback_leibler_f16.c \
../Src/StatisticsFunctions/arm_kullback_leibler_f32.c \
../Src/StatisticsFunctions/arm_kullback_leibler_f64.c \
../Src/StatisticsFunctions/arm_logsumexp_dot_prod_f16.c \
../Src/StatisticsFunctions/arm_logsumexp_dot_prod_f32.c \
../Src/StatisticsFunctions/arm_logsumexp_f16.c \
../Src/StatisticsFunctions/arm_logsumexp_f32.c \
../Src/StatisticsFunctions/arm_max_f16.c \
../Src/StatisticsFunctions/arm_max_f32.c \
../Src/StatisticsFunctions/arm_max_f64.c \
../Src/StatisticsFunctions/arm_max_no_idx_f16.c \
../Src/StatisticsFunctions/arm_max_no_idx_f32.c \
../Src/StatisticsFunctions/arm_max_no_idx_f64.c \
../Src/StatisticsFunctions/arm_max_no_idx_q15.c \
../Src/StatisticsFunctions/arm_max_no_idx_q31.c \
../Src/StatisticsFunctions/arm_max_no_idx_q7.c \
../Src/StatisticsFunctions/arm_max_q15.c \
../Src/StatisticsFunctions/arm_max_q31.c \
../Src/StatisticsFunctions/arm_max_q7.c \
../Src/StatisticsFunctions/arm_mean_f16.c \
../Src/StatisticsFunctions/arm_mean_f32.c \
../Src/StatisticsFunctions/arm_mean_f64.c \
../Src/StatisticsFunctions/arm_mean_q15.c \
../Src/StatisticsFunctions/arm_mean_q31.c \
../Src/StatisticsFunctions/arm_mean_q7.c \
../Src/StatisticsFunctions/arm_min_f16.c \
../Src/StatisticsFunctions/arm_min_f32.c \
../Src/StatisticsFunctions/arm_min_f64.c \
../Src/StatisticsFunctions/arm_min_no_idx_f16.c \
../Src/StatisticsFunctions/arm_min_no_idx_f32.c \
../Src/StatisticsFunctions/arm_min_no_idx_f64.c \
../Src/StatisticsFunctions/arm_min_no_idx_q15.c \
../Src/StatisticsFunctions/arm_min_no_idx_q31.c \
../Src/StatisticsFunctions/arm_min_no_idx_q7.c \
../Src/StatisticsFunctions/arm_min_q15.c \
../Src/StatisticsFunctions/arm_min_q31.c \
../Src/StatisticsFunctions/arm_min_q7.c \
../Src/StatisticsFunctions/arm_mse_f16.c \
../Src/StatisticsFunctions/arm_mse_f32.c \
../Src/StatisticsFunctions/arm_mse_f64.c \
../Src/StatisticsFunctions/arm_mse_q15.c \
../Src/StatisticsFunctions/arm_mse_q31.c \
../Src/StatisticsFunctions/arm_mse_q7.c \
../Src/StatisticsFunctions/arm_power_f16.c \
../Src/StatisticsFunctions/arm_power_f32.c \
../Src/StatisticsFunctions/arm_power_f64.c \
../Src/StatisticsFunctions/arm_power_q15.c \
../Src/StatisticsFunctions/arm_power_q31.c \
../Src/StatisticsFunctions/arm_power_q7.c \
../Src/StatisticsFunctions/arm_rms_f16.c \
../Src/StatisticsFunctions/arm_rms_f32.c \
../Src/StatisticsFunctions/arm_rms_q15.c \
../Src/StatisticsFunctions/arm_rms_q31.c \
../Src/StatisticsFunctions/arm_std_f16.c \
../Src/StatisticsFunctions/arm_std_f32.c \
../Src/StatisticsFunctions/arm_std_f64.c \
../Src/StatisticsFunctions/arm_std_q15.c \
../Src/StatisticsFunctions/arm_std_q31.c \
../Src/StatisticsFunctions/arm_var_f16.c \
../Src/StatisticsFunctions/arm_var_f32.c \
../Src/StatisticsFunctions/arm_var_f64.c \
../Src/StatisticsFunctions/arm_var_q15.c \
../Src/StatisticsFunctions/arm_var_q31.c 

OBJS += \
./Src/StatisticsFunctions/StatisticsFunctions.o \
./Src/StatisticsFunctions/StatisticsFunctionsF16.o \
./Src/StatisticsFunctions/arm_absmax_f16.o \
./Src/StatisticsFunctions/arm_absmax_f32.o \
./Src/StatisticsFunctions/arm_absmax_f64.o \
./Src/StatisticsFunctions/arm_absmax_no_idx_f16.o \
./Src/StatisticsFunctions/arm_absmax_no_idx_f32.o \
./Src/StatisticsFunctions/arm_absmax_no_idx_f64.o \
./Src/StatisticsFunctions/arm_absmax_no_idx_q15.o \
./Src/StatisticsFunctions/arm_absmax_no_idx_q31.o \
./Src/StatisticsFunctions/arm_absmax_no_idx_q7.o \
./Src/StatisticsFunctions/arm_absmax_q15.o \
./Src/StatisticsFunctions/arm_absmax_q31.o \
./Src/StatisticsFunctions/arm_absmax_q7.o \
./Src/StatisticsFunctions/arm_absmin_f16.o \
./Src/StatisticsFunctions/arm_absmin_f32.o \
./Src/StatisticsFunctions/arm_absmin_f64.o \
./Src/StatisticsFunctions/arm_absmin_no_idx_f16.o \
./Src/StatisticsFunctions/arm_absmin_no_idx_f32.o \
./Src/StatisticsFunctions/arm_absmin_no_idx_f64.o \
./Src/StatisticsFunctions/arm_absmin_no_idx_q15.o \
./Src/StatisticsFunctions/arm_absmin_no_idx_q31.o \
./Src/StatisticsFunctions/arm_absmin_no_idx_q7.o \
./Src/StatisticsFunctions/arm_absmin_q15.o \
./Src/StatisticsFunctions/arm_absmin_q31.o \
./Src/StatisticsFunctions/arm_absmin_q7.o \
./Src/StatisticsFunctions/arm_entropy_f16.o \
./Src/StatisticsFunctions/arm_entropy_f32.o \
./Src/StatisticsFunctions/arm_entropy_f64.o \
./Src/StatisticsFunctions/arm_kullback_leibler_f16.o \
./Src/StatisticsFunctions/arm_kullback_leibler_f32.o \
./Src/StatisticsFunctions/arm_kullback_leibler_f64.o \
./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f16.o \
./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f32.o \
./Src/StatisticsFunctions/arm_logsumexp_f16.o \
./Src/StatisticsFunctions/arm_logsumexp_f32.o \
./Src/StatisticsFunctions/arm_max_f16.o \
./Src/StatisticsFunctions/arm_max_f32.o \
./Src/StatisticsFunctions/arm_max_f64.o \
./Src/StatisticsFunctions/arm_max_no_idx_f16.o \
./Src/StatisticsFunctions/arm_max_no_idx_f32.o \
./Src/StatisticsFunctions/arm_max_no_idx_f64.o \
./Src/StatisticsFunctions/arm_max_no_idx_q15.o \
./Src/StatisticsFunctions/arm_max_no_idx_q31.o \
./Src/StatisticsFunctions/arm_max_no_idx_q7.o \
./Src/StatisticsFunctions/arm_max_q15.o \
./Src/StatisticsFunctions/arm_max_q31.o \
./Src/StatisticsFunctions/arm_max_q7.o \
./Src/StatisticsFunctions/arm_mean_f16.o \
./Src/StatisticsFunctions/arm_mean_f32.o \
./Src/StatisticsFunctions/arm_mean_f64.o \
./Src/StatisticsFunctions/arm_mean_q15.o \
./Src/StatisticsFunctions/arm_mean_q31.o \
./Src/StatisticsFunctions/arm_mean_q7.o \
./Src/StatisticsFunctions/arm_min_f16.o \
./Src/StatisticsFunctions/arm_min_f32.o \
./Src/StatisticsFunctions/arm_min_f64.o \
./Src/StatisticsFunctions/arm_min_no_idx_f16.o \
./Src/StatisticsFunctions/arm_min_no_idx_f32.o \
./Src/StatisticsFunctions/arm_min_no_idx_f64.o \
./Src/StatisticsFunctions/arm_min_no_idx_q15.o \
./Src/StatisticsFunctions/arm_min_no_idx_q31.o \
./Src/StatisticsFunctions/arm_min_no_idx_q7.o \
./Src/StatisticsFunctions/arm_min_q15.o \
./Src/StatisticsFunctions/arm_min_q31.o \
./Src/StatisticsFunctions/arm_min_q7.o \
./Src/StatisticsFunctions/arm_mse_f16.o \
./Src/StatisticsFunctions/arm_mse_f32.o \
./Src/StatisticsFunctions/arm_mse_f64.o \
./Src/StatisticsFunctions/arm_mse_q15.o \
./Src/StatisticsFunctions/arm_mse_q31.o \
./Src/StatisticsFunctions/arm_mse_q7.o \
./Src/StatisticsFunctions/arm_power_f16.o \
./Src/StatisticsFunctions/arm_power_f32.o \
./Src/StatisticsFunctions/arm_power_f64.o \
./Src/StatisticsFunctions/arm_power_q15.o \
./Src/StatisticsFunctions/arm_power_q31.o \
./Src/StatisticsFunctions/arm_power_q7.o \
./Src/StatisticsFunctions/arm_rms_f16.o \
./Src/StatisticsFunctions/arm_rms_f32.o \
./Src/StatisticsFunctions/arm_rms_q15.o \
./Src/StatisticsFunctions/arm_rms_q31.o \
./Src/StatisticsFunctions/arm_std_f16.o \
./Src/StatisticsFunctions/arm_std_f32.o \
./Src/StatisticsFunctions/arm_std_f64.o \
./Src/StatisticsFunctions/arm_std_q15.o \
./Src/StatisticsFunctions/arm_std_q31.o \
./Src/StatisticsFunctions/arm_var_f16.o \
./Src/StatisticsFunctions/arm_var_f32.o \
./Src/StatisticsFunctions/arm_var_f64.o \
./Src/StatisticsFunctions/arm_var_q15.o \
./Src/StatisticsFunctions/arm_var_q31.o 

C_DEPS += \
./Src/StatisticsFunctions/StatisticsFunctions.d \
./Src/StatisticsFunctions/StatisticsFunctionsF16.d \
./Src/StatisticsFunctions/arm_absmax_f16.d \
./Src/StatisticsFunctions/arm_absmax_f32.d \
./Src/StatisticsFunctions/arm_absmax_f64.d \
./Src/StatisticsFunctions/arm_absmax_no_idx_f16.d \
./Src/StatisticsFunctions/arm_absmax_no_idx_f32.d \
./Src/StatisticsFunctions/arm_absmax_no_idx_f64.d \
./Src/StatisticsFunctions/arm_absmax_no_idx_q15.d \
./Src/StatisticsFunctions/arm_absmax_no_idx_q31.d \
./Src/StatisticsFunctions/arm_absmax_no_idx_q7.d \
./Src/StatisticsFunctions/arm_absmax_q15.d \
./Src/StatisticsFunctions/arm_absmax_q31.d \
./Src/StatisticsFunctions/arm_absmax_q7.d \
./Src/StatisticsFunctions/arm_absmin_f16.d \
./Src/StatisticsFunctions/arm_absmin_f32.d \
./Src/StatisticsFunctions/arm_absmin_f64.d \
./Src/StatisticsFunctions/arm_absmin_no_idx_f16.d \
./Src/StatisticsFunctions/arm_absmin_no_idx_f32.d \
./Src/StatisticsFunctions/arm_absmin_no_idx_f64.d \
./Src/StatisticsFunctions/arm_absmin_no_idx_q15.d \
./Src/StatisticsFunctions/arm_absmin_no_idx_q31.d \
./Src/StatisticsFunctions/arm_absmin_no_idx_q7.d \
./Src/StatisticsFunctions/arm_absmin_q15.d \
./Src/StatisticsFunctions/arm_absmin_q31.d \
./Src/StatisticsFunctions/arm_absmin_q7.d \
./Src/StatisticsFunctions/arm_entropy_f16.d \
./Src/StatisticsFunctions/arm_entropy_f32.d \
./Src/StatisticsFunctions/arm_entropy_f64.d \
./Src/StatisticsFunctions/arm_kullback_leibler_f16.d \
./Src/StatisticsFunctions/arm_kullback_leibler_f32.d \
./Src/StatisticsFunctions/arm_kullback_leibler_f64.d \
./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f16.d \
./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f32.d \
./Src/StatisticsFunctions/arm_logsumexp_f16.d \
./Src/StatisticsFunctions/arm_logsumexp_f32.d \
./Src/StatisticsFunctions/arm_max_f16.d \
./Src/StatisticsFunctions/arm_max_f32.d \
./Src/StatisticsFunctions/arm_max_f64.d \
./Src/StatisticsFunctions/arm_max_no_idx_f16.d \
./Src/StatisticsFunctions/arm_max_no_idx_f32.d \
./Src/StatisticsFunctions/arm_max_no_idx_f64.d \
./Src/StatisticsFunctions/arm_max_no_idx_q15.d \
./Src/StatisticsFunctions/arm_max_no_idx_q31.d \
./Src/StatisticsFunctions/arm_max_no_idx_q7.d \
./Src/StatisticsFunctions/arm_max_q15.d \
./Src/StatisticsFunctions/arm_max_q31.d \
./Src/StatisticsFunctions/arm_max_q7.d \
./Src/StatisticsFunctions/arm_mean_f16.d \
./Src/StatisticsFunctions/arm_mean_f32.d \
./Src/StatisticsFunctions/arm_mean_f64.d \
./Src/StatisticsFunctions/arm_mean_q15.d \
./Src/StatisticsFunctions/arm_mean_q31.d \
./Src/StatisticsFunctions/arm_mean_q7.d \
./Src/StatisticsFunctions/arm_min_f16.d \
./Src/StatisticsFunctions/arm_min_f32.d \
./Src/StatisticsFunctions/arm_min_f64.d \
./Src/StatisticsFunctions/arm_min_no_idx_f16.d \
./Src/StatisticsFunctions/arm_min_no_idx_f32.d \
./Src/StatisticsFunctions/arm_min_no_idx_f64.d \
./Src/StatisticsFunctions/arm_min_no_idx_q15.d \
./Src/StatisticsFunctions/arm_min_no_idx_q31.d \
./Src/StatisticsFunctions/arm_min_no_idx_q7.d \
./Src/StatisticsFunctions/arm_min_q15.d \
./Src/StatisticsFunctions/arm_min_q31.d \
./Src/StatisticsFunctions/arm_min_q7.d \
./Src/StatisticsFunctions/arm_mse_f16.d \
./Src/StatisticsFunctions/arm_mse_f32.d \
./Src/StatisticsFunctions/arm_mse_f64.d \
./Src/StatisticsFunctions/arm_mse_q15.d \
./Src/StatisticsFunctions/arm_mse_q31.d \
./Src/StatisticsFunctions/arm_mse_q7.d \
./Src/StatisticsFunctions/arm_power_f16.d \
./Src/StatisticsFunctions/arm_power_f32.d \
./Src/StatisticsFunctions/arm_power_f64.d \
./Src/StatisticsFunctions/arm_power_q15.d \
./Src/StatisticsFunctions/arm_power_q31.d \
./Src/StatisticsFunctions/arm_power_q7.d \
./Src/StatisticsFunctions/arm_rms_f16.d \
./Src/StatisticsFunctions/arm_rms_f32.d \
./Src/StatisticsFunctions/arm_rms_q15.d \
./Src/StatisticsFunctions/arm_rms_q31.d \
./Src/StatisticsFunctions/arm_std_f16.d \
./Src/StatisticsFunctions/arm_std_f32.d \
./Src/StatisticsFunctions/arm_std_f64.d \
./Src/StatisticsFunctions/arm_std_q15.d \
./Src/StatisticsFunctions/arm_std_q31.d \
./Src/StatisticsFunctions/arm_var_f16.d \
./Src/StatisticsFunctions/arm_var_f32.d \
./Src/StatisticsFunctions/arm_var_f64.d \
./Src/StatisticsFunctions/arm_var_q15.d \
./Src/StatisticsFunctions/arm_var_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Src/StatisticsFunctions/%.o Src/StatisticsFunctions/%.su: ../Src/StatisticsFunctions/%.c Src/StatisticsFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/manuel/Escritorio/Taller_v/workspace_brazo/CMSIS-repo/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/manuel/Escritorio/Taller_v/workspace_brazo/CMSIS-repo/Drivers/CMSIS/Include" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-StatisticsFunctions

clean-Src-2f-StatisticsFunctions:
	-$(RM) ./Src/StatisticsFunctions/StatisticsFunctions.d ./Src/StatisticsFunctions/StatisticsFunctions.o ./Src/StatisticsFunctions/StatisticsFunctions.su ./Src/StatisticsFunctions/StatisticsFunctionsF16.d ./Src/StatisticsFunctions/StatisticsFunctionsF16.o ./Src/StatisticsFunctions/StatisticsFunctionsF16.su ./Src/StatisticsFunctions/arm_absmax_f16.d ./Src/StatisticsFunctions/arm_absmax_f16.o ./Src/StatisticsFunctions/arm_absmax_f16.su ./Src/StatisticsFunctions/arm_absmax_f32.d ./Src/StatisticsFunctions/arm_absmax_f32.o ./Src/StatisticsFunctions/arm_absmax_f32.su ./Src/StatisticsFunctions/arm_absmax_f64.d ./Src/StatisticsFunctions/arm_absmax_f64.o ./Src/StatisticsFunctions/arm_absmax_f64.su ./Src/StatisticsFunctions/arm_absmax_no_idx_f16.d ./Src/StatisticsFunctions/arm_absmax_no_idx_f16.o ./Src/StatisticsFunctions/arm_absmax_no_idx_f16.su ./Src/StatisticsFunctions/arm_absmax_no_idx_f32.d ./Src/StatisticsFunctions/arm_absmax_no_idx_f32.o ./Src/StatisticsFunctions/arm_absmax_no_idx_f32.su ./Src/StatisticsFunctions/arm_absmax_no_idx_f64.d ./Src/StatisticsFunctions/arm_absmax_no_idx_f64.o ./Src/StatisticsFunctions/arm_absmax_no_idx_f64.su ./Src/StatisticsFunctions/arm_absmax_no_idx_q15.d ./Src/StatisticsFunctions/arm_absmax_no_idx_q15.o ./Src/StatisticsFunctions/arm_absmax_no_idx_q15.su ./Src/StatisticsFunctions/arm_absmax_no_idx_q31.d ./Src/StatisticsFunctions/arm_absmax_no_idx_q31.o ./Src/StatisticsFunctions/arm_absmax_no_idx_q31.su ./Src/StatisticsFunctions/arm_absmax_no_idx_q7.d ./Src/StatisticsFunctions/arm_absmax_no_idx_q7.o ./Src/StatisticsFunctions/arm_absmax_no_idx_q7.su ./Src/StatisticsFunctions/arm_absmax_q15.d ./Src/StatisticsFunctions/arm_absmax_q15.o ./Src/StatisticsFunctions/arm_absmax_q15.su ./Src/StatisticsFunctions/arm_absmax_q31.d ./Src/StatisticsFunctions/arm_absmax_q31.o ./Src/StatisticsFunctions/arm_absmax_q31.su ./Src/StatisticsFunctions/arm_absmax_q7.d ./Src/StatisticsFunctions/arm_absmax_q7.o ./Src/StatisticsFunctions/arm_absmax_q7.su ./Src/StatisticsFunctions/arm_absmin_f16.d ./Src/StatisticsFunctions/arm_absmin_f16.o ./Src/StatisticsFunctions/arm_absmin_f16.su ./Src/StatisticsFunctions/arm_absmin_f32.d ./Src/StatisticsFunctions/arm_absmin_f32.o ./Src/StatisticsFunctions/arm_absmin_f32.su ./Src/StatisticsFunctions/arm_absmin_f64.d ./Src/StatisticsFunctions/arm_absmin_f64.o ./Src/StatisticsFunctions/arm_absmin_f64.su ./Src/StatisticsFunctions/arm_absmin_no_idx_f16.d ./Src/StatisticsFunctions/arm_absmin_no_idx_f16.o ./Src/StatisticsFunctions/arm_absmin_no_idx_f16.su ./Src/StatisticsFunctions/arm_absmin_no_idx_f32.d ./Src/StatisticsFunctions/arm_absmin_no_idx_f32.o ./Src/StatisticsFunctions/arm_absmin_no_idx_f32.su ./Src/StatisticsFunctions/arm_absmin_no_idx_f64.d ./Src/StatisticsFunctions/arm_absmin_no_idx_f64.o ./Src/StatisticsFunctions/arm_absmin_no_idx_f64.su ./Src/StatisticsFunctions/arm_absmin_no_idx_q15.d ./Src/StatisticsFunctions/arm_absmin_no_idx_q15.o ./Src/StatisticsFunctions/arm_absmin_no_idx_q15.su ./Src/StatisticsFunctions/arm_absmin_no_idx_q31.d ./Src/StatisticsFunctions/arm_absmin_no_idx_q31.o ./Src/StatisticsFunctions/arm_absmin_no_idx_q31.su ./Src/StatisticsFunctions/arm_absmin_no_idx_q7.d ./Src/StatisticsFunctions/arm_absmin_no_idx_q7.o ./Src/StatisticsFunctions/arm_absmin_no_idx_q7.su ./Src/StatisticsFunctions/arm_absmin_q15.d ./Src/StatisticsFunctions/arm_absmin_q15.o ./Src/StatisticsFunctions/arm_absmin_q15.su ./Src/StatisticsFunctions/arm_absmin_q31.d ./Src/StatisticsFunctions/arm_absmin_q31.o ./Src/StatisticsFunctions/arm_absmin_q31.su ./Src/StatisticsFunctions/arm_absmin_q7.d ./Src/StatisticsFunctions/arm_absmin_q7.o ./Src/StatisticsFunctions/arm_absmin_q7.su ./Src/StatisticsFunctions/arm_entropy_f16.d ./Src/StatisticsFunctions/arm_entropy_f16.o ./Src/StatisticsFunctions/arm_entropy_f16.su ./Src/StatisticsFunctions/arm_entropy_f32.d ./Src/StatisticsFunctions/arm_entropy_f32.o ./Src/StatisticsFunctions/arm_entropy_f32.su ./Src/StatisticsFunctions/arm_entropy_f64.d ./Src/StatisticsFunctions/arm_entropy_f64.o ./Src/StatisticsFunctions/arm_entropy_f64.su ./Src/StatisticsFunctions/arm_kullback_leibler_f16.d ./Src/StatisticsFunctions/arm_kullback_leibler_f16.o ./Src/StatisticsFunctions/arm_kullback_leibler_f16.su ./Src/StatisticsFunctions/arm_kullback_leibler_f32.d ./Src/StatisticsFunctions/arm_kullback_leibler_f32.o ./Src/StatisticsFunctions/arm_kullback_leibler_f32.su ./Src/StatisticsFunctions/arm_kullback_leibler_f64.d ./Src/StatisticsFunctions/arm_kullback_leibler_f64.o ./Src/StatisticsFunctions/arm_kullback_leibler_f64.su ./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f16.d ./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f16.o ./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f16.su ./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f32.d ./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f32.o ./Src/StatisticsFunctions/arm_logsumexp_dot_prod_f32.su ./Src/StatisticsFunctions/arm_logsumexp_f16.d ./Src/StatisticsFunctions/arm_logsumexp_f16.o ./Src/StatisticsFunctions/arm_logsumexp_f16.su ./Src/StatisticsFunctions/arm_logsumexp_f32.d ./Src/StatisticsFunctions/arm_logsumexp_f32.o ./Src/StatisticsFunctions/arm_logsumexp_f32.su ./Src/StatisticsFunctions/arm_max_f16.d ./Src/StatisticsFunctions/arm_max_f16.o ./Src/StatisticsFunctions/arm_max_f16.su ./Src/StatisticsFunctions/arm_max_f32.d ./Src/StatisticsFunctions/arm_max_f32.o ./Src/StatisticsFunctions/arm_max_f32.su ./Src/StatisticsFunctions/arm_max_f64.d ./Src/StatisticsFunctions/arm_max_f64.o ./Src/StatisticsFunctions/arm_max_f64.su ./Src/StatisticsFunctions/arm_max_no_idx_f16.d ./Src/StatisticsFunctions/arm_max_no_idx_f16.o ./Src/StatisticsFunctions/arm_max_no_idx_f16.su ./Src/StatisticsFunctions/arm_max_no_idx_f32.d ./Src/StatisticsFunctions/arm_max_no_idx_f32.o ./Src/StatisticsFunctions/arm_max_no_idx_f32.su ./Src/StatisticsFunctions/arm_max_no_idx_f64.d ./Src/StatisticsFunctions/arm_max_no_idx_f64.o ./Src/StatisticsFunctions/arm_max_no_idx_f64.su ./Src/StatisticsFunctions/arm_max_no_idx_q15.d
	-$(RM) ./Src/StatisticsFunctions/arm_max_no_idx_q15.o ./Src/StatisticsFunctions/arm_max_no_idx_q15.su ./Src/StatisticsFunctions/arm_max_no_idx_q31.d ./Src/StatisticsFunctions/arm_max_no_idx_q31.o ./Src/StatisticsFunctions/arm_max_no_idx_q31.su ./Src/StatisticsFunctions/arm_max_no_idx_q7.d ./Src/StatisticsFunctions/arm_max_no_idx_q7.o ./Src/StatisticsFunctions/arm_max_no_idx_q7.su ./Src/StatisticsFunctions/arm_max_q15.d ./Src/StatisticsFunctions/arm_max_q15.o ./Src/StatisticsFunctions/arm_max_q15.su ./Src/StatisticsFunctions/arm_max_q31.d ./Src/StatisticsFunctions/arm_max_q31.o ./Src/StatisticsFunctions/arm_max_q31.su ./Src/StatisticsFunctions/arm_max_q7.d ./Src/StatisticsFunctions/arm_max_q7.o ./Src/StatisticsFunctions/arm_max_q7.su ./Src/StatisticsFunctions/arm_mean_f16.d ./Src/StatisticsFunctions/arm_mean_f16.o ./Src/StatisticsFunctions/arm_mean_f16.su ./Src/StatisticsFunctions/arm_mean_f32.d ./Src/StatisticsFunctions/arm_mean_f32.o ./Src/StatisticsFunctions/arm_mean_f32.su ./Src/StatisticsFunctions/arm_mean_f64.d ./Src/StatisticsFunctions/arm_mean_f64.o ./Src/StatisticsFunctions/arm_mean_f64.su ./Src/StatisticsFunctions/arm_mean_q15.d ./Src/StatisticsFunctions/arm_mean_q15.o ./Src/StatisticsFunctions/arm_mean_q15.su ./Src/StatisticsFunctions/arm_mean_q31.d ./Src/StatisticsFunctions/arm_mean_q31.o ./Src/StatisticsFunctions/arm_mean_q31.su ./Src/StatisticsFunctions/arm_mean_q7.d ./Src/StatisticsFunctions/arm_mean_q7.o ./Src/StatisticsFunctions/arm_mean_q7.su ./Src/StatisticsFunctions/arm_min_f16.d ./Src/StatisticsFunctions/arm_min_f16.o ./Src/StatisticsFunctions/arm_min_f16.su ./Src/StatisticsFunctions/arm_min_f32.d ./Src/StatisticsFunctions/arm_min_f32.o ./Src/StatisticsFunctions/arm_min_f32.su ./Src/StatisticsFunctions/arm_min_f64.d ./Src/StatisticsFunctions/arm_min_f64.o ./Src/StatisticsFunctions/arm_min_f64.su ./Src/StatisticsFunctions/arm_min_no_idx_f16.d ./Src/StatisticsFunctions/arm_min_no_idx_f16.o ./Src/StatisticsFunctions/arm_min_no_idx_f16.su ./Src/StatisticsFunctions/arm_min_no_idx_f32.d ./Src/StatisticsFunctions/arm_min_no_idx_f32.o ./Src/StatisticsFunctions/arm_min_no_idx_f32.su ./Src/StatisticsFunctions/arm_min_no_idx_f64.d ./Src/StatisticsFunctions/arm_min_no_idx_f64.o ./Src/StatisticsFunctions/arm_min_no_idx_f64.su ./Src/StatisticsFunctions/arm_min_no_idx_q15.d ./Src/StatisticsFunctions/arm_min_no_idx_q15.o ./Src/StatisticsFunctions/arm_min_no_idx_q15.su ./Src/StatisticsFunctions/arm_min_no_idx_q31.d ./Src/StatisticsFunctions/arm_min_no_idx_q31.o ./Src/StatisticsFunctions/arm_min_no_idx_q31.su ./Src/StatisticsFunctions/arm_min_no_idx_q7.d ./Src/StatisticsFunctions/arm_min_no_idx_q7.o ./Src/StatisticsFunctions/arm_min_no_idx_q7.su ./Src/StatisticsFunctions/arm_min_q15.d ./Src/StatisticsFunctions/arm_min_q15.o ./Src/StatisticsFunctions/arm_min_q15.su ./Src/StatisticsFunctions/arm_min_q31.d ./Src/StatisticsFunctions/arm_min_q31.o ./Src/StatisticsFunctions/arm_min_q31.su ./Src/StatisticsFunctions/arm_min_q7.d ./Src/StatisticsFunctions/arm_min_q7.o ./Src/StatisticsFunctions/arm_min_q7.su ./Src/StatisticsFunctions/arm_mse_f16.d ./Src/StatisticsFunctions/arm_mse_f16.o ./Src/StatisticsFunctions/arm_mse_f16.su ./Src/StatisticsFunctions/arm_mse_f32.d ./Src/StatisticsFunctions/arm_mse_f32.o ./Src/StatisticsFunctions/arm_mse_f32.su ./Src/StatisticsFunctions/arm_mse_f64.d ./Src/StatisticsFunctions/arm_mse_f64.o ./Src/StatisticsFunctions/arm_mse_f64.su ./Src/StatisticsFunctions/arm_mse_q15.d ./Src/StatisticsFunctions/arm_mse_q15.o ./Src/StatisticsFunctions/arm_mse_q15.su ./Src/StatisticsFunctions/arm_mse_q31.d ./Src/StatisticsFunctions/arm_mse_q31.o ./Src/StatisticsFunctions/arm_mse_q31.su ./Src/StatisticsFunctions/arm_mse_q7.d ./Src/StatisticsFunctions/arm_mse_q7.o ./Src/StatisticsFunctions/arm_mse_q7.su ./Src/StatisticsFunctions/arm_power_f16.d ./Src/StatisticsFunctions/arm_power_f16.o ./Src/StatisticsFunctions/arm_power_f16.su ./Src/StatisticsFunctions/arm_power_f32.d ./Src/StatisticsFunctions/arm_power_f32.o ./Src/StatisticsFunctions/arm_power_f32.su ./Src/StatisticsFunctions/arm_power_f64.d ./Src/StatisticsFunctions/arm_power_f64.o ./Src/StatisticsFunctions/arm_power_f64.su ./Src/StatisticsFunctions/arm_power_q15.d ./Src/StatisticsFunctions/arm_power_q15.o ./Src/StatisticsFunctions/arm_power_q15.su ./Src/StatisticsFunctions/arm_power_q31.d ./Src/StatisticsFunctions/arm_power_q31.o ./Src/StatisticsFunctions/arm_power_q31.su ./Src/StatisticsFunctions/arm_power_q7.d ./Src/StatisticsFunctions/arm_power_q7.o ./Src/StatisticsFunctions/arm_power_q7.su ./Src/StatisticsFunctions/arm_rms_f16.d ./Src/StatisticsFunctions/arm_rms_f16.o ./Src/StatisticsFunctions/arm_rms_f16.su ./Src/StatisticsFunctions/arm_rms_f32.d ./Src/StatisticsFunctions/arm_rms_f32.o ./Src/StatisticsFunctions/arm_rms_f32.su ./Src/StatisticsFunctions/arm_rms_q15.d ./Src/StatisticsFunctions/arm_rms_q15.o ./Src/StatisticsFunctions/arm_rms_q15.su ./Src/StatisticsFunctions/arm_rms_q31.d ./Src/StatisticsFunctions/arm_rms_q31.o ./Src/StatisticsFunctions/arm_rms_q31.su ./Src/StatisticsFunctions/arm_std_f16.d ./Src/StatisticsFunctions/arm_std_f16.o ./Src/StatisticsFunctions/arm_std_f16.su ./Src/StatisticsFunctions/arm_std_f32.d ./Src/StatisticsFunctions/arm_std_f32.o ./Src/StatisticsFunctions/arm_std_f32.su ./Src/StatisticsFunctions/arm_std_f64.d ./Src/StatisticsFunctions/arm_std_f64.o ./Src/StatisticsFunctions/arm_std_f64.su ./Src/StatisticsFunctions/arm_std_q15.d ./Src/StatisticsFunctions/arm_std_q15.o ./Src/StatisticsFunctions/arm_std_q15.su ./Src/StatisticsFunctions/arm_std_q31.d ./Src/StatisticsFunctions/arm_std_q31.o ./Src/StatisticsFunctions/arm_std_q31.su ./Src/StatisticsFunctions/arm_var_f16.d ./Src/StatisticsFunctions/arm_var_f16.o ./Src/StatisticsFunctions/arm_var_f16.su ./Src/StatisticsFunctions/arm_var_f32.d ./Src/StatisticsFunctions/arm_var_f32.o ./Src/StatisticsFunctions/arm_var_f32.su ./Src/StatisticsFunctions/arm_var_f64.d ./Src/StatisticsFunctions/arm_var_f64.o ./Src/StatisticsFunctions/arm_var_f64.su
	-$(RM) ./Src/StatisticsFunctions/arm_var_q15.d ./Src/StatisticsFunctions/arm_var_q15.o ./Src/StatisticsFunctions/arm_var_q15.su ./Src/StatisticsFunctions/arm_var_q31.d ./Src/StatisticsFunctions/arm_var_q31.o ./Src/StatisticsFunctions/arm_var_q31.su

.PHONY: clean-Src-2f-StatisticsFunctions

