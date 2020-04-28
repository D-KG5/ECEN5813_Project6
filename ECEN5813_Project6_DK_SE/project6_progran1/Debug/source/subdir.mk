################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/dac.c \
../source/global_defines.c \
../source/led_control.c \
../source/logger.c \
../source/main.c \
../source/mtb.c \
../source/semihost_hardfault.c 

OBJS += \
./source/dac.o \
./source/global_defines.o \
./source/led_control.o \
./source/logger.o \
./source/main.o \
./source/mtb.o \
./source/semihost_hardfault.o 

C_DEPS += \
./source/dac.d \
./source/global_defines.d \
./source/led_control.d \
./source/logger.d \
./source/main.d \
./source/mtb.d \
./source/semihost_hardfault.d 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MKL25Z128VLK4_cm0plus -DCPU_MKL25Z128VLK4 -DDEBUG -DFSL_RTOS_FREE_RTOS -DFRDM_KL25Z -DFREEDOM -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSDK_DEBUGCONSOLE_UART -D__MCUXPRESSO -D__USE_CMSIS -I"C:\Users\sagar\Documents\MCUXpressoIDE_11.1.0_3209\workspace\project6_progran1\board" -I"C:\Users\sagar\Documents\MCUXpressoIDE_11.1.0_3209\workspace\project6_progran1\source" -I"C:\Users\sagar\Documents\MCUXpressoIDE_11.1.0_3209\workspace\project6_progran1" -I"C:\Users\sagar\Documents\MCUXpressoIDE_11.1.0_3209\workspace\project6_progran1\drivers" -I"C:\Users\sagar\Documents\MCUXpressoIDE_11.1.0_3209\workspace\project6_progran1\CMSIS" -I"C:\Users\sagar\Documents\MCUXpressoIDE_11.1.0_3209\workspace\project6_progran1\utilities" -I"C:\Users\sagar\Documents\MCUXpressoIDE_11.1.0_3209\workspace\project6_progran1\freertos" -I"C:\Users\sagar\Documents\MCUXpressoIDE_11.1.0_3209\workspace\project6_progran1\startup" -O0 -fno-common -g -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


