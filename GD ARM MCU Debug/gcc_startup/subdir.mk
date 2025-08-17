################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../gcc_startup/startup_gd32f30x_hd.S 

OBJS += \
./gcc_startup/startup_gd32f30x_hd.o 

S_UPPER_DEPS += \
./gcc_startup/startup_gd32f30x_hd.d 


# Each subdirectory must supply rules for building sources it contributes
gcc_startup/%.o: ../gcc_startup/%.S gcc_startup/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GD ARM MCU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -Wa,-adhlns=$@.lst   -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-gcc_startup

clean-gcc_startup:
	-$(RM) ./gcc_startup/startup_gd32f30x_hd.d ./gcc_startup/startup_gd32f30x_hd.o

.PHONY: clean-gcc_startup

