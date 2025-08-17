################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Firmware/CMSIS/GD/GD32F30x/Source/syscalls.c \
../Firmware/CMSIS/GD/GD32F30x/Source/system_gd32f30x.c 

C_DEPS += \
./Firmware/CMSIS/GD/GD32F30x/Source/syscalls.d \
./Firmware/CMSIS/GD/GD32F30x/Source/system_gd32f30x.d 

OBJS += \
./Firmware/CMSIS/GD/GD32F30x/Source/syscalls.o \
./Firmware/CMSIS/GD/GD32F30x/Source/system_gd32f30x.o 


# Each subdirectory must supply rules for building sources it contributes
Firmware/CMSIS/GD/GD32F30x/Source/%.o: ../Firmware/CMSIS/GD/GD32F30x/Source/%.c Firmware/CMSIS/GD/GD32F30x/Source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GD ARM MCU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu11 -DGD32F30X_HD -DGD_ECLIPSE_GCC -DUSE_STDPERIPH_DRIVER -I"../Firmware/CMSIS/" -I"../Firmware/CMSIS/GD/GD32F30x/Include/" -I"../Firmware/GD32F30x_standard_peripheral/Include/" -I"../inc/" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -Wa,-adhlns=$@.lst   -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-Firmware-2f-CMSIS-2f-GD-2f-GD32F30x-2f-Source

clean-Firmware-2f-CMSIS-2f-GD-2f-GD32F30x-2f-Source:
	-$(RM) ./Firmware/CMSIS/GD/GD32F30x/Source/syscalls.d ./Firmware/CMSIS/GD/GD32F30x/Source/syscalls.o ./Firmware/CMSIS/GD/GD32F30x/Source/system_gd32f30x.d ./Firmware/CMSIS/GD/GD32F30x/Source/system_gd32f30x.o

.PHONY: clean-Firmware-2f-CMSIS-2f-GD-2f-GD32F30x-2f-Source

