################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/gd32f307c_eval.c \
../src/gd32f30x_it.c \
../src/main.c \
../src/systick.c 

C_DEPS += \
./src/gd32f307c_eval.d \
./src/gd32f30x_it.d \
./src/main.d \
./src/systick.d 

OBJS += \
./src/gd32f307c_eval.o \
./src/gd32f30x_it.o \
./src/main.o \
./src/systick.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GD ARM MCU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu11 -DGD32F30X_HD -DGD_ECLIPSE_GCC -DUSE_STDPERIPH_DRIVER -I"../Firmware/CMSIS/" -I"../Firmware/CMSIS/GD/GD32F30x/Include/" -I"../Firmware/GD32F30x_standard_peripheral/Include/" -I"../inc/" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -Wa,-adhlns=$@.lst   -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/gd32f307c_eval.d ./src/gd32f307c_eval.o ./src/gd32f30x_it.d ./src/gd32f30x_it.o ./src/main.d ./src/main.o ./src/systick.d ./src/systick.o

.PHONY: clean-src

