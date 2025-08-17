################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include gcc_startup/subdir.mk
-include Firmware/GD32F30x_standard_peripheral/Source/subdir.mk
-include Firmware/CMSIS/GD/GD32F30x/Source/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := CAN_TEST
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH_HEX += \
CAN_TEST.hex \

SECONDARY_FLASH_BIN += \
CAN_TEST.bin \

SECONDARY_LIST += \
CAN_TEST.lst \

SECONDARY_SIZE += \
CAN_TEST.siz \


# All Target
all: main-build

# Main-build Target
main-build: CAN_TEST.elf secondary-outputs

# Tool invocations
CAN_TEST.elf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GD ARM MCU C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -Wl,-Map,"CAN_TEST.map" -T gd32f30x_flash.ld -Xlinker --gc-sections -L"../ldscripts/" --specs=nano.specs --specs=nosys.specs -o "CAN_TEST.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

CAN_TEST.hex: CAN_TEST.elf makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GD ARM MCU Flash Image(Hex)'
	arm-none-eabi-objcopy -O ihex  "CAN_TEST.elf"  "CAN_TEST.hex"
	@echo 'Finished building: $@'
	@echo ' '

CAN_TEST.bin: CAN_TEST.elf makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GD ARM MCU Flash Image(Bin)'
	arm-none-eabi-objcopy -O binary  "CAN_TEST.elf"  "CAN_TEST.bin"
	@echo 'Finished building: $@'
	@echo ' '

CAN_TEST.lst: CAN_TEST.elf makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GD ARM MCU Listing'
	arm-none-eabi-objdump --source --all-headers --demangle --line-numbers --wide "CAN_TEST.elf" > "CAN_TEST.lst"
	@echo 'Finished building: $@'
	@echo ' '

CAN_TEST.siz: CAN_TEST.elf makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GD ARM MCU Print Size'
	arm-none-eabi-size --format=berkeley "CAN_TEST.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) CAN_TEST.bin CAN_TEST.elf CAN_TEST.hex CAN_TEST.lst CAN_TEST.siz
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH_HEX) $(SECONDARY_FLASH_BIN) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents main-build

-include ../makefile.targets
