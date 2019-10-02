################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Static_Code/System/subdir.mk
-include Sources/subdir.mk
-include Project_Settings/Startup_Code/subdir.mk
-include Generated_Code/subdir.mk
-include .metadata/.plugins/org.eclipse.cdt.make.core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
MCCAR_V_tests.hex \

SECONDARY_LIST += \
MCCAR_V_tests.lst \

SECONDARY_SIZE += \
MCCAR_V_tests.siz \


# All Target
all: MCCAR_V_tests.elf secondary-outputs

# Tool invocations
MCCAR_V_tests.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Project_Settings/Linker_Files/ProcessorExpert.ld" -Xlinker --gc-sections -L"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\lib\Debug" -Wl,-Map,"MCCAR_V_tests.map" -specs=nano.specs -specs=nosys.specs -o "MCCAR_V_tests.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

MCCAR_V_tests.hex: MCCAR_V_tests.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "MCCAR_V_tests.elf"  "MCCAR_V_tests.hex"
	@echo 'Finished building: $@'
	@echo ' '

MCCAR_V_tests.lst: MCCAR_V_tests.elf
	@echo 'Invoking: Cross ARM GNU Create Listing'
	arm-none-eabi-objdump --source --all-headers --demangle --line-numbers --wide "MCCAR_V_tests.elf" > "MCCAR_V_tests.lst"
	@echo 'Finished building: $@'
	@echo ' '

MCCAR_V_tests.siz: MCCAR_V_tests.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=sysv "MCCAR_V_tests.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(C++_DEPS)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(CC_DEPS)$(SECONDARY_FLASH)$(CPP_DEPS)$(CXX_DEPS)$(C_UPPER_DEPS)$(SECONDARY_LIST)$(S_UPPER_DEPS) MCCAR_V_tests.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
