################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Static_Code/System/CPU_Init.c \
../Static_Code/System/Peripherals_Init.c \
../Static_Code/System/Vectors.c 

OBJS += \
./Static_Code/System/CPU_Init.o \
./Static_Code/System/Peripherals_Init.o \
./Static_Code/System/Vectors.o 

C_DEPS += \
./Static_Code/System/CPU_Init.d \
./Static_Code/System/Peripherals_Init.d \
./Static_Code/System/Vectors.d 


# Each subdirectory must supply rules for building sources it contributes
Static_Code/System/%.o: ../Static_Code/System/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DKDS_BUILD=1 -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\lib\include\DrivingControl" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Static_Code\PDD" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Static_Code\IO_Map" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Sources" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Generated_Code" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Static_Code\System" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Static_Code/System" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Static_Code/PDD" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Static_Code/IO_Map" -I"C:\Freescale\KDS_v3\eclipse\ProcessorExpert/lib/Kinetis/pdd/inc" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Sources" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


