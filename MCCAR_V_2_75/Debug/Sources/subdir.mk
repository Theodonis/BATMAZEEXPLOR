################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/ADC.c \
../Sources/Application.c \
../Sources/DrivingExplore_Interface.c \
../Sources/Encoder.c \
../Sources/Event.c \
../Sources/Events.c \
../Sources/Exploration_Drive.c \
../Sources/Explore.c \
../Sources/Exploring_Com.c \
../Sources/ExplororeDrivingControll.c \
../Sources/FSM.c \
../Sources/IMU.c \
../Sources/Logging.c \
../Sources/MazeHndl.c \
../Sources/Motor.c \
../Sources/PID.c \
../Sources/Platform.c \
../Sources/TargetInField_Position.c \
../Sources/main.c 

OBJS += \
./Sources/ADC.o \
./Sources/Application.o \
./Sources/DrivingExplore_Interface.o \
./Sources/Encoder.o \
./Sources/Event.o \
./Sources/Events.o \
./Sources/Exploration_Drive.o \
./Sources/Explore.o \
./Sources/Exploring_Com.o \
./Sources/ExplororeDrivingControll.o \
./Sources/FSM.o \
./Sources/IMU.o \
./Sources/Logging.o \
./Sources/MazeHndl.o \
./Sources/Motor.o \
./Sources/PID.o \
./Sources/Platform.o \
./Sources/TargetInField_Position.o \
./Sources/main.o 

C_DEPS += \
./Sources/ADC.d \
./Sources/Application.d \
./Sources/DrivingExplore_Interface.d \
./Sources/Encoder.d \
./Sources/Event.d \
./Sources/Events.d \
./Sources/Exploration_Drive.d \
./Sources/Explore.d \
./Sources/Exploring_Com.d \
./Sources/ExplororeDrivingControll.d \
./Sources/FSM.d \
./Sources/IMU.d \
./Sources/Logging.d \
./Sources/MazeHndl.d \
./Sources/Motor.d \
./Sources/PID.d \
./Sources/Platform.d \
./Sources/TargetInField_Position.d \
./Sources/main.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DKDS_BUILD=1 -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\lib\include\DrivingControl" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Static_Code\PDD" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Static_Code\IO_Map" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Sources" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Generated_Code" -I"C:\Users\Theo\Documents\Fachhochschule\Sem_9\BAT\3_Software\git\BATMAZEEXPLOR\MCCAR_V_2_75\Static_Code\System" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Static_Code/System" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Static_Code/PDD" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Static_Code/IO_Map" -I"C:\Freescale\KDS_v3\eclipse\ProcessorExpert/lib/Kinetis/pdd/inc" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Sources" -I"C:/Users/Theo/Documents/Fachhochschule/Sem_9/BAT/3_Software/git/BATMAZEEXPLOR/MCCAR_V_2_75/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


