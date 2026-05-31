################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BaseSystemInterface.c \
../Core/Src/Controller.c \
../Core/Src/DCMotor.c \
../Core/Src/Gripper.c \
../Core/Src/JoystickInterface.c \
../Core/Src/KalmanFilterDCMotor.c \
../Core/Src/MD20A.c \
../Core/Src/ModbusRTU.c \
../Core/Src/PWM.c \
../Core/Src/QEI.c \
../Core/Src/Robot.c \
../Core/Src/SCurve.c \
../Core/Src/SerialFrame.c \
../Core/Src/TaskManager.c \
../Core/Src/Trapezoid.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/BaseSystemInterface.o \
./Core/Src/Controller.o \
./Core/Src/DCMotor.o \
./Core/Src/Gripper.o \
./Core/Src/JoystickInterface.o \
./Core/Src/KalmanFilterDCMotor.o \
./Core/Src/MD20A.o \
./Core/Src/ModbusRTU.o \
./Core/Src/PWM.o \
./Core/Src/QEI.o \
./Core/Src/Robot.o \
./Core/Src/SCurve.o \
./Core/Src/SerialFrame.o \
./Core/Src/TaskManager.o \
./Core/Src/Trapezoid.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/BaseSystemInterface.d \
./Core/Src/Controller.d \
./Core/Src/DCMotor.d \
./Core/Src/Gripper.d \
./Core/Src/JoystickInterface.d \
./Core/Src/KalmanFilterDCMotor.d \
./Core/Src/MD20A.d \
./Core/Src/ModbusRTU.d \
./Core/Src/PWM.d \
./Core/Src/QEI.d \
./Core/Src/Robot.d \
./Core/Src/SCurve.d \
./Core/Src/SerialFrame.d \
./Core/Src/TaskManager.d \
./Core/Src/Trapezoid.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BaseSystemInterface.cyclo ./Core/Src/BaseSystemInterface.d ./Core/Src/BaseSystemInterface.o ./Core/Src/BaseSystemInterface.su ./Core/Src/Controller.cyclo ./Core/Src/Controller.d ./Core/Src/Controller.o ./Core/Src/Controller.su ./Core/Src/DCMotor.cyclo ./Core/Src/DCMotor.d ./Core/Src/DCMotor.o ./Core/Src/DCMotor.su ./Core/Src/Gripper.cyclo ./Core/Src/Gripper.d ./Core/Src/Gripper.o ./Core/Src/Gripper.su ./Core/Src/JoystickInterface.cyclo ./Core/Src/JoystickInterface.d ./Core/Src/JoystickInterface.o ./Core/Src/JoystickInterface.su ./Core/Src/KalmanFilterDCMotor.cyclo ./Core/Src/KalmanFilterDCMotor.d ./Core/Src/KalmanFilterDCMotor.o ./Core/Src/KalmanFilterDCMotor.su ./Core/Src/MD20A.cyclo ./Core/Src/MD20A.d ./Core/Src/MD20A.o ./Core/Src/MD20A.su ./Core/Src/ModbusRTU.cyclo ./Core/Src/ModbusRTU.d ./Core/Src/ModbusRTU.o ./Core/Src/ModbusRTU.su ./Core/Src/PWM.cyclo ./Core/Src/PWM.d ./Core/Src/PWM.o ./Core/Src/PWM.su ./Core/Src/QEI.cyclo ./Core/Src/QEI.d ./Core/Src/QEI.o ./Core/Src/QEI.su ./Core/Src/Robot.cyclo ./Core/Src/Robot.d ./Core/Src/Robot.o ./Core/Src/Robot.su ./Core/Src/SCurve.cyclo ./Core/Src/SCurve.d ./Core/Src/SCurve.o ./Core/Src/SCurve.su ./Core/Src/SerialFrame.cyclo ./Core/Src/SerialFrame.d ./Core/Src/SerialFrame.o ./Core/Src/SerialFrame.su ./Core/Src/TaskManager.cyclo ./Core/Src/TaskManager.d ./Core/Src/TaskManager.o ./Core/Src/TaskManager.su ./Core/Src/Trapezoid.cyclo ./Core/Src/Trapezoid.d ./Core/Src/Trapezoid.o ./Core/Src/Trapezoid.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

