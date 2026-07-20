################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/cdc_jobs.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/kinematics.c \
../Core/Src/main.c \
../Core/Src/motion.c \
../Core/Src/nextion_uart.c \
../Core/Src/odrive_link.c \
../Core/Src/odrive_uart.c \
../Core/Src/packet_protocol.c \
../Core/Src/quadspi.c \
../Core/Src/rtc.c \
../Core/Src/sequence.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/trapezoid.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/cdc_jobs.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/kinematics.o \
./Core/Src/main.o \
./Core/Src/motion.o \
./Core/Src/nextion_uart.o \
./Core/Src/odrive_link.o \
./Core/Src/odrive_uart.o \
./Core/Src/packet_protocol.o \
./Core/Src/quadspi.o \
./Core/Src/rtc.o \
./Core/Src/sequence.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/trapezoid.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/cdc_jobs.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/kinematics.d \
./Core/Src/main.d \
./Core/Src/motion.d \
./Core/Src/nextion_uart.d \
./Core/Src/odrive_link.d \
./Core/Src/odrive_uart.d \
./Core/Src/packet_protocol.d \
./Core/Src/quadspi.d \
./Core/Src/rtc.d \
./Core/Src/sequence.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/trapezoid.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/cdc_jobs.cyclo ./Core/Src/cdc_jobs.d ./Core/Src/cdc_jobs.o ./Core/Src/cdc_jobs.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/kinematics.cyclo ./Core/Src/kinematics.d ./Core/Src/kinematics.o ./Core/Src/kinematics.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motion.cyclo ./Core/Src/motion.d ./Core/Src/motion.o ./Core/Src/motion.su ./Core/Src/nextion_uart.cyclo ./Core/Src/nextion_uart.d ./Core/Src/nextion_uart.o ./Core/Src/nextion_uart.su ./Core/Src/odrive_link.cyclo ./Core/Src/odrive_link.d ./Core/Src/odrive_link.o ./Core/Src/odrive_link.su ./Core/Src/odrive_uart.cyclo ./Core/Src/odrive_uart.d ./Core/Src/odrive_uart.o ./Core/Src/odrive_uart.su ./Core/Src/packet_protocol.cyclo ./Core/Src/packet_protocol.d ./Core/Src/packet_protocol.o ./Core/Src/packet_protocol.su ./Core/Src/quadspi.cyclo ./Core/Src/quadspi.d ./Core/Src/quadspi.o ./Core/Src/quadspi.su ./Core/Src/rtc.cyclo ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/rtc.su ./Core/Src/sequence.cyclo ./Core/Src/sequence.d ./Core/Src/sequence.o ./Core/Src/sequence.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/trapezoid.cyclo ./Core/Src/trapezoid.d ./Core/Src/trapezoid.o ./Core/Src/trapezoid.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

