################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/startup_stm32f091xc.s 

C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_dma.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_gpio.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_it.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_rtc.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_timers.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_uart.c 

OBJS += \
./H23Rx/H23Rx.o \
./H23Rx/H23Rx_dma.o \
./H23Rx/H23Rx_gpio.o \
./H23Rx/H23Rx_it.o \
./H23Rx/H23Rx_rtc.o \
./H23Rx/H23Rx_timers.o \
./H23Rx/H23Rx_uart.o \
./H23Rx/startup_stm32f091xc.o 

S_DEPS += \
./H23Rx/startup_stm32f091xc.d 

C_DEPS += \
./H23Rx/H23Rx.d \
./H23Rx/H23Rx_dma.d \
./H23Rx/H23Rx_gpio.d \
./H23Rx/H23Rx_it.d \
./H23Rx/H23Rx_rtc.d \
./H23Rx/H23Rx_timers.d \
./H23Rx/H23Rx_uart.d 


# Each subdirectory must supply rules for building sources it contributes
H23Rx/H23Rx.o: D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx.c H23Rx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH23R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H23Rx -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H23Rx/H23Rx.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H23Rx/H23Rx_dma.o: D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_dma.c H23Rx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH23R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H23Rx -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H23Rx/H23Rx_dma.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H23Rx/H23Rx_gpio.o: D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_gpio.c H23Rx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH23R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H23Rx -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H23Rx/H23Rx_gpio.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H23Rx/H23Rx_it.o: D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_it.c H23Rx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH23R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H23Rx -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H23Rx/H23Rx_it.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H23Rx/H23Rx_rtc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_rtc.c H23Rx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH23R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H23Rx -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H23Rx/H23Rx_rtc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H23Rx/H23Rx_timers.o: D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_timers.c H23Rx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH23R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H23Rx -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H23Rx/H23Rx_timers.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H23Rx/H23Rx_uart.o: D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/H23Rx_uart.c H23Rx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH23R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H23Rx -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H23Rx/H23Rx_uart.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H23Rx/startup_stm32f091xc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H23R0x/H23Rx/startup_stm32f091xc.s H23Rx/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -MMD -MP -MF"H23Rx/startup_stm32f091xc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@" "$<"

