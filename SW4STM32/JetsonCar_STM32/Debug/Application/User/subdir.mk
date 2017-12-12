################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/freertos.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/main.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/stm32f4xx_hal_msp.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/stm32f4xx_hal_timebase_TIM.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/stm32f4xx_it.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/usb_device.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/usbd_cdc_if.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/usbd_conf.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/usbd_desc.c 

OBJS += \
./Application/User/freertos.o \
./Application/User/main.o \
./Application/User/stm32f4xx_hal_msp.o \
./Application/User/stm32f4xx_hal_timebase_TIM.o \
./Application/User/stm32f4xx_it.o \
./Application/User/usb_device.o \
./Application/User/usbd_cdc_if.o \
./Application/User/usbd_conf.o \
./Application/User/usbd_desc.o 

C_DEPS += \
./Application/User/freertos.d \
./Application/User/main.d \
./Application/User/stm32f4xx_hal_msp.d \
./Application/User/stm32f4xx_hal_timebase_TIM.d \
./Application/User/stm32f4xx_it.d \
./Application/User/usb_device.d \
./Application/User/usbd_cdc_if.d \
./Application/User/usbd_conf.d \
./Application/User/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/freertos.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/freertos.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/freertos.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/main.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/main.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_hal_msp.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/stm32f4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/stm32f4xx_hal_msp.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_hal_timebase_TIM.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/stm32f4xx_hal_timebase_TIM.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/stm32f4xx_hal_timebase_TIM.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_it.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/stm32f4xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/stm32f4xx_it.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/usb_device.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/usb_device.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/usb_device.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/usbd_cdc_if.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/usbd_cdc_if.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/usbd_cdc_if.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/usbd_conf.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/usbd_conf.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/usbd_conf.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/usbd_desc.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Src/usbd_desc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/usbd_desc.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


