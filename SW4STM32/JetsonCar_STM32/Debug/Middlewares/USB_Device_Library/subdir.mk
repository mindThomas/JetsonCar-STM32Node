################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./Middlewares/USB_Device_Library/usbd_cdc.o \
./Middlewares/USB_Device_Library/usbd_core.o \
./Middlewares/USB_Device_Library/usbd_ctlreq.o \
./Middlewares/USB_Device_Library/usbd_ioreq.o 

C_DEPS += \
./Middlewares/USB_Device_Library/usbd_cdc.d \
./Middlewares/USB_Device_Library/usbd_core.d \
./Middlewares/USB_Device_Library/usbd_ctlreq.d \
./Middlewares/USB_Device_Library/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/USB_Device_Library/usbd_cdc.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/USB_Device_Library/usbd_cdc.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Device_Library/usbd_core.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/USB_Device_Library/usbd_core.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Device_Library/usbd_ctlreq.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/USB_Device_Library/usbd_ctlreq.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Device_Library/usbd_ioreq.o: C:/Users/Thomas/Dropbox/_Uni\ AAU/P9/Self-driving\ RC\ car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DHAL_UART_MODULE_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F405xx -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/VL53L0X" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Libraries/XV11_LiDAR" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Thomas/Dropbox/_Uni AAU/P9/Self-driving RC car/STM32/Projects/JetsonCar_STM32/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/USB_Device_Library/usbd_ioreq.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


