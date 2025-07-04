################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32_AI_AudioPreprocessing_Library/Src/common_tables.c \
../Drivers/STM32_AI_AudioPreprocessing_Library/Src/dct.c \
../Drivers/STM32_AI_AudioPreprocessing_Library/Src/feature_extraction.c \
../Drivers/STM32_AI_AudioPreprocessing_Library/Src/mel_filterbank.c \
../Drivers/STM32_AI_AudioPreprocessing_Library/Src/window.c 

OBJS += \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/common_tables.o \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/dct.o \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/feature_extraction.o \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/mel_filterbank.o \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/window.o 

C_DEPS += \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/common_tables.d \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/dct.d \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/feature_extraction.d \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/mel_filterbank.d \
./Drivers/STM32_AI_AudioPreprocessing_Library/Src/window.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32_AI_AudioPreprocessing_Library/Src/common_tables.o: ../Drivers/STM32_AI_AudioPreprocessing_Library/Src/common_tables.c Drivers/STM32_AI_AudioPreprocessing_Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DARM_MATH_CM7 -DDEBUG '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-AI/App -I../X-CUBE-AI -I../Middlewares/ST/AI/Inc -I../Drivers/STM32_AI_AudioPreprocessing_Library/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32_AI_AudioPreprocessing_Library/Src/common_tables.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32_AI_AudioPreprocessing_Library/Src/dct.o: ../Drivers/STM32_AI_AudioPreprocessing_Library/Src/dct.c Drivers/STM32_AI_AudioPreprocessing_Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DARM_MATH_CM7 -DDEBUG '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-AI/App -I../X-CUBE-AI -I../Middlewares/ST/AI/Inc -I../Drivers/STM32_AI_AudioPreprocessing_Library/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32_AI_AudioPreprocessing_Library/Src/dct.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32_AI_AudioPreprocessing_Library/Src/feature_extraction.o: ../Drivers/STM32_AI_AudioPreprocessing_Library/Src/feature_extraction.c Drivers/STM32_AI_AudioPreprocessing_Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DARM_MATH_CM7 -DDEBUG '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-AI/App -I../X-CUBE-AI -I../Middlewares/ST/AI/Inc -I../Drivers/STM32_AI_AudioPreprocessing_Library/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32_AI_AudioPreprocessing_Library/Src/feature_extraction.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32_AI_AudioPreprocessing_Library/Src/mel_filterbank.o: ../Drivers/STM32_AI_AudioPreprocessing_Library/Src/mel_filterbank.c Drivers/STM32_AI_AudioPreprocessing_Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DARM_MATH_CM7 -DDEBUG '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-AI/App -I../X-CUBE-AI -I../Middlewares/ST/AI/Inc -I../Drivers/STM32_AI_AudioPreprocessing_Library/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32_AI_AudioPreprocessing_Library/Src/mel_filterbank.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32_AI_AudioPreprocessing_Library/Src/window.o: ../Drivers/STM32_AI_AudioPreprocessing_Library/Src/window.c Drivers/STM32_AI_AudioPreprocessing_Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DARM_MATH_CM7 -DDEBUG '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-AI/App -I../X-CUBE-AI -I../Middlewares/ST/AI/Inc -I../Drivers/STM32_AI_AudioPreprocessing_Library/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32_AI_AudioPreprocessing_Library/Src/window.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

