################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../28265785-e314-proj/Demo1/Core/Startup/startup_stm32f411retx.s 

OBJS += \
./28265785-e314-proj/Demo1/Core/Startup/startup_stm32f411retx.o 

S_DEPS += \
./28265785-e314-proj/Demo1/Core/Startup/startup_stm32f411retx.d 


# Each subdirectory must supply rules for building sources it contributes
28265785-e314-proj/Demo1/Core/Startup/%.o: ../28265785-e314-proj/Demo1/Core/Startup/%.s 28265785-e314-proj/Demo1/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-28265785-2d-e314-2d-proj-2f-Demo1-2f-Core-2f-Startup

clean-28265785-2d-e314-2d-proj-2f-Demo1-2f-Core-2f-Startup:
	-$(RM) ./28265785-e314-proj/Demo1/Core/Startup/startup_stm32f411retx.d ./28265785-e314-proj/Demo1/Core/Startup/startup_stm32f411retx.o

.PHONY: clean-28265785-2d-e314-2d-proj-2f-Demo1-2f-Core-2f-Startup

