################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include CMSIS/efm32tg/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SIRKA_EFM32TG110_DRA.axf

# Tool invocations
SIRKA_EFM32TG110_DRA.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb  --specs=nano.specs -Xlinker --gc-sections -Xlinker -Map="SIRKA_EFM32TG110_DRA.map" -T "C:\SiliconLabs\SimplicityStudio\v2\workspace\SIRKA_EFM32TG110_DRA\GNU ARM 4.7.3 - Release\SIRKA_EFM32TG110_DRA - 16KB.ld" -o SIRKA_EFM32TG110_DRA.axf "./src/BMX055.o" "./src/calc.o" "./src/crc.o" "./src/debug.o" "./src/delay.o" "./src/emu.o" "./src/inits.o" "./src/irq.o" "./src/msc.o" "./src/sirka.o" "./src/spi.o" "./src/timer.o" "./src/usart.o" "./src/watchdog.o" "./emlib/em_gpio.o" "./emlib/em_system.o" "./CMSIS/efm32tg/startup_gcc_efm32tg.o" "./CMSIS/efm32tg/system_efm32tg.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

	@echo 'Building hex file: SIRKA_EFM32TG110_DRA.hex'
	arm-none-eabi-objcopy -O ihex "SIRKA_EFM32TG110_DRA.axf" "SIRKA_EFM32TG110_DRA.hex"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "SIRKA_EFM32TG110_DRA.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) SIRKA_EFM32TG110_DRA.axf
	-@echo ' '

post-build:
	-arm-none-eabi-objcopy -O binary "SIRKA_EFM32TG110_DRA.axf" "SIRKA_EFM32TG110_DRA.bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
