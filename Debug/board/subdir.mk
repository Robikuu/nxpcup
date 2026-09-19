################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../board/board.c \
../board/clock_config.c \
../board/hardware_init.c \
../board/peripherals.c \
../board/pin_mux.c 

C_DEPS += \
./board/board.d \
./board/clock_config.d \
./board/hardware_init.d \
./board/peripherals.d \
./board/pin_mux.d 

OBJS += \
./board/board.o \
./board/clock_config.o \
./board/hardware_init.o \
./board/peripherals.o \
./board/pin_mux.o 


# Each subdirectory must supply rules for building sources it contributes
board/%.o: ../board/%.c board/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_MCXN947VDF -DCPU_MCXN947VDF_cm33 -DCPU_MCXN947VDF_cm33_core0 -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=0 -DMCUX_META_BUILD -DMCXN947_cm33_core0_SERIES -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_OS_BAREMETAL -D__NEWLIB__ -I"D:\robotica\workspace\nxpcup-ciorba_curata\drivers" -I"D:\robotica\workspace\nxpcup-ciorba_curata\CMSIS" -I"D:\robotica\workspace\nxpcup-ciorba_curata\CMSIS\m-profile" -I"D:\robotica\workspace\nxpcup-ciorba_curata\device" -I"D:\robotica\workspace\nxpcup-ciorba_curata\device\periph" -I"D:\robotica\workspace\nxpcup-ciorba_curata\utilities" -I"D:\robotica\workspace\nxpcup-ciorba_curata\utilities\str" -I"D:\robotica\workspace\nxpcup-ciorba_curata\utilities\debug_console_lite" -I"D:\robotica\workspace\nxpcup-ciorba_curata\component\uart" -I"D:\robotica\workspace\nxpcup-ciorba_curata\CMSIS_driver\Include" -I"D:\robotica\workspace\nxpcup-ciorba_curata\board" -I"D:\robotica\workspace\nxpcup-ciorba_curata\source" -I"D:\robotica\workspace\nxpcup-ciorba_curata\include" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -fno-builtin -imacros "D:\robotica\workspace\nxpcup-ciorba_curata\source\mcux_config.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-board

clean-board:
	-$(RM) ./board/board.d ./board/board.o ./board/clock_config.d ./board/clock_config.o ./board/hardware_init.d ./board/hardware_init.o ./board/peripherals.d ./board/peripherals.o ./board/pin_mux.d ./board/pin_mux.o

.PHONY: clean-board

