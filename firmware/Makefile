##############################################################################
# Makefile — UAVCAN-to-CAN ESC bridge firmware for STM32F405/407
#
# Prerequisites
# -------------
#   arm-none-eabi-gcc toolchain (apt install gcc-arm-none-eabi)
#   STM32CubeF4 package (set CUBE_PATH below or pass on command line)
#
# Usage
#   make              — build firmware ELF
#   make flash        — flash via OpenOCD (ST-Link)
#   make clean        — remove build artefacts
##############################################################################

TARGET    := uav_can_bridge
BUILD_DIR := build

# Path to the STM32CubeF4 package root
CUBE_PATH ?= $(HOME)/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1

# ---- Toolchain -----------------------------------------------------------
CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

# ---- MCU flags -----------------------------------------------------------
MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# ---- Source files --------------------------------------------------------
SRC_DIRS := Core/Src

SRCS := \
  Core/Src/main.c            \
  Core/Src/canard.c          \
  Core/Src/uavcan_esc.c      \
  Core/Src/can_bridge.c      \
  Core/Src/bsp_stm32f4.c

# STM32 HAL sources (only the ones we actually use)
HAL_SRC := $(CUBE_PATH)/Drivers/STM32F4xx_HAL_Driver/Src
SRCS += \
  $(HAL_SRC)/stm32f4xx_hal.c          \
  $(HAL_SRC)/stm32f4xx_hal_can.c      \
  $(HAL_SRC)/stm32f4xx_hal_gpio.c     \
  $(HAL_SRC)/stm32f4xx_hal_rcc.c      \
  $(HAL_SRC)/stm32f4xx_hal_rcc_ex.c   \
  $(HAL_SRC)/stm32f4xx_hal_cortex.c   \
  $(HAL_SRC)/stm32f4xx_hal_pwr_ex.c

# CMSIS startup
SRCS += $(CUBE_PATH)/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s

# System init (provides SystemInit and SystemCoreClock)
SRCS += $(CUBE_PATH)/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c

# ---- Include paths -------------------------------------------------------
INCS := \
  -ICore/Inc \
  -I$(CUBE_PATH)/Drivers/STM32F4xx_HAL_Driver/Inc \
  -I$(CUBE_PATH)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
  -I$(CUBE_PATH)/Drivers/CMSIS/Include

# ---- Preprocessor defines ------------------------------------------------
DEFS := -DSTM32F405xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000U

# ---- Linker script -------------------------------------------------------
LDSCRIPT := STM32F405RGTx_FLASH.ld

# ---- Compiler/linker flags -----------------------------------------------
CFLAGS  := $(MCU) $(DEFS) $(INCS) \
            -std=c11 -Wall -Wextra -Wpedantic \
            -ffunction-sections -fdata-sections \
            -O2 -g

LDFLAGS := $(MCU) -specs=nosys.specs -specs=nano.specs \
            -T$(LDSCRIPT) \
            -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
            -Wl,--gc-sections

# ---- Object files --------------------------------------------------------
OBJS := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(basename $(SRCS))))

# ---- Rules ---------------------------------------------------------------
.PHONY: all clean flash

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $<

$(BUILD_DIR)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	        -c "program $< 0x08000000 verify reset exit"
