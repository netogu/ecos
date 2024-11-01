#------------------------------------------------+
#  Target: ecos
#  MCU: STM32G474xx
#  Board:
#------------------------------------------------+
TARGET = ecos
DEVICE = STM32G474xx

#------------------------------------------------+
# Build Variables
#------------------------------------------------+
DEBUG = 1
OPT = -Og

#------------------------------------------------+
# Debuger
#------------------------------------------------+
DEBUGER = openocd
DEBUGER_CONF = src/drivers/stm32g4/scripts/stm32g4_openocd.cfg
GDB = arm-none-eabi-gdb

#------------------------------------------------+
# Paths
#------------------------------------------------+
BUILD_DIR = build
TINYUSB = src/external/tinyusb/src
MICROSHELL = src/external/microshell/src
TINYPRINTF = src/external/tiny_printf
FREERTOS = src/external/freertos

#------------------------------------------------+
# Sources
#------------------------------------------------+
C_SOURCES = $(wildcard src/board/stm32g4/*.c)
C_SOURCES += $(wildcard src/main.c) 
C_SOURCES += $(wildcard src/board/*.c) 
C_SOURCES += $(wildcard src/hal/*.c) 
C_SOURCES += $(wildcard src/shell/*.c) 
C_SOURCES += $(wildcard src/rtos/*.c) 
C_SOURCES += $(wildcard src/tasks/*.c) 
C_SOURCES += $(wildcard src/drivers/*.c) 
C_SOURCES += $(wildcard src/drivers/stm32g4/*.c) 
C_SOURCES += $(wildcard src/drivers/power/*.c) 
C_SOURCES += $(wildcard src/lib/*.c)
C_SOURCES += $(wildcard $(TINYPRINTF)/*.c)
C_SOURCES += $(wildcard $(TINYUSB)/tusb.c)
C_SOURCES += $(wildcard $(TINYUSB)/common/*.c)
C_SOURCES += $(wildcard $(TINYUSB)/device/*.c)
C_SOURCES += $(wildcard $(TINYUSB)/typec/*.c)
C_SOURCES += $(wildcard $(TINYUSB)/class/cdc/*.c)
C_SOURCES += $(wildcard $(TINYUSB)/portable/st/stm32_fsdev/dcd_stm32_fsdev.c)
C_SOURCES += $(wildcard $(TINYUSB)/portable/st/typec/typec_stm32.c)
C_SOURCES += $(wildcard $(MICROSHELL)/src/*.c)
C_SOURCES += $(wildcard $(MICROSHELL)/src/commands/*.c)
C_SOURCES += $(wildcard $(FREERTOS)/*.c)
C_SOURCES += $(FREERTOS)/portable/GCC/ARM_CM4F/port.c
C_SOURCES += $(FREERTOS)/portable/MemMang/heap_3.c

#------------------------------------------------+
# ASM Sources
#------------------------------------------------+
ASM_SOURCES =  \
src/drivers/stm32g4/stm32g4_startup.s

#------------------------------------------------+
# Includes
#------------------------------------------------+
AS_INCLUDES = 

C_INCLUDES =  \
-Isrc \
-Isrc/drivers \
-Isrc/drivers/stm32g4 \
-Isrc/shell \
-Isrc/board \
-Isrc/hal \
-Isrc/rtos \
-Isrc/tasks \
-Isrc/external/CMSIS/Device/ST/STM32G4xx/Include \
-Isrc/external/CMSIS/Include \
-Isrc/external/CMSIS-DSP/Include \
-I$(TINYUSB)  \
-I$(TINYPRINTF) \
-I$(MICROSHELL) \
-I$(MICROSHELL)/inc \
-I$(FREERTOS)/include \
-I$(FREERTOS)/portable/GCC/ARM_CM4F \

#------------------------------------------------+
# Linker 
#------------------------------------------------+
LDSCRIPT = src/drivers/stm32g4/scripts/stm32g474_linker_script.ld
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
LDFLAGS += -nostartfiles
LDFLAGS += --specs=nano.specs --specs=nosys.specs -u _print_float -u _scanf_float

#------------------------------------------------+
# Toolchain
#------------------------------------------------+
PREFIX = arm-none-eabi-
# PREFIX = ~/Dev/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#------------------------------------------------+
# Compiler
#------------------------------------------------+
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI) -D$(DEVICE)

C_DEFS =  \
-DSTM32G474xx -std=c99 \
-fshort-enums \
-Wall -fdata-sections -ffunction-sections \
-ffast-math \
-Ofast \
-Wall -Wextra -Werror \
-Wdouble-promotion \
-Wsign-compare \
#-fshort-wchar \

AS_DEFS = \
-Wall -fdata-sections -ffunction-sections

ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) 
CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) 

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += -nostdlib

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#------------------------------------------------+
# Build
#------------------------------------------------+

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@ echo "  CC    " $<
	@ $(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@ $(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@ echo "  LD    " $(notdir $@)
	@ $(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@ $(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@ $(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@ $(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#------------------------------------------------+
# Size of modules
#------------------------------------------------+
binsize: $(OBJECTS)
	@ echo "Size of modules:"
	@ $(SZ) $(OBJECTS)
	@ echo "Size of firmware:"
	@ $(SZ) $(BUILD_DIR)/$(TARGET).elf

#------------------------------------------------+
# Clean
#------------------------------------------------+
clean:
	-rm -fR $(BUILD_DIR)
  
#------------------------------------------------+
# Flash
#------------------------------------------------+
flash:
	$(DEBUGER) -f $(DEBUGER_CONF) -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

#------------------------------------------------+
# Debug
#------------------------------------------------+
debug:
	$(DEBUGER) -f $(DEBUGER_CONF) & $(GDB) -ex 'target extended-remote localhost:3333' $(BUILD_DIR)/$(TARGET).elf

#------------------------------------------------+
# Dependencies
#------------------------------------------------+
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
