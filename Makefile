MODULE = bluepill
OPENCM3_DIR = external/libopencm3
PROJECT = app_$(MODULE)
BUILD_DIR = build
SRC_DIR = src

TARGET ?= stm32/f1
DEVICE = stm32f103c8t6

SRC = $(wildcard src/*.c)
OBJS = $(SRC:.c=.o)

CFLAGS          += -Os -ggdb3
CFLAGS          += -std=c17
CPPFLAGS        += -MD
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

# Enable color loggink
CFLAGS          += -DLOG_ENABLE_COLORS=1


include $(OPENCM3_DIR)/mk/gcc-config.mk
# Linker File Autogeneration
include $(OPENCM3_DIR)/mk/genlink-config.mk
# LDSCRIPT = src/generated.$(DEVICE).ld
LDSCRIPT = $(SRC_DIR)/stm32f103c8t6.ld


.PHONY: all bin lib lib-clean flash clean

all: lib
	$(MAKE) bin

bin:  $(BUILD_DIR)/$(PROJECT).bin
	@echo "\\033[1;37;42m--- Building $(PROJECT) ---\\033[0m"


lib: 
	@echo "\\033[1;37;42m--- Building Libraries ---\\033[0m"
	$(MAKE) -C $(OPENCM3_DIR) TARGETS=$(TARGET)

lib-clean: 
	$(MAKE) -C $(OPENCM3_DIR) TARGETS=$(TARGET) clean


flash: all
	@echo "\\033[1;37;42m--- Flashing $(PROJECT).bin to device ---\\033[0m"
	st-flash write $(BUILD_DIR)/$(PROJECT).bin 0x8000000 
	

clean: lib-clean
	@echo "\\033[1;37;42m--- Cleaning $(PROJECT) build ---\\033[0m"
	$(Q)$(RM) -rf $(BUILD_DIR)/$(PROJECT).*
	$(Q)$(RM) -rf $(BUILD_DIR)/$(OBJS) $(OBJS:.o=.d)
	$(Q)$(RM) -rf $(SRC_DIR)/generated.*.ld


# include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
