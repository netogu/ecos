######################################################################
#  Make Rules
######################################################################

PREFIX	?= arm-none-eabi
SRC_DIR	  := src
RTOS_DIR	:= src/rtos
LIB_DIR  	:= lib
NEWLIB_C_DIR := /usr/include/newlib
NEWLIB_CPP_DIR := /usr/include/newlib/c++/10.3.1
BUILD_DIR	:= build
BINARY	:= $(BUILD_DIR)/out


LIBNAME	= opencm3_stm32f1
DEFS	+= -DSTM32F1

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd
ASFLAGS		= -mthumb -mcpu=cortex-m3

CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
OBJCOPY		:= $(PREFIX)-objcopy
SIZE		:= $(PREFIX)-size
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb
STFLASH		= $(shell which st-flash)
STYLECHECK	:= /checkpatch.pl
STYLECHECKFLAGS	:= --no-tree -f --terse --mailback
STYLECHECKFILES	:= $(shell find . -name '*.[ch]')
OPT		:= -Os -g
CSTD		?= -std=c99
CXXSTD		?= -std=c++17

SRCFILES	= $(wildcard $(SRC_DIR)/*.c) 
SRCFILES += $(wildcard $(SRC_DIR)/*.cpp) 
SRCFILES += $(wildcard $(RTOS_DIR)/*.c)
SRCFILES += $(wildcard $(LIB_DIR)/*.c)
TEMP1	= $(patsubst %.c,%.o,$(SRCFILES))
TEMP2	= $(patsubst %.asm,%.o,$(TEMP1))
OBJS	= $(patsubst %.cpp,%.o,$(TEMP2))

LDSCRIPT	?= $(SRC_DIR)/linker_script.ld

TGT_CFLAGS	+= $(OPT) $(CSTD)
TGT_CFLAGS	+= $(ARCH_FLAGS)
TGT_CFLAGS	+= -Wextra -Wshadow -Wimplicit-function-declaration
TGT_CFLAGS	+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
TGT_CFLAGS	+= -fno-common -ffunction-sections -fdata-sections
TGT_CFLAGS	+= -I.
TGT_CFLAGS	+= -I$(OPENCM3_DIR)/include
TGT_CFLAGS  += -I$(LIB_DIR)
TGT_CFLAGS	+= -I$(SRC_DIR) -I$(RTOS_DIR)
TGT_CFLAGS	+= -I$(NEWLIB_C_DIR)


TGT_CXXFLAGS	+= $(OPT) $(CXXSTD)
TGT_CXXFLAGS	+= $(ARCH_FLAGS)
TGT_CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
TGT_CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections
TGT_CXXFLAGS	+= -I.
TGT_CXXFLAGS	+= -I$(OPENCM3_DIR)/include
TGT_CXXFLAGS	+= -I$(SRC_DIR) -I$(RTOS_DIR)
TGT_CXXFLAGS  += -I$(LIB_DIR)


TGT_CPPFLAGS	+= -MD
TGT_CPPFLAGS	+= -Wall -Wundef
TGT_CPPFLAGS	+= $(DEFS)
TGT_CPPFLAGS	+= -I.
TGT_CPPFLAGS	+= -I$(OPENCM3_DIR)/include
TGT_CPPFLAGS	+= -I$(SRC_DIR) -I$(RTOS_DIR)
TGT_CPPFLAGS  += -I$(LIB_DIR)
TGT_CPPFLAGS  += -I$(NEWLIB_CPP_DIR)

TGT_LDFLAGS	+= --static -nostartfiles
TGT_LDFLAGS	+= -T$(LDSCRIPT)
TGT_LDFLAGS	+= $(ARCH_FLAGS)
TGT_LDFLAGS	+= -Wl,-Map=$(*).map
TGT_LDFLAGS	+= -Wl,--gc-sections

# LDLIBS		+= -specs=nosys.specs
# LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
# # LDLIBS		+= -L$(SRC_DIR)/rtos/libwwg -lwwg
# LDLIBS		+= -L$(OPENCM3_DIR)/lib -lopencm3_stm32f1

.SUFFIXES:	.elf .bin .hex .srec .list .map .images
.SECONDEXPANSION:
.SECONDARY:


.PHONY: setup images clean elf bin hex srec list all

all: setup elf
setup: 
	mkdir -p $(BUILD_DIR)
elf:	$(DEPS) $(BINARY).elf | setup
bin:	$(DEPS) $(BINARY).bin
hex:	$(DEPS) $(BINARY).hex
srec:	$(DEPS) $(BINARY).srec
list:	$(DEPS) $(BINARY).list


# Define a helper macro for debugging make errors online
# you can type "make print-OPENCM3_DIR" and it will show you
# how that ended up being resolved by all of the included
# makefiles.
print-%:
	@echo $*=$($*)


%.images: %.bin %.hex %.srec %.list %.map
	@#printf "*** $* images generated ***\n"

%.bin: %.elf
	@#printf "  OBJCOPY $(*).bin\n"
	$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@#printf "  OBJCOPY $(*).hex\n"
	$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@#printf "  OBJCOPY $(*).srec\n"
	$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@#printf "  OBJDUMP $(*).list\n"
	$(OBJDUMP) -S $(*).elf > $(*).list

%.elf %.map: $(OBJS) $(LDSCRIPT)
	$(LD) $(TGT_LDFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $(*).elf
	$(SIZE) $(BINARY).elf

%.o: %.c
	@#printf "  CC      $(*).c\n"
	$(CC) $(TGT_CFLAGS) $(CFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $(*).o -c $(*).c

%.o: %.cxx
	@#printf "  CXX     $(*).cxx\n"
	$(CXX) $(TGT_CXXFLAGS) $(CXXFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $(*).o -c $(*).cxx

%.o: %.cpp
	@#printf "  CXX     $(*).cpp\n"
	$(CXX) $(TGT_CXXFLAGS) $(CXXFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $(*).o -c $(*).cpp

%.o: %.asm
	$(AS) $(ASFLAGS) -o $*.o -c $<

clean:
	@#printf "  CLEAN\n"
	$(RM) *.o *.d generated.* $(OBJS) $(patsubst %.o,%.d,$(OBJS))


clobber: clean
	rm -fv *.elf *.bin *.hex *.srec *.list *.map $(CLOBBER)
	rm -rv $(BUILD_DIR)

# Flash 64k Device
flash:	$(BINARY).bin
	$(STFLASH) $(FLASHSIZE) write $(BINARY).bin 0x8000000 

# Flash 128k Device
bigflash: $(BINARY).bin
	$(STFLASH) --flash=128k write $(BINARY).bin 0x8000000


-include $(OBJS:.o=.d)

# End

