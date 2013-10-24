# board_flymaple.mk
#
# Build ArduPlane for Flymaple http://www.open-drone.org/flymaple

# cope with relative paths
ifeq ($(wildcard $(LIBMAPLE_PATH)/wirish),)
LIBMAPLE_PATH := $(shell cd $(SKETCHBOOK)/../libmaple && pwd)
endif

ifeq ($(wildcard $(LIBMAPLE_PATH)/wirish),)
$(error ERROR: failed to find libmaple - please see libraries/AP_HAL_FLYMAPLE/FlymaplePortingNotes.txt)
endif


TOOLCHAIN = ARM

#include $(MK_DIR)/find_arduino.mk
include $(MK_DIR)/find_tools.mk

HARDWARE := leaflabs
MCU := STM32F103RE
FAMILY := cortex-m3
F_CPU := 72000000L
LINKER := maple_RET6/flash.ld
HARDWARE_CORE := maple
UPLOADER := dfu-util
USBID := 1EAF:0003
PRODUCT_ID := 0003
LD_MEM_DIR := sram_64k_flash_512k

#
# Tool options
#
DEFINES         =   -DF_CPU=$(F_CPU) -DMCU_$(MCU) -DBOARD_$(BOARD) -DERROR_LED_PORT=GPIOA -DERROR_LED_PIN=5 -DVECT_TAB_FLASH 
DEFINES        +=   -DSKETCH=\"$(SKETCH)\"
DEFINES        +=   $(EXTRAFLAGS) # from user config.mk
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD)
WARNFLAGS       =   -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wno-psabi
WARNFLAGS      +=   -Wwrite-strings -Wformat=2 
WARNFLAGSCXX    =   -Wno-reorder
DEPFLAGS        =   -MD -MT $@

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char 
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char

ASOPTS          =   -x assembler-with-cpp 
LISTOPTS        =   -adhlns=$(@:.o=.lst)

NATIVE_CPUFLAGS     = -D_GNU_SOURCE
NATIVE_CPULDFLAGS   = -g
NATIVE_OPTFLAGS     = -O0 -g

ARM_CPUFLAGS        = -mcpu=$(FAMILY) -march=armv7-m -mthumb -DBOARD_maple -DMCU_$(MCU)
ARM_CPULDFLAGS      = -T$(LIBMAPLE_PATH)/support/ld/flash.ld -L $(LIBMAPLE_PATH)/support/ld/stm32/series/stm32f1/performance -L $(LIBMAPLE_PATH)/support/ld/stm32/mem/$(LD_MEM_DIR) -L $(LIBMAPLE_PATH)/support/ld -L $(LIBMAPLE_PATH)/support/ld/toolchains/generic 
ARM_OPTFLAGS        = -Os

CPUFLAGS= $($(TOOLCHAIN)_CPUFLAGS)
CPULDFLAGS= $($(TOOLCHAIN)_CPULDFLAGS)
OPTFLAGS= $($(TOOLCHAIN)_OPTFLAGS)

CXXFLAGS        =   $(CPUFLAGS) $(DEFINES) -Os -g3 -gdwarf-2 -nostdlib \
                   -ffunction-sections -fdata-sections \
                   -fno-rtti -fno-exceptions -Wl,--gc-sections $(OPTFLAGS)
CXXFLAGS       +=   $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS          =   $(CPUFLAGS) $(DEFINES) -Os -g3 -gdwarf-2 -nostdlib \
                   -ffunction-sections -fdata-sections \
                   -fno-exceptions -Wl,--gc-sections $(OPTFLAGS)
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   $(CPUFLAGS) $(DEFINES) -x assembler-with-cpp 
ASFLAGS        +=   $(ASOPTS)

LDFLAGS         =   $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS) -mcpu=cortex-m3 -mthumb \
           -Xlinker --gc-sections \
           -Xassembler --march=armv7-m -Wall 
LDFLAGS        +=   -Wl,--gc-sections -Wl,-Map -Wl,$(SKETCHMAP) $(CPULDFLAGS)

# under certain situations with certain avr-gcc versions the --relax flag causes
# a bug. Give the user a way to disable this flag per-sketch.
# I know this is a rotten hack but we're really close to sunset on AVR.
EXCLUDE_RELAX := $(wildcard $(SRCROOT)/norelax.inoflag)
ifeq ($(EXCLUDE_RELAX),)
#  LDFLAGS      +=   -Wl,--relax
endif

LIBS = -lm

ifeq ($(VERBOSE),)
v = @
else
v =
endif

COREOBJS = $(LIBMAPLE_PATH)/build/libmaple/*.o $(LIBMAPLE_PATH)/build/libmaple/usb/stm32f1/*.o $(LIBMAPLE_PATH)/build/libmaple/stm32f1/*.o $(LIBMAPLE_PATH)/build/libmaple/usb/usb_lib/*.o $(LIBMAPLE_PATH)/build/libmaple/usb/usb_lib/*.o $(LIBMAPLE_PATH)/build/libmaple/stm32f1/performance/*.o $(LIBMAPLE_PATH)/build/wirish/*.o $(LIBMAPLE_PATH)/build/wirish/boards/maple/*.o $(LIBMAPLE_PATH)/build/wirish/stm32f1/*.o $(LIBMAPLE_PATH)/build/libraries/Wire/*.o
COREINCLUDES = -I$(LIBMAPLE_PATH)/libmaple/include/libmaple -I$(LIBMAPLE_PATH)/wirish/include/wirish -I$(LIBMAPLE_PATH)/libraries  -I$(LIBMAPLE_PATH)/libmaple/include/libmaple -I$(LIBMAPLE_PATH)/wirish/include/wirish -I$(LIBMAPLE_PATH)/libraries   -I$(LIBMAPLE_PATH)/libmaple/include -I$(LIBMAPLE_PATH)/libmaple/stm32f1/include -I$(LIBMAPLE_PATH)/wirish/include -I$(LIBMAPLE_PATH)/wirish/boards/maple/include  -I$(LIBMAPLE_PATH)/libraries/Wire

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS) $(COREOBJS)

################################################################################
# Built products
#

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf

# HEX file
SKETCHHEX		=	$(BUILDROOT)/$(SKETCH).hex

# BIN file
SKETCHBIN		=	$(BUILDROOT)/$(SKETCH).bin

# EEP file
SKETCHEEP		=	$(BUILDROOT)/$(SKETCH).eep

# Map file
SKETCHMAP		=	$(BUILDROOT)/$(SKETCH).map

# All of the objects that may be built
ALLOBJS			=	$(SKETCHOBJS) $(LIBOBJS)

# All of the dependency files that may be generated
ALLDEPS			=	$(ALLOBJS:%.o=%.d)

################################################################################
# Targets
#

all: $(SKETCHELF) $(SKETCHEEP) $(SKETCHHEX)

print-%:
	echo "$*=$($*)"

flymaple-upload: upload

.PHONY: upload
upload: $(SKETCHBIN)
	$(LIBMAPLE_PATH)/support/scripts/reset.py && sleep 1 &&  $(UPLOADER) -a1 -d $(USBID) -D $(SKETCHBIN) -R

debug:
	$(AVARICE) --mkII --capture --jtag usb :4242 & \
	gnome-terminal -x $(GDB) $(SKETCHELF) & \
	echo -e '\n\nat the gdb prompt type "target remote localhost:4242"'

# this allows you to flash your image via JTAG for when you
# have completely broken your USB
jtag-program:
	$(AVARICE) --mkII --jtag usb --erase --program --file $(SKETCHELF)

################################################################################
# Rules
#

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

# Link the final object
$(SKETCHELF):	$(SKETCHOBJS) $(LIBOBJS)
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

# Create the hex file
$(SKETCHHEX):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O ihex -R .eeprom $< $@

# Create the bin file
$(SKETCHBIN):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -v -O binary $< $@

# Create the eep file
$(SKETCHEEP):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O ihex -j.eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

#
# Build sketch objects
#
SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) $(COREINCLUDES)

$(BUILDROOT)/%.o: $(BUILDROOT)/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< -I$(SRCROOT) $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

#
# Build library objects from sources in the sketchbook
#
SLIB_INCLUDES	=	-I$(dir $<)/utility $(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) $(COREINCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

