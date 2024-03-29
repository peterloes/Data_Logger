####################################################################
# Makefile                                                         #
#                                                                  #
# Set WINDOWSCS or LINUXCS to match your environment, e.g.         #
# export LINUXCS=/opt/cross/gcc-arm-none-eabi-4_8-2014q3           #
# export PATH=${PATH}:${LINUXCS}/bin                               #
#                                                                  #
####################################################################

.SUFFIXES:				# ignore builtin rules
.PHONY: all debug release clean

####################################################################
# Definitions                                                      #
####################################################################

DEVICE = EFM32G230F128
PROJECTNAME = Data_Logger

OBJ_DIR = build
EXE_DIR = exe
LST_DIR = lst

####################################################################
# Definitions of toolchain.                                        #
# You might need to do changes to match your system setup          #
####################################################################

# Change path to the tools according to your system configuration
# DO NOT add trailing whitespace chars, they do matter !
WINDOWSCS  ?= GNU Tools ARM Embedded\4.7 2012q4
LINUXCS    ?= /cad/arm-embedded/gcc-arm-none-eabi-4_7-2012q4

RMDIRS     := rm -rf
RMFILES    := rm -f
ALLFILES   := /*.*
NULLDEVICE := /dev/null
SHELLNAMES := $(ComSpec)$(COMSPEC)

# Try autodetecting the environment
ifeq ($(SHELLNAMES),)
  # Assume we are making on a Linux platform
  TOOLDIR := $(LINUXCS)
else
  QUOTE :="
  ifneq ($(COMSPEC),)
    # Assume we are making on a mingw/msys/cygwin platform running on Windows
    # This is a convenient place to override TOOLDIR, DO NOT add trailing
    # whitespace chars, they do matter !
    TOOLDIR := $(PROGRAMFILES)/$(WINDOWSCS)
    ifeq ($(findstring cygdrive,$(shell set)),)
      # We were not on a cygwin platform
      NULLDEVICE := NUL
    endif
  else
    # Assume we are making on a Windows platform
    # This is a convenient place to override TOOLDIR, DO NOT add trailing
    # whitespace chars, they do matter !
    SHELL      := $(SHELLNAMES)
    TOOLDIR    := $(ProgramFiles)/$(WINDOWSCS)
    RMDIRS     := rd /s /q
    RMFILES    := del /s /q
    ALLFILES   := \*.*
    NULLDEVICE := NUL
  endif
endif

# Create directories and do a clean which is compatible with parallell make
$(shell mkdir $(OBJ_DIR)>$(NULLDEVICE) 2>&1)
$(shell mkdir $(EXE_DIR)>$(NULLDEVICE) 2>&1)
$(shell mkdir $(LST_DIR)>$(NULLDEVICE) 2>&1)
ifeq (clean,$(findstring clean, $(MAKECMDGOALS)))
  ifneq ($(filter $(MAKECMDGOALS),all debug release),)
    $(shell $(RMFILES) $(OBJ_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
    $(shell $(RMFILES) $(EXE_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
    $(shell $(RMFILES) $(LST_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
  endif
endif

CC      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-gcc$(QUOTE)
LD      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-ld$(QUOTE)
AR      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-ar$(QUOTE)
OBJCOPY = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-objcopy$(QUOTE)
DUMP    = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-objdump$(QUOTE)

####################################################################
# Flags                                                            #
####################################################################

# -MMD : Don't generate dependencies on system header files.
# -MP  : Add phony targets, useful when a h-file is removed from a project.
# -MF  : Specify a file to write the dependencies to.
DEPFLAGS = -MMD -MP -MF $(@:.o=.d)

#
# Add -Wa,-ahld=$(LST_DIR)/$(@F:.o=.lst) to CFLAGS to produce assembly list files
#
override CFLAGS += -D$(DEVICE) -Wall -Wextra -mcpu=cortex-m3 -mthumb \
-mfix-cortex-m3-ldrd -ffunction-sections \
-fdata-sections -fomit-frame-pointer -DSTKG8XX  \
-Wa,-ahld=$(LST_DIR)/$(@F:.o=.lst) \
$(DEPFLAGS)

override ASMFLAGS += -x assembler-with-cpp -D$(DEVICE) -Wall -Wextra -mcpu=cortex-m3 -mthumb

#
# NOTE: The -Wl,--gc-sections flag may interfere with debugging using gdb.
#
override LDFLAGS += -Xlinker -Map=$(LST_DIR)/$(PROJECTNAME).map -mcpu=cortex-m3 \
-mthumb -T../Device/EnergyMicro/EFM32G/Source/GCC/efm32g.ld \
 -Wl,--gc-sections

LIBS = -Wl,--start-group -lgcc -lc -lnosys   -Wl,--end-group

# All include pathes are local now
INCLUDEPATHS += \
-I.. \
-I../CMSIS/Include \
-I../Device/EnergyMicro/EFM32G/Include \
-I../emlib/inc \
-I../fatfs/inc \
-I../drivers

####################################################################
# Files                                                            #
####################################################################

# All source files are local now
C_SRC +=  \
../DMA_ControlBlock.c \
../Device/EnergyMicro/EFM32G/Source/system_efm32g.c \
../emlib/src/em_assert.c \
../emlib/src/em_cmu.c \
../emlib/src/em_dma.c \
../emlib/src/em_emu.c \
../emlib/src/em_gpio.c \
../emlib/src/em_leuart.c \
../emlib/src/em_usart.c \
../emlib/src/em_i2c.c \
../emlib/src/em_int.c \
../emlib/src/em_rtc.c \
../emlib/src/em_adc.c \
../emlib/src/em_system.c \
../fatfs/src/diskio.c \
../fatfs/src/ff.c \
../main.c \
../debug.c \
../drivers/LEUART.c \
../drivers/clock.c \
../drivers/Control.c \
../drivers/CfgData.c \
../drivers/AlarmClock.c \
../drivers/ExtInt.c \
../drivers/Keys.c \
../drivers/Logging.c \
../drivers/microsd.c \
../drivers/SensorMon.c \
../drivers/Update.c

s_SRC += 

S_SRC +=  \
../Device/EnergyMicro/EFM32G/Source/GCC/startup_efm32g.S

####################################################################
# Rules                                                            #
####################################################################

C_FILES = $(notdir $(C_SRC) )
S_FILES = $(notdir $(S_SRC) $(s_SRC) )
#make list of source paths, sort also removes duplicates
C_PATHS = $(sort $(dir $(C_SRC) ) )
S_PATHS = $(sort $(dir $(S_SRC) $(s_SRC) ) )

C_OBJS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.o))
S_OBJS = $(if $(S_SRC), $(addprefix $(OBJ_DIR)/, $(S_FILES:.S=.o)))
s_OBJS = $(if $(s_SRC), $(addprefix $(OBJ_DIR)/, $(S_FILES:.s=.o)))
C_DEPS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.d))
OBJS = $(C_OBJS) $(S_OBJS) $(s_OBJS) $(OBJ_DIR)/version.o

vpath %.c $(C_PATHS)
vpath %.s $(S_PATHS)
vpath %.S $(S_PATHS)

# Default build is debug build
all:      debug

debug:    CFLAGS += -DDEBUG -DDEBUG_EFM -O0 -g
debug:    $(EXE_DIR)/$(PROJECTNAME).bin

release:  CFLAGS += -DNDEBUG -Os
release:  $(EXE_DIR)/$(PROJECTNAME).bin

# rule when to update version file
$(OBJ_DIR)/version.o: $(C_OBJS) $(S_OBJS) $(s_OBJS) ../version.c

# Create objects from C SRC files
$(OBJ_DIR)/%.o: %.c
	@echo "Building file: $<"
	$(CC) $(CFLAGS) $(INCLUDEPATHS) -c -o $@ $<

# Assemble .s/.S files
$(OBJ_DIR)/%.o: %.s
	@echo "Assembling $<"
	$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

$(OBJ_DIR)/%.o: %.S
	@echo "Assembling $<"
	$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

# Link
$(EXE_DIR)/$(PROJECTNAME).out: $(OBJS)
	@echo "Linking target: $@"
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $(EXE_DIR)/$(PROJECTNAME).out

# Create binary file
$(EXE_DIR)/$(PROJECTNAME).bin: $(EXE_DIR)/$(PROJECTNAME).out
	@echo "Creating binary file"
	$(OBJCOPY) -O binary $(EXE_DIR)/$(PROJECTNAME).out $(EXE_DIR)/$(PROJECTNAME).bin
	$(OBJCOPY) -O ihex $(EXE_DIR)/$(PROJECTNAME).out $(EXE_DIR)/$(PROJECTNAME).hex
	# Produce assembly listing of entire program
	$(DUMP) -h -S -C $(EXE_DIR)/$(PROJECTNAME).out >$(LST_DIR)/$(PROJECTNAME)_out.lst

clean:
ifeq ($(filter $(MAKECMDGOALS),all debug release),)
	$(RMFILES) $(OBJ_DIR)$(ALLFILES) $(LST_DIR)$(ALLFILES)
endif

# include auto-generated dependency files (explicit rules)
ifneq (clean,$(findstring clean, $(MAKECMDGOALS)))
-include $(C_DEPS)
endif

doc:
	cd ..;			\
	doxygen
