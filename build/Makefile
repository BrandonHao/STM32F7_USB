# List of available make goals:
#
# all                     Default target, builds the project
# clean                   Clean up the project
# rebuild                 Rebuild the project
# debug_flash             Builds the project and debug in flash
# debug_sram              Builds the project and debug in sram
#
# doc                     Build the documentation
# cleandoc                Clean up the documentation
# rebuilddoc              Rebuild the documentation
#
# \file
#
# Copyright (c) 2011 - 2018 Microchip Technology Inc. and its subsidiaries.
#
# \asf_license_start
#
# \page License
#
# Subject to your compliance with these terms, you may use Microchip
# software and any derivatives exclusively with Microchip products.
# It is your responsibility to comply with third party license terms applicable
# to your use of third party software (including open source software) that
# may accompany Microchip software.
#
# THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
# WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
# INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
# AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
# LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
# LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
# SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
# POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
# ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
# RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
# THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
#
# \asf_license_stop
#

# Include the config.mk file from the current working path, e.g., where the
# user called make.
include config.mk

# Tool to use to generate documentation from the source code
DOCGEN          ?= doxygen

# Look for source files relative to the top-level source directory
VPATH           := $(PROJ_PATH) $(LIB_PATH)

# Output target file
target          := $(TARGET_FLASH)
linker_script   := $(PROJ_PATH)/$(LINKER_SCRIPT_FLASH)

# Output project name (target name minus suffix)
project         := $(basename $(target))

# Allow override of operating system detection. The user can add OS=Linux or
# OS=Windows on the command line to explicit set the host OS.
#
# This allows to work around broken uname utility on certain systems.
ifdef OS
  ifeq ($(strip $(OS)), Linux)
    os_type     := Linux
  endif
  ifeq ($(strip $(OS)), Windows)
    os_type     := windows32_64
  endif
endif

os_type         ?= $(strip $(shell uname))

ifeq ($(os_type),windows32)
os              := Windows
else
ifeq ($(os_type),windows64)
os              := Windows
else
ifeq ($(os_type),windows32_64)
os              ?= Windows
else
ifeq ($(os_type),)
os              := Windows
else
# Default to Linux style operating system. Both Cygwin and mingw are fully
# compatible (for this Makefile) with Linux.
os              := Linux
endif
endif
endif
endif

# Output documentation directory and configuration file.
docdir          := ../doxygen/html
doccfg          := ../doxygen/doxyfile.doxygen

CROSS           ?= arm-none-eabi-
AR              := $(CROSS)ar
AS              := $(CROSS)as
CC              := $(CROSS)gcc
CPP             := $(CROSS)gcc -E
CXX             := $(CROSS)g++
LD              := $(CROSS)g++
NM              := $(CROSS)nm
OBJCOPY         := $(CROSS)objcopy
OBJDUMP         := $(CROSS)objdump
SIZE            := $(CROSS)size
GDB             := $(CROSS)gdb

RM              := rm
ifeq ($(os),Windows)
RMDIR           := rmdir /S /Q
else
RMDIR           := rmdir -p --ignore-fail-on-non-empty
endif

# On Windows, we need to override the shell to force the use of cmd.exe
ifeq ($(os),Windows)
SHELL           := cmd
endif

# Strings for beautifying output
MSG_CLEAN_FILES         = "RM      *.o *.d"
MSG_CLEAN_DIRS          = "RMDIR   $(strip $(clean-dirs))"
MSG_CLEAN_DOC           = "RMDIR   $(docdir)"
MSG_MKDIR               = "MKDIR   $(dir $@)"

MSG_INFO                = "INFO    "
MSG_PREBUILD            = "PREBUILD  $(PREBUILD_CMD)"
MSG_POSTBUILD           = "POSTBUILD $(POSTBUILD_CMD)"

MSG_ARCHIVING           = "AR      $@"
MSG_ASSEMBLING          = "AS      $@"
MSG_BINARY_IMAGE        = "OBJCOPY $@"
MSG_COMPILING           = "CC      $@"
MSG_COMPILING_CXX       = "CXX     $@"
MSG_EXTENDED_LISTING    = "OBJDUMP $@"
MSG_IHEX_IMAGE          = "OBJCOPY $@"
MSG_LINKING             = "LN      $@"
MSG_PREPROCESSING       = "CPP     $@"
MSG_SIZE                = "SIZE    $@"
MSG_SYMBOL_TABLE        = "NM      $@"

MSG_GENERATING_DOC      = "DOXYGEN $(docdir)"

# Don't use make's built-in rules and variables
MAKEFLAGS       += -rR

# Don't print 'Entering directory ...'
MAKEFLAGS       += --no-print-directory

# Function for reversing the order of a list
reverse = $(if $(1),$(call reverse,$(wordlist 2,$(words $(1)),$(1)))) $(firstword $(1))

# Hide command output by default, but allow the user to override this
# by adding V=1 on the command line.
#
# This is inspired by the Kbuild system used by the Linux kernel.
ifdef V
  ifeq ("$(origin V)", "command line")
    VERBOSE = $(V)
  endif
endif
ifndef VERBOSE
  VERBOSE = 0
endif

ifeq ($(VERBOSE), 1)
  Q =
else
  Q = @
endif

arflags-gnu-y           := $(ARFLAGS)
asflags-gnu-y           := $(ASFLAGS)
cflags-gnu-y            := $(CFLAGS)
cxxflags-gnu-y          := $(CXXFLAGS)
cppflags-gnu-y          := $(CPPFLAGS)
cpuflags-gnu-y          :=
dbgflags-gnu-y          := $(DBGFLAGS)
libflags-gnu-y          := $(foreach LIB,$(LIBS),-l$(LIB))
ldflags-gnu-y           := $(LDFLAGS)
flashflags-gnu-y        :=
clean-files             :=
clean-dirs              :=

clean-files             += $(wildcard $(target) $(project).map)
clean-files             += $(wildcard $(project).hex $(project).bin)
clean-files             += $(wildcard $(project).lss $(project).sym)
clean-files             += $(wildcard $(build))

# Use pipes instead of temporary files for communication between processes
cflags-gnu-y    += -pipe
asflags-gnu-y   += -pipe
ldflags-gnu-y   += -pipe

# Archiver flags.
arflags-gnu-y   += rcs

# Always enable warnings. And be very careful about implicit
# declarations.
cflags-gnu-y    += -Wall -Wmissing-prototypes
cflags-gnu-y    += -Werror-implicit-function-declaration
cxxflags-gnu-y  += -Wall -Wextra
# IAR doesn't allow arithmetic on void pointers, so warn about that.
cflags-gnu-y    += -Wpointer-arith
cxxflags-gnu-y  += -Wpointer-arith

# Preprocessor flags.
cppflags-gnu-y  += $(foreach INC,$(addprefix $(PRJ_PATH)/,$(INC_PATH)),-I$(INC))
asflags-gnu-y   += $(foreach INC,$(addprefix $(PRJ_PATH)/,$(INC_PATH)),'-Wa,-I$(INC)')

# HAL and other
cppflags-gnu-y  += $(foreach inc-dir,$(INC_DIR),-I$(inc-dir))
asflags-gnu-y   += $(foreach inc-dir,$(INC_DIR),'-Wa,-I$(inc-dir)')

# Tinyusb
cppflags-gnu-y  += $(foreach inc-dir,$(TINYUSB_INC),-I$(inc-dir))
asflags-gnu-y   += $(foreach inc-dir,$(TINYUSB_INC),'-Wa,-I$(inc-dir)')

# Main app
cppflags-gnu-y  += $(foreach inc-dir,$(APP_INC),-I$(inc-dir))
asflags-gnu-y   += $(foreach inc-dir,$(APP_INC),'-Wa,-I$(inc-dir)')

# CPU specific flags.
cpuflags-gnu-y  += -mcpu=$(ARCH) -mthumb -D=$(PART)

# Dependency file flags.
depflags        = -MD -MP -MQ $@

# Debug specific flags.
ifdef BUILD_DEBUG_LEVEL
dbgflags-gnu-y  += -g$(BUILD_DEBUG_LEVEL)
else
dbgflags-gnu-y  += -g3
endif

# Optimization specific flags.
ifdef BUILD_OPTIMIZATION
optflags-gnu-y  = -O$(BUILD_OPTIMIZATION)
else
optflags-gnu-y  = $(OPTIMIZATION)
endif

# Always preprocess assembler files.
asflags-gnu-y   += -x assembler-with-cpp
# Compile C files using the GNU99 standard.
cflags-gnu-y    += -std=gnu11
# Compile C++ files using the GNU++98 standard.
cxxflags-gnu-y  += -std=gnu++11

# Don't use strict aliasing (very common in embedded applications).
cflags-gnu-y    += -fno-strict-aliasing
cxxflags-gnu-y  += -fno-strict-aliasing

# Separate each function and data into its own separate section to allow
# garbage collection of unused sections.
cflags-gnu-y    += -ffunction-sections -fdata-sections
cxxflags-gnu-y  += -ffunction-sections -fdata-sections

# Various cflags.
cflags-gnu-y += -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
cflags-gnu-y += -Wmain -Wparentheses
cflags-gnu-y += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
cflags-gnu-y += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
cflags-gnu-y += -Wshadow -Wbad-function-cast -Wwrite-strings
cflags-gnu-y += -Wsign-compare -Waggregate-return
cflags-gnu-y += -Wmissing-declarations
cflags-gnu-y += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
cflags-gnu-y += -Wpacked -Wredundant-decls -Wnested-externs -Winline -Wlong-long
cflags-gnu-y += -Wunreachable-code
cflags-gnu-y += -Wcast-align -Wno-missing-braces
cflags-gnu-y += --param max-inline-insns-single=500

# Garbage collect unreferred sections when linking.
ldflags-gnu-y   += -Wl,--gc-sections
ldflags-gnu-y   += -Wl,-static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group

# Use the linker script if provided by the project.
ifneq ($(strip $(linker_script)),)
ldflags-gnu-y   += -Wl,-T $(linker_script)
endif

# Output a link map file and a cross reference table
ldflags-gnu-y   += -Wl,-Map=$(project).map,--cref -Wl,--print-memory-usage

# Add library search paths relative to the top level directory.
ldflags-gnu-y   += $(foreach _LIB_PATH,$(addprefix $(PRJ_PATH)/,$(LIB_PATH)),-L$(_LIB_PATH))

a_flags  = $(cpuflags-gnu-y) $(cppflags-gnu-y) $(asflags-gnu-y) -D__ASSEMBLY__
c_flags  = $(cpuflags-gnu-y) $(dbgflags-gnu-y) $(optflags-gnu-y) $(cppflags-gnu-y) $(cflags-gnu-y)
cxx_flags= $(cpuflags-gnu-y) $(dbgflags-gnu-y) $(optflags-gnu-y) $(cppflags-gnu-y) $(cxxflags-gnu-y)
l_flags  = -Wl,--entry=Reset_Handler -Wl,--cref $(cpuflags-gnu-y) $(optflags-gnu-y) $(ldflags-gnu-y)
ar_flags = $(arflags-gnu-y)

# Source files list and part informations must already be included before
# running this makefile

# If a custom build directory is specified, use it -- force trailing / in directory name.
ifdef BUILD_DIR
	build-dir       := $(dir $(BUILD_DIR))$(if $(notdir $(BUILD_DIR)),$(notdir $(BUILD_DIR))/)
else
	build-dir        =
endif

# Create object files list from source files list.
# Objects generated for STM32, HAL, and CMSIS files
obj-y-base              := $(addprefix $(build-dir), $(addsuffix .o,$(basename $(CORE_SRC) $(TINYUSB_SRC) $(TUSB_PORT_SRC) $(STM_SRC))))
# Objects for this specific app
obj-y-app               := $(addprefix $(build-dir), $(addsuffix .o,$(basename $(APP_SRC))))
# Full list of all objects
obj-y					:= $(obj-y-base) $(obj-y-app)

# Create dependency files list from source files list.
dep-files               := $(wildcard $(foreach f,$(obj-y),$(basename $(f)).d))

clean-files             += $(wildcard $(obj-y))
clean-files             += $(dep-files)

clean-dirs              += $(call reverse,$(sort $(wildcard $(dir $(obj-y)))))

# Default target.
.PHONY: all
all: prebuild $(target) $(project).lss $(project).sym $(project).hex $(project).bin postbuild

prebuild:
ifneq ($(strip $(PREBUILD_CMD)),)
	@echo $(MSG_PREBUILD)
	$(Q)$(PREBUILD_CMD)
endif

postbuild:
ifneq ($(strip $(POSTBUILD_CMD)),)
	@echo $(MSG_POSTBUILD)
	$(Q)$(POSTBUILD_CMD)
endif

# Clean up the project.
.PHONY: clean
clean:
	@echo $(clean-dirs)
	@$(if $(strip $(clean-files)),echo $(MSG_CLEAN_FILES))
	$(if $(strip $(clean-files)),$(Q)$(RM) $(clean-files),)
	@$(if $(strip $(clean-dirs)),echo $(MSG_CLEAN_DIRS))
# Remove created directories, and make sure we only remove existing
# directories, since recursive rmdir might help us a bit on the way.
ifeq ($(os),Windows)
	$(Q)$(if $(strip $(clean-dirs)),                        \
			$(RMDIR) $(strip $(subst /,\,$(clean-dirs))))
else
	$(Q)$(if $(strip $(clean-dirs)),                        \
		for directory in $(strip $(clean-dirs)); do     \
			if [ -d "$$directory" ]; then           \
				$(RMDIR) $$directory;           \
			fi                                      \
		done                                            \
	)
endif

# Rebuild the project.
.PHONY: rebuild
rebuild: clean all

.PHONY: objfiles
objfiles: $(obj-y-app) $(obj-y-base)

# Suppress warnings for STM32, HAL, and CMSIS files
$(obj-y-base): EXTRA_FLAGS := -w

# Create object files from C source files.
$(build-dir)%.o: %.c $(MAKEFILE_PATH) config.mk
	$(Q)test -d $(dir $@) || echo $(MSG_MKDIR)
ifeq ($(os),Windows)
	$(Q)test -d $(patsubst %/,%,$(dir $@)) || mkdir $(subst /,\,$(dir $@))
else
	$(Q)test -d $(dir $@) || mkdir -p $(dir $@)
endif
	@echo $(MSG_COMPILING)
	$(Q)$(CC) $(c_flags) $(EXTRA_FLAGS) -c $< -o $@

# Create object files from C++ source files.
$(build-dir)%.o: %.cpp $(MAKEFILE_PATH) config.mk
	$(Q)test -d $(dir $@) || echo $(MSG_MKDIR)
ifeq ($(os),Windows)
	$(Q)test -d $(patsubst %/,%,$(dir $@)) || mkdir $(subst /,\,$(dir $@))
else
	$(Q)test -d $(dir $@) || mkdir -p $(dir $@)
endif
	@echo $(MSG_COMPILING_CXX)
	$(Q)$(CXX) $(cxx_flags) -c $< -o $@

# Preprocess and assemble: create object files from assembler source files.
$(build-dir)%.o: %.s $(MAKEFILE_PATH) config.mk
	$(Q)test -d $(dir $@) || echo $(MSG_MKDIR)
ifeq ($(os),Windows)
	$(Q)test -d $(patsubst %/,%,$(dir $@)) || mkdir $(subst /,\,$(dir $@))
else
	$(Q)test -d $(dir $@) || mkdir -p $(dir $@)
endif
	@echo $(MSG_ASSEMBLING)
	$(Q)$(CC) $(a_flags) -c $< -o $@

# Include all dependency files to add depedency to all header files in use.
include $(dep-files)

# Link the object files into an ELF file. Also make sure the target is rebuilt
# if the common Makefile.sam.in or project config.mk is changed.
$(target): $(linker_script) $(MAKEFILE_PATH) config.mk $(obj-y)
	@echo $(MSG_LINKING)
	$(Q)$(LD) $(l_flags) $(obj-y) $(libflags-gnu-y) -o $@
	@echo $(MSG_SIZE)
	$(Q)$(SIZE) -Ax $@
	# $(Q)$(SIZE) -Bx $@

# Create extended function listing from target output file.
%.lss: $(target)
	@echo $(MSG_EXTENDED_LISTING)
	$(Q)$(OBJDUMP) -h -S $< > $@

# Create symbol table from target output file.
%.sym: $(target)
	@echo $(MSG_SYMBOL_TABLE)
	$(Q)$(NM) -n $< > $@

# Create Intel HEX image from ELF output file.
%.hex: $(target)
	@echo $(MSG_IHEX_IMAGE)
	$(Q)$(OBJCOPY) -O ihex $(flashflags-gnu-y)  $< $@

# Create binary image from ELF output file.
%.bin: $(target)
	@echo $(MSG_BINARY_IMAGE)
	$(Q)$(OBJCOPY) -O binary $< $@

# Provide information about the detected host operating system.
.SECONDARY: info-os
info-os:
	@echo $(MSG_INFO)$(os) build host detected

# Build Doxygen generated documentation.
.PHONY: doc
doc:
	@echo $(MSG_GENERATING_DOC)
	$(Q)cd $(dir $(doccfg)) && $(DOCGEN) $(notdir $(doccfg))

# Clean Doxygen generated documentation.
.PHONY: cleandoc
cleandoc:
	@$(if $(wildcard $(docdir)),echo $(MSG_CLEAN_DOC))
	$(Q)$(if $(wildcard $(docdir)),$(RM) --recursive $(docdir))

# Rebuild the Doxygen generated documentation.
.PHONY: rebuilddoc
rebuilddoc: cleandoc doc