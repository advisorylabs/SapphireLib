################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Pinned explicitly (rather than trusting the kernel template's default,
# currently gnu23/gnu++26) so the whole team builds against the same,
# well-supported standard for the life of the season.
C_STANDARD:=gnu17
CXX_STANDARD:=gnu++20

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1
LIBNAME:=sapphirelib
# Kept in sync with SAPPHIRELIB_VERSION in include/sapphirelib/version.hpp
VERSION:=0.1.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME, including its subdirectories
# (chassis/, control/, util/) since headers aren't all flat in one folder.
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp $(INCDIR)/$(LIBNAME)/*/*.h $(INCDIR)/$(LIBNAME)/*/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

################################################################################
######################## Toolchain consistency guard ###########################
# bin/ keeps no record of which compiler produced it, and make only rebuilds
# the objects whose sources changed — so a build from a shell with a
# different arm-none-eabi-g++ first on PATH silently mixes objects from two
# GCC major versions into one image. That links without complaint and then
# faults on the brain before LVGL paints anything, which reads as a dead
# program rather than as a build problem. Stamp the compiler version into
# bin/ and refuse to build on a mismatch instead of shipping that image.
#
# clean/all are exempt: clearing bin/ is the fix, and `all` cleans first. The
# check is skipped entirely if the version can't be read, so it can only ever
# act on information it actually has.

TOOLCHAIN_VERSION:=$(shell $(CXX) -dumpfullversion 2>/dev/null || $(CXX) -dumpversion 2>/dev/null)
TOOLCHAIN_STAMP:=$(BINDIR)/.toolchain-version

ifneq ($(TOOLCHAIN_VERSION),)
ifeq ($(filter clean clean-template all,$(MAKECMDGOALS)),)
STAMPED_TOOLCHAIN:=$(shell cat $(TOOLCHAIN_STAMP) 2>/dev/null)
ifneq ($(STAMPED_TOOLCHAIN),)
ifneq ($(STAMPED_TOOLCHAIN),$(TOOLCHAIN_VERSION))
$(error bin/ was built with $(ARCHTUPLE)g++ $(STAMPED_TOOLCHAIN) but $(TOOLCHAIN_VERSION) is first on PATH. Mixing them links fine and then crashes on the brain. Build from a shell using $(STAMPED_TOOLCHAIN) — VS Code's PROS terminal — or run `make clean` to rebuild everything with $(TOOLCHAIN_VERSION).)
endif
endif
$(shell mkdir -p $(BINDIR) && echo $(TOOLCHAIN_VERSION) > $(TOOLCHAIN_STAMP))
endif
endif
