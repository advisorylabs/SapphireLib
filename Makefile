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
# Only src/sapphirelib/** belongs in the library archive, and so in the cold
# package and the published template. Everything else directly under src/
# (main.cpp, src/robot/, anything added later) is this robot's program and
# links straight into the hot image. It's an allowlist rather than a list of
# robot files, so a new robot file can't silently ship inside sapphirelib.a.
# (Headers need nothing: TEMPLATE_FILES below only takes include/sapphirelib/,
# so include/robot/ never ships either.)
EXCLUDE_SRC_FROM_LIB+=$(filter-out $(SRCDIR)/$(LIBNAME),$(wildcard $(SRCDIR)/*))

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
# the objects whose sources changed, so a build from a shell with a
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
$(error bin/ was built with $(ARCHTUPLE)g++ $(STAMPED_TOOLCHAIN) but $(TOOLCHAIN_VERSION) is first on PATH. Mixing them links fine and then crashes on the brain. Build from a shell using $(STAMPED_TOOLCHAIN), VS Code's PROS terminal, or run `make clean` to rebuild everything with $(TOOLCHAIN_VERSION).)
endif
endif
$(shell mkdir -p $(BINDIR) && echo $(TOOLCHAIN_VERSION) > $(TOOLCHAIN_STAMP))
endif
endif

################################################################################
############################### Example checking ###############################
# Nothing else compiles examples/; they aren't part of the program or the
# library, so an API change that breaks one would otherwise only surface once
# a team has copied it into their own project. `make check-examples`
# syntax-checks each example with the same compiler and flags as src/main.cpp
# (as if it had been copied there), writes no objects, and fails if any of them
# doesn't compile. Every example is checked even after one fails, so a single
# run lists them all.

.PHONY: check-examples
check-examples:
	@status=0; \
	for example in $(sort $(wildcard $(ROOT)/examples/*.cpp)); do \
		echo "Checking $$example"; \
		$(CXX) -fsyntax-only $(INCLUDE) $(CXXFLAGS) $(EXTRA_CXXFLAGS) "$$example" || status=1; \
	done; \
	exit $$status
