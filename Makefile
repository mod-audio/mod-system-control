#!/usr/bin/make -f
# Makefile for mod-system-control #
# ------------------------------- #
# Created by falkTX
#

# ---------------------------------------------------------------------------------------------------------------------
# Base environment vars

CC  ?= $(CROSS_COMPILE)gcc
CXX ?= $(CROSS_COMPILE)g++

PREFIX ?= /usr/local
BINDIR  = $(PREFIX)/bin

# ---------------------------------------------------------------------------------------------------------------------
# Set build and link flags

BASE_FLAGS  = -Wall -Wextra -pipe -fPIC -DPIC -pthread -MD -MP -fno-common
BASE_FLAGS += -Werror=implicit-function-declaration
BASE_FLAGS += -Werror=shadow
BASE_OPTS   = -O2 -fdata-sections -ffunction-sections
LINK_OPTS   = -fdata-sections -ffunction-sections -Wl,--gc-sections -Wl,-O1 -Wl,--as-needed

ifneq ($(SKIP_STRIPPING),true)
LINK_OPTS  += -Wl,--strip-all
endif

ifeq ($(DEBUG),true)
BASE_FLAGS  += -DDEBUG -O0 -g
LINK_OPTS    =
else
BASE_FLAGS  += -DNDEBUG $(BASE_OPTS) -fvisibility=hidden
CXXFLAGS    += -fvisibility-inlines-hidden
endif

BUILD_C_FLAGS   = $(BASE_FLAGS) -std=gnu99 $(CFLAGS)
BUILD_CXX_FLAGS = $(BASE_FLAGS) -std=gnu++11 $(CXXFLAGS)
LINK_FLAGS      = $(LINK_OPTS) $(LDFLAGS) -Wl,--no-undefined

# for serial port
BASE_FLAGS     += $(shell pkg-config --cflags libserialport)
LINK_FLAGS     += $(shell pkg-config --libs libserialport)

# for systemd notify
BASE_FLAGS     += $(shell pkg-config --cflags libsystemd)
LINK_FLAGS     += $(shell pkg-config --libs libsystemd)

# ---------------------------------------------------------------------------------------------------------------------
# Strict test build

ifeq ($(TESTBUILD),true)
BASE_FLAGS += -Werror -Wabi=98 -Wcast-qual -Wclobbered -Wconversion -Wdisabled-optimization
BASE_FLAGS += -Wdouble-promotion -Wfloat-equal -Wlogical-op -Wpointer-arith -Wsign-conversion
BASE_FLAGS += -Wformat=2 -Woverlength-strings
BASE_FLAGS += -Wformat-truncation=2 -Wformat-overflow=2
BASE_FLAGS += -Wstringop-overflow=4 -Wstringop-truncation
BASE_FLAGS += -Wmissing-declarations -Wredundant-decls
BASE_FLAGS += -Wshadow  -Wundef -Wuninitialized -Wunused
BASE_FLAGS += -Wstrict-aliasing -fstrict-aliasing
BASE_FLAGS += -Wstrict-overflow -fstrict-overflow
BASE_FLAGS += -Wduplicated-branches -Wduplicated-cond -Wnull-dereference
CFLAGS     += -Winit-self -Wjump-misses-init -Wmissing-prototypes -Wnested-externs -Wstrict-prototypes -Wwrite-strings
CXXFLAGS   += -Wc++0x-compat -Wc++11-compat
CXXFLAGS   += -Wnon-virtual-dtor -Woverloaded-virtual
CXXFLAGS   += -Wzero-as-null-pointer-constant
ifneq ($(DEBUG),true)
CXXFLAGS   += -Weffc++
endif
endif

# ---------------------------------------------------------------------------------------------------------------------
# Build rules

SOURCES = main.c reply.c serial_io.c serial_rw.c
OBJECTS = $(SOURCES:%.c=build/%.c.o)
TARGETS = mod-system-control

all: $(TARGETS)

mod-system-control: $(OBJECTS)
	$(CC) $^ $(BUILD_C_FLAGS) $(LINK_FLAGS) -lm -o $@

# tests/full: tests/full.c.o system-control.c
# 	$(CC) $< $(ALSA_CFLAGS) $(JACK_CFLAGS) $(BUILD_C_FLAGS) $(JACK_LIBS) $(ALSA_LIBS) $(LINK_FLAGS) -lm -o $@

build/%.c.o: src/%.c
	-$(shell mkdir -p build)
	$(CC) $< $(BUILD_C_FLAGS) -c -o $@

clean:
	rm -rf $(TARGETS) build/

install: all
	install -d $(DESTDIR)$(BINDIR)
	install -m 755 mod-system-control $(DESTDIR)$(BINDIR)

# ---------------------------------------------------------------------------------------------------------------------

test: tests/socat-scope.sh $(TARGETS)
	$< $(CURDIR)/mod-system-control

# ---------------------------------------------------------------------------------------------------------------------

-include $(OBJECTS:%.o=%.d)

# ---------------------------------------------------------------------------------------------------------------------
