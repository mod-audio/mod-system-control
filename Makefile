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
BASE_FLAGS  += $(BASE_OPTS) -fvisibility=hidden
CXXFLAGS    += -fvisibility-inlines-hidden
endif

BUILD_C_FLAGS   = $(BASE_FLAGS) -std=gnu99 $(CFLAGS)
BUILD_CXX_FLAGS = $(BASE_FLAGS) -std=gnu++11 $(CXXFLAGS)
LINK_FLAGS      = $(LINK_OPTS) $(LDFLAGS) -Wl,--no-undefined

# for serial port
BASE_FLAGS     += $(shell pkg-config --cflags libserialport)
LINK_FLAGS_SP   = $(shell pkg-config --libs libserialport)

# for systemd notify
BASE_FLAGS     += $(shell pkg-config --cflags libsystemd)
LINK_FLAGS_SD   = $(shell pkg-config --libs libsystemd)

# ---------------------------------------------------------------------------------------------------------------------
# Strict test build

ifeq ($(TESTBUILD),true)
BASE_FLAGS += -Werror -Wabi=98 -Wclobbered -Wconversion -Wdisabled-optimization -Wno-sign-conversion
BASE_FLAGS += -Wdouble-promotion -Wfloat-equal -Wlogical-op -Wpointer-arith
BASE_FLAGS += -Wformat=2 -Woverlength-strings
BASE_FLAGS += -Wformat-truncation=2 -Wformat-overflow=2
BASE_FLAGS += -Wstringop-overflow=4 -Wstringop-truncation
BASE_FLAGS += -Wmissing-declarations -Wredundant-decls
BASE_FLAGS += -Wshadow  -Wundef -Wuninitialized -Wunused
BASE_FLAGS += -Wstrict-aliasing -fstrict-aliasing
BASE_FLAGS += -Wstrict-overflow -fstrict-overflow
BASE_FLAGS += -Wduplicated-branches -Wduplicated-cond -Wnull-dereference
CFLAGS     += -Winit-self -Wjump-misses-init -Wmissing-prototypes -Wnested-externs -Wstrict-prototypes -Wwrite-strings
endif

# ---------------------------------------------------------------------------------------------------------------------
# Build rules

SOURCES_main      = main.c cli.c reply.c serial_io.c serial_rw.c
SOURCES_test_fake = test.c cli.c fakeserial.c reply.c serial_rw.c
SOURCES_test_real = test.c cli.c serial_io.c reply.c serial_rw.c
OBJECTS_main      = $(SOURCES_main:%.c=build/%.c.o)
OBJECTS_test_fake = $(SOURCES_test_fake:%.c=build/%.c.o)
OBJECTS_test_real = $(SOURCES_test_real:%.c=build/%.c.o)

TARGETS = mod-system-control test-fake test-real

all: $(TARGETS)

mod-system-control: $(OBJECTS_main)
	$(CC) $^ $(BUILD_C_FLAGS) $(LINK_FLAGS) $(LINK_FLAGS_SP) $(LINK_FLAGS_SD) -lm -o $@

test-fake: $(OBJECTS_test_fake) /var/cache/mod/tag
	$(CC) $(filter %.o,$^) $(BUILD_C_FLAGS) $(LINK_FLAGS) -lm -o $@

test-real: $(OBJECTS_test_real) /var/cache/mod/tag
	$(CC) $(filter %.o,$^) $(BUILD_C_FLAGS) $(LINK_FLAGS) $(LINK_FLAGS_SP) -lm -o $@

test-fake-run: test-fake
	env PATH=$(CURDIR)/tests/bin:$(PATH) ./test-fake

/var/cache/mod/tag:
	mkdir -p /var/cache/mod
	echo "MDW01D01-00001" > $@

build/test.c.o: src/test.c
	-$(shell mkdir -p build)
	$(CC) $< $(BUILD_C_FLAGS) -c -o $@

build/%.c.o: src/%.c
	-$(shell mkdir -p build)
	$(CC) $< $(BUILD_C_FLAGS) -DNDEBUG -c -o $@

clean:
	rm -rf $(TARGETS) build/

install: all
	install -d $(DESTDIR)$(BINDIR)
	install -m 755 mod-system-control $(DESTDIR)$(BINDIR)

# ---------------------------------------------------------------------------------------------------------------------

-include $(OBJECTS_main:%.o=%.d)
-include $(OBJECTS_test_fake:%.o=%.d)
-include $(OBJECTS_test_real:%.o=%.d)

# ---------------------------------------------------------------------------------------------------------------------
