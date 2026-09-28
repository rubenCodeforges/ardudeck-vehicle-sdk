CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -Werror -O1 -g
INCLUDE  = -I../include -I../src
SRC      = $(wildcard ../src/*.c) test_sdk.c

all: run

run_tests: $(SRC) $(wildcard ../include/*.h) $(wildcard ../src/*.h)
	$(CC) $(CFLAGS) $(INCLUDE) $(SRC) -o $@

run: run_tests
	./run_tests

# The build the SDK promises: no allocation, no threads, no sockets, no libm.
strict: $(SRC)
	$(CC) -std=c99 -Wall -Wextra -Werror -Wconversion -Wshadow -Wpedantic \
	      -Wstrict-prototypes -Wmissing-prototypes $(INCLUDE) -c $(wildcard ../src/*.c)
	@rm -f *.o
	@echo "strict build clean"

# Prove the feature switches actually compile out.
minimal: $(wildcard ../src/*.c)
	$(CC) $(CFLAGS) $(INCLUDE) -DAD_MAX_MISSION_ITEMS=0 -DAD_MAX_PARAMS=0 \
	      -DAD_MAX_CALIBRATIONS=0 -DAD_WITH_COMMANDS=0 -c $(wildcard ../src/*.c)
	@rm -f *.o
	@echo "telemetry-only build clean"

clean:
	rm -f run_tests *.o

.PHONY: all run strict minimal clean

# A vehicle on a real socket, for pointing the conformance tool at.
fake_vehicle: fake_vehicle.c $(wildcard ../src/*.c) $(wildcard ../src/*.h) $(wildcard ../include/*.h)
	$(CC) $(CFLAGS) $(INCLUDE) -D_DEFAULT_SOURCE -D_DARWIN_C_SOURCE \
	      fake_vehicle.c $(wildcard ../src/*.c) -o $@

conform: fake_vehicle
	$(MAKE) -C ../conformance
	./conform.sh

# Cross-compile for 32-bit bare targets. No sysroot: test/freestanding/ supplies the
# three libc functions the SDK uses, so this proves portability rather than assuming it.
#
# Xtensa (ESP32 classic) needs Espressif's clang fork and is not checked here; the
# RISC-V ESP32 variants and any ARM part are. A pass here is not a substitute for
# building in the vendor's own toolchain, it just catches the 32-bit assumptions early.
TARGETS = armv7m-none-eabi thumbv7em-none-eabi riscv32-unknown-elf armv6m-none-eabi
portable:
	@ran=0; for t in $(TARGETS); do \
	  if ! clang -target $$t -c -x c /dev/null -o /dev/null >/dev/null 2>&1; then \
	    echo "  skip $$t (this clang has no backend for it)"; continue; fi; \
	  clang -target $$t -std=c99 -ffreestanding -Wall -Wextra -Werror -Wconversion \
	        -Wshadow -Os -I../include -I../src -Ifreestanding \
	        -c $(wildcard ../src/*.c) 2>&1 \
	    && { echo "  ok   $$t"; ran=$$((ran+1)); } || { echo "  FAIL $$t"; rm -f *.o; exit 1; }; \
	done; \
	rm -f *.o; \
	[ $$ran -gt 0 ] || { echo "  no cross targets available"; exit 1; }; \
	echo "portable: builds clean on $$ran bare 32-bit target(s)"
