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

# Examples are what people copy, so a broken one is a broken first impression. Compiled
# only: they have no main, and reference the reader's own firmware through externs.
examples:
	@for f in ../examples/*/ardudeck_link.c; do \
	  [ -f "$$f" ] || continue; \
	  $(CC) -std=c99 -Wall -Wextra -Werror $(INCLUDE) -fsyntax-only $$f 2>&1 \
	    && echo "  ok   $$f" || { echo "  FAIL $$f"; exit 1; }; \
	done
	@$(MAKE) --no-print-directory transports
	@for v in rover quad; do \
	  $(MAKE) --no-print-directory -C ../examples/$$v clean >/dev/null 2>&1 || true; \
	  $(MAKE) --no-print-directory -C ../examples/$$v >/dev/null 2>&1 \
	    && echo "  ok   examples/$$v links and builds" \
	    || { echo "  FAIL examples/$$v"; exit 1; }; \
	  $(MAKE) --no-print-directory -C ../examples/$$v clean >/dev/null 2>&1 || true; \
	done
	@echo "every example compiles"

# The transport examples target real boards, so each is checked against just enough of
# that platform's headers to be worth trusting. See platform-shims/README.md.
transports:
	@for pair in esp32:esp32_udp.c stm32:stm32_uart.c zephyr:zephyr_uart.c \
	             linux:linux_udp.c; do \
	  p=$${pair%%:*}; f=$${pair##*:}; \
	  inc=""; [ "$$p" = linux ] || inc="-Iplatform-shims/$$p"; \
	  $(CC) -std=c99 -Wall -Wextra -Werror $(INCLUDE) $$inc -fsyntax-only \
	    ../examples/transports/$$f 2>&1 \
	    && echo "  ok   transports/$$f ($$p)" \
	    || { echo "  FAIL transports/$$f ($$p)"; exit 1; }; \
	done
	@c++ -std=c++11 -Wall -Wextra -Werror $(INCLUDE) -Iplatform-shims/arduino \
	  -include Arduino.h -fsyntax-only -x c++ ../examples/transports/arduino_serial.ino \
	  && echo "  ok   transports/arduino_serial.ino (arduino)" \
	  || { echo "  FAIL transports/arduino_serial.ino"; exit 1; }
