CC     ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2
# _DEFAULT_SOURCE for cfmakeraw and the BSD socket extras on glibc.
CFLAGS += -D_DEFAULT_SOURCE -D_DARWIN_C_SOURCE
SRC     = main.c checks.c mav.c link.c

ardudeck-conform: $(SRC) conform.h ../src/ad_msg_defs.h
	$(CC) $(CFLAGS) -I../src $(SRC) -o $@ -lm

clean:
	rm -f ardudeck-conform

.PHONY: clean
