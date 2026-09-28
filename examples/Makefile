# Every example is compiled here, so one that stops building fails the suite rather than
# somebody's bench. They are not linked: there is no main, and none is wanted.
CC     ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -I../include -I../src

EXAMPLES = $(wildcard */vehicle.c)

all:
	@for f in $(EXAMPLES); do \
	  $(CC) $(CFLAGS) -c $$f -o /dev/null 2>&1 \
	    && echo "  ok   $$f" || { echo "  FAIL $$f"; exit 1; }; \
	done
	@echo "every example compiles"

.PHONY: all
