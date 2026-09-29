# A rover you can run. Builds the firmware, the SDK and the integration into one binary.
SDK   := ../..
CFLAGS := -std=c99 -Wall -Wextra -Werror -I$(SDK)/include -I.
SRC   := vehicle.c ardudeck_link.c main.c $(wildcard $(SDK)/src/*.c)

rover: $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

run: rover
	./rover

clean:
	rm -f rover

.PHONY: run clean
