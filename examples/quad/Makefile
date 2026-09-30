# A quad you can fly. Builds the firmware, the SDK and the integration into one binary.
SDK   := ../..
CFLAGS := -std=c99 -Wall -Wextra -Werror -I$(SDK)/include -I.
SRC   := vehicle.c ardudeck_link.c main.c $(wildcard $(SDK)/src/*.c)

quad: $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

run: quad
	./quad

clean:
	rm -f quad

.PHONY: run clean
