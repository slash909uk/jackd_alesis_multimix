# Go build stuff..

CFLAGS=$(shell pkg-config --cflags libusb-1.0 jack)
LIBS=$(shell pkg-config --libs libusb-1.0 jack) -lm

all: bin bin/jackd_alesis_multimix

clean:
	rm -rf bin

bin:
	mkdir -p bin

bin/%: %.c
	$(CC) -o $@ $(CFLAGS) $< $(LIBS)
