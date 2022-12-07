PROGRAMS := RS41D RS92D

LDLIBS = -lm
CC = gcc
CFLAGS = -O3

.PHONY: all
all: $(PROGRAMS)

RS41D: RS41D.o demod_mod.o bch_ecc_mod.o

RS92D: RS92D.o demod_mod.o bch_ecc_mod.o

demod_mod.o: CFLAGS += -Ofast
demod_mod.o: demod_mod.h

bch_ecc_mod.o: bch_ecc_mod.h

.PHONY: clean
clean:
	rm -f $(PROGRAMS) $(PROGRAMS:=.o)
	rm -f demod_mod.o
	rm -f bch_ecc_mod.o

