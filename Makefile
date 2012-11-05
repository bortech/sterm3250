CC = gcc
CFLAGS = -Wall
PROG = sterm3250

all: $(PROG)

$(PROG): $(PROG).o

$(PROG).o: $(PROG).c


.PHONY: clean
clean:
	rm -f $(PROG) *.o

