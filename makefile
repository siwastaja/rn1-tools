CC = gcc
LD = gcc

CFLAGS = -Wall
LDFLAGS = -lncurses -lm

DEPS =

all: prog

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

prog: prog.o
	$(LD) -o prog $^ $(LDFLAGS) 

e:
	nano prog.c
