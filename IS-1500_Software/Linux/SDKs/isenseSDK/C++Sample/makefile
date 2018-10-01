#
# Makefile for Linux
#

ifndef BITS
BITS = -m32
#BITS = -m64
endif

C =		gcc $(BITS) -c -DUNIX -DLINUX
L =		gcc $(BITS)
LIBS =		-ldl

all:  		ismain

ismain:		main.o isense.o
		$(L) -o $@ main.o isense.o $(LIBS)

main.o:		main.c *.h
		$(C) main.c

isense.o:	isense.c *.h
		$(C) isense.c

clean:
	  rm -f *.o ismain
