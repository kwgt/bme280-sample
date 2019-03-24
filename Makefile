CFLAGS  := -O2 -Wall
LDFLAGS := -lpthread

all : test

%.o.c :
	${CC} ${CFLAGS} -c -o $@ $<

test : main.o i2c.o
	${CC} -o $@ $^ ${LDFLAGS}

plot :
	gnuplot plot.gp

clean :
	rm -f test *.o *.svg
