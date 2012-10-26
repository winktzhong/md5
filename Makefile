CC=gcc
CFLAGS=-g -ansi -Wall -O2 -funroll-loops -c
LDFLAGS=-O2
LDLIBS=-lm
SOURCES=main.c md5.c geometry.c
HEADERS=geometry.h
OBJECTS=$(addsuffix .o, $(basename ${SOURCES}))
EXECUTABLE=md5

all: $(EXECUTABLE)

zip: Makefile $(SOURCES) $(HEADERS)
	zip -r mypackage.zip $^

$(EXECUTABLE): $(OBJECTS)

$(OBJECTS): %.o: %.c $(HEADERS)

clean:
	rm -f $(EXECUTABLE) $(OBJECTS)

.PHONY: all clean
