SOURCES=main.c md5anim.c md5model.c md5lex.c geometry/quat.c geometry/v3.c
HEADERS=       md5anim.h md5model.h md5lex.h geometry/quat.h geometry/v3.h

PKG=gl glew allegro-5.0
CFLAGS=-g -ansi -Wall -O3 -funroll-loops -c \
	   -Igeometry `pkg-config --cflags $(PKG)`
LDFLAGS=-O3
LDLIBS=-lm `pkg-config --libs $(PKG)`

CC=gcc
OBJECTS=$(addsuffix .o, $(basename ${SOURCES}))
EXECUTABLE=main

all: $(EXECUTABLE)

zip: Makefile $(SOURCES) $(HEADERS)
	zip -r mypackage.zip $^

$(EXECUTABLE): $(OBJECTS)

$(OBJECTS): %.o: %.c $(HEADERS)

clean:
	rm -f $(EXECUTABLE) $(OBJECTS)

.PHONY: all clean
