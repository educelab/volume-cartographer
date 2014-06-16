OBJECTS=scalpel.o
CC=clang++
CFLAGS=-g -Wall -Werror -Wextra -Wshadow `pkg-config --cflags opencv`
LIBS=`pkg-config --libs opencv`
BINARY=scalpel

all: $(BINARY)

$(BINARY): $(OBJECTS)
	$(CC) $(CFLAGS) $(LIBS) $(OBJECTS) -o $(BINARY)

clean:
	rm -f $(OBJECTS) $(BINARY)

.PHONY: clean
