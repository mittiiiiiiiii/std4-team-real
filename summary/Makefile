CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -lm
TARGET = task11
SOURCES = task11.c geometry.c graph.c
HEADERS = geometry.h graph.h

all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CC) -o $(TARGET) $(SOURCES) $(CFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
