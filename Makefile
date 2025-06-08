# Makefile for Distance Vector routing algorithm
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g
TARGET = DistanceVector
SOURCE = DistanceVector.c

# Default target
all: $(TARGET)

# Build the executable
$(TARGET): $(SOURCE)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCE)

# Clean up compiled files
clean:
	rm -f $(TARGET)

# Phony targets
.PHONY: all clean