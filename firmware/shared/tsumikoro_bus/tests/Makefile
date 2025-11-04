# Makefile for Tsumikoro Bus Protocol Unit Tests
#
# Copyright (c) 2025 Yann Ramin
# SPDX-License-Identifier: Apache-2.0

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -Werror -std=c11 -g -O0
CPPFLAGS = -I../include
LDFLAGS =

# Directories
SRC_DIR = ../src
INC_DIR = ../include
TEST_DIR = .
BUILD_DIR = build

# Source files
SOURCES = $(SRC_DIR)/tsumikoro_crc8.c \
          $(SRC_DIR)/tsumikoro_protocol.c

# Test executables
TEST_EXECUTABLES = test_crc8 test_protocol

# Object files
OBJECTS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SOURCES))

# Default target
.PHONY: all
all: $(TEST_EXECUTABLES)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

# Build test_crc8
test_crc8: $(BUILD_DIR)/test_crc8.o $(BUILD_DIR)/tsumikoro_crc8.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_crc8.o: test_crc8.c test_framework.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

# Build test_protocol
test_protocol: $(BUILD_DIR)/test_protocol.o $(BUILD_DIR)/tsumikoro_protocol.o $(BUILD_DIR)/tsumikoro_crc8.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_protocol.o: test_protocol.c test_framework.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

# Run tests
.PHONY: test
test: all
	@echo ""
	@echo "========================================="
	@echo "Running Tsumikoro Bus Protocol Unit Tests"
	@echo "========================================="
	@./test_crc8
	@echo ""
	@./test_protocol
	@echo ""
	@echo "========================================="
	@echo "All test suites completed"
	@echo "========================================="

# Run individual tests
.PHONY: test-crc8
test-crc8: test_crc8
	./test_crc8

.PHONY: test-protocol
test-protocol: test_protocol
	./test_protocol

# Clean build artifacts
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(TEST_EXECUTABLES)

# Help target
.PHONY: help
help:
	@echo "Tsumikoro Bus Protocol Unit Tests"
	@echo ""
	@echo "Available targets:"
	@echo "  all            - Build all tests (default)"
	@echo "  test           - Build and run all tests"
	@echo "  test-crc8      - Run CRC8 tests only"
	@echo "  test-protocol  - Run protocol tests only"
	@echo "  clean          - Remove build artifacts"
	@echo "  help           - Show this help message"
