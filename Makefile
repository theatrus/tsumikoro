# Tsumikoro Build System - Top-Level Makefile
# Builds all firmware projects (STM32 and ESP32)

.PHONY: help build build-all clean clean-all setup
.PHONY: build-servo build-ministepper build-bridge
.PHONY: clean-servo clean-ministepper clean-bridge
.PHONY: test test-bus-handler

# Build directories
FIRMWARE_DIR = firmware
BUILD_DIR = $(FIRMWARE_DIR)/build
SERVO_BUILD_DIR = $(BUILD_DIR)/servo-g030
MINISTEPPER_BUILD_DIR = $(BUILD_DIR)/ministepper-g071
BRIDGE_DIR = $(FIRMWARE_DIR)/tsumikoro-bridge

# CMake projects
SERVO_PROJECT = tsumikoro-servo
SERVO_MCU = STM32G030
MINISTEPPER_PROJECT = tsumikoro-ministepper
MINISTEPPER_MCU = STM32G071

# Default target
help:
	@echo "Tsumikoro Build System"
	@echo ""
	@echo "Build Targets:"
	@echo "  build              - Build all firmware projects"
	@echo "  build-servo        - Build servo controller (STM32G030)"
	@echo "  build-ministepper  - Build ministepper controller (STM32G071)"
	@echo "  build-bridge       - Build ESP32/ESPHome bridge"
	@echo ""
	@echo "Clean Targets:"
	@echo "  clean              - Clean all build artifacts"
	@echo "  clean-servo        - Clean servo controller build"
	@echo "  clean-ministepper  - Clean ministepper controller build"
	@echo "  clean-bridge       - Clean ESP32 bridge build"
	@echo ""
	@echo "Test Targets:"
	@echo "  test               - Run all tests"
	@echo "  test-bus-handler   - Run bus handler unit tests"
	@echo ""
	@echo "Setup:"
	@echo "  setup              - Setup all project dependencies"
	@echo ""
	@echo "Example workflow:"
	@echo "  make build         # Build all projects"
	@echo "  make clean         # Clean everything"
	@echo "  make build-servo   # Build only servo controller"

# Build all projects
build: build-servo build-ministepper build-bridge
	@echo ""
	@echo "✅ All projects built successfully!"

build-all: build

# Build servo controller (STM32G030)
build-servo:
	@echo "========================================"
	@echo "Building Servo Controller (STM32G030)"
	@echo "========================================"
	@mkdir -p $(SERVO_BUILD_DIR)
	@cd $(SERVO_BUILD_DIR) && \
		cmake ../.. -DPROJECT=$(SERVO_PROJECT) -DMCU=$(SERVO_MCU) && \
		make -j$$(nproc)
	@echo ""
	@echo "✅ Servo controller built successfully"
	@echo "   Output: $(SERVO_BUILD_DIR)/tsumikoro-servo.elf"
	@echo ""

# Build ministepper controller (STM32G071)
build-ministepper:
	@echo "========================================"
	@echo "Building Ministepper (STM32G071)"
	@echo "========================================"
	@mkdir -p $(MINISTEPPER_BUILD_DIR)
	@cd $(MINISTEPPER_BUILD_DIR) && \
		cmake ../.. -DPROJECT=$(MINISTEPPER_PROJECT) -DMCU=$(MINISTEPPER_MCU) && \
		make -j$$(nproc)
	@echo ""
	@echo "✅ Ministepper built successfully"
	@echo "   Output: $(MINISTEPPER_BUILD_DIR)/tsumikoro-ministepper.elf"
	@echo ""

# Build ESP32 bridge
build-bridge:
	@echo "========================================"
	@echo "Building ESP32 Bridge"
	@echo "========================================"
	@cd $(BRIDGE_DIR) && $(MAKE) build
	@echo ""
	@echo "✅ ESP32 bridge built successfully"
	@echo ""

# Clean all projects
clean: clean-servo clean-ministepper clean-bridge
	@echo "✅ All build artifacts cleaned"

# Clean servo controller
clean-servo:
	@echo "Cleaning servo controller build..."
	@rm -rf $(SERVO_BUILD_DIR)

# Clean ministepper controller
clean-ministepper:
	@echo "Cleaning ministepper controller build..."
	@rm -rf $(MINISTEPPER_BUILD_DIR)

# Clean ESP32 bridge
clean-bridge:
	@echo "Cleaning ESP32 bridge build..."
	@cd $(BRIDGE_DIR) && $(MAKE) clean

# Setup all projects
setup:
	@echo "Setting up project dependencies..."
	@echo "Initializing git submodules..."
	git submodule update --init --recursive
	@echo ""
	@echo "Setting up ESP32 bridge environment..."
	@cd $(BRIDGE_DIR) && $(MAKE) setup
	@echo ""
	@echo "✅ Setup complete!"
	@echo ""
	@echo "Prerequisites for STM32 builds:"
	@echo "  - ARM GCC toolchain (arm-none-eabi-gcc)"
	@echo "  - CMake 3.20+"
	@echo "  - Make or Ninja"
	@echo ""
	@echo "See firmware/README.md for detailed installation instructions."

# Test targets
test:
	@echo "========================================"
	@echo "Running All Tests"
	@echo "========================================"
	@$(MAKE) test-bus-handler

test-bus-handler:
	@echo "Running bus protocol unit tests..."
	@mkdir -p $(BUILD_DIR)/tests
	@cd $(BUILD_DIR)/tests && \
		cmake ../.. -DPROJECT=tests && \
		make && \
		./test_bus_handler && \
		./test_crc8 && \
		./test_protocol && \
		./test_integration
	@echo "✅ All bus handler tests passed"
