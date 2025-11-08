# Makefile for Tsumikoro Firmware Projects
# Provides convenient targets for building STM32 and ESP32 projects
#
# IMPORTANT: Keep targets in sync with .github/workflows/build.yml!
# When adding/removing firmware targets, update both:
# - firmware/Makefile (this file)
# - .github/workflows/build.yml (CI builds)

.PHONY: help clean test reset
.PHONY: ministepper-g071 servo-g030 servo-g071 nucleo-g071rb nucleo-g071rb-freertos
.PHONY: flash-ministepper-g071 flash-servo-g030 flash-servo-g071 flash-nucleo-g071rb flash-nucleo-g071rb-freertos
.PHONY: esp-bridge esp-bridge-upload esp-bridge-logs

# Default target
help:
	@echo "Tsumikoro Firmware Build System"
	@echo ""
	@echo "STM32 Build Targets (using CMake + arm-none-eabi-gcc):"
	@echo "  ministepper-g071        Build tsumikoro-ministepper for STM32G071"
	@echo "  servo-g030              Build tsumikoro-servo for STM32G030"
	@echo "  servo-g071              Build tsumikoro-servo for STM32G071"
	@echo "  nucleo-g071rb           Build NUCLEO-G071RB firmware (bare-metal)"
	@echo "  nucleo-g071rb-freertos  Build NUCLEO-G071RB firmware (FreeRTOS)"
	@echo ""
	@echo "STM32 Flash Targets (using st-flash):"
	@echo "  flash-ministepper-g071         Flash tsumikoro-ministepper"
	@echo "  flash-servo-g030               Flash tsumikoro-servo (STM32G030)"
	@echo "  flash-servo-g071               Flash tsumikoro-servo (STM32G071)"
	@echo "  flash-nucleo-g071rb            Flash NUCLEO-G071RB (bare-metal)"
	@echo "  flash-nucleo-g071rb-freertos   Flash NUCLEO-G071RB (FreeRTOS)"
	@echo "  reset                          Reset STM32 target"
	@echo ""
	@echo "ESP32 Projects (using ESPHome):"
	@echo "  esp-bridge          Build tsumikoro-bridge (ESP32-S3)"
	@echo "  esp-bridge-upload   Upload tsumikoro-bridge firmware"
	@echo "  esp-bridge-logs     View tsumikoro-bridge logs"
	@echo ""
	@echo "Testing:"
	@echo "  test                Run protocol unit tests"
	@echo ""
	@echo "Cleaning:"
	@echo "  clean               Clean all build artifacts"
	@echo "  clean-<project>     Clean specific project"
	@echo ""
	@echo "Examples:"
	@echo "  make nucleo-g071rb-freertos            # Build FreeRTOS firmware"
	@echo "  make flash-nucleo-g071rb-freertos      # Flash FreeRTOS firmware"
	@echo "  make servo-g030 flash-servo-g030       # Build and flash servo"
	@echo "  make reset                              # Reset STM32"
	@echo "  make test                               # Run tests"

# STM32 Projects
ministepper-g071:
	@echo "Building tsumikoro-ministepper for STM32G071..."
	@mkdir -p build/ministepper-g071
	@cd build/ministepper-g071 && \
		cmake ../.. -DPROJECT=tsumikoro-ministepper -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release && \
		make -j$$(nproc)
	@echo "Build complete: build/ministepper-g071/tsumikoro-ministepper.elf"

servo-g030:
	@echo "Building tsumikoro-servo for STM32G030..."
	@mkdir -p build/servo-g030
	@cd build/servo-g030 && \
		cmake ../.. -DPROJECT=tsumikoro-servo -DMCU=STM32G030 -DCMAKE_BUILD_TYPE=Release && \
		make -j$$(nproc)
	@echo "Build complete: build/servo-g030/tsumikoro-servo.elf"

servo-g071:
	@echo "Building tsumikoro-servo for STM32G071..."
	@mkdir -p build/servo-g071
	@cd build/servo-g071 && \
		cmake ../.. -DPROJECT=tsumikoro-servo -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release && \
		make -j$$(nproc)
	@echo "Build complete: build/servo-g071/tsumikoro-servo.elf"

nucleo-g071rb:
	@echo "Building nucleo-g071rb firmware (bare-metal)..."
	@mkdir -p build/nucleo-g071rb
	@cd build/nucleo-g071rb && \
		cmake ../.. -DPROJECT=nucleo-g071rb -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release && \
		make -j$$(nproc)
	@echo "Build complete: build/nucleo-g071rb/nucleo-g071rb.elf"

nucleo-g071rb-freertos:
	@echo "Building nucleo-g071rb-freertos firmware (FreeRTOS)..."
	@mkdir -p build/nucleo-g071rb-freertos
	@cd build/nucleo-g071rb-freertos && \
		cmake ../.. -DPROJECT=nucleo-g071rb-freertos -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release && \
		make -j$$(nproc)
	@echo "Build complete: build/nucleo-g071rb-freertos/nucleo-g071rb-freertos.elf"

# STM32 Flash Targets
flash-ministepper-g071:
	@echo "Flashing tsumikoro-ministepper for STM32G071..."
	st-flash --reset write build/ministepper-g071/tsumikoro-ministepper/tsumikoro-ministepper.bin 0x08000000

flash-servo-g030:
	@echo "Flashing tsumikoro-servo for STM32G030..."
	st-flash --reset write build/servo-g030/tsumikoro-servo/tsumikoro-servo.bin 0x08000000

flash-servo-g071:
	@echo "Flashing tsumikoro-servo for STM32G071..."
	st-flash --reset write build/servo-g071/tsumikoro-servo/tsumikoro-servo.bin 0x08000000

flash-nucleo-g071rb:
	@echo "Flashing nucleo-g071rb firmware (bare-metal)..."
	st-flash --reset write build/nucleo-g071rb/nucleo-g071rb/nucleo-g071rb.bin 0x08000000

flash-nucleo-g071rb-freertos:
	@echo "Flashing nucleo-g071rb-freertos firmware (FreeRTOS)..."
	st-flash --reset write build/nucleo-g071rb-freertos/nucleo-g071rb-freertos/nucleo-g071rb-freertos.bin 0x08000000

# Reset STM32 target
reset:
	@echo "Resetting STM32 target..."
	st-flash reset

# ESP32 Projects
esp-bridge:
	@echo "Building tsumikoro-bridge (ESP32-S3)..."
	@$(MAKE) -C tsumikoro-bridge build

esp-bridge-upload:
	@echo "Uploading tsumikoro-bridge..."
	@$(MAKE) -C tsumikoro-bridge upload

esp-bridge-logs:
	@echo "Viewing tsumikoro-bridge logs..."
	@$(MAKE) -C tsumikoro-bridge logs

# Testing
test:
	@echo "Running protocol unit tests..."
	@cd shared/tsumikoro_bus/tests && \
		make clean && make && \
		./test_crc8 && \
		./test_protocol && \
		./test_integration && \
		./test_bus_handler
	@echo "All tests passed!"

# Cleaning
clean:
	@echo "Cleaning all build artifacts..."
	@./clean.sh
	@$(MAKE) -C tsumikoro-bridge clean
	@echo "Clean complete!"

clean-ministepper-g071:
	@./clean.sh ministepper-g071

clean-servo-g030:
	@./clean.sh servo-g030

clean-servo-g071:
	@./clean.sh servo-g071

clean-nucleo-g071rb:
	@./clean.sh nucleo-g071rb

clean-nucleo-g071rb-freertos:
	@./clean.sh nucleo-g071rb-freertos

clean-esp-bridge:
	@$(MAKE) -C tsumikoro-bridge clean
