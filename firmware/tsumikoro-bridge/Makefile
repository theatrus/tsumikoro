# Makefile for tsumikoro-bridge ESPHome project
# Uses uv for Python dependency management

.PHONY: help setup build upload logs clean validate dashboard recreate-venv

DEVICE_NAME = tsumikoro-bridge
CONFIG_FILE = esphome/$(DEVICE_NAME).yaml
VENV_DIR = .venv

help:
	@echo "Tsumikoro Bridge ESPHome Build System"
	@echo ""
	@echo "Available targets:"
	@echo "  setup         - Install dependencies using uv"
	@echo "  recreate-venv - Recreate virtual environment with pip support"
	@echo "  build         - Compile firmware"
	@echo "  upload        - Upload firmware to device"
	@echo "  logs          - View device logs"
	@echo "  clean         - Clean build artifacts"
	@echo "  validate      - Validate configuration"
	@echo "  dashboard     - Start ESPHome dashboard"
	@echo ""
	@echo "Example usage:"
	@echo "  make setup          # First time setup"
	@echo "  make recreate-venv  # Recreate venv if needed"
	@echo "  make build          # Build firmware"
	@echo "  make upload         # Upload to device"

# Create venv with pip support (needed for PlatformIO)
$(VENV_DIR):
	@echo "Creating virtual environment with pip support..."
	@if ! command -v uv &> /dev/null; then \
		echo "Error: uv is not installed"; \
		echo "Install with: curl -LsSf https://astral.sh/uv/install.sh | sh"; \
		exit 1; \
	fi
	uv venv --seed

# Recreate venv from scratch
recreate-venv:
	@echo "Recreating virtual environment..."
	rm -rf $(VENV_DIR)
	$(MAKE) $(VENV_DIR)
	$(MAKE) setup

setup: $(VENV_DIR)
	@echo "Setting up tsumikoro-bridge environment with uv..."
	uv sync
	@echo "Setup complete!"
	@echo "Don't forget to copy esphome/secrets.yaml.example to esphome/secrets.yaml"

build:
	@echo "Building $(DEVICE_NAME)..."
	uv run esphome compile $(CONFIG_FILE)

upload:
	@echo "Uploading $(DEVICE_NAME)..."
	uv run esphome upload $(CONFIG_FILE)

upload-ota:
	@echo "Uploading $(DEVICE_NAME) over-the-air..."
	uv run esphome upload $(CONFIG_FILE) --device $(DEVICE_NAME).local

logs:
	@echo "Viewing logs from $(DEVICE_NAME)..."
	uv run esphome logs $(CONFIG_FILE)

clean:
	@echo "Cleaning build artifacts..."
	uv run esphome clean $(CONFIG_FILE)
	rm -rf .esphome

validate:
	@echo "Validating configuration..."
	uv run esphome config $(CONFIG_FILE)

dashboard:
	@echo "Starting ESPHome dashboard..."
	uv run esphome dashboard esphome/

# Generate encryption key
generate-key:
	@echo "Generating new API encryption key:"
	@uv run python -c "import secrets; print(secrets.token_bytes(32).hex())"

# Check secrets file
check-secrets:
	@if [ ! -f esphome/secrets.yaml ]; then \
		echo "Warning: esphome/secrets.yaml not found!"; \
		echo "Copy esphome/secrets.yaml.example to esphome/secrets.yaml and configure it."; \
		exit 1; \
	fi
