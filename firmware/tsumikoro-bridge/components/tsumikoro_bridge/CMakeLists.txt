# Tsumikoro Bridge ESPHome Component
# Build timestamp: 2025-11-08 12:02 - Enable HAL debug flag

# Get the shared bus protocol source directory
set(TSUMIKORO_BUS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/tsumikoro_bus")

# Add the bus protocol sources (symlinked to shared files)
idf_component_register(
    SRCS
        "tsumikoro_bridge.cpp"
        "tsumikoro_hal_esp32.c"
        "tsumikoro_bus.c"
        "tsumikoro_protocol.c"
        "tsumikoro_crc8.c"
        "tsumikoro_rtos.c"
    INCLUDE_DIRS
        "."
        "${TSUMIKORO_BUS_DIR}/include"
    REQUIRES
        driver
        esp_timer
)

# Enable debug logging and RTOS mode for bus protocol
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    TSUMIKORO_HAL_DEBUG=1
    TSUMIKORO_BUS_DEBUG=1
    TSUMIKORO_BUS_USE_RTOS=1
)
