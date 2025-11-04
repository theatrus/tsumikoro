# Tsumikoro Bridge ESPHome Component

# Get the shared bus protocol source directory
set(TSUMIKORO_BUS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/tsumikoro_bus")

# Add the bus protocol sources
idf_component_register(
    SRCS
        "tsumikoro_bridge.cpp"
        "${TSUMIKORO_BUS_DIR}/src/tsumikoro_bus.c"
        "${TSUMIKORO_BUS_DIR}/src/tsumikoro_protocol.c"
        "${TSUMIKORO_BUS_DIR}/src/tsumikoro_crc.c"
        "${TSUMIKORO_BUS_DIR}/hal/esp32/tsumikoro_hal_esp32.c"
    INCLUDE_DIRS
        "."
        "${TSUMIKORO_BUS_DIR}/include"
        "${TSUMIKORO_BUS_DIR}/hal/esp32"
    REQUIRES
        driver
        esp_timer
)
