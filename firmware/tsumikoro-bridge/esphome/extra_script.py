"""PlatformIO extra script to add Tsumikoro bus protocol sources to build"""
Import("env")

# Add the bus protocol C sources to the build
env.BuildSources(
    "$BUILD_DIR/tsumikoro_bus",
    "$PROJECT_SRC_DIR/src",
    "+<*.c>"
)

env.BuildSources(
    "$BUILD_DIR/tsumikoro_hal",
    "$PROJECT_SRC_DIR/esp32",
    "+<*.c>"
)
