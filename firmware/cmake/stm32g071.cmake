# STM32G071 MCU Configuration

set(MCU_FAMILY STM32G0)
set(MCU_LINE STM32G071xx)

# CPU and FPU settings
set(CPU "-mcpu=cortex-m0plus")
set(FPU "")
set(FLOAT_ABI "")

# Common compiler flags
set(MCU_FLAGS "${CPU} -mthumb ${FPU} ${FLOAT_ABI}")

# C flags
set(CMAKE_C_FLAGS_INIT "${MCU_FLAGS} -Wall -Wextra -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Os -g -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG")

# ASM flags
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS} -x assembler-with-cpp")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCU_FLAGS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage")

# MCU-specific definitions
add_compile_definitions(
    ${MCU_LINE}
    ${MCU_FAMILY}
    USE_HAL_DRIVER
)
