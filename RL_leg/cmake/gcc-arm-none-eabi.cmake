set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Prefer an explicitly selected toolchain, then PATH, then the workspace copy.
set(ARM_TOOLCHAIN_BIN "$ENV{ARM_TOOLCHAIN_BIN}" CACHE PATH "ARM GNU toolchain bin directory")
if(NOT ARM_TOOLCHAIN_BIN)
    find_program(ARM_GCC arm-none-eabi-gcc
        HINTS "${CMAKE_CURRENT_LIST_DIR}/../../.tools/arm/usr/bin")
    if(NOT ARM_GCC)
        message(FATAL_ERROR "ARM GCC not found. Install gcc-arm-none-eabi or set ARM_TOOLCHAIN_BIN.")
    endif()
    get_filename_component(ARM_TOOLCHAIN_BIN "${ARM_GCC}" DIRECTORY)
endif()

# Resolve each executable instead of reconstructing extensionless paths.
# On Windows, CMake needs the actual .exe path for cross compilers.
find_program(ARM_GCC arm-none-eabi-gcc
    HINTS "${ARM_TOOLCHAIN_BIN}" REQUIRED)
find_program(ARM_GXX arm-none-eabi-g++
    HINTS "${ARM_TOOLCHAIN_BIN}" REQUIRED)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy
    HINTS "${ARM_TOOLCHAIN_BIN}" REQUIRED)
find_program(ARM_SIZE arm-none-eabi-size
    HINTS "${ARM_TOOLCHAIN_BIN}" REQUIRED)

set(CMAKE_C_COMPILER "${ARM_GCC}")
set(CMAKE_CXX_COMPILER "${ARM_GXX}")
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
set(CMAKE_OBJCOPY "${ARM_OBJCOPY}" CACHE FILEPATH "objcopy")
set(CMAKE_SIZE "${ARM_SIZE}" CACHE FILEPATH "size")

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT "${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_CXX_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -fno-exceptions -fno-rtti -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS} -x assembler-with-cpp")
set(CMAKE_C_FLAGS_DEBUG_INIT "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Og -g3")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCU_FLAGS} --specs=nano.specs --specs=nosys.specs -T\"${CMAKE_CURRENT_LIST_DIR}/../STM32F427xx_FLASH.ld\" -Wl,-Map=RL_leg.map,--gc-sections,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES m)
