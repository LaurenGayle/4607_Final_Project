# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Pull in Pico Extras for the I2S audio libraries
set(PICO_EXTRAS_PATH "${PICO_SDK_PATH}/../pico-extras" CACHE PATH "Path to Pico Extras")
include(${PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

project(Demo_Scripts C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Demo_Scripts Demo_Scripts.c )

pico_set_program_name(Demo_Scripts "Demo_Scripts")
pico_set_program_version(Demo_Scripts "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Demo_Scripts 0)
pico_enable_stdio_usb(Demo_Scripts 1)

# Add the standard library to the build
target_link_libraries(Demo_Scripts
        pico_stdlib
        hardware_i2c
        hardware_pio
        hardware_dma
        pico_audio_i2s
        pico_cyw43_arch_poll
        pico_btstack_cyw43
        pico_btstack_classic
        pico_btstack_sbc_decoder)

# Bluetooth only - no Wi-Fi / lwIP networking stack
target_compile_definitions(Demo_Scripts PRIVATE CYW43_LWIP=0)

# Add the standard include files to the build
target_include_directories(Demo_Scripts PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Demo_Scripts 
        
        )

pico_add_extra_outputs(Demo_Scripts)

