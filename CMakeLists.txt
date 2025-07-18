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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(skynet_pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(freertos)
add_subdirectory(micro_ros)
add_subdirectory(bsp)
add_subdirectory(icm45686)
add_subdirectory(app)

# Add executable. Default name is the project name, version 0.1

add_executable(skynet_pico src/main.c)

pico_set_program_name(skynet_pico "skynet_pico")
pico_set_program_version(skynet_pico "0.1")

# Modify the below lines to enable/disable output over UART/USB

add_compile_definitions(
    PICO_STDIO_UART=1
    PICO_DEFAULT_UART=1
    PICO_DEFAULT_UART_TX_PIN=4
    PICO_DEFAULT_UART_RX_PIN=5
)

pico_enable_stdio_uart(skynet_pico 1)
pico_enable_stdio_usb(skynet_pico 0)

# Add the standard library to the build
target_link_libraries(skynet_pico
    pico_stdlib
)

# Add the standard include files to the build
target_include_directories(skynet_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(skynet_pico 
    bsp
    app
)

pico_add_extra_outputs(skynet_pico)
