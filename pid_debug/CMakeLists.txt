add_library(pid_debug 
    pid_debug.c
    protocol.c
)

target_link_libraries(pid_debug
    pico_stdlib
)

target_include_directories(pid_debug PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
