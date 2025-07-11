add_library(cyw43_lwip INTERFACE)

target_compile_definitions(cyw43_lwip INTERFACE 
    NO_SYS=0
)

target_link_libraries(cyw43_lwip INTERFACE
    pico_cyw43_arch_lwip_sys_freertos
    my_freertos
)

target_include_directories(cyw43_lwip INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
