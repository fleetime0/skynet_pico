add_library(app 
    app.c
)

target_link_libraries(app
    pico_stdlib
    my_freertos
    cyw43_lwip
    my_micro_ros
    bsp
    icm45686
)

target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
