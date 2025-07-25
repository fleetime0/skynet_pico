add_library(app 
    app.c
    app_oled.c
    app_motion.c
    app_pid.c
    app_bat.c
    app_flash.c
    skynet_node.c
)

target_link_libraries(app
    pico_stdlib
    my_freertos
    my_micro_ros
    bsp
    icm45686
    pid_debug
)

target_compile_definitions(app PRIVATE
    PID_ASSISTANT_EN=0
)

target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
