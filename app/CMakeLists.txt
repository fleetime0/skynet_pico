add_library(app 
    app.c
)

target_link_libraries(app
    pico_stdlib
    my_micro_ros
)

target_include_directories(app 
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
