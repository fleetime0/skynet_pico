add_library(microros STATIC IMPORTED GLOBAL)
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk/libmicroros/libmicroros.a
)

add_library(my_micro_ros 
    my_micro_ros.c
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/pico_uart_transport.c
)

target_link_libraries(my_micro_ros
    pico_stdlib
    hardware_dma
    microros
    my_freertos
)

target_include_directories(my_micro_ros 
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk/libmicroros/include
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/transport
)
