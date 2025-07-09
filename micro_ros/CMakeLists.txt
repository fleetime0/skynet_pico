add_library(microros STATIC IMPORTED GLOBAL)
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk/libmicroros/libmicroros.a
)

set(transport ${CMAKE_CURRENT_SOURCE_DIR}/transport/pico_uart_transport.c)
set(cyw43_backend "")

if(PICO_CYW43_SUPPORTED)
    set(transport ${CMAKE_CURRENT_SOURCE_DIR}/transport/pico_wifi_transport.c)
    list(APPEND cyw43_backend
        pico_cyw43_arch_lwip_sys_freertos
    )
endif()

add_library(my_micro_ros 
    ${transport}
    my_micro_ros.c
)

if(PICO_CYW43_SUPPORTED)
    target_compile_definitions(my_micro_ros PRIVATE MICRO_ROS_WIFI_SUPPORTED=1)
else()
    target_compile_definitions(my_micro_ros PRIVATE MICRO_ROS_WIFI_SUPPORTED=0)
endif()

target_compile_definitions(my_micro_ros PUBLIC 
    NO_SYS=0
)

target_link_libraries(my_micro_ros
    pico_stdlib
    microros
    my_freertos
    ${cyw43_backend}
)

target_include_directories(my_micro_ros 
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk/libmicroros/include
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/transport
)
