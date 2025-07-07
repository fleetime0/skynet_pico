link_directories(libmicroros)

# link_directories(libmicroros)
# add_executable(main
#     main.c
#     interface/common/transport_common.c
#     interface/wifi/pico_wifi_transport.c
#     interface/uart/pico_uart_transport.c
# )
# target_link_libraries(main
#     pico_stdlib
#     microros
#     hardware_pwm
# )

# target_include_directories(main PUBLIC
#     libmicroros/include
#     ${CMAKE_CURRENT_LIST_DIR}
#     ${CMAKE_CURRENT_LIST_DIR}/..
# )