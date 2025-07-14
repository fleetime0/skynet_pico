add_library(bsp 
    bsp.c
    bsp_led.c
    bsp_beep.c
    bsp_key.c
    bsp_adc.c
    bsp_icm_i2c.c
)

target_link_libraries(bsp
    pico_stdlib
    hardware_adc
    hardware_i2c
    cyw43_lwip
)

target_include_directories(bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
