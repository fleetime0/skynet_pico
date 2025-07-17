add_library(bsp 
    bsp.c
    bsp_led.c
    bsp_beep.c
    bsp_key.c
    bsp_adc.c
    bsp_icm_i2c.c
    bsp_ssd1306.c
    oled_fonts.c
    bsp_motor.c
    bsp_encoder.c
    bsp_flash.c
)

pico_generate_pio_header(bsp ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_link_libraries(bsp
    pico_stdlib
    pico_cyw43_arch_none
    hardware_adc
    hardware_i2c
    hardware_spi
    hardware_pwm
    hardware_pio
    hardware_flash
    hardware_sync
)

target_include_directories(bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
