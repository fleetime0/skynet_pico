set(FREERTOS_KERNEL_PATH "${CMAKE_SOURCE_DIR}/freertos/FreeRTOS-Kernel")
include(${CMAKE_SOURCE_DIR}/cmake/FreeRTOS_Kernel_import.cmake)

add_library(my_freertos INTERFACE)

target_compile_definitions(my_freertos INTERFACE 
    configNUMBER_OF_CORES=1
)

target_link_libraries(my_freertos INTERFACE
    FreeRTOS-Kernel-Heap4
)

target_include_directories(my_freertos INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
