set(imu_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/inv_imu_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/inv_imu_driver_advanced.c
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/inv_imu_driver_aux1.c
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/inv_imu_edmp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/inv_imu_selftest.c
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/inv_imu_transport.c
)

add_library(icm45686
    ${imu_sources}
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MadgwickAHRS.c
)

target_link_libraries(icm45686
    bsp
)

target_include_directories(icm45686
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/imu
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
