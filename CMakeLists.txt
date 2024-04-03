cmake_minimum_required(VERSION 3.12)

# initialize pico_sdk from GIT
# (note this can come from environment, CMake cache etc)
# set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()

include(pico_sdk_import.cmake)

project(pico_microphone)

# initialize the Pico SDK
pico_sdk_init()

###################################################
# Microphone array library
###################################################
add_library(pico_microphone_array_i2s INTERFACE)

target_sources(pico_microphone_array_i2s INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/microphone_array_i2s.c
)

target_include_directories(pico_microphone_array_i2s INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)

pico_generate_pio_header(pico_microphone_array_i2s ${CMAKE_CURRENT_LIST_DIR}/src/microphone_array_i2s.pio)

target_link_libraries(pico_microphone_array_i2s INTERFACE pico_stdlib hardware_dma hardware_pio)

###################################################

###################################################
# SK9822 library
###################################################
add_library(pico_9822 INTERFACE)

target_sources(pico_9822 
    INTERFACE ${CMAKE_CURRENT_LIST_DIR}/src/sk9822.c
)

target_include_directories(pico_9822 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)

pico_generate_pio_header(pico_9822 ${CMAKE_CURRENT_LIST_DIR}/src/sk9822.pio)

target_link_libraries(pico_9822 INTERFACE pico_stdlib hardware_dma hardware_pio)
###################################################

###################################################
# SK9822 animation
###################################################
add_library(pico_9822_led_animation INTERFACE)

target_sources(pico_9822_led_animation 
    INTERFACE ${CMAKE_CURRENT_LIST_DIR}/src/sk9822_led_animation.c
)

target_include_directories(pico_9822_led_animation INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)

target_link_libraries(pico_9822_led_animation INTERFACE pico_stdlib pico_9822)
###################################################

add_subdirectory("examples/hello_microphone_array")
add_subdirectory("examples/usb_microphone_array")
add_subdirectory("examples/usb_microphone_array_led")
add_subdirectory("examples/sk9822")
