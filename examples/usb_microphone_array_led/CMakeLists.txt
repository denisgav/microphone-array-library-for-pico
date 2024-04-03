cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(usb_microphone_array_led
    main.c
    usb_descriptors.c
    usb_microphone.c
)

target_include_directories(usb_microphone_array_led PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(usb_microphone_array_led PRIVATE tinyusb_device tinyusb_board pico_stdlib pico_microphone_array_i2s pico_9822 pico_9822_led_animation)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(usb_microphone_array_led)
