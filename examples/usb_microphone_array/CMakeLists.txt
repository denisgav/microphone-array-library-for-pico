cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(usb_microphone_array
    main.c
    usb_descriptors.c
    usb_microphone.c
)

target_include_directories(usb_microphone_array PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(usb_microphone_array PRIVATE tinyusb_device tinyusb_board pico_stdlib pico_microphone_array_i2s)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(usb_microphone_array)
