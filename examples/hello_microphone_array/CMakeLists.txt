cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(hello_microphone_array
    main.c
)

target_link_libraries(hello_microphone_array  pico_microphone_array_i2s  pico_stdlib)

# enable usb output, disable uart output
pico_enable_stdio_usb(hello_microphone_array 1)
pico_enable_stdio_uart(hello_microphone_array 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(hello_microphone_array)
