cmake_minimum_required(VERSION 3.12)

add_executable(sk9822)

target_sources(sk9822 PRIVATE main.c)

target_link_libraries(sk9822 PRIVATE
        pico_stdlib
        hardware_pio
        pico_9822
        )

pico_add_extra_outputs(sk9822)

# add url via pico_set_program_url
#example_auto_set_url(sk9822)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(sk9822)
