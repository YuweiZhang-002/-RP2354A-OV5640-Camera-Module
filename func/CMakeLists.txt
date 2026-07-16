# func is a reusable driver layer.
# The executable entry point lives in the top-level CMakeLists.txt.

add_library(func STATIC
    ov5640.c
    ov5640_set.c
    timer.c
    cam_pio.c
    fpga_pio.c
    image_process.c
)

target_compile_options(func PRIVATE -O3)

target_include_directories(func PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../header
)

pico_generate_pio_header(func
    ${CMAKE_CURRENT_LIST_DIR}/../cam_pio.pio
)

pico_generate_pio_header(func
    ${CMAKE_CURRENT_LIST_DIR}/../fpga_pio.pio
)

target_link_libraries(func PUBLIC
    header
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_spi
    hardware_pio
    hardware_dma
    hardware_pwm
    hardware_clocks
)
