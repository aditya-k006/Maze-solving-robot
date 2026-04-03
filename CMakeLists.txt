cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(maze_robot C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Generate the PIO header for the software UART (used for TFMini Right sensor)
pico_generate_pio_header(maze_robot
    ${CMAKE_CURRENT_LIST_DIR}/src/sensors/uart_rx.pio
)

add_executable(maze_robot
    src/main.c
    src/motor/l298n.c
    src/motor/mecanum.c
    src/sensors/mpu6050.c
    src/sensors/tfmini.c
    src/control/pid.c
    src/maze/map.c
    src/maze/floodfill.c
)

target_include_directories(maze_robot PRIVATE src)

target_link_libraries(maze_robot
    pico_stdlib
    hardware_uart
    hardware_i2c
    hardware_pwm
    hardware_pio
    hardware_clocks
    m   # math library for fabsf etc.
)

pico_enable_stdio_usb(maze_robot 1)
pico_enable_stdio_uart(maze_robot 0)

pico_add_extra_outputs(maze_robot)
