cmake_minimum_required(VERSION 3.25)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

project(Vector C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17) 

pico_sdk_init()


add_executable(${PROJECT_NAME}
    main.c
    LCD_display/lcd_display.c
)

include_directories(./LCD_display)

pico_add_extra_outputs(${PROJECT_NAME})

# pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/PIO_PROJECT.pio)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    
    hardware_gpio
    hardware_pio
    hardware_spi
    hardware_pwm
)


pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)