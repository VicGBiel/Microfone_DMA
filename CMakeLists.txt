cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(valid_mic C CXX ASM)
pico_sdk_init()

add_subdirectory(lib/FatFs_SPI)    

add_executable(${PROJECT_NAME}  
        valid_mic.c 
        )
        
pico_generate_pio_header(valid_mic ${CMAKE_CURRENT_LIST_DIR}/i2s_mic.pio)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${PROJECT_NAME} 
        pico_stdlib
        hardware_pio
        hardware_dma  
        hardware_irq   
        hardware_clocks
        FatFs_SPI
        )

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})