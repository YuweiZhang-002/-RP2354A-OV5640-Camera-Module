# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13...3.27)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()


if(NOT DEFINED PICO_PLATFORM)
    set(PICO_PLATFORM rp2350)
endif()
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/pico-sdk-master/pico_sdk_import.cmake)

project(new_camera_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(header)
add_subdirectory(func)

add_executable(new_camera_project_app
    main.c
)

target_link_libraries(new_camera_project_app
    PRIVATE
        pico_stdlib
        func
)

# stdio 走 USB；UART0(GP0/GP1) 让给 IMU→FPGA 数据线 + PWM 参考时钟，避免抢占
pico_enable_stdio_uart(new_camera_project_app 0)
pico_enable_stdio_usb(new_camera_project_app 1)

pico_add_extra_outputs(new_camera_project_app)