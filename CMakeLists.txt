cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_embedded_labs)

if(NOT DEFINED HW_BOARD)
    message(FATAL_ERROR "HW_BOARD is not defined. Pass -DHW_BOARD=ATSAMV71 or -DHW_BOARD=STM32MP257")
endif()

message(STATUS "HW_BOARD = ${HW_BOARD}")

if(HW_BOARD STREQUAL "ATSAMV71")
    target_compile_definitions(app PRIVATE HW_BOARD_ATSAMV71)

elseif(HW_BOARD STREQUAL "STM32MP257")
    target_compile_definitions(app PRIVATE HW_BOARD_STM32MP257)

else()
    message(FATAL_ERROR "Unsupported HW_BOARD: ${HW_BOARD}")
endif()

target_sources(app PRIVATE
    src/main.c

    apps/vehicle_command_parser/vehicle_command_parser.c
    #apps/vehicle_control_manager/vehicle_control_manager.c

    #drivers/motor_driver/motor_driver.c
)

if("${HW_BOARD}" STREQUAL "ATSAMV71")
    target_sources(app PRIVATE
        apps/bluetooth_rx/bluetooth_rx.c
    )
endif()

if("${HW_BOARD}" STREQUAL "STM32MP257")
    target_sources(app PRIVATE
        apps/stm32mp257_m33_ipc/ipc.c
    )
endif()

target_include_directories(app PRIVATE
    include

    apps/vehicle_command_parser
    #apps/vehicle_control_manager

    #drivers/motor_driver
)

if("${HW_BOARD}" STREQUAL "ATSAMV71")
    target_include_directories(app PRIVATE
        apps/bluetooth_rx
    )
endif()

if("${HW_BOARD}" STREQUAL "STM32MP257")
    target_include_directories(app PRIVATE
        apps/stm32mp257_m33_ipc
    )
endif()