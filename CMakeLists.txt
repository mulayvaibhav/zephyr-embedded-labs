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

    apps/vehicle_command_parser/vehicle_command_parser.c
)

if("${HW_BOARD}" STREQUAL "ATSAMV71")
    target_sources(app PRIVATE
        src/atsamv71_main.c

        apps/bluetooth_rx/bluetooth_rx.c
        apps/vehicle_control_manager/vehicle_control_manager.c
        apps/vehicle_command_parser/vehicle_ipc_msg_processor.c
        
        apps/net/udp.c
        apps/net/tcp.c
        apps/net/echo-server.c

        drivers/motor_driver/motor_driver.c
    )
endif()

if("${HW_BOARD}" STREQUAL "STM32MP257")
    target_sources(app PRIVATE
        src/stm32mp257_m33_main.c

        apps/stm32mp257_m33/ipc/ipc.c
        apps/stm32mp257_m33/rpmsg/rpmsg_processor.c
    )
endif()

target_include_directories(app PRIVATE
    include

    apps/vehicle_command_parser
    
)

if("${HW_BOARD}" STREQUAL "ATSAMV71")
    target_include_directories(app PRIVATE
        apps/bluetooth_rx
        apps/vehicle_control_manager
        apps/net

        drivers/motor_driver
    )
endif()

if("${HW_BOARD}" STREQUAL "STM32MP257")
    target_include_directories(app PRIVATE
        apps/stm32mp257_m33/ipc
        apps/stm32mp257_m33/rpmsg
    )
endif()