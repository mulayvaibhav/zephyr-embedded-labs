cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_embedded_labs)

target_sources(app PRIVATE
    src/main.c

    apps/bluetooth_rx/bluetooth_rx.c
    apps/vehicle_command_parser/vehicle_command_parser.c
    apps/vehicle_control_manager/vehicle_control_manager.c

    drivers/motor_driver/motor_driver.c
)

target_include_directories(app PRIVATE
    include
    apps/bluetooth_rx
    apps/vehicle_command_parser
    apps/vehicle_control_manager

    drivers/motor_driver
)