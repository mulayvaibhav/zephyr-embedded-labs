# Zephyr Embedded Labs

A multi-board Zephyr RTOS project demonstrating embedded software development across MCU peripherals, motor control, UART-based command interfaces, and portable application architecture.

This repository is structured as a professional embedded software codebase where application logic, reusable modules, board-specific configuration, and documentation are clearly separated. The project currently targets the **Microchip ATSAMV71 Xplained Ultra** board and is planned for porting to the **STM32MP257 Cortex-M33** core.

The goal of this project is to demonstrate practical embedded software skills using Zephyr RTOS, including peripheral integration, modular C design, board portability, DeviceTree-based configuration, and production-style project organization.

---

## Project Overview

This project demonstrates:

* Zephyr RTOS application development
* GPIO, UART, PWM, I2C, and SPI peripheral integration
* UART-based Bluetooth command reception
* DC motor control using external motor drivers
* L298N motor driver abstraction
* Modular embedded C architecture
* Board-specific configuration using Zephyr DeviceTree overlays
* Reusable application and driver-style components
* Multi-board portability from ATSAMV71 to STM32MP257 Cortex-M33
* Structured documentation for board bring-up, debugging, and porting

---

## Target Platforms

### Current Platform

```text
Board: Microchip ATSAMV71 Xplained Ultra
MCU:   ATSAMV71Q21B
Core:  ARM Cortex-M7
RTOS:  Zephyr
```

Current Zephyr board target:

```text
sam_v71_xult/samv71q21b
```

### Planned Platform

```text
Board: STM32MP257F-DK
MCU:   STM32MP257
Core:  ARM Cortex-M33
RTOS:  Zephyr on Cortex-M33
```

The STM32MP257 platform includes Cortex-A35 application cores and a Cortex-M33 microcontroller core. This repository is organized so selected Zephyr modules can be ported from ATSAMV71 to the STM32MP257 Cortex-M33 environment.

---

## Repository Structure

```text
zephyr-embedded-labs/
├── README.md
├── CMakeLists.txt
├── prj.conf
├── .gitignore
│
├── src/
│   └── main.c
│
├── include/
│   └── app_config.h
│
├── apps/
│   ├── gpio_led_counter/
│   │   ├── gpio_led_counter.c
│   │   └── gpio_led_counter.h
│   │
│   ├── uart_console/
│   │   ├── uart_console.c
│   │   └── uart_console.h
│   │
│   ├── bluetooth_rx/
│   │   ├── bluetooth_rx.c
│   │   └── bluetooth_rx.h
│   │
│   ├── pwm_led_fade/
│   │   ├── pwm_led_fade.c
│   │   └── pwm_led_fade.h
│   │
│   ├── motor_control_basic/
│   │   ├── motor_control_basic.c
│   │   └── motor_control_basic.h
│   │
│   ├── l298n_motor_driver/
│   │   ├── l298n_motor_driver.c
│   │   └── l298n_motor_driver.h
│   │
│   └── bluetooth_car_control/
│       ├── bluetooth_car_control.c
│       └── bluetooth_car_control.h
│
├── common/
│   ├── include/
│   │   ├── motor_types.h
│   │   └── app_error.h
│   │
│   └── src/
│       ├── motor_types.c
│       └── app_error.c
│
├── boards/
│   ├── sam_v71_xult/
│   │   ├── app.overlay
│   │   └── README.md
│   │
│   └── stm32mp257f_dk/
│       ├── app.overlay
│       └── README.md
│
└── docs/
    ├── atsamv71-xplained-ultra.md
    ├── stm32mp257f-dk-m33.md
    ├── porting-notes.md
    └── debugging-flashing.md
```

---

## Architecture

This repository uses a single Zephyr application entry point with multiple feature modules.

```text
src/main.c        -> Common application entry point
include/          -> Global project configuration
apps/             -> Feature-specific modules
common/           -> Reusable shared code
boards/           -> Board-specific overlays and configuration notes
docs/             -> Bring-up, debugging, and porting documentation
```

The design avoids creating a separate `main.c` for every feature. Instead, each feature is implemented as a separate module with its own `.c` and `.h` files. The active module is selected from a common configuration header.

This approach demonstrates:

* Clean project structure
* Modular C design
* Separation of application logic from board configuration
* Scalable code organization
* Easier portability across hardware platforms

---

## Application Selection

The active module is selected in:

```text
include/app_config.h
```

Example:

```c
#define APP_SELECT APP_BLUETOOTH_CAR_CONTROL
```

The root `src/main.c` calls the selected application module.

Example concept:

```c
int main(void)
{
    printk("Zephyr Embedded Labs\n");

#if APP_SELECT == APP_GPIO_LED_COUNTER
    gpio_led_counter_run();

#elif APP_SELECT == APP_UART_CONSOLE
    uart_console_run();

#elif APP_SELECT == APP_BLUETOOTH_RX
    bluetooth_rx_run();

#elif APP_SELECT == APP_MOTOR_CONTROL_BASIC
    motor_control_basic_run();

#elif APP_SELECT == APP_BLUETOOTH_CAR_CONTROL
    bluetooth_car_control_run();

#endif

    return 0;
}
```

This keeps the entry point stable while allowing multiple embedded features to be developed and tested independently.

---

## Modules

### GPIO LED Counter

Demonstrates GPIO output control using Zephyr GPIO APIs.

Responsibilities:

* Configure GPIO output
* Toggle LED
* Implement periodic timing using Zephyr kernel sleep APIs
* Validate board-level GPIO configuration

---

### UART Console

Demonstrates UART communication using Zephyr serial APIs.

Responsibilities:

* Initialize UART device
* Send data over UART
* Receive UART data
* Validate console and serial output configuration

---

### Bluetooth RX

Demonstrates UART-based Bluetooth command reception.

Responsibilities:

* Interface with an external Bluetooth module over UART
* Receive byte streams from a mobile application
* Handle printable and non-printable command bytes
* Provide a foundation for command parsing

---

### PWM LED Fade

Demonstrates PWM signal generation.

Responsibilities:

* Configure PWM output
* Generate variable duty-cycle signals
* Validate PWM timer configuration
* Provide a foundation for motor-speed control

---

### Motor Control Basic

Demonstrates GPIO-based DC motor direction control.

Responsibilities:

* Drive motor-control GPIO pins
* Implement forward, reverse, stop, and brake behavior
* Validate external motor driver wiring
* Separate motor-control logic from the application entry point

---

### L298N Motor Driver

Reusable motor driver abstraction for L298N-based DC motor control.

Responsibilities:

* Encapsulate L298N control logic
* Provide motor direction APIs
* Support integration with higher-level applications
* Prepare for PWM-based speed control

Example API direction:

```c
void l298n_motor_forward(void);
void l298n_motor_reverse(void);
void l298n_motor_stop(void);
void l298n_motor_brake(void);
```

---

### Bluetooth Car Control

Application-level module combining UART command reception and motor control.

Responsibilities:

* Receive movement commands over UART Bluetooth
* Decode command bytes
* Map commands to car movement actions
* Control motor driver module
* Implement forward, reverse, left, right, and stop behavior

This module demonstrates integration of multiple embedded subsystems into one application-level feature.

---

## Board Configuration Strategy

Board-specific configuration is isolated inside the `boards/` directory.

```text
boards/sam_v71_xult/app.overlay
boards/stm32mp257f_dk/app.overlay
```

Application modules should avoid hardcoded MCU pins. Instead, GPIO, UART, PWM, and peripheral mappings should be handled through Zephyr DeviceTree overlays.

Preferred approach:

```c
#define MOTOR_IN1_NODE DT_ALIAS(motor_in1)
#define MOTOR_IN2_NODE DT_ALIAS(motor_in2)
```

Each board overlay can then map the required aliases to the correct physical pins.

This provides:

* Cleaner application code
* Reduced hardware dependency
* Easier board migration
* Better maintainability
* Professional Zephyr-style configuration

---

## Build Instructions

Build for ATSAMV71 Xplained Ultra:

```bash
west build -b sam_v71_xult/samv71q21b . -p always -- -DEXTRA_DTC_OVERLAY_FILE=boards/sam_v71_xult/sam_v71.overlay
```

Flash the target:

```bash
west flash
```

For J-Link/Ozone based workflows, the generated ELF file can be loaded from:

```text
build/zephyr/zephyr.elf
```

---

## Configuration

Project-wide Zephyr configuration is kept in:

```text
prj.conf
```

Example configuration options may include:

```text
CONFIG_GPIO=y
CONFIG_SERIAL=y
CONFIG_UART_CONSOLE=y
CONFIG_PRINTK=y
CONFIG_PWM=y
```

Additional peripheral-specific options should be enabled only when required by the active module.

---

## Porting Plan: ATSAMV71 to STM32MP257 Cortex-M33

The repository is designed to support a controlled port from ATSAMV71 to STM32MP257 Cortex-M33.

Porting steps:

1. Build and validate a minimal Zephyr application for STM32MP257 Cortex-M33
2. Confirm serial console output
3. Add STM32MP257-specific DeviceTree overlay
4. Map GPIO aliases used by existing modules
5. Validate GPIO output
6. Validate UART communication
7. Validate PWM output
8. Port motor-control modules
9. Port Bluetooth UART command handling
10. Add board-specific documentation
11. Evaluate Cortex-A35 Linux to Cortex-M33 communication using RPMsg/OpenAMP

---

## Development Environment

Current development workflow:

```text
Host OS: Windows with WSL2 Ubuntu
Editor: Visual Studio Code
Build System: west + CMake
RTOS: Zephyr
Debugger: J-Link / Ozone
Target: ATSAMV71 Xplained Ultra
```

Planned expansion:

```text
Target: STM32MP257F-DK
RTOS Core: Cortex-M33
Linux Core: Cortex-A35
Inter-core Communication: RPMsg / OpenAMP
```

---

## Skills Demonstrated

This repository demonstrates the following embedded software skills:

* Zephyr RTOS application development
* Embedded C module design
* Peripheral driver integration
* GPIO, UART, PWM, I2C, and SPI usage
* DeviceTree overlay configuration
* CMake-based embedded build configuration
* Board bring-up and hardware validation
* Motor driver integration
* UART protocol handling
* Debugging using serial console and external debuggers
* Multi-board portability
* Cortex-M based embedded development
* Preparation for heterogeneous SoC development with STM32MP257

---

## Development Guidelines

The project follows these guidelines:

* Keep `src/main.c` minimal
* Keep each feature inside its own module under `apps/`
* Keep shared types and utilities under `common/`
* Keep board-specific configuration under `boards/`
* Avoid hardcoded physical pin numbers in application modules
* Prefer DeviceTree aliases for hardware mappings
* Keep APIs small and clear
* Document board-specific setup and debugging steps
* Keep commits focused and traceable

---

## Roadmap

Planned implementation order:

1. GPIO LED counter
2. UART console transmit and receive
3. UART Bluetooth receive
4. PWM output
5. Basic DC motor control
6. L298N driver abstraction
7. Bluetooth-controlled car application
8. I2C sensor integration
9. SPI peripheral experiment
10. STM32MP257 Cortex-M33 bring-up
11. STM32MP257 module port
12. RPMsg/OpenAMP communication between Linux and Cortex-M33

---

## Repository Description

Portable Zephyr RTOS project demonstrating MCU peripheral integration, motor control, UART Bluetooth command handling, and multi-board embedded software architecture for ATSAMV71 and STM32MP257 Cortex-M33.

---

## License

This project is currently maintained as a personal embedded software portfolio project. A license can be added later if the repository is published as an open-source reference.
