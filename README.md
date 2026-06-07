# Zephyr LED Counter on ATSAMV71

This is my first custom Zephyr RTOS application for the Microchip/Atmel SAM V71 Xplained Ultra board.

The application blinks LED0 and prints a counter value over the UART serial console.

## Board

Target board:

```bash
sam_v71_xult/samv71q21b
```

## Features

- Blinks LED0
- Prints a counter on UART
- Uses Zephyr GPIO driver
- Uses `printk()` for logging
- Uses `k_msleep()` for timing delay

## Project Structure

```text
led_counter/
├── CMakeLists.txt
├── prj.conf
├── README.md
├── .gitignore
└── src/
    └── main.c
```

## Prerequisites

Zephyr is installed separately at:

```bash
~/zephyrproject/zephyr
```

Python virtual environment is located at:

```bash
~/zephyrproject/.venv
```

## Build

Activate the Zephyr Python environment:

```bash
source ~/zephyrproject/.venv/bin/activate
```

Set the Zephyr base path:

```bash
export ZEPHYR_BASE=~/zephyrproject/zephyr
```

Build the application:

```bash
west build -p always -b sam_v71_xult/samv71q21b .
```

## Flash

Make sure the board is connected and attached to WSL2.

From Windows PowerShell as Administrator:

```powershell
usbipd list
usbipd attach --wsl --busid 4-3
```

Replace `4-3` with the actual BUSID of the EDBG/CMSIS-DAP debugger.

Then flash from WSL:

```bash
west flash
```

## Serial Console

Open the UART console:

```bash
picocom -b 115200 /dev/ttyACM0
```

Expected output:

```text
My first Zephyr app started on ATSAMV71
Counter = 0, LED = ON
Counter = 1, LED = OFF
Counter = 2, LED = ON
```

Exit `picocom`:

```text
Ctrl + A
Ctrl + X
```

## Clean Build

To rebuild from scratch:

```bash
west build -p always -b sam_v71_xult/samv71q21b .
```

## Notes

- Do not commit the `build/` directory.
- Do not commit Zephyr source code or the Zephyr SDK into this repository.
- This repository contains only the custom application source code.