#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include "bluetooth_rx.h"

#define BT_UART_NODE DT_NODELABEL(usart0)

static const struct device *bt_uart = DEVICE_DT_GET(BT_UART_NODE);
static char cmd_buf[4] = {0}; // 3 chars + null terminator

static void parse_direction_stream(char incoming_char)
{
    // Shift the buffer left by 1 position to make room for the new character
    cmd_buf[0] = cmd_buf[1];
    cmd_buf[1] = cmd_buf[2];
    cmd_buf[2] = incoming_char;
    cmd_buf[3] = '\0'; // Ensure string is always null-terminated

    // Check 3-character commands using strcmp
    if (strcmp(cmd_buf, "DWN") == 0) {
        printk("Command Detected: DOWN\n");
        // Add your down-movement code here
        printk("STOP\n");
        memset(cmd_buf, 0, sizeof(cmd_buf)); // Clear buffer after match
    } 
    else if (strcmp(cmd_buf, "LFT") == 0) {
        printk("Command Detected: LEFT\n");
        // Add your left-movement code here
        memset(cmd_buf, 0, sizeof(cmd_buf));
    } 
    else if (strcmp(cmd_buf, "RGT") == 0) {
        printk("Command Detected: RIGHT\n");
        // Add your right-movement code here
        memset(cmd_buf, 0, sizeof(cmd_buf));
    }
    // Check 2-character command ("UP") using the last two positions
    else if (cmd_buf[1] == 'U' && cmd_buf[2] == 'P') {
        printk("Command Detected: UP\n");
        // Add your up-movement code here
        printk("Start running forward\n");
        memset(cmd_buf, 0, sizeof(cmd_buf));
    }
}

static void bt_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    static uint32_t count = 0;
    ARG_UNUSED(user_data);
    // Directs Zephyr to update internal peripheral tracking status
    uart_irq_update(dev);

    // CRITICAL: Loop while data is available in the hardware FIFO
    while (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) > 0) {
            parse_direction_stream(c);
            if (c >= 32 && c <= 126) {
                printk("BT RX: char='%c', hex=0x%02X\n", c, c);
            } else {
                printk("BT RX: non-printable, hex=0x%02X\n", c);
            }
        }
    }

    printk("%d\n", ++count);
}

void bluetooth_rx_init(void) {
    if (!device_is_ready(bt_uart)) {
        printk("Bluetooth UART not ready\n");
        return;
    }

    uart_irq_callback_user_data_set(bt_uart, bt_uart_cb, NULL);
    uart_irq_rx_enable(bt_uart);

    printk("Bluetooth UART test started\n");
}