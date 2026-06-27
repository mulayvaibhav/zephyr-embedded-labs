#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "bluetooth_rx.h"
#include "vehicle_command.h"
#include "vehicle_command_parser.h"
#include "vehicle_control_manager.h"

#define BT_UART_NODE DT_NODELABEL(usart0)

#define BT_THREAD_STACK_SIZE 1024
#define BT_THREAD_PRIORITY   5

#define BT_RX_MSGQ_SIZE      64
#define BT_ROLLING_BUF_SIZE  16

static const struct device *bt_uart = DEVICE_DT_GET(BT_UART_NODE);

K_MSGQ_DEFINE(bt_rx_msgq, sizeof(uint8_t), BT_RX_MSGQ_SIZE, 4);

K_THREAD_STACK_DEFINE(bt_stack_area, BT_THREAD_STACK_SIZE);
static struct k_thread bt_thread_data;
static k_tid_t bt_thread_id;

static void bluetooth_worker_thread(void *p1, void *p2, void *p3);

static bool bt_is_allowed_char(uint8_t c)
{
    return ((c >= 'A') && (c <= 'Z')) ||
           ((c >= 'a') && (c <= 'z')) ||
           ((c >= '0') && (c <= '9')) ||
           (c == '_');
}

static char bt_to_upper(uint8_t c)
{
    return (char)toupper((unsigned char)c);
}

static void shift_append_char(char *buf, size_t buf_size, char c)
{
    size_t len;

    if ((buf == NULL) || (buf_size < 2)) {
        return;
    }

    len = strlen(buf);

    if (len >= (buf_size - 1)) {
        memmove(&buf[0], &buf[1], buf_size - 2);
        buf[buf_size - 2] = c;
        buf[buf_size - 1] = '\0';
    } else {
        buf[len] = c;
        buf[len + 1] = '\0';
    }
}

static bool rolling_buffer_ends_with(const char *buf, const char *token)
{
    size_t buf_len;
    size_t token_len;

    if ((buf == NULL) || (token == NULL)) {
        return false;
    }

    buf_len = strlen(buf);
    token_len = strlen(token);

    if (buf_len < token_len) {
        return false;
    }

    return strcmp(&buf[buf_len - token_len], token) == 0;
}

static bool detect_command_from_stream(char incoming_char,
                                       char *detected_cmd,
                                       size_t detected_cmd_size)
{
    static char rolling_buf[BT_ROLLING_BUF_SIZE];

    if ((detected_cmd == NULL) || (detected_cmd_size == 0)) {
        return false;
    }

    detected_cmd[0] = '\0';

    shift_append_char(rolling_buf, sizeof(rolling_buf), incoming_char);

    /*
     * HC-05 / BLE Controller currently sends:
     *   UP
     *   DWN
     *   LFT
     *   RGT
     *
     * Keep these as transport-specific raw commands.
     * vehicle_parse_ascii_command() will convert them to VehicleMotionCommand.
     */

    if (rolling_buffer_ends_with(rolling_buf, "UP")) {
        strncpy(detected_cmd, "UP", detected_cmd_size - 1);
        detected_cmd[detected_cmd_size - 1] = '\0';
        return true;
    }

    if (rolling_buffer_ends_with(rolling_buf, "DWN")) {
        strncpy(detected_cmd, "DWN", detected_cmd_size - 1);
        detected_cmd[detected_cmd_size - 1] = '\0';
        return true;
    }

    if (rolling_buffer_ends_with(rolling_buf, "DOWN")) {
        strncpy(detected_cmd, "DOWN", detected_cmd_size - 1);
        detected_cmd[detected_cmd_size - 1] = '\0';
        return true;
    }

    if (rolling_buffer_ends_with(rolling_buf, "LFT")) {
        strncpy(detected_cmd, "LFT", detected_cmd_size - 1);
        detected_cmd[detected_cmd_size - 1] = '\0';
        return true;
    }

    if (rolling_buffer_ends_with(rolling_buf, "LEFT")) {
        strncpy(detected_cmd, "LEFT", detected_cmd_size - 1);
        detected_cmd[detected_cmd_size - 1] = '\0';
        return true;
    }

    if (rolling_buffer_ends_with(rolling_buf, "RGT")) {
        strncpy(detected_cmd, "RGT", detected_cmd_size - 1);
        detected_cmd[detected_cmd_size - 1] = '\0';
        return true;
    }

    if (rolling_buffer_ends_with(rolling_buf, "RIGHT")) {
        strncpy(detected_cmd, "RIGHT", detected_cmd_size - 1);
        detected_cmd[detected_cmd_size - 1] = '\0';
        return true;
    }

    if (rolling_buffer_ends_with(rolling_buf, "C")) {
        strncpy(detected_cmd, "C", detected_cmd_size - 1);
        detected_cmd[detected_cmd_size - 1] = '\0';
        return true;
    }

    //if (rolling_buffer_ends_with(rolling_buf, "D")) {
    //    strncpy(detected_cmd, "D", detected_cmd_size - 1);
    //    detected_cmd[detected_cmd_size - 1] = '\0';
    //    return true;
    //}

    return false;
}

static void bt_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    ARG_UNUSED(user_data);

    uart_irq_update(dev);

    while (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) > 0) {
            /*
             * Do not parse here.
             * Do not printk here.
             * Just queue the byte and return quickly.
             */
            (void)k_msgq_put(&bt_rx_msgq, &c, K_NO_WAIT);
        }
    }
}

void bluetooth_rx_init(void)
{
    uint8_t dummy;

    if (!device_is_ready(bt_uart)) {
        printk("Bluetooth UART not ready\n");
        return;
    }

    /*
     * Flush stale bytes before enabling RX interrupt.
     * This protects against garbage like 0x00 generated during board init.
     */
    while (uart_fifo_read(bt_uart, &dummy, 1) > 0) {
        printk("Bluetooth UART flush: dropped 0x%02X\n", dummy);
    }

    bt_thread_id = k_thread_create(
        &bt_thread_data,
        bt_stack_area,
        K_THREAD_STACK_SIZEOF(bt_stack_area),
        bluetooth_worker_thread,
        NULL, NULL, NULL,
        BT_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );

    uart_irq_callback_user_data_set(bt_uart, bt_uart_cb, NULL);
    uart_irq_rx_enable(bt_uart);

    printk("Bluetooth UART RX started\n");
}

static void bluetooth_worker_thread(void *p1, void *p2, void *p3)
{
    uint8_t c;
    char normalized_char;
    char detected_cmd[16];

    vehicle_motion_command_t out_cmd;
    vehicle_control_manager_t *vehicle_manager = NULL;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        if (vehicle_manager == NULL) {
            vehicle_manager = get_vehicle_manager_inst();
        }

        if (k_msgq_get(&bt_rx_msgq, &c, K_FOREVER) != 0) {
            continue;
        }

        /*
         * Ignore NUL and other garbage bytes.
         * This is important because board init can create transient junk.
         */
        if (c == '\0') {
            printk("BT RX: ignored NUL byte\n");
            continue;
        }

        if (!bt_is_allowed_char(c)) {
            printk("BT RX: ignored invalid byte hex=0x%02X\n", c);
            continue;
        }

        normalized_char = bt_to_upper(c);

        printk("BT RX: char='%c', hex=0x%02X\n", normalized_char, c);

        if (!detect_command_from_stream(normalized_char,
                                        detected_cmd,
                                        sizeof(detected_cmd))) {
            continue;
        }

        printk("Command Detected: %s\n", detected_cmd);

        if (vehicle_manager == NULL) {
            printk("Vehicle manager not ready\n");
            continue;
        }

        memset(&out_cmd, 0, sizeof(out_cmd));

        if (vehicle_parse_ascii_command(detected_cmd,
                                        VEHICLE_SOURCE_HC05_UART,
                                        k_uptime_get_32(),
                                        &out_cmd)) {
            vehicle_control_manager_handle_command(vehicle_manager,
                                                   &out_cmd);

            printk("Vehicle command accepted: raw=%s type=%d linear=%d angular=%d speed=%u\n",
                   detected_cmd,
                   out_cmd.command_type,
                   out_cmd.linear_x,
                   out_cmd.angular_z,
                   out_cmd.speed_limit_pct);
        } else {
            printk("Vehicle command parse failed: %s\n", detected_cmd);
        }
    }
}