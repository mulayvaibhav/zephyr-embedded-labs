#if 0

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>



#define LED0_NODE    DT_ALIAS(led0)
#define PA06_NODE    DT_ALIAS(pa06_out)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "led0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(PA06_NODE, okay)
#error "pa06_out alias is not defined"
#endif

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec pa06 = GPIO_DT_SPEC_GET(PA06_NODE, gpios);

int main(void)
{
    int ret;
    bool state = false;
    int counter = 0;

    if (!gpio_is_ready_dt(&led0)) {
        printk("LED0 GPIO device is not ready\n");
        return 0;
    }

    if (!gpio_is_ready_dt(&pa06)) {
        printk("PA06 GPIO device is not ready\n");
        return 0;
    }

    /*
     * GPIO_OUTPUT_INACTIVE configures the pin as output.
     * Since we do NOT use GPIO_OPEN_DRAIN, this is normal push-pull output.
     */
    ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure LED0\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&pa06, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure PA06\n");
        return 0;
    }

    printk("PA06 configured as push-pull GPIO output\n");

    while (1) {
        state = !state;

        gpio_pin_set_dt(&led0, state);
        gpio_pin_set_dt(&pa06, state);

        printk("Counter = %d, PA06 = %s\n",
               counter,
               state ? "HIGH" : "LOW");

        counter++;
        k_msleep(1000);
    }

    return 0;
}

#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#define BT_UART_NODE DT_NODELABEL(usart0)

static const struct device *bt_uart = DEVICE_DT_GET(BT_UART_NODE);

static void bt_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    static uint32_t count = 0;

    ARG_UNUSED(user_data);

    uart_irq_update(dev);

    while (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) == 1) {

            if (c >= 32 && c <= 126) {
                printk("BT RX: char='%c', hex=0x%02X\n", c, c);
            } else {
                printk("BT RX: non-printable, hex=0x%02X\n", c);
            }
        }
    }

    printk("%d\n", ++count);
}

int main(void)
{
    if (!device_is_ready(bt_uart)) {
        printk("Bluetooth UART not ready\n");
        return 0;
    }

    uart_irq_callback_user_data_set(bt_uart, bt_uart_cb, NULL);
    uart_irq_rx_enable(bt_uart);

    printk("Bluetooth UART test started\n");

    while (1) {
        uart_poll_out(bt_uart, 'H');
        uart_poll_out(bt_uart, 'i');
        uart_poll_out(bt_uart, '\r');
        uart_poll_out(bt_uart, '\n');

        k_sleep(K_SECONDS(1));
    }
}