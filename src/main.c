#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define LED0_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "led0 alias is not defined in the board devicetree"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
    int ret;
    int counter = 0;
    bool led_state = false;

    if (!gpio_is_ready_dt(&led)) {
        printk("LED GPIO device is not ready\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure LED GPIO\n");
        return 0;
    }

    printk("My first Zephyr app started on ATSAMV71\n");

    while (1) {
        led_state = !led_state;
        gpio_pin_set_dt(&led, led_state);

        printk("Counter = %d, LED = %s\n",
               counter,
               led_state ? "ON" : "OFF");

        counter++;
        k_msleep(1000);
    }

    return 0;
}
