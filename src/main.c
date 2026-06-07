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