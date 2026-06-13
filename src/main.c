#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "bluetooth_rx.h"

int main(void)
{
    bluetooth_rx_init();

    while (1) {
        
        k_sleep(K_SECONDS(1));
    }
}