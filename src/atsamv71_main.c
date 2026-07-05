#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>

#include <errno.h>
#include <string.h>

#include "vehicle_control_manager.h"
#include "motor_driver.h"
#include "bluetooth_rx.h"

int main(void)
{
    printk("\n\n----->>>> \t Custom motor control starting \t <<<<-----\n\n");
    int ret = 0;

    vehicle_control_config_t config = {
        .default_speed_limit_pct = 40,
        .default_ttl_ms = 400,

        /*
         * Gentle initial values:
         * 0% to 100% takes about 1.25 seconds.
         * 100% to 0% takes about 0.6 seconds.
         */
        .ramp_up_pct_per_sec = 80,
        .ramp_down_pct_per_sec = 160,

        .max_update_dt_ms = 100,
    };


    /* Initialize motor driver */
    ret = motor_driver_init();
    if (ret != 0) {
        printk("motor_driver_init failed: %d\n", ret);
        //return 0;
    }
    else {
        printk("motor_driver_init success\n");
    }

    /* Initialize Vehicle command manager */
    vehicle_control_manager_init( &config );


    bluetooth_rx_init();

    while (1) {        
        k_sleep(K_SECONDS(1));
    }
}