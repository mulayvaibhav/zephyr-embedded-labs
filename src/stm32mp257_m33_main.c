#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>

#include <errno.h>
#include <string.h>

#include "ipc.h"

static int cmd_vehicle(const struct shell *shell, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(shell, "Usage: vehicle <forward|backward|left|right|stop|estop>");
        return -EINVAL;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "forward") == 0) {
        shell_print(shell, "vehicle: forward");
    } else if (strcmp(cmd, "backward") == 0) {
        shell_print(shell, "vehicle: backward");
    } else if (strcmp(cmd, "left") == 0) {
        shell_print(shell, "vehicle: left");
    } else if (strcmp(cmd, "right") == 0) {
        shell_print(shell, "vehicle: right");
    } else if (strcmp(cmd, "stop") == 0) {
        shell_print(shell, "vehicle: stop");
    } else if (strcmp(cmd, "estop") == 0) {
        shell_print(shell, "vehicle: emergency stop");
    } else {
        shell_error(shell, "Unknown vehicle command: %s", cmd);
        return -EINVAL;
    }

    return 0;
}

SHELL_CMD_REGISTER(vehicle, NULL, "Vehicle control command", cmd_vehicle);

int main(void)
{
    printk("\n\n----->>>> \t Custom motor control starting \t <<<<-----\n\n");

#ifdef HW_BOARD_ATSAMV71
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
#endif

    start_stm32mp257_openamp_ipc();

    while (1) {        
        k_sleep(K_SECONDS(1));
    }
}