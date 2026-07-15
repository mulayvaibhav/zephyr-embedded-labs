#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include "vehicle_control_manager.h"
#include "motor_driver.h"

#include <string.h>
#include <stdlib.h>

#define DEFAULT_SPEED_LIMIT_PCT       (25u)
#define SAFE_MAX_SPEED_LIMIT_PCT      (25u)

#define DEFAULT_TTL_MS                (400u)
#define DEFAULT_RAMP_UP_PCT_PER_SEC   (80u)
#define DEFAULT_RAMP_DOWN_PCT_PER_SEC (160u)
#define DEFAULT_MAX_UPDATE_DT_MS      (100u)

#define MY_STACK_SIZE 1024
#define MY_PRIORITY   5

K_THREAD_STACK_DEFINE(motor_driver_stack_area, MY_STACK_SIZE);

static struct k_thread motor_driver_thread_data;
static k_tid_t motor_driver_thread_id;

static vehicle_control_manager_t mgr;

static void run_motor_driver_thread(void *p1, void *p2, void *p3);

/**
 * @brief Clamp a signed 16-bit value between a minimum and maximum value.
 *
 * @param value Value to clamp.
 * @param min_value Minimum allowed value.
 * @param max_value Maximum allowed value.
 *
 * @return Clamped signed 16-bit value.
 */
static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief Clamp an unsigned 8-bit value between a minimum and maximum value.
 *
 * @param value Value to clamp.
 * @param min_value Minimum allowed value.
 * @param max_value Maximum allowed value.
 *
 * @return Clamped unsigned 8-bit value.
 */
static uint8_t clamp_u8(uint8_t value, uint8_t min_value, uint8_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief Clamp an unsigned 32-bit value between a minimum and maximum value.
 *
 * @param value Value to clamp.
 * @param min_value Minimum allowed value.
 * @param max_value Maximum allowed value.
 *
 * @return Clamped unsigned 32-bit value.
 */
static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief Get the absolute value of a signed 16-bit value.
 *
 * @param value Input signed 16-bit value.
 *
 * @return Absolute value.
 */
static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

/**
 * @brief Check whether two non-zero signed values have different signs.
 *
 * @param a First value.
 * @param b Second value.
 *
 * @return true if values are non-zero and have opposite signs, false otherwise.
 */
static bool different_sign_nonzero(int16_t a, int16_t b)
{
    return ((a > 0) && (b < 0)) || ((a < 0) && (b > 0));
}

/**
 * @brief Move a signed value toward a target value by a limited step.
 *
 * @param current Current value.
 * @param target Target value.
 * @param step Maximum step size.
 *
 * @return Updated value moved toward target.
 */
static int16_t move_toward_i16(int16_t current, int16_t target, uint16_t step)
{
    if (current == target) {
        return current;
    }

    if (step == 0u) {
        step = 1u;
    }

    if (current < target) {
        int32_t next = (int32_t)current + (int32_t)step;

        if (next > target) {
            next = target;
        }

        return (int16_t)next;
    }

    int32_t next = (int32_t)current - (int32_t)step;

    if (next < target) {
        next = target;
    }

    return (int16_t)next;
}

/**
 * @brief Calculate the ramp step for a given rate and elapsed time.
 *
 * @param rate_pct_per_sec Ramp rate in percentage per second.
 * @param dt_ms Elapsed time in milliseconds.
 *
 * @return Step size in percentage points.
 */
static uint16_t calculate_step(uint16_t rate_pct_per_sec, uint32_t dt_ms)
{
    uint32_t step = ((uint32_t)rate_pct_per_sec * dt_ms) / 1000u;

    if (step == 0u) {
        step = 1u;
    }

    if (step > 100u) {
        step = 100u;
    }

    return (uint16_t)step;
}

/**
 * @brief Apply slew-rate limiting to a motor percentage command.
 *
 * This function prevents sudden speed changes. If the motor direction changes
 * from forward to reverse or reverse to forward, it first ramps the command
 * down to zero before ramping up in the opposite direction.
 *
 * @param current Current motor percentage.
 * @param target Target motor percentage.
 * @param step_up Ramp-up step.
 * @param step_down Ramp-down step.
 *
 * @return Slew-limited motor percentage.
 */
static int16_t slew_limit_motor_pct(int16_t current,
                                    int16_t target,
                                    uint16_t step_up,
                                    uint16_t step_down)
{
    int16_t effective_target = target;
    uint16_t step;

    if (different_sign_nonzero(current, target)) {
        effective_target = 0;
        step = step_down;
    } else {
        if (abs_i16(target) > abs_i16(current)) {
            step = step_up;
        } else {
            step = step_down;
        }
    }

    return move_toward_i16(current, effective_target, step);
}

/**
 * @brief Calculate left and right motor target percentages.
 *
 * This function converts normalized linear and angular vehicle commands into
 * differential-drive left and right motor target percentages.
 *
 * @param mgr Vehicle control manager instance.
 */
static void calculate_differential_targets(vehicle_control_manager_t *mgr)
{
    int32_t left;
    int32_t right;
    int32_t max_abs;
    int32_t speed_limit;

    int16_t linear_x = clamp_i16(mgr->active_cmd.linear_x,
                                 VEHICLE_AXIS_MIN,
                                 VEHICLE_AXIS_MAX);

    int16_t angular_z = clamp_i16(mgr->active_cmd.angular_z,
                                  VEHICLE_AXIS_MIN,
                                  VEHICLE_AXIS_MAX);

    /*
     * Sign convention:
     *
     * angular_z > 0 means turn/rotate left.
     *
     * For pivot-left:
     *   linear_x  = 0
     *   angular_z = +1000
     *
     * We want:
     *   left  = -1000
     *   right = +1000
     *
     * So:
     *   left  = linear_x - angular_z
     *   right = linear_x + angular_z
     */
    left = (int32_t)linear_x - (int32_t)angular_z;
    right = (int32_t)linear_x + (int32_t)angular_z;

    /*
     * Normalize so that mixing never exceeds +/-1000.
     */
    max_abs = labs(left);

    if (labs(right) > max_abs) {
        max_abs = labs(right);
    }

    if (max_abs > VEHICLE_AXIS_MAX) {
        left = (left * VEHICLE_AXIS_MAX) / max_abs;
        right = (right * VEHICLE_AXIS_MAX) / max_abs;
    }

    /*
     * Enforce the safe maximum speed limit.
     */
    speed_limit = clamp_u8(mgr->current_speed_limit_pct,
                           VEHICLE_SPEED_MIN_PCT,
                           SAFE_MAX_SPEED_LIMIT_PCT);

    /*
     * Convert normalized request into signed motor percentage.
     * This is still only the target before ramp limiting.
     */
    mgr->target_left_pct = (int16_t)((left * speed_limit) / VEHICLE_AXIS_MAX);
    mgr->target_right_pct = (int16_t)((right * speed_limit) / VEHICLE_AXIS_MAX);

    mgr->target_left_pct = clamp_i16(mgr->target_left_pct, -100, +100);
    mgr->target_right_pct = clamp_i16(mgr->target_right_pct, -100, +100);
}

/**
 * @brief Check whether the active motion command has timed out.
 *
 * @param mgr Vehicle control manager instance.
 * @param now_ms Current uptime in milliseconds.
 *
 * @return true if no active command exists or the command has timed out,
 *         false otherwise.
 */
static bool command_has_timed_out(const vehicle_control_manager_t *mgr,
                                  uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (!mgr->has_active_motion_cmd) {
        return true;
    }

    elapsed_ms = now_ms - mgr->last_command_time_ms;

    return elapsed_ms > mgr->active_cmd.ttl_ms;
}

/**
 * @brief Get the global vehicle control manager instance.
 *
 * @return Pointer to the global vehicle control manager instance.
 */
vehicle_control_manager_t *get_vehicle_manager_inst(void)
{
    return &mgr;
}

/**
 * @brief Initialize the vehicle control manager.
 *
 * This initializes the manager configuration, applies safety limits, clears
 * runtime state, and starts the motor driver thread.
 *
 * @param config Optional configuration. If NULL, default configuration is used.
 */
void vehicle_control_manager_init(const vehicle_control_config_t *config)
{
    memset(&mgr, 0, sizeof(vehicle_control_manager_t));

    if (config != NULL) {
        mgr.config = *config;
    } else {
        mgr.config.default_speed_limit_pct = DEFAULT_SPEED_LIMIT_PCT;
        mgr.config.default_ttl_ms = DEFAULT_TTL_MS;
        mgr.config.ramp_up_pct_per_sec = DEFAULT_RAMP_UP_PCT_PER_SEC;
        mgr.config.ramp_down_pct_per_sec = DEFAULT_RAMP_DOWN_PCT_PER_SEC;
        mgr.config.max_update_dt_ms = DEFAULT_MAX_UPDATE_DT_MS;
    }

    mgr.config.default_speed_limit_pct =
        clamp_u8(mgr.config.default_speed_limit_pct,
                 0u,
                 SAFE_MAX_SPEED_LIMIT_PCT);

    if (mgr.config.default_ttl_ms == 0u) {
        mgr.config.default_ttl_ms = DEFAULT_TTL_MS;
    }

    if (mgr.config.ramp_up_pct_per_sec == 0u) {
        mgr.config.ramp_up_pct_per_sec = DEFAULT_RAMP_UP_PCT_PER_SEC;
    }

    if (mgr.config.ramp_down_pct_per_sec == 0u) {
        mgr.config.ramp_down_pct_per_sec = DEFAULT_RAMP_DOWN_PCT_PER_SEC;
    }

    if (mgr.config.max_update_dt_ms == 0u) {
        mgr.config.max_update_dt_ms = DEFAULT_MAX_UPDATE_DT_MS;
    }

    mgr.current_speed_limit_pct = mgr.config.default_speed_limit_pct;

    motor_driver_thread_id = k_thread_create(
        &motor_driver_thread_data,
        motor_driver_stack_area,
        K_THREAD_STACK_SIZEOF(motor_driver_stack_area),
        run_motor_driver_thread,
        NULL,
        NULL,
        NULL,
        MY_PRIORITY,
        0,
        K_NO_WAIT
    );

    ARG_UNUSED(motor_driver_thread_id);
}

/**
 * @brief Handle an incoming vehicle motion command.
 *
 * This function validates the command version, updates speed limits, handles
 * stop/emergency-stop commands, and stores active motion commands.
 *
 * @param mgr Vehicle control manager instance.
 * @param cmd Incoming vehicle motion command.
 */
int vehicle_control_manager_handle_command(vehicle_control_manager_t *mgr,
                                            const vehicle_motion_command_t *cmd)
{
    if ((mgr == NULL) || (cmd == NULL)) {
        return -1;
    }

    if (cmd->version != VEHICLE_COMMAND_VERSION) {
        return -1;
    }

    switch (cmd->command_type) {
    case VEHICLE_CMD_SET_SPEED:
        mgr->current_speed_limit_pct =
            clamp_u8(cmd->speed_limit_pct,
                     VEHICLE_SPEED_MIN_PCT,
                     SAFE_MAX_SPEED_LIMIT_PCT);
        break;

    case VEHICLE_CMD_STOP:
        /*
         * Normal stop:
         * - Clears emergency-stop latch.
         * - Removes active motion command.
         * - Requests target speed zero.
         * - Actual motor output ramps down through update().
         */
        mgr->emergency_stop_latched = false;
        mgr->has_active_motion_cmd = false;
        mgr->target_left_pct = 0;
        mgr->target_right_pct = 0;
        break;

    case VEHICLE_CMD_EMERGENCY_STOP:
        /*
         * Emergency stop:
         * - Immediate.
         * - Latched.
         * - Motor output goes to zero without normal ramp.
         */
        mgr->emergency_stop_latched = true;
        mgr->has_active_motion_cmd = false;
        mgr->target_left_pct = 0;
        mgr->target_right_pct = 0;
        mgr->current_left_pct = 0;
        mgr->current_right_pct = 0;
        break;

    case VEHICLE_CMD_MOTION:
        if (mgr->emergency_stop_latched) {
            return -1;
        }

        mgr->active_cmd = *cmd;

        mgr->active_cmd.linear_x =
            clamp_i16(mgr->active_cmd.linear_x,
                      VEHICLE_AXIS_MIN,
                      VEHICLE_AXIS_MAX);

        mgr->active_cmd.angular_z =
            clamp_i16(mgr->active_cmd.angular_z,
                      VEHICLE_AXIS_MIN,
                      VEHICLE_AXIS_MAX);

        if (mgr->active_cmd.ttl_ms == 0u) {
            mgr->active_cmd.ttl_ms = mgr->config.default_ttl_ms;
        }

        if (mgr->active_cmd.speed_limit_pct > 0u) {
            mgr->current_speed_limit_pct =
                clamp_u8(mgr->active_cmd.speed_limit_pct,
                         VEHICLE_SPEED_MIN_PCT,
                         SAFE_MAX_SPEED_LIMIT_PCT);
        }

        mgr->last_command_time_ms = cmd->timestamp_ms;
        mgr->has_active_motion_cmd = true;
        mgr->command_timeout_active = false;
        break;

    case VEHICLE_CMD_HEARTBEAT:
    default:
        break;
    }

    return 0;
}

/**
 * @brief Update the vehicle control manager and calculate motor output.
 *
 * This function should be called periodically. It handles command timeout,
 * target calculation, ramp limiting, emergency-stop behavior, and final
 * left/right motor output generation.
 *
 * @param mgr Vehicle control manager instance.
 * @param now_ms Current uptime in milliseconds.
 *
 * @return Vehicle motor output command.
 */
vehicle_motor_output_t vehicle_control_manager_update(vehicle_control_manager_t *mgr,
                                                      uint32_t now_ms)
{
    vehicle_motor_output_t output = {0};
    uint32_t dt_ms;
    uint16_t step_up;
    uint16_t step_down;

    if (mgr == NULL) {
        return output;
    }

    if (mgr->last_update_time_ms == 0u) {
        mgr->last_update_time_ms = now_ms;
    }

    dt_ms = now_ms - mgr->last_update_time_ms;
    mgr->last_update_time_ms = now_ms;

    dt_ms = clamp_u32(dt_ms, 1u, mgr->config.max_update_dt_ms);

    if (mgr->emergency_stop_latched) {
        mgr->current_left_pct = 0;
        mgr->current_right_pct = 0;
        mgr->target_left_pct = 0;
        mgr->target_right_pct = 0;

        output.left_pct = 0;
        output.right_pct = 0;
        output.emergency_stop_active = true;
        output.command_timeout_active = false;

        return output;
    }

    if (command_has_timed_out(mgr, now_ms)) {
        mgr->command_timeout_active = true;
        mgr->target_left_pct = 0;
        mgr->target_right_pct = 0;
    } else {
        mgr->command_timeout_active = false;
        calculate_differential_targets(mgr);
    }

    step_up = calculate_step(mgr->config.ramp_up_pct_per_sec, dt_ms);
    step_down = calculate_step(mgr->config.ramp_down_pct_per_sec, dt_ms);

    mgr->current_left_pct =
        slew_limit_motor_pct(mgr->current_left_pct,
                             mgr->target_left_pct,
                             step_up,
                             step_down);

    mgr->current_right_pct =
        slew_limit_motor_pct(mgr->current_right_pct,
                             mgr->target_right_pct,
                             step_up,
                             step_down);

    output.left_pct = mgr->current_left_pct;
    output.right_pct = mgr->current_right_pct;
    output.emergency_stop_active = mgr->emergency_stop_latched;
    output.command_timeout_active = mgr->command_timeout_active;

    return output;
}

/**
 * @brief Clear the emergency-stop latch.
 *
 * This resets the emergency-stop state and clears current and target motor
 * output values.
 *
 * @param mgr Vehicle control manager instance.
 */
void vehicle_control_manager_clear_emergency_stop(vehicle_control_manager_t *mgr)
{
    if (mgr == NULL) {
        return;
    }

    mgr->emergency_stop_latched = false;
    mgr->has_active_motion_cmd = false;
    mgr->target_left_pct = 0;
    mgr->target_right_pct = 0;
    mgr->current_left_pct = 0;
    mgr->current_right_pct = 0;
}

/**
 * @brief Motor driver thread.
 *
 * This thread periodically updates the vehicle control manager and applies
 * the resulting left/right motor output to the motor driver.
 *
 * @param p1 Unused thread argument.
 * @param p2 Unused thread argument.
 * @param p3 Unused thread argument.
 */
static void run_motor_driver_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    vehicle_motor_output_t motor_out = {0};
    vehicle_control_manager_t *mgr = NULL;

    int ret = 0;
    int16_t last_left_pct = 0;
    int16_t last_right_pct = 0;
    bool output_applied_once = false;

    while (1) {
        if (mgr == NULL) {
            mgr = get_vehicle_manager_inst();

            if (mgr == NULL) {
                printk("Motor thread: vehicle manager not ready\n");
                k_msleep(100);
                continue;
            }
        }

        motor_out = vehicle_control_manager_update(mgr, k_uptime_get_32());

        if ((!output_applied_once) ||
            (last_left_pct != motor_out.left_pct) ||
            (last_right_pct != motor_out.right_pct)) {

            printk("Applying motor outputs: left_pct=%d, right_pct=%d\n",
                   motor_out.left_pct,
                   motor_out.right_pct);

            ret = motor_driver_set_left_right(motor_out.left_pct,
                                              motor_out.right_pct);

            if (ret != 0) {
                printk("Motor driver failed, ret=%d\n", ret);

                /*
                 * Do not update last_left_pct / last_right_pct here.
                 * We want to retry next loop.
                 */
            } else {
                last_left_pct = motor_out.left_pct;
                last_right_pct = motor_out.right_pct;
                output_applied_once = true;
            }
        }

        k_msleep(20);
    }
}

int vehicle_control_manager_submit_command( const vehicle_motion_command_t *cmd ) {
    vehicle_control_manager_handle_command( get_vehicle_manager_inst(), cmd );

    return 0;
}