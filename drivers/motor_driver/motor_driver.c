#include "motor_driver.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

/*
 * Required devicetree aliases:
 *
 * GPIO direction pins:
 *   motor-lf-in1
 *   motor-lf-in2
 *   motor-lr-in1
 *   motor-lr-in2
 *   motor-rf-in1
 *   motor-rf-in2
 *   motor-rr-in1
 *   motor-rr-in2
 *
 * PWM enable pins:
 *   motor-lf-pwm
 *   motor-lr-pwm
 *   motor-rf-pwm
 *   motor-rr-pwm
 *
 * Notes:
 * - ENA/ENB on L298N should be connected to PWM pins.
 * - IN1/IN2/IN3/IN4 should be connected to GPIO pins.
 * - Remove ENA/ENB jumper caps from the L298N boards if present.
 * - Motor battery ground, L298N ground, and ATSAMV71 ground must be common.
 */

#define MOTOR_DIRECTION_CHANGE_DEADTIME_MS  (5)

/*
 * If any motor spins opposite to the expected direction,
 * change that motor's invert flag to 1.
 */
#define MOTOR_LF_INVERT_DIRECTION   (0)
#define MOTOR_LR_INVERT_DIRECTION   (0)
#define MOTOR_RF_INVERT_DIRECTION   (0)
#define MOTOR_RR_INVERT_DIRECTION   (0)

typedef enum {
    MOTOR_ID_LEFT_FRONT = 0,
    MOTOR_ID_LEFT_REAR,
    MOTOR_ID_RIGHT_FRONT,
    MOTOR_ID_RIGHT_REAR,
    MOTOR_ID_COUNT
} motor_id_t;

typedef struct {
    const char *name;

    struct gpio_dt_spec in1;
    struct gpio_dt_spec in2;
    struct pwm_dt_spec pwm;

    bool invert_direction;

    int16_t current_pct;
} motor_channel_t;

static motor_channel_t g_motors[MOTOR_ID_COUNT] = {
    [MOTOR_ID_LEFT_FRONT] = {
        .name = "left_front",
        .in1 = GPIO_DT_SPEC_GET(DT_ALIAS(motor_lf_in1), gpios),
        .in2 = GPIO_DT_SPEC_GET(DT_ALIAS(motor_lf_in2), gpios),
        .pwm = PWM_DT_SPEC_GET(DT_ALIAS(motor_lf_pwm)),
        .invert_direction = MOTOR_LF_INVERT_DIRECTION,
        .current_pct = 0,
    },

    [MOTOR_ID_LEFT_REAR] = {
        .name = "left_rear",
        .in1 = GPIO_DT_SPEC_GET(DT_ALIAS(motor_lr_in1), gpios),
        .in2 = GPIO_DT_SPEC_GET(DT_ALIAS(motor_lr_in2), gpios),
        .pwm = PWM_DT_SPEC_GET(DT_ALIAS(motor_lr_pwm)),
        .invert_direction = MOTOR_LR_INVERT_DIRECTION,
        .current_pct = 0,
    },

    [MOTOR_ID_RIGHT_FRONT] = {
        .name = "right_front",
        .in1 = GPIO_DT_SPEC_GET(DT_ALIAS(motor_rf_in1), gpios),
        .in2 = GPIO_DT_SPEC_GET(DT_ALIAS(motor_rf_in2), gpios),
        .pwm = PWM_DT_SPEC_GET(DT_ALIAS(motor_rf_pwm)),
        .invert_direction = MOTOR_RF_INVERT_DIRECTION,
        .current_pct = 0,
    },

    [MOTOR_ID_RIGHT_REAR] = {
        .name = "right_rear",
        .in1 = GPIO_DT_SPEC_GET(DT_ALIAS(motor_rr_in1), gpios),
        .in2 = GPIO_DT_SPEC_GET(DT_ALIAS(motor_rr_in2), gpios),
        .pwm = PWM_DT_SPEC_GET(DT_ALIAS(motor_rr_pwm)),
        .invert_direction = MOTOR_RR_INVERT_DIRECTION,
        .current_pct = 0,
    },
};

/**
 * @brief Clamp a signed motor percentage to the valid range.
 *
 * Ensures that motor output never exceeds the supported driver range.
 *
 * @param pct Requested signed motor percentage.
 *            Expected logical range is -100 to +100.
 *
 * @return Clamped motor percentage in range -100 to +100.
 */
static int16_t clamp_motor_pct(int16_t pct)
{
    if (pct > 100) {
        return 100;
    }

    if (pct < -100) {
        return -100;
    }

    return pct;
}

/**
 * @brief Check whether a motor command changes directly between directions.
 *
 * Detects transitions such as:
 *
 * - forward  → backward
 * - backward → forward
 *
 * This is used to insert a short deadtime before reversing motor direction.
 * The deadtime helps protect the H-bridge and reduces mechanical shock.
 *
 * @param old_pct Previous signed motor percentage.
 *                Positive = forward, negative = backward, zero = stopped.
 *
 * @param new_pct New requested signed motor percentage.
 *                Positive = forward, negative = backward, zero = stopped.
 *
 * @return true if the command changes directly between forward and reverse.
 * @return false otherwise.
 */
static bool different_nonzero_direction(int16_t old_pct, int16_t new_pct)
{
    return ((old_pct > 0) && (new_pct < 0)) ||
           ((old_pct < 0) && (new_pct > 0));
}

/**
 * @brief Set PWM duty cycle for one motor channel.
 *
 * Converts a duty-cycle percentage into a PWM pulse width using the PWM period
 * defined in devicetree.
 *
 * Example:
 *
 * - duty_pct = 0   → 0% duty
 * - duty_pct = 50  → 50% duty
 * - duty_pct = 100 → 100% duty
 *
 * @param motor Pointer to the motor channel configuration.
 *              Must not be NULL.
 *
 * @param duty_pct PWM duty cycle percentage.
 *                 Valid range: 0 to 100.
 *                 Values above 100 are clamped to 100.
 *
 * @return 0 on success.
 * @return Negative errno-style value if pwm_set_dt() fails.
 */
static int motor_set_pwm_percent(const motor_channel_t *motor,
                                 uint8_t duty_pct)
{
    uint64_t pulse_ns;

    if (motor == NULL) {
        return -EINVAL;
    }

    if (duty_pct > 100u) {
        duty_pct = 100u;
    }

    pulse_ns = ((uint64_t)motor->pwm.period * duty_pct) / 100u;

    return pwm_set_dt(&motor->pwm,
                      motor->pwm.period,
                      (uint32_t)pulse_ns);
}

/**
 * @brief Set L298N direction pins for one motor channel.
 *
 * Controls the L298N IN1/IN2 pins according to the signed motor command.
 *
 * Logic:
 *
 * - signed_pct > 0:
 *     forward direction
 *
 * - signed_pct < 0:
 *     backward direction
 *
 * - signed_pct == 0:
 *     coast stop, both direction pins low
 *
 * If the motor channel has invert_direction enabled, forward and backward are
 * swapped. This is useful when a motor is physically mounted or wired in the
 * opposite direction.
 *
 * @param motor Pointer to the motor channel configuration.
 *              Must not be NULL.
 *
 * @param signed_pct Signed motor percentage.
 *                   Positive = forward.
 *                   Negative = backward.
 *                   Zero = stop/coast.
 *
 * @return 0 on success.
 * @return Negative errno-style value if gpio_pin_set_dt() fails.
 */
static int motor_set_direction_pins(const motor_channel_t *motor,
                                    int16_t signed_pct)
{
    int ret;
    bool forward;

    if (motor == NULL) {
        return -EINVAL;
    }

    if (signed_pct == 0) {
        ret = gpio_pin_set_dt(&motor->in1, 0);
        if (ret != 0) {
            return ret;
        }

        ret = gpio_pin_set_dt(&motor->in2, 0);
        if (ret != 0) {
            return ret;
        }

        return 0;
    }

    forward = signed_pct > 0;

    if (motor->invert_direction) {
        forward = !forward;
    }

    if (forward) {
        ret = gpio_pin_set_dt(&motor->in1, 1);
        if (ret != 0) {
            return ret;
        }

        ret = gpio_pin_set_dt(&motor->in2, 0);
        if (ret != 0) {
            return ret;
        }
    } else {
        ret = gpio_pin_set_dt(&motor->in1, 0);
        if (ret != 0) {
            return ret;
        }

        ret = gpio_pin_set_dt(&motor->in2, 1);
        if (ret != 0) {
            return ret;
        }
    }

    return 0;
}

/**
 * @brief Stop one motor channel.
 *
 * Stops a single motor using coast-stop behavior:
 *
 * - PWM duty = 0%
 * - IN1 = 0
 * - IN2 = 0
 *
 * Also updates the motor's internal current_pct state to 0.
 *
 * @param motor Pointer to the motor channel to stop.
 *              Must not be NULL.
 *
 * @return 0 on success.
 * @return Negative errno-style value if GPIO or PWM operation fails.
 */
static int motor_stop_one(motor_channel_t *motor)
{
    int ret;

    if (motor == NULL) {
        return -EINVAL;
    }

    ret = motor_set_pwm_percent(motor, 0u);
    if (ret != 0) {
        return ret;
    }

    ret = motor_set_direction_pins(motor, 0);
    if (ret != 0) {
        return ret;
    }

    motor->current_pct = 0;

    return 0;
}

/**
 * @brief Apply a signed output command to one motor channel.
 *
 * This function applies direction and PWM duty to a single motor.
 *
 * Input meaning:
 *
 * - +100 = forward at 100%
 * - +50  = forward at 50%
 * - 0    = stop
 * - -50  = backward at 50%
 * - -100 = backward at 100%
 *
 * Safety behavior:
 *
 * - If the command reverses direction, PWM is first set to 0%.
 * - A short deadtime is inserted before the new direction is applied.
 * - Direction pins are updated before PWM duty is enabled.
 *
 * This function does not perform acceleration ramping. It expects the caller
 * to provide already-ramped motor percentage values.
 *
 * @param motor Pointer to the motor channel to control.
 *              Must not be NULL.
 *
 * @param pct Signed motor output percentage.
 *            Valid range: -100 to +100.
 *            Values outside this range are clamped.
 *
 * @return 0 on success.
 * @return Negative errno-style value if GPIO or PWM operation fails.
 */
static int motor_set_one(motor_channel_t *motor,
                         int16_t pct)
{
    int ret;
    uint8_t duty_pct;

    if (motor == NULL) {
        return -EINVAL;
    }

    pct = clamp_motor_pct(pct);

    /*
     * If changing directly from forward to reverse or reverse to forward,
     * first remove PWM and let the H-bridge settle briefly.
     */
    if (different_nonzero_direction(motor->current_pct, pct)) {
        ret = motor_set_pwm_percent(motor, 0u);
        if (ret != 0) {
            return ret;
        }

        k_msleep(MOTOR_DIRECTION_CHANGE_DEADTIME_MS);
    }

    if (pct == 0) {
        return motor_stop_one(motor);
    }

    duty_pct = (uint8_t)abs(pct);

    /*
     * Safer sequence:
     *   1. PWM = 0
     *   2. Set direction pins
     *   3. Apply PWM duty
     */
    ret = motor_set_pwm_percent(motor, 0u);
    if (ret != 0) {
        return ret;
    }

    ret = motor_set_direction_pins(motor, pct);
    if (ret != 0) {
        return ret;
    }

    ret = motor_set_pwm_percent(motor, duty_pct);
    if (ret != 0) {
        return ret;
    }

    motor->current_pct = pct;

    return 0;
}

/**
 * @brief Check whether GPIO and PWM devices for one motor are ready.
 *
 * Verifies that:
 *
 * - IN1 GPIO device is ready.
 * - IN2 GPIO device is ready.
 * - PWM device is ready.
 *
 * This should be called during initialization before configuring pins or
 * applying motor commands.
 *
 * @param motor Pointer to the motor channel configuration.
 *              Must not be NULL.
 *
 * @return 0 if all devices are ready.
 * @return -ENODEV if any required GPIO/PWM device is not ready.
 */
static int motor_check_ready(const motor_channel_t *motor)
{
    if (motor == NULL) {
        return -EINVAL;
    }

    if (!device_is_ready(motor->in1.port)) {
        printk("Motor %s IN1 GPIO device not ready\n", motor->name);
        return -ENODEV;
    }

    if (!device_is_ready(motor->in2.port)) {
        printk("Motor %s IN2 GPIO device not ready\n", motor->name);
        return -ENODEV;
    }

    if (!device_is_ready(motor->pwm.dev)) {
        printk("Motor %s PWM device not ready\n", motor->name);
        return -ENODEV;
    }

    return 0;
}

/**
 * @brief Configure direction GPIO pins for one motor.
 *
 * Configures the motor's IN1 and IN2 pins as output pins and initializes them
 * to inactive state.
 *
 * Initial state:
 *
 * - IN1 = 0
 * - IN2 = 0
 *
 * This prevents accidental motor movement during boot.
 *
 * @param motor Pointer to the motor channel configuration.
 *              Must not be NULL.
 *
 * @return 0 on success.
 * @return Negative errno-style value if GPIO configuration fails.
 */
static int motor_configure_gpio(const motor_channel_t *motor)
{
    int ret;

    if (motor == NULL) {
        return -EINVAL;
    }

    ret = gpio_pin_configure_dt(&motor->in1, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Failed to configure %s IN1, ret=%d\n", motor->name, ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&motor->in2, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Failed to configure %s IN2, ret=%d\n", motor->name, ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Initialize all motor driver outputs.
 *
 * Checks all GPIO and PWM devices, configures direction GPIO pins, and stops
 * all motors.
 *
 * This function must be called once before using:
 *
 * - motor_driver_set_left_right()
 * - motor_driver_stop_all()
 *
 * @return 0 on success.
 * @return Negative errno-style value on initialization failure.
 */
int motor_driver_init(void)
{
    int ret;

    printk("motor_driver_init: start\n");

    for (size_t i = 0; i < MOTOR_ID_COUNT; i++) {
        printk("motor_driver_init: checking %s\n", g_motors[i].name);

        ret = motor_check_ready(&g_motors[i]);
        printk("motor_driver_init: motor_check_ready(%s) = %d\n",
               g_motors[i].name, ret);
        if (ret != 0) {
            return ret;
        }

        ret = motor_configure_gpio(&g_motors[i]);
        printk("motor_driver_init: motor_configure_gpio(%s) = %d\n",
               g_motors[i].name, ret);
        if (ret != 0) {
            return ret;
        }

        ret = motor_stop_one(&g_motors[i]);
        printk("motor_driver_init: motor_stop_one(%s) = %d\n",
               g_motors[i].name, ret);
        if (ret != 0) {
            return ret;
        }
    }

    printk("motor_driver_init: done\n");

    return 0;
}

/**
 * @brief Set left and right motor group outputs.
 *
 * Applies the same signed output percentage to both motors on each side:
 *
 * - left_pct  → left-front and left-rear motors
 * - right_pct → right-front and right-rear motors
 *
 * This function is intended to receive output from the vehicle control manager
 * after mixing, speed limiting, timeout handling, and ramp limiting have
 * already been applied.
 *
 * @param left_pct Signed output for left motor group.
 *                 Range: -100 to +100.
 *
 * @param right_pct Signed output for right motor group.
 *                  Range: -100 to +100.
 *
 * @return 0 on success.
 * @return Negative errno-style value if any motor update fails.
 */
int motor_driver_set_left_right(int16_t left_pct,
                                int16_t right_pct)
{
    int ret;

    left_pct = clamp_motor_pct(left_pct);
    right_pct = clamp_motor_pct(right_pct);

    ret = motor_set_one(&g_motors[MOTOR_ID_LEFT_FRONT], left_pct);
    if (ret != 0) {
        return ret;
    }

    ret = motor_set_one(&g_motors[MOTOR_ID_LEFT_REAR], left_pct);
    if (ret != 0) {
        return ret;
    }

    ret = motor_set_one(&g_motors[MOTOR_ID_RIGHT_FRONT], right_pct);
    if (ret != 0) {
        return ret;
    }

    ret = motor_set_one(&g_motors[MOTOR_ID_RIGHT_REAR], right_pct);
    if (ret != 0) {
        return ret;
    }

    return 0;
}

/**
 * @brief Stop all four motors immediately.
 *
 * Applies coast-stop to:
 *
 * - left-front motor
 * - left-rear motor
 * - right-front motor
 * - right-rear motor
 *
 * Coast-stop means:
 *
 * - PWM duty = 0%
 * - direction pins = inactive
 *
 * @return 0 on success.
 * @return First negative errno-style value if any motor fails to stop.
 */
int motor_driver_stop_all(void)
{
    int ret;
    int first_error = 0;

    for (size_t i = 0u; i < MOTOR_ID_COUNT; i++) {
        ret = motor_stop_one(&g_motors[i]);
        if ((ret != 0) && (first_error == 0)) {
            first_error = ret;
        }
    }

    return first_error;
}