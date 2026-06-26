#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the motor driver hardware.
 *
 * Configures all GPIO direction pins and PWM output channels used for the
 * four DC motors connected through two L298N motor drivers.
 *
 * This function also forces all motors into a safe stopped state during
 * initialization:
 *
 * - PWM duty = 0%
 * - IN1 = 0
 * - IN2 = 0
 *
 * @return 0 on success.
 * @return Negative errno-style value on failure.
 *
 * Typical failure causes:
 * - GPIO device not ready.
 * - PWM device not ready.
 * - GPIO pin configuration failed.
 * - PWM setup failed.
 */
int motor_driver_init(void);

/**
 * @brief Set left-side and right-side motor output.
 *
 * This is the main API used by the vehicle control manager.
 *
 * The function applies the same signed output value to both motors on each
 * side of the 4WD vehicle:
 *
 * - left_pct controls left-front and left-rear motors.
 * - right_pct controls right-front and right-rear motors.
 *
 * The input values are signed percentages:
 *
 * - +100 = forward at 100%
 * - +50  = forward at 50%
 * - 0    = stop
 * - -50  = backward at 50%
 * - -100 = backward at 100%
 *
 * This function does not perform acceleration ramping. Ramping should already
 * be handled by the vehicle control manager. This driver only applies the
 * final motor command to GPIO and PWM hardware.
 *
 * @param left_pct Signed output percentage for the left motor group.
 *                 Valid range: -100 to +100.
 *                 Values outside this range are clamped internally.
 *
 * @param right_pct Signed output percentage for the right motor group.
 *                  Valid range: -100 to +100.
 *                  Values outside this range are clamped internally.
 *
 * @return 0 on success.
 * @return Negative errno-style value on GPIO/PWM failure.
 */
int motor_driver_set_left_right(int16_t left_pct, int16_t right_pct);

/**
 * @brief Immediately stop all motors.
 *
 * Stops all four motors using coast-stop behavior:
 *
 * - PWM duty = 0%
 * - IN1 = 0
 * - IN2 = 0
 *
 * This does not actively brake the motors. It simply disables motor drive.
 *
 * Use this for:
 * - System initialization.
 * - Normal stop.
 * - Timeout stop.
 * - Emergency stop, if you want motor power removed immediately.
 *
 * @return 0 on success.
 * @return First negative errno-style error if one or more motors fail to stop.
 */
int motor_driver_stop_all(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */