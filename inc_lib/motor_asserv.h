# ifndef __MOTOR_ASSERV_H
# define __MOTOR_ASSERV_H

# include "pid.h"
# include "motors.h"
# include "encoders.h"


/**
 * @brief Drive a motor using PID control based on velocity in RPS
 * @param delta_ms Time since last update (milliseconds)
 * @param target_rps Target velocity (revolutions per second)
 * @param current_rps Measured velocity (revolutions per second)
 * @param motor Motor handle
 * @param settings PID configuration
 * @param runtime PID runtime state
 */
void motor_drive_pid_rps(float delta_ms, float target_rps, float current_rps, const struct MotorHandle motor, const struct PidSettings settings, struct PidRuntime *runtime);


/**
 * @brief Drive a motor using PID control based on position in encoder ticks
 * @param delta_ms Time since last update (milliseconds)
 * @param target_ticks Target position (encoder ticks)
 * @param current_ticks Current position (encoder ticks)
 * @param motor Motor handle
 * @param settings PID configuration
 * @param runtime PID runtime state
 */
void motor_drive_pid_ticks(float delta_ms, int32_t target_ticks, int32_t current_ticks, 
                            const struct MotorHandle motor, const struct PidSettings settings, 
                            struct PidRuntime *runtime);

# endif