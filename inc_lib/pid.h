# ifndef __PID_H
# define __PID_H

/**
 * @file pid.h
 * 
 * @brief PID with all the needed extra features
 */


 /**
  * @brief Settings for the PID
  */
struct PidSettings
{
	// the coefficients
	float kp, ki, kd;

	// feedforward coefficient (for PID_Run_V2)
	float kff;

	// the limit of the contribution of each coefficient
	float p_lim, i_lim, k_lim;

	// the absolute maximum value of the PID's output
	float max_output;
	// the absolute minimum value of the PID's output
	float min_output;

	// The ratio used to filter the derivative (the higher the more smoothing)
	float fratio;

	// the decay of the integral (gradual reduction per iteration to prevent windup)
	float decay;

};


/**
 * @brief All the variables representing the current state of a PID
 */
struct PidRuntime
{
	float p;

	/** The total accumulated integral */
	float i;

	/** The current filtered derivative */
	float d;

	/** The output of the PID the last time it was ran */
	float last_err;

	/** The last measurement value (for derivative-on-measurement in PID_Run_V2) */
	float last_measurement;
};


/**
 * @brief Resets all of the runtime values to default.
 */
void PID_reset_runtime(struct PidRuntime *runtime);


/**
 * @brief Manually sets the last value
 */
void PID_set_runtime_last_value(struct PidRuntime *runtime, float last_val);


/**
 * @brief runs the PID.
 *
 * @note The runtine struct will be updated.
 *
 * @param delta The delta time since the last PID run.
 */
float PID_Run(struct PidRuntime *runtime, const struct PidSettings *settings, float current_val, float target_val, float delta);


/**
 * @brief runs the PID with derivative-on-measurement and feedforward.
 *
 * This version improves upon PID_Run with:
 * - Derivative calculated on measurement (not error) to avoid derivative kick on setpoint changes
 * - Feedforward term (kff * target) for better tracking performance
 *
 * @note The runtime struct will be updated.
 *
 * @param runtime Pointer to the PID runtime state
 * @param settings Pointer to the PID settings (must include kff)
 * @param current_val Current measured value
 * @param target_val Desired target value
 * @param delta Delta time since the last PID run (in sec)
 * @return float PID output command
 */
float PID_Run_V2(struct PidRuntime *runtime, const struct PidSettings *settings, float current_val, float target_val, float delta);

float PID_Run_V2_debug(struct PidRuntime *runtime, const struct PidSettings *settings, float current_val, float target_val, float delta);

# endif // __PID_H