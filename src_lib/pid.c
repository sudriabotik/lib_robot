# include "pid.h"
# include "robot_data.h"


void PID_reset_runtime(struct PidRuntime *runtime)
{
	runtime->p = 0;
	runtime->d = 0;
	runtime->i = 0;

	runtime->last_err = 0;
}


void PID_set_runtime_last_value(struct PidRuntime *runtime, float last_val)
{
	runtime->last_err = last_val;
}




float PID_Run(struct PidRuntime *runtime, const struct PidSettings *settings, float current_val, float target_val, float delta)
{
	float out;

	runtime->p = target_val - current_val;

	/** filtered derivative */
	runtime->d = (runtime->last_err - runtime->p) * delta * (1 - settings->fratio)
				+ runtime->d * settings->fratio;

	runtime->i = runtime->i + runtime->p * delta;

	// decay
	if (runtime->i > settings->decay * delta) runtime->i -= settings->decay * delta;
	else if (runtime->i > 0) runtime->i = 0;
	else if (runtime->i < -settings->decay * delta) runtime->i += settings->decay * delta;
	else if (runtime->i < 0) runtime->i = 0;

	// clamp the integral
	if (runtime->i < -settings->i_lim) runtime->i = -settings->i_lim;
	if (runtime->i > settings->i_lim) runtime->i = settings->i_lim;

	// calculates the output
	out = runtime->p * settings->kp + runtime->i * settings->ki + runtime->d * settings->kd;

	debug_printf("P_contribution =%.3f ", runtime->p * settings->kp); 
	debug_printf("I_contribution =%.3f ", runtime->i * settings->ki); 
	debug_printf("D_contribution =%.3f ", runtime->d * settings->kd); 
	debug_printf("out_before_clamp=%.3f ", out); 

	// clamp the final output
	if (out > settings->max_output) out = settings->max_output;
	else if (out < -settings->max_output) out = -settings->max_output;

	if (out >= 0 && out < settings->min_output) out = settings->min_output;
	else if (out <= 0 && out > -settings->min_output) out = -settings->min_output;

	runtime->last_err = runtime->p;

	debug_printf("pid_out=%.3f ", out); // claude

	return out;
}


/* PID_Run_V2: Professional PID with Derivative-on-Measurement and Feedforward
 *
 * Improvements over PID_Run:
 * 1. Derivative-on-Measurement: Calculates derivative from measurement changes,
 *    not error changes. This eliminates "derivative kick" when setpoint changes.
 * 2. Feedforward: Adds proportional term based on target value for better
 *    tracking performance and faster response.
 *
 * Mathematical formulation:
 *   error = target - measurement
 *   P = error * Kp
 *   I = integral(error) * Ki
 *   D = -(measurement - measurement_old) / dt * Kd  (note the negative sign!)
 *   FF = target * Kff
 *   output = P + I + D + FF
 */
float PID_Run_V2(struct PidRuntime *runtime, const struct PidSettings *settings, float current_val, float target_val, float delta)
{
	float out;

	// Calculate error (proportional term)
	runtime->p = target_val - current_val;

	// Derivative-on-Measurement: d/dt of measurement (not error!)
	// Negative sign because we want to oppose changes in measurement
	float measurement_derivative = -(current_val - runtime->last_measurement) / delta;
	runtime->d = measurement_derivative * (1 - settings->fratio) + runtime->d * settings->fratio;

	// Integral accumulation
	runtime->i = runtime->i + runtime->p * delta;

	// Integral decay (anti-windup)
	if (runtime->i > settings->decay * delta) runtime->i -= settings->decay * delta;
	else if (runtime->i > 0) runtime->i = 0;
	else if (runtime->i < -settings->decay * delta) runtime->i += settings->decay * delta;
	else if (runtime->i < 0) runtime->i = 0;

	// Clamp the integral
	if (runtime->i < -settings->i_lim) runtime->i = -settings->i_lim;
	if (runtime->i > settings->i_lim) runtime->i = settings->i_lim;

	// Calculate output with feedforward
	out = runtime->p * settings->kp
	    + runtime->i * settings->ki
	    + runtime->d * settings->kd
	    + target_val * settings->kff;  // Feedforward term

	// Clamp the final output
	if (out > settings->max_output) out = settings->max_output;
	else if (out < -settings->max_output) out = -settings->max_output;

	if (out >= 0 && out < settings->min_output) out = settings->min_output;
	else if (out <= 0 && out > -settings->min_output) out = -settings->min_output;

	// Store for next iteration
	runtime->last_err = runtime->p;
	runtime->last_measurement = current_val;

	debug_printf("pid_out=%.3f ", out); // claude

	return out;
}
