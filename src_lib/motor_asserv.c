# include "motor_asserv.h"

# include "stm32g431xx.h"
# include <inttypes.h>
# include "stdio.h"
# include "main.h"
# include "robot_data.h"

/* OLD
void motor_drive_pid(float delta_ms, float rps, const struct MotorHandle motor, const struct Encoder16Handle encoder, const struct PidSettings settings, struct PidRuntime *runtime)
{
	float target_ticks_per_ms = (rps * encoder.ticks_per_revolution) / 1000; // how many ticks to pass in 1 ms
	float current_ticks_per_ms = (float)encoder.total_count_delta / delta_ms;
	debug_printf("target_ticks=%.3f current_ticks=%.3f ", target_ticks_per_ms, current_ticks_per_ms); // claude
	float drive = PID_Run(runtime, &settings, current_ticks_per_ms, target_ticks_per_ms, delta_ms);
	//printf("target ticks : %f\n", current_ticks_per_ms);
	motor_set_pwm_percent_debug(motor, drive);
}
*/ 

void motor_drive_pid_rps(float delta_ms, float target_rps, float current_rps, const struct MotorHandle motor, const struct PidSettings settings, struct PidRuntime *runtime)
{
	//debug_printf("target_rps=%.3f current_rps=%.3f ", target_rps, current_rps);
	float pwm_percent = PID_Run(runtime, &settings, current_rps, target_rps, delta_ms);
	motor_set_pwm_percent_debug(motor, pwm_percent);
}

motor_drive_pid_ticks(float delta_ms, int32_t target_ticks, int32_t current_ticks, 
                            const struct MotorHandle motor, const struct PidSettings settings, 
                            struct PidRuntime *runtime)
{
	debug_printf("target_ticks=%ld current_ticks=%ld \n ", target_ticks, current_ticks);
	
	// Debug encoder hardware and software state
	if (motor.pwm_ccr == motor_R.pwm_ccr) {
		debug_printf("TIM3_CNT=%lu rollover=%ld situation=%lu CR1=0x%lx SMCR=0x%lx CCER=0x%lx ", 
			(unsigned long)TIM3->CNT, 
			encoder_R.rollover_count, 
			encoder_R.situation,
			(unsigned long)TIM3->CR1,
			(unsigned long)TIM3->SMCR,  // Slave Mode Control Register (mode encodeur)
			(unsigned long)TIM3->CCER); // Capture/Compare Enable Register
	} else if (motor.pwm_ccr == motor_L.pwm_ccr) {
		debug_printf("TIM4_CNT=%lu rollover=%ld situation=%lu CR1=0x%lx SMCR=0x%lx CCER=0x%lx ", 
			(unsigned long)TIM4->CNT, 
			encoder_L.rollover_count, 
			encoder_L.situation,
			(unsigned long)TIM4->CR1,
			(unsigned long)TIM4->SMCR,  // Slave Mode Control Register (mode encodeur)
			(unsigned long)TIM4->CCER); // Capture/Compare Enable Register
	}

	// PID works directly with ticks as floats
	float pwm_percent = PID_Run(runtime, &settings, (float)current_ticks, (float)target_ticks, delta_ms);
	
	motor_set_pwm_percent_debug(motor, pwm_percent);
}