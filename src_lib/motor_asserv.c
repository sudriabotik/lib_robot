# include "motor_asserv.h"

# include "stm32g431xx.h"
# include <inttypes.h>
# include "stdio.h"
# include "main.h"
# include "robot_data.h"

void motor_drive_pid(float delta_ms, float rps, const struct MotorHandle motor, const struct Encoder16Handle encoder, const struct PidSettings settings, struct PidRuntime *runtime)
{
	float target_ticks_per_ms = (rps * encoder.ticks_per_revolution) / 1000; // how many ticks to pass in 1 ms
	float current_ticks_per_ms = (float)encoder.total_count_delta / delta_ms;
	debug_printf("target_ticks=%.3f current_ticks=%.3f ", target_ticks_per_ms, current_ticks_per_ms); // claude
	float drive = PID_Run(runtime, &settings, current_ticks_per_ms, target_ticks_per_ms, delta_ms);
	//printf("target ticks : %f\n", current_ticks_per_ms);
	motor_drive_debug(motor, drive);
}