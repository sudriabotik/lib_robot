# include "motors.h"

# include <math.h>
# include "robot_data.h"


void motor_set_pwm_percent(struct MotorHandle handle, float pwm_percent)
{
	if ((pwm_percent < 0) ^ (handle.reverse)) HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_RESET);
	
	if (pwm_percent < -handle.pwm_limit) pwm_percent = -handle.pwm_limit;
	else if (pwm_percent > handle.pwm_limit) pwm_percent = handle.pwm_limit;
	*handle.pwm_ccr = (uint32_t)((handle.pwm_arr * (fabs(pwm_percent))) / 100.0f);
}

void motor_set_pwm_percent_debug(struct MotorHandle handle, float pwm_percent)
{
	//printf(" %i\r\n", (pwm_percent < 0) ^ (handle.reverse));
	if ((pwm_percent < 0) ^ (handle.reverse)) HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_RESET);

	if (pwm_percent < -handle.pwm_limit) pwm_percent = -handle.pwm_limit;
	else if (pwm_percent > handle.pwm_limit) pwm_percent = handle.pwm_limit;
	uint32_t duty_cycle = (uint32_t)((handle.pwm_arr * (fabs(pwm_percent))) / 100.0f);
	*handle.pwm_ccr = duty_cycle;
	debug_printf("pwm_percent=%.2f%% duty=%lu dir=%d\n", pwm_percent, duty_cycle, (pwm_percent < 0) ^ (handle.reverse)); // claude
}