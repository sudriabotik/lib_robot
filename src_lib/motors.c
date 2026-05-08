# include "motors.h"

# include <math.h>
# include "robot_data.h"


void motor_drive(struct MotorHandle handle, float speed)
{
	if ((speed < 0) ^ (handle.reverse)) HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_RESET);
	
	if (speed < -handle.pwm_limit) speed = -handle.pwm_limit;
	else if (speed > handle.pwm_limit) speed = handle.pwm_limit;
	*handle.pwm_ccr = (uint32_t)((handle.pwm_arr * (fabs(speed))) / 100.0f);
}

void motor_drive_debug(struct MotorHandle handle, float speed)
{
	//printf(" %i\r\n", (speed < 0) ^ (handle.reverse));
	if ((speed < 0) ^ (handle.reverse)) HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_RESET);

	if (speed < -handle.pwm_limit) speed = -handle.pwm_limit;
	else if (speed > handle.pwm_limit) speed = handle.pwm_limit;
	uint32_t duty_cycle = (uint32_t)((handle.pwm_arr * (fabs(speed))) / 100.0f);
	*handle.pwm_ccr = duty_cycle;
	debug_printf("speed=%.2f%% duty=%lu dir=%d\r\n", speed, duty_cycle, (speed < 0) ^ (handle.reverse)); // claude
}