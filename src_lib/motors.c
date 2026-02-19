# include "motors.h"

# include <math.h>


void motor_drive(struct MotorHandle handle, float speed)
{
	if ((speed < 0) ^ (handle.reverse)) HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(handle.dir_gpio_port, handle.dir_gpio_pin, GPIO_PIN_RESET);
	
	if (speed < -MOTOR_LIMIT) speed = -MOTOR_LIMIT;
	else if (speed > MOTOR_LIMIT) speed = MOTOR_LIMIT;
	*handle.pwm_ccr = (uint32_t)((handle.pwm_arr * (fabs(speed))) / 100.0f);
}