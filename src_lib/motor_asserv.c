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

void motor_drive_pid_rpm(float delta_s, float target_rpm, float current_rpm, const struct MotorHandle motor, const struct PidSettings settings, struct PidRuntime *runtime)
{
	//debug_printf("target_rps=%.3f current_rps=%.3f ", target_rps, current_rps);
	float pwm_percent = PID_Run(runtime, &settings, current_rpm, target_rpm, delta_s);
	motor_set_pwm_percent_debug(motor, pwm_percent);
}

void motor_drive_pid_ticks(float delta_ms, int32_t target_ticks, int32_t current_ticks, 
                            const struct MotorHandle motor, const struct PidSettings settings, 
                            struct PidRuntime *runtime)
{
	debug_printf("target_ticks=%ld current_ticks=%ld \n ", target_ticks, current_ticks);
	
	// Debug encoder hardware and software state
	if (motor.pwm_ccr == motor_R.pwm_ccr) {
		uint8_t gpio_ch1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		uint8_t gpio_ch2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
		
		uint32_t sr_value = TIM3->SR;
		
		debug_printf("TIM3_CNT=%lu ", (unsigned long)TIM3->CNT);
		debug_printf("SR=0x%lx ", (unsigned long)sr_value);

		// ===== NOUVEAU: DÉTECTION DES ERREURS D'ENCODEUR =====
		if (sr_value & TIM_SR_TERRF) {
			debug_printf("ENCODER_TRANSITION_ERROR!_RESETTING! ");
			// Sauvegarde
			uint32_t cnt_backup = TIM3->CNT;
			int32_t rollover_backup = encoder_R.rollover_count;
			
			// Hard reset du timer
			TIM3->CR1 &= ~TIM_CR1_CEN;  // Stop
			TIM3->SR = 0;                // Clear ALL flags
			TIM3->CNT = cnt_backup;      // Restore count
			TIM3->CR1 |= TIM_CR1_CEN;    // Restart
			
		}

		if (sr_value & TIM_SR_IERRF) {
			debug_printf("ENCODER_INDEX_ERROR! ");
			TIM3->SR &= ~TIM_SR_IERRF;  // Clear index error
		}

		// ===== DIAGNOSTIC SYSTÈME =====
		// RCC - Clock control
		uint32_t rcc_apb1enr = RCC->APB1ENR1;
		uint32_t rcc_apb1rstr = RCC->APB1RSTR1;
		debug_printf("RCC_TIM3_CLK=%lu ", (rcc_apb1enr & RCC_APB1ENR1_TIM3EN) ? 1 : 0);
		debug_printf("RCC_TIM3_RST=%lu ", (rcc_apb1rstr & RCC_APB1RSTR1_TIM3RST) ? 1 : 0);

		// Timer config registers
		debug_printf("TIM3_ARR=%lu ", (unsigned long)TIM3->ARR);
		debug_printf("TIM3_PSC=%lu ", (unsigned long)TIM3->PSC);
		debug_printf("TIM3_DIER=0x%lx ", (unsigned long)TIM3->DIER);

		// GPIO configuration
		uint32_t gpiob_moder = GPIOB->MODER;
		uint32_t gpiob_afr0 = GPIOB->AFR[0];
		uint32_t pb4_mode = (gpiob_moder >> 8) & 0x3;
		uint32_t pb4_af = (gpiob_afr0 >> 16) & 0xF;
		uint32_t pb5_mode = (gpiob_moder >> 10) & 0x3;
		uint32_t pb5_af = (gpiob_afr0 >> 20) & 0xF;
		debug_printf("PB4_MODE=%lu PB4_AF=%lu ", pb4_mode, pb4_af);
		debug_printf("PB5_MODE=%lu PB5_AF=%lu ", pb5_mode, pb5_af);

		// System fault registers
		debug_printf("SCB_CFSR=0x%lx ", SCB->CFSR);

	} else if (motor.pwm_ccr == motor_L.pwm_ccr) {
	// ===== // ===== LECTURE DIRECTE DES GPIO ENCODEUR TIM4 =====
		uint8_t gpio_ch1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
		uint8_t gpio_ch2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
		
		uint32_t sr_value = TIM4->SR;
		
		debug_printf("TIM4_CNT=%lu ", (unsigned long)TIM4->CNT);
		debug_printf("SR=0x%lx ", (unsigned long)sr_value);


		// ===== NOUVEAU: DÉTECTION DES ERREURS D'ENCODEUR =====
		if (sr_value & TIM_SR_TERRF) {
			debug_printf("ENCODER_TRANSITION_ERROR!_RESETTING! ");
			// Sauvegarde
			uint32_t cnt_backup = TIM4->CNT;
			int32_t rollover_backup = encoder_R.rollover_count;
			
			// Hard reset du timer
			TIM4->CR1 &= ~TIM_CR1_CEN;  // Stop
			TIM4->SR = 0;                // Clear ALL flags
			TIM4->CNT = cnt_backup;      // Restore count
			TIM4->CR1 |= TIM_CR1_CEN;    // Restart
			
		}
		if (sr_value & TIM_SR_DIRF) {
			debug_printf("ENCODER_DIR_CHANGE! ");
			TIM4->SR &= ~TIM_SR_DIRF;   // Clear direction change flag
		}
		if (sr_value & TIM_SR_IERRF) {
			debug_printf("ENCODER_INDEX_ERROR! ");
			TIM4->SR &= ~TIM_SR_IERRF;  // Clear index error
		}

				// ===== DIAGNOSTIC SYSTÈME TIM4 =====
		// AJOUTE CES LIGNES AVANT ligne 124:
		uint32_t rcc_apb1enr = RCC->APB1ENR1;
		uint32_t rcc_apb1rstr = RCC->APB1RSTR1;
		uint32_t gpiob_moder = GPIOB->MODER;

		debug_printf("RCC_TIM4_CLK=%lu ", (rcc_apb1enr & RCC_APB1ENR1_TIM4EN) ? 1 : 0);
		debug_printf("RCC_TIM4_RST=%lu ", (rcc_apb1rstr & RCC_APB1RSTR1_TIM4RST) ? 1 : 0);
		debug_printf("TIM4_ARR=%lu ", (unsigned long)TIM4->ARR);
		debug_printf("TIM4_PSC=%lu ", (unsigned long)TIM4->PSC);
		debug_printf("TIM4_DIER=0x%lx ", (unsigned long)TIM4->DIER);

		uint32_t gpiob_afr0_tim4 = GPIOB->AFR[0];
		uint32_t pb6_mode = (gpiob_moder >> 12) & 0x3;
		uint32_t pb6_af = (gpiob_afr0_tim4 >> 24) & 0xF;
		uint32_t pb7_mode = (gpiob_moder >> 14) & 0x3;
		uint32_t pb7_af = (gpiob_afr0_tim4 >> 28) & 0xF;
		debug_printf("PB6_MODE=%lu PB6_AF=%lu ", pb6_mode, pb6_af);
		debug_printf("PB7_MODE=%lu PB7_AF=%lu ", pb7_mode, pb7_af);
	}

	// PID works directly with ticks as floats
	float pwm_percent = PID_Run(runtime, &settings, (float)current_ticks, (float)target_ticks, delta_ms);
	
	motor_set_pwm_percent_debug(motor, pwm_percent);
}
