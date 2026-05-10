# include "encoders.h"

# include <inttypes.h>
# include <stdio.h>


uint32_t GetEncoderSituation(struct Encoder16Handle *handle)
{
	if (*handle->tim_counter < handle->close_distance) return ENCODER_SITUATION_UNDERFLOW_CLOSE;
	if (*handle->tim_counter > 0xFFFF - handle->close_distance) return ENCODER_SITUATION_OVERFLOW_CLOSE;
	return ENCODER_SITUATION_NOTHING;
}

int Encoder16Reset(struct Encoder16Handle *handle)
{
	if (handle == 0) return -1;

	handle->_rollover_count = 0; // set the rollover count to 0
	handle->_total_count = 0;
	handle->_total_count_delta = 0;
	*handle->tim_counter = 0; // set the timer counter to 0
	handle->_situation = GetEncoderSituation(handle);
	
	Encoder16Update(handle); // update the calculated values

	return 0;
}

int Encoder16Update(struct Encoder16Handle *handle)
{
	if (handle == 0) return -1;

	uint32_t current_situation = GetEncoderSituation(handle);

	if (handle->_situation == ENCODER_SITUATION_OVERFLOW_CLOSE && current_situation == ENCODER_SITUATION_UNDERFLOW_CLOSE)
	{
		handle->_rollover_count ++;
	}
	if (handle->_situation == ENCODER_SITUATION_UNDERFLOW_CLOSE && current_situation == ENCODER_SITUATION_OVERFLOW_CLOSE)
	{
		handle->_rollover_count --;
	}

	handle->_situation = current_situation;

	int32_t prev_count = handle->_total_count;
	handle->_total_count = (int32_t)*handle->tim_counter + 0xFFFF * handle->_rollover_count;

	handle->_total_count_delta = handle->_total_count - prev_count;
	return 0;
}

void Encoder16PrintStatus(struct Encoder16Handle handle)
{
	printf("processed absolute position : %"PRIi32"\n", handle._total_count);
	printf("position delta : %"PRIi32"\n", handle._total_count_delta);
	printf("rollover count : %"PRIi32"\n", handle._rollover_count);
	printf("situation : %"PRIu32"\n", handle._situation);
	printf("counter register raw : %"PRIu32"\n", TIM3->CNT);
}

int32_t Encoder16GetTicks(struct Encoder16Handle* handle)
{
	return handle->_total_count * handle->multiplier;
}

int32_t Encoder16GetTicksDelta(struct Encoder16Handle* handle)
{
	return handle->_total_count_delta * handle->multiplier;
}

// old
float encoder_get_rps(const struct Encoder16Handle encoder, float delta_ms)
{
	float ticks_per_ms = (float)encoder._total_count_delta / delta_ms;
	ticks_per_ms *= encoder.multiplier; // direction
	return (ticks_per_ms * 1000.0f) / encoder.ticks_per_revolution;
}

float encoder_get_rpm(const struct Encoder16Handle encoder, float delta_s)
{
	float ticks_per_s = (float)encoder._total_count_delta / delta_s;
	ticks_per_s *= encoder.multiplier; // direction
	return (ticks_per_s * 60.0f) / encoder.ticks_per_revolution;
}
