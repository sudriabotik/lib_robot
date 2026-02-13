#include "queue_statemachine.h"

#include <stdio.h>



struct TmpStateMachine TSM_init()
{
	struct TmpStateMachine machine =
	{
		.index = 0,
		.construction_index = 0,
		.load_construction_queue = 0,
	};

	for (uint_t i = 0; i <= TMP_STATEMACHINE_LENGTH; i++) machine.state_queue[i] = 0;

	return machine;
}

int_t TSM_reset_construction(struct TmpStateMachine *machine)
{
	machine->construction_index = 0;
	return 0;
}

int_t TSM_enqueue_state(struct TmpStateMachine *machine, struct TmpState *state, struct TmpStateEnv env)
{
	if (TSM_is_construction_full(machine)) return -1;

	machine->construction_state_queue[machine->construction_index] = state;
	machine->construction_envs_queue[machine->construction_index] = env;
	machine->construction_index ++;

	return 0;
}

int_t TSM_ready_construction(struct TmpStateMachine *machine)
{
	machine->load_construction_queue = 1;
	return 0;
}

uint_t TSM_get_construction_index(struct TmpStateMachine *machine)
{
	return machine->construction_index;
}

int_t TSM_is_construction_full(struct TmpStateMachine *machine)
{
	return machine->construction_index >= TMP_STATEMACHINE_LENGTH;
}

int TSM_is_busy(struct TmpStateMachine *machine)
{
	return machine->state_queue[machine->index] != 0;
}

int TSM_update(struct TmpStateMachine *machine, float delta_time)
{
	struct TmpState *current_state = machine->state_queue[machine->index];
	struct TmpStateEnv *current_env = &machine->envs_queue[machine->index];

	// tests if a new state sequence is ready to be loaded in
	if (machine->load_construction_queue)
	{
		machine->load_construction_queue = 0;
	
		// exits the current state
		if (current_state != 0) if (current_state->stop != 0) current_state->stop(machine, current_env, delta_time);
		// loads the construction state queue as the current state queue
		uint_t i = 0;
		for (i = 0; i < machine->construction_index; i++)
		{
			machine->state_queue[i] = machine->construction_state_queue[i];
			machine->envs_queue[i] = machine->construction_envs_queue[i];
		}
		for (; i < TMP_STATEMACHINE_LENGTH; i++)
		{
			machine->state_queue[i] = 0;
		}
		machine->index = 0;
		machine->state_finished = 0;

		// updates the current state and env
		current_state = machine->state_queue[machine->index];
		current_env = &machine->envs_queue[machine->index];


		// wakes up the new state
		if (current_state != 0) if (current_state->stop != 0) current_state->wake(machine, current_env, delta_time);

		printf("loaded new state queue\n");
	}

	// test if the currently active state has finished
	if (machine->state_finished == 1)
	{
		// exits the current state
		if (current_state != 0) if (current_state->stop != 0) current_state->stop(machine, current_env, delta_time);

		if (machine->index < TMP_STATEMACHINE_LENGTH)
		{
			machine->index ++;

			// updates the current state and env
			current_state = machine->state_queue[machine->index];
			current_env = &machine->envs_queue[machine->index];

			// wakes up the current state
			if (current_state != 0) if (current_state->stop != 0) current_state->wake(machine, current_env, delta_time);

			machine->state_finished = 0;
		}

	}

	if (current_state != 0) current_state->run(machine, current_env, delta_time);

	return 0;
}

int_t TSM_set_state_finished(struct TmpStateMachine *machine)
{
	machine->state_finished = 1;
	return 0;
}