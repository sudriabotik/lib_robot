#ifndef __TMP_STATEMACHINE_H
#define __TMP_STATEMACHINE_H

/**
 * @file queue_statemachine.h
 *
 * @brief The statemachine used for movement control
 */



/**
 * @brief The queue size for the statemachine
 */
# define TMP_STATEMACHINE_LENGTH 4


// early declarations
struct TmpStateMachine;
struct TmpStateEnv;



typedef unsigned int uint_t;
typedef int int_t;

typedef void (*TmpStateCall)(struct TmpStateMachine*, struct TmpStateEnv*, float delta_time);


/**
 * @brief A collection of variables, used by states as arguments and persistent storage.
 * A state may not use all variables, this single struct has enough variables to accomodate all of the states.
 */
struct TmpStateEnv
{
	int n;
};



struct TmpState
{
	TmpStateCall wake;
	TmpStateCall run;
	TmpStateCall stop;	
};



struct TmpStateMachine
{
	struct TmpState* state_queue[TMP_STATEMACHINE_LENGTH+1];
	struct TmpStateEnv envs_queue[TMP_STATEMACHINE_LENGTH+1];
	uint_t index;
	int_t state_finished;

	struct TmpState* construction_state_queue[TMP_STATEMACHINE_LENGTH];
	struct TmpStateEnv construction_envs_queue[TMP_STATEMACHINE_LENGTH];
	uint_t construction_index;
	uint_t load_construction_queue;
};



/**
 * @brief Always use this function to initialize a new statemachine struct.
 * It will initialize some fields of the struct.
 * @return The initialized struct.
 */
struct TmpStateMachine TSM_init();

/**
 * @brief Empties the construction states queue, allowing it to be redefined from the start.
 * @return 0
 */
int_t TSM_reset_construction(struct TmpStateMachine *machine);

/**
 * @brief Put this state and the associated environment to the construction queue.
 * @return 0 if successful, -1 id the queue is full.
 */
int_t TSM_enqueue_state(struct TmpStateMachine *machine, struct TmpState *state, struct TmpStateEnv env);

/**
 * @brief Flags the construction queue as ready to be loaded. It will be moved to the active queue the next time the statemachine is updated.
 * @return 0
 */
int_t TSM_ready_construction(struct TmpStateMachine *machine);

/**
 * @brief Get the index where the next state we add to the construction queue will be put.
 * @return The current construction index.
 */
uint_t TSM_get_construction_index(struct TmpStateMachine *machine);

/**
 * @brief Check if the construction queue is full.
 * @return 1 if the construction queue is full, 0 otherwise.
 */
int_t TSM_is_construction_full(struct TmpStateMachine *machine);

/**
 * @brief Check if the statemachine is still executing any state from the queue.
 * @return 1 if the statemachine is busy, 0 otherwise.
 */
int_t TSM_is_busy(struct TmpStateMachine *machine);

/**
 * @brief Put this state and the associated environment to the construction queue.
 * @return 0
 */
int_t TSM_update(struct TmpStateMachine *machine, float delta_time);

/**
 * @brief Indicates the current state is done, and the statemachine may go to the next one.
 * @return 0
 */
int_t TSM_set_state_finished(struct TmpStateMachine *machine);



#endif
