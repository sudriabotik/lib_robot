# Queue Statemachine


## Features

- possibilité de créer plusieurs instances d'un même type de machine d'état
- les états peuvent être arragés en queue puis exécutés séquentiellement par la machine d'état


## Créer un nouveau type de machine d'état à partir de la template

### Renommer les types

Les structs et types attachés à un type de machine d'état ont tous le même préfix.
Dans la template la machine s'appelle "Temporary", donc les structs et les types de fonctions s'appellent
```
TmpState
TmpStateMachine
TmpStateEnv
TmpStateCall
```

et toutes les fonctions attachées à la machine commencent par ```TSM``` pour Temporary State Machine.

Le plus pour renommer c'est de faire un remplacement sensible à la casse dans vscode. Il faut remplacer 
- Tmp
- TMP
- tmp
par le préfix qu'on veut.


### La structure StateEnv

C'est une structure qui est faite pour contenir toutes les variables utilisées par tous les état. Ca évite que des états utilisent des variables extérieures et permet de faire tourner en même temps autant d'instances de la machine d'état qu'on veut.

Exemple du StateEnv pour l'asservissement du robot :
``` C
/**
 * @brief A collection of variables, used by states as arguments and persistent storage.
 * A state may not use all variables, this single struct has enough variables to accomodate all of the states.
 */
struct MvStateEnv
{
	float distance;
	float speed;
	float acceleration;

	struct FuncPositionSlope pos_slope;
	struct Trapezoid trapezoid;

	enum Facing facing;

	float elapsed_time;
	float real_outcome;
};

```

### Créer les états


Il faut remplir des structs State avec les fonctiond de wake, run et stop.
Comme exemple je suis cette structure en général :

.h :
``` C
extern struct MvState MV_STATE_TRANSLATION;
struct MvStateEnv genenv_mv_state_translation(float acceleration, float speed, float distance);
```
.c :
``` C
void state_translation_wake(struct MvStateMachine* statemachine, struct MvStateEnv* env, float delta_time)
{
	// code de start de l'état
}

void state_translation_run(struct MvStateMachine* statemachine, struct MvStateEnv* env, float delta_time)
{
	// code pour chaque update

	if (exit_condition())
	{
		MSM_set_state_finished(statemachine); // indique à la machine d'état qu'elle peut passer à la suite
	}

	
}

void state_translation_stop(struct MvStateMachine* statemachine, struct MvStateEnv* env, float delta_time)
{
	// code de sortie de l'état
}

struct MvState MV_STATE_TRANSLATION =
{
	.wake = state_translation_wake,
	.run = state_translation_run,
	.stop = state_translation_stop
};

struct MvStateEnv genenv_mv_state_translation(float acceleration, float speed, float distance)
{
	struct MvStateEnv tmp =
	{
		.acceleration = acceleration, // the acceleration in mm per second square
		.speed = speed, // the speed, in mm per second
		.distance = distance // the total distance to do, in mm
	};
	return tmp;
}

```

Je trouve que faire une fonction genenv_mv_state_translation est bien pour éviter de se tromper et être sûr que toutes les variables nécessaires sont initialisées.


## Exemple d'utilisation

``` C
MSM_reset_construction(&MV_STATEMACHINE);
MSM_enqueue_state(&MV_STATEMACHINE, &MV_STATE_ROTATION, genenv_mv_state_rotation(angular_accel, angular_speed, angle_command));
MSM_enqueue_state(&MV_STATEMACHINE, &MV_STATE_RECALIBRATION, genenv_mv_state_recalibration(linear_accel, linear_speed, facing));
MSM_ready_construction(&MV_STATEMACHINE);
```