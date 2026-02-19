# include "coords.h"

# define __USE_MISC
# include <math.h>
# undef __USE_MISC
# include <stdio.h>



struct Point2D point2D_from_f(float x_mm, float y_mm)
{
	struct Point2D tmp = {.x = x_mm, .y = y_mm};
	return tmp;
}

float coord_get_target_angle_p(struct Point2D from, struct Point2D target)
{
	return atan2f(target.y - from.y, target.x - from.x);
}

float coord_get_target_angle_f(float from_x, float from_y, float target_x, float target_y)
{
	return atan2f(target_y - from_y, target_x - from_x);
}

float coord_normalize_angle(float angle_rad)
{
	float result = angle_rad;

	// normalise dans [-pi, pi]
	if (result > M_PI) result -= 2.0f * M_PI;
	if (result < -M_PI) result += 2.0f * M_PI;

	return result;
}

float coord_get_delta_angle(float from_rad, float to_rad)
{
	float delta = to_rad - from_rad;
	return coord_normalize_angle(delta);
}

struct Point2D coord_vectorize(float dir_rad, float length)
{
	struct Point2D result = {.x = cosf(dir_rad) * length, .y = sinf(dir_rad) * length};
	return result;
}

float coord_get_distance(struct Point2D from, struct Point2D target)
{
	return sqrtf(
		(from.x - target.x) * (from.x - target.x) +
		(from.y - target.y) * (from.y - target.y)
	);
}



void spacestate_set_pos_auto(struct SpaceState *spacestate, struct Point2D new_pos, float delta_time_ms)
{
	struct Point2D delta = point2D_from_f(new_pos.x - spacestate->pos_x_mm, new_pos.y - spacestate->pos_y_mm);
	spacestate->x_vel = delta.x * 1000 / delta_time_ms;
	spacestate->y_vel = delta.y * 1000 / delta_time_ms;
	spacestate->lin_vel = spacestate->x_vel * spacestate->x_vel + spacestate->y_vel * spacestate->y_vel;
	spacestate->pos_x_mm = new_pos.x;
	spacestate->pos_y_mm = new_pos.y;
}

void spacestate_set_dir_auto(struct SpaceState *spacestate, float new_dir, float delta_time_ms);

void spacestate_set_pos(struct SpaceState *spacestate, struct Point2D new_pos)
{
	spacestate->pos_x_mm = new_pos.x;
	spacestate->pos_y_mm = new_pos.y;
}

void spacestate_set_dir(struct SpaceState *spacestate, float new_dir)
{
	spacestate->dir_rad = new_dir;
}

void spacestate_print(struct SpaceState *spacestate, const char* title)
{
	printf(
		"spacestate %s :\n"
		"position : x:%.2f y:%.2f mm\n"
		"orientation : %.2f degrees\n"
		"linear speed : %.2f mm/s\n"
		"angular speed : %.2f deg/s\n",
		title,
		spacestate->pos_x_mm, spacestate->pos_y_mm,
		spacestate->dir_rad * 180.0f / M_PI,
		spacestate->lin_vel,
		spacestate->ang_vel * 180.0f / M_PI
	);
}