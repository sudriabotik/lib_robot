# ifndef __COORDS_H
# define __COORDS_H



/**
 * @brief Coordonnées d'un point sur le terrain (mm).
 */
struct Point2D
{
	float x;
	float y;
};

/**
 * @brief Pseudo-constructor for Point2D
 * @param x_mm x position in mm
 * @param y_mm y position in mm
 */
struct Point2D point2D_from_f(float x_mm, float y_mm);



/**
 * @brief A structure containing all the informations about the current position, orientation, and speed of the robot.
 * Follows trigonometric rules.
 * @warning Instead of modifying the struct's field directly, use the provided functions.
 */
struct SpaceState
{
	// the position in mm along the x axis
	float pos_x_mm;
	// the position in mm along the y axis
	float pos_y_mm;

	// the angle between the forward vector and the positive X vector.
	float dir_rad;

	// the linear velocity in mm/s
	float lin_vel;
	// the angular velocity in rad/s
	float ang_vel;

	float x_vel;
	float y_vel;
};

/**
 * @brief Updates the position stored in the spacestate.
 * Uses the delta_time_ms argument and the previous position to deduce the linear speed.
 */
void spacestate_set_pos_auto(struct SpaceState *spacestate, struct Point2D new_pos, float delta_time_ms);

/**
 * @brief Updates the orientation stored in the spacestate.
 * Uses the delta_time_ms argument and the previous orientation to deduce the angulat speed.
 * @param new_dir the new orientation in radians
 */
void spacestate_set_dir_auto(struct SpaceState *spacestate, float new_dir, float delta_time_ms);

/**
 * @brief Sets the position
 */
void spacestate_set_pos(struct SpaceState *spacestate, struct Point2D new_pos);

/**
 * @brief Sets the orientation.
 * @param new_dir the new orientation in radians
 */
void spacestate_set_dir(struct SpaceState *spacestate, float new_dir);

/**
 * @brief Formatted print of a SpaceState's contents
 */
void spacestate_print(struct SpaceState *spacestate, const char* title);


/**
 * @brief Calcule l'angle absolu (radians) que le robot doit avoir pour faire
 *        face au point cible.
 */
float coord_get_target_angle_p(struct Point2D from, struct Point2D target);

/**
 * @brief Calcule l'angle absolu (radians) que le robot doit avoir pour faire
 *        face au point cible.
 */
float coord_get_target_angle_f(float from_x, float from_y, float target_x, float target_y);

/**
 * @brief Normalize the angle
 */
float coord_normalize_angle(float angle_rad);

/**
 * @brief Calcule la rotation relative (radians) à effectuer pour passer de from_rad à to_rad.
 */
float coord_get_delta_angle(float from_rad, float to_rad);

/**
 * @brief Converts an angle and distance to a vector with xy coords.
 */
struct Point2D coord_vectorize(float dir_rad, float length);



/**
 * @brief Calcule la distance (mm) à parcourir jusqu'au point cible.
 *        Retourne une valeur négative si face == FACE_ARRIERE.
 */
float coord_get_distance(struct Point2D from, struct Point2D target);


# endif