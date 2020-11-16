// Oct-2020
/*	fsnav_ins_motion
	
	fsnav plugins for ins position and velocity algorithms
*/
void fsnav_ins_motion_euler           (void); // velocity and position using first-order Euler integration
void fsnav_ins_motion_sculling        (void); // planned for future development
void fsnav_ins_motion_vertical_damping(void); // vertical error buildup damping
