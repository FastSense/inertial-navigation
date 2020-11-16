// Oct-2020
/*	fsnav_ins_alignment 
	
	fsnav plugins for ins initial alignment (initial attitude matrix determination):
*/
void fsnav_ins_alignment_static      (void); // conventional sensor output averaging on a static base
void fsnav_ins_alignment_rotating    (void); // gravity vector approximation in inertial reference
void fsnav_ins_alignment_rotating_rpy(void); // gravity vector approximation in inertial reference, but matrix is computed via roll, pitch and yaw=true heading, if defined
