// Jan-2021
/*	fsnav_ins_attitude
	
	fsnav plugins for ins angular rate integration:
*/
void fsnav_ins_attitude_rodrigues(void); // via Euler vector using Rodrigues' rotation formula
void fsnav_ins_attitude_madgwick (void); // via Madgwick filter fused with accelerometer data
