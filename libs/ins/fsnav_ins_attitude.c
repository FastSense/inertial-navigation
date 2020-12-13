// Dec-2020
/*	fsnav_ins_attitude
	
	fsnav plugins for ins angular rate integration:
	
	- fsnav_ins_attitude_rodrigues
		Combines angular rate components or their integrals into Euler rotation vector 
		for both instrumental frame and navigation frame. Then applies Rodrigues' rotation formula 
		to each frame. Then derives the transition matrix between them. Quaternion and angles are also updated.
		Recommended for navigation/tactical grade systems.

	- (planned) fsnav_ins_attitude_madgwick
		Planned for future development
*/

#include <math.h>

#include "../fsnav.h"

// fsnav bus version check
#define FSNAV_INS_ATTITUDE_BUS_VERSION_REQUIRED 8
#if FSNAV_BUS_VERSION < FSNAV_INS_ATTITUDE_BUS_VERSION_REQUIRED
	#error "fsnav bus version check failed, consider fetching the latest version"
#endif

/* fsnav_ins_attitude_rodrigues - fsnav plugin
	
	Combines angular rate components or their integrals into Euler rotation vector 
	for both instrumental frame and navigation frame. Then applies Rodrigues' rotation formula 
	to each frame. Then derives the transition matrix between them. Quaternion and angles are also updated.
	Recommended for navigation/tactical grade systems.

	description:
		    
		L(t+dt) = A L(t) C^T,
		    
		where
		A = E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2
		C = E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2,
		a = w*dt, c = (W + u)*dt
		        [  0   v3 -v2 ]
		[v x] = [ -v3  0   v1 ] for v = a, v = c
	            [  v2 -v1  0  ]
				
		fsnav core function fsnav_linal_eul2mat safely uses Taylor expansions for |a|,|c| < 2^-8
		
	uses:
		fsnav->imu->t
		fsnav->imu->sol.L
		fsnav->imu->sol.L_valid
		fsnav->imu->w
		fsnav->imu->w_valid
		fsnav->imu->W
		fsnav->imu->W_valid
		fsnav->imu->sol.llh
		fsnav->imu->sol.llh_valid

	changes:
		fsnav->imu->sol.L
		fsnav->imu->sol.L_valid
		fsnav->imu->sol.q
		fsnav->imu->sol.q_valid
		fsnav->imu->sol.rpy
		fsnav->imu->sol.rpy_valid

	cfg parameters:
		none
*/
void fsnav_ins_attitude_rodrigues(void) {

	static double t0 = -1; // previous time
	static double 		   
		 C[9],             // intermediate matrix
		*L;                // pointer to attitude matrix in solution

	double 
		   dt,	           // time step
		   a[3];           // Euler rotation vector
	size_t i;              // common index variable


	// check if imu data has been initialized
	if (fsnav->imu == NULL)
		return;

	if (fsnav->mode == 0) {		// init

		// drop validity flags
		fsnav->imu->sol.  q_valid = 0;
		fsnav->imu->sol.  L_valid = 0;
		fsnav->imu->sol.rpy_valid = 0;
		// set matrix pointer to imu->sol
		L = fsnav->imu->sol.L;	
		// identity quaternion
		for (i = 1, fsnav->imu->sol.q[0] = 1; i < 4; i++)
			fsnav->imu->sol.q[i] = 0;
		fsnav->imu->sol.  q_valid = 1;
		// identity attitude matrix
		for (i = 0; i < 9; i++)
			L[i] = ((i%4) == 0) ? 1 : 0; // for 3x3 matrix, each 4-th element is diagonal
		fsnav->imu->sol.  L_valid = 1;
		// attitude angles for identity matrix
		fsnav->imu->sol.rpy[0] = -fsnav->imu_const.pi/2;	// roll             -90 deg
		fsnav->imu->sol.rpy[1] =  0;						// pitch              0 deg
		fsnav->imu->sol.rpy[2] = +fsnav->imu_const.pi/2;	// yaw=true heading +90 deg
		fsnav->imu->sol.rpy_valid = 1;
		// reset previous time
		t0 = -1;

	}

	else if (fsnav->mode < 0) {	// termination
		// do nothing
	}
	else						// main cycle
	{
		// check for crucial data initialized
		if (!fsnav->imu->sol.L_valid || !fsnav->imu->w_valid)
			return;
		// time variables
		if (t0 < 0) { // first touch
			t0 = fsnav->imu->t;
			return;
		}
		dt = fsnav->imu->t - t0;
		t0 = fsnav->imu->t;
		// a = w*dt
		for (i = 0; i < 3; i++)
			a[i] = fsnav->imu->w[i]*dt;
		// L = (E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2)*L
		fsnav_linal_eul2mat(C,a);  // C <- A = E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2
		for (i = 0; i < 3; i++) { // L = A*L
			a[0] = L[0+i], a[1] = L[3+i], a[2] = L[6+i];
			L[0+i] = C[0]*a[0] + C[1]*a[1] + C[2]*a[2];
			L[3+i] = C[3]*a[0] + C[4]*a[1] + C[5]*a[2];
			L[6+i] = C[6]*a[0] + C[7]*a[1] + C[8]*a[2];
		}
		// a <- c = (W + u)*dt
		for (i = 0; i < 3; i++)
			a[i] = fsnav->imu->W_valid ? fsnav->imu->W[i] : 0;
		if (fsnav->imu->sol.llh_valid) {
			a[1] += fsnav->imu_const.u*cos(fsnav->imu->sol.llh[1]);
			a[2] += fsnav->imu_const.u*sin(fsnav->imu->sol.llh[1]);
		}
		for (i = 0; i < 3; i++)
			a[i] *= dt;
		// L = L*(E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2)^T
		fsnav_linal_eul2mat(C,a);     // C = E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2
		for (i = 0; i < 9; i += 3) { // L = L*C^T
			a[0] = L[i+0], a[1] = L[i+1], a[2] = L[i+2];
			L[i+0] = a[0]*C[0] + a[1]*C[1] + a[2]*C[2];
			L[i+1] = a[0]*C[3] + a[1]*C[4] + a[2]*C[5];
			L[i+2] = a[0]*C[6] + a[1]*C[7] + a[2]*C[8];
		}
		// renew quaternion
		fsnav_linal_mat2quat(fsnav->imu->sol.q ,L);
		fsnav->imu->sol.q_valid   = 1;
		// renew angles
		fsnav_linal_mat2rpy(fsnav->imu->sol.rpy,L);
		fsnav->imu->sol.rpy_valid = 1;

	}

}