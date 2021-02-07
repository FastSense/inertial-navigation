// Jan-2021
/*	fsnav_ins_motion
	
	fsnav plugins for ins position and velocity algorithms:
	
	- fsnav_ins_motion_euler
		Numerically integrates Newton's second Law in local level navigation frame 
		using modified Euler's method (with midpoint for attitude matrix) over the Earth reference ellipsoid. 
		Updates velocity and geographical coordinates. 
		Not suitable near the Earth's poles, at outer-space altitudes, and/or over-Mach velocities.
		
	- (planned) fsnav_ins_motion_sculling
		Planned for future development
		
	- fsnav_ins_motion_vertical_damping
		Restrains vertical error exponential growth by either damping
		vertical velocity to zero, or to an external reference like
		altitude/rate derived from air data system and/or gnss, etc.
		Recommended when using normal gravity model and/or long navigation timeframe.
*/

#include <stdlib.h>
#include <math.h>

#include "../fsnav.h"

// fsnav bus version check
#define FSNAV_INS_MOTION_BUS_VERSION_REQUIRED 8
#if FSNAV_BUS_VERSION < FSNAV_INS_MOTION_BUS_VERSION_REQUIRED
	#error "fsnav bus version check failed, consider fetching the latest version"
#endif

// service functions
double fsnav_ins_motion_parse_double(const char *token, char *src, const size_t len, double range[2], const double default_value);
void   fsnav_ins_motion_flip_sol_over_pole(fsnav_sol *sol);

/* fsnav_ins_motion_euler - fsnav plugin
	
	Numerically integrates Newton's second Law in local level navigation frame 
	using modified Euler's method (with midpoint for attitude matrix) over the Earth reference ellipsoid. 
	Updates velocity and geographical coordinates. 
	Not suitable near the Earth's poles, at outer-space altitudes, and/or over-Mach velocities.

	description:
	    
		V  (t+dt) = V  (t) + dt*([(W + 2u) x]*V (t) +  L^T(t+dt/2)*f + g)
		lon(t+dt) = lon(t) + dt*              Ve(t)/((Re + alt(t))*cos(lat(t)))
		lat(t+dt) = lat(t) + dt*              Vn(t)/( Rn + alt(t))
		alt(t+dt) = alt(t) + dt*              Vu(t)              
		    
		where
		    [Ve]          [  0   v3 -v2 ]
		V = [Vn], [v x] = [ -v3  0   v1 ] for v = W + 2u,
			[Vu]          [  v2 -v1  0  ]
		Rn, Re are curvature radii
		L is attitude matrix

		do not use at the Earth's poles
		
	uses:
		fsnav->imu->t
		fsnav->imu->sol.llh
		fsnav->imu->sol.llh_valid
		fsnav->imu->sol.v
		fsnav->imu->sol.v_valid
		fsnav->imu->sol.L
		fsnav->imu->sol.L_valid
		fsnav->imu->f
		fsnav->imu->f_valid
		fsnav->imu->w
		fsnav->imu->w_valid
		fsnav->imu->g
		fsnav->imu->g_valid
		fsnav->imu->W
		fsnav->imu->W_valid

	changes:
		fsnav->imu->sol.v
		fsnav->imu->sol.v_valid
		fsnav->imu->sol.llh
		fsnav->imu->sol.llh_valid

	cfg parameters:
		{imu: lon} - starting longitude, degrees
			type :   floating point
			range:   -180..+180
			default: 0
			example: {imu: lon = 37.6}
		{imu: lat} - starting latitude, degrees
			type :   floating point
			range:   -90..+90
			default: 0
			example: {imu: lat = 55.7}
		{imu: alt} - starting altitude, meters
			type :   floating point
			range:   -20000..+50000
			default: 0
			example: {imu: alt = 151.3}
*/
void fsnav_ins_motion_euler(void) {

	const char 
		lon_token[] = "lon",        // starting longitude parameter name in configuration
		lat_token[] = "lat",        // starting latitude  parameter name in configuration
		alt_token[] = "alt";        // starting altitude  parameter name in configuration

	const double 
		lon_range[] = {-180, +180},	// longitude range, 0 by default
		lat_range[] = { -90,  +90},	// latitude  range, 0 by default
		alt_range[] = {-20e3,50e3},	// altitude  range, 0 by default
		eps = 1.0/0x0100;			// 2^-8, guaranteed non-zero value in IEEE754 half-precision format

	static double t0  = -1;         // previous time

	double dt;                      // time step
	double
		sphi, cphi,	                // sine and cosine of latitude
		Rn_h, Re_h,	                // south-to-north and east-to-west curvature radii, altitude-adjusted
		e2s2, e4s4,	                // e^2*sin(phi)^2, (e^2*sin(phi)^2)^2
		dvrel[3],	                // proper acceleration in navigation frame
		dvcor[3],	                // Coriolis acceleration in navigation frame
		C_2[9];                     // intermediate matrix/vector for modified Euler component
	size_t i, j;                    // common index variables


	// check if imu data has been initialized
	if (fsnav->imu == NULL)
		return;

	if (fsnav->mode == 0) {		// init

		// drop validity flags
		fsnav->imu->sol.  v_valid = 0;
		fsnav->imu->sol.llh_valid = 0;		
		// parse parameters from configuration	
		fsnav->imu->sol.llh[0] = // starting longitude
			fsnav_ins_motion_parse_double(lon_token, fsnav->imu->cfg,fsnav->imu->cfglength, (double *)lon_range,0)
			/fsnav->imu_const.rad2deg; // degrees to radians
		fsnav->imu->sol.llh[1] = // starting latitude
			fsnav_ins_motion_parse_double(lat_token, fsnav->imu->cfg,fsnav->imu->cfglength, (double *)lat_range,0)
			/fsnav->imu_const.rad2deg; // degrees to radians
		fsnav->imu->sol.llh[2] = // starting altitude
			fsnav_ins_motion_parse_double(alt_token, fsnav->imu->cfg,fsnav->imu->cfglength, (double *)alt_range,0);
		// raise coordinates validity flag
		fsnav->imu->sol.llh_valid = 1;
		// zero velocity at start
		for (i = 0; i < 3; i++)
			fsnav->imu->sol.v[i] = 0;
		fsnav->imu->sol.v_valid = 1;
		// reset previous time
		t0 = -1;

	}

	else if (fsnav->mode < 0) {	// termination
		// do nothing
	}

	else						// main cycle
	{
		// check for crucial data initialized
		if (   !fsnav->imu->sol.  v_valid 
			|| !fsnav->imu->sol.llh_valid 
			|| !fsnav->imu->sol.  L_valid 
			|| !fsnav->imu->f_valid 
			|| !fsnav->imu->g_valid)
			return;
		// time variables
		if (t0 < 0) { // first touch
			t0 = fsnav->imu->t;
			return;
		}
		dt = fsnav->imu->t - t0;
		t0 = fsnav->imu->t;
		// ellipsoid geometry
		sphi = sin(fsnav->imu->sol.llh[1]);
		cphi = cos(fsnav->imu->sol.llh[1]);
		e2s2 = fsnav->imu_const.e2*sphi*sphi;
		e4s4 = e2s2*e2s2;					
		Re_h = fsnav->imu_const.a*(1 + e2s2/2 + 3*e4s4/8);	                                        // Taylor expansion within 0.5 m, not altitude-adjusted
		Rn_h = Re_h*(1 - fsnav->imu_const.e2)*(1 + e2s2 + e4s4 + e2s2*e4s4) + fsnav->imu->sol.llh[2]; // Taylor expansion within 0.5 m
		Re_h += fsnav->imu->sol.llh[2];						                                        // adjust for altitude
		// drop validity flags
		fsnav->imu->W_valid       = 0;
		fsnav->imu->sol.llh_valid = 0;
		fsnav->imu->sol.  v_valid = 0;
		// angular rate of navigation frame relative to the Earth
		fsnav->imu->W[0] = -fsnav->imu->sol.v[1]/Rn_h;
		fsnav->imu->W[1] =  fsnav->imu->sol.v[0]/Re_h;	
		if (cphi < eps) { // check for Earth pole proximity
			fsnav->imu->W[2] = 0;    // freeze
			fsnav->imu->W_valid = 0; // drop validity
		}
		else {
			fsnav->imu->W[2] = fsnav->imu->sol.v[0]/Re_h*sphi/cphi;
			fsnav->imu->W_valid = 1;
		}
		// velocity
			// Coriolis acceleration
		dvrel[0] = fsnav->imu->W[0];
		dvrel[1] = fsnav->imu->W[1] + 2*fsnav->imu_const.u*cphi;
		dvrel[2] = fsnav->imu->W[2] + 2*fsnav->imu_const.u*sphi;
		fsnav_linal_cross3x1(dvcor, fsnav->imu->sol.v, dvrel);
			// proper acceleration
		if (fsnav->imu->w_valid) { // if able to calculate attitude mid-point using gyroscopes
			for (i = 0; i < 3; i++)
				dvrel[i] = fsnav->imu->w[i]*dt/2; // midpoint rotation Euler vector
			fsnav_linal_eul2mat(C_2, dvrel); // midpoint attitude matrix factor
			for (i = 0; i < 3; i++) // multiply C_2*f, store in the first row of C_2
				for (j = 1, C_2[i] = C_2[i*3]*fsnav->imu->f[0]; j < 3; j++)
					C_2[i] += C_2[i*3+j]*fsnav->imu->f[j];
			fsnav_linal_mmul1T(dvrel, fsnav->imu->sol.L, C_2, 3, 3, 1); // dvrel = L^T(t+dt)*C_2(w*dt/2)*f
		}
		else // otherwise, go on with only the current attitude matrix
			fsnav_linal_mmul1T(dvrel, fsnav->imu->sol.L, fsnav->imu->f, 3, 3, 1);
			// velocity update
		for (i = 0; i < 3; i++)
			fsnav->imu->sol.v[i] += (dvcor[i] + dvrel[i] + fsnav->imu->g[i])*dt;
		fsnav->imu->sol.v_valid = 1;
		// coordinates
		if (cphi < eps) { // check for Earth pole proximity
			fsnav->imu->sol.llh[2] += fsnav->imu->sol.v[2]				*dt;
			fsnav->imu->sol.llh_valid = 0; // drop validity
		}
		else {
			fsnav->imu->sol.llh[0] += fsnav->imu->sol.v[0]/(Re_h*cphi)	*dt;
			fsnav->imu->sol.llh[1] += fsnav->imu->sol.v[1]/ Rn_h			*dt;
			fsnav->imu->sol.llh[2] += fsnav->imu->sol.v[2]				*dt;
			// flip latitude if crossed a pole
			if (fsnav->imu->sol.llh[1] < -fsnav->imu_const.pi/2) { // South pole
				fsnav->imu->sol.llh[1] = -fsnav->imu_const.pi - fsnav->imu->sol.llh[1];
				fsnav_ins_motion_flip_sol_over_pole(&(fsnav->imu->sol));			
			}
			if (fsnav->imu->sol.llh[1] > +fsnav->imu_const.pi/2) { // North pole
				fsnav->imu->sol.llh[1] = +fsnav->imu_const.pi - fsnav->imu->sol.llh[1];
				fsnav_ins_motion_flip_sol_over_pole(&(fsnav->imu->sol));			
			}
			// adjust longitude into range
			while (fsnav->imu->sol.llh[0] < -fsnav->imu_const.pi)
				fsnav->imu->sol.llh[0] += 2*fsnav->imu_const.pi;
			while (fsnav->imu->sol.llh[0] > +fsnav->imu_const.pi)
				fsnav->imu->sol.llh[0] -= 2*fsnav->imu_const.pi;
			fsnav->imu->sol.llh_valid = 1;
		}
	}

}

/* fsnav_ins_motion_vertical_damping - fsnav plugin
	
	Restrains vertical error exponential growth by either damping
	vertical velocity to zero, or to an external reference like
	altitude/rate derived from air data system and/or gnss, etc.
	Recommended when using normal gravity model and/or long navigation timeframe.

	description:
		performs vertical channel correction using
		- zero vertical velocity model
		- air data altitude and altitude rate of change
		- (planned) gnss-derived altitude and vertical velocity
		- etc.
								      [x]          [v]
		algorithm state vector is y = [v], dy/dt = [q], M[q^2] = vvs^2

	uses:
		fsnav->imu->t
		fsnav->air->alt
		fsnav->air->alt_valid
		fsnav->air->alt_std
		fsnav->air->vv
		fsnav->air->vv_valid
		fsnav->air->vv_std
		fsnav->imu->sol.llh[2]
		fsnav->imu->sol.llh_valid
		fsnav->imu->sol.  v[2]
		fsnav->imu->sol.  v_valid

	changes:
		fsnav->imu->sol.llh[2]
		fsnav->imu->sol.  v[2]

	cfg parameters:
		{imu: vertical_damping_stdev} - vertical velocity stdev (vvs), m/s
			type :   floating point
			range:   >0
			default: 2^20 (no damping)
			example: {imu: vertical_damping_stdev = 9e9}
			set vvs = 0 to force zero vertical velocity
			negative values result in default
*/
void fsnav_ins_motion_vertical_damping(void) {

	const char vvs_token[] = "vertical_damping_stdev"; // vertical velocity stdev parameter name in configuration

	const double 
		vvs_def = (double)(0x100000),                  // 2^20, vertical velocity stdev default value
		sqrt2   = 1.4142135623730951;                  // sqrt(2)

	static double 
		t0           = -1, // previous time
		vvs          =  0, // vertical velocity stdev
		air_alt_last =  0; // previous value of air altitude

	double 
		dt,	  // time step
		x,    // inertial altitude
		v,    // inertial vertical velocity
		z,    // reference information
		s,    // reference information a priori stdev
		h[2], // model coefficients
		y[2], // algorithm state vector
		S[3], // upper-triangular part of cobariance Cholesky factorization
		K[2], // Kalman gain
		w;    // weight


	// check if imu data has been initialized
	if (fsnav->imu == NULL)
		return;

	if (fsnav->mode == 0) {		// init

		// reset time
		t0 = -1;
		// parse vertical velocity stdev from configuration string
		vvs = fsnav_ins_motion_parse_double(vvs_token, fsnav->imu->cfg, fsnav->imu->cfglength, NULL, vvs_def);
		if (vvs < 0)
			vvs = vvs_def;

	}

	else if (fsnav->mode < 0) {	// termination
		// do nothing
	}
	else						// main cycle
	{
		// time variables
		if (t0 < 0) {
			t0 = fsnav->imu->t;
			if (fsnav->air != NULL && fsnav->air->alt_valid)
				air_alt_last = fsnav->air->alt;
			return;
		}
		dt = fsnav->imu->t - t0;
		t0 = fsnav->imu->t;
		if (dt <= 0)
			return;
		// estimation init 
		if (fsnav->imu->sol.llh_valid) x = fsnav->imu->sol.llh[2]; else x = 0;
		if (fsnav->imu->sol.  v_valid) v = fsnav->imu->sol.  v[2]; else v = 0;
		y[0] = x, y[1] = v;
		S[0] = vvs_def*dt, S[1] = 0, S[2] = 1;
		// zero vertical velocity
		if (fsnav->imu->sol.v_valid) {
			s = vvs;
			z = 0.0;
			h[0] = 0, h[1] = 1;
			fsnav_linal_kalman_update(y,S,K, z,h,s, 2);
		}
		// check for air data
		if (fsnav->air != NULL) {
			if (fsnav->air->alt_valid && fsnav->imu->sol.llh_valid) { // altitude data present
				// altitude
				s = sqrt2*( (fsnav->air->alt_std > 0) ? (fsnav->air->alt_std) : vvs );
				z = fsnav->air->alt + air_alt_last; // decorrelated with velocity information
				h[0] = 2, h[1] = -dt;
				fsnav_linal_kalman_update(y,S,K, z,h,s, 2);
				// altitude rate of change
				z = fsnav->air->alt - air_alt_last; // decorrelated with altitude information
				h[0] = 0, h[1] = dt;
				fsnav_linal_kalman_update(y,S,K, z,h,s, 2);
				air_alt_last = fsnav->air->alt;
			}
			if (fsnav->air->vv_valid && fsnav->imu->sol.v_valid) { // vertical velocity data present
				z = fsnav->air->vv - fsnav->imu->sol.v[2];
				s = (fsnav->air-> vv_std > 0) ? (fsnav->air-> vv_std) : vvs;
				fsnav_linal_kalman_update(y,S,K, z,h,s, 2);
			}
		}

		// check for gnss data
		//
		// TBD
		//

		// vertical channel correction
		s = sqrt(S[0]*S[0] + S[1]*S[1]);  // altitude stdev estimate
		w = s + S[2]*dt;                  // sum of altitude stdev and velocity contribution into stdev
		if (fsnav->imu->sol.llh_valid) 
			fsnav->imu->sol.llh[2] = y[0]; // replace altitude by its estimated value
		if (fsnav->imu->sol.  v_valid) {
			fsnav->imu->sol.llh[2] += 2*s      /w*(y[1]-v)*dt; // weighted correction to hold dx/dt = v
			fsnav->imu->sol.  v[2] = y[1]; // replace vertical velocity by its estimated value
		}
		if (fsnav->imu->sol.llh_valid) 
			fsnav->imu->sol.  v[2] += 2*S[2]*dt/w*(y[0]-x)/dt; // weighted correction to hold dx/dt = v

	}

}





// service functions
double fsnav_ins_motion_parse_double(const char *token, char *src, const size_t len, double *range, const double default_value) {

	char   *cfg_ptr; // pointer to a substring
	double  val;     // value

	// ensure variable to be initialized
	val = default_value;
	// parse token from src
	cfg_ptr = fsnav_locate_token(token, src, len, '=');
	if (cfg_ptr != NULL)
		val = atof(cfg_ptr);
	// if not found or out of range, set to default
	if ( cfg_ptr == NULL || (range != NULL && (range[0] > range[1] || val < range[0] || range[1] < val)) )
		val = default_value;
	return val;

}

void fsnav_ins_motion_flip_sol_over_pole(fsnav_sol *sol) {

	size_t i;

	// longitude
	sol->llh[0] +=  fsnav->imu_const.pi;
	// velocity
	sol->v  [0]  = -sol->v[0];
	sol->v  [1]  = -sol->v[1];
	// attitude matrix
	for (i = 0; i < 3; i++) {
		sol->L[i*3+0] = -sol->L[i*3+0];
		sol->L[i*3+1] = -sol->L[i*3+1];
	}
	// update quaternion
	fsnav_linal_mat2quat(sol->q  ,sol->L);
	// update angles
	fsnav_linal_mat2rpy (sol->rpy,sol->L);

}
