// Dec-2020
/*	fsnav_ins_gravity 
	
	fsnav plugins for gravity model calculations:

	- fsnav_ins_gravity_constant 
		Takes the magnitude of average accelerometer output vector 
		and puts it into vertical gravity component, which then remains constant.
		Recommended for low-grade systems, especially when no reference coordinates available.

	- fsnav_ins_gravity_normal
		Computes conventional Earth normal gravity model as in GRS80, etc.,
		but takes Earth model constants from fsnav->imu_const variables.
		Accounts for both latitude and altitude, as well as for plumb line curvature above ellipsoid.
		Recommended for conventional navigation grade systems.

	- (planned) fsnav_ins_gravity_egm08
		Planned for future development.
	
*/

#include <stdlib.h>
#include <math.h>

#include "../fsnav.h"

// fsnav bus version check
#define FSNAV_INS_GRAVITY_BUS_VERSION_REQUIRED 8
#if FSNAV_BUS_VERSION < FSNAV_INS_GRAVITY_BUS_VERSION_REQUIRED
	#error "fsnav bus version check failed, consider fetching the latest version"
#endif

/* fsnav_ins_gravity_constant - fsnav plugin
	
	Takes the magnitude of average accelerometer output vector 
	and puts it into vertical gravity component, which then remains constant.
	Recommended for low-grade systems, especially when no reference coordinates available.

	description:
		    [   0  ]
		g = [   0  ] = const,
		    [-|<f>|]
		where <f> is the specific force vector as measured by accelerometers
		averaged over an initial alignment period specified in the configuration string
	
	uses:
		fsnav->imu->t
		fsnav->imu->f
		fsnav->imu->f_valid

	changes:
		fsnav->imu->g
		fsnav->imu->g_valid

	cfg parameters:
		{imu: alignment} - imu initial alignment duration, sec
			type :   floating point
			range:   0+ to +inf
			default: 60
			example: {imu: alignment = 300}
*/
void fsnav_ins_gravity_constant(void) {

	const char   t0_token[] = "alignment"; // alignment duration parameter name in configuration
	const double t0_default = 60;          // default alignment duration

	static double
		f[3] = {0,0,0}, // average specific force vector measured by accelerometers
		g3   =  0,      // its norm
		t0   = -1;      // imu initial alignment duration
	static long n = 0;  // number of measurements used so far

	char   *cfg_ptr;    // pointer to a substring in configuration
	double  n1_n;       // (n - 1)/n
	size_t  i;          // index


	// check if imu data has been initialized
	if (fsnav->imu == NULL)
		return;

	if (fsnav->mode == 0) {		// init

		// initial values
		n  =  0;
		t0 = -1;
		// zero average specific force
		for (i = 0; i < 3; i++)
			f[i] = 0;
		// parse alignment duration from configuration string
		cfg_ptr = fsnav_locate_token(t0_token, fsnav->imu->cfg, fsnav->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			t0 = atof(cfg_ptr);		
		// if not found or invalid, set to default
		if (cfg_ptr == NULL || t0 <= 0)
			t0 = t0_default;

	}
	else if (fsnav->mode < 0) {	// terminate
		// do nothing
	}

	else						// main cycle
	{
		// validity flag down
		fsnav->imu->g_valid = 0;

		if (fsnav->imu->t <= t0 && fsnav->imu->f_valid) { // while alignment goes on, and accelerometer measurements are available
			n++;
			n1_n = (n - 1.0)/n;
			for (i = 0; i < 3; i++)
				f[i] = f[i]*n1_n + fsnav->imu->f[i]/n; // averaging components
			g3 = -fsnav_linal_vnorm(f,3);                   // renew magnitude, negative
		}
		// zero Eastern and Northern components
		fsnav->imu->g[0] = 0;
		fsnav->imu->g[1] = 0;
		// vertical component, negative magnitude of average measured specific force vector
		fsnav->imu->g[2] = g3;
		// validity flag up
		fsnav->imu->g_valid = 1;
	}

}

/* fsnav_ins_gravity_normal - fsnav plugin
	
	Computes conventional Earth normal gravity model as in GRS80, etc.,
	but takes Earth model constants from fsnav->imu_const variables.
	Accounts for both latitude and altitude, as well as for plumb line curvature above ellipsoid.
	Recommended for conventional navigation grade systems.

	description:
		if fsnav->imu->sol.llh coordinates are valid, takes them as reference point to calculate gravity at
		else if fsnav->sol (hybrid solution) coordinates are valid, uses them
		otherwise takes latitude of pi/4 and zero altitude as reference point

		g[0] = gE = 0
		g[1] = gN = -fg*sin(2*lat)*h/a
		g[2] = gU = -ge*[1 + fg*sin(lat)^2 - f4/4*sin(2*lat)^2]*[1 - 2*(1 + f*cos(2*lat) + m)*h/a]

		where

		- lat   is  geographical latitude
		- h     is  geographical altitude above Earth ellipsoid
		- a     is  Earth ellipsoid semimajor axis
		- fg    is  Earth gravity flattening
		- ge    is  Earth gravity at equator
		- f     is  Earth ellipsoid flattening
		- m, f4 are Earth gravity model auxiliary constants
	
	uses:
		fsnav->imu->sol.llh
		fsnav->imu->sol.llh_valid
		fsnav->sol.llh
		fsnav->sol.llh_valid

	changes:
		fsnav->imu->g
		fsnav->imu->g_valid

	cfg parameters:
		none
*/
void fsnav_ins_gravity_normal(void) {

	static double
		f    = 0, // Earth ellipsoid flattening f = (a - b)/a
		m    = 0, // gravitational parameter, m = [u^2 a^2 b]/[GM], ratio between centrifugal and gravitational accelerations on the equator of a shpere having the same mass and volume as the Earth does
		f4_4 = 0; // coefficient for the second harmonic term, f4/4 = 5/2 f m - 1/2 f^2

	double 
		lat,	  // geographical latitude		
		h,		  // geographical altitude from reference ellipsoid
		sinlat,	  //   sine of latitude
		sin2lat,  //   sine of twofold latitude
		cos2lat,  // cosine of twofold latitude
		h_a,	  // ratio between altitude and ellipsoid semimajor axis
		b_a;	  // ratio between Earth ellipsoid semiminor and semimajor axes, b/a = sqrt(1 - e^2)


	// check if imu data has been initialized
	if (fsnav->imu == NULL)
		return;

	if (fsnav->mode == 0) {		// init
		
		// ratio between Earth ellipsoid semiminor and semimajor axes
		b_a = sqrt(1 - fsnav->imu_const.e2); // b/a = sqrt(1 - e^2)
		// Earth ellipsoid flattening 
		f = 1 - b_a; // as from definitions: e^2 = (a^2 - b^2)/a^2, f = (a - b)/a
		// gravitational parameter, as derived from section 3 of Geodetic Reference System 80 by H. Moritz (GRS-80): 
			// from ge = GM/ab (1 - m - m/6 e'q0'/q0), 
			// and  gp = GM/a^2 (1 + m/3 e'q0'/q0), 
			// and  f + fg = u^2 b / ge (1 + e'/2 q0'/q0)
			// let  k   = 1/2 e'q0'/q0 = (f + fg)/(u^2 b) ge - 1, 
			// let  eps = 1 / (gp/ge*a/b - 1) = 1/((fg + 1)/sqrt(1 - e^2) - 1), 
			// then m   = 1/(k*(eps + 1/3) + eps + 1)
		f4_4 = (f + fsnav->imu_const.fg)/(fsnav->imu_const.u*fsnav->imu_const.u*fsnav->imu_const.a*b_a)*fsnav->imu_const.ge - 1; // use f4_4 variable instead of k
		m    = 1/((fsnav->imu_const.fg + 1)/b_a - 1);
		m    = 1/(f4_4*(m+1.0/3) + m + 1);
		// coefficient for the second harmonic term
		f4_4 = f/8*(5*m - f); // f4 = -1/2 f^2 + 5/2 f m, as in (2-115), Physical Geodesy, W.Heiskanen H.Moritz, 1993, p. 76, or section 3 of Geodetic Reference System 80 by H. Moritz (GRS-80), corrected for skipped minus sign

	}

	else if (fsnav->mode < 0) {	// terminate
		// do nothing
	}

	else						// main cycle
	{
		// validity flag down
		fsnav->imu->g_valid = 0;
		// define latitude and altitude
		if (fsnav->imu->sol.llh_valid) { // from imu data, if valid
			lat = fsnav->imu->sol.llh[1];
			h   = fsnav->imu->sol.llh[2];
		}
		else if (fsnav->sol.llh_valid) { // from hybrid solution, if valid
			lat = fsnav->sol.llh[1];
			h   = fsnav->sol.llh[2];
		}
		else {                          // default values
			lat = fsnav->imu_const.pi/4; 
			h   = 0;
		}
		sinlat  = sin(  lat);
		sin2lat = sin(2*lat);
		cos2lat = cos(2*lat);
		h_a = h/fsnav->imu_const.a;
		// Eastern component - zero for normal gravity
		fsnav->imu->g[0] = 0;
		// Northern deflection with altitude above ellipsoid from plumb line curvature, as in (5-34), Physical Geodesy, W.Heiskanen H.Moritz, 1993, p. 196
		fsnav->imu->g[1] = -fsnav->imu_const.fg*sin2lat*h_a;
		// vertical component, as from section 3 of Geodetic Reference System 80 by H. Moritz (GRS-80)
		fsnav->imu->g[2] = -fsnav->imu_const.ge*
			(1 + fsnav->imu_const.fg*sinlat*sinlat - f4_4*sin2lat*sin2lat)*
			(1 - 2*(1 + f*cos2lat + m)*h_a);
		// validity flag up
		fsnav->imu->g_valid = 1;

	}

}
