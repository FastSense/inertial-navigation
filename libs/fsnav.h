// Nov-2020
// FSNAV core header file

#ifndef FSNAV_H_
#define FSNAV_H_

#include <stddef.h>

// FSNAV core declarations
#define FSNAV_BUS_VERSION 11 // current bus version





// Julian-type time epoch
typedef struct {
	int    Y; // Year
	int    M; // Month
	int    D; // Day
	int    h; // hour
	int    m; // minute
	double s; // seconds
} fsnav_time_epoch;





// navigation solution structure
typedef struct {
	double  x[3];          // cartesian coordinates, meters
	char    x_valid;       // validity flag (0/1), or a number of valid measurements used
	double  x_std;         // coordinate RMS ("standard") deviation estimate, meters
						   
	double  llh[3];        // geodetic coordinates: longitude (rad), latitude (rad), height (meters)
	char    llh_valid;     // validity flag (0/1), or a number of valid measurements used
						   
	double  v[3];          // relative-to-Earth velocity vector coordinates in local-level geodetic cartesian frame, meters per second
	char    v_valid;       // validity flag (0/1), or a number of valid measurements used
	double  v_std;         // velocity RMS ("standard") deviation estimate, meters per second
						   
	double  q[4];          // attitude quaternion, relative to local-level geodetic cartesian frame
	char    q_valid;       // validity flag (0/1), or a number of valid measurements used
						   
	double  L[9];          // attitude matrix for the transition from local-level geodetic cartesian frame, row-wise: L[0] = L_11, L[1] = L_12, ..., L[8] = L[33]
	char    L_valid;       // validity flag (0/1), or a number of valid measurements used
						   
	double  rpy[3];        // attitude angles relative to local-level geodetic cartesian frame: roll (rad), pitch (rad), yaw (rad)
	char    rpy_valid;     // validity flag (0/1), or a number of valid measurements used
						   
	double  dt;            // clock bias
	char    dt_valid;      // validity flag (0/1), or a number of valid measurements used
						   
	double* metrics;       // application-specific solution metrics
	size_t  metrics_count; // number of application-specific metrics, given in cfg ("metrics_count = ..."), 2 by default, 255 max
} fsnav_sol;





// IMU
	// inertial navigation constants
typedef struct {
	double
		pi,      // pi
		rad2deg, // 180/pi
		// Earth parameters as in GRS-80 by H. Moritz, Journal of Geodesy (2000) 74 (1): pp. 128-162
		u,       // Earth rotation rate, rad/s
		a,       // Earth ellipsoid semi-major axis, m
		e2,      // Earth ellipsoid first eccentricity squared
		ge,      // Earth normal gravity at the equator, m/s^2
		fg;      // Earth normal gravity flattening
} fsnav_imu_const;

	// inertial measurement unit
typedef struct {
	char*  cfg;       // pointer to IMU configuration substring
	size_t cfglength; // IMU configuration substring length

	double t;         // measurement update time (as per IMU clock), used to calculate time step when needed

	double w[3];      // up to 3 gyroscope measurements
	char   w_valid;   // validity flag (0/1), or a number of valid components

	double f[3];      // up to 3 accelerometer measurements
	char   f_valid;   // validity flag (0/1), or a number of valid components

	double Tw[3];      // temperature of gyroscopes 
	char   Tw_valid;   // validity flag (0/1), or a number of valid components

	double Tf[3];      // temperature of accelerometers
	char   Tf_valid;   // validity flag (0/1), or a number of valid components

	double W[3];      // angular velocity of the local level reference frame
	char   W_valid;   // validity flag (0/1), or a number of valid components

	double g[3];      // current gravity acceleration vector
	char   g_valid;   // validity flag (0/1), or a number of valid components

	fsnav_sol sol;     // inertial solution
} fsnav_imu;





// GNSS
	// GNSS satellite data
typedef struct {
	double* eph;         // array of satellite ephemeris as defined by RINEX format (starting with toc: year, month, day, hour, min, sec, clock bias, etc., system-dependent)
	char    eph_valid;   // validity flag (0/1)
	long    eph_counter; // counter or bitfield used to indicate whether all navigation subframes/ephemeris are collected, if needed;

	double  Deltatsv;    // SV PRN code phase time offset (seconds), SV slock correction term to be subtracted: 
	                     // GPS as in Section 20.3.3.3.3.1 of IS-GPS-200J (22 May 2018) p. 96
	                     // GLONASS as in Section 3.3.3 of ICD GLONASS Edition 5.1 2008, minus sign, tau_c if present in fsnav_gnss_glo.clock_corr[0]

	double  t_em;        // time of signal emission
	char    t_em_valid;  // validity flag (0/1)
	double  x[3];        // satellite coordinates
	char    x_valid;     // validity flag (0/1)
	double  v[3];        // satellite velocity vector
	char    v_valid;     // validity flag (0/1)

	double  sinEl;       // sine of satellite elevation angle
	char    sinEl_valid; // validity flag (0/1)

	double* obs;         // satellite observables array, defined at runtime
	char*   obs_valid;   // satellite observables validity flag array (0/1)
} fsnav_gnss_sat;

	// GPS system constants
typedef struct {
	double
		mu,     // Earth grav constant as in GPS interface specs, m^3/s^2
		u,      // Earth rotation rate as in GPS interface specs, rad/s
		a,      // Earth ellipsoid semi-major axis, m
		e2,     // Earth ellipsoid first eccentricity squared
		F,      // relativistic correction constant as in GPS interface specs, sec/sqrt(m)
		F1, L1, // nominal frequency and wavelength for L1 signal as in GPS interface specs, Hz and m
		F2, L2; // nominal frequency and wavelength for L2 signal as in GPS interface specs, Hz and m
} fsnav_gps_const;

	// GLONASS system constants
typedef struct {
	double
		mu,       // Earth grav constant as in GLONASS ICD, m^3/s^2
		J02,      // second zonal harmonic of geopotential
		u,        // Earth rotation rate as in GLONASS ICD, rad/s
		a,        // Earth ellipsoid semi-major axis, m
		e2,       // Earth ellipsoid first eccentricity squared as in GLONASS ICD
		F01, dF1, // nominal centre frequency and channel separation for L1 signal as in GLONASS ICD, Hz
		F02, dF2; // nominal centre frequency and channel wavelength for L2 signal as in GLONASS ICD, Hz
} fsnav_glo_const;

	// Galileo system constants
typedef struct {
	double
		mu,       // Earth grav constant as in Galileo interface specs, m^3/s^2
		u,        // Earth rotation rate as in Galileo interface specs, rad/s
		a,        // Earth ellipsoid semi-major axis, m
		e2,       // Earth ellipsoid first eccentricity squared
		F,        // relativistic correction constant as in Galileo interface specs, sec/sqrt(m)
		F1, L1,   // nominal frequency and wavelength for E1 signal as in Galileo interface specs, Hz and m
		F5a, L5a, // nominal frequency and wavelength for E5a signal as in Galileo interface specs, Hz and m
		F5b, L5b, // nominal frequency and wavelength for E5b signal as in Galileo interface specs, Hz and m
		F6, L6;   // nominal frequency and wavelength for E6 signal as in Galileo interface specs, Hz and m
} fsnav_gal_const;

	// BeiDou system constants
typedef struct {
	double
		mu,       // Earth grav constant as in BeiDou interface specs, m^3/s^2
		u,        // Earth rotation rate as in BeiDou interface specs, rad/s
		a,        // Earth ellipsoid semi-major axis as in CGCS2000, m
		e2,       // Earth ellipsoid first eccentricity squared, as in CGCS2000
		F,        // relativistic correction constant as in BeiDou interface specs, sec/sqrt(m)
		leap_sec, // leap seconds between BeiDou time and GPS time as of 01-Jan-2006
		B1, L1,   // nominal frequency and wavelength for B1 signal as in BeiDou interface specs, Hz and m
		B2, L2;   // nominal frequency and wavelength for B2 signal as in BeiDou interface specs, Hz and m
} fsnav_bds_const;

	// GNSS constants
typedef struct {
	double
		pi,             // circumference-to-diameter ratio
		c,              // speed of light, m/s
		sec_in_w,       // seconds in a week
		sec_in_d;       // seconds in a day
	// constellation-specific constants
	fsnav_gps_const gps; // GPS constants
	fsnav_glo_const glo; // GLONASS constants
	fsnav_gal_const gal; // Galileo constants
	fsnav_bds_const bds; // BeiDou constants
} fsnav_gnss_const;

	// GPS constellation data
typedef struct {
	char*          cfg;              // GPS configuration substring
	size_t         cfglength;        // GPS configuration substring length

	size_t         max_sat_count;    // maximum supported number of satellites
	size_t         max_eph_count;    // maximum supported number of ephemeris

	fsnav_gnss_sat* sat;              // GPS satellites
	char           (*obs_types)[4];  // observation types according to RINEX: C1C, etc.; an array of 3-character null-terminated strings in the same order as in satellites
	size_t         obs_count;        // number of observation types

	double         iono_a[4];        // ionospheric model parameters from GPS almanac
	double         iono_b[4];
	char           iono_valid;       // validity flag (0/1)

	double         clock_corr[4];    // clock correction parameters from GPS almanac: e.g. a0, a1, gps_second, gps_week for GPS to UTC, optional
	char           clock_corr_to[2]; // time system, which the correction results into: GP - GPS, UT - UTC, GA - Galileo, etc.
	char           clock_corr_valid; // validity flag (0/1)
} fsnav_gnss_gps;

	// GLONASS constellation data
typedef struct {
	char*          cfg;              // GLONASS configuration substring
	size_t         cfglength;        // GLONASS configuration substring length

	size_t         max_sat_count;    // maximum supported number of satellites
	size_t         max_eph_count;    // maximum supported number of ephemeris

	fsnav_gnss_sat* sat;              // GLONASS satellites
	int*           freq_slot;        // frequency numbers
	char           (*obs_types)[4];  // observation types according to RINEX: C1C, etc.; an array of 3-character null-terminated strings in the same order as in satellites
	size_t         obs_count;        // number of observation types

	double         clock_corr[4];    // clock correction parameters from GLONASS almanac: e.g. -tauC, zero, Na_day_number, N4_four_year_interval for GLONASS to UTC, optional
	char           clock_corr_to[2]; // time system, which the correction results into: GP - GPS, UT - UTC, GA - Galileo, etc.
	char           clock_corr_valid; // validity flag (0/1)
} fsnav_gnss_glo;

	// Galileo constellation data
typedef struct {
	char*          cfg;              // Galileo configuration substring
	size_t         cfglength;        // Galileo configuration substring length

	size_t         max_sat_count;    // maximum supported number of satellites
	size_t         max_eph_count;    // maximum supported number of ephemeris

	fsnav_gnss_sat* sat;              // Galileo satellites
	char           (*obs_types)[4];  // observation types according to RINEX: C1C, etc.; an array of 3-character null-terminated strings in the same order as in satellites
	size_t         obs_count;        // number of observation types

	double         iono[3];          // ionospheric model parameters from Galileo almanac
	char           iono_valid;       // validity flag (0/1)

	double         clock_corr[4];    // clock correction parameters from Galileo almanac: e.g. a0, a1, gal_second, gal_week for GAL to UTC, optional
	char           clock_corr_to[2]; // time system, which the correction results into: GP - GPS, UT - UTC, GA - Galileo, etc.
	char           clock_corr_valid; // validity flag (0/1)
} fsnav_gnss_gal;

	// BeiDou constellation data
typedef struct {
	char*          cfg;              // BeiDou configuration substring
	size_t         cfglength;        // BeiDouconfiguration substring length

	size_t         max_sat_count;    // maximum supported number of satellites
	size_t         max_eph_count;    // maximum supported number of ephemeris

	fsnav_gnss_sat* sat;              // BeiDou satellites
	char           (*obs_types)[4];  // observation types according to RINEX: C1C, etc.; an array of 3-character null-terminated strings in the same order as in satellites
	size_t         obs_count;        // number of observation types

	double         iono_a[4];        // ionospheric model parameters from BeiDou almanac
	double         iono_b[4];
	char           iono_valid;       // validity flag (0/1)

	double         clock_corr[4];    // clock correction parameters from BeiDou almanac: e.g. a0, a1, bds_second, bds_week for BDS to UTC, optional
	char           clock_corr_to[2]; // time system, which the correction results into: GP - GPS, UT - UTC, GA - Galileo, etc.
	char           clock_corr_valid; // validity flag (0/1)
} fsnav_gnss_bds;

	// GNSS operation settings
typedef struct {
	double sinEl_mask;    // elevation angle mask, sine of

	double code_sigma;    // pseudorange measurement rmsdev (sigma), meters
	double phase_sigma;   // carrier phase measurement rmsdev (sigma), cycles
	double doppler_sigma; // doppler measurement rmsdev (sigma), Hz

	double ant_pos[3];    // antenna coordinates in the instrumental frame
	double ant_pos_tol;   // antenna position tolerance (-1 if undefined)

	double leap_sec_def;  // default value of leap seconds ( <= 0 if undefined)
} fsnav_gnss_settings;

	// GNSS data
typedef struct {
	char*              cfg;             // full GNSS configuration substring pointer, NULL if gnss is not used
	size_t             cfglength;       // full GNSS configuration substring length

	char*              cfg_settings;    // pointer to a part of GNSS configuration substring common to all systems
	size_t             settings_length; // length of the part of GNSS configuration substring common to all systems

	fsnav_gnss_settings settings;        // GNSS operation settings

	fsnav_gnss_gps*     gps;             // GPS constellation data pointer
	fsnav_gnss_glo*     glo;             // GLONASS constellation data pointer
	fsnav_gnss_gal*     gal;             // Galileo constellation data pointer
	fsnav_gnss_bds*     bds;             // BeiDou constellation data pointer

	fsnav_time_epoch    epoch;           // current GNSS time epoch
	int                leap_sec;        // current number of leap seconds (for UTC by default, but may also be used for BDS leap second for BDS-only processing)
	char               leap_sec_valid;  // validity flag (0/1)

	fsnav_sol           sol;             // current GNSS solution
} fsnav_gnss;





// air data
typedef struct {
	char*  cfg;         // pointer to air data configuration substring
	size_t cfglength;   // air data configuration substring length

	double t;           // measurement update time (as per air data computer clock)

	double alt;         // barometric altitude
	double alt_std;     // estimated standard deviation, <0 if undefined
	char   alt_valid;   // validity flag (0/1)

	double vv;          // vertical velocity (vertical speed/rate of climb and descent)
	double vv_std;      // estimated standard deviation, <0 if undefined
	char   vv_valid;    // validity flag (0/1)

	double speed;       // airspeed
	double speed_std;   // estimated standard deviation, <0 if undefined
	char   speed_valid; // validity flag (0/1)
} fsnav_air;





// reference data
typedef struct {
	char*  cfg;       // pointer to reference data configuration substring
	size_t cfglength; // reference data configuration substring length

	double t;         // reference time

	double g[3];      // reference gravity acceleration vector
	char   g_valid;   // validity flag (0/1), or a number of valid components

	fsnav_sol sol;     // reference data
} fsnav_ref;





// BUS
	// scheduled plugin structure
typedef struct {
	void(*func)(void); // pointer to plugin function to execute
	int cycle;         // tick cycle (period) to execute
	int shift;         // tick within a cycle to execute at (shift)
	int tick;          // current tick
} fsnav_plugin;

	// core structure
typedef struct {
	fsnav_plugin* plugins;           // plugin array pointer
	size_t       plugin_count;      // number of plugins
	size_t       current_plugin_id; // current plugin in plugin execution list
	size_t       exit_plugin_id;    // index of a plugin that initiated termination, or UINT_MAX by default
	char         host_termination;  // identifier of termination being called by host
} fsnav_core;

	// bus data to be used in host application
typedef struct {
	// bus version to be used at runtime
	int ver;

	// main functions to be used in host app
		// basic
	char(*add_plugin)(void(*func)(void));                                 // add plugin to the plugin execution list,                             input: pointer to plugin function,                       output: OK/not OK (1/0)
	char(*init)      (char* cfg        );                                 // initialize the bus, except for core,                                 input: configuration string (see description),           output: OK/not OK (1/0)
	char(*step)      (void             );                                 // step through the plugin execution list,                                                                                       output: OK/not OK (1/0)
	char(*terminate) (void             );                                 // terminate operation,                                                                                                          output: OK/not OK (1/0)
		
		// advanced scheduling
	char(*remove_plugin)    (void(*func   )(void)                      ); // remove all instances of a plugin from the plugin execution list,     input: pointer to plugin function to be removed,         output: OK/not OK (1/0)
	char(*replace_plugin)   (void(*oldfunc)(void), void(*newfunc)(void)); // replace all instances of the plugin by another one,                  input: pointers to old and new plugin functions,         output: OK/not OK (1/0)
	char(*schedule_plugin)  (void(*func   )(void), int cycle, int shift); // add scheduled plugin to the plugin execution list,                   input: pointer to plugin function, cycle, shift,         output: OK/not OK (1/0)
	char(*reschedule_plugin)(void(*func   )(void), int cycle, int shift); // reschedule all instances of the plugin in the plugin execution list, input: pointer to plugin function, new cycle, new shift, output: OK/not OK (1/0)
	char(*suspend_plugin)   (void(*func   )(void)                      ); // suspend all instances of the plugin in the plugin execution list,    input: pointer to plugin function,                       output: OK/not OK (1/0)
	char(*resume_plugin)    (void(*func   )(void)                      ); // resume all instances of the plugin in the plugin execution list,     input: pointer to plugin function,                       output: OK/not OK (1/0)
	
	fsnav_core       core;            // core instances

	char*           cfg;             // full configuration string
	size_t          cfglength;       // full configuration string length

	char*           cfg_settings;    // pointer to a part of the configuration string common to all subsystems
	size_t          settings_length; // length of the part of the configuration string common to all subsystems

	fsnav_imu_const  imu_const;       // inertial navigation constants, initialized independent of imu structure
	fsnav_imu*       imu;             // inertial measurement unit data pointer

	fsnav_gnss_const gnss_const;      // global navigation satellite system constants, initialized independent of gnss structure
	fsnav_gnss*      gnss;            // global navigation satellite system data pointer
	size_t          gnss_count;      // number of gnss instances

	fsnav_air*       air;             // air data subsystem pointer
	fsnav_ref*       ref;             // reference data subsystem pointer

	double          t;               // system time (as per main clock for integrated systems)
	int             mode;            // operation mode: 0 - init, <0 termination, >0 normal operation
	fsnav_sol        sol;             // navigation solution (hybrid/integrated, etc.)
} fsnav_struct;

extern fsnav_struct* fsnav;





// basic parsing
char* fsnav_locate_token(const char* token, char* src, const size_t len, const char delim); // locate a token (and delimiter, when given) within a configuration string





// time routines
int  fsnav_time_epochs_compare    (fsnav_time_epoch* date1, fsnav_time_epoch* date2         ); // compare time epochs: +1 if date1 is later than date2, 0 if equal (within 1/32768 sec, half-precision compliant), -1 otherwise
long fsnav_time_days_between_dates(fsnav_time_epoch epoch_from, fsnav_time_epoch epoch_to   ); // calculate days elapsed from one date to another, based on Rata Die serial date from day one on 0001/01/01
char fsnav_time_gps2epoch         (fsnav_time_epoch* epoch, unsigned int week, double sec  ); // convert GPS week and seconds to GPS Gregorian date/time (DOES NOT include leap seconds)
char fsnav_time_epoch2gps         (unsigned int* week, double* sec, fsnav_time_epoch* epoch); // convert GPS Gregorian date/time to GPS week and seconds (DOES NOT include leap seconds)





// linear algebra functions
	// conventional operations
double fsnav_linal_dot     (double* u, double* v, const size_t m                                              ); // calculate dot product
double fsnav_linal_vnorm   (double* u, const size_t m                                                         ); // calculate l_2 vector norm, i.e. sqrt(u^T*u)
void   fsnav_linal_cross3x1(double* res, double* u, double* v                                                 ); // calculate cross product for 3x1 vectors
void   fsnav_linal_mmul    (double* res, double* a, double* b, const size_t n, const size_t n1, const size_t m); // multiply two matrices:                        res = a*b,   where a is n x n1, b is n1 x m, res is n x m
void   fsnav_linal_mmul1T  (double* res, double* a, double* b, const size_t n, const size_t m, const size_t n1); // multiply two matrices (first is transposed):  res = a^T*b, where a is n x m,  b is n x n1, res is m x n1
void   fsnav_linal_mmul2T  (double* res, double* a, double* b, const size_t n, const size_t m, const size_t n1); // multiply two matrices (second is transposed): res = a*b^T, where a is n x m,  b is n1 x m, res is n x n1
void   fsnav_linal_qmul    (double* res, double* q, double* r                                                 ); // multiply 4x1 quaternions:                     res = q x r, with res0, q0, r0 being scalar parts
	
	// space rotation representation
void fsnav_linal_mat2quat(double* q, double* R  ); // calculate quaternion q (with q0 being scalar part) corresponding to 3x3 attitude matrix R
void fsnav_linal_quat2mat(double* R, double* q  ); // calculate 3x3 attitude matrix R corresponding to quaternion q (with q0 being scalar part)
void fsnav_linal_rpy2mat (double* R, double* rpy); // calculate 3x3 transition matrix R from E-N-U corresponding to roll, pitch and yaw (radians, airborne frame: X longitudinal, Z right-wing)
void fsnav_linal_mat2rpy (double* rpy, double* R); // calculate roll, pitch and yaw (radians, airborne frame: X longitudinal, Z right-wing) corresponding to 3x3 transition matrix R from E-N-U
void fsnav_linal_eul2mat (double* R, double* e  ); // calculate 3x3 rotation matrix R for 3x1 Euler vector e via Rodrigues' formula: R = E + sin|e|/|e|*[e,] + (1-cos|e|)/|e|^2*[e,]^2 
	
	// routines for m x m upper-triangular matrices U lined up in a single-dimension array u
		// index conversion
void fsnav_linal_u_ij2k(size_t* k, const size_t i, const size_t j, const size_t m); // convert index for upper-triangular matrix lined up in a single-dimension array: (i,j) ->  k
void fsnav_linal_u_k2ij(size_t* i, size_t* j, const size_t k, const size_t m     ); // convert index for upper-triangular matrix lined up in a single-dimension array:  k    -> (i,j)
		
		// conventional matrix operations
void fsnav_linal_u_mul   (double* res, double* u, double* v, const size_t n, const size_t m); // multiply upper-triangular matrix lined up in a single-dimension array by a regular matrix:                             res = U*v
void fsnav_linal_uT_mul_v(double* res, double* u, double* v, const size_t m                ); // multiply upper-triangular matrix lined up in a single-dimension array of m(m+1)/2 x 1 (transposed) by vector:          res = U^T*v
void fsnav_linal_u_inv   (double* res, double* u, const size_t m                           ); // invert upper-triangular matrix lined up in a single-dimension array of m(m+1)/2 x 1:                                   res = U^-1
void fsnav_linal_uuT     (double* res, double* u, const size_t m                           ); // calculate square (with transposition) of upper-triangular matrix lined up in a single-dimension array of m(m+1)/2 x 1: res = U*U^T
	
	// matrix factorizations
void fsnav_linal_chol(double* S, double* P, const size_t m); // calculate Cholesky upper-triangular factorization P = S*S^T, where P is symmetric positive-definite matrix

	// square root Kalman filtering
char   fsnav_linal_check_measurement_residual(double* x, double* S, double z, double* h, double sigma, double k_sigma, const size_t n); // check measurement residual magnitude against predicted covariance level
double fsnav_linal_kalman_update             (double* x, double* S, double* K, double z, double* h, double sigma, const size_t n     ); // perform square root Kalman filter update     phase
void   fsnav_linal_kalman_predict_I_qI       (double* S, double  q2, const size_t n                                                  ); // perform square root Kalman filter prediction phase: identity         state transition, scalar         process noise covariance
void   fsnav_linal_kalman_predict_I_qIr      (double* S, double  q2, const size_t n, const size_t m                                  ); // perform square root Kalman filter prediction phase: identity         state transition, reduced scalar process noise covariance
void   fsnav_linal_kalman_predict_I_diag     (double* S, double* q2, const size_t n, const size_t m                                  ); // perform square root Kalman filter prediction phase: identity         state transition, diagonal       process noise covariance
void   fsnav_linal_kalman_predict_I          (double* S, double* Q, const size_t n, const size_t m                                   ); // perform square root Kalman filter prediction phase: identity         state transition
void   fsnav_linal_kalman_predict_U_diag     (double* x, double* S, double* U, double* q2, const size_t n, const size_t m            ); // perform square root Kalman filter prediction phase: upper triangular state transition, diagonal       process noise covariance
void   fsnav_linal_kalman_predict_U          (double* x, double* S, double* U, double* Q, const size_t n, const size_t m             ); // perform square root Kalman filter prediction phase: upper triangular state transition

#endif // FSNAV_H_
