// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AP_TECS.h
/// @brief   Combined Total Energy Speed & Height Control. This is a instance of an
/// AP_SpdHgtControl class

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to 
 *    height priority
 *  - Underspeed protection that demands maximum throttle switches pitch angle control 
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 */

#ifndef AP_TECS_H
#define AP_TECS_H

#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Param.h>
#include <AP_Vehicle.h>
#include <AP_SpdHgtControl.h>
#include <DataFlash.h>

class AP_TECS : public AP_SpdHgtControl {
public:
	AP_TECS(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms) :
		_ahrs(ahrs),
		aparm(parms)
		{
			AP_Param::setup_object_defaults(this, var_info);
		}

	// Update of the estimated height and height rate internal state
	// Update of the inertial speed rate internal state
	// Should be called at 50Hz or greater
	// hgt_afe is the height above field elevation (takeoff height)
	void update_50hz(float hgt_afe);

	// Update the control loop calculations
    void update_pitch_throttle(int32_t hgt_dem_cm, 
                               int32_t EAS_dem_cm, 
                               enum FlightStage flight_stage,
                               int32_t ptchMinCO_cd,
                               int16_t throttle_nudge,
							   float hgt_afe);

	// demanded throttle in percentage
	// should return 0 to 100
	int32_t get_throttle_demand(void) {return int32_t(_throttle_dem * 100.0f);}
	
	// demanded pitch angle in centi-degrees
	// should return between -9000 to +9000
	int32_t get_pitch_demand(void) { return int32_t(_pitch_dem * 5729.5781f);}
	
	// Rate of change of velocity along X body axis in m/s^2
	float get_VXdot(void) { return _vel_dot; }

	// log data on internal state of the controller. Called at 10Hz
	void log_data(DataFlash_Class &dataflash, uint8_t msgid);

	// this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

	struct PACKED log_TECS_Tuning {
		LOG_PACKET_HEADER;
		float hgt;
		float dhgt;
		float hgt_dem;
		float dhgt_dem;
		float spd_dem;
		float spd;
		float dspd;
		float ithr;
		float iptch;
		float thr;
		float ptch;
		float dspd_dem;
	} log_tuning;

private:
    // Last time update_50Hz was called
    uint32_t _update_50hz_last_usec;
	
    // Last time update_speed was called
    uint32_t _update_speed_last_usec;
	
    // Last time update_pitch_throttle was called
    uint32_t _update_pitch_throttle_last_usec;

	// reference to the AHRS object
    AP_AHRS &_ahrs;

	const AP_Vehicle::FixedWing &aparm;

	// TECS tuning parameters
	AP_Float _hgtCompFiltOmega;
    AP_Float _spdCompFiltOmega;
    AP_Float _maxClimbRate;
    AP_Float _minSinkRate;
    AP_Float _maxSinkRate;
    AP_Float _timeConst;
    AP_Float _ptchDamp;
    AP_Float _thrDamp;
    AP_Float _integGain;
    AP_Float _vertAccLim;
	AP_Float _rollComp;
	AP_Float _spdWeight;
	
	// throttle demand in the range from 0.0 to 1.0
    float _throttle_dem;
	
	// pitch angle demand in radians
    float _pitch_dem;
	
	// Integrator state 1 - height filter second derivative
	float _integ1_state;
	
	// Integrator state 2 - height rate
	float _integ2_state;

	// Integrator state 3 - height
	float _integ3_state;

	// Integrator state 4 - airspeed filter first derivative
	float _integ4_state;

	// Integrator state 5 - true airspeed
	float _integ5_state;

	// Integrator state 6 - throttle integrator
	float _integ6_state;

	// Integrator state 6 - pitch integrator
	float _integ7_state;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Rate of change of speed along X axis
	float _vel_dot;

    // Equivalent airspeed
    float _EAS;

    // True airspeed limits
    float _TASmax;
    float _TASmin;

    // Current and last true airspeed demand
    float _TAS_dem;
    float _TAS_dem_last;

    // Equivalent airspeed demand
    float _EAS_dem;

    // height demands
    float _hgt_dem;
    float _hgt_dem_in_old;
    float _hgt_dem_adj;
    float _hgt_dem_adj_last;
    float _hgt_rate_dem;
	float _hgt_dem_prev;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;

	// Total energy rate filter state
	float _STEdotErrLast;

    // Underspeed condition
    bool _underspeed;

    // Bad descent condition caused by unachievable airspeed demand
    bool _badDescent;

    // climbout mode
    enum FlightStage _flight_stage;

	// throttle demand before limiting
	float _throttle_dem_unc;

	// pitch demand before limiting
	float _pitch_dem_unc;

	// Maximum and minimum specific total energy rate limits
	float _STEdot_max;
	float _STEdot_min;

	// Maximum and minimum floating point throttle limits
	float _THRmaxf;
	float _THRminf;

	// Maximum and minimum floating point pitch limits
	float _PITCHmaxf;
	float _PITCHminf;

    // Specific energy quantities
    float _SPE_dem;
    float _SKE_dem;
    float _SPEdot_dem;
    float _SKEdot_dem;
    float _SPE_est;
    float _SKE_est;
    float _SPEdot;
    float _SKEdot;

	// Specific energy error quantities
	float _STE_error;

	// Time since last update of main TECS loop (seconds)
	float _DT;

    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(void);

    // Update the demanded airspeed
    void _update_speed_demand(void);

    // Update the demanded height
    void _update_height_demand(void);

	// Detect an underspeed condition
	void _detect_underspeed(void);

	// Update Specific Energy Quantities
	void _update_energies(void);

	// Update Demanded Throttle
	void _update_throttle(void);

	// Update Demanded Throttle Non-Airspeed
	void _update_throttle_option(int16_t throttle_nudge);

	// Detect Bad Descent
	void _detect_bad_descent(void);

	// Update Demanded Pitch Angle
	void _update_pitch(void);

	// Initialise states and variables
	void _initialise_states(int32_t ptchMinCO_cd, float hgt_afe);

	// Calculate specific total energy rate limits
	void _update_STE_rate_lim(void);

    // declares a 5point average filter using floats
	AverageFilterFloat_Size5 _vdot_filter;
};

#define TECS_LOG_FORMAT(msg) { msg, sizeof(AP_TECS::log_TECS_Tuning),	\
							   "TECS", "ffffffffffff", "h,dh,h_dem,dh_dem,sp_dem,sp,dsp,ith,iph,th,ph,dsp_dem" }

#endif //AP_TECS_H
