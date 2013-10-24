/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_AIRSPEED_H__
#define __AP_AIRSPEED_H__

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <AP_Airspeed_Backend.h>
#include <AP_Airspeed_analog.h>
#include <AP_Airspeed_PX4.h>
#include <AP_Airspeed_I2C.h>

class Airspeed_Calibration {
public:
    friend class AP_Airspeed;
    // constructor
    Airspeed_Calibration(const AP_Vehicle::FixedWing &parms);

    // initialise the calibration
    void init(float initial_ratio);

    // take current airspeed in m/s and ground speed vector and return
    // new scaling factor
    float update(float airspeed, const Vector3f &vg);

private:
    // state of kalman filter for airspeed ratio estimation
    Matrix3f P; // covarience matrix
    const float Q0; // process noise matrix top left and middle element
    const float Q1; // process noise matrix bottom right element
    Vector3f state; // state vector
    const float DT; // time delta
    const AP_Vehicle::FixedWing &aparm;
};

class AP_Airspeed
{
public:
    // constructor
    AP_Airspeed(const AP_Vehicle::FixedWing &parms) : 
        _EAS2TAS(1.0f),
        _healthy(false),
        _calibration(parms),
        analog(_pin)
    {
		AP_Param::setup_object_defaults(this, var_info);
    };

    void init(void);

    // read the analog source and update _airspeed
    void        read(void);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void            calibrate();

    // return the current airspeed in m/s
    float           get_airspeed(void) const {
        return _airspeed;
    }

    // return the unfiltered airspeed in m/s
    float           get_raw_airspeed(void) const {
        return _raw_airspeed;
    }

    // return the current airspeed in cm/s
    float        get_airspeed_cm(void) const {
        return _airspeed*100;
    }

    // return the current airspeed ratio (dimensionless)
    float        get_airspeed_ratio(void) const {
        return _ratio;
    }

    // set the airspeed ratio (dimensionless)
    void        set_airspeed_ratio(float ratio) {
        _ratio.set(ratio);
    }

    // return true if airspeed is enabled, and airspeed use is set
    bool        use(void) const {
        return _enable && _use && _offset != 0 && _healthy;
    }

    // return true if airspeed is enabled
    bool        enabled(void) const {
        return _enable;
    }

    // force disable the airspeed sensor
    void        disable(void) {
        _enable.set(0);
    }

    // used by HIL to set the airspeed
    void        set_HIL(float airspeed) {
        _airspeed = airspeed;
    }

    // return the differential pressure in Pascal for the last
    // airspeed reading. Used by the calibration code
    float get_differential_pressure(void) const {
        return max(_last_pressure - _offset, 0);
    }

    // set the apparent to true airspeed ratio
    void set_EAS2TAS(float v) {
        _EAS2TAS = v;
    }

    // get the apparent to true airspeed ratio
    float get_EAS2TAS(void) const {
        return _EAS2TAS;
    }

    // update airspeed ratio calibration
    void update_calibration(const Vector3f &vground);

	// log data to MAVLink
	void log_mavlink_send(mavlink_channel_t chan, const Vector3f &vground);

    static const struct AP_Param::GroupInfo var_info[];


private:
    AP_Float        _offset;
    AP_Float        _ratio;
    AP_Int8         _use;
    AP_Int8         _enable;
    AP_Int8         _pin;
    AP_Int8         _autocal;
    float           _raw_airspeed;
    float           _airspeed;
    float			_last_pressure;
    float           _EAS2TAS;
    bool		    _healthy;

    Airspeed_Calibration _calibration;
    float _last_saved_ratio;
    uint8_t _counter;

    float get_pressure(void);

    AP_Airspeed_Analog analog;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    AP_Airspeed_PX4    digital;
#else
    AP_Airspeed_I2C    digital;
#endif
};

#endif // __AP_AIRSPEED_H__

