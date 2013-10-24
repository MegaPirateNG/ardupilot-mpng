// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_Sprayer.h
/// @brief	Crop sprayer library

/**
    The crop spraying functionality can be enabled in ArduCopter by doing the following:
        1. add this line to APM_Config.h
            #define SPRAYER ENABLED
        2. set CH7_OPT or CH8_OPT parameter to 15 to allow turning the sprayer on/off from one of these channels
        3. set RC10_FUNCTION to 22 to enable the servo output controlling the pump speed on RC10
        4. set RC11_FUNCTION to 23 to enable the spervo output controlling the spinner on RC11
        5. ensure the RC10_MIN, RC10_MAX, RC11_MIN, RC11_MAX accurately hold the min and maximum servo values you could possibly output to the pump and spinner
        6. set the SPRAY_SPINNER to the pwm value the spinner should spin at when on
        7. set the SPRAY_PUMP_RATE to the value the pump should servo should move to when the vehicle is travelling 1m/s expressed as a percentage (i.e. 0 ~ 100) of the full servo range.  I.e. 0 = the pump will not operate, 100 = maximum speed at 1m/s.  50 = 1/2 speed at 1m/s, full speed at 2m/s
        8. set the SPRAY_PUMP_MIN to the minimum value that the pump servo should move to while engaged expressed as a percentage (i.e. 0 ~ 100) of the full servo range
        9. set the SPRAY_SPEED_MIN to the minimum speed (in cm/s) the vehicle should be moving at before the pump and sprayer are turned on.  0 will mean the pump and spinner will always be on when the system is enabled with ch7/ch8 switch
**/

#ifndef AC_SPRAYER_H
#define AC_SPRAYER_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_InertialNav.h>     // Inertial Navigation library

#define AC_SPRAYER_DEFAULT_PUMP_RATE        10       // default quantity of spray per meter travelled
#define AC_SPRAYER_DEFAULT_PUMP_MIN         0       // default minimum pump speed expressed as a percentage from 0 to 100
#define AC_SPRAYER_DEFAULT_SPINNER_PWM      1300    // default speed of spinner (higher means spray is throw further horizontally
#define AC_SPRAYER_DEFAULT_SPEED_MIN        100     // we must be travelling at least 1m/s to begin spraying
#define AC_SPRAYER_DEFAULT_TURN_ON_DELAY    100     // delay between when we reach the minimum speed and we begin spraying.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   1000    // shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class AC_Sprayer {

public:

    /// Constructor
    AC_Sprayer(AP_InertialNav* inav);

    /// enable - allows sprayer to be enabled/disabled.  Note: this does not update the eeprom saved value
    void enable(bool true_false);

    /// enabled - returns true if fence is enabled
    bool enabled() const { return _enabled; }

    /// test_pump - set to true to turn on pump as if travelling at 1m/s as a test
    void test_pump(bool true_false) { _flags.testing = true_false; }

    /// To-Do: add function to decode pilot input from channel 6 tuning knob

    /// set_pump_rate - sets desired quantity of spray when travelling at 1m/s as a percentage of the pumps maximum rate
    void set_pump_rate(float pct_at_1ms) { _pump_pct_1ms.set(pct_at_1ms); }

    /// update - adjusts servo positions based on speed and requested quantity
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    // pointers to other objects we depend upon
    AP_InertialNav* _inav;

    // parameters
    AP_Int8         _enabled;               // top level enable/disable control
    AP_Float        _pump_pct_1ms;          // desired pump rate (expressed as a percentage of top rate) when travelling at 1m/s
    AP_Int8         _pump_min_pct;          // minimum pump rate (expressed as a percentage from 0 to 100)
    AP_Int16        _spinner_pwm;           // pwm rate of spinner
    AP_Float        _speed_min;             // minimum speed in cm/s above which the sprayer will be started

    // flag bitmask
    struct sprayer_flags_type {
        uint8_t spraying    : 1;            // 1 if we are currently spraying
        uint8_t testing     : 1;            // 1 if we are testing the sprayer and should output a minimum value
    } _flags;

    // internal variables
    uint32_t        _speed_over_min_time;   // time at which we reached speed minimum
    uint32_t        _speed_under_min_time;  // time at which we fell below speed minimum
};

#endif /* AC_SPRAYER_H */
