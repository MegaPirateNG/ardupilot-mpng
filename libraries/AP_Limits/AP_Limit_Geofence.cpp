// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	limits.cpp
/// @brief	Imposes limits on location (geofence), altitude and other parameters.
/// Each limit breach will trigger an action or set of actions to recover.
/// Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#include "AP_Limit_Geofence.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Limit_Geofence::var_info[] PROGMEM = {
    // @Param: FNC_ON
    // @DisplayName: Enable Geofence
    // @Description: Setting this to Enabled(1) will enable the geofence. Setting this to Disabled(0) will disable the geofence
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("FNC_ON",  0,      AP_Limit_Geofence,      _enabled, 0),

    // @Param: FNC_REQ
    // @DisplayName: Require Geofence
    // @Description: Setting this to Enabled(1) will make being inside the geofence a required check before arming the vehicle.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("FNC_REQ", 1,      AP_Limit_Geofence,      _required, 0),

    // @Param: FNC_SMPL
    // @DisplayName: Require Geofence
    // @Description: "Simple" geofence (enabled - 1) is based on a radius from the home position, "Complex" (disabled - 0) define a complex fence by lat/long positions
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("FNC_SMPL", 2,     AP_Limit_Geofence,      _simple, 0),

    // @Param: FNC_RAD
    // @DisplayName: Require Geofence
    // @Description: Radius of fenced area in meters. A value of 20 creates a 20-meter radius circle (40-meter diameter) from the home point.
    // @Units: Meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FNC_RAD", 3,      AP_Limit_Geofence,      _radius, 0),

    // @Param: FNC_TOT
    // @DisplayName: Total number of geofence points
    // @Description: Total number of geofence points.  This parameter should not be updated manually
    // @Range: 0 6
    // @Increment: 1
    AP_GROUPINFO("FNC_TOT", 4,      AP_Limit_Geofence,      _fence_total, 0),
    AP_GROUPEND


};


AP_Limit_Geofence::AP_Limit_Geofence(uint16_t efs, uint8_t f_wp_s,
        uint8_t max_fp, GPS *&gps, const struct Location *h_loc,
        const struct Location *c_loc) :
    AP_Limit_Module(AP_LIMITS_GEOFENCE),
    _gps(gps),
    _current_loc(c_loc),
    _home(h_loc),
    _eeprom_fence_start(efs),
    _fence_wp_size(f_wp_s),
    _max_fence_points(max_fp),
    _boundary_uptodate(false)
{
    AP_Param::setup_object_defaults(this, var_info);
    update_boundary();
}




bool AP_Limit_Geofence::triggered() {

    // reset trigger before checking
    _triggered = false;

    // never trigger while disabled
    if (!_enabled) return false;

    // if Geofence is required and we don't know where we are, trigger.
    if (_required && (!_gps || !_gps->fix || !_home || !_current_loc)) {

        // TRIGGER
        _triggered = true;
    }

    uint32_t distance;

    // simple mode, pointers to current and home exist.
    if (_simple && _current_loc && _home) {
        distance = (uint32_t) get_distance(*_current_loc, *_home);
        if (distance > 0 &&  distance > (uint16_t) _radius) {

            // TRIGGER
            _triggered = true;
        }
    }
    else {

        // COMPLEX GEOFENCE  mode

        // check boundary and update if necessary
        if (!_boundary_uptodate) {
            update_boundary();
        }

        // if boundary is correct, and current_loc exists check if we
        // are inside the fence.
        if (boundary_correct() && _current_loc) {
            Vector2l location;
            location.x = _current_loc->lat;
            location.y = _current_loc->lng;
            // trigger if outside
            if (Polygon_outside(location, &_boundary[1], _fence_total-1)) {
                // TRIGGER
                _triggered = true;
            }
        } else {
            // boundary incorrect
            // If geofence is required and our boundary fence is incorrect,
            // we trigger.
            if (_required) {
                // TRIGGER
                _triggered = true;
            }
        }
    }
    return _triggered;
}



AP_Int8 AP_Limit_Geofence::fence_total() {
    return _fence_total;
}

// save a fence point
void AP_Limit_Geofence::set_fence_point_with_index(Vector2l &point, uint8_t i)
{
    if (i >= (unsigned)fence_total()) {
        // not allowed
        return;
    }

    uint16_t mem = _eeprom_fence_start + (i * _fence_wp_size);

    hal.storage->write_dword(mem, point.x);
    mem += 4;
    hal.storage->write_dword(mem, point.y);

    _boundary_uptodate = false;
}

/*
 *  fence boundaries fetch/store
 */
Vector2l AP_Limit_Geofence::get_fence_point_with_index(uint8_t i)
{
    Vector2l ret;

    if (i > (unsigned) fence_total()) {
        return Vector2l(0,0);
    }

    // read fence point
    uint16_t mem = _eeprom_fence_start + (i * _fence_wp_size);
    ret.x = hal.storage->read_dword(mem);
    mem += 4;
    ret.y = hal.storage->read_dword(mem);

    return ret;
}

void AP_Limit_Geofence::update_boundary() {
    if (!_simple && _fence_total > 0) {

        for (uint8_t i = 0; i < (uint8_t) _fence_total; i++) {
            _boundary[i] = get_fence_point_with_index(i);
        }

        _boundary_uptodate = true;

    }
}

bool AP_Limit_Geofence::boundary_correct() {

    if (Polygon_complete(&_boundary[1], _fence_total - 1) &&
        !Polygon_outside(_boundary[0], &_boundary[1], _fence_total - 1)) {
        return true;
    } else return false;
}


