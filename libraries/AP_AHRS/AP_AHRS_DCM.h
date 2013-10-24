#ifndef __AP_AHRS_DCM_H__
#define __AP_AHRS_DCM_H__
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  DCM based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

class AP_AHRS_DCM : public AP_AHRS
{
public:
    // Constructors
    AP_AHRS_DCM(AP_InertialSensor *ins, GPS *&gps) :
        AP_AHRS(ins, gps),
        _last_declination(0),
        _mag_earth(1,0)
    {
        _dcm_matrix.identity();

        // these are experimentally derived from the simulator
        // with large drift levels
        _ki = 0.0087;
        _ki_yaw = 0.01;
    }

    // return the smoothed gyro vector corrected for drift
    const Vector3f get_gyro(void) const {
        return _omega + _omega_P + _omega_yaw_P;
    }
    const Matrix3f &get_dcm_matrix(void) const {
        return _dcm_matrix;
    }

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const {
        return _omega_I;
    }

    // Methods
    void            update(void);
    void            reset(bool recover_eulers = false);

    // dead-reckoning support
    bool get_position(struct Location &loc);

    // status reporting
    float           get_error_rp(void);
    float           get_error_yaw(void);

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate(void) {
        return _wind;
    }

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float *airspeed_ret);

    bool            use_compass(void) const;

private:
    float _ki;
    float _ki_yaw;

    // Methods
    void            matrix_update(float _G_Dt);
    void            normalize(void);
    void            check_matrix(void);
    bool            renorm(Vector3f const &a, Vector3f &result);
    void            drift_correction(float deltat);
    void            drift_correction_yaw(void);
    float           yaw_error_compass();
    void            euler_angles(void);
    void            estimate_wind(Vector3f &velocity);
    bool            have_gps(void) const;

    // primary representation of attitude
    Matrix3f _dcm_matrix;

    Vector3f _gyro_vector;                      // Store the gyros turn rate in a vector
    Vector3f _accel_vector;                     // current accel vector

    Vector3f _omega_P;                          // accel Omega proportional correction
    Vector3f _omega_yaw_P;                      // proportional yaw correction
    Vector3f _omega_I;                          // Omega Integrator correction
    Vector3f _omega_I_sum;
    float _omega_I_sum_time;
    Vector3f _omega;                            // Corrected Gyro_Vector data

    // P term gain based on spin rate
    float           _P_gain(float spin_rate);

    // state to support status reporting
    float _renorm_val_sum;
    uint16_t _renorm_val_count;
    float _error_rp_sum;
    uint16_t _error_rp_count;
    float _error_rp_last;
    float _error_yaw_sum;
    uint16_t _error_yaw_count;
    float _error_yaw_last;

    // time in millis when we last got a GPS heading
    uint32_t _gps_last_update;

    // state of accel drift correction
    Vector3f _ra_sum;
    Vector3f _last_velocity;
    float _ra_deltat;
    uint32_t _ra_sum_start;

    // the earths magnetic field
    float _last_declination;
    Vector2f _mag_earth;

    // whether we have GPS lock
    bool _have_gps_lock;

    // the lat/lng where we last had GPS lock
    int32_t _last_lat;
    int32_t _last_lng;

    // position offset from last GPS lock
    float _position_offset_north;
    float _position_offset_east;

    // whether we have a position estimate
    bool _have_position;

    // support for wind estimation
    Vector3f _last_fuse;
    Vector3f _last_vel;
    uint32_t _last_wind_time;
    float _last_airspeed;

    // estimated wind in m/s
    Vector3f _wind;
};

#endif // __AP_AHRS_DCM_H__
