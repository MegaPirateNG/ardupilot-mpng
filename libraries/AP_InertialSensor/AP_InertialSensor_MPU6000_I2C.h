/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6000_I2C_H__
#define __AP_INERTIAL_SENSOR_MPU6000_I2C_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

class AP_InertialSensor_MPU6000_I2C : public AP_InertialSensor
{
public:

    AP_InertialSensor_MPU6000_I2C();

    /* Concrete implementation of AP_InertialSensor functions: */
    bool                update();
    float               get_gyro_drift_rate();

    // num_samples_available - get number of samples read from the sensors
    bool            sample_available();
    bool                        wait_for_sample(uint16_t timeout_ms);
    void                 _poll_data(void);

    // get_delta_time returns the time period in seconds overwhich the sensor data was collected
    float            	get_delta_time();
    
    // Init I2C Bypass mode
    void                        hardware_init_i2c_bypass();

protected:
    uint16_t                    _init_sensor( Sample_rate sample_rate );
  	float                       _delta_time;

private:

    static void                 _read_data_from_timerprocess();
    static void                 _read_data_transaction();
    bool                        hardware_init(Sample_rate sample_rate);

    static AP_HAL::Semaphore *_i2c_sem;

    uint16_t					          _num_samples;

    float                       _temp;

    float                       _temp_to_celsius( uint16_t );

    static const float          _gyro_scale;

    static const uint8_t        _gyro_data_index[3];
    static const int8_t         _gyro_data_sign[3];

    static const uint8_t        _accel_data_index[3];
    static const int8_t         _accel_data_sign[3];

    static const uint8_t        _temp_data_index;
    static uint8_t              mpu_addr; 

    // ensure we can't initialise twice
    bool                        _initialised;
    static int16_t              _mpu6000_product_id;

  	static uint16_t             _micros_per_sample;

    // support for updating filter at runtime
    uint8_t                     _last_filter_hz;
  
    void _set_filter_register(uint8_t filter_hz, uint8_t default_filter);

};

#endif // __AP_INERTIAL_SENSOR_MPU6000_I2C_H__
