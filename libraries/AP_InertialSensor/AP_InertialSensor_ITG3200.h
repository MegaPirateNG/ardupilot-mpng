
#ifndef __AP_INERTIAL_SENSOR_ITG3200_H__
#define __AP_INERTIAL_SENSOR_ITG3200_H__

#include <string.h>
#include <stdint.h>

#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

// MPNG board types
#define CRIUS_V1  1
#define RCTIMER_CRIUS_V2 2
#define HK_RED_MULTIWII_PRO 3
#define BLACK_VORTEX 4

class AP_InertialSensor_ITG3200 : public AP_InertialSensor
{
public:

  AP_InertialSensor_ITG3200(uint8_t board_type);
  
  /* Concrete implementation of AP_InertialSensor functions: */
  bool 					update();
  float 				get_gyro_drift_rate();
  
    // get number of samples read from the sensors
  uint16_t				num_samples_available();
  
  // get_delta_time returns the time period in seconds overwhich the sensor data was collected
  float					get_delta_time();
  
  // Init I2C Bypass mode
  void                  hardware_init_i2c_bypass();
  
protected:
	uint16_t 			_init_sensor( Sample_rate sample_rate );
	
private:
  static bool read(uint32_t);
    
  static void           _read_data_from_timerprocess();
  static void           _read_data_transaction();
  static void           _poll_data(uint32_t now);
  void 					wait_for_sample();
  bool 					hardware_init(Sample_rate sample_rate);

  static AP_HAL::Semaphore *_i2c_sem;

  uint16_t				_num_samples;

  float                 _temp;
  
  float 				_temp_to_celsius( uint16_t );

  static const float 	_gyro_scale;

  static uint8_t 		_gyro_data_index[3];
  static int8_t 		_gyro_data_sign[3];

  static uint8_t 		_accel_data_index[3];
  static int8_t 		_accel_data_sign[3];

  static const uint8_t 		_temp_data_index;
  
  static bool healthy;

  static int16_t _data[7];
  
  static uint8_t _board_Type;
  static uint8_t _accel_addr;
  
 // ensure we can't initialise twice
  unsigned _initialised:1; 
};

#endif // __AP_INERTIAL_SENSOR_ITG3200_H__
