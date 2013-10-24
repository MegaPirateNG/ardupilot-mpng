/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_Compass_HMC5843.cpp - Arduino Library for HMC5843 I2C magnetometer
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

// AVR LibC Includes
#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_Compass_HMC5843.h"

extern const AP_HAL::HAL& hal;

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

// read_register - read a register value
bool AP_Compass_HMC5843::read_register(uint8_t address, uint8_t *value)
{
    if (hal.i2c->readRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        healthy = false;
        return false;
    }
    return true;
}

// write_register - update a register value
bool AP_Compass_HMC5843::write_register(uint8_t address, uint8_t value)
{
    if (hal.i2c->writeRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        healthy = false;
        return false;
    }
    return true;
}

// Read Sensor data
bool AP_Compass_HMC5843::read_raw()
{
    uint8_t buff[6];

    if (hal.i2c->readRegisters(COMPASS_ADDRESS, 0x03, 6, buff) != 0) {
        if (healthy) {
			hal.i2c->setHighSpeed(false);
        }
        healthy = false;
        _i2c_sem->give();
        return false;
    }

    int16_t rx, ry, rz;
    rx = (((int16_t)buff[0]) << 8) | buff[1];
    if (product_id == AP_COMPASS_TYPE_HMC5883L) {
        rz = (((int16_t)buff[2]) << 8) | buff[3];
        ry = (((int16_t)buff[4]) << 8) | buff[5];
    } else {
        ry = (((int16_t)buff[2]) << 8) | buff[3];
        rz = (((int16_t)buff[4]) << 8) | buff[5];
    }
    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_HMC5843::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
   uint32_t tnow = hal.scheduler->micros();
   if (healthy && _accum_count != 0 && (tnow - _last_accum_time) < 13333) {
	  // the compass gets new data at 75Hz
	  return;
   }

   if (!_i2c_sem->take(1)) {
       // the bus is busy - try again later
       return;
   }
   bool result = read_raw();
   _i2c_sem->give();

   if (result) {
	  // the _mag_N values are in the range -2048 to 2047, so we can
	  // accumulate up to 15 of them in an int16_t. Let's make it 14
	  // for ease of calculation. We expect to do reads at 10Hz, and
	  // we get new data at most 75Hz, so we don't expect to
	  // accumulate more than 8 before a read
	  _mag_x_accum += _mag_x;
	  _mag_y_accum += _mag_y;
	  _mag_z_accum += _mag_z;
	  _accum_count++;
	  if (_accum_count == 14) {
		 _mag_x_accum /= 2;
		 _mag_y_accum /= 2;
		 _mag_z_accum /= 2;
		 _accum_count = 7;
	  }
	  _last_accum_time = tnow;
   }
}


/*
 *  re-initialise after a IO error
 */
bool AP_Compass_HMC5843::re_initialise()
{
    if (!write_register(ConfigRegA, _base_config) ||
        !write_register(ConfigRegB, magGain) ||
        !write_register(ModeRegister, ContinuousConversion))
        return false;
    return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_HMC5843::init()
{
    int numAttempts = 0, good_count = 0;
    bool success = false;
    uint8_t calibration_gain = 0x20;
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    float gain_multiple = 1.0;

    hal.scheduler->delay(10);

    _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get HMC5843 semaphore"));
    }

    // determine if we are using 5843 or 5883L
    _base_config = 0;
    if (!write_register(ConfigRegA, SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation) ||
        !read_register(ConfigRegA, &_base_config)) {
        healthy = false;
        _i2c_sem->give();
        return false;
    }
    if ( _base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
        // a 5883L supports the sample averaging config
        product_id = AP_COMPASS_TYPE_HMC5883L;
        calibration_gain = 0x60;
        expected_x = 766;
        expected_yz  = 713;
        gain_multiple = 660.0 / 1090;  // adjustment for runtime vs calibration gain
    } else if (_base_config == (NormalOperation | DataOutputRate_75HZ<<2)) {
        product_id = AP_COMPASS_TYPE_HMC5843;
    } else {
        // not behaving like either supported compass type
        _i2c_sem->give();
        return false;
    }

    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;

    while ( success == 0 && numAttempts < 20 && good_count < 5)
    {
        // record number of attempts at initialisation
        numAttempts++;

        // force positiveBias (compass should return 715 for all channels)
        if (!write_register(ConfigRegA, PositiveBiasConfig))
            continue;      // compass not responding on the bus
        hal.scheduler->delay(50);

        // set gains
        if (!write_register(ConfigRegB, calibration_gain) ||
            !write_register(ModeRegister, SingleConversion))
            continue;

        // read values from the compass
        hal.scheduler->delay(50);
        if (!read_raw())
            continue;      // we didn't read valid values

        hal.scheduler->delay(10);

        float cal[3];

        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

        if (cal[0] > 0.7f && cal[0] < 1.3f &&
            cal[1] > 0.7f && cal[1] < 1.3f &&
            cal[2] > 0.7f && cal[2] < 1.3f) {
            good_count++;
            calibration[0] += cal[0];
            calibration[1] += cal[1];
            calibration[2] += cal[2];
        }

#if 0
        /* useful for debugging */
        hal.console->printf_P(PSTR("MagX: %d MagY: %d MagZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
        hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), cal[0], cal[1], cal[2]);
#endif
    }

    if (good_count >= 5) {
        calibration[0] = calibration[0] * gain_multiple / good_count;
        calibration[1] = calibration[1] * gain_multiple / good_count;
        calibration[2] = calibration[2] * gain_multiple / good_count;
        success = true;
    } else {
        /* best guess */
        calibration[0] = 1.0;
        calibration[1] = 1.0;
        calibration[2] = 1.0;
    }

    // leave test mode
    if (!re_initialise()) {
        _i2c_sem->give();
        return false;
    }

    _i2c_sem->give();
    _initialised = true;

	// perform an initial read
	healthy = true;
	read();

    return success;
}

// Read Sensor data
bool AP_Compass_HMC5843::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return false;
    }
    if (!healthy) {
        if (hal.scheduler->millis() < _retry_time) {
            return false;
        }
        if (!re_initialise()) {
            _retry_time = hal.scheduler->millis() + 1000;
			hal.i2c->setHighSpeed(false);
            return false;
        }
    }

	if (_accum_count == 0) {
	   accumulate();
	   if (!healthy || _accum_count == 0) {
		  // try again in 1 second, and set I2c clock speed slower
		  _retry_time = hal.scheduler->millis() + 1000;
		  hal.i2c->setHighSpeed(false);
		  return false;
	   }
	}

	mag_x = _mag_x_accum * calibration[0] / _accum_count;
	mag_y = _mag_y_accum * calibration[1] / _accum_count;
	mag_z = _mag_z_accum * calibration[2] / _accum_count;
	_accum_count = 0;
	_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    last_update = hal.scheduler->micros(); // record time of update

    // rotate to the desired orientation
    Vector3f rot_mag = Vector3f(mag_x,mag_y,mag_z);
    if (product_id == AP_COMPASS_TYPE_HMC5883L) {
        rot_mag.rotate(ROTATION_YAW_90);
    }

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    rot_mag.rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    rot_mag.rotate((enum Rotation)_orientation.get());

    if (!_external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        rot_mag.rotate(_board_orientation);
    }

    rot_mag += _offset.get();

    // apply motor compensation
    if(_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
        _motor_offset = _motor_compensation.get() * _thr_or_curr;
        rot_mag += _motor_offset;
    }else{
        _motor_offset.x = 0;
        _motor_offset.y = 0;
        _motor_offset.z = 0;
    }

    mag_x = rot_mag.x;
    mag_y = rot_mag.y;
    mag_z = rot_mag.z;
    healthy = true;

    return true;
}
