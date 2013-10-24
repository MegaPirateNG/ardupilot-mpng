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
  backend driver for airspeed from a I2C MS4525D0 sensor
 */

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Airspeed_I2C.h>

extern const AP_HAL::HAL& hal;

#define I2C_ADDRESS_MS4525DO	0x28

// probe and initialise the sensor
bool AP_Airspeed_I2C::init(void)
{
    _measure();
    hal.scheduler->delay(10);
    _collect();
    if (_last_sample_time_ms != 0) {
        hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_Airspeed_I2C::_timer));
        return true;
    }
    return false;
}

// start a measurement
void AP_Airspeed_I2C::_measure(void)
{
    _measurement_started_ms = 0;
    if (hal.i2c->writeRegisters(I2C_ADDRESS_MS4525DO, 0, 0, NULL) != 0) {
        return;
    }
    _measurement_started_ms = hal.scheduler->millis();
}

// read the values from the sensor
void AP_Airspeed_I2C::_collect(void)
{
    uint8_t data[4];
    _measurement_started_ms = 0;

    if (hal.i2c->read(I2C_ADDRESS_MS4525DO, 4, data) != 0) {
        return;
    }
    
	uint8_t status = data[0] & 0xC0;
	if (status == 2) {
        return;
	} else if (status == 3) {
        return;
	}

	int16_t dp_raw, dT_raw;
	dp_raw = (data[0] << 8) + data[1];
	dp_raw = 0x3FFF & dp_raw;
	dT_raw = (data[2] << 8) + data[3];
	dT_raw = (0xFFE0 & dT_raw) >> 5;

	_temperature = ((200 * dT_raw) / 2047) - 50;
	_pressure = fabs(dp_raw - (16384 / 2.0f));
    _last_sample_time_ms = hal.scheduler->millis();
}

// 1kHz timer
void AP_Airspeed_I2C::_timer(void)
{
    if (_measurement_started_ms == 0) {
        _measure();
        return;
    }
    if ((hal.scheduler->millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_I2C::get_differential_pressure(float &pressure)
{
    if ((hal.scheduler->millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    pressure = _pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_I2C::get_temperature(float &temperature)
{
    if ((hal.scheduler->millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    temperature = _temperature;
    return true;
}


