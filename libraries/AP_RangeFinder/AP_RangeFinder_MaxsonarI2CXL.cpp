// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_RangeFinder_MaxsonarI2CXL.cpp - Arduino Library for MaxBotix I2C XL sonar
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       datasheet: http://www.maxbotix.com/documents/I2CXL-MaxSonar-EZ_Datasheet.pdf
 *
 *       Sensor should be connected to the I2C port
 *
 *       Variables:
 *               bool healthy : indicates whether last communication with sensor was successful
 *
 *       Methods:
 *               take_reading(): ask the sonar to take a new distance measurement
 *               read() : read last distance measured (in cm)
 *
 */

// AVR LibC Includes
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_MaxsonarI2CXL::AP_RangeFinder_MaxsonarI2CXL( FilterInt16 *filter ) :
    RangeFinder(NULL, filter),
    healthy(true),
    _addr(AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR)
{
    min_distance = AP_RANGE_FINDER_MAXSONARI2CXL_MIN_DISTANCE;
    max_distance = AP_RANGE_FINDER_MAXSONARI2CXL_MAX_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////

// take_reading - ask sensor to make a range reading
bool AP_RangeFinder_MaxsonarI2CXL::take_reading()
{
    // take range reading and read back results
    uint8_t tosend[1] = 
        { AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING };
    if (hal.i2c->write(_addr, 1, tosend) != 0) {
        healthy = false;
        return false;
    }else{
        healthy = true;
        return true;
    }
}

// read - return last value measured by sensor
int AP_RangeFinder_MaxsonarI2CXL::read()
{
    uint8_t buff[2];
    int16_t ret_value = 0;

    // take range reading and read back results
    if (hal.i2c->read(_addr, 2, buff) != 0) {
        healthy = false;
    }else{
        // combine results into distance
        ret_value = buff[0] << 8 | buff[1];
        healthy = true;
    }
    
    // ensure distance is within min and max
    ret_value = constrain_float(ret_value, min_distance, max_distance);
    
    ret_value = _mode_filter->apply(ret_value);
    
    return ret_value;
}
