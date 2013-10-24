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
#ifndef AP_VEHICLE_H
#define AP_VEHICLE_H
/*
  this header holds a parameter structure for each vehicle type for
  parameters needed by multiple libraries
 */

class AP_Vehicle {

public:
    /*
      common parameters for fixed wing aircraft
     */
    struct FixedWing {
        AP_Int8 throttle_min;
        AP_Int8 throttle_max;	
        AP_Int8 throttle_slewrate;
        AP_Int8 throttle_cruise;
        AP_Int16 airspeed_min;
        AP_Int16 airspeed_max;
        AP_Int16 pitch_limit_max_cd;
        AP_Int16 pitch_limit_min_cd;        
    };
};

#endif // AP_VEHICLE_H
