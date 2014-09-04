/*
	@author 	Nils Högberg
	@contact 	nils.hogberg@gmail.com

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

#ifndef frsky_h
#define frsky_h

#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Baro.h>
#include <AP_BattMonitor.h>

#include "defines.h"

class FrSky
{
	
public:
		FrSky(AP_AHRS &ahrs, AP_BattMonitor &battery);
		~FrSky(void);
		void init(AP_HAL::UARTDriver *port, uint8_t frsky_type);
		void send_frames(uint8_t control_mode);
		void sendFrSky5Hz();
		void sendFrSky1Hz();
		void sendFrSky05Hz();
#if 0
		void printValues();
#endif		
private:
		unsigned char		frskyBuffer[64];
		int					bufferLength;
		unsigned char		addBufferData(const char id);
		unsigned char		writeBuffer(const int length);
		byte				lsByte(int value);
		byte				msByte(int value);
		AP_HAL::UARTDriver *_port;
		bool _initialised;
		AP_AHRS &_ahrs;
		AP_BattMonitor &_battery;
		uint32_t _last_frame1_ms;
    	uint32_t _last_frame2_ms;
};

#endif
