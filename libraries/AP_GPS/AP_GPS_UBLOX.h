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

//
//  u-blox UBX GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//  UBlox Lea6H protocol: http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf
#ifndef __AP_GPS_UBLOX_H__
#define __AP_GPS_UBLOX_H__

#include <AP_HAL.h>
#include <AP_Common.h>
#include "GPS.h"

/*
 *  try to put a UBlox into binary mode. This is in two parts. 
 *
 * First we send a ubx binary message that enables the NAV_SOL message
 * at rate 1. Then we send a NMEA message to set the baud rate to our
 * desired rate. The reason for doing the NMEA message second is if we
 * send it first the second message will be ignored for a baud rate
 * change.
 * The reason we need the NAV_SOL rate message at all is some uBlox
 * modules are configured with all ubx binary messages off, which
 * would mean we would never detect it.
 */
#define UBLOX_SET_BINARY "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0003,0001,38400,0*26\r\n"

class AP_GPS_UBLOX : public GPS
{
public:
	AP_GPS_UBLOX() :
		GPS(),
		_step(0),
		_msg_id(0),
		_payload_length(0),
		_payload_counter(0),
		_fix_count(0),
		_disable_counter(0),
		next_fix(GPS::FIX_NONE),
        need_rate_update(false),
        rate_update_step(0)
		{}

    // Methods
    virtual void                    init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE);
    virtual bool                    read();
    static bool _detect(uint8_t );

    static const prog_char          _ublox_set_binary[];
    static const uint8_t            _ublox_set_binary_size;

    float       get_lag() { return 0.5; }   // ublox lag is lower than the default 1second

private:
    // u-blox UBX protocol essentials
    struct PACKED ubx_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t length;
    };
    struct PACKED ubx_cfg_nav_rate {
        uint16_t measure_rate_ms;
        uint16_t nav_rate;
        uint16_t timeref;
    };
    struct PACKED ubx_cfg_msg_rate {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rate;
    };
    struct PACKED ubx_cfg_nav_settings {
        uint16_t mask;
        uint8_t dynModel;
        uint8_t fixMode;
        int32_t fixedAlt;
        uint32_t fixedAltVar;
        int8_t minElev;
        uint8_t drLimit;
        uint16_t pDop;
        uint16_t tDop;
        uint16_t pAcc;
        uint16_t tAcc;
        uint8_t staticHoldThresh;
        uint8_t res1;
        uint32_t res2;
        uint32_t res3;
        uint32_t res4;
    };

    struct PACKED ubx_nav_posllh {
        uint32_t time;                                  // GPS msToW
        int32_t longitude;
        int32_t latitude;
        int32_t altitude_ellipsoid;
        int32_t altitude_msl;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
    };
    struct PACKED ubx_nav_status {
        uint32_t time;                                  // GPS msToW
        uint8_t fix_type;
        uint8_t fix_status;
        uint8_t differential_status;
        uint8_t res;
        uint32_t time_to_first_fix;
        uint32_t uptime;                                // milliseconds
    };
    struct PACKED ubx_nav_solution {
        uint32_t time;
        int32_t time_nsec;
        int16_t week;
        uint8_t fix_type;
        uint8_t fix_status;
        int32_t ecef_x;
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t position_accuracy_3d;
        int32_t ecef_x_velocity;
        int32_t ecef_y_velocity;
        int32_t ecef_z_velocity;
        uint32_t speed_accuracy;
        uint16_t position_DOP;
        uint8_t res;
        uint8_t satellites;
        uint32_t res2;
    };
    struct PACKED ubx_nav_velned {
        uint32_t time;                                  // GPS msToW
        int32_t ned_north;
        int32_t ned_east;
        int32_t ned_down;
        uint32_t speed_3d;
        uint32_t speed_2d;
        int32_t heading_2d;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
    };
    // Receive buffer
    union PACKED {
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_solution solution;
        ubx_nav_velned velned;
        ubx_cfg_nav_settings nav_settings;
        uint8_t bytes[];
    } _buffer;

    enum ubs_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        CLASS_ACK = 0x05,
        CLASS_CFG = 0x06,
        MSG_ACK_NACK = 0x00,
        MSG_ACK_ACK = 0x01,
        MSG_POSLLH = 0x2,
        MSG_STATUS = 0x3,
        MSG_SOL = 0x6,
        MSG_VELNED = 0x12,
        MSG_CFG_PRT = 0x00,
        MSG_CFG_RATE = 0x08,
        MSG_CFG_SET_RATE = 0x01,
        MSG_CFG_NAV_SETTINGS = 0x24
    };
    enum ubs_nav_fix_type {
        FIX_NONE = 0,
        FIX_DEAD_RECKONING = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_GPS_DEAD_RECKONING = 4,
        FIX_TIME = 5
    };
    enum ubx_nav_status_bits {
        NAV_STATUS_FIX_VALID = 1
    };

    // Packet checksum accumulators
    uint8_t         _ck_a;
    uint8_t         _ck_b;

    // State machine state
    uint8_t         _step;
    uint8_t         _msg_id;
    uint16_t        _payload_length;
    uint16_t        _payload_counter;

	// 8 bit count of fix messages processed, used for periodic
	// processing
    uint8_t			_fix_count;

    uint8_t         _class;

    // do we have new position information?
    bool            _new_position;

    // do we have new speed information?
    bool            _new_speed;

    uint8_t         _disable_counter;

    // Buffer parse & GPS state update
    bool        _parse_gps();

    // used to update fix between status and position packets
    Fix_Status  next_fix;

    bool need_rate_update;
    uint8_t rate_update_step;
    uint32_t _last_fix_time;

    void 	    _configure_navigation_rate(uint16_t rate_ms);
    void        _configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    void        _configure_gps(void);
    void        _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b);
    void        _send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size);
    void		send_next_rate_update(void);

};

#endif // __AP_GPS_UBLOX_H__
