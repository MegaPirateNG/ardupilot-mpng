// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0
#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

#define DEBUG 0
#define SERVO_MAX 4500	// This value represents 45 degrees and is just an arbitrary representation of servo max travel.

// active altitude sensor
// ----------------------
#define SONAR 0
#define BARO 1

// CH 7 control
enum ch7_option {
    CH7_DO_NOTHING=0,
    CH7_SAVE_WP=1
};

#define T6 1000000
#define T7 10000000

// GPS type codes - use the names, not the numbers
#define GPS_PROTOCOL_NONE	-1
#define GPS_PROTOCOL_NMEA	0
#define GPS_PROTOCOL_SIRF	1
#define GPS_PROTOCOL_UBLOX	2
#define GPS_PROTOCOL_IMU	3
#define GPS_PROTOCOL_MTK	4
#define GPS_PROTOCOL_HIL	5
#define GPS_PROTOCOL_MTK19	6
#define GPS_PROTOCOL_AUTO	7

// HIL enumerations
#define HIL_MODE_DISABLED			0
#define HIL_MODE_ATTITUDE			1
#define HIL_MODE_SENSORS			2

// Auto Pilot modes
// ----------------
enum mode {
    MANUAL=0,
	LEARNING=2,
    STEERING=3,
    HOLD=4,
    AUTO=10,
    RTL=11,
    GUIDED=15,
    INITIALISING=16
};

// types of failsafe events
#define FAILSAFE_EVENT_THROTTLE (1<<0)
#define FAILSAFE_EVENT_GCS      (1<<1)
#define FAILSAFE_EVENT_RC       (1<<2)

// Commands - Note that APM now uses a subset of the MAVLink protocol commands.  See enum MAV_CMD in the GCS_Mavlink library
#define CMD_BLANK 0 // there is no command stored in the mem location requested
#define NO_COMMAND 0
#define WAIT_COMMAND 255

// Command/Waypoint/Location Options Bitmask
//--------------------
#define MASK_OPTIONS_RELATIVE_ALT	(1<<0)		// 1 = Relative altitude

//repeating events
#define NO_REPEAT 0
#define CH_5_TOGGLE 1
#define CH_6_TOGGLE 2
#define CH_7_TOGGLE 3
#define CH_8_TOGGLE 4
#define RELAY_TOGGLE 5
#define STOP_REPEAT 10

#define MAV_CMD_CONDITION_YAW 23

//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream
enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
    MSG_LOCATION,
    MSG_EXTENDED_STATUS1,
    MSG_EXTENDED_STATUS2,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_CURRENT_WAYPOINT,
    MSG_VFR_HUD,
    MSG_RADIO_OUT,
    MSG_RADIO_IN,
    MSG_RAW_IMU1,
    MSG_RAW_IMU3,
    MSG_GPS_RAW,
    MSG_SERVO_OUT,
    MSG_NEXT_WAYPOINT,
    MSG_NEXT_PARAM,
    MSG_STATUSTEXT,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_HWSTATUS,
    MSG_RANGEFINDER,
    MSG_RETRY_DEFERRED // this must be last
};

//  Logging parameters
#define LOG_CTUN_MSG	        0x01
#define LOG_NTUN_MSG    		0x02
#define LOG_PERFORMANCE_MSG		0x03
#define LOG_CMD_MSG			    0x04
#define LOG_CURRENT_MSG 		0x05
#define LOG_STARTUP_MSG 		0x06
#define LOG_SONAR_MSG 		    0x07
#define LOG_ATTITUDE_MSG        0x08
#define LOG_MODE_MSG            0x09
#define LOG_COMPASS_MSG         0x0A
#define LOG_CAMERA_MSG          0x0B

#define TYPE_AIRSTART_MSG		0x00
#define TYPE_GROUNDSTART_MSG	0x01
#define MAX_NUM_LOGS			100

#define MASK_LOG_ATTITUDE_FAST 	(1<<0)
#define MASK_LOG_ATTITUDE_MED 	(1<<1)
#define MASK_LOG_GPS 			(1<<2)
#define MASK_LOG_PM 			(1<<3)
#define MASK_LOG_CTUN 			(1<<4)
#define MASK_LOG_NTUN			(1<<5)
#define MASK_LOG_MODE			(1<<6)
#define MASK_LOG_IMU			(1<<7)
#define MASK_LOG_CMD			(1<<8)
#define MASK_LOG_CURRENT		(1<<9)
#define MASK_LOG_SONAR   		(1<<10)
#define MASK_LOG_COMPASS   		(1<<11)
#define MASK_LOG_CAMERA   		(1<<12)

// Waypoint Modes
// ----------------
#define ABS_WP 0
#define REL_WP 1

// Command Queues
// ---------------
#define COMMAND_MUST 0
#define COMMAND_MAY 1
#define COMMAND_NOW 2

// Events
// ------
#define EVENT_WILL_REACH_WAYPOINT 1
#define EVENT_SET_NEW_COMMAND_INDEX 2
#define EVENT_LOADED_WAYPOINT 3
#define EVENT_LOOP 4

// Climb rate calculations
#define	ALTITUDE_HISTORY_LENGTH 8	//Number of (time,altitude) points to regress a climb rate from

#define RELAY_PIN 47


// sonar
#define MAX_SONAR_XL 0
#define MAX_SONAR_LV 1
#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters
#define AN4			4
#define AN5			5

#define SPEEDFILT 400			// centimeters/second; the speed below which a groundstart will be triggered


// EEPROM addresses
#define EEPROM_MAX_ADDR		4096
// parameters get the first 1KiB of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x500 // where in memory home WP is stored + all other WP
#define WP_SIZE 15

#define MAX_WAYPOINTS  ((EEPROM_MAX_ADDR - WP_START_BYTE) / WP_SIZE) - 1 // - 1 to be safe

// convert a boolean (0 or 1) to a sign for multiplying (0 maps to 1, 1 maps to -1)
#define BOOL_TO_SIGN(bvalue) ((bvalue)?-1:1)

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

// InertialSensor driver types
#define CONFIG_INS_OILPAN  1
#define CONFIG_INS_MPU6000 2
#define CONFIG_INS_HIL     3
#define CONFIG_INS_PX4     4
#define CONFIG_INS_FLYMAPLE 5
#define CONFIG_INS_L3G4200D 6
#define CONFIG_INS_MPU6000_I2C 7

// compass driver types
#define AP_COMPASS_HMC5843   1
#define AP_COMPASS_PX4       2
#define AP_COMPASS_HIL       3

#endif // _DEFINES_H
