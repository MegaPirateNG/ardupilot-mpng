// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// User specific config file.  Any items listed in config.h can be overridden here.

// Select Megapirate board type:
//#define MPNG_BOARD_TYPE   CRIUS_V1
/*
 * RCTIMER_CRIUS_V2    -- (DEFAULT!!!) Use ONLY for RCTimer CRIUS V2 board
 * CRIUS_V1            -- Use this define for RCTimer CRIUS V1(1.1) board and all HobbyKing AIOP boards
 * HK_RED_MULTIWII_PRO -- HobbyKing MultiWii Pro board with ITG3205 and BMA180, BMP085 sensors
 */

// Currently not supported
#define CONFIG_SONAR DISABLED

// GPS port speed (Serial2) 38400 by default
#define SERIAL2_BAUD 38400

//#define COPTER_LEDS DISABLED 

// GPS driver selection
//#define GPS_PROTOCOL GPS_PROTOCOL_NONE
/*
	GPS_PROTOCOL_AUTO   (Default)
	GPS_PROTOCOL_NONE
	GPS_PROTOCOL_NMEA
	GPS_PROTOCOL_SIRF
	GPS_PROTOCOL_UBLOX
	GPS_PROTOCOL_IMU
	GPS_PROTOCOL_MTK
	GPS_PROTOCOL_HIL
	GPS_PROTOCOL_MTK19
*/

// QuadCopter selected by default
//#define FRAME_CONFIG HEXA_FRAME
/*
 *  options:
 *  QUAD_FRAME
 *  TRI_FRAME
 *  HEXA_FRAME
 *  Y6_FRAME
 *  OCTA_FRAME
 *  OCTA_QUAD_FRAME
 *  HELI_FRAME
 */

//#define HIL_MODE              HIL_MODE_SENSORS    // build for hardware-in-the-loop simulation
//#define HIL_MODE              HIL_MODE_ATTITUDE    // build for hardware-in-the-loop simulation

// If Arduino IDE hang while uploading firmware to your board, try to change string below, just enter some random characters
#define BOOTLOADER_BUGFIX "234fs34567"

// User Hooks : For User Developed code that you wish to run
// Put your variable definitions into the UserVariables.h file (or another file name and then change the #define below).
//#define USERHOOK_VARIABLES "UserVariables.h"
// Put your custom code into the UserCode.pde with function names matching those listed below and ensure the appropriate #define below is uncommented below
//#define USERHOOK_INIT userhook_init();                    // for code to be run once at startup
//#define USERHOOK_FASTLOOP userhook_FastLoop();            // for code to be run at 100hz
//#define USERHOOK_50HZLOOP userhook_50Hz();                // for code to be run at 50hz
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // for code to be run at 10hz
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();            // for code to be run at 3.3hz
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // for code to be run at 1hz
