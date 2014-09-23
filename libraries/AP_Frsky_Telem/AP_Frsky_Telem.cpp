/*

  Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

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
   FRSKY Telemetry library
   for the moment it only handle hub port telemetry
   the sport reference are only here to simulate the frsky module and use opentx simulator. it will eventually be removed
*/
#include <stdio.h>
#include <AP_Frsky_Telem.h>
extern const AP_HAL::HAL& hal;

void AP_Frsky_Telem::init(AP_HAL::UARTDriver *port, enum FrSkyProtocol protocol)
{
    if (port == NULL) {
	return;
    }
    _port = port;    
    _protocol = protocol;
    if (_protocol == FrSkySPORT) {
	_port->begin(57600);
	gps_call = 0;
	fas_call = 0;
	vario_call = 0 ;
	various_call = 0 ;
	sem_init(&sem,0,1);
	hal.scheduler->register_io_process(AP_HAL_MEMBERPROC(&AP_Frsky_Telem::sport_tick));
    }else {// if this is D-port then spec says 9600 baud
	_port->begin(9600);
    }
    _initialised = true;    
}


/* 
   simple crc implementation for FRSKY telem S-PORT
*/
void AP_Frsky_Telem::calc_crc (uint8_t byte){
    crc += byte; //0-1FF
    crc += crc >> 8; //0-100
    crc &= 0x00ff;
    crc += crc >> 8; //0-0FF
    crc &= 0x00ff;
}

/*
 * send the crc at the end of the S-PORT frame
 */
void AP_Frsky_Telem::send_crc() {
    frsky_send_byte(0x00ff-crc);
    crc = 0;
}


/*
  send 1 byte and do the byte stuffing Frsky stuff 
  This can send more than 1 byte eventually
*/
void AP_Frsky_Telem::frsky_send_byte(uint8_t value)
{
    if (_protocol == FrSkyDPORT) {
	const uint8_t x5E[] = { 0x5D, 0x3E };
	const uint8_t x5D[] = { 0x5D, 0x3D };
	switch (value) {
	case 0x5E:
	    _port->write( x5E, sizeof(x5E));
	    break;

	case 0x5D:
	    _port->write( x5D, sizeof(x5D));
	    break;

	default:
	    _port->write(&value, sizeof(value));
	    break;
	}
    } else { //SPORT
	calc_crc(value);
	const uint8_t x7E[] = { 0x7D, 0x5E };
	const uint8_t x7D[] = { 0x7D, 0x5D };
	switch (value) {
	case 0x7E:
	    _port->write( x7E, sizeof(x7E));
	    break;

	case 0x7D:
	    _port->write( x7D, sizeof(x7D));
	    break;
	
	default:
	    _port->write(&value, sizeof(value));
	    break;
	}
    }
}

/**
 * Sends a 0x5E start/stop byte.
 */
void AP_Frsky_Telem::frsky_send_hub_startstop()
{
    static const uint8_t c = 0x5E;
    _port->write(&c, sizeof(c));
}

/*
  add sport protocol for frsky tx module 
*/
void AP_Frsky_Telem::frsky_send_sport_prim()
{
    static const uint8_t c = 0x10;
    frsky_send_byte(c);
}


/**
 * Sends one data id/value pair.
 */
void  AP_Frsky_Telem::frsky_send_data(uint8_t id, int16_t data)
{
    static const uint8_t zero = 0x0;

    /* Cast data to unsigned, because signed shift might behave incorrectly */
    uint16_t udata = data;

    if (_protocol == FrSkySPORT) {
	frsky_send_sport_prim();
	frsky_send_byte(id);
	frsky_send_byte(zero);
    } else {
	frsky_send_hub_startstop();
	frsky_send_byte(id);
    }

    frsky_send_byte(udata); /* LSB */
    frsky_send_byte(udata >> 8); /* MSB */

    if (_protocol == FrSkySPORT) {
	//Sport expect 32 bits data but we use only 16 bits data, so we send 0 for MSB
	frsky_send_byte(zero);
	frsky_send_byte(zero);
	send_crc();
    }
}

/*
 * frsky_send_baro_meters : send altitude in Meters based on ahrs estimate
 */
void AP_Frsky_Telem::calc_baro_alt(){
    float baro_alt = 0; // in meters
    bool posok = _inertial_nav.altitude_ok();
    baro_alt = _inertial_nav.get_altitude();
    if  (posok) {
	baro_alt = _inertial_nav.get_altitude() * 0.01f; // convert to meters
    }
    /*
      Note that this isn't actually barometric altitude, it is the
      AHRS estimate of altitdue above home.
    */
    baro_alt_meters = (int16_t)baro_alt;
    baro_alt_cm = (baro_alt - abs(baro_alt_meters)) * 100;
 
}

/**
 * Formats the decimal latitude/longitude to the required degrees/minutes.
 */
float  AP_Frsky_Telem::frsky_format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/*
 * prepare lattitude and longitude information stored in member variables
 */

void AP_Frsky_Telem::calc_gps_position(){

    course_in_degrees = (_ahrs.yaw_sensor / 100) % 360;

    const AP_GPS &gps = _ahrs.get_gps();
    float lat;
    float lon ;
    float alt ;
    float speed;
    pos_gps_ok = (gps.status() >= 3);
    if  (pos_gps_ok){
	Location loc = gps.location();//get gps instance 0
	
	lat = frsky_format_gps(fabsf(loc.lat/10000000.0));
	latdddmm = lat;
	latmmmm = (lat - latdddmm) * 10000;
	lat_ns = (loc.lat < 0) ? 'S' : 'N';
	
	lon = frsky_format_gps(fabsf(loc.lng/10000000.0));
	londddmm = lon;
	lonmmmm = (lon - londddmm) * 10000;
	lon_ew = (loc.lng < 0) ? 'W' : 'E';

	alt = loc.alt * 0.01f;
	alt_gps_meters = (int16_t)alt;
	alt_gps_cm = (alt - abs(alt_gps_meters)) * 100;

	speed = gps.ground_speed ();
	speed_in_meter = speed;
	speed_in_centimeter = (speed - speed_in_meter) * 100;
    } else {
	latdddmm = 0;
	latmmmm = 0;
	lat_ns = 0;
	londddmm = 0;
	lonmmmm = 0;
	alt_gps_meters = 0;
	alt_gps_cm = 0;
	speed_in_meter = 0;
	speed_in_centimeter = 0;
    }
}

/*
 * prepare battery information stored in member variables
 */
void AP_Frsky_Telem::calc_battery (){
    batt_remaining = roundf(_battery.capacity_remaining_pct());
    batt_volts = roundf(_battery.voltage() * 10.0f);
    batt_amps = roundf(_battery.current_amps() * 10.0f);
}

/*
 * prepare sats information stored in member variables
 */
void AP_Frsky_Telem::calc_gps_sats (){
    // GPS status is sent as num_sats*10 + status, to fit into a uint8_t
    const AP_GPS &gps = _ahrs.get_gps();
    gps_sats = gps.num_sats() * 10 + gps.status();
}

/*
 * send number of gps satellite and gps status eg: 73 means 7 satellite and 3d lock
 */
void AP_Frsky_Telem::send_gps_sats (){
    frsky_send_data(FRSKY_ID_TEMP2, gps_sats);
}

/*
 * send control_mode as Temperature 1 (TEMP1)
 */
void AP_Frsky_Telem::send_mode (void){
    frsky_send_data(FRSKY_ID_TEMP1, _mode);
}

/*
 * send barometer altitude integer part . Initialize baro altitude
 */
void AP_Frsky_Telem::send_baro_alt_m (void){
    frsky_send_data(FRSKY_ID_BARO_ALT_BP, baro_alt_meters);
}

/*
 * send barometer altitude decimal part
 */
void AP_Frsky_Telem::send_baro_alt_cm (void){
    frsky_send_data(FRSKY_ID_BARO_ALT_AP, baro_alt_cm);
}

/*
 * send battery remaining
 */
void AP_Frsky_Telem::send_batt_remain (void){
    frsky_send_data(FRSKY_ID_FUEL, batt_remaining);
}

/*
 * send battery voltage 
 */
void AP_Frsky_Telem::send_batt_volts (void){
    frsky_send_data(FRSKY_ID_VFAS, batt_volts);
}

/*
 * send current consumptiom 
 */
void AP_Frsky_Telem::send_current (void){
    frsky_send_data(FRSKY_ID_CURRENT, batt_amps);
}

/*
 * send heading in degree based on AHRS and not GPS 
 */
void AP_Frsky_Telem::send_heading (void){
    frsky_send_data(FRSKY_ID_GPS_COURS_BP, course_in_degrees);
}

/*
 * send gps lattitude degree and minute integer part; Initialize gps info
 */
void AP_Frsky_Telem::send_gps_lat_dd (void){
    frsky_send_data(FRSKY_ID_GPS_LAT_BP, latdddmm);
}

/*
 * send gps lattitude minutes decimal part 
 */
void AP_Frsky_Telem::send_gps_lat_mm (void){
    frsky_send_data(FRSKY_ID_GPS_LAT_AP, latmmmm);
}

/*
 * send gps North / South information 
 */
void AP_Frsky_Telem::send_gps_lat_ns (void){
    frsky_send_data(FRSKY_ID_GPS_LAT_NS, lat_ns);
}

/*
 * send gps longitude degree and minute integer part 
 */
void AP_Frsky_Telem::send_gps_lon_dd (void){
    frsky_send_data(FRSKY_ID_GPS_LONG_BP, londddmm);
}

/*
 * send gps longitude minutes decimal part 
 */
void AP_Frsky_Telem::send_gps_lon_mm (void){
    frsky_send_data(FRSKY_ID_GPS_LONG_AP, lonmmmm);
}

/*
 * send gps East / West information 
 */
void AP_Frsky_Telem::send_gps_lon_ew (void){
    frsky_send_data(FRSKY_ID_GPS_LONG_EW, lon_ew);
}

/*
 * send gps speed integer part
 */
void AP_Frsky_Telem::send_gps_speed_meter (void){
    frsky_send_data(FRSKY_ID_GPS_SPEED_BP, speed_in_meter);
}

/*
 * send gps speed decimal part
 */
void AP_Frsky_Telem::send_gps_speed_cm (void){
    frsky_send_data(FRSKY_ID_GPS_SPEED_AP, speed_in_centimeter);
}

/*
 * send gps altitude integer part
 */
void AP_Frsky_Telem::send_gps_alt_meter (void){
    frsky_send_data(FRSKY_ID_GPS_ALT_BP, alt_gps_meters);
}

/*
 * send gps altitude decimals
 */
void AP_Frsky_Telem::send_gps_alt_cm (void){
    frsky_send_data(FRSKY_ID_GPS_ALT_AP, alt_gps_cm);
}

/*
 * send prearm error
 */
void AP_Frsky_Telem::send_prearm_error (void){
    frsky_send_data(FRSKY_ID_PREARM, 1);
}

/*
 * send frame 1 every 200ms with baro alt, nb sats, batt volts and amp, control_mode
 */

void AP_Frsky_Telem::send_hub_frame(){
    uint32_t now = hal.scheduler->millis();

    // send frame1 every 200ms
    if (now - _last_frame1_ms > 200) {
        _last_frame1_ms = now;
	calc_gps_sats();
	send_gps_sats ();
 	send_mode ();

	calc_battery();
	send_batt_remain ();
	send_batt_volts ();
	send_current ();
	send_prearm_error();

	calc_baro_alt();
	send_baro_alt_m ();
	send_baro_alt_cm ();
    }
    // send frame2 every second
    if (now - _last_frame2_ms > 1000) {
        _last_frame2_ms = now;
	send_heading();
	calc_gps_position();
	if (pos_gps_ok){
	    send_gps_lat_dd ();
	    send_gps_lat_mm ();
	    send_gps_lat_ns ();
	    send_gps_lon_dd ();
	    send_gps_lon_mm ();
	    send_gps_lon_ew ();
	    send_gps_speed_meter ();
	    send_gps_speed_cm ();
	    send_gps_alt_meter ();
	    send_gps_alt_cm ();
	}
    }
}


/*
  send telemetry frames. Should be called at 50Hz. The high rate is to
  allow this code to poll for serial bytes coming from the receiver
  for the SPort protocol
*/
void AP_Frsky_Telem::send_frames(uint8_t control_mode)
{
    if (!_initialised) {
        return;
    }
    
    if (_protocol == FrSkySPORT) {
	sem_wait (&sem);	
	_mode=control_mode;

	calc_baro_alt();
	calc_gps_position();
	calc_gps_sats();
	calc_battery();
	sem_post(&sem);
    }else{
	_mode=control_mode;
	send_hub_frame();
    }
}


void AP_Frsky_Telem::sport_tick(void){
#define START_FRAME 0x7E
#define END_FRAME 0
    static int8_t sport_status = END_FRAME;
    while (_port->available()){
    	int16_t readbyte = _port->read();
	if (_port->txspace() < 19) {
	    ::printf ("tx space too small\n");
	}else{
	    if (readbyte == 0x7E){
		sport_status = START_FRAME;
	    } else{
		if (sport_status == START_FRAME){
		    if (sem_trywait(&sem))
			return;

		    switch (readbyte){
		    case DATA_ID_FAS: 
			switch (fas_call){
			case 0:
			    send_batt_volts();
			    break;
			case 1:
			    send_current();
			    break;
			}
			fas_call++;
			if (fas_call > 1) fas_call = 0;
			break;
		    case DATA_ID_GPS:
			switch (gps_call) {
			case 0:
			    send_gps_lat_dd ();
			    break;
			case 1:
			    send_gps_lat_mm ();
			    break;
			case 2:
			    send_gps_lat_ns ();
			    break;
			case 3:
			    send_gps_lon_dd ();
			    break;
			case 4:
			    send_gps_lon_mm ();
			    break;
			case 5:
			    send_gps_lon_ew ();
			    break;
			case 6:
			    send_gps_speed_meter ();
			    break;
			case 7:
			    send_gps_speed_cm ();
			    break;
			case 8:
			    send_gps_alt_meter ();
			    break;
			case 9:
			    send_gps_alt_cm ();
			    break;
			case 10:
			    send_heading();
			    break;
			}
		       
			gps_call++;
			if (gps_call > 10) gps_call = 0;
			break;
		    case DATA_ID_VARIO:
			switch (vario_call){
			case 0 :
			    send_baro_alt_m ();
			    break;
			case 1:
			    send_baro_alt_cm ();
			    break;
			}
			vario_call ++;
			if (vario_call > 1) vario_call = 0;
			break;
		    case DATA_ID_SP2UR:
			switch (various_call){
			case 0 :
			    send_gps_sats ();
			    break;
			case 1:
			    send_mode ();
			    break;
			}
			various_call++;
			if (various_call > 1) various_call = 0;
			break;
		    }
		    sport_status = END_FRAME;
		    sem_post(&sem);

		}
	    }	
	}
    }
}
