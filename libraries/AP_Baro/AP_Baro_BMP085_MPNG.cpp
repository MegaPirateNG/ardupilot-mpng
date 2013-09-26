/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_Baro_BMP085_MPNG.h"

extern const AP_HAL::HAL& hal;


uint32_t volatile AP_Baro_BMP085_MPNG::_s_D1;
uint32_t volatile AP_Baro_BMP085_MPNG::_s_D2;
uint8_t volatile AP_Baro_BMP085_MPNG::_d1_count;
uint8_t volatile AP_Baro_BMP085_MPNG::_d2_count;
uint8_t AP_Baro_BMP085_MPNG::_state;
uint32_t AP_Baro_BMP085_MPNG::_timer;
bool volatile AP_Baro_BMP085_MPNG::_updated;
AP_HAL::Semaphore* AP_Baro_BMP085_MPNG::_i2c_sem = NULL;
int32_t                         AP_Baro_BMP085_MPNG::_raw_press;
int32_t                         AP_Baro_BMP085_MPNG::_raw_temp;
    
#define BMP085_ADDRESS 0x77
#define READ_PRESS_TIMEOUT 25000 // reading press timeout, if oss=3, timout = 25ms
#define READ_TEMP_TIMEOUT 4500
#define OVERSAMPLING 3

static volatile uint32_t _baro_read_timeout = READ_TEMP_TIMEOUT;


// Public Methods //////////////////////////////////////////////////////////////
bool AP_Baro_BMP085_MPNG::init()
{
    healthy = false;
    _i2c_sem = hal.i2c->get_semaphore();

    if (_i2c_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_BMP085_MPNG: NULL serial driver"));
        return false; /* never reached */
    }

    uint8_t buff[22];
    
    hal.scheduler->suspend_timer_procs(); 
    
    if (!_i2c_sem->take(1000)){
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_BMP085_MPNG: failed to take "
                    "serial semaphore for init"));
        return false; /* never reached */
    }

    hal.scheduler->delay(4);

    // We read the factory calibration
    // The on-chip CRC is not used
    if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xAA, 22, buff) != 0) {
        return false;
    }
    _i2c_sem->give();

    ac1 = ((int16_t)buff[0] << 8) | buff[1];
    ac2 = ((int16_t)buff[2] << 8) | buff[3];
    ac3 = ((int16_t)buff[4] << 8) | buff[5];
    ac4 = ((int16_t)buff[6] << 8) | buff[7];
    ac5 = ((int16_t)buff[8] << 8) | buff[9];
    ac6 = ((int16_t)buff[10] << 8) | buff[11];
    b1 = ((int16_t)buff[12] << 8) | buff[13];
    b2 = ((int16_t)buff[14] << 8) | buff[15];
    mb = ((int16_t)buff[16] << 8) | buff[17];
    mc = ((int16_t)buff[18] << 8) | buff[19];
    md = ((int16_t)buff[20] << 8) | buff[21];


    //Send a command to read Temp first
    _timer = hal.scheduler->micros();
    _state = 0;
    Temp=0;
    Press=0;

		hal.scheduler->resume_timer_procs();

    hal.scheduler->register_timer_process( AP_Baro_BMP085_MPNG::_poll_data );

    healthy = true;
    return true;
}


void AP_Baro_BMP085_MPNG::_poll_data(uint32_t now)
{
	if ( now - _timer > _baro_read_timeout) {
		_timer = now;
	
	  if (hal.scheduler->in_timerprocess()) {
	      _read_data_from_timerprocess();
	      
	  } else {
	      /* Synchronous read - take semaphore */
	      bool got = _i2c_sem->take(10);
	      if (got) {
          	_read_data_transaction(); 
	          _i2c_sem->give();
	      } else {
	          hal.scheduler->panic(
	                  PSTR("PANIC: AP_Baro_BMP085_MPNG::_poll_data "
	                       "failed to take I2C semaphore synchronously"));
	      }
	  }
	}
}

void AP_Baro_BMP085_MPNG::_read_data_from_timerprocess()
{
    static uint8_t semfail_ctr = 0;
    bool got = _i2c_sem->take_nonblocking();
    if (!got) { 
        semfail_ctr++;
        if (semfail_ctr > 100) {
            hal.scheduler->panic(PSTR("PANIC: failed to take I2C semaphore "
                        "100 times in AP_Baro_BMP085_MPNG::"
                        "_read_data_from_timerprocess"));
        }
        return;
    } else {
        semfail_ctr = 0;
    }   

    _read_data_transaction();

    _i2c_sem->give();
}
// Read the sensor. This is a state machine
void AP_Baro_BMP085_MPNG::_read_data_transaction()
{
    if (_state == 0) {
			ReadTemp();
			_state++;
			Command_ReadPress();
    } else {
			ReadPress();
        // Now a new reading exists
			_updated = true;
      if (_state == 5) {
	    	Command_ReadTemp();
        _state = 0;
      } else {
				_state++;
				Command_ReadPress();
        }
    }
}

uint8_t AP_Baro_BMP085_MPNG::read()
{
    bool updated = _updated;
    if (updated) {
        _calculate();
        _last_update = hal.scheduler->millis();
    }
    return updated ? 1 : 0;
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_BMP085_MPNG::_calculate()
{
				int32_t x1, x2, x3, b3, b5, b6, p;
        uint32_t b4, b7;
        int32_t tmp;

        // See Datasheet page 13 for this formulas
        // Based also on Jee Labs BMP085 example code. Thanks for share.
        // Temperature calculations
        x1 = ((int32_t)_raw_temp - ac6) * ac5 >> 15;
        x2 = ((int32_t) mc << 11) / (x1 + md);
        b5 = x1 + x2;
        Temp = (b5 + 8) >> 4;

        // Pressure calculations
        b6 = b5 - 4000;
        x1 = (b2 * (b6 * b6 >> 12)) >> 11;
        x2 = ac2 * b6 >> 11;
        x3 = x1 + x2;
        //b3 = (((int32_t) ac1 * 4 + x3)<<OVERSAMPLING + 2) >> 2; // BAD
        //b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;  //OK for oss=0
        tmp = ac1;
        tmp = (tmp*4 + x3)<<OVERSAMPLING;
        b3 = (tmp+2)/4;
        x1 = ac3 * b6 >> 13;
        x2 = (b1 * (b6 * b6 >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
        b7 = ((uint32_t)  _raw_press - b3) * (50000 >> OVERSAMPLING);
        p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
        Press = p + ((x1 + x2 + 3791) >> 4);
}

float AP_Baro_BMP085_MPNG::get_pressure()
{
    return Press;
}

float AP_Baro_BMP085_MPNG::get_temperature()
{
    // callers want the temperature in 0.1C units
    return Temp;
}

int32_t AP_Baro_BMP085_MPNG::get_raw_pressure() {
    return _raw_press;
}

int32_t AP_Baro_BMP085_MPNG::get_raw_temp() {
    return _raw_temp;
}

// Send command to Read Pressure
void AP_Baro_BMP085_MPNG::Command_ReadPress()
{
	hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4, 0x34+(OVERSAMPLING << 6));
	_baro_read_timeout = READ_PRESS_TIMEOUT;
}

// Read Raw Pressure values
void AP_Baro_BMP085_MPNG::ReadPress()
{
	uint8_t buf[3];

  hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, sizeof(buf), buf);
	
	_raw_press = (((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2])) >> (8 - OVERSAMPLING);
}

// Send Command to Read Temperature
void AP_Baro_BMP085_MPNG::Command_ReadTemp()
{
	hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4, 0x2E);
	_baro_read_timeout = READ_TEMP_TIMEOUT;
}

// Read Raw Temperature values
void AP_Baro_BMP085_MPNG::ReadTemp()
{
	uint8_t buf[2];
  hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, sizeof(buf), buf);
	_raw_temp = buf[0];
	_raw_temp = (_raw_temp << 8) | buf[1];
}
 
