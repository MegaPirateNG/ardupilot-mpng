#include "AP_InertialSensor_ITG3200.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// *********************
// I2C general functions
// *********************
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1); 

#define ITG3200_ADDRESS  	0x68 // 0xD0

// accelerometer scaling
#define ACCEL_SCALE_1G    	(GRAVITY_MSS / 2730.0)


#define GYRO_SMPLRT_50HZ 19 // 1KHz/(divider+1)
#define GYRO_SMPLRT_100HZ 9 // 1KHz/(divider+1)
#define GYRO_SMPLRT_200HZ 4 // 1KHz/(divider+1)

#define GYRO_DLPF_CFG_5HZ 6
#define GYRO_DLPF_CFG_10HZ 5
#define GYRO_DLPF_CFG_20HZ 4
#define GYRO_DLPF_CFG_42HZ 3
#define GYRO_DLPF_CFG_98HZ 2

// ITG-3200 14.375 LSB/degree/s
const float AP_InertialSensor_ITG3200::_gyro_scale = 0.0012141421; // ToRad(1/14.375)

uint8_t AP_InertialSensor_ITG3200::_gyro_data_index[3] = { 1, 2, 0 };
uint8_t AP_InertialSensor_ITG3200::_accel_data_index[3] = { 4, 5, 6 };
int8_t AP_InertialSensor_ITG3200::_gyro_data_sign[3] = { 1, 1, -1 };
int8_t AP_InertialSensor_ITG3200::_accel_data_sign[3] = { 1, 1, -1 };

const uint8_t AP_InertialSensor_ITG3200::_temp_data_index = 3;

uint8_t AP_InertialSensor_ITG3200::_accel_addr;

/* Static I2C device driver */
AP_HAL::Semaphore* AP_InertialSensor_ITG3200::_i2c_sem = NULL;

static volatile uint32_t _ins_timer = 0;

AP_InertialSensor_ITG3200::AP_InertialSensor_ITG3200(uint8_t board_type): AP_InertialSensor()
{
	_initialised = false;

	// HK MultiWii and BV has default chip configuration
	// other boards must be described here
/*	if (board_type == HK_RED_MULTIWII_PRO || board_type == BLACK_VORTEX) {
		_gyro_data_index[0]  =  1;
		_gyro_data_index[1]  =  2;
		_gyro_data_index[2]  =  0;
		_gyro_data_sign[0]   = 1;
		_gyro_data_sign[1]   = 1;
		_gyro_data_sign[2]   = -1;
	
		_accel_data_index[0] = 4;
		_accel_data_index[1] = 5;
		_accel_data_index[2] = 6;
		_accel_data_sign[0]  = 1;
		_accel_data_sign[1]  = 1;
		_accel_data_sign[2]  = -1;
	}*/

	if (board_type == BLACK_VORTEX) {
		_accel_addr = 0x41;
	} else {
		_accel_addr = 0x40;
	} 
}

uint16_t AP_InertialSensor_ITG3200::_init_sensor(Sample_rate sample_rate)
{
	if (_initialised) return 1;
		_initialised = true;
		
	_i2c_sem = hal.i2c->get_semaphore();

    hal.scheduler->suspend_timer_procs();

    uint8_t tries = 0;
    do {
        bool success = hardware_init(sample_rate);
        if (success) {
        	break;
        } else {
            hal.scheduler->delay(50); // delay for 50ms
                hal.console->println_P(
                        PSTR("ITG3200 init failed"));
        }
        if (tries++ > 5) {
            hal.scheduler->panic(PSTR("PANIC: failed to boot ITG3200 5 times")); 
        }
    } while (1);

    hal.scheduler->resume_timer_procs();


    /* read the first lot of data.
     * _read_data_transaction requires the spi semaphore to be taken by
     * its caller. */
    _ins_timer = hal.scheduler->micros();

    // start the timer process to read samples
    hal.scheduler->register_timer_process(_poll_data);
	return 1;
	
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];

// how many values we've accumulated since last read
static volatile uint16_t _count;

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

void AP_InertialSensor_ITG3200::wait_for_sample()
{
    uint32_t tstart = hal.scheduler->micros();
    while (num_samples_available() == 0) {
        uint32_t now = hal.scheduler->micros();
        uint32_t dt = now - tstart;
        if (dt > 50000) {
            hal.scheduler->panic(
                    PSTR("PANIC: AP_InertialSensor_ITG3200::update waited 50ms for data from interrupt"));
        }
    }
}

bool AP_InertialSensor_ITG3200::update( void )
{
	int32_t sum[7];
	float count_scale;
	Vector3f accel_scale = _accel_scale.get();
		
	// wait for at least 1 sample
	wait_for_sample();

	// disable interrupts for mininum time
	hal.scheduler->suspend_timer_procs();
    /** ATOMIC SECTION w/r/t TIMER PROCESS */
    {
        for (int i=0; i<7; i++) {
            sum[i] = _sum[i];
            _sum[i] = 0;
        }

        _num_samples = _count;
        _count = 0;
    }
    hal.scheduler->resume_timer_procs();
	
	count_scale = 1.0f / _num_samples;
	
	_gyro = Vector3f(_gyro_data_sign[0] * sum[_gyro_data_index[0]],
					 _gyro_data_sign[1] * sum[_gyro_data_index[1]],
					 _gyro_data_sign[2] * sum[_gyro_data_index[2]]);
	_gyro.rotate(_board_orientation);
	_gyro *= _gyro_scale * count_scale;
	_gyro -= _gyro_offset;
	
	_accel = Vector3f(_accel_data_sign[0] * sum[_accel_data_index[0]],
					  _accel_data_sign[1] * sum[_accel_data_index[1]],
					  _accel_data_sign[2] * sum[_accel_data_index[2]]);
	
	_accel.rotate(_board_orientation);
    _accel *= count_scale * ACCEL_SCALE_1G;
    _accel.x *= accel_scale.x;
    _accel.y *= accel_scale.y;
    _accel.z *= accel_scale.z;
	_accel -= _accel_offset;
	
	
    _temp    = _temp_to_celsius(sum[_temp_data_index] * count_scale);

   	return true;
}


// get_delta_time returns the time period in seconds overwhich the sensor data was collected
float AP_InertialSensor_ITG3200::get_delta_time() 
{
    // the sensor runs at 200Hz
    return 0.005 * _num_samples;
}

float AP_InertialSensor_ITG3200::_temp_to_celsius ( uint16_t regval )
{
    return 20.0;
}

void AP_InertialSensor_ITG3200::_poll_data(uint32_t now)
{
	if (now - _ins_timer > 5000) {
		_ins_timer = now;
	
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
	                  PSTR("PANIC: AP_InertialSensor_ITG3200::_poll_data "
	                       "failed to take I2C semaphore synchronously"));
	      }
	  }
	}
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_ITG3200::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

// get number of samples read from the sensors
uint16_t AP_InertialSensor_ITG3200::num_samples_available()
{
    return _count; 
}



/*================ HARDWARE FUNCTIONS ==================== */
void AP_InertialSensor_ITG3200::_read_data_from_timerprocess()
{
    static uint8_t semfail_ctr = 0;
    bool got = _i2c_sem->take_nonblocking();
    if (!got) { 
        semfail_ctr++;
        if (semfail_ctr > 100) {
            hal.scheduler->panic(PSTR("PANIC: failed to take I2C semaphore "
                        "100 times in AP_InertialSensor_ITG3200::"
                        "_read_data_from_timerprocess"));
        }
        return;
    } else {
        semfail_ctr = 0;
    }   

    _read_data_transaction();

    _i2c_sem->give();
}

void AP_InertialSensor_ITG3200::_read_data_transaction()
{
	// now read the data
	uint8_t rawGYR[8];
	uint8_t rawACL[6];
	memset(rawGYR,0,8);
	memset(rawACL,0,6);
	
	hal.i2c->readRegisters(ITG3200_ADDRESS, 0X1B, 8, rawGYR);
	_sum[3] += (int16_t)(((uint16_t)rawGYR[0] << 8) | rawGYR[1]);
	_sum[0] += (int16_t)(((uint16_t)rawGYR[6] << 8) | rawGYR[7]);
	_sum[1] += (int16_t)(((uint16_t)rawGYR[4] << 8) | rawGYR[5]);
	_sum[2] += (int16_t)(((uint16_t)rawGYR[2] << 8) | rawGYR[3]);
	
		
	hal.i2c->readRegisters(_accel_addr, 0x02, 6, rawACL);
	_sum[4] += (int16_t)(((uint16_t)rawACL[3] << 8) | rawACL[2]) >> 2;
	_sum[5] += (int16_t)(((uint16_t)rawACL[1] << 8) | rawACL[0]) >> 2;
	_sum[6] += (int16_t)(((uint16_t)rawACL[5] << 8) | rawACL[4]) >> 2;
	
	
	_count++;
	if (_count == 0) {
		// rollover - v unlikely
		memset((void*)_sum, 0, sizeof(_sum));
	}
}

bool AP_InertialSensor_ITG3200::hardware_init(Sample_rate sample_rate)
{

	if (!_i2c_sem->take(100)) {
		hal.scheduler->panic(PSTR("ITG3200: Unable to get semaphore"));
	}
	
	// Chip reset
	hal.scheduler->delay(10);
	hal.i2c->writeRegister(ITG3200_ADDRESS, 0x3E, 0x80);
	hal.scheduler->delay(5);
	hal.i2c->writeRegister(ITG3200_ADDRESS, 0x15, 4);
	hal.scheduler->delay(5);
	hal.i2c->writeRegister(ITG3200_ADDRESS, 0x16, 0x18+3);
	hal.scheduler->delay(5);
	hal.i2c->writeRegister(ITG3200_ADDRESS, 0x3E, 0x03);
	hal.scheduler->delay(10);
			
	hal.i2c->writeRegister(_accel_addr, 0x0D, 1<<4);
	hal.scheduler->delay(1);
	hal.i2c->writeRegister(_accel_addr, 0x35, 3<<1);
	hal.scheduler->delay(1);
	hal.i2c->writeRegister(_accel_addr, 0x20, 0<<4);
	hal.scheduler->delay(10);
	
	_i2c_sem->give();
    return true;
}