/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_BMP085_MPNG_H__
#define __AP_BARO_BMP085_MPNG_H__

#include <AP_HAL.h>
#include "AP_Baro.h"

class AP_Baro_BMP085_MPNG : public AP_Baro
{
public:
    AP_Baro_BMP085_MPNG(){
    	healthy = false;
    };

    /* AP_Baro public interface: */
    bool            init();
    uint8_t         read();
    float           get_pressure(); // in mbar*100 units
    float           get_temperature(); // in celsius degrees * 100 units

    int32_t         get_raw_pressure();
    int32_t         get_raw_temp();

    void            _calculate();

private:
    static AP_HAL::Semaphore *_i2c_sem;

    /* Asynchronous handler functions: */
    static void                     _update(uint32_t );
		static void _poll_data(uint32_t now);
		static void _read_data_from_timerprocess();
		static void _read_data_transaction();
	
    /* Asynchronous state: */
    static volatile bool            _updated;
    static volatile uint8_t         _d1_count;
    static volatile uint8_t         _d2_count;
    static volatile uint32_t        _s_D1, _s_D2;
    
    static uint8_t                  _state;
    static uint32_t                 _timer;
    /* Gates access to asynchronous state: */
    static bool                     _sync_access;

    float                           Temp;
    float                           Press;
		static void 	Command_ReadPress();
		static void	ReadPress();
		static void	Command_ReadTemp();
		static void	ReadTemp();


    static int32_t                         _raw_press;
    static int32_t                         _raw_temp;
    // Internal calibration registers
    int16_t                         ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t                        ac4, ac5, ac6;
};

#endif //  __AP_BARO_BMP085_MPNG_H__
