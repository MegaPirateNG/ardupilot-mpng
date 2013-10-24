// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Example code for the AP_HAL AVRUARTDriver, based on FastSerial
//
// This code is placed into the public domain.
//

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup(void)
{
    hal.uartA->begin(115200);
    
    //
    // Test printing things
    //
        
    hal.uartA->print("test");
    hal.uartA->println(" begin");
    hal.uartA->println(1000);
    hal.uartA->println(1000, 8);
    hal.uartA->println(1000, 10);
    hal.uartA->println(1000, 16);
    hal.uartA->println_P(PSTR("progmem"));
    int x = 3;
    int *ptr = &x;
    hal.uartA->printf("printf %d %u %#x %p %f %s\n", -1000, 1000, 1000, ptr, 1.2345, PSTR("progmem"));
    hal.uartA->printf_P(PSTR("printf_P %d %u %#x %p %f %s\n"), -1000, 1000, 1000, ptr, 1.2345, PSTR("progmem"));
    hal.uartA->println("done");
}

void loop(void)
{
    int c;
    //
    // Perform a simple loopback operation.
    //
    c = hal.uartA->read();
    if (-1 != c)
        hal.uartA->write(c);
}

AP_HAL_MAIN();
