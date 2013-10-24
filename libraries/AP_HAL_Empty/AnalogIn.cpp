#include "AnalogIn.h"

using namespace Empty;

EmptyAnalogSource::EmptyAnalogSource(float v) :
    _v(v)
{}

float EmptyAnalogSource::read_average() {
    return _v;
}

float EmptyAnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float EmptyAnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float EmptyAnalogSource::read_latest() {
    return _v;
}

void EmptyAnalogSource::set_pin(uint8_t p)
{}

void EmptyAnalogSource::set_stop_pin(uint8_t p)
{}

void EmptyAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

EmptyAnalogIn::EmptyAnalogIn()
{}

void EmptyAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* EmptyAnalogIn::channel(int16_t n) {
    return new EmptyAnalogSource(1.11);
}


