
#ifndef __AP_HAL_MPNG_SPI_DEVICES_H__
#define __AP_HAL_MPNG_SPI_DEVICES_H__

#include <AP_HAL.h>
#include "AP_HAL_MPNG_Namespace.h"

class MPNG::AVRSPI0DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    AVRSPI0DeviceDriver(
            MPNG::AVRDigitalSource *cs_pin,
            uint8_t spcr,
            uint8_t spsr
    ) :
        _cs_pin(cs_pin),
        _spcr(spcr),
        _spsr(spsr)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);

    static MPNG::AVRSemaphore _semaphore;

    MPNG::AVRDigitalSource *_cs_pin;
    const uint8_t _spcr;
    const uint8_t _spsr;
};


class MPNG::AVRSPI2DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    AVRSPI2DeviceDriver(
            MPNG::AVRDigitalSource *cs_pin,
            uint8_t ucsr2c,
            uint16_t ubrr2
    ) :
        _cs_pin(cs_pin),
        _ucsr2c(ucsr2c),
        _ubrr2(ubrr2)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);

    static MPNG::AVRSemaphore _semaphore;

    MPNG::AVRDigitalSource *_cs_pin;
    uint8_t _ucsr2c;
    uint16_t _ubrr2;
};

class MPNG::AVRSPI3DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    AVRSPI3DeviceDriver(
            MPNG::AVRDigitalSource *cs_pin,
            uint8_t ucsr3c,
            uint16_t ubrr3
    ) :
        _cs_pin(cs_pin),
        _ucsr3c(ucsr3c),
        _ubrr3(ubrr3)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);
    void _transfer(const uint8_t *data, uint16_t size);
    static MPNG::AVRSemaphore _semaphore;

    MPNG::AVRDigitalSource *_cs_pin;
    uint8_t _ucsr3c;
    uint16_t _ubrr3;

};


#endif // __AP_HAL_MPNG_SPI_DEVICES_H__
