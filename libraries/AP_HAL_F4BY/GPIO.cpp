/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4BY

#include "GPIO.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* F4BY headers */
#include <drivers/drv_led.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_gpio.h>
#include <modules/px4iofirmware/protocol.h>
#include <arch/board/board.h>
#include <board_config.h>

#define LOW     0
#define HIGH    1

extern const AP_HAL::HAL& hal;

using namespace F4BY;

F4BYGPIO::F4BYGPIO()
{}

void F4BYGPIO::init()
{
#ifdef CONFIG_ARCH_BOARD_F4BY
    _led_fd = open(LED_DEVICE_PATH, O_RDWR);
    if (_led_fd == -1) {
        hal.scheduler->panic("Unable to open " LED_DEVICE_PATH);
    }
    if (ioctl(_led_fd, LED_OFF, LED_BLUE) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO LED BLUE\n");
    }
    if (ioctl(_led_fd, LED_OFF, LED_RED) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED RED\n");
    }
    if (ioctl(_led_fd, LED_OFF, LED_GREEN) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED RED\n");
    }
    if (ioctl(_led_fd, LED_OFF, LED_YELLOW) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED RED\n");
    }
#endif
    _tone_alarm_fd = open("/dev/tone_alarm", O_WRONLY);
    if (_tone_alarm_fd == -1) {
        hal.scheduler->panic("Unable to open /dev/tone_alarm");
    }

    _gpio_fmu_fd = open(F4BY_DEVICE_PATH, 0);
    if (_gpio_fmu_fd == -1) {
        hal.scheduler->panic("Unable to open GPIO");
    }
#ifdef CONFIG_ARCH_BOARD_F4BY
    if (ioctl(_gpio_fmu_fd, GPIO_CLEAR, GPIO_EXT_1) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO_1\n");
    }
#endif

    // also try to setup for the relay pins on the IO board
    _gpio_io_fd = open(F4BY_DEVICE_PATH, O_RDWR);
    if (_gpio_io_fd == -1) {
        hal.console->printf("GPIO: Unable to open px4io\n");
    }
}

void F4BYGPIO::pinMode(uint8_t pin, uint8_t output)
{
    switch (pin) {
    case F4BY_GPIO_FMU_SERVO_PIN(0) ... F4BY_GPIO_FMU_SERVO_PIN(7):
        ioctl(_gpio_fmu_fd, output?GPIO_SET_OUTPUT:GPIO_SET_INPUT, 1U<<(pin-F4BY_GPIO_FMU_SERVO_PIN(0)));
        break;
    }
}

int8_t F4BYGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t F4BYGPIO::read(uint8_t pin) {
    uint32_t relays = 0;
    switch (pin) {

#ifdef GPIO_EXT_1
        case F4BY_GPIO_EXT_FMU_RELAY1_PIN:
            ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & GPIO_EXT_1)?HIGH:LOW;
#endif

#ifdef GPIO_EXT_2
        case F4BY_GPIO_EXT_FMU_RELAY2_PIN:
            ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & GPIO_EXT_2)?HIGH:LOW;
            break;
#endif

#ifdef F4BYIO_P_SETUP_RELAYS_POWER1
        case F4BY_GPIO_EXT_IO_RELAY1_PIN:
            ioctl(_gpio_io_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & F4BYIO_P_SETUP_RELAYS_POWER1)?HIGH:LOW;
#endif

#ifdef F4BYIO_P_SETUP_RELAYS_POWER2
        case F4BY_GPIO_EXT_IO_RELAY2_PIN:
            ioctl(_gpio_io_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & F4BYIO_P_SETUP_RELAYS_POWER2)?HIGH:LOW;
#endif

#ifdef F4BYIO_P_SETUP_RELAYS_ACC1
        case F4BY_GPIO_EXT_IO_ACC1_PIN:
            ioctl(_gpio_io_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & F4BYIO_P_SETUP_RELAYS_ACC1)?HIGH:LOW;
#endif

#ifdef F4BYIO_P_SETUP_RELAYS_ACC2
        case F4BY_GPIO_EXT_IO_ACC2_PIN:
            ioctl(_gpio_io_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & F4BYIO_P_SETUP_RELAYS_ACC2)?HIGH:LOW;
#endif

    case F4BY_GPIO_FMU_SERVO_PIN(0) ... F4BY_GPIO_FMU_SERVO_PIN(7): {
        uint32_t v = 0;
        ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&v);
        return (v & (1U<<(pin-F4BY_GPIO_FMU_SERVO_PIN(0))))?HIGH:LOW;
    }
    }
    return LOW;
}

void F4BYGPIO::write(uint8_t pin, uint8_t value)
{
    switch (pin) {

#ifdef CONFIG_ARCH_BOARD_F4BY
        case HAL_GPIO_A_LED_PIN:    // Arming LED
            if (value == LOW) {
                ioctl(_led_fd, LED_ON, LED_RED);
            } else {
                ioctl(_led_fd, LED_OFF, LED_RED);
            }
            break;

        case HAL_GPIO_B_LED_PIN:    // not used yet 
            if (value == LOW) { 
                ioctl(_led_fd, LED_ON, LED_GREEN);
            } else { 
                ioctl(_led_fd, LED_OFF, LED_GREEN);
            }
            break;

        case HAL_GPIO_C_LED_PIN:    // GPS LED 
            if (value == LOW) { 
                ioctl(_led_fd, LED_ON, LED_BLUE);
            } else { 
                ioctl(_led_fd, LED_OFF, LED_BLUE);
            }
            break;

        case HAL_GPIO_D_LED_PIN:    // GPS LED 
            if (value == LOW) { 
                ioctl(_led_fd, LED_ON, LED_YELLOW);
            } else { 
                ioctl(_led_fd, LED_OFF, LED_YELLOW);
            }
            break;            
#endif

        case F4BY_GPIO_PIEZO_PIN:    // Piezo beeper 
            if (value == LOW) { // this is inverted 
                ioctl(_tone_alarm_fd, TONE_SET_ALARM, 3);    // Alarm on !! 
                //::write(_tone_alarm_fd, &user_tune, sizeof(user_tune));
            } else { 
                ioctl(_tone_alarm_fd, TONE_SET_ALARM, 0);    // Alarm off !! 
            }
            break;

#ifdef GPIO_EXT_1
        case F4BY_GPIO_EXT_FMU_RELAY1_PIN:
            ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_EXT_1);
            break;
#endif

#ifdef GPIO_EXT_2
        case F4BY_GPIO_EXT_FMU_RELAY2_PIN:
            ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_EXT_2);
            break;
#endif

#ifdef F4BYIO_P_SETUP_RELAYS_POWER1
        case F4BY_GPIO_EXT_IO_RELAY1_PIN:
            ioctl(_gpio_io_fd, value==LOW?GPIO_CLEAR:GPIO_SET, F4BYIO_P_SETUP_RELAYS_POWER1);
            break;
#endif

#ifdef F4BYIO_P_SETUP_RELAYS_POWER2
        case F4BY_GPIO_EXT_IO_RELAY2_PIN:
            ioctl(_gpio_io_fd, value==LOW?GPIO_CLEAR:GPIO_SET, F4BYIO_P_SETUP_RELAYS_POWER2);
            break;
#endif

#ifdef F4BYIO_P_SETUP_RELAYS_ACC1
        case F4BY_GPIO_EXT_IO_ACC1_PIN:
            ioctl(_gpio_io_fd, value==LOW?GPIO_CLEAR:GPIO_SET, F4BYIO_P_SETUP_RELAYS_ACC1);
            break;
#endif

#ifdef F4BYIO_P_SETUP_RELAYS_ACC2
        case F4BY_GPIO_EXT_IO_ACC2_PIN:
            ioctl(_gpio_io_fd, value==LOW?GPIO_CLEAR:GPIO_SET, F4BYIO_P_SETUP_RELAYS_ACC2);
            break;
#endif

    case F4BY_GPIO_FMU_SERVO_PIN(0) ... F4BY_GPIO_FMU_SERVO_PIN(7):
        ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, 1U<<(pin-F4BY_GPIO_FMU_SERVO_PIN(0)));
        break;
    }
}

void F4BYGPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* F4BYGPIO::channel(uint16_t n) {
    return new F4BYDigitalSource(0);
}

/* Interrupt interface: */
bool F4BYGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    return true;
}

/*
  return true when USB connected
 */
bool F4BYGPIO::usb_connected(void)
{
    return stm32_gpioread(GPIO_OTGFS_VBUS);
}


F4BYDigitalSource::F4BYDigitalSource(uint8_t v) :
    _v(v)
{}

void F4BYDigitalSource::mode(uint8_t output)
{}

uint8_t F4BYDigitalSource::read() {
    return _v;
}

void F4BYDigitalSource::write(uint8_t value) {
    _v = value;
}

void F4BYDigitalSource::toggle() {
    _v = !_v;
}

#endif // CONFIG_HAL_BOARD
