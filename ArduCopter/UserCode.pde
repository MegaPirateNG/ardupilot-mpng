/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT

const HAL_MPNG& hal_mpng = AP_HAL_BOARD_DRIVER;

void userhook_init()
{
  hal_mpng.uartD->begin(38400);
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
  hal_mpng.uartD->print("$LED,");
  hal_mpng.uartD->print(current_loc.alt/100,DEC);
  hal_mpng.uartD->print(",");
  hal_mpng.uartD->print(control_mode);
  hal_mpng.uartD->print(",");
  hal_mpng.uartD->print(ahrs.roll_sensor/100,DEC);
  hal_mpng.uartD->print(",");
  hal_mpng.uartD->print(ahrs.pitch_sensor/100,DEC);
  hal_mpng.uartD->print(",");
  hal_mpng.uartD->print(motors.armed());
  hal_mpng.uartD->println("#");
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif