// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
  #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar->calculate_scaler(g.sonar_type, 3.3f);
  #else
    sonar->calculate_scaler(g.sonar_type, 5.0f);
  #endif
}
 #endif

static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    barometer.calibrate();
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static int32_t read_barometer(void)
{
    barometer.read();
    return barometer.get_altitude() * 100.0f;
}

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    // exit immediately if sonar is disabled
    if( !g.sonar_enabled ) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar->read();

    if (temp_alt >= sonar->min_distance && temp_alt <= sonar->max_distance * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = cos_pitch_x * cos_roll_x;
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}


#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

static void init_optflow()
{
#if OPTFLOW == ENABLED
    if( optflow.init() == false ) {
        g.optflow_enabled = false;
        cliSerial->print_P(PSTR("\nFailed to Init OptFlow "));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }else{
        // suspend timer while we set-up SPI communication
        hal.scheduler->suspend_timer_procs();

        optflow.set_orientation(OPTFLOW_ORIENTATION);   // set optical flow sensor's orientation on aircraft
        optflow.set_frame_rate(2000);                   // set minimum update rate (which should lead to maximum low light performance
        optflow.set_resolution(OPTFLOW_RESOLUTION);     // set optical flow sensor's resolution
        optflow.set_field_of_view(OPTFLOW_FOV);         // set optical flow sensor's field of view

        // resume timer
        hal.scheduler->resume_timer_procs();
    }
#endif      // OPTFLOW == ENABLED
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
static void read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        compass.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->voltage_average() * 50;
    receiver_rssi = constrain_int16(ret, 0, 255);
}
