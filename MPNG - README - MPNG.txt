Release Notes for MegaPirateNG 3.0.1 R2 (ArduPilot 3.0.1)

IMPORTANT NOTICE!!!
IT3200 gyro doesn't supported in this version!!! Only MPU6050 supported!

=== How to compile MegaPirateNG ===
Follow instructions at: http://www.megapirateng.com/compile-mpng-sources/


=== Arduino board pin mapping ===
Serial ports:
Serial0 (RX0,TX0) - USB, OSD (Minim OSD) or Telemetry (3DR, Xbee, Bluetooth)
Serial1 (RX1,TX1) - OSD which not compatible with Mavlink (Remzibi, E-OSD, FrSky) 
Serial2 (RX2,TX2) - GPS
Serial3 (RX3,TX3) - OSD (Minim OSD) or Telemetry (3DR, Xbee, Bluetooth)

*** Copter LEDs ***
A2-A7

*** Voltage & Current sensor ***
A0 - Voltage
A1 - Current


*** Sonar ***
Currently sonar is not supported

*** Motor mapping ***
Look motor mapping at: http://code.google.com/p/megapirateng/wiki/confighw?wl=en
Version history
*** 3.0.1 R3
Added ability to use Serial1 in UserCode.pde
Added ability to disable internal compass in AP_IntertialSensor_MPU6000_I2C.cpp
Added ability to select MPNG compatible boards in APM_Config.h
Added ability to change Serial2 port speed in APM_Config.h
Fixed Serial2 initialization (RXD pullup)
Fixed GPS driver selecting in APM_Config.h (AC 3.1)
Main loop rate lowered to 100Hz (AC 3.1)
Fixed pin mapping for Copter LEDs and Voltage&Current sensor

*** 3.0.1 R2
Fixed compassmot
Little optimized MPU6050 readings.
Added method to fix bootloader bug (uploading hang).

*** 3.0.1 R1
Initial port of ArduCopter 3.0.1