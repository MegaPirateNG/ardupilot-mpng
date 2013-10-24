# Release Notes for MegaPirateNG 3.1 R1 (based on ArduPilot 3.1-rc4)

## How to compile MegaPirateNG
Follow instructions at: http://www.megapirateng.com/compile-mpng-sources/


## Arduino board pin mapping
Serial ports:
Serial0 (RX0,TX0) - USB, OSD (Minim OSD) or Telemetry (3DR, Xbee, Bluetooth)
Serial1 (RX1,TX1) - OSD which not compatible with Mavlink (Remzibi, E-OSD, FrSky) 
Notice! Serial1 currently not supported!
Serial2 (RX2,TX2) - GPS
Serial3 (RX3,TX3) - OSD (Minim OSD) or Telemetry (3DR, Xbee, Bluetooth)

## Sonar
Currently sonar is not supported

## Motor mapping
Look motor mapping at: http://code.google.com/p/megapirateng/wiki/confighw?wl=en