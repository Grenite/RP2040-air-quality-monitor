# Firmware for the RP2040 Air Quality Monitor
Hey there! This repo contains the source code used for the [Air Quality Monitor](https://www.hackster.io/norm10/raspberry-pi-pico-powered-air-quality-monitor-8972e0) project found on Hackster Io. Feel free the browse and get inspired by the code I wrote.

## About this firmware
Uses the Raspberry Pi Pico framework + FreeRTOS to get air quality data from the Adafruit sensors and display them to the ILI9341 touch display. Touch is also enabled via the TSC2007 driver to toggle through the data pages.

# Building
This is what I used to build the firmware:
- Visual Studio Code IDE
  - Raspberry Pi Pico Extension (Under development as of writing - a lot might have changed by now!)
    - V2.0.0 SDK for RP2040
  - Cmake Extension
- I used the cmake extension instead of the Pico extension to configure and compile the firmware, once you have the extensions installed and configured the projected, it should be as simple as build and program  

# References
This firmware wouldn't be possible without the following libraries: 
- [UGFX Graphics library](https://git.ugfx.io/uGFX/uGFX) (commit bb88845622b66265cf56b0ae9de759935cc142b4)
- [Bosch BSEC2 Sensor Library for BME688](https://github.com/boschsensortec/Bosch-BSEC2-Library.git) (commit 24eccb80f1f1cbafcc7121f134b075fe9b37902c) 
