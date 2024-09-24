# IOT AC Controller Arduino
Companion repository for [iot-ac-controller](https://github.com/derekpock/iot-ac-controller), containing various Arduino sketchbooks for various IoT capability!

This repo is not super well organized, but some notes to get started:
1. Copy the [secrets.example.hpp](secrets.example.hpp) file into the sketchbook folder you want to use, and rename to `secrets.hpp`. Change the values as needed.
2. In general, the four folders in this repo are a progression of new functionality added for different configurations. Starting with the oldest code:
   1. iotBumpSensor - This is for Arduino Nano 33 IoT devices and only involves an external thermistor and resistor for temperature measurement. Some extra, unneeded accelerometer code remains as well.
   2. esp8266Temp - First rendition of a raw esp8266 with a thermistor and resistor for temperature measurement. This is mostly the iotBumpSensor adapted for a different Wi-Fi scheme.
   3. esp8266TempOled - Improvements for esp8266 to also report info via an attached OLED display.
   4. esp8266TempOledIr - Further improvements that adds an IR transmitter in addition to the existing OLED and thermistor-resistor sensors. This IR is used to turn an Whynter portable AC unit on/off depending on responses from the ArduinoServer in iot-ac-controller.