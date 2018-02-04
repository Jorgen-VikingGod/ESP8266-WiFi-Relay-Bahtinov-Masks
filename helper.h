/*
 * helper.h
 * ----------------------------------------------------------------------------
 * helper methods and defines
 * ----------------------------------------------------------------------------
 * Source:     https://github.com/Jorgen-VikingGod/ESP8266-WiFi-Relay-Bahtinov-Masks
 * Copyright:  Copyright (c) 2017 Juergen Skrotzky
 * Author:     Juergen Skrotzky <JorgenVikingGod@gmail.com>
 * License:    MIT License
 * Created on: 29.Sep. 2017
 * ----------------------------------------------------------------------------
 */

#ifndef _HELPER_H
#define _HELPER_H

/*
 * Debug mode
 */
#define _debug 1

/*
 * mask servo container
 * ----------------------------------------------------------------------------
 */
struct sServo {
  uint8_t pin;
  uint8_t open;
  uint8_t close;
  uint8_t state;
  bool sweep;
  uint16_t pulseMin;
  uint16_t pulseMax;
  sServo(uint8_t servoPin, uint8_t servoOpen = 0, uint8_t servoClose = 180, uint16_t servoPulseMin = 130, uint16_t servoPulseMax = 150, uint8_t servoState = 1, bool servoSweep = false) {
    pin = servoPin;
    open = servoOpen;
    close = servoClose;
    pulseMin = servoPulseMin;
    pulseMax = servoPulseMax;
    state = servoState;
    sweep = servoSweep;
  }
  uint16_t currentPulseLength() {
    uint8_t pos = (state ? open : close);
    return pulseLength(pos);
  }
  uint16_t pulseLength(uint8_t pos) {
    return map(pos, 0, 180, pulseMin, pulseMax);
  }
};

/*
 * relay container
 * ----------------------------------------------------------------------------
 */
struct sRelay {
  uint8_t pin;
  uint8_t type;
  uint8_t state;
  uint16_t pulse;
  sRelay(uint8_t relayPin, uint8_t relayType = HIGH, uint8_t relayState = LOW, uint16_t relayPulse = 500) {
    pin = relayPin;
    type = relayType;
    state = relayState;
    pulse = relayPulse;
  }
};

/*
 * format ip address as String
 * ----------------------------------------------------------------------------
 */
String printIP(IPAddress adress) {
  return (String)adress[0] + "." + (String)adress[1] + "." + (String)adress[2] + "." + (String)adress[3];
}

/*
 * debug messages
 * ----------------------------------------------------------------------------
 */
template <typename... Args>
void DEBUG_PRINTF(const char *format, Args &&...args) {
  if (_debug) {
    Serial.printf(format, args...);
  }
}
template <typename Generic>
void DEBUG_PRINT(Generic text) {
  if (_debug) {
    Serial.print(text);    
  }
}
template <typename Generic, typename Format>
void DEBUG_PRINT(Generic text, Format format) {
  if (_debug) {
    Serial.print(text, format);
  }
}
template <typename Generic>
void DEBUG_PRINTLN(Generic text) {
  if (_debug) {
    Serial.println(text);    
  }
}
template <typename Generic, typename Format>
void DEBUG_PRINTLN(Generic text, Format format) {
  if (_debug) {
    Serial.println(text, format);
  }
}

#endif // _HELPER_H

