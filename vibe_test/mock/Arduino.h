#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "MockSystem.h"

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

inline void pinMode(uint8_t pin, uint8_t mode) {}
inline void digitalWrite(uint8_t pin, uint8_t val) { MockSystem::set_digital_pin(pin, val == HIGH); }
inline int digitalRead(uint8_t pin) { return MockSystem::get_digital_pin(pin) ? HIGH : LOW; }
inline int analogRead(uint8_t pin) { return MockSystem::get_analog_pin(pin); }
inline void analogWrite(uint8_t pin, int val) { /* Store somewhere? */ }

inline unsigned long millis() { return (unsigned long)(MockSystem::virtual_micros / 1000); }
inline unsigned long micros() { return (unsigned long)MockSystem::virtual_micros; }
inline void delay(unsigned long ms) { MockSystem::advance_time(ms * 1000); }
inline void delayMicroseconds(unsigned int us) { MockSystem::advance_time(us); }

class Serial_ {
public:
    void begin(unsigned long baud) {}
    void print(const char* s) { printf("%s", s); }
    void print(float f, int p = 2) { printf("%.*f", p, f); }
    void print(int i) { printf("%d", i); }
    void print(unsigned long i) { printf("%lu", i); }
    void println(const char* s = "") { printf("%s\n", s); }
    void println(float f, int p = 2) { printf("%.*f\n", p, f); }
    void println(int i) { printf("%d\n", i); }
    void println(unsigned long i) { printf("%lu\n", i); }
};

extern Serial_ Serial;

#endif
