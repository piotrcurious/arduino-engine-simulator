#pragma once
// =============================================================
// mock_arduino.h  –  Minimal Arduino API shim for Linux/FLTK
// All state lives as C++17 inline variables; no separate .cpp needed.
// =============================================================
#include <cstdint>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <algorithm>

// ---- primitive types ----------------------------------------
typedef uint8_t byte;

// ---- pin constants ------------------------------------------
constexpr int INPUT        = 0;
constexpr int OUTPUT       = 1;
constexpr int INPUT_PULLUP = 2;

#define HIGH          1
#define LOW           0

// ---- analog pin aliases (Arduino Uno mapping) ---------------
#define A0 ((uint8_t)14)
#define A1 ((uint8_t)15)
#define A2 ((uint8_t)16)
#define A3 ((uint8_t)17)
#define A4 ((uint8_t)18)
#define A5 ((uint8_t)19)

// ---- hardware state (C++17 inline: one shared instance) -----
namespace MockHW {
    inline int digital_io[20] = {};   // digital pins 0-19 (in/out)
    inline int analog_in[6]   = {};   // A0-A5 (index = pin-14)
    inline int pwm_out[20]    = {};   // analogWrite targets
}

// ---- digital I/O --------------------------------------------
inline void pinMode     (uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, int val) { if (pin < 20) MockHW::digital_io[pin] = val; }
inline int  digitalRead (uint8_t pin)           { return (pin < 20) ? MockHW::digital_io[pin] : 0; }

// ---- analog I/O ---------------------------------------------
inline void analogWrite (uint8_t pin, int val)  { if (pin < 20) MockHW::pwm_out[pin] = val; }
inline int  analogRead  (uint8_t pin) {
    int i = (int)pin - 14;
    return (i >= 0 && i < 6) ? MockHW::analog_in[i] : 0;
}

// ---- time ---------------------------------------------------
namespace _ArduinoTime {
    inline std::chrono::steady_clock::time_point t0 =
        std::chrono::steady_clock::now();
}
inline unsigned long millis() {
    using namespace std::chrono;
    return (unsigned long)duration_cast<milliseconds>(
        steady_clock::now() - _ArduinoTime::t0).count();
}
inline unsigned long micros() {
    using namespace std::chrono;
    return (unsigned long)duration_cast<microseconds>(
        steady_clock::now() - _ArduinoTime::t0).count();
}

// ---- math helpers -------------------------------------------
// constrain: all same type
template<typename T>
inline T constrain(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }

// map: two-type variant so map(float,float,float,int,int) works
template<typename T, typename U>
inline T map(T x, T in_min, T in_max, U out_min, U out_max) {
    return (T)((double)(x - in_min) * (double)(out_max - out_min)
               / (double)(in_max - in_min) + (double)out_min);
}

// ---- Serial stub --------------------------------------------
struct _SerialClass {
    void begin(long) {}
    void print  (const char* s, int = -1) { fputs(s, stdout); }
    void print  (float v, int p = 1)      { printf("%.*f", p, (double)v); }
    void print  (int   v, int = -1)       { printf("%d", v); }
    void println(const char* s, int = -1) { puts(s); }
    void println(float v, int p = 1)      { printf("%.*f\n", p, (double)v); }
    void println(int   v, int = -1)       { printf("%d\n", v); }
};
inline _SerialClass Serial;
