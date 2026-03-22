#pragma once
#include <cstdint>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <algorithm>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef uint8_t byte;

constexpr int ARD_INPUT        = 0;
constexpr int ARD_OUTPUT       = 1;
constexpr int ARD_INPUT_PULLUP = 2;
#define ARD_HIGH          1
#define ARD_LOW           0

#ifndef INPUT
#define INPUT ARD_INPUT
#endif
#ifndef OUTPUT
#define OUTPUT ARD_OUTPUT
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP ARD_INPUT_PULLUP
#endif
#ifndef HIGH
#define HIGH ARD_HIGH
#endif
#ifndef LOW
#define LOW ARD_LOW
#endif

#define A0 ((uint8_t)14)
#define A1 ((uint8_t)15)
#define A2 ((uint8_t)16)
#define A3 ((uint8_t)17)
#define A4 ((uint8_t)18)
#define A5 ((uint8_t)19)

namespace MockHW {
    inline std::atomic<int> digital_io[20];
    inline std::atomic<int> analog_in[6];
    inline std::atomic<int> pwm_out[20];

    inline void init() {
        for(int i=0; i<20; i++) digital_io[i].store(0);
        for(int i=0; i<6; i++) analog_in[i].store(0);
        for(int i=0; i<20; i++) pwm_out[i].store(0);
    }
}

inline void pinMode     (uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, int val) { if (pin < 20) MockHW::digital_io[pin].store(val); }
inline int  digitalRead (uint8_t pin)           { return (pin < 20) ? MockHW::digital_io[pin].load() : 0; }
inline void analogWrite (uint8_t pin, int val)  { if (pin < 20) MockHW::pwm_out[pin].store(val); }
inline int  analogRead  (uint8_t pin) {
    int i = (int)pin - 14;
    return (i >= 0 && i < 6) ? MockHW::analog_in[i].load() : 0;
}

namespace _ArduinoTime {
    inline std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
}
inline unsigned long millis() {
    using namespace std::chrono;
    return (unsigned long)duration_cast<milliseconds>(steady_clock::now() - _ArduinoTime::t0).count();
}
inline unsigned long micros() {
    using namespace std::chrono;
    return (unsigned long)duration_cast<microseconds>(steady_clock::now() - _ArduinoTime::t0).count();
}

template<typename T>
inline T constrain(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }

template<typename T, typename U>
inline T map(T x, T in_min, T in_max, U out_min, U out_max) {
    return (T)((double)(x - in_min) * (double)(out_max - out_min) / (double)(in_max - in_min) + (double)out_min);
}

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
