#pragma once
// TimerOne stub: the 1 ms tick is handled by the simulation thread in main.cpp.
// This header exists only to satisfy the #include in the original sketch.
struct _Timer1Class {
    void initialize(unsigned long /*period_us*/) {}
    void attachInterrupt(void (*)())              {}
    void detachInterrupt()                        {}
    void setPeriod(unsigned long)                 {}
    void start()                                  {}
    void stop()                                   {}
};
inline _Timer1Class Timer1;
