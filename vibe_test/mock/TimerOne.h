#ifndef TIMERONE_H
#define TIMERONE_H

#include <functional>

class TimerOne {
public:
    void initialize(unsigned long microseconds = 1000000) {
        _period = microseconds;
    }
    void attachInterrupt(void (*isr)()) {
        _isr = isr;
    }
    void detachInterrupt() {
        _isr = nullptr;
    }
    void tick() {
        if (_isr) _isr();
    }
    unsigned long getPeriod() const { return _period; }
private:
    unsigned long _period = 1000000;
    void (*_isr)() = nullptr;
};

extern TimerOne Timer1;

#endif
