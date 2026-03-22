#ifndef MOCK_SYSTEM_H
#define MOCK_SYSTEM_H

#include <stdint.h>
#include <map>
#include <vector>
#include <functional>

namespace MockSystem {
    // Current virtual time in microseconds
    extern uint64_t virtual_micros;

    // Pin states
    extern std::map<uint8_t, bool> digital_pins;
    extern std::map<uint8_t, int> analog_pins;
    extern std::map<uint8_t, int> pwm_pins;

    // Listeners for pin changes
    typedef std::function<void(uint8_t, bool)> PinChangeListener;
    extern std::map<uint8_t, std::vector<PinChangeListener>> listeners;

    void set_digital_pin(uint8_t pin, bool val);
    bool get_digital_pin(uint8_t pin);
    void set_analog_pin(uint8_t pin, int val);
    int get_analog_pin(uint8_t pin);
    void add_listener(uint8_t pin, PinChangeListener listener);

    void advance_time(uint32_t us);
}

#endif
