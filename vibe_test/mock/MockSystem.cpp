#include "MockSystem.h"

namespace MockSystem {
    uint64_t virtual_micros = 0;
    std::map<uint8_t, bool> digital_pins;
    std::map<uint8_t, int> analog_pins;
    std::map<uint8_t, int> pwm_pins;
    std::map<uint8_t, std::vector<PinChangeListener>> listeners;

    void set_digital_pin(uint8_t pin, bool val) {
        if (digital_pins[pin] != val) {
            digital_pins[pin] = val;
            for (auto& listener : listeners[pin]) {
                listener(pin, val);
            }
        }
    }

    bool get_digital_pin(uint8_t pin) {
        return digital_pins[pin];
    }

    void set_analog_pin(uint8_t pin, int val) {
        analog_pins[pin] = val;
    }

    int get_analog_pin(uint8_t pin) {
        return analog_pins[pin];
    }

    void add_listener(uint8_t pin, PinChangeListener listener) {
        listeners[pin].push_back(listener);
    }

    void advance_time(uint32_t us) {
        virtual_micros += us;
    }
}
