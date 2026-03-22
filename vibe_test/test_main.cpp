#include "mock/Arduino.h"
#include "mock/TimerOne.h"
#include "EngineSimulator.h"
#include <iostream>
#include <vector>

// Forward declarations of Arduino functions in engine.cpp
void setup();
void loop();
extern volatile bool run_simulation_tick;

class VirtualECU {
public:
    void update(uint64_t now_us) {
        bool crank = MockSystem::get_digital_pin(Pins::CRANK_POS);
        bool cam = MockSystem::get_digital_pin(Pins::CAM_POS);

        // Simple crank tooth detection
        if (crank && !prev_crank) {
            uint64_t tooth_dt = now_us - last_tooth_us;
            last_tooth_us = now_us;

            if (tooth_dt > last_tooth_dt * 1.5 && last_tooth_dt > 0) {
                // Gap detected! This is tooth 0 (after 58 and 59 are missing)
                tooth_count = 0;
                sync = true;
            } else {
                tooth_count++;
            }
            last_tooth_dt = tooth_dt;

            if (sync) {
                process_tooth(now_us);
            }
        }
        prev_crank = crank;
        prev_cam = cam;

        // Drive injectors and ignition based on state set in process_tooth
        for(int i=0; i<Engine::NUM_CYLINDERS; i++) {
            if (now_us >= inj_end_us[i]) {
                MockSystem::set_digital_pin(Pins::INJ[i], false);
            }
            if (now_us >= ign_end_us[i]) {
                MockSystem::set_digital_pin(Pins::IGN[i], false);
            }
        }
    }

    void process_tooth(uint64_t now_us) {
        // engine.cpp: cam_pos_state = (crank_angle_720_deg < 360.0f);
        if (MockSystem::get_digital_pin(Pins::CAM_POS)) {
             cycle_720_offset = 0;
        } else {
             cycle_720_offset = 360;
        }

        int angle_360 = tooth_count * 6;
        int angle_720 = (angle_360 + cycle_720_offset) % 720;

        // Firing order: 0, 2, 3, 1
        // Starts: 0, 180, 360, 540

        check_and_fire(0, angle_720, 0, now_us);
        check_and_fire(2, angle_720, 180, now_us);
        check_and_fire(3, angle_720, 360, now_us);
        check_and_fire(1, angle_720, 540, now_us);
    }

    void check_and_fire(int cyl_idx, int current_angle, int tdc_angle, uint64_t now_us) {
        // Fire injector at 360 deg before TDC (start of Intake stroke)
        int inj_angle = (tdc_angle - 360 + 720) % 720;
        if (current_angle == inj_angle && !inj_fired[cyl_idx]) {
            MockSystem::set_digital_pin(Pins::INJ[cyl_idx], true);
            inj_end_us[cyl_idx] = now_us + 10000; // 10ms pulse
            inj_fired[cyl_idx] = true;
        } else if (current_angle != inj_angle) {
            inj_fired[cyl_idx] = false;
        }

        // Fire ignition 12 degrees before TDC (exactly 2 teeth)
        int ign_angle = (tdc_angle - 12 + 720) % 720;
        if (current_angle == ign_angle && !ign_fired[cyl_idx]) {
             MockSystem::set_digital_pin(Pins::IGN[cyl_idx], true);
             ign_end_us[cyl_idx] = now_us + 1000; // 1ms pulse
             ign_fired[cyl_idx] = true;
        } else if (current_angle != ign_angle) {
             ign_fired[cyl_idx] = false;
        }
    }

private:
    bool prev_crank = false;
    bool prev_cam = false;
    uint64_t last_tooth_us = 0;
    uint64_t last_tooth_dt = 0;
    int tooth_count = 0;
    bool sync = false;
    int cycle_720_offset = 0;

    uint64_t inj_end_us[Engine::NUM_CYLINDERS] = {0};
    uint64_t ign_end_us[Engine::NUM_CYLINDERS] = {0};
    bool inj_fired[Engine::NUM_CYLINDERS] = {false};
    bool ign_fired[Engine::NUM_CYLINDERS] = {false};
};

int main() {
    setup();
    VirtualECU ecu;

    uint64_t total_steps = 250000; // 50 seconds

    // Hold start button for first 3 seconds
    MockSystem::set_digital_pin(Pins::START_BUTTON, false); // Active LOW
    MockSystem::set_analog_pin(Pins::THROTTLE_ADC, 300); // Idle-ish throttle

    for (uint64_t i = 0; i < total_steps; ++i) {
        if (MockSystem::virtual_micros > 3000000) {
            MockSystem::set_digital_pin(Pins::START_BUTTON, true); // Release
        }

        // Run Timer ISR
        static uint64_t last_timer_tick = 0;
        if (MockSystem::virtual_micros - last_timer_tick >= Timer1.getPeriod()) {
            Timer1.tick();
            last_timer_tick = MockSystem::virtual_micros;
        }

        loop();
        ecu.update(MockSystem::virtual_micros);

        MockSystem::advance_time(200); // 200us per step
    }

    return 0;
}
