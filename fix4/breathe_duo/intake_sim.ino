#include <Arduino.h>

namespace Pins {
    // Communication
    const uint8_t CAM_INPUT = 2;      // Reads CAM signal from Engine Sim
    const uint8_t MAP_OUTPUT = 3;     // Sends MAP PWM to Engine Sim

    // Tuning Knobs
    const uint8_t THROTTLE_POT = A0;
    const uint8_t VE_LOW_RPM_POT = A1;
    const uint8_t VE_HIGH_RPM_POT = A2;
}

namespace Config {
    const float VE_LOW_RPM_POINT = 1000.0f;
    const float VE_HIGH_RPM_POINT = 5000.0f;
    const float ATMOSPHERIC_PRESSURE_PA = 101325.0f;
}

// Global state variables
float current_rpm = 0.0f;
float throttle_fraction = 0.0f;
float volumetric_efficiency = 0.85;
float manifold_pressure_pa = Config::ATMOSPHERIC_PRESSURE_PA;

void setup() {
    Serial.begin(115200);
    Serial.println("Air Intake & Tuning Module - Booting...");

    pinMode(Pins::CAM_INPUT, INPUT);
    pinMode(Pins::MAP_OUTPUT, OUTPUT);
}

void loop() {
    read_inputs_and_calculate_rpm();
    calculate_ve();
    calculate_map();
    output_map_pwm();
    print_telemetry();

    delay(20);
}

void read_inputs_and_calculate_rpm() {
    // Read throttle position
    throttle_fraction = analogRead(Pins::THROTTLE_POT) / 1023.0;

    // !! CHANGE: Calculate RPM from the CAM signal's period. !!
    // Set a timeout of 100ms. If no pulse is detected, the engine is stopped.
    // This timeout corresponds to a minimum speed of ~300 RPM.
    unsigned long high_duration_us = pulseIn(Pins::CAM_INPUT, HIGH, 100000);

    if (high_duration_us == 0) {
        current_rpm = 0; // Engine is stopped or too slow
    } else {
        // The cam signal is a 50% duty cycle square wave.
        // The period of one cycle (high + low) corresponds to 180 cam degrees.
        // 180 cam degrees = 360 crank degrees = 1 full crank revolution.
        unsigned long period_us = high_duration_us * 2;
        
        // RPM = (60 seconds * 1,000,000 microseconds/second) / period_in_microseconds
        current_rpm = 60000000.0f / period_us;
    }
}

void calculate_ve() {
    // This function is unchanged. It uses the newly calculated RPM.
    float ve_at_low_rpm = map(analogRead(Pins::VE_LOW_RPM_POT), 0, 1023, 70, 100) / 100.0f;
    float ve_at_high_rpm = map(analogRead(Pins::VE_HIGH_RPM_POT), 0, 1023, 70, 100) / 100.0f;

    if (current_rpm <= Config::VE_LOW_RPM_POINT) {
        volumetric_efficiency = ve_at_low_rpm;
    } else if (current_rpm >= Config::VE_HIGH_RPM_POINT) {
        volumetric_efficiency = ve_at_high_rpm;
    } else {
        float rpm_fraction = (current_rpm - Config::VE_LOW_RPM_POINT) / (Config::VE_HIGH_RPM_POINT - Config::VE_LOW_RPM_POINT);
        volumetric_efficiency = ve_at_low_rpm + rpm_fraction * (ve_at_high_rpm - ve_at_low_rpm);
    }
}

void calculate_map() {
    // This function is unchanged.
    float base_map = Config::ATMOSPHERIC_PRESSURE_PA * (throttle_fraction + (1.0 - throttle_fraction) * 0.3);
    float vacuum_effect = (current_rpm / 8000.0f) * 70000.0f * (1.0 - throttle_fraction);
    float target_map = base_map - vacuum_effect;

    manifold_pressure_pa = 0.90 * manifold_pressure_pa + 0.10 * constrain(target_map, 25000.0, Config::ATMOSPHERIC_PRESSURE_PA);
}

void output_map_pwm() {
    // This function is unchanged.
    int map_pwm_val = map(manifold_pressure_pa, 25000, 101325, 0, 255);
    analogWrite(Pins::MAP_OUTPUT, constrain(map_pwm_val, 0, 255));
}

void print_telemetry() {
    // This function is unchanged.
    static unsigned long last_print = 0;
    if (millis() - last_print > 200) {
        last_print = millis();
        Serial.print("RPM: "); Serial.print(current_rpm, 0);
        Serial.print(" | Thr: "); Serial.print(throttle_fraction * 100.0, 0);
        Serial.print("% | VE: "); Serial.print(volumetric_efficiency * 100.0, 1);
        Serial.print("% | MAP: "); Serial.print(manifold_pressure_pa / 1000.0, 1);
        Serial.println(" kPa");
    }
}
