#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// --- Pin Definitions ---
// The dedicated RPM_OUTPUT pin has been removed.
namespace Pins {
    constexpr uint8_t MAP_INPUT = 2;   // Reads PWM from Air Intake Sim

    constexpr uint8_t CAM_POS = 4;
    constexpr uint8_t CRANK_POS = 5;
    constexpr uint8_t INJ[] = {A0, A1, A2, A3};
    constexpr uint8_t IGN[] = {6, 7, 8, 10};
    constexpr uint8_t LAMBDA = 11;
    constexpr uint8_t START_BUTTON = 12;
    constexpr uint8_t FORCE_ADC = A5;
}

// Assume constants are in a header file for brevity (no changes from previous version)
#include "FullSimulatorConstants.h" 

class EngineSimulator {
public:
    void setup();
    void update();
    void write_telemetry();

private:
    void read_external_inputs();
    void measure_ecu_inputs();
    void update_dynamics();
    void update_outputs();
    float dps_to_rpm(float dps) { return dps / 6.0f; }

    // State Variables
    // ... (All state variables are the same) ...
    float manifold_pressure_pa = 101325.0f;
};

// --- Class Implementation ---

void EngineSimulator::setup() {
    pinMode(Pins::MAP_INPUT, INPUT);
    pinMode(Pins::CAM_POS, OUTPUT);
    pinMode(Pins::CRANK_POS, OUTPUT);
    // ... (rest of setup is unchanged) ...
}

void EngineSimulator::update() {
    read_external_inputs(); // Read the MAP value first
    measure_ecu_inputs();
    update_dynamics();
    update_outputs();
}

void EngineSimulator::read_external_inputs() {
    // Reads the MAP value from the tuner module via PWM.
    unsigned long map_pulse_us = pulseIn(Pins::MAP_INPUT, HIGH, 3000);
    manifold_pressure_pa = map(map_pulse_us, 0, 2040, 25000, 101325);
    manifold_pressure_pa = constrain(manifold_pressure_pa, 25000, 101325);
}

void EngineSimulator::update_dynamics() {
    // This function is completely unchanged. It calculates torque based on the
    // externally-provided MAP value and its own internal state.
    // ...
}

void EngineSimulator::update_outputs() {
    // !! CHANGE: The logic to send RPM via PWM has been removed. !!

    // The rest of the outputs are generated as before.
    int lambda_pwm = map(clamp(lambda_overall, 0.5f, 1.5f), 0.5f, 1.5f, 0, 255);
    analogWrite(Pins::LAMBDA, lambda_pwm);

    float cam_angle_deg = fmod(flywheel_angle_deg / 2.0f, 360.0f);
    bool cam_pos_state = (fmod(cam_angle_deg, 180.0f) < 90.0f);
    digitalWrite(Pins::CAM_POS, cam_pos_state);

    int tooth = floor(flywheel_angle_deg / 6.0f);
    bool crank_state = (fmod(flywheel_angle_deg, 6.0f) < 3.0f) && (tooth < 58);
    digitalWrite(Pins::CRANK_POS, crank_state);
}

// ... (The rest of the class, ISR, and main loop remain unchanged) ...
