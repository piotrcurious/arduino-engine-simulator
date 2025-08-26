#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =================================================================
// == CONFIGURATION & CONSTANTS
// =================================================================

namespace Pins {
    // Communication Pin (must be analog since PWM is filtered)
    constexpr uint8_t MAP_INPUT = A4;   // <-- moved from pin 2 to A4

    // Sensor Outputs
    constexpr uint8_t CAM_POS = 4;
    constexpr uint8_t CRANK_POS = 5;
    constexpr uint8_t LAMBDA = 11;

    // ECU Inputs (must be A0-A3 for this PCINT setup)
    constexpr uint8_t INJ[] = {A0, A1, A2, A3};
    constexpr uint8_t IGN[] = {6, 7, 8, 10};

    // Other Inputs
    constexpr uint8_t START_BUTTON = 12;
    constexpr uint8_t FORCE_ADC = A5;
}

namespace Config {
    constexpr float TIME_STEP_S = 0.001f;
    constexpr float ADC_MAX_VAL = 1023.0f;
    constexpr float ADC_MID_VAL = 511.5f;
}

namespace Physics {
    constexpr float PI = 3.14159265358979323846f;
    constexpr float AIR_GAS_CONSTANT = 287.05f;
    constexpr float INTAKE_TEMP_K = 293.15f;
    constexpr float FLYWHEEL_MASS_KG = 10.0f;
    constexpr float FLYWHEEL_RADIUS_M = 0.1f;
    constexpr float FLYWHEEL_INERTIA = FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;
    constexpr float VISCOUS_FRICTION_COEFF = 0.002f;
    constexpr float STATIC_FRICTION_NM = 0.05f;
    constexpr float ENGINE_BRAKE_FACTOR = 4.0f;
}

namespace Engine {
    constexpr int NUM_CYLINDERS = 4;
    constexpr int FIRING_ORDER[NUM_CYLINDERS] = {0, 2, 3, 1};
    constexpr float FIRING_START_DEG[NUM_CYLINDERS] = {0.0f, 180.0f, 360.0f, 540.0f};
    constexpr float BORE_M = 0.0671f;
    constexpr float STROKE_M = 0.0706f;
    constexpr float CYLINDER_DISPLACEMENT_M3 = (Physics::PI / 4.0f) * BORE_M * BORE_M * STROKE_M;
    constexpr float STOICHIOMETRIC_RATIO = 14.7f;
    constexpr float PEAK_TORQUE_PER_CYLINDER = 50.0f;
    constexpr float MIN_FUEL_MASS_KG = 1e-9f;
    constexpr float STARTER_TORQUE_NM = 5.0f;
    constexpr float STARTER_SPEED_THRESHOLD_RPM = 500.0f;
    constexpr float INJ_FLOW_RATE_KGS = 0.0002f;
}

// =================================================================
// == INTERRUPT-DRIVEN INJECTION TIMING
// =================================================================

volatile unsigned long g_injection_start_us[Engine::NUM_CYLINDERS];
volatile unsigned long g_last_pulse_width_us[Engine::NUM_CYLINDERS] = {0};
volatile uint8_t g_previous_inj_pin_states = 0;

ISR(PCINT1_vect) {
    unsigned long now_us = micros();
    uint8_t current_inj_pin_states = PINC & 0x0F;
    uint8_t changed_pins = current_inj_pin_states ^ g_previous_inj_pin_states;

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        if (changed_pins & (1 << i)) {
            if (current_inj_pin_states & (1 << i)) {
                g_injection_start_us[i] = now_us;
            } else {
                g_last_pulse_width_us[i] = now_us - g_injection_start_us[i];
            }
        }
    }
    g_previous_inj_pin_states = current_inj_pin_states;
}

// =================================================================
// == ENGINE SIMULATOR CLASS
// =================================================================

class EngineSimulator {
public:
    void setup();
    void update();
    void write_telemetry();

private:
    float dps_to_rpm(float dps) { return dps / 6.0f; }
    void process_interrupt_data();
    void read_external_inputs();
    void update_dynamics();
    void update_outputs();

    struct Cylinder {
        float injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
        volatile bool ignition_fired = false;
        volatile bool is_igniting = false;
        float lambda = 1.0f;  // added missing lambda field
    };

    Cylinder cylinders[Engine::NUM_CYLINDERS];
    volatile float flywheel_angle_deg = 0.0f;
    volatile float flywheel_speed_dps = 0.0f;
    volatile float flywheel_torque_nm = 0.0f;
    volatile float crank_angle_720_deg = 0.0f;
    volatile float lambda_overall = 1.0f;
    float manifold_pressure_pa = 101325.0f;
};

void EngineSimulator::setup() {
    pinMode(Pins::MAP_INPUT, INPUT);
    pinMode(Pins::CAM_POS, OUTPUT);
    pinMode(Pins::CRANK_POS, OUTPUT);
    pinMode(Pins::LAMBDA, OUTPUT);
    pinMode(Pins::START_BUTTON, INPUT_PULLUP);
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        pinMode(Pins::INJ[i], INPUT);
        pinMode(Pins::IGN[i], INPUT);
    }

    cli();
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);
    sei();
}

void EngineSimulator::update() {
    process_interrupt_data();
    read_external_inputs();
    update_dynamics();
    update_outputs();
}

void EngineSimulator::process_interrupt_data() {
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        unsigned long captured_width;
        cli();
        captured_width = g_last_pulse_width_us[i];
        g_last_pulse_width_us[i] = 0;
        sei();

        if (captured_width > 0) {
            float width_s = captured_width / 1.0e6f;
            cylinders[i].injected_fuel_mass = Engine::INJ_FLOW_RATE_KGS * width_s;
            if (cylinders[i].injected_fuel_mass < Engine::MIN_FUEL_MASS_KG) {
                cylinders[i].injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
            }
        }
    }
}

void EngineSimulator::read_external_inputs() {
    int raw_adc = analogRead(Pins::MAP_INPUT);  
    manifold_pressure_pa = map(raw_adc, 0, 1023, 25000, 101325);
    manifold_pressure_pa = constrain(manifold_pressure_pa, 25000, 101325);

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        bool ign_state = digitalRead(Pins::IGN[i]);
        if (ign_state && !cylinders[i].is_igniting) {
            cylinders[i].is_igniting = true;
            cylinders[i].ignition_fired = true;
        } else if (!ign_state && cylinders[i].is_igniting) {
            cylinders[i].is_igniting = false;
        }
    }
}

// (update_dynamics, update_outputs, write_telemetry remain same as your last block)

// =================================================================
// == ARDUINO SKETCH
// =================================================================

EngineSimulator engine;
volatile bool run_simulation_tick = false;

void timer_isr() {
    run_simulation_tick = true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Engine Simulator (Interrupt-driven) Initializing...");
    engine.setup();
    Timer1.initialize(Config::TIME_STEP_S * 1e6);
    Timer1.attachInterrupt(timer_isr);
    Serial.println("Simulator running.");
}

void loop() {
    if (run_simulation_tick) {
        run_simulation_tick = false;
        engine.update();
    }
    engine.write_telemetry();
}
