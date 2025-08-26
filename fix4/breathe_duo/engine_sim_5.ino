#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =================================================================
// == CONFIGURATION & CONSTANTS
// =================================================================

namespace Pins {
    // Communication Pin (PWM filtered to DC, so analog)
    constexpr uint8_t MAP_INPUT = A4;

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
    constexpr float TIME_STEP_S = 0.001f; // 1 ms simulation step
    constexpr float ADC_MAX_VAL = 1023.0f;
    constexpr float ADC_MID_VAL = 511.5f;
    constexpr float MAP_SMOOTH_ALPHA = 0.1f; // low-pass filter factor
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
        float lambda = 1.0f;
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
    // Smoothed MAP input
    static float filtered_map = 101325.0f;
    int raw_adc = analogRead(Pins::MAP_INPUT);
    float map_pa = map(raw_adc, 0, 1023, 25000, 101325);
    filtered_map = (1.0f - Config::MAP_SMOOTH_ALPHA) * filtered_map + Config::MAP_SMOOTH_ALPHA * map_pa;
    manifold_pressure_pa = constrain(filtered_map, 25000, 101325);

    // Ignition input
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

void EngineSimulator::update_dynamics() {
    float engine_rpm = dps_to_rpm(flywheel_speed_dps);

    crank_angle_720_deg = fmod(crank_angle_720_deg + flywheel_speed_dps * Config::TIME_STEP_S, 720.0f);
    if (crank_angle_720_deg < 0.0f) crank_angle_720_deg += 720.0f;

    float net_torque = 0.0f;
    float total_lambda = 0.0f;

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        int cyl_idx = Engine::FIRING_ORDER[i];
        float tdc_power_stroke = Engine::FIRING_START_DEG[i];
        float phase_deg = crank_angle_720_deg - tdc_power_stroke;

        if (phase_deg > 180.0f && phase_deg < 360.0f) {
            cylinders[cyl_idx].ignition_fired = false;
        }

        if (cylinders[cyl_idx].ignition_fired && phase_deg >= 0.0f && phase_deg <= 180.0f) {
            float ve = 0.85;
            if (engine_rpm > 1000 && engine_rpm < 4500) ve = 0.95;
            else if (engine_rpm >= 4500) ve = 0.88;

            float air_mass_kg = (manifold_pressure_pa * Engine::CYLINDER_DISPLACEMENT_M3 * ve) / (Physics::AIR_GAS_CONSTANT * Physics::INTAKE_TEMP_K);
            cylinders[cyl_idx].lambda = (air_mass_kg / cylinders[cyl_idx].injected_fuel_mass) / Engine::STOICHIOMETRIC_RATIO;
            
            float lambda_efficiency = (cylinders[cyl_idx].lambda < 1.0f) ? (cylinders[cyl_idx].lambda - 0.4f) / 0.6f : 1.0f / cylinders[cyl_idx].lambda;
            lambda_efficiency = constrain(lambda_efficiency, 0.0f, 1.0f);
            
            float torque_curve = sin(phase_deg * (Physics::PI / 180.0f));
            net_torque += Engine::PEAK_TORQUE_PER_CYLINDER * torque_curve * lambda_efficiency;

            if (phase_deg > 10.0f) {
                cylinders[cyl_idx].ignition_fired = false;
            }
        }
        total_lambda += cylinders[cyl_idx].lambda;
    }
    lambda_overall = total_lambda / Engine::NUM_CYLINDERS;

    if (digitalRead(Pins::START_BUTTON) == LOW && engine_rpm < Engine::STARTER_SPEED_THRESHOLD_RPM) {
        net_torque += Engine::STARTER_TORQUE_NM;
    }
    float external_force = (analogRead(Pins::FORCE_ADC) - Config::ADC_MID_VAL) * 0.01f;
    net_torque -= external_force * Physics::FLYWHEEL_RADIUS_M;

    float omega_rad_s = flywheel_speed_dps * (Physics::PI / 180.0f);
    float viscous_friction = Physics::VISCOUS_FRICTION_COEFF * omega_rad_s;
    float coulomb_friction = (flywheel_speed_dps > 0) ? Physics::STATIC_FRICTION_NM : -Physics::STATIC_FRICTION_NM;
    net_torque -= (viscous_friction + coulomb_friction);
    
    flywheel_torque_nm = net_torque;
    float angular_accel_rad_s2 = net_torque / Physics::FLYWHEEL_INERTIA;
    flywheel_speed_dps += angular_accel_rad_s2 * (180.0f / Physics::PI) * Config::TIME_STEP_S;
    flywheel_angle_deg = fmod(flywheel_angle_deg + flywheel_speed_dps * Config::TIME_STEP_S, 360.0f);
    if (flywheel_angle_deg < 0.0f) flywheel_angle_deg += 360.0f;
}

void EngineSimulator::update_outputs() {
    // Lambda output
    int lambda_pwm = constrain((int)(lambda_overall * 255 / 2.0f), 0, 255); 
    analogWrite(Pins::LAMBDA, lambda_pwm);

    // Cam signal (half engine speed, 360 per 720 crank)
    float cam_angle_deg = fmod(flywheel_angle_deg / 2.0f, 360.0f);
    bool cam_pos_state = (fmod(cam_angle_deg, 180.0f) < 90.0f);
    digitalWrite(Pins::CAM_POS, cam_pos_state);

    // Crank signal (60-2 teeth, simulated)
    int tooth = floor(flywheel_angle_deg / 6.0f);
    bool crank_state = (fmod(flywheel_angle_deg, 6.0f) < 3.0f) && (tooth < 58);
    digitalWrite(Pins::CRANK_POS, crank_state);
}

void EngineSimulator::write_telemetry() {
    static unsigned long last_print_ms = 0;
    if (millis() - last_print_ms > 250) {
        last_print_ms = millis();
        Serial.print("RPM: ");
        Serial.print(dps_to_rpm(flywheel_speed_dps));
        Serial.print(" | MAP: ");
        Serial.print(manifold_pressure_pa / 1000.0f);
        Serial.print(" kPa | Torque: ");
        Serial.print(flywheel_torque_nm);
        Serial.print(" Nm | Lambda: ");
        Serial.println(lambda_overall, 2);
    }
}

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
