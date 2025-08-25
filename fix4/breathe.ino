#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =================================================================
// == CONFIGURATION & CONSTANTS
// =================================================================

namespace Pins {
    constexpr uint8_t CAM_POS = 2;
    constexpr uint8_t CRANK_POS = 3; // New output for crank position sensor
    constexpr uint8_t INJ[] = {4, 5, 6, 7};
    constexpr uint8_t IGN[] = {8, 9, 10, 11};
    constexpr uint8_t LAMBDA = 12;
    constexpr uint8_t START_BUTTON = 13;
    constexpr uint8_t THROTTLE_ADC = A0;
    constexpr uint8_t FORCE_ADC = A1;
}

namespace Config {
    constexpr float TIME_STEP_S = 0.001f; // 1 ms update interval
    enum class CamWheelType { WHEEL_1, WHEEL_2, WHEEL_3 };
    constexpr CamWheelType CAM_WHEEL = CamWheelType::WHEEL_1;
    constexpr float ADC_MAX_VAL = 1023.0f;
    constexpr float ADC_MID_VAL = 511.5f;
}

namespace Physics {
    constexpr float PI = 3.14159265358979323846f;
    constexpr float AIR_DENSITY_KGM3 = 1.225f;
    constexpr float ATMOSPHERIC_PRESSURE_PA = 101325.0f;
    constexpr float AIR_GAS_CONSTANT = 287.05f; // R for dry air J/(kg·K)
    constexpr float INTAKE_TEMP_K = 293.15f;    // 20°C

    constexpr float FLYWHEEL_MASS_KG = 10.0f;
    constexpr float FLYWHEEL_RADIUS_M = 0.1f;
    constexpr float FLYWHEEL_INERTIA = FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;

    constexpr float VISCOUS_FRICTION_COEFF = 0.002f;
    constexpr float STATIC_FRICTION_NM = 0.05f;
    constexpr float ENGINE_BRAKE_FACTOR = 4.0f;
    constexpr int THROTTLE_BRAKE_THRESHOLD = 50;
}

namespace Engine {
    constexpr int NUM_CYLINDERS = 4;
    constexpr int FIRING_ORDER[NUM_CYLINDERS] = {0, 2, 3, 1};
    constexpr float FIRING_START_DEG[NUM_CYLINDERS] = {0.0f, 180.0f, 360.0f, 540.0f};

    constexpr float BORE_M = 0.0671f;
    constexpr float STROKE_M = 0.0706f;
    constexpr float CYLINDER_DISPLACEMENT_M3 = (Physics::PI / 4.0f) * BORE_M * BORE_M * STROKE_M;
    constexpr float CYLINDER_AREA_M2 = Physics::PI * (BORE_M / 2.0f) * (BORE_M / 2.0f);
    
    constexpr float STOICHIOMETRIC_RATIO = 14.7f;
    constexpr float PEAK_TORQUE_PER_CYLINDER = 50.0f; // Max ideal torque from one cylinder's combustion [Nm]
    constexpr float MIN_FUEL_MASS_KG = 1e-9f;

    constexpr float STARTER_TORQUE_NM = 5.0f;
    constexpr float STARTER_SPEED_THRESHOLD_RPM = 500.0f;

    constexpr float INJ_FLOW_RATE_KGS = 0.0002f;
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
    // Helper to convert speed units
    float dps_to_rpm(float dps) { return dps / 6.0f; }

    // Private methods for simulation stages
    void measure_ecu_inputs();
    void update_dynamics();
    void update_outputs();
    
    // Cylinder-specific state
    struct Cylinder {
        volatile float injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
        volatile bool is_injecting = false;
        volatile unsigned long injection_start_us = 0;
        
        volatile bool ignition_fired = false;
        volatile bool is_igniting = false;
        volatile unsigned long ignition_start_us = 0;

        float air_mass_kg = 0.0f;
        float lambda = 1.0f;
    };

    // Global engine state
    Cylinder cylinders[Engine::NUM_CYLINDERS];
    volatile float flywheel_angle_deg = 0.0f;
    volatile float flywheel_speed_dps = 0.0f;
    volatile float flywheel_torque_nm = 0.0f;
    volatile float crank_angle_720_deg = 0.0f;
    volatile float lambda_overall = 1.0f;
    float manifold_pressure_pa = Physics::ATMOSPHERIC_PRESSURE_PA;
};

// =================================================================
// == CLASS IMPLEMENTATION
// =================================================================

void EngineSimulator::setup() {
    // Pin Modes
    pinMode(Pins::CAM_POS, OUTPUT);
    pinMode(Pins::CRANK_POS, OUTPUT);
    pinMode(Pins::LAMBDA, OUTPUT);
    pinMode(Pins::START_BUTTON, INPUT_PULLUP);
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        pinMode(Pins::INJ[i], INPUT);
        pinMode(Pins::IGN[i], INPUT);
    }
}

void EngineSimulator::update() {
    measure_ecu_inputs();
    update_dynamics();
    update_outputs();
}

void EngineSimulator::measure_ecu_inputs() {
    unsigned long now_us = micros();
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        // --- Measure Injector Pulses ---
        bool inj_state = digitalRead(Pins::INJ[i]);
        if (inj_state && !cylinders[i].is_injecting) { // Rising Edge
            cylinders[i].is_injecting = true;
            cylinders[i].injection_start_us = now_us;
        } else if (!inj_state && cylinders[i].is_injecting) { // Falling Edge
            cylinders[i].is_injecting = false;
            unsigned long width_us = now_us - cylinders[i].injection_start_us;
            cylinders[i].injected_fuel_mass = Engine::INJ_FLOW_RATE_KGS * (width_us / 1e6f);
            if (cylinders[i].injected_fuel_mass < Engine::MIN_FUEL_MASS_KG) {
                cylinders[i].injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
            }
        }

        // --- Measure Ignition Pulses ---
        bool ign_state = digitalRead(Pins::IGN[i]);
        if (ign_state && !cylinders[i].is_igniting) { // Rising edge signals spark
            cylinders[i].is_igniting = true;
            cylinders[i].ignition_fired = true; // Latch that this cylinder has received a signal to fire
        } else if (!ign_state && cylinders[i].is_igniting) {
            cylinders[i].is_igniting = false;
        }
    }
}

void EngineSimulator::update_dynamics() {
    // 1. Read Analog Inputs & Update Manifold Pressure
    int throttle_adc = analogRead(Pins::THROTTLE_ADC);
    float throttle_frac = throttle_adc / Config::ADC_MAX_VAL;
    float engine_rpm = dps_to_rpm(flywheel_speed_dps);

    // Simple MAP model: Pressure drops with low throttle and high RPM (vacuum)
    // At WOT, pressure approaches atmospheric. At idle, it's low.
    float target_map = Physics::ATMOSPHERIC_PRESSURE_PA * (throttle_frac + (1.0 - throttle_frac) * 0.3); // Base on throttle
    target_map -= (engine_rpm / 8000.0f) * 70000.0f * (1.0 - throttle_frac); // Add vacuum effect from speed
    manifold_pressure_pa = 0.95 * manifold_pressure_pa + 0.05 * clamp(target_map, 25000.0f, Physics::ATMOSPHERIC_PRESSURE_PA); // Smooth MAP changes

    // 2. Calculate Combustion Torque
    crank_angle_720_deg = fmod(crank_angle_720_deg + flywheel_speed_dps * Config::TIME_STEP_S, 720.0f);
    if (crank_angle_720_deg < 0.0f) crank_angle_720_deg += 720.0f;

    float net_torque = 0.0f;
    float total_lambda = 0.0f;

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        int cyl_idx = Engine::FIRING_ORDER[i];
        float tdc_power_stroke = Engine::FIRING_START_DEG[i];
        float phase_deg = crank_angle_720_deg - tdc_power_stroke;

        // Reset ignition flag after the power stroke window is over
        if (phase_deg > 180.0f && phase_deg < 360.0f) {
            cylinders[cyl_idx].ignition_fired = false;
        }

        // Is this cylinder in its power stroke window AND has it been told to ignite?
        if (cylinders[cyl_idx].ignition_fired && phase_deg >= 0.0f && phase_deg <= 180.0f) {
            // -- Volumetric Efficiency (VE) Lookup --
            // VE is typically highest in the mid-range RPM. This is a simple model.
            float ve = 0.85; // Default VE
            if (engine_rpm > 1000 && engine_rpm < 4500) ve = 0.95;
            else if (engine_rpm >= 4500) ve = 0.88;

            // -- Air & Fuel Calculation --
            cylinders[cyl_idx].air_mass_kg = (manifold_pressure_pa * Engine::CYLINDER_DISPLACEMENT_M3 * ve) / (Physics::AIR_GAS_CONSTANT * Physics::INTAKE_TEMP_K);
            float fuel_mass = cylinders[cyl_idx].injected_fuel_mass;
            cylinders[cyl_idx].lambda = (cylinders[cyl_idx].air_mass_kg / fuel_mass) / Engine::STOICHIOMETRIC_RATIO;
            
            // -- Torque Calculation --
            // Efficiency drops when mixture is not stoichiometric (lambda != 1)
            float lambda_efficiency = (cylinders[cyl_idx].lambda < 1.0f) ? (cylinders[cyl_idx].lambda - 0.4f) / 0.6f : 1.0f / cylinders[cyl_idx].lambda;
            lambda_efficiency = clamp(lambda_efficiency, 0.0f, 1.0f);
            
            // Model torque curve over the 180-degree power stroke using a sine wave
            float torque_curve = sin(phase_deg * (Physics::PI / 180.0f));
            net_torque += Engine::PEAK_TORQUE_PER_CYLINDER * torque_curve * lambda_efficiency;

            // Once combustion starts, we consume the fuel and reset the ignition flag
            // to prevent re-firing in the same cycle.
            if (phase_deg > 10.0f) { // Give it a few degrees before resetting
                cylinders[cyl_idx].ignition_fired = false;
            }
        }
        total_lambda += cylinders[cyl_idx].lambda;
    }
    lambda_overall = total_lambda / Engine::NUM_CYLINDERS;

    // 3. Add Other Torques (Starter, Friction, Load)
    if (digitalRead(Pins::START_BUTTON) == LOW && engine_rpm < Engine::STARTER_SPEED_THRESHOLD_RPM) {
        net_torque += Engine::STARTER_TORQUE_NM;
    }
    float external_force = (analogRead(Pins::FORCE_ADC) - Config::ADC_MID_VAL) * 0.01f;
    net_torque -= external_force * Physics::FLYWHEEL_RADIUS_M;

    float omega_rad_s = flywheel_speed_dps * (Physics::PI / 180.0f);
    float viscous_friction = Physics::VISCOUS_FRICTION_COEFF * omega_rad_s;
    if (throttle_adc < Physics::THROTTLE_BRAKE_THRESHOLD) {
        viscous_friction *= Physics::ENGINE_BRAKE_FACTOR;
    }
    float coulomb_friction = (flywheel_speed_dps > 0) ? Physics::STATIC_FRICTION_NM : -Physics::STATIC_FRICTION_NM;
    net_torque -= (viscous_friction + coulomb_friction);
    
    // 4. Integrate Dynamics
    flywheel_torque_nm = net_torque;
    float angular_accel_rad_s2 = net_torque / Physics::FLYWHEEL_INERTIA;
    flywheel_speed_dps += angular_accel_rad_s2 * (180.0f / Physics::PI) * Config::TIME_STEP_S;
    flywheel_angle_deg = fmod(flywheel_angle_deg + flywheel_speed_dps * Config::TIME_STEP_S, 360.0f);
    if (flywheel_angle_deg < 0.0f) flywheel_angle_deg += 360.0f;
}

void EngineSimulator::update_outputs() {
    // Lambda Sensor (PWM)
    int lambda_pwm = map(clamp(lambda_overall, 0.5f, 1.5f), 0.5f, 1.5f, 0, 255);
    analogWrite(Pins::LAMBDA, lambda_pwm);

    // Cam Sensor
    float cam_angle_deg = fmod(flywheel_angle_deg / 2.0f, 360.0f);
    bool cam_pos_state = (fmod(cam_angle_deg, 180.0f) < 90.0f);
    digitalWrite(Pins::CAM_POS, cam_pos_state);

    // 60-2 Crank Sensor
    // 60 teeth per rev = 6 degrees per tooth (3 deg high, 3 deg low)
    // The "missing" teeth are usually at a specific angle relative to TDC cyl 1.
    // Let's place the gap from tooth 58 to 60 (angle 348 to 360)
    int tooth = floor(flywheel_angle_deg / 6.0f);
    bool crank_state = true;
    if (tooth >= 58) { // Missing tooth gap
        crank_state = false;
    } else {
        crank_state = (fmod(flywheel_angle_deg, 6.0f) < 3.0f);
    }
    digitalWrite(Pins::CRANK_POS, crank_state);
}

void EngineSimulator::write_telemetry() {
    static unsigned long last_print_ms = 0;
    if (millis() - last_print_ms > 250) {
        last_print_ms = millis();
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "RPM: %.0f | MAP: %.0f kPa | Torque: %.2f Nm | Lambda: %.2f",
                 dps_to_rpm(flywheel_speed_dps), manifold_pressure_pa / 1000.0f, flywheel_torque_nm, lambda_overall);
        Serial.println(buffer);
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
    Serial.println("Enhanced Engine Simulator Initializing...");

    engine.setup();

    unsigned long period_us = (unsigned long)(Config::TIME_STEP_S * 1e6f);
    Timer1.initialize(period_us);
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
