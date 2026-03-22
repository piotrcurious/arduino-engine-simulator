#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =================================================================
// == SMALL HELPERS
// =================================================================

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    if (fabsf(in_max - in_min) < 1e-9f) return out_min;
    return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

static inline uint8_t lambda_to_pwm(float lambda) {
    // Map lambda 0.5..1.5 -> PWM 0..255
    float clamped = clampf(lambda, 0.5f, 1.5f);
    float norm = (clamped - 0.5f) / 1.0f;
    int pwm = (int)(norm * 255.0f + 0.5f);
    return (uint8_t)clampf((float)pwm, 0.0f, 255.0f);
}

// =================================================================
// == CONFIGURATION & CONSTANTS
// =================================================================

namespace Pins {
    constexpr uint8_t CAM_POS = 2;
    constexpr uint8_t CRANK_POS = 3;
    constexpr uint8_t INJ[] = {4, 5, 6, 7};
    constexpr uint8_t IGN[] = {8, 9, 10, 11};
    // Note: Pin 12 is NOT a PWM pin on standard AVR (Uno/Nano).
    // If using an Uno/Nano, move this to a PWM-capable pin (3, 5, 6, 9, 10, 11).
    constexpr uint8_t LAMBDA = 12;
    constexpr uint8_t START_BUTTON = 13;
    constexpr uint8_t THROTTLE_ADC = A0;
    constexpr uint8_t FORCE_ADC = A1;
}

namespace Config {
    constexpr float TARGET_DT_S = 0.0002f; // 200 us (5 kHz)
    enum class CamWheelType { WHEEL_1, WHEEL_2, WHEEL_3 };
    constexpr CamWheelType CAM_WHEEL = CamWheelType::WHEEL_1;
    constexpr float ADC_MAX_VAL = 1023.0f;
    constexpr float ADC_MID_VAL = 511.5f;
}

namespace Physics {
    constexpr float PI_VAL = 3.14159265f;
    constexpr float AIR_GAS_CONSTANT = 287.05f;
    constexpr float INTAKE_TEMP_K = 293.15f;
    constexpr float ATMOSPHERIC_PRESSURE_PA = 101325.0f;

    constexpr float FLYWHEEL_MASS_KG = 10.0f;
    constexpr float FLYWHEEL_RADIUS_M = 0.1f;
    constexpr float FLYWHEEL_INERTIA = FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;

    constexpr float VISCOUS_FRICTION_COEFF = 0.005f;
    constexpr float STATIC_FRICTION_NM = 0.5f;
    constexpr float ENGINE_BRAKE_FACTOR = 3.0f;
    constexpr int THROTTLE_BRAKE_THRESHOLD = 50;
}

namespace Engine {
    constexpr int NUM_CYLINDERS = 4;
    constexpr int FIRING_ORDER[NUM_CYLINDERS] = {0, 2, 3, 1};
    constexpr float FIRING_START_DEG[NUM_CYLINDERS] = {0.0f, 180.0f, 360.0f, 540.0f};

    constexpr float BORE_M = 0.0671f;
    constexpr float STROKE_M = 0.0706f;
    constexpr float CYLINDER_DISPLACEMENT_M3 = (Physics::PI_VAL / 4.0f) * BORE_M * BORE_M * STROKE_M;

    constexpr float STOICHIOMETRIC_RATIO = 14.7f;
    constexpr float PEAK_TORQUE_PER_CYLINDER = 55.0f;
    constexpr float COMPRESSION_TORQUE_LOSS = 15.0f;
    constexpr float MIN_FUEL_MASS_KG = 1e-9f;

    constexpr float STARTER_TORQUE_NM = 12.0f;
    constexpr float STARTER_SPEED_THRESHOLD_RPM = 400.0f;
    constexpr float INJ_FLOW_RATE_KGS = 0.0002f;
}

// =================================================================
// == ENGINE SIMULATOR CLASS
// =================================================================

class EngineSimulator {
public:
    void setup();
    void update(float dt);
    void write_telemetry();

private:
    float dps_to_rpm(float dps) const { return dps * (60.0f / 360.0f); }

    void measure_ecu_inputs();
    void update_dynamics(float dt);
    void update_outputs();

    struct Cylinder {
        volatile float injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
        volatile bool is_injecting = false;
        volatile unsigned long injection_start_us = 0;

        volatile bool ignition_fired = false;
        volatile bool is_igniting = false;

        float air_mass_kg = 0.0f;
        float lambda = 1.0f;
    };

    Cylinder cylinders[Engine::NUM_CYLINDERS];

    volatile float flywheel_angle_deg = 0.0f;
    volatile float flywheel_speed_dps = 0.0f;
    volatile float flywheel_torque_nm = 0.0f;
    volatile float crank_angle_720_deg = 0.0f;
    volatile float lambda_overall = 1.0f;

    float manifold_pressure_pa = Physics::ATMOSPHERIC_PRESSURE_PA;
    int current_throttle_adc = 0;
    int current_force_adc = (int)Config::ADC_MID_VAL;
    bool toggle_adc_read = false;
};

// =================================================================
// == CLASS IMPLEMENTATION
// =================================================================

void EngineSimulator::setup() {
    pinMode(Pins::CAM_POS, OUTPUT);
    pinMode(Pins::CRANK_POS, OUTPUT);
    pinMode(Pins::LAMBDA, OUTPUT);

    pinMode(Pins::START_BUTTON, INPUT_PULLUP);

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        pinMode(Pins::INJ[i], INPUT);
        pinMode(Pins::IGN[i], INPUT);
    }

    digitalWrite(Pins::CAM_POS, LOW);
    digitalWrite(Pins::CRANK_POS, LOW);
    analogWrite(Pins::LAMBDA, 0);
}

void EngineSimulator::update(float dt) {
    measure_ecu_inputs();
    update_dynamics(dt);
    update_outputs();
}

void EngineSimulator::measure_ecu_inputs() {
    unsigned long now_us = micros();

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        // Injection pulse width capture
        bool inj_state = (digitalRead(Pins::INJ[i]) == HIGH);

        if (inj_state && !cylinders[i].is_injecting) {
            cylinders[i].is_injecting = true;
            cylinders[i].injection_start_us = now_us;
        } else if (!inj_state && cylinders[i].is_injecting) {
            cylinders[i].is_injecting = false;
            float width_s = (now_us - cylinders[i].injection_start_us) / 1000000.0f;
            cylinders[i].injected_fuel_mass = fmaxf(Engine::INJ_FLOW_RATE_KGS * width_s,
                                                    Engine::MIN_FUEL_MASS_KG);
        }

        // Ignition edge capture
        bool ign_state = (digitalRead(Pins::IGN[i]) == HIGH);
        if (ign_state && !cylinders[i].is_igniting) {
            cylinders[i].is_igniting = true;
            cylinders[i].ignition_fired = true;
        } else if (!ign_state && cylinders[i].is_igniting) {
            cylinders[i].is_igniting = false;
        }
    }
}

void EngineSimulator::update_dynamics(float dt) {
    // Alternate ADC reads to save loop time
    if (toggle_adc_read) {
        current_throttle_adc = analogRead(Pins::THROTTLE_ADC);
    } else {
        current_force_adc = analogRead(Pins::FORCE_ADC);
    }
    toggle_adc_read = !toggle_adc_read;

    float throttle_frac = clampf(current_throttle_adc / Config::ADC_MAX_VAL, 0.0f, 1.0f);
    float engine_rpm = fmaxf(dps_to_rpm(flywheel_speed_dps), 0.0f);

    // MAP calculation
    float target_map = Physics::ATMOSPHERIC_PRESSURE_PA *
                       (throttle_frac + (1.0f - throttle_frac) * 0.3f);
    target_map -= (engine_rpm / 8000.0f) * 70000.0f * (1.0f - throttle_frac);
    target_map = clampf(target_map, 20000.0f, Physics::ATMOSPHERIC_PRESSURE_PA);

    // Smooth manifold pressure with dt-aware alpha
    float alpha = clampf(dt * 50.0f, 0.0f, 1.0f);
    manifold_pressure_pa += alpha * (target_map - manifold_pressure_pa);

    crank_angle_720_deg = fmodf(crank_angle_720_deg + flywheel_speed_dps * dt, 720.0f);
    if (crank_angle_720_deg < 0.0f) crank_angle_720_deg += 720.0f;

    float net_torque = 0.0f;
    float total_lambda = 0.0f;

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        int cyl_idx = Engine::FIRING_ORDER[i];
        float phase_deg = fmodf(crank_angle_720_deg - Engine::FIRING_START_DEG[i] + 720.0f, 720.0f);

        // Reset ignition tracking outside the power window
        if (phase_deg > 180.0f && phase_deg < 360.0f) {
            cylinders[cyl_idx].ignition_fired = false;
            cylinders[cyl_idx].injected_fuel_mass = Engine::MIN_FUEL_MASS_KG; // consume fuel once per cycle
            cylinders[cyl_idx].lambda = 1.0f;
        }

        // Power stroke logic
        if (cylinders[cyl_idx].ignition_fired && phase_deg >= 0.0f && phase_deg <= 180.0f) {
            float ve = (engine_rpm > 1000.0f && engine_rpm < 4500.0f)
                         ? 0.95f
                         : ((engine_rpm >= 4500.0f) ? 0.88f : 0.85f);

            cylinders[cyl_idx].air_mass_kg =
                (manifold_pressure_pa * Engine::CYLINDER_DISPLACEMENT_M3 * ve) /
                (Physics::AIR_GAS_CONSTANT * Physics::INTAKE_TEMP_K);

            float fuel_mass = cylinders[cyl_idx].injected_fuel_mass;

            // Standalone auto-fuel logic
            if (fuel_mass <= Engine::MIN_FUEL_MASS_KG && throttle_frac > 0.05f) {
                fuel_mass = cylinders[cyl_idx].air_mass_kg / Engine::STOICHIOMETRIC_RATIO;
            }

            fuel_mass = fmaxf(fuel_mass, Engine::MIN_FUEL_MASS_KG);
            cylinders[cyl_idx].lambda =
                (cylinders[cyl_idx].air_mass_kg / fuel_mass) / Engine::STOICHIOMETRIC_RATIO;

            float lambda_efficiency = 0.0f;
            if (cylinders[cyl_idx].lambda < 1.0f) {
                lambda_efficiency = (cylinders[cyl_idx].lambda - 0.4f) / 0.6f;
            } else {
                lambda_efficiency = 1.0f / cylinders[cyl_idx].lambda;
            }
            lambda_efficiency = clampf(lambda_efficiency, 0.0f, 1.0f);

            // Power generation
            net_torque += Engine::PEAK_TORQUE_PER_CYLINDER *
                          sinf(phase_deg * (Physics::PI_VAL / 180.0f)) *
                          lambda_efficiency;
        } else {
            // Keep inactive cylinders neutral for telemetry
            cylinders[cyl_idx].lambda = 1.0f;
        }

        // Compression stroke logic
        if (phase_deg > 540.0f && phase_deg <= 720.0f) {
            net_torque -= Engine::COMPRESSION_TORQUE_LOSS *
                          sinf((phase_deg - 540.0f) * (Physics::PI_VAL / 180.0f));
        }

        // Standalone auto-ignition logic
        if (!cylinders[cyl_idx].ignition_fired && phase_deg < 10.0f && engine_rpm > 200.0f) {
            cylinders[cyl_idx].ignition_fired = true;
        }

        total_lambda += cylinders[cyl_idx].lambda;
    }

    lambda_overall = total_lambda / (float)Engine::NUM_CYLINDERS;

    // Starter motor logic
    if (digitalRead(Pins::START_BUTTON) == LOW && engine_rpm < Engine::STARTER_SPEED_THRESHOLD_RPM) {
        net_torque += Engine::STARTER_TORQUE_NM;
    }

    // External load
    float external_force = (current_force_adc - (int)Config::ADC_MID_VAL) * 0.05f;
    net_torque -= external_force * Physics::FLYWHEEL_RADIUS_M;

    // Friction model
    float omega_rad_s = flywheel_speed_dps * (Physics::PI_VAL / 180.0f);
    float viscous_friction = Physics::VISCOUS_FRICTION_COEFF * omega_rad_s;
    if (current_throttle_adc < Physics::THROTTLE_BRAKE_THRESHOLD) {
        viscous_friction *= Physics::ENGINE_BRAKE_FACTOR;
    }

    // Stabilized coulomb friction to prevent 0-crossing jitter
    float coulomb_friction = 0.0f;
    if (flywheel_speed_dps > 1.0f) coulomb_friction = Physics::STATIC_FRICTION_NM;
    else if (flywheel_speed_dps < -1.0f) coulomb_friction = -Physics::STATIC_FRICTION_NM;

    net_torque -= (viscous_friction + coulomb_friction);
    flywheel_torque_nm = net_torque;

    // Kinematics integration: α = τ / I
    float angular_accel_rad_s2 = net_torque / Physics::FLYWHEEL_INERTIA;
    flywheel_speed_dps += angular_accel_rad_s2 * (180.0f / Physics::PI_VAL) * dt;

    // Prevent reverse rotation for standard engines
    if (flywheel_speed_dps < 0.0f) flywheel_speed_dps = 0.0f;

    flywheel_angle_deg = fmodf(flywheel_angle_deg + flywheel_speed_dps * dt, 360.0f);
    if (flywheel_angle_deg < 0.0f) flywheel_angle_deg += 360.0f;
}

void EngineSimulator::update_outputs() {
    uint8_t lambda_pwm = lambda_to_pwm(lambda_overall);
    analogWrite(Pins::LAMBDA, lambda_pwm);

    // Cam output: 2:1 crank-to-cam ratio
    float cam_angle_deg = crank_angle_720_deg / 2.0f;
    bool cam_pos_state = (fmodf(cam_angle_deg, 180.0f) < 90.0f);
    digitalWrite(Pins::CAM_POS, cam_pos_state ? HIGH : LOW);

    // 60-2 style crank signal approximation on 360 deg wheel
    float crank_angle_360_deg = fmodf(flywheel_angle_deg, 360.0f);
    int tooth = (int)floorf(crank_angle_360_deg / 6.0f);

    bool crank_state = true;
    if (tooth >= 58) {
        crank_state = false; // missing teeth gap
    } else {
        crank_state = (fmodf(crank_angle_360_deg, 6.0f) < 3.0f);
    }

    digitalWrite(Pins::CRANK_POS, crank_state ? HIGH : LOW);
}

void EngineSimulator::write_telemetry() {
    static unsigned long last_print_ms = 0;
    unsigned long now_ms = millis();

    if (now_ms - last_print_ms > 100) {
        last_print_ms = now_ms;
        Serial.print("RPM:"); Serial.print(dps_to_rpm(flywheel_speed_dps), 0);
        Serial.print(" MAP_kPa:"); Serial.print(manifold_pressure_pa / 1000.0f, 1);
        Serial.print(" TQ_Nm:"); Serial.print(flywheel_torque_nm, 1);
        Serial.print(" LAMBDA:"); Serial.println(lambda_overall, 2);
    }
}

// =================================================================
// == ARDUINO SKETCH
// =================================================================

EngineSimulator engine;
volatile bool run_simulation_tick = false;
unsigned long last_update_us = 0;

void timer_isr() {
    run_simulation_tick = true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Engine Simulator...");

    engine.setup();

    unsigned long period_us = (unsigned long)(Config::TARGET_DT_S * 1000000.0f);
    Timer1.initialize(period_us);
    Timer1.attachInterrupt(timer_isr);

    last_update_us = micros();
    Serial.println("Simulator online.");
}

void loop() {
    if (run_simulation_tick) {
        run_simulation_tick = false;

        unsigned long now = micros();
        float dt = (now - last_update_us) / 1000000.0f;
        last_update_us = now;

        // Safety clamp in case of a long stall
        if (dt > 0.05f) dt = 0.05f;
        if (dt < 0.0f) dt = Config::TARGET_DT_S;

        engine.update(dt);
    }

    engine.write_telemetry();
}
