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

static inline float deg2rad(float deg) {
    return deg * 0.017453292519943295f;
}

static inline float rad2deg(float rad) {
    return rad * 57.29577951308232f;
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
    constexpr uint8_t LAMBDA = 12;   // Not PWM on Uno/Nano
    constexpr uint8_t START_BUTTON = 13;
    constexpr uint8_t THROTTLE_ADC = A0;
    constexpr uint8_t FORCE_ADC = A1;
}

namespace Config {
    constexpr float TARGET_DT_S = 0.0002f;
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
    constexpr float EXHAUST_PRESSURE_PA = 108000.0f; // slight backpressure

    constexpr float GAMMA_AIR = 1.34f;

    constexpr float FLYWHEEL_MASS_KG = 10.0f;
    constexpr float FLYWHEEL_RADIUS_M = 0.1f;
    constexpr float FLYWHEEL_INERTIA = FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;

    constexpr float VISCOUS_FRICTION_COEFF = 0.0055f;
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
    constexpr float CONNECTING_ROD_M = 0.125f; // informational only

    constexpr float SWEEP_VOLUME_M3 = (Physics::PI_VAL / 4.0f) * BORE_M * BORE_M * STROKE_M;
    constexpr float COMPRESSION_RATIO = 10.0f;
    constexpr float CLEARANCE_VOLUME_M3 = SWEEP_VOLUME_M3 / (COMPRESSION_RATIO - 1.0f);
    constexpr float BDC_VOLUME_M3 = CLEARANCE_VOLUME_M3 + SWEEP_VOLUME_M3;
    constexpr float TDC_VOLUME_M3 = CLEARANCE_VOLUME_M3;

    constexpr float STOICHIOMETRIC_RATIO = 14.7f;
    constexpr float FUEL_LHV_J_PER_KG = 42.8e6f;

    constexpr float STARTER_TORQUE_NM = 12.0f;
    constexpr float STARTER_SPEED_THRESHOLD_RPM = 400.0f;
    constexpr float INJ_FLOW_RATE_KGS = 0.0002f;

    constexpr float COMBUSTION_EFFICIENCY = 0.95f;
    constexpr float TORQUE_SCALE = 0.28f;      // converts pressure torque to a more realistic crank output
    constexpr float INTAKE_PUMP_SCALE = 0.08f;
    constexpr float EXHAUST_PUMP_SCALE = 0.06f;
    constexpr float COMPRESSION_PUMP_SCALE = 0.22f;
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
    enum class Stroke : uint8_t {
        POWER,
        EXHAUST,
        INTAKE,
        COMPRESSION
    };

    struct Cylinder {
        // ECU input edge tracking
        bool inj_prev = false;
        bool ign_prev = false;

        bool injector_open = false;
        bool ignition_high = false;

        unsigned long injection_start_us = 0;
        unsigned long spark_start_us = 0;

        // Combustion model state
        bool spark_event_seen = false;
        float spark_energy_released_j = 0.0f;
        float trapped_air_mass_kg = 0.0f;
        float trapped_fuel_mass_kg = 0.0f;
        float burned_fuel_fraction = 0.0f;

        // Telemetry/state
        float lambda = 1.0f;
        float cylinder_pressure_pa = Physics::ATMOSPHERIC_PRESSURE_PA;
        float last_cycle_phase_deg = 0.0f;
        bool cycle_completed = false;
    };

    float dps_to_rpm(float dps) const { return dps * (60.0f / 360.0f); }
    float rpm_to_dps(float rpm) const { return rpm * (360.0f / 60.0f); }

    void measure_ecu_inputs();
    void update_dynamics(float dt);
    void update_outputs();
    void update_cylinder(int idx, float phase_deg, float engine_rpm, float throttle_frac, float dt, float& net_torque);

    Stroke stroke_for_phase(float phase_deg) const;
    float volumetric_efficiency(float rpm, float throttle_frac) const;
    float cylinder_volume_from_tdc(float theta_from_tdc_deg) const;
    float cylinder_pressure_at_phase(int idx, float phase_deg, float engine_rpm, float throttle_frac, float volume_m3) const;
    float combustion_pressure_boost(int idx, float phase_deg, float volume_m3, float engine_rpm) const;

    Cylinder cylinders[Engine::NUM_CYLINDERS];

    float flywheel_angle_deg = 0.0f;
    float flywheel_speed_dps = 0.0f;
    float flywheel_torque_nm = 0.0f;
    float crank_angle_720_deg = 0.0f;
    float lambda_overall = 1.0f;

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

EngineSimulator::Stroke EngineSimulator::stroke_for_phase(float phase_deg) const {
    if (phase_deg < 180.0f) return Stroke::POWER;
    if (phase_deg < 360.0f) return Stroke::EXHAUST;
    if (phase_deg < 540.0f) return Stroke::INTAKE;
    return Stroke::COMPRESSION;
}

float EngineSimulator::volumetric_efficiency(float rpm, float throttle_frac) const {
    float ve = 0.58f + 0.30f * throttle_frac;

    if (rpm > 1200.0f && rpm < 4500.0f) {
        ve += 0.08f;
    } else if (rpm >= 4500.0f) {
        ve -= 0.05f;
    }

    return clampf(ve, 0.45f, 0.96f);
}

float EngineSimulator::cylinder_volume_from_tdc(float theta_from_tdc_deg) const {
    float theta = clampf(theta_from_tdc_deg, 0.0f, 180.0f);
    float x = deg2rad(theta);
    return Engine::TDC_VOLUME_M3 + 0.5f * Engine::SWEEP_VOLUME_M3 * (1.0f - cosf(x));
}

float EngineSimulator::cylinder_pressure_at_phase(int idx, float phase_deg, float engine_rpm, float throttle_frac, float volume_m3) const {
    (void)idx;
    (void)engine_rpm;
    (void)throttle_frac;

    Stroke stroke = stroke_for_phase(phase_deg);
    float intake_pressure = manifold_pressure_pa;
    float pressure = Physics::ATMOSPHERIC_PRESSURE_PA;

    if (stroke == Stroke::POWER) {
        float theta_from_tdc = phase_deg;
        float v = fmaxf(volume_m3, Engine::TDC_VOLUME_M3);
        float compression_end_pressure =
            intake_pressure * powf(Engine::BDC_VOLUME_M3 / Engine::TDC_VOLUME_M3, Physics::GAMMA_AIR);

        float expansion_ratio = Engine::TDC_VOLUME_M3 / v;
        float base = compression_end_pressure * powf(expansion_ratio, Physics::GAMMA_AIR);

        pressure = base;
    } else if (stroke == Stroke::COMPRESSION) {
        float theta_from_tdc = 720.0f - phase_deg;
        float v = fmaxf(cylinder_volume_from_tdc(theta_from_tdc), Engine::TDC_VOLUME_M3);
        pressure = intake_pressure * powf(Engine::BDC_VOLUME_M3 / v, Physics::GAMMA_AIR);
    } else if (stroke == Stroke::INTAKE) {
        pressure = intake_pressure;
    } else {
        pressure = Physics::EXHAUST_PRESSURE_PA;
    }

    return pressure;
}

float EngineSimulator::combustion_pressure_boost(int idx, float phase_deg, float volume_m3, float engine_rpm) const {
    (void)idx;

    const Cylinder& cyl = cylinders[idx];
    if (!cyl.spark_event_seen || cyl.trapped_fuel_mass_kg <= Engine::MIN_FUEL_MASS_KG) {
        return 0.0f;
    }

    float t_since_spark = (micros() - cyl.spark_start_us) / 1000000.0f;

    // Time constant is mildly speed-sensitive; burn is faster at higher rpm in crank-angle terms.
    float tau_s = clampf(0.0065f - (engine_rpm / 1500000.0f), 0.0030f, 0.0080f);
    float burn_fraction = 1.0f - expf(-t_since_spark / tau_s);
    burn_fraction = clampf(burn_fraction, 0.0f, 1.0f);

    float afr = cyl.trapped_air_mass_kg / fmaxf(cyl.trapped_fuel_mass_kg, Engine::INJ_FLOW_RATE_KGS * 0.001f);
    float lambda = afr / Engine::STOICHIOMETRIC_RATIO;

    float mixture_factor = 1.0f;
    if (lambda < 0.75f) {
        mixture_factor = 0.55f + 0.6f * (lambda / 0.75f);
    } else if (lambda <= 1.05f) {
        mixture_factor = 1.0f;
    } else {
        mixture_factor = 1.0f / (1.0f + 1.8f * (lambda - 1.0f));
    }
    mixture_factor = clampf(mixture_factor, 0.0f, 1.0f);

    float heat_release_j = cyl.trapped_fuel_mass_kg * Engine::FUEL_LHV_J_PER_KG *
                           Engine::COMBUSTION_EFFICIENCY * burn_fraction * mixture_factor;

    // Convert released heat into an approximate pressure rise.
    // Constant-volume ideal-gas relation approximation: dP ≈ (gamma - 1) * Q / V
    float delta_p = (Physics::GAMMA_AIR - 1.0f) * heat_release_j / fmaxf(volume_m3, 1e-6f);

    // Guard against absurd spikes while still allowing strong combustion.
    delta_p = clampf(delta_p, 0.0f, 3.5e6f);
    return delta_p;
}

void EngineSimulator::measure_ecu_inputs() {
    unsigned long now_us = micros();

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        // Injection pulse capture
        bool inj_state = (digitalRead(Pins::INJ[i]) == HIGH);

        if (inj_state && !cylinders[i].inj_prev) {
            cylinders[i].injector_open = true;
            cylinders[i].injection_start_us = now_us;
        } else if (!inj_state && cylinders[i].inj_prev) {
            cylinders[i].injector_open = false;
            float width_s = (now_us - cylinders[i].injection_start_us) / 1000000.0f;
            float fuel_mass = Engine::INJ_FLOW_RATE_KGS * width_s;
            if (fuel_mass > 0.0f) {
                cylinders[i].trapped_fuel_mass_kg += fuel_mass;
            }
        }
        cylinders[i].inj_prev = inj_state;

        // Ignition edge capture
        bool ign_state = (digitalRead(Pins::IGN[i]) == HIGH);

        if (ign_state && !cylinders[i].ign_prev) {
            cylinders[i].ignition_high = true;
            cylinders[i].spark_event_seen = true;
            cylinders[i].spark_start_us = now_us;
            cylinders[i].spark_energy_released_j = 0.0f;
        } else if (!ign_state && cylinders[i].ign_prev) {
            cylinders[i].ignition_high = false;
        }
        cylinders[i].ign_prev = ign_state;
    }
}

void EngineSimulator::update_cylinder(int idx, float phase_deg, float engine_rpm, float throttle_frac, float dt, float& net_torque) {
    Cylinder& cyl = cylinders[idx];
    Stroke stroke = stroke_for_phase(phase_deg);

    float theta_from_tdc_deg = 0.0f;
    if (stroke == Stroke::POWER) {
        theta_from_tdc_deg = phase_deg;
    } else if (stroke == Stroke::COMPRESSION) {
        theta_from_tdc_deg = 720.0f - phase_deg;
    } else if (stroke == Stroke::INTAKE) {
        theta_from_tdc_deg = phase_deg - 360.0f;
    } else {
        theta_from_tdc_deg = phase_deg - 180.0f;
    }

    theta_from_tdc_deg = clampf(theta_from_tdc_deg, 0.0f, 180.0f);
    float volume_m3 = cylinder_volume_from_tdc(theta_from_tdc_deg);
    float piston_area_m2 = (Physics::PI_VAL * Engine::BORE_M * Engine::BORE_M) * 0.25f;
    float crank_radius_m = Engine::STROKE_M * 0.5f;

    float ve = volumetric_efficiency(engine_rpm, throttle_frac);
    float target_air_mass = (manifold_pressure_pa * Engine::SWEEP_VOLUME_M3 * ve) /
                            (Physics::AIR_GAS_CONSTANT * Physics::INTAKE_TEMP_K);

    // Intake charging: bring trapped air mass toward target
    if (stroke == Stroke::INTAKE) {
        float progress = clampf((phase_deg - 360.0f) / 180.0f, 0.0f, 1.0f);
        float valve_factor = sinf(progress * Physics::PI_VAL); // 0..1..0
        float fill_rate = clampf(dt * (8.0f + engine_rpm / 900.0f), 0.0f, 1.0f);
        cyl.trapped_air_mass_kg += (target_air_mass - cyl.trapped_air_mass_kg) * fill_rate * valve_factor;
        cyl.cylinder_pressure_pa = manifold_pressure_pa;
    }

    // Close the intake charge near end of intake so the cylinder traps charge for compression.
    if (phase_deg > 525.0f && phase_deg < 540.0f) {
        cyl.trapped_air_mass_kg = target_air_mass;
    }

    // Pressure model
    float pressure_pa = cylinder_pressure_at_phase(idx, phase_deg, engine_rpm, throttle_frac, volume_m3);

    // Add combustion pressure if spark has happened
    if (stroke == Stroke::POWER || stroke == Stroke::COMPRESSION) {
        pressure_pa += combustion_pressure_boost(idx, phase_deg, volume_m3, engine_rpm);
    }

    // Track lambda based on trapped air/fuel
    float fuel_mass_safe = fmaxf(cyl.trapped_fuel_mass_kg, 1e-10f);
    cyl.lambda = (cyl.trapped_air_mass_kg / fuel_mass_safe) / Engine::STOICHIOMETRIC_RATIO;
    cyl.lambda = clampf(cyl.lambda, 0.2f, 10.0f);

    // Convert cylinder pressure to crank torque
    float theta_rad = deg2rad(theta_from_tdc_deg);
    float sin_term = sinf(theta_rad);

    float torque_nm = 0.0f;

    if (stroke == Stroke::POWER) {
        float net_pressure = pressure_pa - Physics::EXHAUST_PRESSURE_PA;
        torque_nm = net_pressure * piston_area_m2 * crank_radius_m * sin_term * Engine::TORQUE_SCALE;
    } else if (stroke == Stroke::COMPRESSION) {
        float net_pressure = pressure_pa - manifold_pressure_pa;
        torque_nm = -net_pressure * piston_area_m2 * crank_radius_m * sin_term * Engine::COMPRESSION_PUMP_SCALE;
    } else if (stroke == Stroke::INTAKE) {
        float net_pressure = manifold_pressure_pa - Physics::ATMOSPHERIC_PRESSURE_PA;
        torque_nm = -fmaxf(net_pressure, 0.0f) * piston_area_m2 * crank_radius_m * sin_term * Engine::INTAKE_PUMP_SCALE;
    } else {
        float net_pressure = Physics::EXHAUST_PRESSURE_PA - Physics::ATMOSPHERIC_PRESSURE_PA;
        torque_nm = -fmaxf(net_pressure, 0.0f) * piston_area_m2 * crank_radius_m * sin_term * Engine::EXHAUST_PUMP_SCALE;
    }

    // Add mild combustion shaping: strongest near 10-20 deg ATDC
    if (stroke == Stroke::POWER && cyl.spark_event_seen) {
        float t_since_spark = (micros() - cyl.spark_start_us) / 1000000.0f;
        float burn_tau_s = clampf(0.0045f + (4000.0f / fmaxf(engine_rpm, 400.0f)) * 0.0005f, 0.0045f, 0.0075f);
        float burn = 1.0f - expf(-t_since_spark / burn_tau_s);
        burn = clampf(burn, 0.0f, 1.0f);

        float combustion_shape = expf(-0.5f * powf((phase_deg - 14.0f) / 18.0f, 2.0f));
        torque_nm += (burn * combustion_shape) * 35.0f * clampf(1.2f - fabsf(cyl.lambda - 1.0f), 0.0f, 1.2f);
    }

    net_torque += torque_nm;

    // Burnout / cycle cleanup
    if (phase_deg > 180.0f && phase_deg < 200.0f) {
        // Power stroke has ended; consumed fuel is cleared for the next cycle.
        cyl.trapped_fuel_mass_kg = 0.0f;
        cyl.spark_event_seen = false;
        cyl.spark_energy_released_j = 0.0f;
        cyl.burned_fuel_fraction = 0.0f;
    }

    if (phase_deg > 350.0f && phase_deg < 360.0f) {
        // Right before intake opens, residual gas model can be approximated as a small carryover.
        cyl.trapped_air_mass_kg = fmaxf(cyl.trapped_air_mass_kg * 0.15f, 0.0f);
    }

    cyl.cylinder_pressure_pa = pressure_pa;
    cyl.last_cycle_phase_deg = phase_deg;
}

void EngineSimulator::update_dynamics(float dt) {
    // Alternate ADC reads to reduce time spent on analogRead
    if (toggle_adc_read) {
        current_throttle_adc = analogRead(Pins::THROTTLE_ADC);
    } else {
        current_force_adc = analogRead(Pins::FORCE_ADC);
    }
    toggle_adc_read = !toggle_adc_read;

    float throttle_frac = clampf(current_throttle_adc / Config::ADC_MAX_VAL, 0.0f, 1.0f);
    float engine_rpm = fmaxf(dps_to_rpm(flywheel_speed_dps), 0.0f);

    // Manifold pressure dynamic model: throttle opens pressure toward atmosphere, speed pulls it down.
    float target_map = Physics::ATMOSPHERIC_PRESSURE_PA * (0.25f + 0.75f * throttle_frac);
    target_map -= (engine_rpm / 8000.0f) * 30000.0f * (1.0f - throttle_frac);
    target_map = clampf(target_map, 25000.0f, Physics::ATMOSPHERIC_PRESSURE_PA);

    float map_tau = 0.040f + (1.0f - throttle_frac) * 0.090f;
    float alpha = clampf(dt / fmaxf(map_tau, 1e-4f), 0.0f, 1.0f);
    manifold_pressure_pa += alpha * (target_map - manifold_pressure_pa);

    // Advance crank angle
    crank_angle_720_deg = fmodf(crank_angle_720_deg + flywheel_speed_dps * dt, 720.0f);
    if (crank_angle_720_deg < 0.0f) crank_angle_720_deg += 720.0f;

    float net_torque = 0.0f;

    // Per-cylinder phase-based model
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        int cyl_idx = Engine::FIRING_ORDER[i];
        float phase_deg = fmodf(crank_angle_720_deg - Engine::FIRING_START_DEG[i] + 720.0f, 720.0f);
        update_cylinder(cyl_idx, phase_deg, engine_rpm, throttle_frac, dt, net_torque);
    }

    // Estimate overall lambda
    float lambda_sum = 0.0f;
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        lambda_sum += cylinders[i].lambda;
    }
    lambda_overall = lambda_sum / (float)Engine::NUM_CYLINDERS;

    // Starter motor logic
    if (digitalRead(Pins::START_BUTTON) == LOW && engine_rpm < Engine::STARTER_SPEED_THRESHOLD_RPM) {
        net_torque += Engine::STARTER_TORQUE_NM;
    }

    // External load
    float external_force_n = (current_force_adc - Config::ADC_MID_VAL) * 0.05f;
    net_torque -= external_force_n * Physics::FLYWHEEL_RADIUS_M;

    // Friction model
    float omega_rad_s = flywheel_speed_dps * (Physics::PI_VAL / 180.0f);
    float viscous_friction = Physics::VISCOUS_FRICTION_COEFF * omega_rad_s;

    if (current_throttle_adc < Physics::THROTTLE_BRAKE_THRESHOLD) {
        viscous_friction *= Physics::ENGINE_BRAKE_FACTOR;
    }

    float coulomb_friction = 0.0f;
    if (flywheel_speed_dps > 1.0f) coulomb_friction = Physics::STATIC_FRICTION_NM;
    else if (flywheel_speed_dps < -1.0f) coulomb_friction = -Physics::STATIC_FRICTION_NM;

    net_torque -= (viscous_friction + coulomb_friction);
    flywheel_torque_nm = net_torque;

    // Integrate angular acceleration
    float angular_accel_rad_s2 = net_torque / Physics::FLYWHEEL_INERTIA;
    flywheel_speed_dps += rad2deg(angular_accel_rad_s2) * dt;

    if (flywheel_speed_dps < 0.0f) flywheel_speed_dps = 0.0f;

    flywheel_angle_deg = fmodf(flywheel_angle_deg + flywheel_speed_dps * dt, 360.0f);
    if (flywheel_angle_deg < 0.0f) flywheel_angle_deg += 360.0f;
}

void EngineSimulator::update_outputs() {
    uint8_t lambda_pwm = lambda_to_pwm(lambda_overall);
    analogWrite(Pins::LAMBDA, lambda_pwm);

    // Cam output: 2:1 crank-to-cam ratio
    float cam_angle_deg = crank_angle_720_deg * 0.5f;
    bool cam_pos_state = (fmodf(cam_angle_deg, 180.0f) < 90.0f);
    digitalWrite(Pins::CAM_POS, cam_pos_state ? HIGH : LOW);

    // 60-2 crank wheel signal
    float crank_angle_360_deg = fmodf(flywheel_angle_deg, 360.0f);
    if (crank_angle_360_deg < 0.0f) crank_angle_360_deg += 360.0f;

    int tooth = (int)floorf(crank_angle_360_deg / 6.0f);
    bool crank_state = true;

    if (tooth >= 58) {
        crank_state = false; // missing tooth gap
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

        if (dt > 0.05f) dt = 0.05f;
        if (dt < 0.0f) dt = Config::TARGET_DT_S;

        engine.update(dt);
    }

    engine.write_telemetry();
}
