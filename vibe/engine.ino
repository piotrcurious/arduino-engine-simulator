#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =================================================================
// == SMALL HELPERS
// =================================================================

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float wrap360(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

static inline float wrap720(float deg) {
    deg = fmodf(deg, 720.0f);
    if (deg < 0.0f) deg += 720.0f;
    return deg;
}

static inline float deg2rad(float deg) {
    return deg * 0.017453292519943295f;
}

static inline float rad2deg(float rad) {
    return rad * 57.29577951308232f;
}

static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    if (fabsf(in_max - in_min) < 1e-9f) return out_min;
    return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

static inline uint8_t lambda_to_pwm(float lambda) {
    float clamped = clampf(lambda, 0.5f, 1.5f);
    float norm = (clamped - 0.5f) / 1.0f;
    int pwm = (int)(norm * 255.0f + 0.5f);
    return (uint8_t)clampf((float)pwm, 0.0f, 255.0f);
}

// Signed flow from A to B. Positive = mass goes A -> B.
// Very simple compressible-orifice approximation, good enough for a lumped model.
static inline float signed_orifice_mdot(float p_a, float p_b, float t_a, float area, float cd, float r_gas) {
    if (area <= 0.0f) return 0.0f;
    float dp = p_a - p_b;
    if (fabsf(dp) < 1.0f) return 0.0f;
    float rho_a = p_a / fmaxf(r_gas * t_a, 1e-6f);
    rho_a = fmaxf(rho_a, 0.05f);
    float mdot = cd * area * sqrtf(2.0f * rho_a * fabsf(dp));
    return (dp > 0.0f) ? mdot : -mdot;
}

static inline float valve_lift_window(float phase_deg, float open_deg, float close_deg, float ramp_deg) {
    if (phase_deg < open_deg || phase_deg > close_deg) return 0.0f;

    float lift = 1.0f;
    if (phase_deg < open_deg + ramp_deg) {
        float x = clampf((phase_deg - open_deg) / ramp_deg, 0.0f, 1.0f);
        lift = 0.5f * (1.0f - cosf(Physics::PI_VAL * x));
    } else if (phase_deg > close_deg - ramp_deg) {
        float x = clampf((close_deg - phase_deg) / ramp_deg, 0.0f, 1.0f);
        lift = 0.5f * (1.0f - cosf(Physics::PI_VAL * x));
    }
    return clampf(lift, 0.0f, 1.0f);
}

static inline float wiebe(float x, float a, float m) {
    // x in [0,1]
    x = clampf(x, 0.0f, 1.0f);
    return 1.0f - expf(-a * powf(x, m + 1.0f));
}

// =================================================================
// == CONFIGURATION & CONSTANTS
// =================================================================

namespace Pins {
    constexpr uint8_t CAM_POS = 2;
    constexpr uint8_t CRANK_POS = 3;
    constexpr uint8_t INJ[] = {4, 5, 6, 7};
    constexpr uint8_t IGN[] = {8, 9, 10, 11};
    constexpr uint8_t LAMBDA = 12; // not PWM on Uno/Nano
    constexpr uint8_t START_BUTTON = 13;
    constexpr uint8_t THROTTLE_ADC = A0;
    constexpr uint8_t FORCE_ADC = A1;
}

namespace Config {
    constexpr float TARGET_DT_S = 0.0002f;
    constexpr float ADC_MAX_VAL = 1023.0f;
    constexpr float ADC_MID_VAL = 511.5f;
}

namespace Physics {
    constexpr float PI_VAL = 3.14159265f;
    constexpr float AIR_GAS_CONSTANT = 287.05f;
    constexpr float GAMMA_AIR = 1.34f;
    constexpr float INTAKE_TEMP_K = 293.15f;
    constexpr float ATMOSPHERIC_PRESSURE_PA = 101325.0f;
    constexpr float EXHAUST_PRESSURE_PA = 108000.0f;

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
    constexpr float SWEEP_VOLUME_M3 = (Physics::PI_VAL / 4.0f) * BORE_M * BORE_M * STROKE_M;

    constexpr float COMPRESSION_RATIO = 10.0f;
    constexpr float CLEARANCE_VOLUME_M3 = SWEEP_VOLUME_M3 / (COMPRESSION_RATIO - 1.0f);
    constexpr float TDC_VOLUME_M3 = CLEARANCE_VOLUME_M3;
    constexpr float BDC_VOLUME_M3 = CLEARANCE_VOLUME_M3 + SWEEP_VOLUME_M3;

    constexpr float MANIFOLD_VOLUME_M3 = 0.0028f;
    constexpr float MANIFOLD_TEMP_K = 305.0f;
    constexpr float EXHAUST_TEMP_K = 900.0f;

    constexpr float STOICHIOMETRIC_RATIO = 14.7f;
    constexpr float FUEL_LHV_J_PER_KG = 42.8e6f;

    constexpr float INJ_FLOW_RATE_KGS = 0.0002f;
    constexpr float STARTER_TORQUE_NM = 12.0f;
    constexpr float STARTER_SPEED_THRESHOLD_RPM = 400.0f;

    constexpr float THROTTLE_MAX_AREA_M2 = 3.0e-4f;
    constexpr float VALVE_MAX_AREA_M2 = 2.4e-4f;
    constexpr float CD_THROTTLE = 0.72f;
    constexpr float CD_VALVE = 0.68f;

    constexpr float INTAKE_OPEN_START_DEG = 345.0f;
    constexpr float INTAKE_OPEN_END_DEG   = 575.0f;
    constexpr float EXHAUST_OPEN_START_DEG = 165.0f;
    constexpr float EXHAUST_OPEN_END_DEG   = 395.0f;
    constexpr float VALVE_RAMP_DEG = 25.0f;

    constexpr float BURN_DURATION_DEG = 60.0f;
    constexpr float WIEBE_A = 5.5f;
    constexpr float WIEBE_M = 2.0f;
    constexpr float IGNITION_LEAD_DEG = 12.0f; // spark before TDC modelled as event timing, not automatic advance

    constexpr float BASE_TORQUE_SCALE = 0.25f;
    constexpr float INTAKE_PUMP_SCALE = 0.06f;
    constexpr float EXHAUST_PUMP_SCALE = 0.06f;
    constexpr float COMPRESSION_PUMP_SCALE = 0.18f;
    constexpr float COMBUSTION_EFFICIENCY = 0.92f;
}

// =================================================================
// == ENGINE SIMULATOR
// =================================================================

class EngineSimulator {
public:
    void setup();
    void update(float dt);
    void write_telemetry();

private:
    enum class Stroke : uint8_t { POWER, EXHAUST, INTAKE, COMPRESSION };

    struct Cylinder {
        // ECU input edge tracking
        bool inj_prev = false;
        bool ign_prev = false;

        bool injector_open = false;
        bool ignition_active = false;

        unsigned long injection_start_us = 0;

        // Mixture / thermodynamics
        float air_mass_kg = 1.0e-5f;
        float fuel_mass_kg = 0.0f;
        float temp_k = Physics::INTAKE_TEMP_K;
        float pressure_pa = Physics::ATMOSPHERIC_PRESSURE_PA;

        // Spark / combustion
        float spark_angle_720_deg = -1.0f;
        bool spark_valid = false;
        float burn_progress = 0.0f;

        // Geometry tracking
        float last_volume_m3 = Engine::BDC_VOLUME_M3;
        float last_phase_deg = 0.0f;

        // Telemetry
        float lambda = 1.0f;
    };

    Cylinder cylinders[Engine::NUM_CYLINDERS];

    float manifold_mass_kg;
    float manifold_temp_k = Engine::MANIFOLD_TEMP_K;
    float manifold_pressure_pa = Physics::ATMOSPHERIC_PRESSURE_PA;

    float flywheel_angle_deg = 0.0f;
    float flywheel_speed_dps = 0.0f;
    float flywheel_torque_nm = 0.0f;
    float crank_angle_720_deg = 0.0f;
    float lambda_overall = 1.0f;

    int current_throttle_adc = 0;
    int current_force_adc = (int)Config::ADC_MID_VAL;
    bool toggle_adc_read = false;

    float dps_to_rpm(float dps) const { return dps * (60.0f / 360.0f); }
    float rpm_to_dps(float rpm) const { return rpm * (360.0f / 60.0f); }

    Stroke stroke_for_phase(float phase_deg) const;
    float cylinder_volume_from_tdc(float theta_from_tdc_deg) const;
    float cylinder_pressure_from_state(const Cylinder& c, float volume_m3) const;
    float manifold_pressure_from_state() const;
    void measure_ecu_inputs();
    void update_dynamics(float dt);
    void update_cylinder(int idx, float phase_deg, float engine_rpm, float throttle_frac, float dt, float& net_torque);
    void update_outputs();
};

EngineSimulator engine;
volatile bool run_simulation_tick = false;
unsigned long last_update_us = 0;

void timer_isr() {
    run_simulation_tick = true;
}

// =================================================================
// == IMPLEMENTATION
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

    manifold_mass_kg = (Physics::ATMOSPHERIC_PRESSURE_PA * Engine::MANIFOLD_VOLUME_M3) /
                       (Physics::AIR_GAS_CONSTANT * Engine::MANIFOLD_TEMP_K);

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

float EngineSimulator::cylinder_volume_from_tdc(float theta_from_tdc_deg) const {
    float theta = clampf(theta_from_tdc_deg, 0.0f, 180.0f);
    float x = deg2rad(theta);
    return Engine::TDC_VOLUME_M3 + 0.5f * Engine::SWEEP_VOLUME_M3 * (1.0f - cosf(x));
}

float EngineSimulator::cylinder_pressure_from_state(const Cylinder& c, float volume_m3) const {
    float total_mass = c.air_mass_kg + c.fuel_mass_kg;
    total_mass = fmaxf(total_mass, 1e-9f);
    float R = Physics::AIR_GAS_CONSTANT;
    return (total_mass * R * c.temp_k) / fmaxf(volume_m3, 1e-9f);
}

float EngineSimulator::manifold_pressure_from_state() const {
    return (manifold_mass_kg * Physics::AIR_GAS_CONSTANT * manifold_temp_k) / Engine::MANIFOLD_VOLUME_M3;
}

void EngineSimulator::measure_ecu_inputs() {
    unsigned long now_us = micros();

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        bool inj_state = (digitalRead(Pins::INJ[i]) == HIGH);

        if (inj_state && !cylinders[i].inj_prev) {
            cylinders[i].injector_open = true;
            cylinders[i].injection_start_us = now_us;
        } else if (!inj_state && cylinders[i].inj_prev) {
            cylinders[i].injector_open = false;
            float width_s = (now_us - cylinders[i].injection_start_us) / 1000000.0f;
            float fuel_mass = Engine::INJ_FLOW_RATE_KGS * width_s;
            if (fuel_mass > 0.0f) {
                cylinders[i].fuel_mass_kg += fuel_mass;
            }
        }
        cylinders[i].inj_prev = inj_state;

        bool ign_state = (digitalRead(Pins::IGN[i]) == HIGH);

        if (ign_state && !cylinders[i].ign_prev) {
            cylinders[i].ignition_active = true;
            cylinders[i].spark_angle_720_deg = crank_angle_720_deg;
            cylinders[i].spark_valid = true;
            cylinders[i].burn_progress = 0.0f;
        } else if (!ign_state && cylinders[i].ign_prev) {
            cylinders[i].ignition_active = false;
        }
        cylinders[i].ign_prev = ign_state;
    }
}

void EngineSimulator::update_cylinder(int idx, float phase_deg, float engine_rpm, float throttle_frac, float dt, float& net_torque) {
    Cylinder& c = cylinders[idx];
    Stroke stroke = stroke_for_phase(phase_deg);

    // Cylinder volume
    float theta_from_tdc_deg = 0.0f;
    if (stroke == Stroke::POWER) {
        theta_from_tdc_deg = phase_deg;
    } else if (stroke == Stroke::EXHAUST) {
        theta_from_tdc_deg = phase_deg - 180.0f;
    } else if (stroke == Stroke::INTAKE) {
        theta_from_tdc_deg = phase_deg - 360.0f;
    } else {
        theta_from_tdc_deg = 720.0f - phase_deg;
    }
    theta_from_tdc_deg = clampf(theta_from_tdc_deg, 0.0f, 180.0f);

    float volume_m3 = cylinder_volume_from_tdc(theta_from_tdc_deg);
    float piston_area_m2 = (Physics::PI_VAL * Engine::BORE_M * Engine::BORE_M) * 0.25f;
    float crank_radius_m = Engine::STROKE_M * 0.5f;

    // Valve openings
    float intake_lift = valve_lift_window(
        phase_deg,
        Engine::INTAKE_OPEN_START_DEG,
        Engine::INTAKE_OPEN_END_DEG,
        Engine::VALVE_RAMP_DEG
    );
    float exhaust_lift = valve_lift_window(
        phase_deg,
        Engine::EXHAUST_OPEN_START_DEG,
        Engine::EXHAUST_OPEN_END_DEG,
        Engine::VALVE_RAMP_DEG
    );

    float intake_area = Engine::VALVE_MAX_AREA_M2 * intake_lift;
    float exhaust_area = Engine::VALVE_MAX_AREA_M2 * exhaust_lift;

    // Manifold state
    manifold_pressure_pa = manifold_pressure_from_state();

    // 1) Intake flow
    float old_air = c.air_mass_kg;
    float old_fuel = c.fuel_mass_kg;
    float old_total = fmaxf(old_air + old_fuel, 1e-9f);

    float dm_intake = signed_orifice_mdot(
        manifold_pressure_pa,
        c.pressure_pa,
        manifold_temp_k,
        intake_area,
        Engine::CD_VALVE,
        Physics::AIR_GAS_CONSTANT
    ) * dt;

    if (fabsf(dm_intake) > 0.0f) {
        // Positive = manifold -> cylinder, negative = cylinder -> manifold
        manifold_mass_kg -= dm_intake;
        c.air_mass_kg += dm_intake;

        if (dm_intake > 0.0f) {
            float new_total = fmaxf(c.air_mass_kg + c.fuel_mass_kg, 1e-9f);
            c.temp_k = (old_total * c.temp_k + dm_intake * manifold_temp_k) / new_total;
        } else {
            // Backflow removes mixture proportionally
            float out_mass = -dm_intake;
            float new_total = fmaxf(c.air_mass_kg + c.fuel_mass_kg, 1e-9f);
            if (new_total < old_total) {
                float air_frac = old_air / old_total;
                float fuel_frac = old_fuel / old_total;
                c.air_mass_kg = fmaxf(c.air_mass_kg, 0.0f);
                c.fuel_mass_kg = fmaxf(c.fuel_mass_kg, 0.0f);
                c.air_mass_kg = fmaxf(c.air_mass_kg - out_mass * air_frac, 0.0f);
                c.fuel_mass_kg = fmaxf(c.fuel_mass_kg - out_mass * fuel_frac, 0.0f);
            }
        }
    }

    // 2) Exhaust flow
    float dm_exhaust = 0.0f;
    if (exhaust_area > 0.0f) {
        dm_exhaust = signed_orifice_mdot(
            c.pressure_pa,
            Physics::EXHAUST_PRESSURE_PA,
            c.temp_k,
            exhaust_area,
            Engine::CD_VALVE,
            Physics::AIR_GAS_CONSTANT
        ) * dt;

        if (dm_exhaust > 0.0f) {
            float total = fmaxf(c.air_mass_kg + c.fuel_mass_kg, 1e-9f);
            float remove = fminf(dm_exhaust, 0.95f * total);
            float air_frac = c.air_mass_kg / total;
            float fuel_frac = c.fuel_mass_kg / total;

            c.air_mass_kg = fmaxf(c.air_mass_kg - remove * air_frac, 0.0f);
            c.fuel_mass_kg = fmaxf(c.fuel_mass_kg - remove * fuel_frac, 0.0f);
        }
    }

    // 3) Compression/expansion thermodynamics
    if ((stroke == Stroke::COMPRESSION || stroke == Stroke::POWER) &&
        intake_area <= 1e-8f && exhaust_area <= 1e-8f) {
        if (c.last_volume_m3 > 1e-9f) {
            float ratio = c.last_volume_m3 / volume_m3;
            c.temp_k *= powf(ratio, Physics::GAMMA_AIR - 1.0f);
        }
    }

    // 4) Combustion with Wiebe burn law
    float combustion_lambda = (c.air_mass_kg / fmaxf(c.fuel_mass_kg, 1e-10f)) / Engine::STOICHIOMETRIC_RATIO;
    c.lambda = clampf(combustion_lambda, 0.2f, 10.0f);

    bool in_burn_window = false;
    float delta_since_spark_deg = 0.0f;

    if (c.spark_valid && c.spark_angle_720_deg >= 0.0f) {
        delta_since_spark_deg = phase_deg - c.spark_angle_720_deg;
        if (delta_since_spark_deg < 0.0f) delta_since_spark_deg += 720.0f;

        // Allow spark kernel to bridge TDC, but only release heat in late compression / power
        if ((stroke == Stroke::COMPRESSION && phase_deg > 660.0f) || stroke == Stroke::POWER) {
            if (delta_since_spark_deg >= 0.0f && delta_since_spark_deg <= Engine::BURN_DURATION_DEG) {
                in_burn_window = true;
            }
        }
    }

    if (in_burn_window && c.fuel_mass_kg > 1e-10f) {
        float x = delta_since_spark_deg / Engine::BURN_DURATION_DEG;
        float xb = wiebe(x, Engine::WIEBE_A, Engine::WIEBE_M);
        float dxb = xb - c.burn_progress;
        dxb = clampf(dxb, 0.0f, 1.0f);

        if (dxb > 0.0f) {
            float lambda_eff = 1.0f;
            if (c.lambda < 1.0f) {
                lambda_eff = 0.55f + 0.45f * c.lambda;
            } else {
                lambda_eff = 1.0f / (1.0f + 0.7f * (c.lambda - 1.0f));
            }
            lambda_eff = clampf(lambda_eff, 0.0f, 1.0f);

            float q_release_j = dxb * c.fuel_mass_kg * Engine::FUEL_LHV_J_PER_KG * Engine::COMBUSTION_EFFICIENCY * lambda_eff;

            float total_mass = fmaxf(c.air_mass_kg + c.fuel_mass_kg, 1e-9f);
            float cv = Physics::AIR_GAS_CONSTANT / (Physics::GAMMA_AIR - 1.0f);
            float dT = q_release_j / (total_mass * cv);
            c.temp_k += dT;

            c.burn_progress = xb;
        }
    }

    // Once burn is complete and the cylinder is well into exhaust, clear spent fuel / spark state
    if (c.spark_valid && delta_since_spark_deg > (Engine::BURN_DURATION_DEG + 25.0f)) {
        c.spark_valid = false;
        c.spark_angle_720_deg = -1.0f;
        c.burn_progress = 0.0f;
        c.fuel_mass_kg = 0.0f;
    }

    // Residual gas carryover at cycle reset
    if (phase_deg < c.last_phase_deg) {
        c.spark_valid = false;
        c.spark_angle_720_deg = -1.0f;
        c.burn_progress = 0.0f;
        c.fuel_mass_kg = 0.0f;
        c.air_mass_kg = fmaxf(c.air_mass_kg * 0.15f, 1.0e-6f);
        c.temp_k = clampf(c.temp_k * 0.92f, Physics::INTAKE_TEMP_K, 2800.0f);
    }

    // Keep temperature within sane bounds
    c.temp_k = clampf(c.temp_k, 250.0f, 3200.0f);

    // Pressure from ideal gas
    c.pressure_pa = cylinder_pressure_from_state(c, volume_m3);

    // 5) Torque from pressure acting on piston
    float theta_rad = deg2rad(theta_from_tdc_deg);
    float sin_term = sinf(theta_rad);
    float ref_pressure = Physics::ATMOSPHERIC_PRESSURE_PA;

    if (stroke == Stroke::INTAKE) {
        ref_pressure = manifold_pressure_pa;
    } else if (stroke == Stroke::EXHAUST) {
        ref_pressure = Physics::EXHAUST_PRESSURE_PA;
    } else if (stroke == Stroke::COMPRESSION) {
        ref_pressure = Physics::ATMOSPHERIC_PRESSURE_PA;
    }

    float delta_p = c.pressure_pa - ref_pressure;
    float torque = 0.0f;

    if (stroke == Stroke::POWER) {
        torque =  delta_p * piston_area_m2 * crank_radius_m * sin_term * Engine::BASE_TORQUE_SCALE;
    } else if (stroke == Stroke::COMPRESSION) {
        torque = -fabsf(delta_p) * piston_area_m2 * crank_radius_m * sin_term * Engine::COMPRESSION_PUMP_SCALE;
    } else if (stroke == Stroke::INTAKE) {
        torque = -fabsf(delta_p) * piston_area_m2 * crank_radius_m * sin_term * Engine::INTAKE_PUMP_SCALE;
    } else {
        torque = -fabsf(delta_p) * piston_area_m2 * crank_radius_m * sin_term * Engine::EXHAUST_PUMP_SCALE;
    }

    net_torque += torque;

    c.last_volume_m3 = volume_m3;
    c.last_phase_deg = phase_deg;
}

void EngineSimulator::update_dynamics(float dt) {
    // Alternate ADC reads to reduce cost
    if (toggle_adc_read) {
        current_throttle_adc = analogRead(Pins::THROTTLE_ADC);
    } else {
        current_force_adc = analogRead(Pins::FORCE_ADC);
    }
    toggle_adc_read = !toggle_adc_read;

    float throttle_frac = clampf(current_throttle_adc / Config::ADC_MAX_VAL, 0.0f, 1.0f);
    float engine_rpm = fmaxf(dps_to_rpm(flywheel_speed_dps), 0.0f);

    // Manifold model: throttle flow from atmosphere into a lumped plenum
    float throttle_area = Engine::THROTTLE_MAX_AREA_M2 * throttle_frac * throttle_frac;
    float p_map = manifold_pressure_from_state();

    float dm_throttle = signed_orifice_mdot(
        Physics::ATMOSPHERIC_PRESSURE_PA,
        p_map,
        Engine::MANIFOLD_TEMP_K,
        throttle_area,
        Engine::CD_THROTTLE,
        Physics::AIR_GAS_CONSTANT
    ) * dt;

    manifold_mass_kg += dm_throttle;

    // Small thermal relaxation toward ambient
    manifold_temp_k += (Engine::MANIFOLD_TEMP_K - manifold_temp_k) * clampf(dt * 1.5f, 0.0f, 1.0f);
    manifold_temp_k = clampf(manifold_temp_k, 280.0f, 380.0f);

    manifold_pressure_pa = manifold_pressure_from_state();
    manifold_pressure_pa = clampf(manifold_pressure_pa, 20000.0f, Physics::ATMOSPHERIC_PRESSURE_PA);

    // Advance crank
    crank_angle_720_deg = wrap720(crank_angle_720_deg + flywheel_speed_dps * dt);

    float net_torque = 0.0f;
    float lambda_sum = 0.0f;

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        int cyl_idx = Engine::FIRING_ORDER[i];
        float phase_deg = wrap720(crank_angle_720_deg - Engine::FIRING_START_DEG[i]);
        update_cylinder(cyl_idx, phase_deg, engine_rpm, throttle_frac, dt, net_torque);
        lambda_sum += cylinders[cyl_idx].lambda;
    }

    lambda_overall = lambda_sum / (float)Engine::NUM_CYLINDERS;

    // Starter motor
    if (digitalRead(Pins::START_BUTTON) == LOW &&
        engine_rpm < Engine::STARTER_SPEED_THRESHOLD_RPM) {
        net_torque += Engine::STARTER_TORQUE_NM;
    }

    // External load
    float external_force_n = (current_force_adc - Config::ADC_MID_VAL) * 0.05f;
    net_torque -= external_force_n * Physics::FLYWHEEL_RADIUS_M;

    // Friction
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

    // Integrate rotational dynamics
    float angular_accel_rad_s2 = net_torque / Physics::FLYWHEEL_INERTIA;
    flywheel_speed_dps += rad2deg(angular_accel_rad_s2) * dt;

    if (flywheel_speed_dps < 0.0f) flywheel_speed_dps = 0.0f;

    flywheel_angle_deg = wrap360(flywheel_angle_deg + flywheel_speed_dps * dt);
}

void EngineSimulator::update_outputs() {
    analogWrite(Pins::LAMBDA, lambda_to_pwm(lambda_overall));

    // Cam signal: 2:1 crank-to-cam
    float cam_angle_deg = crank_angle_720_deg * 0.5f;
    bool cam_pos_state = (fmodf(cam_angle_deg, 180.0f) < 90.0f);
    digitalWrite(Pins::CAM_POS, cam_pos_state ? HIGH : LOW);

    // Crank signal: 60-2 style wheel on 360 deg
    float crank_angle_360_deg = wrap360(flywheel_angle_deg);
    int tooth = (int)floorf(crank_angle_360_deg / 6.0f);
    bool crank_state = true;

    if (tooth >= 58) {
        crank_state = false; // missing-tooth gap
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
        Serial.print(" LAMBDA:"); Serial.print(lambda_overall, 2);
        Serial.print(" CylP_kPa:"); Serial.print(cylinders[0].pressure_pa / 1000.0f, 1);
        Serial.println();
    }
}

// =================================================================
// == ARDUINO SKETCH
// =================================================================

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
