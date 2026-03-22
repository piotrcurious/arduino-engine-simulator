#include "mock_arduino.h"
#include "EngineSimulator.h"
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
    x = clampf(x, 0.0f, 1.0f);
    return 1.0f - expf(-a * powf(x, m + 1.0f));
}

// =================================================================
// == ENGINE SIMULATOR IMPLEMENTATION
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
    config = TunableConfig();
    reset();
}

void EngineSimulator::reset() {
    manifold_temp_k = Engine::MANIFOLD_TEMP_K;
    manifold_mass_kg = (Physics::ATMOSPHERIC_PRESSURE_PA * Engine::MANIFOLD_VOLUME_M3) /
                       (Physics::AIR_GAS_CONSTANT * manifold_temp_k);
    manifold_pressure_pa = Physics::ATMOSPHERIC_PRESSURE_PA;
    flywheel_speed_dps = 0;
    flywheel_angle_deg = 0;
    crank_angle_720_deg = 0;
    lambda_overall = 1.0f;

    for (int i = 0; i < Engine::NUM_CYLINDERS; i++) {
        cylinders[i] = Cylinder();
        cylinders[i].temp_k = Physics::INTAKE_TEMP_K;
        cylinders[i].air_mass_kg = 1.0e-5f;
    }
    memset(scope_buffer, 0, 720);
}

float EngineSimulator::cylinder_volume_from_tdc(float theta_from_tdc_deg) const {
    float theta = clampf(theta_from_tdc_deg, 0.0f, 180.0f);
    float x = deg2rad(theta);
    return Engine::TDC_VOLUME_M3 + 0.5f * Engine::SWEEP_VOLUME_M3 * (1.0f - cosf(x));
}

float EngineSimulator::cylinder_pressure_from_state(const Cylinder& c, float volume_m3) const {
    float total_mass = fmaxf(c.air_mass_kg + c.fuel_mass_kg, 1e-10f);
    return (total_mass * Physics::AIR_GAS_CONSTANT * c.temp_k) / fmaxf(volume_m3, 1e-9f);
}

float EngineSimulator::manifold_pressure_from_state() const {
    return (fmaxf(manifold_mass_kg, 1e-10f) * Physics::AIR_GAS_CONSTANT * manifold_temp_k) / Engine::MANIFOLD_VOLUME_M3;
}

void EngineSimulator::measure_ecu_inputs() {
    unsigned long now_us = micros();
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        bool inj_state = (digitalRead(Pins::INJ[i]) == ARD_HIGH);
        if (inj_state && !cylinders[i].inj_prev) {
            cylinders[i].injector_open = true;
            cylinders[i].injection_start_us = now_us;
        } else if (!inj_state && cylinders[i].inj_prev) {
            cylinders[i].injector_open = false;
            float width_s = (float)(now_us - cylinders[i].injection_start_us) / 1000000.0f;
            if (width_s < 0) width_s = 0;
            cylinders[i].fuel_mass_kg += config.inj_flow_rate * width_s;
        }
        cylinders[i].inj_prev = inj_state;

        bool ign_state = (digitalRead(Pins::IGN[i]) == ARD_HIGH);
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
    Stroke stroke = (phase_deg < 180.0f) ? Stroke::POWER :
                    (phase_deg < 360.0f) ? Stroke::EXHAUST :
                    (phase_deg < 540.0f) ? Stroke::INTAKE : Stroke::COMPRESSION;

    float theta_from_tdc_deg = (stroke == Stroke::POWER) ? phase_deg :
                               (stroke == Stroke::EXHAUST) ? phase_deg - 180.0f :
                               (stroke == Stroke::INTAKE) ? phase_deg - 360.0f : 720.0f - phase_deg;
    theta_from_tdc_deg = clampf(theta_from_tdc_deg, 0.0f, 180.0f);

    float volume_m3 = cylinder_volume_from_tdc(theta_from_tdc_deg);
    float intake_lift = valve_lift_window(phase_deg, Engine::INTAKE_OPEN_START_DEG, Engine::INTAKE_OPEN_END_DEG, Engine::VALVE_RAMP_DEG);
    float exhaust_lift = valve_lift_window(phase_deg, Engine::EXHAUST_OPEN_START_DEG, Engine::EXHAUST_OPEN_END_DEG, Engine::VALVE_RAMP_DEG);
    float intake_area = Engine::VALVE_MAX_AREA_M2 * intake_lift;
    float exhaust_area = Engine::VALVE_MAX_AREA_M2 * exhaust_lift;

    float p_map = manifold_pressure_from_state();
    float dm_intake = signed_orifice_mdot(p_map, c.pressure_pa, manifold_temp_k, intake_area, Engine::CD_VALVE, Physics::AIR_GAS_CONSTANT) * dt;
    if (fabsf(dm_intake) > 0.0f) {
        manifold_mass_kg -= dm_intake;
        c.air_mass_kg += dm_intake;
        if (dm_intake > 0.0f) {
            float total = fmaxf(c.air_mass_kg + c.fuel_mass_kg, 1e-10f);
            c.temp_k = ( (total-dm_intake)*c.temp_k + dm_intake*manifold_temp_k ) / total;
        }
    }

    float dm_exhaust = signed_orifice_mdot(c.pressure_pa, Physics::EXHAUST_PRESSURE_PA, c.temp_k, exhaust_area, Engine::CD_VALVE, Physics::AIR_GAS_CONSTANT) * dt;
    if (dm_exhaust > 0.0f) {
        float total = fmaxf(c.air_mass_kg + c.fuel_mass_kg, 1e-10f);
        float remove = fminf(dm_exhaust, 0.9f * total);
        float frac = remove / total;
        c.air_mass_kg -= c.air_mass_kg * frac;
        c.fuel_mass_kg -= c.fuel_mass_kg * frac;
    }

    if ((stroke == Stroke::COMPRESSION || stroke == Stroke::POWER) && intake_area < 1e-7f && exhaust_area < 1e-7f) {
        if (c.last_volume_m3 > 1e-9f) c.temp_k *= powf(c.last_volume_m3 / volume_m3, Physics::GAMMA_AIR - 1.0f);
    }

    c.lambda = (c.air_mass_kg / fmaxf(c.fuel_mass_kg, 1e-10f)) / Engine::STOICHIOMETRIC_RATIO;
    c.lambda = clampf(c.lambda, 0.2f, 10.0f);

    if (c.spark_valid && ((stroke == Stroke::COMPRESSION && phase_deg > 660.0f) || stroke == Stroke::POWER)) {
        float ds = wrap720(phase_deg - c.spark_angle_720_deg);
        if (ds >= 0.0f && ds <= Engine::BURN_DURATION_DEG) {
            float xb = wiebe(ds / Engine::BURN_DURATION_DEG, Engine::WIEBE_A, Engine::WIEBE_M);
            float dxb = clampf(xb - c.burn_progress, 0.0f, 1.0f);
            if (dxb > 0.0f && c.fuel_mass_kg > 1e-10f) {
                float leff = (c.lambda < 1.0f) ? 0.55f + 0.45f*c.lambda : 1.0f/(1.0f + 0.7f*(c.lambda-1.0f));
                float q = dxb * c.fuel_mass_kg * Engine::FUEL_LHV_J_PER_KG * Engine::COMBUSTION_EFFICIENCY * clampf(leff,0.0f,1.0f);
                c.temp_k += q / (fmaxf(c.air_mass_kg + c.fuel_mass_kg, 1e-10f) * (Physics::AIR_GAS_CONSTANT / (Physics::GAMMA_AIR - 1.0f)));
                c.burn_progress = xb;
            }
        }
    }

    if (c.last_phase_deg < 360.0f && phase_deg >= 360.0f) {
        c.spark_valid = false; c.burn_progress = 0.0f; c.fuel_mass_kg = 0.0f;
        c.air_mass_kg = fmaxf(c.air_mass_kg * 0.1f, 1.0e-6f);
        c.temp_k = clampf(c.temp_k * 0.8f, Physics::INTAKE_TEMP_K, 2500.0f);
    }
    c.temp_k = clampf(c.temp_k, 200.0f, 3000.0f);
    c.pressure_pa = cylinder_pressure_from_state(c, volume_m3);

    float sin_t = sinf(deg2rad(theta_from_tdc_deg));
    float pref = (stroke == Stroke::INTAKE) ? p_map : (stroke == Stroke::EXHAUST) ? Physics::EXHAUST_PRESSURE_PA : Physics::ATMOSPHERIC_PRESSURE_PA;
    float torq = (c.pressure_pa - pref) * (Physics::PI_VAL*Engine::BORE_M*Engine::BORE_M*0.25f) * (Engine::STROKE_M*0.5f) * sin_t;
    net_torque += torq * ((stroke == Stroke::POWER) ? config.peak_torque : Engine::COMPRESSION_PUMP_SCALE);

    c.last_volume_m3 = volume_m3; c.last_phase_deg = phase_deg;
}

void EngineSimulator::update_dynamics(float dt) {
    if (toggle_adc_read) current_throttle_adc = analogRead(Pins::THROTTLE_ADC);
    else current_force_adc = analogRead(Pins::FORCE_ADC);
    toggle_adc_read = !toggle_adc_read;

    float tfrac = clampf((float)current_throttle_adc / Config::ADC_MAX_VAL, 0.0f, 1.0f);
    float p_map = manifold_pressure_from_state();
    float dm_t = signed_orifice_mdot(Physics::ATMOSPHERIC_PRESSURE_PA, p_map, Engine::MANIFOLD_TEMP_K, Engine::THROTTLE_MAX_AREA_M2 * tfrac * tfrac, Engine::CD_THROTTLE, Physics::AIR_GAS_CONSTANT) * dt;
    manifold_mass_kg += dm_t;
    manifold_temp_k += (Engine::MANIFOLD_TEMP_K - manifold_temp_k) * clampf(dt * 2.0f, 0.0f, 1.0f);
    manifold_pressure_pa = clampf(manifold_pressure_from_state(), 1000.0f, Physics::ATMOSPHERIC_PRESSURE_PA);

    crank_angle_720_deg = wrap720(crank_angle_720_deg + flywheel_speed_dps * dt);
    float net_t = 0.0f, lsum = 0.0f;
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        update_cylinder(Engine::FIRING_ORDER[i], wrap720(crank_angle_720_deg - Engine::FIRING_START_DEG[i]), getRPM(), tfrac, dt, net_t);
        lsum += cylinders[i].lambda;
    }
    lambda_overall = lsum / 4.0f;

    if (digitalRead(Pins::START_BUTTON) == ARD_LOW && getRPM() < Engine::STARTER_SPEED_THRESHOLD_RPM) net_t += Engine::STARTER_TORQUE_NM;
    net_t -= (current_force_adc - Config::ADC_MID_VAL) * 0.1f * Physics::FLYWHEEL_RADIUS_M;
    float omega = flywheel_speed_dps * (Physics::PI_VAL / 180.0f);
    float fric = config.viscous_friction * omega;
    if (current_throttle_adc < Physics::THROTTLE_BRAKE_THRESHOLD) fric *= config.engine_brake;
    net_t -= (fric + ((flywheel_speed_dps > 1.0f) ? config.static_friction : (flywheel_speed_dps < -1.0f) ? -config.static_friction : 0.0f));

    flywheel_torque_nm = net_t;
    flywheel_speed_dps = fmaxf(0.0f, flywheel_speed_dps + rad2deg(net_t / config.flywheel_inertia) * dt);
    flywheel_angle_deg = wrap360(flywheel_angle_deg + flywheel_speed_dps * dt);
}

float EngineSimulator::getRPM() const { return flywheel_speed_dps / 6.0f; }
void EngineSimulator::setRPM(float rpm) { flywheel_speed_dps = rpm * 6.0f; }

void EngineSimulator::update(float dt) {
    measure_ecu_inputs();
    update_dynamics(dt);
    update_outputs();
}

void EngineSimulator::update_outputs() {
    analogWrite(Pins::LAMBDA, lambda_to_pwm(lambda_overall));
    digitalWrite(Pins::CAM_POS, (crank_angle_720_deg < 360.0f) ? ARD_HIGH : ARD_LOW);
    float a360 = wrap360(flywheel_angle_deg);
    int tooth = (int)(a360 / 6.0f);
    digitalWrite(Pins::CRANK_POS, (tooth < 58 && fmodf(a360, 6.0f) < 3.0f) ? ARD_HIGH : ARD_LOW);

    int idx = (int)crank_angle_720_deg;
    if (idx >= 0 && idx < 720) {
        uint8_t mask = 0;
        for (int i = 0; i < 4; i++) {
            if (MockHW::digital_io[Pins::INJ[i]] == ARD_HIGH) mask |= (1 << i);
            if (MockHW::digital_io[Pins::IGN[i]] == ARD_HIGH) mask |= (1 << (i + 4));
        }
        scope_buffer[idx] = mask;
    }
}

void EngineSimulator::get_telemetry(Telemetry& t) const {
    t.rpm = getRPM(); t.map_kpa = manifold_pressure_pa / 1000.0f; t.torque_nm = flywheel_torque_nm;
    t.lambda = lambda_overall; t.flywheel_angle = flywheel_angle_deg; t.crank_angle_720 = crank_angle_720_deg;
    memcpy(t.scope_buffer, scope_buffer, 720);
}

void EngineSimulator::set_config(const TunableConfig& c) { config = c; }
EngineSimulator::TunableConfig EngineSimulator::get_config() const { return config; }
void EngineSimulator::write_telemetry() {}
