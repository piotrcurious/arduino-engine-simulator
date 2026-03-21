// =============================================================
// engine_sim.cpp  –  EngineSimulator implementation
// Logic is unchanged from the original Arduino sketch; only
// the include and entry-point wiring differ.
// =============================================================
#include "engine_sim.h"
#include <cmath>
#include <cstring>
#include <algorithm>

// ---- setup --------------------------------------------------
void EngineSimulator::setup() {
    pinMode(Pins::CAM_POS,      ARD_OUTPUT);
    pinMode(Pins::CRANK_POS,    ARD_OUTPUT);
    pinMode(Pins::LAMBDA,       ARD_OUTPUT);
    pinMode(Pins::START_BUTTON, ARD_INPUT_PULLUP);
    // Initialise START_BUTTON pull-up: not pressed = ARD_HIGH
    MockHW::digital_io[Pins::START_BUTTON] = ARD_HIGH;
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        pinMode(Pins::INJ[i], ARD_INPUT);
        pinMode(Pins::IGN[i], ARD_INPUT);
    }
}

// ---- top-level tick -----------------------------------------
void EngineSimulator::update() {
    measure_ecu_inputs();
    update_dynamics();
    update_outputs();

    // Record scope data
    int deg = (int)crank_angle_720_deg % 720;
    uint8_t bits = 0;
    for (int i = 0; i < 4; ++i) {
        if (cylinders[i].is_injecting) bits |= (1 << i);
        if (cylinders[i].is_igniting)  bits |= (1 << (i + 4));
    }
    scope_buffer[deg] = bits;
}

// ---- read ECU output pins -----------------------------------
void EngineSimulator::measure_ecu_inputs() {
    unsigned long now_us = micros();
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        bool inj_state = (bool)digitalRead(Pins::INJ[i]);
        if (inj_state && !cylinders[i].is_injecting) {
            cylinders[i].is_injecting       = true;
            cylinders[i].injection_start_us = now_us;
        } else if (!inj_state && cylinders[i].is_injecting) {
            cylinders[i].is_injecting = false;
            unsigned long width_us = now_us - cylinders[i].injection_start_us;
            cylinders[i].injected_fuel_mass =
                config.inj_flow_rate * (width_us / 1e6f);
            if (cylinders[i].injected_fuel_mass < Engine::MIN_FUEL_MASS_KG)
                cylinders[i].injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
        }

        bool ign_state = (bool)digitalRead(Pins::IGN[i]);
        if (ign_state && !cylinders[i].is_igniting) {
            cylinders[i].is_igniting   = true;
            cylinders[i].ignition_fired = true;
        } else if (!ign_state && cylinders[i].is_igniting) {
            cylinders[i].is_igniting = false;
        }
    }
}

// ---- physics update -----------------------------------------
void EngineSimulator::update_dynamics() {
    int   throttle_adc = analogRead(Pins::THROTTLE_ADC);
    float throttle_frac = throttle_adc / Config::ADC_MAX_VAL;
    float engine_rpm    = dps_to_rpm(flywheel_speed_dps);

    float target_map = Physics::ATMOSPHERIC_PRESSURE_PA
                       * (throttle_frac + (1.0 - throttle_frac) * 0.3);
    target_map -= (engine_rpm / 8000.0f) * 70000.0f * (1.0 - throttle_frac);
    manifold_pressure_pa = 0.95f * manifold_pressure_pa
        + 0.05f * constrain(target_map, 25000.0f, Physics::ATMOSPHERIC_PRESSURE_PA);

    crank_angle_720_deg =
        fmod(crank_angle_720_deg + flywheel_speed_dps * Config::TIME_STEP_S, 720.0f);
    if (crank_angle_720_deg < 0.0f) crank_angle_720_deg += 720.0f;

    float net_torque  = 0.0f;
    float total_lambda = 0.0f;

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        int   cyl_idx        = Engine::FIRING_ORDER[i];
        float tdc_power_stroke = Engine::FIRING_START_DEG[i];
        float phase_deg      = crank_angle_720_deg - tdc_power_stroke;
        if (phase_deg < 0) phase_deg += 720.0f;

        // Reset ignition flag after power stroke
        if (phase_deg > 180.0f && phase_deg < 360.0f)
            cylinders[cyl_idx].ignition_fired = false;

        // Combustion torque
        if (cylinders[cyl_idx].ignition_fired &&
            phase_deg >= 0.0f && phase_deg <= 180.0f) {

            float ve = 0.85f;
            if (engine_rpm > 1000.0f && engine_rpm < 4500.0f) ve = 0.95f;
            else if (engine_rpm >= 4500.0f)                    ve = 0.88f;

            cylinders[cyl_idx].air_mass_kg =
                (manifold_pressure_pa * Engine::CYLINDER_DISPLACEMENT_M3 * ve)
                / (Physics::AIR_GAS_CONSTANT * Physics::INTAKE_TEMP_K);

            float fuel_mass = cylinders[cyl_idx].injected_fuel_mass;
            if (fuel_mass <= Engine::MIN_FUEL_MASS_KG && throttle_frac > 0.1f)
                fuel_mass = cylinders[cyl_idx].air_mass_kg / Engine::STOICHIOMETRIC_RATIO;

            cylinders[cyl_idx].lambda =
                (cylinders[cyl_idx].air_mass_kg / fuel_mass)
                / Engine::STOICHIOMETRIC_RATIO;

            float lam_eff = (cylinders[cyl_idx].lambda < 1.0f)
                ? (cylinders[cyl_idx].lambda - 0.4f) / 0.6f
                : 1.0f / cylinders[cyl_idx].lambda;
            lam_eff = constrain(lam_eff, 0.0f, 1.0f);

            float torque_curve =
                sinf(phase_deg * (Physics::PI_VAL / 180.0f));
            net_torque += config.peak_torque
                          * torque_curve * lam_eff;

            if (phase_deg > 170.0f)
                cylinders[cyl_idx].ignition_fired = false;
        }

        // Self-sustaining ignition trigger (no external ECU needed)
        if (!cylinders[cyl_idx].ignition_fired &&
            phase_deg < 5.0f && engine_rpm > 500.0f) {
            cylinders[cyl_idx].ignition_fired = true;
        }

        total_lambda += cylinders[cyl_idx].lambda;
    }
    lambda_overall = total_lambda / Engine::NUM_CYLINDERS;

    // Starter motor
    if (digitalRead(Pins::START_BUTTON) == ARD_LOW &&
        engine_rpm < Engine::STARTER_SPEED_THRESHOLD_RPM)
        net_torque += Engine::STARTER_TORQUE_NM;

    // External load (FORCE_ADC centred at 511.5)
    float external_force =
        (analogRead(Pins::FORCE_ADC) - Config::ADC_MID_VAL) * 0.01f;
    net_torque -= external_force * Physics::FLYWHEEL_RADIUS_M;

    // Friction
    float omega_rad_s = flywheel_speed_dps * (Physics::PI_VAL / 180.0f);
    float viscous     = config.viscous_friction * omega_rad_s;
    if (throttle_adc < Physics::THROTTLE_BRAKE_THRESHOLD)
        viscous *= config.engine_brake;
    float coulomb = (flywheel_speed_dps > 0.0f) ?  config.static_friction
                  : (flywheel_speed_dps < 0.0f) ? -config.static_friction
                  : 0.0f;
    net_torque -= (viscous + coulomb);

    flywheel_torque_nm = net_torque;
    float alpha_rad_s2 = net_torque / config.flywheel_inertia;
    flywheel_speed_dps += alpha_rad_s2 * (180.0f / Physics::PI_VAL) * Config::TIME_STEP_S;
    if (flywheel_speed_dps < 0.0f) flywheel_speed_dps = 0.0f;

    flywheel_angle_deg =
        fmod(flywheel_angle_deg + flywheel_speed_dps * Config::TIME_STEP_S, 360.0f);
    if (flywheel_angle_deg < 0.0f) flywheel_angle_deg += 360.0f;
}

// ---- drive output pins --------------------------------------
void EngineSimulator::update_outputs() {
    int lambda_pwm = map(constrain((float)lambda_overall, 0.5f, 1.5f),
                         0.5f, 1.5f, 0, 255);
    analogWrite(Pins::LAMBDA, lambda_pwm);

    float cam_angle_deg  = fmod(flywheel_angle_deg / 2.0f, 360.0f);
    bool  cam_pos_state  = (fmod(cam_angle_deg, 180.0f) < 90.0f);
    digitalWrite(Pins::CAM_POS, cam_pos_state ? ARD_HIGH : ARD_LOW);

    int  tooth       = (int)(flywheel_angle_deg / 6.0f);
    bool crank_state = (tooth >= 58)
        ? false
        : (fmod(flywheel_angle_deg, 6.0f) < 3.0f);
    digitalWrite(Pins::CRANK_POS, crank_state ? ARD_HIGH : ARD_LOW);
}

// ---- serial telemetry (prints to stdout) --------------------
void EngineSimulator::write_telemetry() {
    static unsigned long last_print_ms = 0;
    if (millis() - last_print_ms > 250) {
        last_print_ms = millis();
        Serial.print("RPM: ");
        Serial.print((int)dps_to_rpm(flywheel_speed_dps));
        Serial.print(" | MAP: ");
        Serial.print(manifold_pressure_pa / 1000.0f, 1);
        Serial.print(" kPa | Torque: ");
        Serial.print(flywheel_torque_nm, 2);
        Serial.print(" Nm | Lambda: ");
        Serial.println(lambda_overall, 2);
    }
}

// ---- telemetry snapshot for UI ------------------------------
void EngineSimulator::get_telemetry(Telemetry& t) const {
    t.rpm       = dps_to_rpm(flywheel_speed_dps);
    t.map_kpa   = manifold_pressure_pa / 1000.0f;
    t.torque_nm = flywheel_torque_nm;
    t.lambda    = lambda_overall;
    t.flywheel_angle = flywheel_angle_deg;
    t.crank_angle_720 = crank_angle_720_deg;
    t.cam_pos   = (MockHW::digital_io[Pins::CAM_POS]   != ARD_LOW);
    t.crank_pos = (MockHW::digital_io[Pins::CRANK_POS] != ARD_LOW);
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        t.cyl_injecting[i] = cylinders[i].is_injecting;
        t.cyl_igniting [i] = (bool)cylinders[i].ignition_fired;
        t.cyl_lambda   [i] = cylinders[i].lambda;
    }
    memcpy((void*)t.scope_buffer, (const void*)scope_buffer, 720);
}

void EngineSimulator::set_config(const TunableConfig& c) {
    config = c;
}

EngineSimulator::TunableConfig EngineSimulator::get_config() const {
    return config;
}
