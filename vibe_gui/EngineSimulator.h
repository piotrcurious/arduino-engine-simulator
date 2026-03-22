#ifndef ENGINE_SIMULATOR_H
#define ENGINE_SIMULATOR_H

#include "EngineConfig.h"
#include <stdint.h>
#include <cstring>

class EngineSimulator {
public:
    struct TunableConfig {
        float flywheel_inertia = Physics::FLYWHEEL_INERTIA;
        float viscous_friction = Physics::VISCOUS_FRICTION_COEFF;
        float static_friction  = Physics::STATIC_FRICTION_NM;
        float engine_brake     = Physics::ENGINE_BRAKE_FACTOR;
        float peak_torque      = Engine::BASE_TORQUE_SCALE;
        float inj_flow_rate    = Engine::INJ_FLOW_RATE_KGS;
    };

    struct Telemetry {
        float rpm        = 0.0f;
        float map_kpa    = 101.325f;
        float torque_nm  = 0.0f;
        float lambda     = 1.0f;
        float flywheel_angle = 0.0f;
        float crank_angle_720 = 0.0f;
        uint8_t scope_buffer[720];
    };

    void setup();
    void update(float dt);
    void write_telemetry();
    float getRPM() const;
    void setRPM(float rpm);
    void reset();
    void get_telemetry(Telemetry& t) const;
    void set_config(const TunableConfig& c);
    TunableConfig get_config() const;

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

    TunableConfig config;
    uint8_t scope_buffer[720] = {0};
};

#endif
