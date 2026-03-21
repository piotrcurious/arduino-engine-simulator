#pragma once
// =============================================================
// engine_sim.h  –  EngineSimulator class declaration
// Ported from the Arduino sketch; all hardware calls go through
// mock_arduino.h.  Telemetry is exposed via get_telemetry().
// =============================================================
#include "mock_arduino.h"
#include <cmath>
#include <vector>

// ----- configuration namespaces (from original sketch) -------
namespace Pins {
    constexpr uint8_t CAM_POS      = 2;
    constexpr uint8_t CRANK_POS    = 3;
    constexpr uint8_t INJ[]        = {4, 5, 6, 7};
    constexpr uint8_t IGN[]        = {8, 9, 10, 11};
    constexpr uint8_t LAMBDA       = 12;
    constexpr uint8_t START_BUTTON = 13;
    constexpr uint8_t THROTTLE_ADC = A0;
    constexpr uint8_t FORCE_ADC    = A1;
}

namespace Config {
    constexpr float TIME_STEP_S = 0.001f;
    enum class CamWheelType { WHEEL_1, WHEEL_2, WHEEL_3 };
    constexpr CamWheelType CAM_WHEEL = CamWheelType::WHEEL_1;
    constexpr float ADC_MAX_VAL = 1023.0f;
    constexpr float ADC_MID_VAL = 511.5f;
}

namespace Physics {
    constexpr float PI_VAL               = 3.14159265358979f;
    constexpr float AIR_DENSITY_KGM3     = 1.225f;
    constexpr float ATMOSPHERIC_PRESSURE_PA = 101325.0f;
    constexpr float AIR_GAS_CONSTANT     = 287.05f;
    constexpr float INTAKE_TEMP_K        = 293.15f;

    constexpr float FLYWHEEL_MASS_KG     = 10.0f;
    constexpr float FLYWHEEL_RADIUS_M    = 0.1f;
    constexpr float FLYWHEEL_INERTIA     = FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;

    constexpr float VISCOUS_FRICTION_COEFF  = 0.002f;
    constexpr float STATIC_FRICTION_NM     = 0.05f;
    constexpr float ENGINE_BRAKE_FACTOR    = 4.0f;
    constexpr int   THROTTLE_BRAKE_THRESHOLD = 50;
}

namespace Engine {
    constexpr int   NUM_CYLINDERS   = 4;
    constexpr int   FIRING_ORDER[NUM_CYLINDERS]      = {0, 2, 3, 1};
    constexpr float FIRING_START_DEG[NUM_CYLINDERS]  = {0.0f, 180.0f, 360.0f, 540.0f};

    constexpr float BORE_M   = 0.0671f;
    constexpr float STROKE_M = 0.0706f;
    constexpr float CYLINDER_DISPLACEMENT_M3 =
        (Physics::PI_VAL / 4.0f) * BORE_M * BORE_M * STROKE_M;

    constexpr float STOICHIOMETRIC_RATIO      = 14.7f;
    constexpr float PEAK_TORQUE_PER_CYLINDER  = 50.0f;
    constexpr float MIN_FUEL_MASS_KG          = 1e-9f;

    constexpr float STARTER_TORQUE_NM             = 10.0f;
    constexpr float STARTER_SPEED_THRESHOLD_RPM   = 600.0f;

    constexpr float INJ_FLOW_RATE_KGS = 0.0002f;
}

// =============================================================
class EngineSimulator {
public:
    struct TunableConfig {
        float flywheel_inertia = Physics::FLYWHEEL_INERTIA;
        float viscous_friction = Physics::VISCOUS_FRICTION_COEFF;
        float static_friction  = Physics::STATIC_FRICTION_NM;
        float engine_brake     = Physics::ENGINE_BRAKE_FACTOR;
        float peak_torque      = Engine::PEAK_TORQUE_PER_CYLINDER;
        float inj_flow_rate    = Engine::INJ_FLOW_RATE_KGS;
    };

    // Snapshot passed to the UI thread every 50 ms
    struct Telemetry {
        float rpm        = 0.0f;
        float map_kpa    = 101.325f;
        float torque_nm  = 0.0f;
        float lambda     = 1.0f;
        float flywheel_angle = 0.0f;
        float crank_angle_720 = 0.0f;
        bool  cam_pos    = false;
        bool  crank_pos  = false;
        bool  cyl_injecting[Engine::NUM_CYLINDERS] = {};
        bool  cyl_igniting [Engine::NUM_CYLINDERS] = {};
        float cyl_lambda   [Engine::NUM_CYLINDERS] = {1,1,1,1};

        // Scope data: bitmask of INJ (bits 0-3) and IGN (bits 4-7) for each degree of 720 cycle
        uint8_t scope_buffer[720];
    };

    void setup();
    void update();
    void write_telemetry();
    void get_telemetry(Telemetry& t) const;
    void set_config(const TunableConfig& c);
    TunableConfig get_config() const;

private:
    static float dps_to_rpm(float dps) { return dps / 6.0f; }

    void measure_ecu_inputs();
    void update_dynamics();
    void update_outputs();

    struct Cylinder {
        volatile float         injected_fuel_mass   = Engine::MIN_FUEL_MASS_KG;
        volatile bool          is_injecting         = false;
        volatile unsigned long injection_start_us   = 0;
        volatile bool          ignition_fired       = false;
        volatile bool          is_igniting          = false;
        float air_mass_kg = 0.0f;
        float lambda      = 1.0f;
    };

    Cylinder cylinders[Engine::NUM_CYLINDERS];
    volatile float flywheel_angle_deg  = 0.0f;
    volatile float flywheel_speed_dps  = 0.0f;
    volatile float flywheel_torque_nm  = 0.0f;
    volatile float crank_angle_720_deg = 0.0f;
    volatile float lambda_overall      = 1.0f;
    float manifold_pressure_pa = Physics::ATMOSPHERIC_PRESSURE_PA;

    TunableConfig config;
    uint8_t scope_buffer[720] = {0};
};
