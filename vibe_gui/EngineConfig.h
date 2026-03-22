#ifndef ENGINE_CONFIG_H
#define ENGINE_CONFIG_H

#include "mock_arduino.h"

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

    constexpr float FLYWHEEL_MASS_KG = 1.0f;
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

    constexpr float INJ_FLOW_RATE_KGS = 0.0010f;
    constexpr float STARTER_TORQUE_NM = 150.0f;
    constexpr float STARTER_SPEED_THRESHOLD_RPM = 800.0f;

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

    constexpr float BASE_TORQUE_SCALE = 1.0f;
    constexpr float INTAKE_PUMP_SCALE = 0.06f;
    constexpr float EXHAUST_PUMP_SCALE = 0.06f;
    constexpr float COMPRESSION_PUMP_SCALE = 0.18f;
    constexpr float COMBUSTION_EFFICIENCY = 0.92f;
}

#endif
