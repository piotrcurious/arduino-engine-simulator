#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =================================================================
// == CONFIGURATION & CONSTANTS
// =================================================================

namespace Pins {
    constexpr uint8_t CAM_POS = 2;
    constexpr uint8_t CRANK_POS = 3;
    constexpr uint8_t INJ[] = {4, 5, 6, 7};
    constexpr uint8_t IGN[] = {8, 9, 10, 11};
    // Note: Pin 12 is NOT a PWM pin on standard AVR (Uno/Nano). 
    // If using an Uno, swap this to Pin 5 or 6 for true analog output.
    constexpr uint8_t LAMBDA = 12; 
    constexpr uint8_t START_BUTTON = 13;
    constexpr uint8_t THROTTLE_ADC = A0;
    constexpr uint8_t FORCE_ADC = A1;
}

namespace Config {
    // Lowered to 200us (5kHz) to better resolve the 60-2 trigger wheel at high RPMs
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
    constexpr float COMPRESSION_TORQUE_LOSS = 15.0f; // Torque lost to compressing air
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
    float dps_to_rpm(float dps) { return dps * (60.0f / 360.0f); }

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
    int current_force_adc = Config::ADC_MID_VAL;
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
}

void EngineSimulator::update(float dt) {
    measure_ecu_inputs();
    update_dynamics(dt);
    update_outputs();
}

void EngineSimulator::measure_ecu_inputs() {
    unsigned long now_us = micros();
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        bool inj_state = digitalRead(Pins::INJ[i]);
        if (inj_state && !cylinders[i].is_injecting) {
            cylinders[i].is_injecting = true;
            cylinders[i].injection_start_us = now_us;
        } else if (!inj_state && cylinders[i].is_injecting) {
            cylinders[i].is_injecting = false;
            float width_s = (now_us - cylinders[i].injection_start_us) / 1000000.0f;
            cylinders[i].injected_fuel_mass = max(Engine::INJ_FLOW_RATE_KGS * width_s, Engine::MIN_FUEL_MASS_KG);
        }

        bool ign_state = digitalRead(Pins::IGN[i]);
        if (ign_state && !cylinders[i].is_igniting) {
            cylinders[i].is_igniting = true;
            cylinders[i].ignition_fired = true;
        } else if (!ign_state && cylinders[i].is_igniting) {
            cylinders[i].is_igniting = false;
        }
    }
}

void EngineSimulator::update_dynamics(float dt) {
    // Alternate ADC reads to save loop time (~104us per read)
    if (toggle_adc_read) {
        current_throttle_adc = analogRead(Pins::THROTTLE_ADC);
    } else {
        current_force_adc = analogRead(Pins::FORCE_ADC);
    }
    toggle_adc_read = !toggle_adc_read;

    float throttle_frac = current_throttle_adc / Config::ADC_MAX_VAL;
    float engine_rpm = dps_to_rpm(flywheel_speed_dps);

    // MAP calculation
    float target_map = Physics::ATMOSPHERIC_PRESSURE_PA * (throttle_frac + (1.0f - throttle_frac) * 0.3f);
    target_map -= (engine_rpm / 8000.0f) * 70000.0f * (1.0f - throttle_frac);
    // Adjust smoothing factor based on variable dt
    float alpha = constrain(dt * 50.0f, 0.0f, 1.0f); 
    manifold_pressure_pa += alpha * (constrain(target_map, 20000.0f, Physics::ATMOSPHERIC_PRESSURE_PA) - manifold_pressure_pa);

    crank_angle_720_deg = fmod(crank_angle_720_deg + flywheel_speed_dps * dt, 720.0f);
    if (crank_angle_720_deg < 0.0f) crank_angle_720_deg += 720.0f;

    float net_torque = 0.0f;
    float total_lambda = 0.0f;

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        int cyl_idx = Engine::FIRING_ORDER[i];
        float phase_deg = fmod(crank_angle_720_deg - Engine::FIRING_START_DEG[i] + 720.0f, 720.0f);

        // Reset ignition tracking outside power stroke
        if (phase_deg > 180.0f && phase_deg < 360.0f) {
            cylinders[cyl_idx].ignition_fired = false;
        }

        // Power Stroke Logic
        if (cylinders[cyl_idx].ignition_fired && phase_deg >= 0.0f && phase_deg <= 180.0f) {
            float ve = (engine_rpm > 1000 && engine_rpm < 4500) ? 0.95f : ((engine_rpm >= 4500) ? 0.88f : 0.85f);

            cylinders[cyl_idx].air_mass_kg = (manifold_pressure_pa * Engine::CYLINDER_DISPLACEMENT_M3 * ve) / (Physics::AIR_GAS_CONSTANT * Physics::INTAKE_TEMP_K);
            float fuel_mass = cylinders[cyl_idx].injected_fuel_mass;

            // Standalone auto-fuel logic
            if (fuel_mass <= Engine::MIN_FUEL_MASS_KG && throttle_frac > 0.05f) {
                 fuel_mass = cylinders[cyl_idx].air_mass_kg / Engine::STOICHIOMETRIC_RATIO;
            }

            cylinders[cyl_idx].lambda = (cylinders[cyl_idx].air_mass_kg / fuel_mass) / Engine::STOICHIOMETRIC_RATIO;
            
            float lambda_efficiency = (cylinders[cyl_idx].lambda < 1.0f) ? (cylinders[cyl_idx].lambda - 0.4f) / 0.6f : 1.0f / cylinders[cyl_idx].lambda;
            lambda_efficiency = constrain(lambda_efficiency, 0.0f, 1.0f);

            // Power generation
            net_torque += Engine::PEAK_TORQUE_PER_CYLINDER * sin(phase_deg * (Physics::PI_VAL / 180.0f)) * lambda_efficiency;
        }
        
        // Compression Stroke Logic (Draws torque from the system)
        if (phase_deg > 540.0f && phase_deg <= 720.0f) {
            net_torque -= Engine::COMPRESSION_TORQUE_LOSS * sin((phase_deg - 540.0f) * (Physics::PI_VAL / 180.0f));
        }

        // Standalone auto-ignition logic
        if (!cylinders[cyl_idx].ignition_fired && phase_deg < 10.0f && engine_rpm > 200) {
            cylinders[cyl_idx].ignition_fired = true;
        }

        total_lambda += cylinders[cyl_idx].lambda;
    }
    
    lambda_overall = total_lambda / Engine::NUM_CYLINDERS;

    // Starter motor logic
    if (digitalRead(Pins::START_BUTTON) == LOW && engine_rpm < Engine::STARTER_SPEED_THRESHOLD_RPM) {
        net_torque += Engine::STARTER_TORQUE_NM;
    }

    // External Load
    float external_force = (current_force_adc - Config::ADC_MID_VAL) * 0.05f;
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

    // Kinematics Integration: α = τ / I
    float angular_accel_rad_s2 = net_torque / Physics::FLYWHEEL_INERTIA;
    flywheel_speed_dps += angular_accel_rad_s2 * (180.0f / Physics::PI_VAL) * dt;
    if (flywheel_speed_dps < 0.0f) flywheel_speed_dps = 0.0f; // Prevent reverse rotation for standard engines

    flywheel_angle_deg = fmod(flywheel_angle_deg + flywheel_speed_dps * dt, 360.0f);
    if (flywheel_angle_deg < 0.0f) flywheel_angle_deg += 360.0f;
}

void EngineSimulator::update_outputs() {
    int lambda_pwm = map(constrain(lambda_overall * 100, 50, 150), 50, 150, 0, 255);
    analogWrite(Pins::LAMBDA, lambda_pwm);

    // Fixed Cam Math
    float cam_angle_deg = crank_angle_720_deg / 2.0f;
    bool cam_pos_state = (fmod(cam_angle_deg, 180.0f) < 90.0f);
    digitalWrite(Pins::CAM_POS, cam_pos_state);

    int tooth = floor(flywheel_angle_deg / 6.0f);
    bool crank_state = true;
    if (tooth >= 58) {
        crank_state = false; // Missing teeth gap
    } else {
        crank_state = (fmod(flywheel_angle_deg, 6.0f) < 3.0f);
    }
    digitalWrite(Pins::CRANK_POS, crank_state);
}

void EngineSimulator::write_telemetry() {
    static unsigned long last_print_ms = 0;
    if (millis() - last_print_ms > 100) { // Slightly faster telemetry refresh
        last_print_ms = millis();
        Serial.print("RPM:"); Serial.print(dps_to_rpm(flywheel_speed_dps), 0);
        Serial.print(" MAP_kPa:"); Serial.print(manifold_pressure_pa / 1000.0f, 1);
        Serial.print(" TQ_Nm:"); Serial.print(flywheel_torque_nm, 1);
        Serial.print(" LMD:"); Serial.println(lambda_overall, 2);
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

    unsigned long period_us = (unsigned long)(Config::TARGET_DT_S * 1e6f);
    Timer1.initialize(period_us);
    Timer1.attachInterrupt(timer_isr);

    last_update_us = micros();
    Serial.println("Simulator online.");
}

void loop() {
    if (run_simulation_tick) {
        run_simulation_tick = false;
        
        // Calculate true dt to keep physics absolutely consistent despite jitter
        unsigned long now = micros();
        float dt = (now - last_update_us) / 1000000.0f;
        last_update_us = now;
        
        // Safety clamp in case system hangs
        if (dt > 0.05f) dt = 0.05f; 

        engine.update(dt);
    }
    engine.write_telemetry();
}
