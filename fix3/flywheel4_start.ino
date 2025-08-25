#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =================================================================
// == CONFIGURATION & CONSTANTS
// =================================================================

// Use namespaces to organize constants and avoid global name clashes.
namespace Pins {
    constexpr uint8_t CAM_POS = 2;
    constexpr uint8_t INJ[] = {3, 4, 5, 6};
    constexpr uint8_t IGN[] = {7, 8, 9, 10};
    constexpr uint8_t LAMBDA = 11;
    constexpr uint8_t START_BUTTON = 12; // external start button (active LOW)
    constexpr uint8_t THROTTLE_ADC = A0;
    constexpr uint8_t FORCE_ADC = A1;
}

namespace Config {
    // Defines the timing of the main simulation loop.
    constexpr float TIME_STEP_S = 0.001f; // 1 ms update interval

    // Cam wheel selection
    enum class CamWheelType { WHEEL_1, WHEEL_2, WHEEL_3 };
    constexpr CamWheelType CAM_WHEEL = CamWheelType::WHEEL_1;
    
    // ADC resolution
    constexpr float ADC_MAX_VAL = 1023.0f;
    constexpr float ADC_MID_VAL = 511.5f; // Center for signed inputs
}

namespace Physics {
    // Physical constants
    constexpr float PI = 3.14159265358979323846f;
    constexpr float AIR_DENSITY_KGM3 = 1.225f;   // kg/m^3
    constexpr float FUEL_DENSITY_KGM3 = 720.0f; // kg/m^3

    // Flywheel properties
    constexpr float FLYWHEEL_MASS_KG = 10.0f;
    constexpr float FLYWHEEL_RADIUS_M = 0.1f;
    constexpr float FLYWHEEL_INERTIA = FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M; // kg·m^2

    // Friction model
    constexpr float VISCOUS_FRICTION_COEFF = 0.002f; // Nm per rad/s
    constexpr float STATIC_FRICTION_NM = 0.05f;      // Coulomb friction
    constexpr float ENGINE_BRAKE_FACTOR = 4.0f;      // Multiplier when throttle is closed
    constexpr int THROTTLE_BRAKE_THRESHOLD = 50;     // ADC units
}

namespace Engine {
    constexpr int NUM_CYLINDERS = 4;
    // Firing order for a typical inline-4 engine: 1-3-4-2
    constexpr int FIRING_ORDER[NUM_CYLINDERS] = {0, 2, 3, 1}; // 0-indexed cylinders
    // Firing events happen every 180 degrees of crank rotation in a 4-cyl engine.
    constexpr float FIRING_START_DEG[NUM_CYLINDERS] = {0.0f, 180.0f, 360.0f, 540.0f};

    // Engine geometry
    constexpr float BORE_M = 0.0671f;
    constexpr float STROKE_M = 0.0706f;
    constexpr float CYLINDER_AREA_M2 = Physics::PI * (BORE_M / 2.0f) * (BORE_M / 2.0f);
    constexpr float THROTTLE_DIAMETER_M = 0.05f;
    constexpr float THROTTLE_AREA_M2 = Physics::PI * (THROTTLE_DIAMETER_M / 2.0f) * (THROTTLE_DIAMETER_M / 2.0f);
    
    // Combustion model
    constexpr float STOICHIOMETRIC_RATIO = 14.7f;
    constexpr float COMBUSTION_PRESSURE_SCALE = 1e5f;
    constexpr float MAX_COMBUSTION_PRESSURE_PA = 5.0e6f; // Pascal, clamp to avoid huge values
    constexpr float MIN_FUEL_MASS_KG = 1e-9f; // Minimum fuel to avoid division by zero

    // Starter motor
    constexpr float STARTER_TORQUE_NM = 5.0f;
    constexpr float STARTER_SPEED_THRESHOLD_DPS = 600.0f; // deg/s

    // ECU Inputs
    constexpr float INJ_FLOW_RATE_KGS = 0.0002f; // Assumed kg per second
}

// =================================================================
// == DATA STRUCTURES
// =================================================================

/**
 * @brief Holds the state for a single engine cylinder.
 */
struct Cylinder {
    // NOTE: 'volatile' is used for variables accessed in both main loop and ISRs.
    // While simulation logic is now in the main loop, pulse measurement is not.
    volatile float injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
    volatile bool is_injecting = false;
    volatile unsigned long injection_start_us = 0;

    // These variables are only updated in the main simulation thread,
    // so they don't strictly need to be volatile.
    float air_mass_kg = 0.0f;
    float lambda = 1.0f;
};

// =================================================================
// == GLOBAL STATE VARIABLES
// =================================================================
// These variables represent the core state of the engine simulation.
// They are marked 'volatile' because they are modified in the main loop
// but can be read asynchronously for telemetry.
volatile float flywheel_angle_deg = 0.0f; // Current angle [0, 360)
volatile float flywheel_speed_dps = 0.0f; // Current speed in degrees/sec
volatile float flywheel_torque_nm = 0.0f; // Last calculated net torque
volatile float crank_angle_720_deg = 0.0f; // Tracks the 4-stroke cycle [0, 720)
volatile float lambda_overall = 1.0f;

// Create an array of Cylinder objects
Cylinder cylinders[Engine::NUM_CYLINDERS];

// This flag is set by the timer ISR to signal the main loop to run a simulation tick.
volatile bool run_simulation_tick = false;

// =================================================================
// == HELPER FUNCTIONS
// =================================================================

static inline float deg_to_rad(float deg) {
    return deg * (Physics::PI / 180.0f);
}

static inline float rad_to_deg(float rad) {
    return rad * (180.0f / Physics::PI);
}

template<typename T>
T clamp(T val, T min_val, T max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// =================================================================
// == INTERRUPT SERVICE ROUTINES (ISRs)
// =================================================================

/**
 * @brief Timer ISR. Kept extremely short and fast.
 * Its only job is to set a flag to trigger the simulation update in the main loop.
 */
void timer_isr() {
    run_simulation_tick = true;
}

/**
 * @brief Measures the pulse width of injector signals.
 * This is called from the main loop but captures timings that could be missed.
 * For higher precision, this could be tied to pin-change interrupts instead of polling.
 */
void measure_injector_pulses() {
    unsigned long now_us = micros();

    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        bool current_state = digitalRead(Pins::INJ[i]);
        
        // Rising edge: injector opens
        if (current_state && !cylinders[i].is_injecting) {
            cylinders[i].injection_start_us = now_us;
            cylinders[i].is_injecting = true;
        }
        // Falling edge: injector closes, calculate fuel injected
        else if (!current_state && cylinders[i].is_injecting) {
            unsigned long width_us = now_us - cylinders[i].injection_start_us;
            float width_s = width_us / 1e6f;
            cylinders[i].injected_fuel_mass = Engine::INJ_FLOW_RATE_KGS * width_s;
            // Prevent fuel from being exactly zero to avoid divide-by-zero errors.
            if (cylinders[i].injected_fuel_mass < Engine::MIN_FUEL_MASS_KG) {
                cylinders[i].injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
            }
            cylinders[i].is_injecting = false;
        }
    }
    // Note: A similar function for ignition timing could be added here if needed.
}


// =================================================================
// == SIMULATION LOGIC
// =================================================================

/**
 * @brief Updates the engine physics and state for one time step.
 */
void update_simulation() {
    // 1. Read Inputs
    int throttle_adc = analogRead(Pins::THROTTLE_ADC);
    int force_adc = analogRead(Pins::FORCE_ADC);

    float throttle_frac = throttle_adc / Config::ADC_MAX_VAL;
    // External force is signed; 512 is center. A real sensor would need calibration.
    float external_force = (force_adc - Config::ADC_MID_VAL) * 0.01f; 

    float net_torque = 0.0f;

    // 2. Calculate Torque from Combustion
    // Update the 720-degree crank angle
    crank_angle_720_deg += flywheel_speed_dps * Config::TIME_STEP_S;
    crank_angle_720_deg = fmod(crank_angle_720_deg, 720.0f);
    if (crank_angle_720_deg < 0.0f) {
        crank_angle_720_deg += 720.0f;
    }

    float total_lambda = 0.0f;
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        int cyl_idx = Engine::FIRING_ORDER[i];
        float start_angle = Engine::FIRING_START_DEG[i];
        float phase_deg = crank_angle_720_deg - start_angle;

        // Check if this cylinder is in its power stroke (0-180 degrees after its firing event)
        if (phase_deg >= 0.0f && phase_deg <= 180.0f) {
            // NOTE: This is a highly simplified air mass model. It doesn't account for
            // volumetric efficiency, manifold pressure, or air velocity.
            cylinders[cyl_idx].air_mass_kg = Engine::THROTTLE_AREA_M2 * throttle_frac * Physics::AIR_DENSITY_KGM3 * Engine::STROKE_M;
            
            float fuel_mass = cylinders[cyl_idx].injected_fuel_mass;
            cylinders[cyl_idx].lambda = (cylinders[cyl_idx].air_mass_kg / fuel_mass) / Engine::STOICHIOMETRIC_RATIO;
            cylinders[cyl_idx].lambda = clamp(cylinders[cyl_idx].lambda, 0.5f, 2.0f);

            // Combustion pressure model. Pressure is highest near stoich (lambda=1).
            // This model is a simple approximation. A real map would be more complex.
            float efficiency_factor = (cylinders[cyl_idx].lambda < 1.0f) ? cylinders[cyl_idx].lambda : (1.0f / cylinders[cyl_idx].lambda);
            float burn_frac = clamp(phase_deg / 180.0f, 0.0f, 1.0f); // Simple linear burn
            float pressure_pa = efficiency_factor * Engine::COMBUSTION_PRESSURE_SCALE * burn_frac;
            pressure_pa = clamp(pressure_pa, 0.0f, Engine::MAX_COMBUSTION_PRESSURE_PA);
            
            float combustion_force = pressure_pa * Engine::CYLINDER_AREA_M2;
            net_torque += combustion_force * Physics::FLYWHEEL_RADIUS_M; // Simplified torque calculation
        }
        total_lambda += cylinders[cyl_idx].lambda;
    }
    lambda_overall = total_lambda / Engine::NUM_CYLINDERS;

    // 3. Add Other Torques (Starter, External Load, Friction)
    // Starter motor torque
    if (digitalRead(Pins::START_BUTTON) == LOW && fabsf(flywheel_speed_dps) < Engine::STARTER_SPEED_THRESHOLD_DPS) {
        net_torque += Engine::STARTER_TORQUE_NM;
    }
    // External load
    net_torque -= external_force * Physics::FLYWHEEL_RADIUS_M;

    // Friction Torque
    float omega_rad_s = deg_to_rad(flywheel_speed_dps);
    float viscous_friction = Physics::VISCOUS_FRICTION_COEFF * omega_rad_s;
    if (throttle_adc < Physics::THROTTLE_BRAKE_THRESHOLD) {
        viscous_friction *= Physics::ENGINE_BRAKE_FACTOR;
    }

    float coulomb_friction = 0.0f;
    if (fabsf(flywheel_speed_dps) > 1e-3f) { // Moving
        coulomb_friction = (flywheel_speed_dps > 0.0f) ? Physics::STATIC_FRICTION_NM : -Physics::STATIC_FRICTION_NM;
    } else { // Static
        if (fabsf(net_torque) < Physics::STATIC_FRICTION_NM) {
            net_torque = 0.0f; // Not enough torque to overcome stiction
        } else {
            coulomb_friction = (net_torque > 0) ? -Physics::STATIC_FRICTION_NM : Physics::STATIC_FRICTION_NM;
        }
    }
    net_torque -= (viscous_friction + coulomb_friction);
    flywheel_torque_nm = net_torque;

    // 4. Integrate Dynamics (Euler method)
    float angular_accel_rad_s2 = net_torque / Physics::FLYWHEEL_INERTIA;
    float angular_accel_deg_s2 = rad_to_deg(angular_accel_rad_s2);

    flywheel_speed_dps += angular_accel_deg_s2 * Config::TIME_STEP_S;
    flywheel_angle_deg += flywheel_speed_dps * Config::TIME_STEP_S;

    // Normalize angle to [0, 360)
    flywheel_angle_deg = fmod(flywheel_angle_deg, 360.0f);
    if (flywheel_angle_deg < 0.0f) {
        flywheel_angle_deg += 360.0f;
    }

    // 5. Update Outputs
    // Lambda sensor output (PWM)
    int lambda_pwm = map(clamp(lambda_overall, 0.0f, 2.0f), 0.0f, 2.0f, 0, 255);
    analogWrite(Pins::LAMBDA, lambda_pwm);

    // Cam sensor output
    float cam_angle_deg = fmod(flywheel_angle_deg / 2.0f, 360.0f);
    bool cam_pos_state = false;
    switch (Config::CAM_WHEEL) {
        case Config::CamWheelType::WHEEL_1: cam_pos_state = (fmod(cam_angle_deg, 180.0f) < 90.0f); break;
        case Config::CamWheelType::WHEEL_2: cam_pos_state = (fmod(cam_angle_deg, 90.0f) < 45.0f); break;
        case Config::CamWheelType::WHEEL_3: cam_pos_state = (fmod(cam_angle_deg, 60.0f) < 30.0f); break;
    }
    digitalWrite(Pins::CAM_POS, cam_pos_state);
}

/**
 * @brief Prints telemetry data to the Serial monitor.
 */
void print_telemetry() {
    static unsigned long last_print_ms = 0;
    if (millis() - last_print_ms > 250) {
        last_print_ms = millis();
        
        // Safely copy volatile data to local variables for printing
        noInterrupts();
        float angle = flywheel_angle_deg;
        float speed = flywheel_speed_dps;
        float torque = flywheel_torque_nm;
        float lam = lambda_overall;
        float crank720 = crank_angle_720_deg;
        interrupts();

        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "Angle:%.2f | Speed:%.2f dps | Torque:%.3f Nm | Lambda:%.3f | Crank720:%.2f",
                 angle, speed, torque, lam, crank720);
        Serial.println(buffer);
    }
}


// =================================================================
// == ARDUINO SETUP & LOOP
// =================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("Engine Simulator Initializing...");

    // Configure pin modes
    pinMode(Pins::CAM_POS, OUTPUT);
    pinMode(Pins::LAMBDA, OUTPUT);
    pinMode(Pins::START_BUTTON, INPUT_PULLUP);
    
    for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
        pinMode(Pins::INJ[i], INPUT);
        pinMode(Pins::IGN[i], INPUT);
    }
    // ADC pins are INPUT by default, no need to set pinMode.

    // Initialize Timer1 for the simulation tick
    unsigned long period_us = (unsigned long)(Config::TIME_STEP_S * 1e6f);
    Timer1.initialize(period_us);
    Timer1.attachInterrupt(timer_isr); // Attach the lightweight ISR

    Serial.println("Engine simulator started.");
}

void loop() {
    // Check if the timer ISR has requested a simulation update.
    if (run_simulation_tick) {
        // It's important to reset the flag immediately to not miss a tick.
        run_simulation_tick = false;

        // Run the main simulation logic
        measure_injector_pulses();
        update_simulation();
    }
    
    // Telemetry and other non-critical tasks can run here.
    print_telemetry();
    
    // A small delay can be added to yield time if other low-priority tasks are needed,
    // but the loop should be free to run as fast as possible to catch the tick flag.
    // delay(1); 
}
