#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// =================================================================
// == CONFIGURATION & CONSTANTS
// =================================================================

namespace Pins {
  // Analog MAP input (PWM from other MCU is RC filtered to DC)
  constexpr uint8_t MAP_INPUT = A4;

  // Sensor Outputs
  constexpr uint8_t CAM_POS = 4;
  constexpr uint8_t CRANK_POS = 5;
  constexpr uint8_t LAMBDA = 11; // PWM (Timer2 on Uno)

  // ECU Inputs (A0-A3 for PCINT1)
  constexpr uint8_t INJ[] = {A0, A1, A2, A3};
  constexpr uint8_t IGN[] = {6, 7, 8, 10}; // digital inputs

  // Other Inputs
  constexpr uint8_t START_BUTTON = 12; // pullup, LOW=pressed
  constexpr uint8_t FORCE_ADC = A5;    // optional "load"
}

namespace Config {
  constexpr float TIME_STEP_S = 0.001f;        // 1 ms physics step
  constexpr float ADC_MAX_VAL = 1023.0f;
  constexpr float ADC_MID_VAL = 511.5f;
  constexpr float MAP_SMOOTH_ALPHA = 0.10f;    // EMA for MAP
  constexpr float MAP_MIN_PA = 25000.0f;       // ~25 kPa (high vacuum)
  constexpr float MAP_MAX_PA = 101325.0f;      // 1 atm
}

namespace Physics {
  constexpr float PI = 3.14159265358979323846f;
  constexpr float AIR_GAS_CONSTANT = 287.05f;  // J/(kg·K)
  constexpr float INTAKE_TEMP_K = 293.15f;     // ~20°C
  constexpr float FLYWHEEL_MASS_KG = 10.0f;
  constexpr float FLYWHEEL_RADIUS_M = 0.1f;
  constexpr float FLYWHEEL_INERTIA =
      FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;
  constexpr float VISCOUS_FRICTION_COEFF = 0.002f; // N·m·s/rad
  constexpr float STATIC_FRICTION_NM = 0.05f;      // N·m (Coulomb)
}

namespace Engine {
  constexpr int   NUM_CYLINDERS = 4;
  constexpr int   FIRING_ORDER[NUM_CYLINDERS] = {0, 2, 3, 1};
  constexpr float FIRING_START_DEG[NUM_CYLINDERS] = {0.0f, 180.0f, 360.0f, 540.0f};
  constexpr float BORE_M = 0.0671f;
  constexpr float STROKE_M = 0.0706f;
  constexpr float CYLINDER_DISPLACEMENT_M3 =
      (Physics::PI / 4.0f) * BORE_M * BORE_M * STROKE_M;
  constexpr float STOICHIOMETRIC_RATIO = 14.7f;
  constexpr float PEAK_TORQUE_PER_CYLINDER = 50.0f; // N·m peak per firing
  constexpr float MIN_FUEL_MASS_KG = 1e-9f;
  constexpr float STARTER_TORQUE_NM = 5.0f;
  constexpr float STARTER_SPEED_THRESHOLD_RPM = 500.0f;
  constexpr float INJ_FLOW_RATE_KGS = 0.0002f; // kg/s during injector on-time
}

// =================================================================
// == HELPERS
// =================================================================

static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  // Avoid integer truncation of Arduino::map()
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

// =================================================================
// == INTERRUPT-DRIVEN INJECTION TIMING (A0-A3 via PCINT1)
// =================================================================

// Shared by ISR and main loop.
volatile unsigned long g_injection_start_us[Engine::NUM_CYLINDERS];
volatile unsigned long g_last_pulse_width_us[Engine::NUM_CYLINDERS] = {0};
volatile uint8_t g_previous_inj_pin_states = 0;

/*
 * PCINT1_vect covers pins A0..A5 (PC0..PC5). We only mask A0..A3.
 * We record injector HIGH pulse widths with microsecond resolution.
 */
ISR(PCINT1_vect) {
  unsigned long now_us = micros();
  uint8_t current_inj_pin_states = PINC & 0x0F; // A0..A3 -> PC0..PC3
  uint8_t changed = current_inj_pin_states ^ g_previous_inj_pin_states;

  for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
    if (changed & (1 << i)) {
      if (current_inj_pin_states & (1 << i)) {
        // rising
        g_injection_start_us[i] = now_us;
      } else {
        // falling
        g_last_pulse_width_us[i] = now_us - g_injection_start_us[i];
      }
    }
  }
  g_previous_inj_pin_states = current_inj_pin_states;
}

// =================================================================
// == ENGINE SIMULATOR
// =================================================================

class EngineSimulator {
public:
  void setup();
  void update();
  void write_telemetry();

private:
  static inline float dps_to_rpm(float dps) { return dps / 6.0f; }

  void process_interrupt_data(); // injector pulse -> fuel mass
  void read_external_inputs();   // MAP, ignition, starter, force
  void update_dynamics();        // physics
  void update_outputs();         // cam, crank, lambda PWM

  struct Cylinder {
    float injected_fuel_mass = Engine::MIN_FUEL_MASS_KG;
    volatile bool ignition_fired = false; // latched by IGN input edge
    volatile bool is_igniting = false;    // edge detection state
    float lambda = 1.0f;
  };

  Cylinder cylinders[Engine::NUM_CYLINDERS];

  volatile float flywheel_angle_deg = 0.0f; // 0..360
  volatile float flywheel_speed_dps = 0.0f; // deg/s
  volatile float flywheel_torque_nm = 0.0f;
  volatile float crank_angle_720_deg = 0.0f; // 0..720 virtual crank
  volatile float lambda_overall = 1.0f;

  float manifold_pressure_pa = Config::MAP_MAX_PA; // Pa
  bool  starter_active = false;
};

void EngineSimulator::setup() {
  // I/O directions
  pinMode(Pins::MAP_INPUT, INPUT);
  pinMode(Pins::CAM_POS, OUTPUT);
  pinMode(Pins::CRANK_POS, OUTPUT);
  pinMode(Pins::LAMBDA, OUTPUT);
  pinMode(Pins::START_BUTTON, INPUT_PULLUP);
  pinMode(Pins::FORCE_ADC, INPUT);

  for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
    pinMode(Pins::INJ[i], INPUT);
    pinMode(Pins::IGN[i], INPUT);
  }

  // Initialize baseline state for PCINT to avoid phantom edges on enable
  noInterrupts();
  g_previous_inj_pin_states = PINC & 0x0F; // A0..A3
  // Enable PCINT1 group (A0..A5), mask A0..A3
  PCICR  |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);
  interrupts();
}

void EngineSimulator::update() {
  process_interrupt_data();
  read_external_inputs();
  update_dynamics();
  update_outputs();
}

void EngineSimulator::process_interrupt_data() {
  for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
    unsigned long us = 0;
    noInterrupts();
    us = g_last_pulse_width_us[i];
    g_last_pulse_width_us[i] = 0;
    interrupts();

    if (us > 0) {
      const float width_s = (float)us * 1e-6f;
      float m = Engine::INJ_FLOW_RATE_KGS * width_s;
      if (m < Engine::MIN_FUEL_MASS_KG) m = Engine::MIN_FUEL_MASS_KG;
      cylinders[i].injected_fuel_mass = m;
    }
  }
}

void EngineSimulator::read_external_inputs() {
  // MAP (EMA smoothing)
  static float map_pa_filt = Config::MAP_MAX_PA;
  int raw = analogRead(Pins::MAP_INPUT);
  float map_pa = mapf((float)raw, 0.0f, 1023.0f, Config::MAP_MIN_PA, Config::MAP_MAX_PA);
  map_pa_filt = (1.0f - Config::MAP_SMOOTH_ALPHA) * map_pa_filt + Config::MAP_SMOOTH_ALPHA * map_pa;
  manifold_pressure_pa = constrain(map_pa_filt, Config::MAP_MIN_PA, Config::MAP_MAX_PA);

  // Ignition edge latch (simple level-based edge detect)
  for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
    bool s = (digitalRead(Pins::IGN[i]) != 0);
    if (s && !cylinders[i].is_igniting) {
      cylinders[i].is_igniting = true;
      cylinders[i].ignition_fired = true; // will be consumed in dynamics
    } else if (!s && cylinders[i].is_igniting) {
      cylinders[i].is_igniting = false;
    }
  }

  // Starter button
  starter_active = (digitalRead(Pins::START_BUTTON) == LOW);
}

void EngineSimulator::update_dynamics() {
  // Advance virtual 720° crank based on current speed
  crank_angle_720_deg += flywheel_speed_dps * Config::TIME_STEP_S;
  // Normalize to [0,720)
  if (crank_angle_720_deg >= 720.0f) crank_angle_720_deg -= 720.0f;
  if (crank_angle_720_deg < 0.0f)    crank_angle_720_deg += 720.0f;

  float net_torque = 0.0f;
  float total_lambda = 0.0f;
  const float rpm = dps_to_rpm(flywheel_speed_dps);

  for (int i = 0; i < Engine::NUM_CYLINDERS; ++i) {
    const int   cyl = Engine::FIRING_ORDER[i];
    const float tdc = Engine::FIRING_START_DEG[i];
    float phase = crank_angle_720_deg - tdc;
    if (phase < 0.0f) phase += 720.0f;

    // Reset latch half cycle later (exhaust/intake region)
    if (phase > 180.0f && phase < 360.0f) {
      cylinders[cyl].ignition_fired = false;
    }

    // Power stroke window: 0..180° after TDC firing start
    if (cylinders[cyl].ignition_fired && phase >= 0.0f && phase <= 180.0f) {
      float ve = 0.85f;
      if (rpm > 1000.0f && rpm < 4500.0f) ve = 0.95f;
      else if (rpm >= 4500.0f) ve = 0.88f;

      const float air_mass =
          (manifold_pressure_pa * Engine::CYLINDER_DISPLACEMENT_M3 * ve) /
          (Physics::AIR_GAS_CONSTANT * Physics::INTAKE_TEMP_K);

      // Avoid div-by-zero
      const float fuel = max(cylinders[cyl].injected_fuel_mass, Engine::MIN_FUEL_MASS_KG);
      cylinders[cyl].lambda = (air_mass / fuel) / Engine::STOICHIOMETRIC_RATIO;

      // Efficiency heuristic vs lambda
      float lambda_eff =
          (cylinders[cyl].lambda < 1.0f)
            ? (cylinders[cyl].lambda - 0.4f) / 0.6f   // 0.4..1.0 -> 0..1
            : 1.0f / cylinders[cyl].lambda;           // lean -> decays
      lambda_eff = constrain(lambda_eff, 0.0f, 1.0f);

      // Crude torque shape over 0..180° (sinusoid)
      const float torque_shape = sinf(phase * (Physics::PI / 180.0f));
      net_torque += Engine::PEAK_TORQUE_PER_CYLINDER * torque_shape * lambda_eff;

      // Clear latch shortly after start so we don't double-fire
      if (phase > 10.0f) {
        cylinders[cyl].ignition_fired = false;
      }
    }

    total_lambda += cylinders[cyl].lambda;
  }

  lambda_overall = total_lambda / (float)Engine::NUM_CYLINDERS;

  // Starter torque assistance
  if (starter_active && rpm < Engine::STARTER_SPEED_THRESHOLD_RPM) {
    net_torque += Engine::STARTER_TORQUE_NM;
  }

  // External "load" via FORCE_ADC (optional pot centered ~512)
  float external_force = (analogRead(Pins::FORCE_ADC) - Config::ADC_MID_VAL) * 0.01f;
  net_torque -= external_force * Physics::FLYWHEEL_RADIUS_M;

  // Friction torques
  const float omega = flywheel_speed_dps * (Physics::PI / 180.0f); // rad/s
  const float visc = Physics::VISCOUS_FRICTION_COEFF * omega;
  const float coul = (flywheel_speed_dps > 0.0f) ? Physics::STATIC_FRICTION_NM
                                                 : (flywheel_speed_dps < 0.0f ? -Physics::STATIC_FRICTION_NM : 0.0f);
  net_torque -= (visc + coul);

  // Integrate dynamics
  flywheel_torque_nm = net_torque;
  const float alpha = net_torque / Physics::FLYWHEEL_INERTIA; // rad/s^2
  flywheel_speed_dps += (alpha * (180.0f / Physics::PI)) * Config::TIME_STEP_S;
  flywheel_angle_deg += flywheel_speed_dps * Config::TIME_STEP_S;

  // Normalize 0..360 for physical angle
  if (flywheel_angle_deg >= 360.0f) flywheel_angle_deg -= 360.0f;
  if (flywheel_angle_deg < 0.0f)    flywheel_angle_deg += 360.0f;
}

void EngineSimulator::update_outputs() {
  // Lambda PWM: clamp 0..2 mapped to 0..255 (saturate outside)
  float lam = lambda_overall;
  if (lam < 0.0f) lam = 0.0f;
  if (lam > 2.0f) lam = 2.0f;
  int pwm = (int)(lam * 255.0f / 2.0f + 0.5f);
  pwm = constrain(pwm, 0, 255);
  analogWrite(Pins::LAMBDA, pwm);

  // CAM: simple 50% duty, half crank speed (360 per 720)
  float cam_deg = fmodf(flywheel_angle_deg / 2.0f, 360.0f);
  bool cam_state = (fmodf(cam_deg, 180.0f) < 90.0f);
  digitalWrite(Pins::CAM_POS, cam_state ? HIGH : LOW);

  // CRANK: 60-2 wheel approximation (missing last 2 teeth each rev)
  // 6° per tooth, toggle HIGH for first 3° of each tooth, skip teeth 58,59
  int   tooth = (int)floorf(flywheel_angle_deg / 6.0f); // 0..59
  bool  tooth_present = (tooth < 58);                   // 58 & 59 missing
  bool  within_high = (fmodf(flywheel_angle_deg, 6.0f) < 3.0f);
  bool  crank_state = tooth_present && within_high;
  digitalWrite(Pins::CRANK_POS, crank_state ? HIGH : LOW);
}

void EngineSimulator::write_telemetry() {
  static unsigned long last_ms = 0;
  unsigned long now = millis();
  if (now - last_ms >= 250) {
    last_ms = now;
    Serial.print(F("RPM: "));    Serial.print(dps_to_rpm(flywheel_speed_dps), 0);
    Serial.print(F(" | MAP: ")); Serial.print(manifold_pressure_pa * 0.001f, 1); Serial.print(F(" kPa"));
    Serial.print(F(" | Torque: ")); Serial.print(flywheel_torque_nm, 2); Serial.print(F(" N·m"));
    Serial.print(F(" | Lambda: ")); Serial.println(lambda_overall, 2);
  }
}

// =================================================================
// == ARDUINO SKETCH
// =================================================================

EngineSimulator engine;
volatile bool run_simulation_tick = false;

void timer_isr() {
  run_simulation_tick = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { } // for Leonardo; harmless on Uno

  Serial.println(F("Engine Simulator (Interrupt-driven) Initializing..."));

  engine.setup();

  // 1 ms tick using Timer1 (Uno pins 9/10 PWM will be affected; we don't use PWM on them)
  Timer1.initialize((unsigned long)(Config::TIME_STEP_S * 1e6f));
  Timer1.attachInterrupt(timer_isr);

  Serial.println(F("Simulator running."));
}

void loop() {
  if (run_simulation_tick) {
    run_simulation_tick = false;
    engine.update();
  }
  engine.write_telemetry();
}
