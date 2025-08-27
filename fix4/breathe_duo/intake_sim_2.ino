#include <Arduino.h>

/*
  Air Intake & Tuning Module (improved)
  - Non-blocking RPM measurement using interrupt (no pulseIn)
  - Configurable cam->crank mapping (periods per crank rev)
  - Time-constant based low-pass filters (rpm, throttle, MAP)
  - Proper float mapping (no integer map() surprises)
  - Glitch rejection on cam input + robust timeout to zero RPM
  - Safer analog scaling, clamps, and telemetry
*/

namespace Pins {
  // Communication
  constexpr uint8_t CAM_INPUT  = 2;  // INT0 on many AVR boards
  constexpr uint8_t MAP_OUTPUT = 3;  // PWM capable on many boards

  // Tuning Knobs
  constexpr uint8_t THROTTLE_POT   = A0;
  constexpr uint8_t VE_LOW_RPM_POT = A1;
  constexpr uint8_t VE_HIGH_RPM_POT= A2;
}

namespace Config {
  // --- Engine/Signal assumptions ---
  // One full CAM waveform period corresponds to this many crank revolutions.
  // In your original comment: one CAM period == 1 crank rev -> set to 1.0f.
  // Adjust if your Engine Sim changes tooth count / cam modeling.
  constexpr float CAM_PERIOD_TO_CRANK_REVS = 1.0f;

  // RPM smoothing (ms). Larger -> smoother but slower response.
  constexpr float RPM_TC_MS = 60.0f;

  // Throttle smoothing (ms).
  constexpr float THROTTLE_TC_MS = 30.0f;

  // MAP exponential smoothing factor (0..1). Keep simple here.
  constexpr float MAP_ALPHA = 0.10f;

  // VE breakpoints (rpm)
  constexpr float VE_LOW_RPM_POINT  = 1000.0f;
  constexpr float VE_HIGH_RPM_POINT = 5000.0f;

  // Pressure model (Pa)
  constexpr float ATMOSPHERIC_PRESSURE_PA = 101325.0f;
  constexpr float MIN_MAP_PA              = 25000.0f;

  // RPM measurement safety
  constexpr uint32_t CAM_TIMEOUT_MS   = 220;   // ~300 rpm min if 1 period = 1 crank rev
  constexpr uint32_t GLITCH_REJECT_US = 100;   // ignore edges closer than this (noise)

  // ADC scale (typical Arduino: 10-bit). If you use another board, update this.
  constexpr float ADC_MAX = 1023.0f;

  // PWM resolution target (most Arduinos are 8-bit analogWrite 0..255)
  constexpr int PWM_MAX = 255;
}

// ------------------------ Helpers ------------------------

template<typename T>
inline T clampT(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

// Float map with clamped input
inline float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  t = clampT(t, 0.0f, 1.0f);
  return lerp(out_min, out_max, t);
}

// One-pole LPF using time constant (ms). alpha = dt/(tc+dt)
inline float lpf_step(float current, float target, float dt_ms, float tc_ms) {
  float alpha = (tc_ms <= 0.0f) ? 1.0f : (dt_ms / (tc_ms + dt_ms));
  return current + alpha * (target - current);
}

// ------------------------ Global state ------------------------

volatile uint32_t g_lastEdgeUs = 0;
volatile uint32_t g_periodUs   = 0;   // last valid period (us)
volatile bool     g_hasPeriod   = false;

float current_rpm         = 0.0f;
float throttle_fraction   = 0.0f;
float volumetric_efficiency = 0.85f;
float manifold_pressure_pa  = Config::ATMOSPHERIC_PRESSURE_PA;

uint32_t g_lastCalcMs = 0;

// ------------------------ ISR ------------------------

void cam_rise_isr() {
  const uint32_t now = micros();
  static uint32_t prev = 0;

  // Compute period between consecutive rising edges
  uint32_t p = now - prev;
  prev = now;

  // Reject obvious glitches
  if (p >= Config::GLITCH_REJECT_US) {
    g_periodUs = p;
    g_lastEdgeUs = now;
    g_hasPeriod = true;
  }
}

// ------------------------ Forward decls ------------------------

void update_rpm();
void read_inputs();
void calculate_ve();
void calculate_map();
void output_map_pwm();
void print_telemetry();

// ------------------------ Setup/Loop ------------------------

void setup() {
  Serial.begin(115200);
  Serial.println(F("Air Intake & Tuning Module - Booting..."));

  pinMode(Pins::CAM_INPUT, INPUT);     // use INPUT or INPUT_PULLUP depending on your sensor wiring
  attachInterrupt(digitalPinToInterrupt(Pins::CAM_INPUT), cam_rise_isr, RISING);

  pinMode(Pins::MAP_OUTPUT, OUTPUT);

  g_lastCalcMs = millis();
}

void loop() {
  // dt for time-constant based filters
  const uint32_t nowMs = millis();
  const float dt_ms = (float)(nowMs - g_lastCalcMs);
  g_lastCalcMs = nowMs;

  read_inputs();        // throttle, VE knobs (smoothed)
  update_rpm();         // non-blocking RPM inference
  calculate_ve();       // uses updated RPM + knobs
  calculate_map();      // pressure model + smoothing
  output_map_pwm();     // PWM proportional to MAP
  print_telemetry();

  // A short delay is fine; all timing is event/ISR based.
  delay(5);
}

// ------------------------ Implementations ------------------------

void update_rpm() {
  // Copy volatile safely
  noInterrupts();
  bool   has = g_hasPeriod;
  uint32_t periodUs = g_periodUs;
  uint32_t lastEdgeUs = g_lastEdgeUs;
  interrupts();

  const uint32_t nowUs = micros();

  // Timeout: if no edge recently, declare 0 RPM
  const uint32_t ageMs = (nowUs >= lastEdgeUs) ? (uint32_t)((nowUs - lastEdgeUs) / 1000UL) : 0UL;
  if (ageMs > Config::CAM_TIMEOUT_MS) {
    current_rpm = 0.0f;
    return;
  }

  // If we have a new/any valid period, compute instantaneous RPM and low-pass it
  if (has && periodUs > 0) {
    // RPM = (60e6 / period_us) * (crank revs per CAM period)
    const float inst_rpm = (60000000.0f / (float)periodUs) * Config::CAM_PERIOD_TO_CRANK_REVS;

    // Smooth to reduce jitter
    const float dt_ms = (float)(millis() - g_lastCalcMs); // note: g_lastCalcMs already updated in loop()
    // Use a tiny positive dt to avoid alpha=0 if called very quickly
    const float applied_dt = dt_ms > 0.5f ? dt_ms : 5.0f;
    current_rpm = lpf_step(current_rpm, inst_rpm, applied_dt, Config::RPM_TC_MS);
  }
}

void read_inputs() {
  // Throttle (0..1), smoothed
  static float thr_s = 0.0f;
  const int raw_thr = analogRead(Pins::THROTTLE_POT);
  const float thr = clampT((float)raw_thr / Config::ADC_MAX, 0.0f, 1.0f);

  const float dt_ms = (float)(millis() - g_lastCalcMs); // small but ok for alpha
  const float applied_dt = dt_ms > 0.5f ? dt_ms : 5.0f;
  thr_s = lpf_step(thr_s, thr, applied_dt, Config::THROTTLE_TC_MS);
  throttle_fraction = thr_s;
}

void calculate_ve() {
  // Read VE knobs as 0.70 .. 1.00 (70%..100%)
  const int raw_low  = analogRead(Pins::VE_LOW_RPM_POT);
  const int raw_high = analogRead(Pins::VE_HIGH_RPM_POT);

  const float ve_at_low_rpm  = fmap((float)raw_low,  0.0f, Config::ADC_MAX, 0.70f, 1.00f);
  const float ve_at_high_rpm = fmap((float)raw_high, 0.0f, Config::ADC_MAX, 0.70f, 1.00f);

  if (current_rpm <= Config::VE_LOW_RPM_POINT) {
    volumetric_efficiency = ve_at_low_rpm;
  } else if (current_rpm >= Config::VE_HIGH_RPM_POINT) {
    volumetric_efficiency = ve_at_high_rpm;
  } else {
    const float rpm_fraction =
        (current_rpm - Config::VE_LOW_RPM_POINT) /
        (Config::VE_HIGH_RPM_POINT - Config::VE_LOW_RPM_POINT);
    volumetric_efficiency = lerp(ve_at_low_rpm, ve_at_high_rpm, clampT(rpm_fraction, 0.0f, 1.0f));
  }
}

void calculate_map() {
  // Simple phenomenological model with VE & throttle
  // base_map rises with throttle; vacuum_effect grows with RPM & closed throttle
  const float base_map =
      Config::ATMOSPHERIC_PRESSURE_PA *
      (throttle_fraction + (1.0f - throttle_fraction) * 0.3f);

  const float vacuum_effect =
      (current_rpm / 8000.0f) * 70000.0f * (1.0f - throttle_fraction);

  const float target_map =
      clampT(base_map - vacuum_effect, Config::MIN_MAP_PA, Config::ATMOSPHERIC_PRESSURE_PA);

  // Blend (EMA)
  manifold_pressure_pa =
      (1.0f - Config::MAP_ALPHA) * manifold_pressure_pa +
      Config::MAP_ALPHA * target_map;
}

void output_map_pwm() {
  // Convert pressure to 0..1, then to PWM 0..PWM_MAX
  const float norm =
      (manifold_pressure_pa - Config::MIN_MAP_PA) /
      (Config::ATMOSPHERIC_PRESSURE_PA - Config::MIN_MAP_PA);
  const int pwm_val = (int)roundf(clampT(norm, 0.0f, 1.0f) * Config::PWM_MAX);

  analogWrite(Pins::MAP_OUTPUT, clampT(pwm_val, 0, Config::PWM_MAX));
}

void print_telemetry() {
  static uint32_t last_print = 0;
  const uint32_t now = millis();
  if (now - last_print > 200) {
    last_print = now;

    Serial.print(F("RPM: ")); Serial.print(current_rpm, 0);
    Serial.print(F(" | Thr: ")); Serial.print(throttle_fraction * 100.0f, 0);
    Serial.print(F("% | VE: ")); Serial.print(volumetric_efficiency * 100.0f, 1);
    Serial.print(F("% | MAP: ")); Serial.print(manifold_pressure_pa / 1000.0f, 1);
    Serial.println(F(" kPa"));
  }
}
