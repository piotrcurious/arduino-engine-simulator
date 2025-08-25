#include <Arduino.h>
#include <TimerOne.h>
#include <math.h>

// define PI safely
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ----------------- CAM WHEEL TYPES -----------------
#define CAM_WHEEL_1 0
#define CAM_WHEEL_2 1
#define CAM_WHEEL_3 2
#define CAM_WHEEL_TYPE CAM_WHEEL_1

// ----------------- PIN ASSIGNMENTS -----------------
#define CAM_POS_PIN 2
#define INJ_1_PIN 3
#define INJ_2_PIN 4
#define INJ_3_PIN 5
#define INJ_4_PIN 6
#define IGN_1_PIN 7
#define IGN_2_PIN 8
#define IGN_3_PIN 9
#define IGN_4_PIN 10
#define LAMBDA_PIN 11
#define THROTTLE_PIN A0
#define FORCE_PIN A1

// ----------------- PHYSICAL CONSTANTS -----------------
#define FLYWHEEL_MASS 10.0f
#define FLYWHEEL_RADIUS 0.1f
#define FLYWHEEL_INERTIA (FLYWHEEL_MASS * FLYWHEEL_RADIUS * FLYWHEEL_RADIUS) // kg·m^2
#define TIME_STEP 0.001f // seconds (1 ms)
#define FORCE_SCALE 0.01f
#define THROTTLE_SCALE 0.01f
#define LAMBDA_SCALE (255.0f/2.0f) // maps lambda [0..2] -> [0..255]

// ----------------- FRICTION / BRAKE -----------------
// Viscous friction (proportional to angular velocity) [Nm per rad/s]
#define FLYWHEEL_FRICTION_COEFF 0.002f
// Coulomb/static friction torque [Nm] (opposes motion)
#define FLYWHEEL_STATIC_FRICTION 0.05f
// Engine braking factor when throttle closed (multiplicative on viscous term)
#define ENGINE_BRAKE_FACTOR 4.0f
#define THROTTLE_BRAKE_THRESHOLD 50 // analog units below which engine braking is applied

// ----------------- ENGINE CONSTANTS -----------------
#define BORE 0.0671f
#define STROKE 0.0706f
#define INJ_FLOW_RATE 0.0002f // m^3/s? user value (kept)
#define INJ_OPEN_TIME 0.001f
#define THROTTLE_DIAMETER 0.05f
#define THROTTLE_AREA (M_PI * THROTTLE_DIAMETER * THROTTLE_DIAMETER / 4.0f)
#define AIR_DENSITY 1.225f
#define FUEL_DENSITY 720.0f
#define OCTANE_RATING 95.0f

// ----------------- COMBUSTION MODEL -----------------
#define MIN_IGNITION_ANGLE -10.0f
#define MAX_IGNITION_ANGLE -30.0f
#define MIN_BURN_RATE 0.001f
#define MAX_BURN_RATE 0.01f
#define STOICHIOMETRIC_RATIO 14.7f
#define COMBUSTION_PRESSURE_SCALE 1e5f
#define MAX_COMBUSTION_PRESSURE (5.0e6f) // clamp to avoid unphysically huge pressure

// ----------------- STATE VARIABLES (share with ISRs -> volatile) -----------------
volatile float flywheel_angle = 0.0f; // degrees
volatile float flywheel_speed = 0.0f; // degrees per second
volatile float flywheel_torque = 0.0f; // last computed torque (for debug)
volatile float cam_angle = 0.0f;
volatile float cam_speed = 0.0f;
volatile float lambda = 1.0f;

volatile float inj_fuel[4] = {1e-9f,1e-9f,1e-9f,1e-9f}; // fuel mass/volume per cylinder (min non-zero)
volatile float ign_angle[4] = {0,0,0,0}; // degrees crank when ignition event registered
volatile float air_mass[4] = {0,0,0,0};
volatile float burn_rate[4] = {0,0,0,0};
volatile float lambda_cyl[4] = {1,1,1,1};

volatile bool cam_pos_state = false;
volatile bool inj_state[4] = {0};
volatile bool ign_state[4] = {0};
volatile unsigned long inj_start[4] = {0};
volatile unsigned long ign_start[4] = {0};

// local temporary arrays (not volatile) used inside ISR are kept on stack

// ----------------- HELPERS -----------------
static inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// clamp helper
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ----------------- ISR: MEASURE -----------------
// NOTE: keep this reasonably fast. It snapshots injector/ignition pulse widths.
void measure() {
  unsigned long now = micros();

  int injPins[4] = {INJ_1_PIN, INJ_2_PIN, INJ_3_PIN, INJ_4_PIN};
  for(int i=0;i<4;i++) {
    bool nowState = digitalRead(injPins[i]);
    if(nowState && !inj_state[i]) {
      // rising edge
      inj_start[i] = now;
    }
    if(!nowState && inj_state[i]) {
      // falling edge -> compute pulse width
      unsigned long width_us = now - inj_start[i];
      float width_s = width_us / 1e6f;
      float fuel = INJ_FLOW_RATE * width_s;
      if(fuel < 1e-9f) fuel = 1e-9f; // prevent zero
      inj_fuel[i] = fuel;
    }
    inj_state[i] = nowState;
  }

  int ignPins[4] = {IGN_1_PIN, IGN_2_PIN, IGN_3_PIN, IGN_4_PIN};
  for(int i=0;i<4;i++) {
    bool nowState = digitalRead(ignPins[i]);
    if(nowState && !ign_state[i]) {
      // rising edge - begin pulse
      ign_start[i] = now;
    }
    if(!nowState && ign_state[i]) {
      // falling edge -> ignition pulse width -> convert to approximate ignition crank position
      unsigned long width_us = now - ign_start[i];
      float width_s = width_us / 1e6f;
      // Estimate angle offset: width_s * angular speed (deg/s) -> degrees
      float estimated_offset_deg = flywheel_speed * width_s;
      // Store the ignition angle as current crank position minus offset (simple estimate)
      float angle_snapshot = flywheel_angle - estimated_offset_deg;
      // Normalize into [0..360)
      while(angle_snapshot < 0.0f) angle_snapshot += 360.0f;
      while(angle_snapshot >= 360.0f) angle_snapshot -= 360.0f;
      ign_angle[i] = angle_snapshot;
    }
    ign_state[i] = nowState;
  }
}

// ----------------- ISR: UPDATE -----------------
void update() {
  // read inputs (ADC calls are relatively slow but ok in 1ms ISR for many Arduinos; if not, move to main loop)
  int throttle_input = analogRead(THROTTLE_PIN); // 0..1023
  int force_input    = analogRead(FORCE_PIN);    // 0..1023

  // scale inputs
  float throttle_torque = throttle_input * THROTTLE_SCALE; // user scale -> Nm (user-defined)
  float external_force = (force_input - 512) * FORCE_SCALE; // signed force
  float net_torque = 0.0f;

  // external / throttle torque
  net_torque += throttle_torque;
  // external force produces a braking torque via radius
  net_torque -= external_force * FLYWHEEL_RADIUS;

  // Cylinder geometry area
  const float cyl_area = M_PI * BORE * BORE / 4.0f;

  // Combustion contribution (smooth over crank phase)
  for(int c=0;c<4;c++) {
    // Estimate incoming air mass for this cylinder (very simplified)
    float throttle_frac = clampf(throttle_input / 1023.0f, 0.0f, 1.0f);
    air_mass[c] = THROTTLE_AREA * throttle_frac * AIR_DENSITY * STROKE * cyl_area;

    // burn_rate derived from octane rating (simple map)
    burn_rate[c] = mapFloat(OCTANE_RATING, 0.0f, 100.0f, MIN_BURN_RATE, MAX_BURN_RATE);

    // compute phase relative to ignition start
    float start = ign_angle[c];
    // Normalize start into 0..360 to avoid negative issues
    float current_angle = flywheel_angle;
    float phase = current_angle - start;
    // Allow wrap-around if start > current_angle
    if(phase < 0.0f) phase += 360.0f;

    // Combustion window: 0..180 degrees (crank) after ignition event
    if(phase >= 0.0f && phase <= 180.0f) {
      // ensure some fuel is present
      float fuel_avail = inj_fuel[c];
      if(fuel_avail < 1e-9f) fuel_avail = 1e-9f;

      // updated lambda for cylinder
      lambda_cyl[c] = (air_mass[c] / fuel_avail) / STOICHIOMETRIC_RATIO;
      // clamp lambda to reasonable range
      lambda_cyl[c] = clampf(lambda_cyl[c], 0.1f, 10.0f);

      // burn fraction grows with phase, giving a smooth pressure ramp
      float burn_frac = clampf(phase / 180.0f, 0.0f, 1.0f);
      float pressure = (1.0f / lambda_cyl[c]) * COMBUSTION_PRESSURE_SCALE * burn_frac;
      pressure = clampf(pressure, 0.0f, MAX_COMBUSTION_PRESSURE);

      // produced force and torque contribution
      float forceComb = pressure * cyl_area;
      float torque_from_cyl = forceComb * FLYWHEEL_RADIUS;

      net_torque += torque_from_cyl;

      // "Consume" a portion of the injected fuel proportional to burn progress (simple model)
      // reduce fuel slightly per step (not to go negative)
      float burn_consumed = fuel_avail * (burn_rate[c] * TIME_STEP) * burn_frac;
      inj_fuel[c] = fmaxf(1e-9f, inj_fuel[c] - burn_consumed);
    } else {
      // outside combustion window -> no combustion torque, but maintain lambda estimate
      if(inj_fuel[c] < 1e-9f) inj_fuel[c] = 1e-9f;
      lambda_cyl[c] = clampf((air_mass[c]/inj_fuel[c])/STOICHIOMETRIC_RATIO, 0.1f, 10.0f);
    }
  }

  // average lambda (for lambda output)
  float lambda_avg = (lambda_cyl[0] + lambda_cyl[1] + lambda_cyl[2] + lambda_cyl[3]) * 0.25f;
  lambda_avg = clampf(lambda_avg, 0.0f, 2.0f);
  lambda = lambda_avg;

  // friction model:
  // convert flywheel_speed (deg/s) to rad/s for viscous computation
  float omega_rad_s = flywheel_speed * (M_PI / 180.0f);
  float viscous = FLYWHEEL_FRICTION_COEFF * omega_rad_s;
  // if throttle is nearly closed, increase viscous braking (engine braking)
  if (throttle_input < THROTTLE_BRAKE_THRESHOLD) viscous *= ENGINE_BRAKE_FACTOR;

  // static/Coulomb friction opposes motion direction
  float coulomb = 0.0f;
  if (fabsf(flywheel_speed) > 1e-3f) {
    coulomb = (flywheel_speed > 0.0f) ? FLYWHEEL_STATIC_FRICTION : -FLYWHEEL_STATIC_FRICTION;
  } else {
    // small velocities -> apply small static friction threshold to hold still
    if (fabsf(net_torque) < FLYWHEEL_STATIC_FRICTION) {
      // torque too small to overcome static friction: zero net torque effect
      net_torque = 0.0f;
    } else {
      // net torque large enough to move, let coulomb oppose motion
      coulomb = (net_torque > 0.0f) ? -FLYWHEEL_STATIC_FRICTION : FLYWHEEL_STATIC_FRICTION;
    }
  }

  // subtract friction torques from net torque
  net_torque -= (viscous + coulomb);

  // Save last torque for external visibility/debug
  flywheel_torque = net_torque;

  // Integrate dynamics
  // angular acceleration (rad/s^2) = torque (Nm) / inertia (kg·m^2)
  float angular_accel_rad_s2 = net_torque / FLYWHEEL_INERTIA;
  // convert angular accel to deg/s^2 for flywheel_speed units
  float angular_accel_deg_s2 = angular_accel_rad_s2 * (180.0f / M_PI);

  // update speed and angle
  flywheel_speed += angular_accel_deg_s2 * TIME_STEP;
  // if speed is extremely small and no net torque, clamp to zero
  if (fabsf(flywheel_speed) < 1e-6f && fabsf(net_torque) < 1e-6f) flywheel_speed = 0.0f;

  flywheel_angle += flywheel_speed * TIME_STEP;
  // normalize angle to [0,360)
  if (flywheel_angle >= 360.0f) flywheel_angle = fmodf(flywheel_angle, 360.0f);
  if (flywheel_angle < 0.0f) flywheel_angle += 360.0f;

  // cam geometry (example: cam rotates at half crank speed)
  cam_angle = fmodf(flywheel_angle / 2.0f, 360.0f);
  cam_speed = flywheel_speed / 2.0f;

  // update lambda output (PWM)
  int lambda_pwm = (int) clampf(lambda * LAMBDA_SCALE, 0.0f, 255.0f);
  analogWrite(LAMBDA_PIN, lambda_pwm);

  // cam sensor logic (simple square wave pattern)
  switch(CAM_WHEEL_TYPE) {
    case CAM_WHEEL_1: cam_pos_state = (fmodf(cam_angle,180.0f) < 90.0f); break;
    case CAM_WHEEL_2: cam_pos_state = (fmodf(cam_angle,90.0f) < 45.0f); break;
    case CAM_WHEEL_3: cam_pos_state = (fmodf(cam_angle,60.0f) < 30.0f); break;
    default: cam_pos_state = (fmodf(cam_angle,180.0f) < 90.0f); break;
  }
  digitalWrite(CAM_POS_PIN, cam_pos_state ? HIGH : LOW);
}

// ----------------- SETUP -----------------
void setup() {
  // pin modes
  pinMode(CAM_POS_PIN, OUTPUT);
  pinMode(LAMBDA_PIN, OUTPUT);

  pinMode(INJ_1_PIN, INPUT);
  pinMode(INJ_2_PIN, INPUT);
  pinMode(INJ_3_PIN, INPUT);
  pinMode(INJ_4_PIN, INPUT);

  pinMode(IGN_1_PIN, INPUT);
  pinMode(IGN_2_PIN, INPUT);
  pinMode(IGN_3_PIN, INPUT);
  pinMode(IGN_4_PIN, INPUT);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(FORCE_PIN, INPUT);

  // initialize timers: call measure() and update() every TIME_STEP seconds
  unsigned long period_us = (unsigned long)(TIME_STEP * 1e6f);
  if (period_us < 1) period_us = 1;
  Timer1.initialize(period_us);

  // Use a lambda that calls both measure and update in sequence.
  // Keep the ISR body small, but both functions are reasonably light for a 1ms timer.
  Timer1.attachInterrupt([](){
    measure();
    update();
  });

  // optional: initialize serial for debugging (not used in ISR)
  Serial.begin(115200);
  Serial.println("Engine sim started");
}

// ----------------- MAIN LOOP -----------------
void loop() {
  // all real-time simulation happens in the Timer ISR.
  // keep loop free for optional telemetry, commands, or logging at lower priority.

  static unsigned long last_print = 0;
  unsigned long now = millis();
  if (now - last_print > 250) { // print every 250 ms
    last_print = now;
    // read volatile variables once into local copies for printing
    noInterrupts();
    float angle = flywheel_angle;
    float speed = flywheel_speed;
    float torque = flywheel_torque;
    float lam = lambda;
    interrupts();

    Serial.print("angle: "); Serial.print(angle);
    Serial.print("  speed (deg/s): "); Serial.print(speed);
    Serial.print("  torque (Nm): "); Serial.print(torque);
    Serial.print("  lambda: "); Serial.println(lam);
  }

  // small delay to let other tasks run
  delay(10);
}
