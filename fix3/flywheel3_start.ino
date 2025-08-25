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
#define START_PIN 12    // external start button (active LOW)
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
#define INJ_FLOW_RATE 0.0002f // user supplied unit
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

// ----------------- STARTER CONSTANTS -----------------
#define STARTER_TORQUE 5.0f              // Nm supplied by starter while cranking
#define STARTER_SPEED_THRESHOLD 600.0f   // deg/s - once engine above this, we consider it 'caught'

// ----------------- CYLINDER MAPPING -----------------
// firing order for 4-cylinder inline: 1-3-4-2 (indexing 0..3 -> cylinder IDs 0..3)
const int firing_order[4] = {0, 2, 3, 1};
// firing start angles for the 720-degree (two-rotation) cycle, in degrees
const float firing_start[4] = {0.0f, 180.0f, 360.0f, 540.0f};

// ----------------- STATE VARIABLES (shared with ISR -> volatile) -----------------
volatile float flywheel_angle = 0.0f; // degrees (kept normalized 0..360)
volatile float flywheel_speed = 0.0f; // degrees per second
volatile float flywheel_torque = 0.0f; // last computed net torque (Nm)
volatile float cam_angle = 0.0f;
volatile float cam_speed = 0.0f;
volatile float lambda = 1.0f;

// crank angle in 0..720° space (keeps track of 2-rev cycle phase); updated from flywheel_speed
volatile float crank_angle720 = 0.0f;

volatile float inj_fuel[4] = {1e-9f,1e-9f,1e-9f,1e-9f}; // fuel mass/volume per cylinder (min non-zero)
volatile float ign_angle[4] = {0,0,0,0}; // (kept for compatibility if ignition pulses used)
volatile float air_mass[4] = {0,0,0,0};
volatile float burn_rate[4] = {0,0,0,0};
volatile float lambda_cyl[4] = {1,1,1,1};

volatile bool cam_pos_state = false;
volatile bool inj_state[4] = {0};
volatile bool ign_state[4] = {0};
volatile unsigned long inj_start[4] = {0};
volatile unsigned long ign_start[4] = {0};

// ----------------- HELPERS -----------------
static inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ----------------- ISR: MEASURE -----------------
// Keep reasonably small: snapshots injector/ignition pulse widths
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
      if(fuel < 1e-9f) fuel = 1e-9f;
      inj_fuel[i] = fuel;
    }
    inj_state[i] = nowState;
  }

  int ignPins[4] = {IGN_1_PIN, IGN_2_PIN, IGN_3_PIN, IGN_4_PIN};
  for(int i=0;i<4;i++) {
    bool nowState = digitalRead(ignPins[i]);
    if(nowState && !ign_state[i]) {
      // rising edge - start
      ign_start[i] = now;
    }
    if(!nowState && ign_state[i]) {
      // falling edge -> estimate ignition angle using measured pulse width and current speed
      unsigned long width_us = now - ign_start[i];
      float width_s = width_us / 1e6f;
      float estimated_offset_deg = flywheel_speed * width_s; // deg approximation
      float angle_snapshot = flywheel_angle - estimated_offset_deg;
      while(angle_snapshot < 0.0f) angle_snapshot += 360.0f;
      while(angle_snapshot >= 360.0f) angle_snapshot -= 360.0f;
      ign_angle[i] = angle_snapshot;
    }
    ign_state[i] = nowState;
  }
}

// ----------------- ISR: UPDATE -----------------
void update() {
  // Read ADCs (note: ADC in ISR may be slow on some boards; ok for basic sims)
  int throttle_input = analogRead(THROTTLE_PIN); // 0..1023
  int force_input    = analogRead(FORCE_PIN);    // 0..1023

  float throttle_torque = throttle_input * THROTTLE_SCALE; // user defined units -> Nm
  float external_force = (force_input - 512) * FORCE_SCALE; // signed
  float net_torque = 0.0f;

  // external / throttle torque
  net_torque += throttle_torque;
  // external force produces a braking torque via radius
  net_torque -= external_force * FLYWHEEL_RADIUS;

  // Starter assist (active low button). Provide starter torque while button pressed AND engine below threshold.
  bool starter_pressed = (digitalRead(START_PIN) == LOW);
  if (starter_pressed && fabsf(flywheel_speed) < STARTER_SPEED_THRESHOLD) {
    net_torque += STARTER_TORQUE;
  }

  const float cyl_area = M_PI * BORE * BORE / 4.0f;

  // Precompute throttle fraction and burn rates
  float throttle_frac = clampf(throttle_input / 1023.0f, 0.0f, 1.0f);
  for(int c=0;c<4;c++) {
    air_mass[c] = THROTTLE_AREA * throttle_frac * AIR_DENSITY * STROKE * cyl_area;
    burn_rate[c] = mapFloat(OCTANE_RATING, 0.0f, 100.0f, MIN_BURN_RATE, MAX_BURN_RATE);
  }

  // Update crank 720° phase using flywheel_speed
  // flywheel_speed is degrees per second; crank_angle720 increments by same angle per step (we interpret flywheel angle as crank angle for mapping; 2 rotations = 720°)
  crank_angle720 += flywheel_speed * TIME_STEP;
  // normalize into [0..720)
  crank_angle720 = fmodf(crank_angle720, 720.0f);
  if (crank_angle720 < 0.0f) crank_angle720 += 720.0f;

  // Combustion mapping using firing order + 720° cycle
  for (int slot = 0; slot < 4; slot++) {
    int cyl = firing_order[slot];            // cylinder index (0..3)
    float start_angle = firing_start[slot];  // start of that cylinder's power stroke in 720° cycle
    float phase = crank_angle720 - start_angle;
    if (phase < 0.0f) phase += 720.0f;        // wrap-around

    // combustion window: 0..180° crank after ignition event -> power stroke
    if (phase >= 0.0f && phase <= 180.0f) {
      // ensure some fuel
      float fuel_avail = inj_fuel[cyl];
      if (fuel_avail < 1e-9f) fuel_avail = 1e-9f;

      // lambda for cylinder
      lambda_cyl[cyl] = (air_mass[cyl] / fuel_avail) / STOICHIOMETRIC_RATIO;
      lambda_cyl[cyl] = clampf(lambda_cyl[cyl], 0.1f, 10.0f);

      // smooth burn fraction across the 180° window
      float burn_frac = clampf(phase / 180.0f, 0.0f, 1.0f);
      float pressure = (1.0f / lambda_cyl[cyl]) * COMBUSTION_PRESSURE_SCALE * burn_frac;
      pressure = clampf(pressure, 0.0f, MAX_COMBUSTION_PRESSURE);

      float forceComb = pressure * cyl_area;
      float torque_from_cyl = forceComb * FLYWHEEL_RADIUS;

      net_torque += torque_from_cyl;

      // simple fuel consumption model - reduce inj_fuel gradually during burn
      float burn_consumed = fuel_avail * (burn_rate[cyl] * TIME_STEP) * burn_frac;
      inj_fuel[cyl] = fmaxf(1e-9f, inj_fuel[cyl] - burn_consumed);
    } else {
      // outside combustion window: keep lambda estimate reasonable
      if (inj_fuel[cyl] < 1e-9f) inj_fuel[cyl] = 1e-9f;
      lambda_cyl[cyl] = clampf((air_mass[cyl] / inj_fuel[cyl]) / STOICHIOMETRIC_RATIO, 0.1f, 10.0f);
    }
  }

  // lambda average for output mapping
  float lambda_avg = (lambda_cyl[0] + lambda_cyl[1] + lambda_cyl[2] + lambda_cyl[3]) * 0.25f;
  lambda_avg = clampf(lambda_avg, 0.0f, 2.0f);
  lambda = lambda_avg;

  // friction model:
  // convert flywheel_speed (deg/s) to rad/s for viscous computation
  float omega_rad_s = flywheel_speed * (M_PI / 180.0f);
  float viscous = FLYWHEEL_FRICTION_COEFF * omega_rad_s;
  // engine braking if throttle nearly closed
  if (throttle_input < THROTTLE_BRAKE_THRESHOLD) viscous *= ENGINE_BRAKE_FACTOR;

  // Coulomb/static friction opposes motion direction
  float coulomb = 0.0f;
  if (fabsf(flywheel_speed) > 1e-3f) {
    coulomb = (flywheel_speed > 0.0f) ? FLYWHEEL_STATIC_FRICTION : -FLYWHEEL_STATIC_FRICTION;
  } else {
    // small velocities -> if net_torque cannot overcome static friction, hold still
    if (fabsf(net_torque) < FLYWHEEL_STATIC_FRICTION) {
      net_torque = 0.0f;
    } else {
      coulomb = (net_torque > 0.0f) ? -FLYWHEEL_STATIC_FRICTION : FLYWHEEL_STATIC_FRICTION;
    }
  }

  net_torque -= (viscous + coulomb);

  // store last torque for telemetry
  flywheel_torque = net_torque;

  // Integrate dynamics
  float angular_accel_rad_s2 = net_torque / FLYWHEEL_INERTIA; // rad/s^2
  float angular_accel_deg_s2 = angular_accel_rad_s2 * (180.0f / M_PI);

  flywheel_speed += angular_accel_deg_s2 * TIME_STEP;
  // deadband very near zero
  if (fabsf(flywheel_speed) < 1e-6f && fabsf(net_torque) < 1e-6f) flywheel_speed = 0.0f;

  // update flywheel_angle (normalized)
  flywheel_angle += flywheel_speed * TIME_STEP;
  // normalize to [0..360)
  flywheel_angle = fmodf(flywheel_angle, 360.0f);
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

  pinMode(START_PIN, INPUT_PULLUP); // active low start button

  // initialize timers: call measure() and update() every TIME_STEP seconds
  unsigned long period_us = (unsigned long)(TIME_STEP * 1e6f);
  if (period_us < 1) period_us = 1;
  Timer1.initialize(period_us);

  // Attach ISR that runs measure() then update()
  Timer1.attachInterrupt([](){
    measure();
    update();
  });

  Serial.begin(115200);
  Serial.println("Engine sim with starter and 720-degree mapping started");
}

// ----------------- MAIN LOOP -----------------
void loop() {
  // all real-time simulation happens in the Timer ISR.
  // use main loop for lower-priority telemetry / control

  static unsigned long last_print = 0;
  unsigned long now = millis();
  if (now - last_print > 250) { // print every 250 ms
    last_print = now;
    // snapshot volatile variables
    noInterrupts();
    float angle = flywheel_angle;
    float speed = flywheel_speed;
    float torque = flywheel_torque;
    float lam = lambda;
    float crank720_snap = crank_angle720;
    interrupts();

    Serial.print("angle: "); Serial.print(angle, 2);
    Serial.print("  speed (deg/s): "); Serial.print(speed, 2);
    Serial.print("  torque (Nm): "); Serial.print(torque, 3);
    Serial.print("  lambda: "); Serial.print(lam, 3);
    Serial.print("  crank720: "); Serial.println(crank720_snap, 2);
  }

  delay(10);
}
