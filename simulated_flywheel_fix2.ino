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
#define FLYWHEEL_INERTIA (FLYWHEEL_MASS * FLYWHEEL_RADIUS * FLYWHEEL_RADIUS)
#define TIME_STEP 0.001f // 1 ms
#define FORCE_SCALE 0.01f
#define THROTTLE_SCALE 0.01f
#define LAMBDA_SCALE (255.0f/2.0f)

// ----------------- ENGINE CONSTANTS -----------------
#define BORE 0.0671f
#define STROKE 0.0706f
#define INJ_FLOW_RATE 0.0002f
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

// ----------------- STATE VARIABLES -----------------
float flywheel_angle = 0.0f;
float flywheel_speed = 0.0f;
float flywheel_torque = 0.0f;
float cam_angle = 0.0f;
float cam_speed = 0.0f;
float lambda = 1.0f;

float inj_fuel[4] = {0,0,0,0};
float ign_angle[4] = {0,0,0,0};
float air_mass[4] = {0,0,0,0};
float burn_rate[4] = {0,0,0,0};
float lambda_cyl[4] = {1,1,1,1};

bool cam_pos_state = false;
bool inj_state[4] = {0};
bool ign_state[4] = {0};
unsigned long inj_start[4] = {0};
unsigned long ign_start[4] = {0};

// ----------------- HELPERS -----------------
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ----------------- ISR: MEASURE -----------------
void measure() {
  unsigned long now = micros();

  int injPins[4] = {INJ_1_PIN, INJ_2_PIN, INJ_3_PIN, INJ_4_PIN};
  for(int i=0;i<4;i++) {
    bool nowState = digitalRead(injPins[i]);
    if(nowState && !inj_state[i]) inj_start[i] = now;
    if(!nowState && inj_state[i]) {
      float width = (now - inj_start[i]) / 1e6f;
      inj_fuel[i] = INJ_FLOW_RATE * width;
      if(inj_fuel[i] < 1e-9f) inj_fuel[i] = 1e-9f; // prevent zero
    }
    inj_state[i] = nowState;
  }

  int ignPins[4] = {IGN_1_PIN, IGN_2_PIN, IGN_3_PIN, IGN_4_PIN};
  for(int i=0;i<4;i++) {
    bool nowState = digitalRead(ignPins[i]);
    if(nowState && !ign_state[i]) ign_start[i] = now;
    if(!nowState && ign_state[i]) {
      float width = (now - ign_start[i]) / 1e6f;
      ign_angle[i] = flywheel_angle - width * flywheel_speed;
    }
    ign_state[i] = nowState;
  }
}

// ----------------- ISR: UPDATE -----------------
void update() {
  // reset torque each cycle
  flywheel_torque = 0.0f;

  int throttle_input = analogRead(THROTTLE_PIN);
  int force_input    = analogRead(FORCE_PIN);

  float throttle_torque = throttle_input * THROTTLE_SCALE;
  float force = (force_input - 512) * FORCE_SCALE;

  flywheel_torque += throttle_torque - force * FLYWHEEL_RADIUS;

  flywheel_speed += (flywheel_torque / FLYWHEEL_INERTIA) * TIME_STEP * (180.0/M_PI);
  flywheel_angle += flywheel_speed * TIME_STEP;
  if(flywheel_angle >= 360.0f) flywheel_angle -= 360.0f;
  if(flywheel_angle < 0.0f) flywheel_angle += 360.0f;

  cam_angle = fmod(flywheel_angle/2.0f,360.0f);
  cam_speed = flywheel_speed/2.0f;

  // combustion
  for(int c=0;c<4;c++) {
    air_mass[c] = THROTTLE_AREA * (throttle_input/1023.0f) * AIR_DENSITY * STROKE * (M_PI*BORE*BORE/4.0f);
    burn_rate[c] = mapFloat(OCTANE_RATING,0,100,MIN_BURN_RATE,MAX_BURN_RATE);

    float start = ign_angle[c];
    float end   = ign_angle[c] + 180.0f;

    if(flywheel_angle >= start && flywheel_angle <= end) {
      float burned = (flywheel_angle-start)*burn_rate[c];
      if(inj_fuel[c] < 1e-9f) inj_fuel[c] = 1e-9f; // avoid div/0
      lambda_cyl[c] = (air_mass[c]/inj_fuel[c]) / STOICHIOMETRIC_RATIO;

      float pressure = (1.0f/lambda_cyl[c]) * COMBUSTION_PRESSURE_SCALE;
      float area = M_PI*BORE*BORE/4.0f;
      float forceComb = pressure*area;
      flywheel_torque += forceComb*FLYWHEEL_RADIUS;
    }
  }

  lambda = (lambda_cyl[0]+lambda_cyl[1]+lambda_cyl[2]+lambda_cyl[3])/4.0f;
  lambda = constrain(lambda,0.0f,2.0f);
  analogWrite(LAMBDA_PIN, constrain(lambda*LAMBDA_SCALE,0,255));

  // cam sensor
  switch(CAM_WHEEL_TYPE) {
    case CAM_WHEEL_1: cam_pos_state = (fmod(cam_angle,180.0f)<90); break;
    case CAM_WHEEL_2: cam_pos_state = (fmod(cam_angle,90.0f)<45); break;
    case CAM_WHEEL_3: cam_pos_state = (fmod(cam_angle,60.0f)<30); break;
  }
  digitalWrite(CAM_POS_PIN,cam_pos_state);
}

// ----------------- SETUP -----------------
void setup() {
  pinMode(CAM_POS_PIN,OUTPUT);
  pinMode(LAMBDA_PIN,OUTPUT);
  pinMode(INJ_1_PIN,INPUT);
  pinMode(INJ_2_PIN,INPUT);
  pinMode(INJ_3_PIN,INPUT);
  pinMode(INJ_4_PIN,INPUT);
  pinMode(IGN_1_PIN,INPUT);
  pinMode(IGN_2_PIN,INPUT);
  pinMode(IGN_3_PIN,INPUT);
  pinMode(IGN_4_PIN,INPUT);
  pinMode(THROTTLE_PIN,INPUT);
  pinMode(FORCE_PIN,INPUT);

  Timer1.initialize(TIME_STEP*1e6);
  Timer1.attachInterrupt([](){
    measure();
    update();
  });
}

void loop() {
  // nothing in main loop
}
