
// Define the cam wheel types
#define CAM_WHEEL_1 0 // 4 teeth, 90 degrees apart
#define CAM_WHEEL_2 1 // 8 teeth, 45 degrees apart
#define CAM_WHEEL_3 2 // 12 teeth, 30 degrees apart

// Choose the cam wheel type
#define CAM_WHEEL_TYPE CAM_WHEEL_1

// Define the pins for the cam position sensor, injector and ignition signals
#define CAM_POS_PIN 2 // digital output
#define INJ_1_PIN 3 // digital input
#define INJ_2_PIN 4 // digital input
#define INJ_3_PIN 5 // digital input
#define INJ_4_PIN 6 // digital input
#define IGN_1_PIN 7 // digital input
#define IGN_2_PIN 8 // digital input
#define IGN_3_PIN 9 // digital input
#define IGN_4_PIN 10 // digital input

// Define the pin for the lambda signal
#define LAMBDA_PIN 11 // analog output

// Define the pins for the throttle and force inputs
#define THROTTLE_PIN A0 // analog input
#define FORCE_PIN A1 // analog input

// Define the constants for the simulation
#define FLYWHEEL_MASS 10.0 // kg
#define FLYWHEEL_RADIUS 0.1 // m
#define FLYWHEEL_INERTIA (FLYWHEEL_MASS * FLYWHEEL_RADIUS * FLYWHEEL_RADIUS) // kg*m^2
#define MAX_RPM 6000.0 // revolutions per minute
#define MAX_ANGLE (MAX_RPM * 6.0) // degrees per second
#define TIME_STEP 0.001 // seconds
#define FORCE_SCALE 0.01 // N per analog unit
#define THROTTLE_SCALE 0.01 // Nm per analog unit
#define LAMBDA_SCALE 255.0 / 2.0 // analog units per lambda unit

// Define the variables for the simulation state
float flywheel_angle = 0.0; // degrees
float flywheel_speed = 0.0; // degrees per second
float flywheel_torque = 0.0; // Nm
float cam_angle = 0.0; // degrees
float cam_speed = 0.0; // degrees per second
float lambda = 1.0; // air-fuel ratio

// Define the variables for the combustion simulation
bool inj_1_state = false; // whether injector 1 is on or off
bool inj_2_state = false; // whether injector 2 is on or off
bool inj_3_state = false; // whether injector 3 is on or off
bool inj_4_state = false; // whether injector 4 is on or off
bool ign_1_state = false; // whether ignition 1 is on or off
bool ign_2_state = false; // whether ignition 2 is on or off
bool ign_3_state = false; // whether ignition 3 is on or off
bool ign_4_state = false; // whether ignition 4 is on or off

// Define the variables for the cam position sensor output
bool cam_pos_state = false; // whether the cam position sensor is high or low

// Define the bore and stroke dimensions of the pistons
#define BORE 67.1 // mm
#define STROKE 70.6 // mm

// Define the Bosch injector specifications
#define INJ_FLOW_RATE 0.0002 // kg/s
#define INJ_OPEN_TIME 0.001 // s

// Define the throttle body specifications
#define THROTTLE_DIAMETER 50.0 // mm
#define THROTTLE_AREA (PI * THROTTLE_DIAMETER * THROTTLE_DIAMETER / 4.0) // mm^2

// Define the air density and fuel density
#define AIR_DENSITY 1.225 // kg/m^3
#define FUEL_DENSITY 720.0 // kg/m^3

// Define the octane rating of the fuel
#define OCTANE_RATING 95.0

// Define the constants for the combustion simulation
#define MIN_IGNITION_ANGLE -10.0 // degrees before TDC
#define MAX_IGNITION_ANGLE -30.0 – degrees before TDC 
#define MIN_BURN_RATE – fraction per degree 
#define MAX_BURN_RATE – fraction per degree 
#define STOICHIOMETRIC_RATIO – air-fuel ratio 
#define COMBUSTION_PRESSURE_SCALE – Pa per lambda unit

// Define the variables for the combustion parameters
float inj_1_fuel = 0.0; // kg
float inj_2_fuel = 0.0; // kg
float inj_3_fuel = 0.0; // kg
float inj_4_fuel = 0.0; // kg
float air_1 = 0.0; // kg
float air_2 = 0.0; // kg
float air_3 = 0.0; // kg
float air_4 = 0.0; // kg
float ign_1_angle = 0.0; // degrees before TDC
float ign_2_angle = 0.0; // degrees before TDC
float ign_3_angle = 0.0; // degrees before TDC
float ign_4_angle = 0.0; // degrees before TDC
float burn_1_rate = 0.0; // fraction per degree
float burn_2_rate = 0.0; // fraction per degree
float burn_3_rate = 0.0; // fraction per degree
float burn_4_rate = 0.0; // fraction per degree

// Define the variables for the timer interrupt
unsigned long inj_1_start = 0; // start time of injector 1 pulse
unsigned long inj_2_start = 0; // start time of injector 2 pulse
unsigned long inj_3_start = 0; // start time of injector 3 pulse
unsigned long inj_4_start = 0; // start time of injector 4 pulse
unsigned long ign_1_start = 0; // start time of ignition 1 pulse
unsigned long ign_2_start = 0; // start time of ignition 2 pulse
unsigned long ign_3_start = 0; // start time of ignition 3 pulse
unsigned long ign_4_start = 0; // start time of ignition 4 pulse

// Define the timer interrupt handler function
void measure() {
  
  // Read the current time in microseconds
  unsigned long now = micros();

  // Check the state of each injector and ignition signal
  bool inj_1_now = digitalRead(INJ_1_PIN);
  bool inj_2_now = digitalRead(INJ_2_PIN);
  bool inj_3_now = digitalRead(INJ_3_PIN);
  bool inj_4_now = digitalRead(INJ_4_PIN);
  bool ign_1_now = digitalRead(IGN_1_PIN);
  bool ign_2_now = digitalRead(IGN_2_PIN);
  bool ign_3_now = digitalRead(IGN_3_PIN);
  bool ign_4_now = digitalRead(IGN_4_PIN);

  // If the injector signal is rising, record the start time
  if (inj_1_now && !inj_1_state) {
    inj_1_start = now;
  }
  if (inj_2_now && !inj_2_state) {
    inj_2_start = now;
  }
  if (inj_3_now && !inj_3_state) {
    inj_3_start = now;
  }
  if (inj_4_now && !inj_4_state) {
    inj_4_start = now;
  }

  // If the injector signal is falling, calculate the pulse width and the fuel amount
  if (!inj_1_now && inj_1_state) {
    float inj_1_width = (now - inj_1_start) / 1000000.0; // seconds
    inj_1_fuel = INJ_FLOW_RATE * inj_1_width; // kg
  }
  if (!inj_2_now && inj_2_state) {
    float inj_2_width = (now - inj_2_start) / 1000000.0; // seconds
    inj_2_fuel = INJ_FLOW_RATE * inj_2_width; // kg
  }
  if (!inj_3_now && inj_3_state) {
    float inj_3_width = (now - inj_3_start) / 1000000.0; // seconds
    inj_3_fuel = INJ_FLOW_RATE * inj_3_width; // kg
  }
  if (!inj_4_now && inj_4_state) {
    float inj_4_width = (now - inj_4_start) / 1000000.0; // seconds
    inj_4_fuel = INJ_FLOW_RATE * inj_4_width; // kg
  }

  // If the ignition signal is rising, record the start time
  if (ign_1_now && !ign_1_state) {
    ign_1_start = now;
  }

  if (ign_2_now && !ign_2_state) {
    ign_2_start = now;
  }
  if (ign_3_now && !ign_3_state) {
    ign_3_start = now;
  }
  if (ign_4_now && !ign_4_state) {
    ign_4_start = now;
  }

  // If the ignition signal is falling, calculate the pulse width and the ignition angle
  if (!ign_1_now && ign_1_state) {
    float ign_1_width = (now - ign_1_start) / 1000000.0; // seconds
    ign_1_angle = flywheel_angle - ign_1_width * flywheel_speed; // degrees before TDC
  }
  if (!ign_2_now && ign_2_state) {
    float ign_2_width = (now - ign_2_start) / 1000000.0; // seconds
    ign_2_angle = flywheel_angle - ign_2_width * flywheel_speed; // degrees before TDC
  }
  if (!ign_3_now && ign_3_state) {
    float ign_3_width = (now - ign_3_start) / 1000000.0; // seconds
    ign_3_angle = flywheel_angle - ign_3_width * flywheel_speed; // degrees before TDC
  }
  if (!ign_4_now && ign_4_state) {
    float ign_4_width = (now - ign_4_start) / 1000000.0; // seconds
    ign_4_angle = flywheel_angle - ign_4_width * flywheel_speed; // degrees before TDC
  }

  // Update the injector and ignition states
  inj_1_state = inj_1_now;
  inj_2_state = inj_2_now;
  inj_3_state = inj_3_now;
  inj_4_state = inj_4_now;
  ign_1_state = ign_1_now;
  ign_2_state = ign_2_now;
  ign_3_state = ign_3_now;
  ign_4_state = ign_4_now;
}

// Define the timer interrupt handler function
void update() {
  
  // Read the throttle and force inputs from the analog pins
  int throttle_input = analogRead(THROTTLE_PIN);
  int force_input = analogRead(FORCE_PIN);

  // Convert the inputs to physical units
  float throttle_torque = throttle_input * THROTTLE_SCALE; // Nm
  float force = (force_input - 512) * FORCE_SCALE; // N

  // Calculate the net torque on the flywheel
  flywheel_torque = throttle_torque - force * FLYWHEEL_RADIUS; // Nm

  // Update the flywheel speed and angle using Euler's method
  flywheel_speed += flywheel_torque / FLYWHEEL_INERTIA * TIME_STEP; // degrees per second
  flywheel_angle += flywheel_speed * TIME_STEP; // degrees

  // Constrain the flywheel angle to [0, 360) degrees
  flywheel_angle = fmod(flywheel_angle, 360.0);

  // Update the cam speed and angle using the flywheel speed
  cam_speed = flywheel_speed / 2.0; // degrees per second
  cam_angle = flywheel_angle / 2.0; // degrees

  // Constrain the cam angle to [0, 360) degrees
  cam_angle = fmod(cam_angle, 360.0);

  // Simulate the combustion process for each cylinder
  // Calculate the air amount based on the throttle position and air density
  // Calculate the burn rate based on the octane rating and a linear interpolation
  // Calculate the lambda value based on the fuel to air ratio and the stoichiometric ratio
  // Calculate the flywheel torque based on the combustion pressure and the piston area
  switch ((int)cam_angle) {
    case 0: // cylinder 1
      air_1 = THROTTLE_AREA * THROTTLE_SCALE * throttle_input * AIR_DENSITY / 1000000.0; // calculate air amount
      burn_1_rate = map(OCTANE_RATING, 0, 100, MIN_BURN_RATE, MAX_BURN_RATE); // calculate burn rate
      if (flywheel_angle >= ign_1_angle && flywheel_angle <= ign_1_angle + 180.0) { // combustion occurs
        float burn_1_fraction = (flywheel_angle - ign_1_angle) * burn_1_rate; // calculate burned fraction
        float lambda_1 = inj_1_fuel / air_1 / STOICHIOMETRIC_RATIO; // calculate lambda value
        float pressure_1 = lambda_1 * COMBUSTION_PRESSURE_SCALE; // calculate combustion pressure
        float area_1 = PI * BORE * BORE / 4.0; // calculate piston area
        float force_1 = pressure_1 * area_1; // calculate combustion force
        flywheel_torque += force_1 * FLYWHEEL_RADIUS; // increase flywheel torque
      }
      break;
    case 90: // cylinder 2
      air_2 = THROTTLE_AREA * THROTTLE_SCALE * throttle_input * AIR_DENSITY / 1000000.0; // calculate air amount
      burn_2_rate = map(OCTANE_RATING, 0, 100, MIN_BURN_RATE, MAX_BURN_RATE); // calculate burn rate
      if (flywheel_angle >= ign_2_angle && flywheel_angle <= ign_2_angle + 180.0) { // combustion occurs
        float burn_2_fraction = (flywheel_angle - ign_2_angle) * burn_2_rate; // calculate burned fraction
        float lambda_2 = inj_2_fuel / air_2 / STOICHIOMETRIC_RATIO; // calculate lambda value
        float pressure_2 = lambda_2 * COMBUSTION_PRESSURE_SCALE; // calculate combustion pressure
        float area_2 = PI * BORE * BORE / 4.0; // calculate piston area
        float force_2 = pressure_2 * area_2; // calculate combustion force
        flywheel_torque += force_2 * FLYWHEEL_RADIUS; // increase flywheel torque
      }

      break;
    case 180: // cylinder 3
      air_3 = THROTTLE_AREA * THROTTLE_SCALE * throttle_input * AIR_DENSITY / 1000000.0; // calculate air amount
      burn_3_rate = map(OCTANE_RATING, 0, 100, MIN_BURN_RATE, MAX_BURN_RATE); // calculate burn rate
      if (flywheel_angle >= ign_3_angle && flywheel_angle <= ign_3_angle + 180.0) { // combustion occurs
        float burn_3_fraction = (flywheel_angle - ign_3_angle) * burn_3_rate; // calculate burned fraction
        float lambda_3 = inj_3_fuel / air_3 / STOICHIOMETRIC_RATIO; // calculate lambda value
        float pressure_3 = lambda_3 * COMBUSTION_PRESSURE_SCALE; // calculate combustion pressure
        float area_3 = PI * BORE * BORE / 4.0; // calculate piston area
        float force_3 = pressure_3 * area_3; // calculate combustion force
        flywheel_torque += force_3 * FLYWHEEL_RADIUS; // increase flywheel torque
      }
      break;
    case 270: // cylinder 4
      air_4 = THROTTLE_AREA * THROTTLE_SCALE * throttle_input * AIR_DENSITY / 1000000.0; // calculate air amount
      burn_4_rate = map(OCTANE_RATING, 0, 100, MIN_BURN_RATE, MAX_BURN_RATE); // calculate burn rate
      if (flywheel_angle >= ign_4_angle && flywheel_angle <= ign_4_angle + 180.0) { // combustion occurs
        float burn_4_fraction = (flywheel_angle - ign_4_angle) * burn_4_rate; // calculate burned fraction
        float lambda_4 = inj_4_fuel / air_4 / STOICHIOMETRIC_RATIO; // calculate lambda value
        float pressure_4 = lambda_4 * COMBUSTION_PRESSURE_SCALE; // calculate combustion pressure
        float area_4 = PI * BORE * BORE / 4.0; // calculate piston area
        float force_4 = pressure_4 * area_4; // calculate combustion force
        flywheel_torque += force_4 * FLYWHEEL_RADIUS; // increase flywheel torque
      }
      break;
    default: // no combustion
      break;
  }

  // Calculate the average lambda value from the four cylinders
  lambda = (lambda_1 + lambda_2 + lambda_3 + lambda_4) / 4.0;

  // Write the lambda value to the analog pin
  analogWrite(LAMBDA_PIN, lambda * LAMBDA_SCALE);

  // Generate the cam position sensor output based on the cam wheel type
  switch (CAM_WHEEL_TYPE) {
    case CAM_WHEEL_1: // 4 teeth, 90 degrees apart
      if (cam_angle >= 0 && cam_angle < 90) {
        cam_pos_state = true;
      } else if (cam_angle >= 90 && cam_angle < 180) {
        cam_pos_state = false;
      } else if (cam_angle >= 180 && cam_angle < 270) {
        cam_pos_state = true;
      } else {
        cam_pos_state = false;
      }
      break;
    case CAM_WHEEL_2: // 8 teeth, 45 degrees apart
      if (cam_angle >= 0 && cam_angle < 45) {
        cam_pos_state = true;
      } else if (cam_angle >= 45 && cam_angle < 90) {
        cam_pos_state = false;
      } else if (cam_angle >= 90 && cam_angle < 135) {
        cam_pos_state = true;
      } else if (cam_angle >= 135 && cam_angle < 180) {
        cam_pos_state = false;
      } else if (cam_angle >= 180 && cam_angle < 225) {
        cam_pos_state = true;
      } else if (cam_angle >= 225 && cam_angle < 270) {
        cam_pos_state = false;
      } else if (cam_angle >= 270 && cam_angle < 315) {
        cam_pos_state = true;
      } else {
        cam_pos_state = false;
      }
      break;
    case CAM_WHEEL_3: // 12 teeth, 30 degrees apart

      if (cam_angle >= 0 && cam_angle < 30) {
        cam_pos_state = true;
      } else if (cam_angle >= 30 && cam_angle < 60) {
        cam_pos_state = false;
      } else if (cam_angle >= 60 && cam_angle < 90) {
        cam_pos_state = true;
      } else if (cam_angle >= 90 && cam_angle < 120) {
        cam_pos_state = false;
      } else if (cam_angle >= 120 && cam_angle < 150) {
        cam_pos_state = true;
      } else if (cam_angle >= 150 && cam_angle < 180) {
        cam_pos_state = false;
      } else if (cam_angle >= 180 && cam_angle < 210) {
        cam_pos_state = true;
      } else if (cam_angle >= 210 && cam_angle < 240) {
        cam_pos_state = false;
      } else if (cam_angle >= 240 && cam_angle < 270) {
        cam_pos_state = true;
      } else if (cam_angle >= 270 && cam_angle < 300) {
        cam_pos_state = false;
      } else if (cam_angle >= 300 && cam_angle < 330) {
        cam_pos_state = true;
      } else {
        cam_pos_state = false;
      }
      break;
    default: // invalid type
      break;
  }

  // Write the cam position sensor output to the digital pin
  digitalWrite(CAM_POS_PIN, cam_pos_state);
}

// Setup the pins and interrupts
void setup() {

  // Set the pin modes
  pinMode(CAM_POS_PIN, OUTPUT);
  pinMode(INJ_1_PIN, INPUT);
  pinMode(INJ_2_PIN, INPUT);
  pinMode(INJ_3_PIN, INPUT);
  pinMode(INJ_4_PIN, INPUT);
  pinMode(IGN_1_PIN, INPUT);
  pinMode(IGN_2_PIN, INPUT);
  pinMode(IGN_3_PIN, INPUT);
  pinMode(IGN_4_PIN, INPUT);
  pinMode(LAMBDA_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(FORCE_PIN, INPUT);

  // Attach the timer interrupts
  Timer1.initialize(TIME_STEP * 1000000); // microseconds
  Timer1.attachInterrupt(measure); // measure the injection and ignition signals
  Timer1.attachInterrupt(update); // update the simulation state
}

// Loop function
void loop() {
  // Do nothing
}
