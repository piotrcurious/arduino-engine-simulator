
// Arduino code for car engine simulator
// Simulates 4 cylinder engine
// Simulates signals like cam position for ECU like Speeduino
// Emulates combustion by reading signals from injector and ignition driving pins of Speeduino
// Computes lambda signal sent to Speeduino based on combustion parameters
// Assumes potentiometer connected to A0 is throttle
// A1 analog input defines force applied to simulated flywheel
// Above 512 is positive force, below 512 is negative force

// Define pins for sensors and actuators
#define THROTTLE_PIN A0 // Potentiometer for throttle
#define FORCE_PIN A1 // Analog input for force on flywheel
#define CAM_PIN 2 // Digital output for cam position signal
#define INJ1_PIN 3 // Digital input for injector 1 signal
#define INJ2_PIN 4 // Digital input for injector 2 signal
#define INJ3_PIN 5 // Digital input for injector 3 signal
#define INJ4_PIN 6 // Digital input for injector 4 signal
#define IGN1_PIN 7 // Digital input for ignition 1 signal
#define IGN2_PIN 8 // Digital input for ignition 2 signal
#define IGN3_PIN 9 // Digital input for ignition 3 signal
#define IGN4_PIN 10 // Digital input for ignition 4 signal
#define LAMBDA_PIN A2 // Analog output for lambda signal

// Define constants for engine parameters
#define CYLINDERS 4 // Number of cylinders
#define CRANK_DEG_PER_REV 720 // Crankshaft degrees per revolution
#define CAM_DEG_PER_REV 360 // Camshaft degrees per revolution
#define CRANK_DEG_PER_CYL CRANK_DEG_PER_REV / CYLINDERS // Crankshaft degrees per cylinder firing
#define CAM_DEG_PER_CYL CAM_DEG_PER_REV / CYLINDERS // Camshaft degrees per cylinder firing
#define MAX_RPM 6000 // Maximum engine RPM
#define MIN_RPM 500 // Minimum engine RPM
#define IDLE_RPM 800 // Idle engine RPM
#define MAX_THROTTLE_ANGLE 90 // Maximum throttle angle in degrees
#define MIN_THROTTLE_ANGLE 0 // Minimum throttle angle in degrees
#define THROTTLE_MAP_SIZE 11 // Size of throttle map array
#define RPM_MAP_SIZE 11 // Size of RPM map array

// Define variables for engine state
int throttleAngle = MIN_THROTTLE_ANGLE; // Throttle angle in degrees
int rpm = IDLE_RPM; // Engine RPM
int crankDeg = 0; // Crankshaft angle in degrees
int camDeg = 0; // Camshaft angle in degrees

// Define arrays for throttle map and RPM map (example values, adjust as needed)
int throttleMap[THROTTLE_MAP_SIZE] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90}; // Throttle angle values in degrees
int rpmMap[RPM_MAP_SIZE] = {500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500}; // RPM values

// Define arrays for injector and ignition signals (example values, adjust as needed)
int inj1Signal[CRANK_DEG_PER_REV] = {0}; // Injector 1 signal array
int inj2Signal[CRANK_DEG_PER_REV] = {0}; // Injector 2 signal array
int inj3Signal[CRANK_DEG_PER_REV] = {0}; // Injector 3 signal array
int inj4Signal[CRANK_DEG_PER_REV] = {0}; // Injector 4 signal array
int ign1Signal[CRANK_DEG_PER_REV] = {0}; // Ignition 1 signal array
int ign2Signal[CRANK_DEG_PER_REV] = {0}; // Ignition 2 signal array
int ign3Signal[CRANK_DEG_PER_REV] = {0}; // Ignition 3 signal array
int ign4Signal[CRANK_DEG_PER_REV] = {0}; // Ignition 4 signal array

// Define variables for combustion parameters (example values, adjust as needed)
float airFuelRatio = 14.7; // Air-fuel ratio
float stoichAirFuelRatio = 14.7; // Stoichiometric air-fuel ratio
float lambda = airFuelRatio / stoichAirFuelRatio; // Lambda value
float lambdaVoltage = map(lambda, 0.6, 1.6, 0.1, 5); // Lambda voltage value

// Define variables for timing parameters
unsigned long prevTime = 0; // Previous time in milliseconds
unsigned long currTime = 0; // Current time in milliseconds
unsigned long deltaTime = 0; // Delta time in milliseconds
unsigned long crankInterval = 0; // Crankshaft interval in milliseconds
unsigned long camInterval = 0; // Camshaft interval in milliseconds

void setup() {
  
  // Set pin modes for sensors and actuators
  pinMode(THROTTLE_PIN, INPUT); // Potentiometer for throttle
  pinMode(FORCE_PIN, INPUT); // Analog input for force on flywheel
  pinMode(CAM_PIN, OUTPUT); // Digital output for cam position signal
  pinMode(INJ1_PIN, INPUT); // Digital input for injector 1 signal
  pinMode(INJ2_PIN, INPUT); // Digital input for injector 2 signal
  pinMode(INJ3_PIN, INPUT); // Digital input for injector 3 signal
  pinMode(INJ4_PIN, INPUT); // Digital input for injector 4 signal
  pinMode(IGN1_PIN, INPUT); // Digital input for ignition 1 signal
  pinMode(IGN2_PIN, INPUT); // Digital input for ignition 2 signal
  pinMode(IGN3_PIN, INPUT); // Digital input for ignition 3 signal
  pinMode(IGN4_PIN, INPUT); // Digital input for ignition 4 signal
}

void loop() {
  
   currTime = millis(); // Get current time in milliseconds
   
   if (currTime - prevTime >= deltaTime) { // Check if delta time has elapsed
    
    prevTime = currTime; // Update previous time
    
    readThrottle(); // Read throttle angle from potentiometer
    
    readForce(); // Read force on flywheel from analog input
    
    calculateRPM(); // Calculate engine RPM based on throttle and force
    
    calculateCrankInterval(); // Calculate crankshaft interval based on RPM
    
    calculateCamInterval(); // Calculate camshaft interval based on RPM
    
    updateCrankDeg(); // Update crankshaft angle
    
    updateCamDeg(); // Update camshaft angle
    
    generateCamSignal(); // Generate cam position signal
    
    readInjSignals(); // Read injector signals from Speeduino
    
    readIgnSignals(); // Read ignition signals from Speeduino
    
    calculateLambda(); // Calculate lambda value based on combustion parameters
    
    generateLambdaSignal(); // Generate lambda signal to Speeduino
    
   }
}

// Function to read throttle angle from potentiometer
void readThrottle() {
  
   int throttleValue = analogRead(THROTTLE_PIN); // Read analog value from potentiometer
   
   throttleAngle = map(throttleValue, MIN_THROTTLE_ANGLE, 0, MAX_THROTTLE_ANGLE, 1023); // Map analog value to throttle angle in degrees
}

// Function to read force on flywheel from analog input
void readForce() {
  
   int forceValue = analogRead(FORCE_PIN); // Read analog value from force input
   
   if (forceValue > 512) { // Positive force applied to flywheel
    
    rpm += map(forceValue, 512, 1023, 0, 100); // Increase RPM based on force value
    
   } else if (forceValue < 512) { // Negative force applied to flywheel
    
    rpm -= map(forceValue, 0, 511, 0, 100); // Decrease RPM based on force value
    
   }
   
   // Constrain RPM to minimum and maximum values
   rpm = constrain(rpm, MIN_RPM, MAX_RPM);
}

// Function to calculate engine RPM based on throttle and force
void calculateRPM() {
  
   // Calculate RPM based on throttle map and RPM map (example formula, adjust as needed)
   rpm = map(throttleAngle, MIN_THROTTLE_ANGLE, MAX_THROTTLE_ANGLE, rpmMap[0], rpmMap[RPM_MAP_SIZE - 1]);
}

// Function to calculate crankshaft interval based on RPM
void calculateCrankInterval() {
  
   // Calculate crankshaft interval in milliseconds (example formula, adjust as needed)
   crankInterval = (60000 / rpm) / CRANK_DEG_PER_REV;
}

// Function to calculate camshaft interval based on RPM
void calculateCamInterval() {
  
   // Calculate camshaft interval in milliseconds (example formula, adjust as needed)
   camInterval = (60000 / rpm) / CAM_DEG_PER_REV;
}

// Function to update crankshaft angle
void updateCrankDeg() {
  
   // Increment crankshaft angle by one degree
   crankDeg++;
   
   // Reset crankshaft angle if it reaches one revolution
   if (crankDeg >= CRANK_DEG_PER_REV) {
    
    crankDeg = 0;
    
   }
}

// Function to update camshaft angle
void updateCamDeg() {
  
   // Increment camshaft angle by one degree
   camDeg++;
   
   // Reset camshaft angle if it reaches one revolution
   if (camDeg >= CAM_DEG_PER_REV) {
    
    camDeg = 0;
    
   }
}

// Function to generate cam position signal
void generateCamSignal() {
  
   // Generate a square wave signal with 50% duty cycle and frequency equal to half of RPM (example formula, adjust as needed)
   digitalWrite(CAM_PIN, (camDeg % CAM_DEG_PER_CYL) < (CAM_DEG_PER_CYL / 2));
}

// Function to read injector signals from Speeduino
void readInjSignals() {
  
   // Read digital values from injector pins
   int inj1Value = digitalRead(INJ1_PIN);
   int inj2Value = digitalRead(INJ2_PIN);
   int inj3Value = digitalRead(INJ3_PIN);
   int inj4Value = digitalRead(INJ4_PIN);
   
   // Store injector values in corresponding arrays at current crankshaft angle index
   inj1Signal[crankDeg] = inj1Value;
   inj2Signal[crankDeg] = inj2Value;
   inj3Signal[crankDeg] = inj3Value;
   inj4Signal[crankDeg] = inj4Value;
}

// Function to read ignition signals from Speeduino
void readIgnSignals() {
  
   // Read digital values from ignition pins
   int ign1Value = digitalRead(IGN1_PIN);
   int ign2Value = digitalRead(IGN2_PIN);
   int ign3Value = digitalRead(IGN3_PIN);
   int ign4Value = digitalRead(IGN4_PIN);
   
   // Store ignition values in corresponding arrays at current crankshaft angle index
   ign1Signal[crankDeg] = ign1Value;
   ign2Signal[crankDeg] = ign2Value;
   ign3Signal[crankDeg] = ign3Value;
   ign4Signal[crankDeg] = ign4Value;
}

// Function to calculate lambda value based on combustion parameters
void calculateLambda() {
  
   // Calculate lambda value based on air-fuel ratio and stoichiometric air-fuel ratio (example formula, adjust as needed)
   lambda = airFuelRatio / stoichAirFuelRatio;
}

// Function to generate lambda signal to Speeduino
void generateLambdaSignal() {
  
   // Calculate lambda voltage based on lambda value (example formula, adjust as needed)
   lambdaVoltage = map(lambda, 0.6, 1.6, 0.1, 5);
   
   // Write analog value to lambda pin
   analogWrite(LAMBDA_PIN, lambdaVoltage);
}

//Source: Conversation with Bing, 4/29/2023
//(1) piotrcurious/arduino-engine-simulator - Github. https://github.com/piotrcurious/arduino-engine-simulator.
//(2) Build a Miniature Self-Driving Car | Science Project. https://www.sciencebuddies.org/science-fair-projects/project-ideas/Robotics_p042/robotics/arduino-self-driving-car.
//(3) Arduino Self-Driving Car : 10 Steps (with Pictures) - Instructables. https://www.instructables.com/Arduino-Self-Driving-Car/.
//(4) Home | Speeduino - Open, easy engine management. https://www.speeduino.com/.
//(5) Downloads | Speeduino - Open, easy engine management. https://speeduino.com/home/support/downloads.
//(6) Speeduino Manual | Speeduino Manual. https://wiki.speeduino.com/.
