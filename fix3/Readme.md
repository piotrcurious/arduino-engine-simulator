Here’s a detailed README for your engine simulation project based on the code we’ve refined together:


---

Engine Simulation with Flywheel Dynamics, Combustion Model, and Crank-Cylinder Mapping

This project implements a simplified internal combustion engine simulation on Arduino hardware.
It models flywheel dynamics, combustion torque, air-fuel mixing, crankshaft-to-cylinder mapping, and includes provisions for starting the engine via an external button.

The simulation outputs a lambda (air-fuel ratio) signal and camshaft position signal, while allowing throttle and load control via analog inputs.


---

✨ Features

Flywheel Dynamics

Models inertia and torque effects of the rotating mass.

Includes viscous + Coulomb friction model to prevent RPM runaway.


Combustion Model

Per-cylinder calculation of torque contribution.

Supports injector pulsewidth measurement for fuel mass calculation.

Airflow determined by throttle plate and ambient air density.

Stoichiometric AFR-based lambda calculation.


Crankshaft & Camshaft Simulation

Flywheel angle tracked in real time.

Camshaft angle derived at half crankshaft speed.

Cam wheel types selectable for testing (3 patterns).

Proper crank angle–to–cylinder mapping for 4-stroke cycle.


Starting Mechanism

External Start Button applies starter torque to spin up the engine.

When RPM passes a threshold, engine continues running via combustion torque.


Torque Model

Throttle torque + combustion torque − load torque − friction torque.


Lambda Output

Lambda value calculated and scaled to PWM (0–255).


Inputs

Throttle (potentiometer on A0).

Load/Force (sensor on A1).

Injectors & Ignition signals (digital).

Start button.


Outputs

Camshaft position pin (digital).

Lambda pin (PWM).




---

⚙️ Hardware Setup

Inputs

Throttle → Potentiometer connected to A0.

Force/Load → Sensor or potentiometer on A1.

Injectors → Digital inputs 3–6.

Ignition → Digital inputs 7–10.

Start Button → Digital input 12 (active HIGH).


Outputs

Camshaft Position → Digital output 2.

Lambda → PWM output 11.



---

🔧 Constants & Parameters

Flywheel

Mass = 10.0 kg

Radius = 0.1 m

Inertia = M * R²

Time Step = 1 ms


Engine Geometry

Bore = 67.1 mm

Stroke = 70.6 mm

Displacement per cylinder ≈ 0.25 L (approx. 1.0 L engine total)


Fuel System

Injector Flow Rate = 0.0002 L/s

Stoichiometric AFR = 14.7:1


Combustion

Ignition angle range: -10° to -30°

Burn rate scaled by octane rating

Pressure scale = 1e5 Pa


Friction

Modeled as:

friction_torque = viscous_coeff * flywheel_speed + coulomb_coeff * sign(flywheel_speed)



---

🚦 Crank-Cylinder Mapping

For a 4-cylinder 4-stroke engine, firing order 1-3-4-2:

Cylinder 1 fires at 0° crank

Cylinder 3 fires at 180° crank

Cylinder 4 fires at 360° crank

Cylinder 2 fires at 540° crank


Each cylinder completes:

1. Intake


2. Compression


3. Combustion (power)


4. Exhaust



Mapping is implemented so that the correct cylinder is active depending on crank angle.


---

▶️ Operation

1. Upload the code to your Arduino.


2. Connect throttle, load, injectors, ignition signals, and start button.


3. Press the start button:

Starter torque spins the flywheel.

If ignition & injection are present, engine sustains itself.



4. Observe:

Lambda signal (PWM on pin 11).

Camshaft sensor signal (pin 2).

Engine behavior changes with throttle and load inputs.





---

📊 Outputs & Logging

lambda (PWM 0–255) → Represents AFR (scaled).

cam_pos_state (digital HIGH/LOW) → Camshaft position.

Internal variables (flywheel_speed, flywheel_angle, torque) can be logged via Serial.print() for debugging (disabled in ISR by default for speed).



---

🚀 Future Improvements

Add intake manifold dynamics (pressure buildup).

More accurate combustion phasing using ignition angle.

Knock detection model.

Closed-loop lambda correction (simulated ECU feedback).

Multiple cam profiles.



---

🛠️ Dependencies

Arduino IDE

TimerOne Library



---

📌 Pinout Summary

Pin	Function	Type

2	Cam Position Sensor	Output
3–6	Injectors (1–4)	Input
7–10	Ignition (1–4)	Input
11	Lambda (PWM)	Output
A0	Throttle Input	Input
A1	Load/Force Input	Input
12	Start Button	Input



---

Would you like me to also draw a system diagram (schematic-style) showing how all these pins connect (engine, throttle, injectors, ignition, start button), so the README has a clear wiring reference?

