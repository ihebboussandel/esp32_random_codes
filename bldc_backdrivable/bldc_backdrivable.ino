#include <SimpleFOC.h>

// --- AS5600 I2C sensor ---
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

// --- Motor and Driver ---
BLDCMotor motor = BLDCMotor(7);                // 7 pole pairs (adjust if needed)
BLDCDriver3PWM driver = BLDCDriver3PWM(4, 5, 0, 16);

// --- Motion variables ---
float home_angle = 0;              // stored home position
float target_angle = 0;            // angle we command
float angles[2] = {0.0, 90 * M_PI/180.0f};  // back and forth between 0 and +1.57 rad
int current_index = 0;

unsigned long last_switch_time = 0;
const unsigned long switch_interval = 1000;  // change position every 1 second (you can change)


// --- Compliance / manual interaction settings ---
const float manual_threshold = 0.08f;   // radians — if sensor deviates more than this, assume manual move
const unsigned long release_delay = 200; // ms — must be still this long before returning
const float still_velocity_threshold = 0.002f; // rad/ms approx

// Gains for soft vs hold
const float soft_P = 0.05f;   // very soft while being moved by hand
const float hold_P = 1.0f;    // P while holding position (still gentle)

// Voltage limits
const float hold_voltage_limit = 4.0f;   // torque while holding
const float free_voltage_limit = 1.0f;   // low torque while letting user move it

// internal state
bool manual_mode = false;
unsigned long manual_detect_time = 0;
unsigned long release_time = 0;
float last_sensor_angle = 0;
unsigned long last_time = 0;


float angleDiff(float a, float b){
  // returns wrapped difference a - b in [-PI, PI]
  float d = a - b;
  while(d > M_PI) d -= 2*M_PI;
  while(d < -M_PI) d += 2*M_PI;
  return d;
}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  Serial.println(F("Initializing..."));

  // --- Initialize sensor ---
  I2Cone.begin(14, 2, 400000); // SDA=14, SCL=2
  sensor.init(&I2Cone);
  motor.linkSensor(&sensor);

  // --- Driver setup ---
  driver.voltage_power_supply = 12;
  driver.voltage_limit = hold_voltage_limit; // initial
  driver.init();
  motor.linkDriver(&driver);

  // --- Control setup ---
  motor.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // --- PID tuning (velocity + angle) ---
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 0.5f;
  motor.LPF_velocity.Tf = 0.1f;

  motor.P_angle.P = hold_P;
  motor.P_angle.I = 0;
  motor.velocity_limit = 100;

  // --- Monitoring ---
  motor.useMonitoring(Serial);

  // --- Initialize motor and FOC ---
  motor.init();
  motor.initFOC();

  // initial positions
  home_angle = sensor.getAngle();        // set current as home
  target_angle = home_angle;
  last_sensor_angle = home_angle;
  last_time = millis();

  Serial.println(F("Motor ready."));
  Serial.println(F("You can turn it by hand; it will return to home when released."));
}

void loop() {
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f; // seconds
  if (dt <= 0) dt = 0.001f;

  // Core FOC loop
  motor.loopFOC();

  // Read sensor angle
  float sensor_ang = sensor.getAngle();

  // compute position error relative to commanded target
  float err = angleDiff(sensor_ang, target_angle);
  float abs_err = fabs(err);

  // approximate angular velocity (rad/s)
  float ang_vel = angleDiff(sensor_ang, last_sensor_angle) / dt; // rad/s
  float abs_ang_vel = fabs(ang_vel);

  // --- Detect manual interaction ---
  if (abs_err > manual_threshold) {
    // user likely turning it -> enter manual mode
    if (!manual_mode) {
      manual_mode = true;
      manual_detect_time = now;
      Serial.println(F("[manual] engaged"));
    }
    // while manual: make controller very soft / low torque
    motor.P_angle.P = soft_P;
    driver.voltage_limit = free_voltage_limit;
  } else {
    // error small: possibly released / stable
    if (manual_mode) {
      // if it was manual and now returned near a position, start release timer
      if (abs_ang_vel < (still_velocity_threshold * 1000.0f)) { // small velocity -> considered stopped
        if (release_time == 0) release_time = now;
        // if still for long enough -> exit manual and go back to home
        if (now - release_time > release_delay) {
          manual_mode = false;
          release_time = 0;
          Serial.println(F("[manual] released — returning to home"));
          // command a smooth return to home
          // We'll keep target_angle = home_angle and motor.move will bring it back
          target_angle = home_angle;
          // restore hold gains & voltage
          motor.P_angle.P = hold_P;
          driver.voltage_limit = hold_voltage_limit;
        }
      } else {
        // moving still, reset release timer
        release_time = 0;
      }
    } else {
      // not in manual mode: maintain normal holding behaviour
      motor.P_angle.P = hold_P;
      driver.voltage_limit = hold_voltage_limit;
    }
  }

  // Apply the commanded target to the motor
  motor.move(target_angle);

  // Optional: allow toggling home with interval (your original behavior)
  if (millis() - last_switch_time > switch_interval) {
    // toggle example target between home and another angle (still gentle)
    current_index = 1 - current_index;
    // compute commanded target relative to home
    float commanded = home_angle + angles[current_index];
    // set new commanded target only when NOT in manual_mode (to avoid fighting the user)
    if (!manual_mode) {
      target_angle = commanded;
      home_angle = commanded; // update home if you want the new position to become home
      Serial.print(F("→ Moving to commanded angle: "));
      Serial.println(target_angle, 3);
    } else {
      Serial.println(F("Skipping auto-move while manual interaction active."));
    }
    last_switch_time = millis();
  }

  // update last values
  last_sensor_angle = sensor_ang;
  last_time = now;

  // small delay to avoid flooding Serial
  delay(2);
}
