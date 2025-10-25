#include <SimpleFOC.h>

// --- AS5600 I2C sensor ---
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

// --- Motor and Driver ---
BLDCMotor motor = BLDCMotor(7);                // 7 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(4, 5, 0, 16);

// --- Motion variables ---
float home_angle = 0;
float target_angle = 0;
float angles[2] = {0.0, 90 * M_PI / 180.0f};
int current_index = 0;

unsigned long last_switch_time = 0;
const unsigned long switch_interval = 1000;

// --- Compliance / manual interaction settings ---
const float manual_threshold = 0.08f;   // radians deviation to detect manual move
const unsigned long release_delay = 200;
const float still_velocity_threshold = 0.002f; // rad/ms

// --- Gains and voltage limits ---
const float soft_P = 0.05f;   // very soft while moving manually
const float hold_P = 1.0f;    // moderate hold
const float strong_P = 6.0f;  // strong when returning to home (higher torque for geared motor)

const float free_voltage_limit = 1.0f;     // low torque while moving manually
const float hold_voltage_limit = 5.0f;     // moderate torque when holding
const float strong_voltage_limit = 9.0f;   // high torque when returning

// --- Timing and ramp control ---
const unsigned long return_ramp_time = 300; // ms to ramp up torque when returning

// --- Internal state ---
bool manual_mode = false;
bool returning_to_home = false;
unsigned long manual_detect_time = 0;
unsigned long release_time = 0;
unsigned long return_start_time = 0;
float last_sensor_angle = 0;
unsigned long last_time = 0;

// Helper for smooth ramp interpolation
float lerpF(float a, float b, float t) {
  if (t <= 0) return a;
  if (t >= 1) return b;
  return a + (b - a) * t;
}

// Helper for angle difference wrapping
float angleDiff(float a, float b) {
  float d = a - b;
  while (d > M_PI) d -= 2 * M_PI;
  while (d < -M_PI) d += 2 * M_PI;
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
  driver.voltage_limit = hold_voltage_limit;
  driver.init();
  motor.linkDriver(&driver);

  // --- Control setup ---
  motor.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // --- PID tuning ---
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 0.5f;
  motor.LPF_velocity.Tf = 0.1f;

  motor.P_angle.P = hold_P;
  motor.P_angle.I = 0;
  motor.velocity_limit = 150;

  motor.useMonitoring(Serial);

  // --- Initialize motor and FOC ---
  motor.init();
  motor.initFOC();

  home_angle = sensor.getAngle();
  target_angle = home_angle;
  last_sensor_angle = home_angle;
  last_time = millis();

  Serial.println(F("Motor ready. Turn it by hand; it will come back stronger to home."));
}

void loop() {
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  if (dt <= 0) dt = 0.001f;

  // Core FOC loop
  motor.loopFOC();

  float sensor_ang = sensor.getAngle();
  float err = angleDiff(sensor_ang, target_angle);
  float abs_err = fabs(err);

  float ang_vel = angleDiff(sensor_ang, last_sensor_angle) / dt;
  float abs_ang_vel = fabs(ang_vel);

  // --- Detect manual movement ---
  if (abs_err > manual_threshold) {
    if (!manual_mode) {
      manual_mode = true;
      returning_to_home = false;
      manual_detect_time = now;
      Serial.println(F("[manual] engaged"));
    }

    // While manually moving, make it soft
    motor.P_angle.P = soft_P;
    driver.voltage_limit = free_voltage_limit;

  } else {
    // Possibly released
    if (manual_mode) {
      if (abs_ang_vel < (still_velocity_threshold * 1000.0f)) {
        if (release_time == 0) release_time = now;

        // If stable long enough, start strong return
        if (now - release_time > release_delay && !returning_to_home) {
          manual_mode = false;
          returning_to_home = true;
          return_start_time = now;

          target_angle = home_angle;

          Serial.println(F("[manual] released — returning to home (strong mode)"));

          // Boost velocity control for the return
          motor.PID_velocity.P = 0.12f;
          motor.velocity_limit = 200;
        }
      } else {
        release_time = 0;
      }
    }
  }

  // --- Handle return-to-home ramp ---
  if (returning_to_home) {
    float t = float(now - return_start_time) / return_ramp_time;
    t = constrain(t, 0.0f, 1.0f);

    // Smooth ramp from hold to strong
    motor.P_angle.P = lerpF(hold_P, strong_P, t);
    driver.voltage_limit = lerpF(hold_voltage_limit, strong_voltage_limit, t);

    // When nearly at home, stop ramping
    if (t >= 1.0f && fabs(angleDiff(sensor_ang, home_angle)) < 0.02f) {
      returning_to_home = false;
      Serial.println(F("[return] arrived — holding position"));
      motor.P_angle.P = strong_P;
      driver.voltage_limit = strong_voltage_limit;
    }
  }

  // Apply control
  motor.move(target_angle);

  // --- Optional automatic toggle (still in your code) ---
  if (millis() - last_switch_time > switch_interval) {
    current_index = 1 - current_index;
    float commanded = home_angle + angles[current_index];
    if (!manual_mode && !returning_to_home) {
      target_angle = commanded;
      home_angle = commanded;
      Serial.print(F("→ Moving to commanded angle: "));
      Serial.println(target_angle, 3);
    } else {
      Serial.println(F("Skipping auto-move (manual/return active)"));
    }
    last_switch_time = millis();
  }

  last_sensor_angle = sensor_ang;
  last_time = now;
  delay(2);
}
