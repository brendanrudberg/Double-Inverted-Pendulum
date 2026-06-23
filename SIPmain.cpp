#include <Arduino.h>
#include <math.h>
#include<ESP32Encoder.h>

#define STEP_PIN 12
#define DIR_PIN 14
#define ENABLE_PIN 27
#define LED_PIN  2 
#define LIMIT_L 25 //was 32
#define LIMIT_R 33
#define ENCODER_X_A 16
#define ENCODER_X_B 17
#define ENCODER_T_A 21 //was 34
#define ENCODER_T_B 22 //was 35

// ============================================================================
//  Hardware globals
// ============================================================================
const float m = 0.08; // pendulum mass (kg)
const float M = 0.4;  // cart mass (kg), measure on hardware
const float g = 9.81; // gravity (m/s^2)
const float l = 0.1;  // 1/2 rod length (m)
const float I = m * (2.0 * l) * (2.0 * l) / 12.0; // Inertia
const float b_friction = 0.3;
const float alpha = 0.5;
float v_filt = 0.0;
const int microstepping = 2;
unsigned long before = 0.0;

const float r = 0.2; // gain factor
const float k = 2.0; // centering spring factor
const float b = 1.5;  // spring damping factor
const float PULLEY_RADIUS = 0.0159155;  // Default pulley radius (10mm)

const float PPR = 5120.0;
const float CPR = 4 * PPR;
float circumference = 2.0 * 3.14159265358979 * PULLEY_RADIUS;
float tick_dist = circumference / CPR;
const double RAD_PER_TICK = (2.0 * 3.141592653589793) / CPR;

bool L_pressed = false;
bool R_pressed = false;
bool hardstop = false;
unsigned long lastTelemetryTime = 0;

ESP32Encoder encoderX;
ESP32Encoder encoderT; 

//Stepper motor constants
unsigned long last_step_time = 0;
float step_period_us = 625.0;
float v_cmd = 0.0;
const float V_MAX    = 0.45;   // m/s, max commanded cart velocity
const float A_MAX = 3.0; 
const float V_DEADBAND = 0.002; // m/s, below which no step is issued
const float STEP_DIST = circumference / (200.0*microstepping); // meters per full step

// Pendulum angle calibration (Fix 3). 0 = hanging down, PI = upright.
float THETA_DOWN_OFFSET = 0.0; // raw getMechanicalAngle() when pendulum hangs still, we measure this on hardware
const float THETA_DIR   = 1.0; // +1 or -1

static inline double wrap_2pi(double a) {
  while (a < 0)        a += TWO_PI;
  while (a >= TWO_PI)  a -= TWO_PI;
  return a;
}
static inline double wrap_pi(double a) { // shortest-arc, (-PI, PI]
  while (a <= -PI) a += TWO_PI;
  while (a >   PI) a -= TWO_PI;
  return a;
}

// --- Damping Injection variables ---
float omega_history[1000] = {0}; 
int history_idx = 0;
const float DAMPING_GAIN = 0.5; // Tune this to "eat" the excess momentum

// ============================================================================
//  Operating mode
// ============================================================================
enum Mode {
  MODE_SINE_TEST       = 0, // open-loop sinusoidal motor test (no pendulum control)
  MODE_SWINGUP         = 1, // closed-loop swing-up (Astrom-Furuta energy method)
  MODE_BALANCE_LQR     = 2, // closed-loop balancing about upright (LQR)
  MODE_SWINGUP_BALANCE = 3  // swing-up, then hand off to LQR near upright
};
int mode = MODE_SWINGUP_BALANCE;
const bool USE_KALMAN = true;

// LQR controller + target state at upright [x, v, theta, omega].
double target_state[4] = {0.0, 0.0, PI, 0.0}; // [x, v, theta, omega], theta=PI = upright

// MODE_SWINGUP_BALANCE: hand off swing-up -> LQR once within this angle of upright.
const double BALANCE_ANGLE_TOL = 0.30; // rad (~17 deg), tune later

double last_x = 0.0;   // encoder1 (motor)
double raw_velocity = 0.0;
double last_theta = 0.0;    // encoder2 (pendulum)
double raw_omega = 0.0;
unsigned long last_time = 0;

// ============================================================================
//  Kalman filter
// ============================================================================
struct KalmanFilter {
  double angle = 0;
  double vel = 0;
  double Q_angle, Q_vel, R;
  double P[2][2] = {{1, 0}, {0, 1}};

  void init(double q_angle, double q_vel, double r) {
    Q_angle = q_angle;
    Q_vel = q_vel;
    R = r;
  }

  double update(double z, double dt) {
    angle += vel * dt;
    angle = z + wrap_pi(angle - z);

    // Uncertainty matrix
    double p00 = P[0][0] + dt*P[1][0] + dt*P[0][1] + dt*dt*P[1][1] + Q_angle*dt;
    double p01 = P[0][1] + dt*P[1][1];
    double p10 = P[1][0] + dt*P[1][1];
    double p11 = P[1][1] + Q_vel*dt;

    P[0][0] = p00;
    P[0][1] = p01;
    P[1][0] = p10;
    P[1][1] = p11;

    double innov = wrap_pi(z - angle);  // Innovation (how wrong we were)

    double S = P[0][0] + R;    // Total uncertainty

    double K0 = P[0][0] / S;   // Angle gain
    double K1 = P[1][0] / S;   // Velocity gain

    angle += K0 * innov;       // nudging angle toward measurement
    angle = z + wrap_pi(angle - z);
    vel   += K1 * innov;       // nudging velocity toward measurement

    double new_p00 = (1.0 - K0) * P[0][0];
    double new_p01 = (1.0 - K0) * P[0][1];
    double new_p10 = P[1][0] - K1 * P[0][0];
    double new_p11 = P[1][1] - K1 * P[0][1];

    P[0][0] = new_p00;
    P[0][1] = new_p01;
    P[1][0] = new_p10;
    P[1][1] = new_p11;

    return angle;
  }
};

KalmanFilter kf1, kf2; // kf1 = motor angle; kf2/kf3 = joint angles

// ============================================================================
//  LQR controller
// ============================================================================
class LQR {
public:
  LQR() {
    for (int i = 0; i < STATE_SIZE; ++i) {
      K[i] = 0.0;
    }
  }

  void set_K(const double K_gains[4]) {
    for (int i = 0; i < STATE_SIZE; ++i) {
      K[i] = K_gains[i];
    }
  }

  double LQR_step(const double current[4], const double target[4]) {
    // Unpack current state
    double x     = current[0]; // Cart Position
    double v     = current[1]; // Cart Velocity
    double theta = current[2]; // Pendulum Angle
    double omega = current[3]; // Pendulum Angular Velocity

    // Unpack target state
    double target_x     = target[0];
    double target_v     = target[1];
    double target_theta = target[2];
    double target_omega = target[3];

    // Error calculation
    double x_error     = target_x - x;
    double v_error     = target_v - v;
    double theta_error = wrap_pi(target_theta - theta);
    double omega_error = target_omega - omega;

    // LQR control law: u = K * (target - current)  ==  -K * (current - target)
    double control_input = K[0] * x_error + K[1] * v_error
                         + K[2] * theta_error + K[3] * omega_error;

    return control_input;
  }

private:
  static const int STATE_SIZE = 4;
  double K[STATE_SIZE];
};

LQR lqr;

// ============================================================================
//  Swing-up (Astrom-Furuta energy method)
// ============================================================================
double swing_step(const double state[4]) {
  double x     = state[0];
  double v     = state[1];
  double theta = state[2]; 
  double omega = state[3]; 

  double E_max = m*g*l;
  double KE = 0.5*I*omega*omega;
  double PE = -m*g*l*cos(theta); 
  double E = KE + PE;
  
  // Calculate the energy error
  double DE = E - E_max;

  // Astrom-Furuta method:
  double sign = (omega * cos(theta)) > 0 ? 1.0 : -1.0;
  double F = r * DE * sign;

  // centering force:
  double s = - k*x - b*v;

  return -F - s;
}

// ============================================================================
//  Calibrate X-Axis (For Calibration)
// ============================================================================

void calibrateXAxis() {
  // Wake up driver coils
  digitalWrite(ENABLE_PIN, LOW); 
  
  // ----------------------------------------------------
  // STEP 1: Move Right until Right Limit Switch is Hit
  // ----------------------------------------------------
  Serial.println(" -> Seeking Right Endstop...");
  digitalWrite(DIR_PIN, LOW); // Set direction toward Right switch
  
  while (digitalRead(LIMIT_R) == LOW) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3300 / microstepping);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(3300 / microstepping);
    yield();
  }
  
  // Right switch hit! Stop pulses instantly and save position
  delay(200); // Tiny pause to allow any physical carriage bouncing to settle
  long right_limit_ticks = (long)encoderX.getCount();
  Serial.print(" -> Right Endstop Found at raw tick count: ");
  Serial.println(right_limit_ticks);

  // ----------------------------------------------------
  // STEP 2: Move Left until Left Limit Switch is Hit
  // ----------------------------------------------------
  Serial.println(" -> Seeking Left Endstop...");
  digitalWrite(DIR_PIN, HIGH); // Reverse direction toward Left switch
  
  while (digitalRead(LIMIT_L) == LOW) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3300 / microstepping);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(3300 / microstepping);
    yield();
  }
  
  // Left switch hit! Stop pulses instantly and save position
  delay(200); // Settle time
  long left_limit_ticks = (long)encoderX.getCount();
  Serial.print(" -> Left Endstop Found at raw tick count: ");
  Serial.println(left_limit_ticks);

  // ----------------------------------------------------
  // STEP 3: Mathematical Recalibration of the Center Point
  // ----------------------------------------------------
  // Midpoint formula: (Right + Left) / 2
  long center_point_ticks = (right_limit_ticks + left_limit_ticks) / 2;
  
  // Find where the carriage currently sits relative to our new center target
  long current_offset = left_limit_ticks - center_point_ticks;
  Serial.print(current_offset);
  // Force shift the hardware encoder counter register!
  encoderX.setCount(current_offset);

  // Calculate total track width in millimeters for debugging telemetry
  long total_travel_ticks = abs(right_limit_ticks - left_limit_ticks);
  float total_travel = (float)total_travel_ticks * tick_dist;

  Serial.println(" -> Coordinate Mapping Restructured!");
  Serial.print(" -> Total Physical Rail Travel Length: ");
  Serial.print(total_travel, 5);
  Serial.println(" m");
  Serial.println(" -> Center Point locked as 0.00 mm coordinate.");

  Serial.println(" -> Backing away from Left Switch to safe zone...");
  digitalWrite(DIR_PIN, LOW); // Set direction back toward the right/center

  L_pressed = false;
  R_pressed = false;
  hardstop = false;
  
  // Move away from the switch by taking 200 clean steps
  for (int i = 0; i < 200; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3000 / microstepping);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(3000 / microstepping);
  }

  // REVERTED: removed the friend's mid-calibration centering call. This was the regression.
  // moveToTarget exercises its own DIR signs (dormant in the working version) and an
  // unverified count-direction assumption; if the sign is wrong it drives into a limit,
  // aborts, and disables -> "fully stopped". Re-add deliberately only after bench-verifying
  // the DIR sign (see note below).
  // moveToTarget(0.0000, tick_dist * 1000.0, 0.0005);
  
  delay(200); // Let the system settle out completely before entering main loop
  Serial.print(" -> Final centered count: ");
  Serial.println((long)encoderX.getCount());
  Serial.println(" -> Safe zone achieved.");
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
}


// ============================================================================
//  Move To Target (For Calibration)
// ============================================================================

void moveToTarget(float target_m, float target_vel_m, float tolerance_m) {
  // 1. Calculate the required pulse timing based on the requested velocity
  // Step Frequency (Hz) = Velocity (m/s) / Distance per tick (m/tick)
  float steps_per_second = (target_vel_m / tick_dist);
  
  // Calculate the microsecond delay needed for half of the square wave cycle
  // Total period (µs) = 1,000,000 / steps_per_second. Delay = period / 2.
  unsigned long step_delay_us = (1000000.0 / steps_per_second) / 2.0;

  // Safety cap: Never allow the step delay to drop into unstable frequencies (<400µs)
  if (step_delay_us < 400) step_delay_us = 400; 

  Serial.print("Initiating Move to: "); Serial.print(target_m, 4); Serial.println(" m");
  
  // Enable the motor driver
  digitalWrite(ENABLE_PIN, LOW);

  // 2. Closed-Loop Execution Loop
  while (true) {
    // Read current physical position
    long current_ticks = (long)encoderX.getCount();
    float current_position_m = (float)current_ticks * tick_dist;

    // Calculate distance remaining to target
    float error = target_m - current_position_m;

    // Check if we have arrived within the allowed tolerance window
    if (abs(error) <= tolerance_m) {
      Serial.println("Target Destination Achieved within tolerance.");
      break; // Exit the motion loop safely
    }

    // Secondary safety check: Stop instantly if a limit switch is pressed (HIGH = pressed)
    if (digitalRead(LIMIT_L) == HIGH || digitalRead(LIMIT_R) == HIGH) {
      digitalWrite(ENABLE_PIN, HIGH); // Shut off motor power instantly
      Serial.println("MOTION ABORTED: Safety limit switch breached mid-travel!");
      break;
    }

    // 3. Set Direction based on the sign of the error.
    // NOTE: these two DIR levels must match the calibration convention
    // (DIR LOW = toward RIGHT, DIR HIGH = toward LEFT). VERIFY ON BENCH before
    // trusting this function: command a small move and confirm the cart goes
    // toward the target, not into a wall.
    if (error > 0) {
      digitalWrite(DIR_PIN, LOW); // Move toward right
    } else {
      digitalWrite(DIR_PIN, HIGH);  // Move toward left
    }

    // 4. Generate a single precise step pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(step_delay_us);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(step_delay_us);

    // Yield control to allow background ESP32 OS tasks and hardware pulse counters to process
    yield();
  }
}

// ============================================================================
//  setup()
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Cart encoder (ESP32Encoder) ---
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderX.attachFullQuad(ENCODER_X_A, ENCODER_X_B);
  encoderX.setCount(0);
  encoderT.attachFullQuad(ENCODER_T_A, ENCODER_T_B);
  encoderT.setCount(0);


  // --- Stepper driver pins ---
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // --- Limit switches ---
  pinMode(LIMIT_L, INPUT_PULLUP);
  pinMode(LIMIT_R, INPUT_PULLUP);

  // --- Enable the driver (Active LOW) ---
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);

  // --- LQR gains ---
  double K_gains[4] = {2.0, 4.0, -15.0, -0.3}; // {K_x, K_v, K_theta, K_omega} 1.0, 1.4856, -11.1477, -1.6336 | 0.7071, 1.2207, -10.3301, -1.5226
  lqr.set_K(K_gains);

  // --- Kalman filter init ---
  kf1.init(1.5, 1.5, 0.005);
  kf2.init(1.5, 1.5, 0.005);

  Serial.println("Setup complete.");

  // --- Calibration ---   
  Serial.println("--- Starting Automatic Boundary Calibration ---");
  calibrateXAxis(); // seek both ends, establish center

  Serial.println("--- Calibration Complete! Centering... ---");                  
  moveToTarget(0.0000, 0.00132680985f, 0.00005f);  // drive to center (achievable tolerance)

  // --- Zero out control-loop state ---
  encoderX.setCount(0);                              // center = 0
  last_x = (double)encoderX.getCount() * tick_dist;  // = 0
  v_cmd = 0.0;
  last_time = micros();                               // first dt is small, not the whole calibration duration
}

// ============================================================================
//  loop()
// ============================================================================
void loop() {
  // --- timing ---
  unsigned long now = micros();
  double dt = (now - last_time) * 1e-6;
  last_time = now;

  // --- Cart position (ESP32Encoder) ---
  long x_ticks = (long)encoderX.getCount();
  double raw_x = (double)x_ticks * tick_dist;

  if (dt > 1e-6) {
    raw_velocity = (raw_x - last_x) / dt;
    v_filt = alpha * v_filt + (1 - alpha) * raw_velocity;
  }
  last_x = raw_x;

  // --- Limit switches ---
  if (digitalRead(LIMIT_L) == HIGH) { //changed from low
    if (!L_pressed) {
      L_pressed = true;
      hardstop = true;
    }
  } else {
    L_pressed = false;
  }

  if (digitalRead(LIMIT_R) == HIGH) { //changed from low
    if (!R_pressed) {
      R_pressed = true;
      hardstop = true;
    }
  } else {
    R_pressed = false;
  }

  if (digitalRead(LIMIT_L) == LOW && digitalRead(LIMIT_R) == LOW) { //changed from high
    hardstop = false;
  }

  double raw_theta = wrap_2pi(THETA_DIR * ((double)encoderT.getCount() * TWO_PI / CPR - THETA_DOWN_OFFSET));

  if (dt > 1e-6) {
    raw_omega = wrap_pi(raw_theta - last_theta) / dt; // shortest-arc handles the 0/2PI seam
  }
  last_theta = raw_theta;

  // Keep kf2 warm whether or not we use its output.
  double kf_theta = kf2.update(raw_theta, dt);

  // Kalman filter on cart position.
  double kf_x = kf1.update(raw_x, dt);
  double x = USE_KALMAN ? kf_x : raw_x;
  double v = USE_KALMAN ? kf1.vel : v_filt;

  // --- Assemble the 4-state vector [x, v, theta, omega] ---
  double theta = USE_KALMAN ? kf_theta : raw_theta;
  double omega = USE_KALMAN ? kf2.vel  : raw_omega;
  double state[4] = {x, v, theta, omega};

  // --- Mode dispatch ---
  switch (mode) {
    case MODE_SINE_TEST: {
      static bool dir = true;
      const double sweep = 0.05;  // +/- 5 cm from center
      if (x > sweep)  dir = false;  // too far right, go left
      if (x < -sweep) dir = true;   // too far left, go right
      digitalWrite(DIR_PIN, dir ? LOW : HIGH);  // match your DIR convention
      step_period_us = 2625.0;
      break;
    }
 
    case MODE_SWINGUP: {
      double u = swing_step(state);
      float v_target = constrain(u / b_friction, -V_MAX, V_MAX);
      float dv_max = A_MAX * dt;                          // max change this iteration
      v_cmd += constrain(v_target - v_cmd, -dv_max, dv_max);
      if (fabs(v_cmd) > V_DEADBAND) {
        digitalWrite(DIR_PIN, v_cmd > 0 ? HIGH : LOW);
        step_period_us = (STEP_DIST / fabs(v_cmd)) * 1e6;
      } else {
        step_period_us = 1e9;
      }
      unsigned long now = millis();
      if (now - before > 100){
        before = now;
        Serial.println("step period: ");
        Serial.print(step_period_us);
        Serial.println(" v cmd: ");
        Serial.print(v_cmd);
      }
      break;
    }
 
    case MODE_BALANCE_LQR: {
        double u = lqr.LQR_step(state, target_state);
        float v_target = constrain(u / b_friction, -V_MAX, V_MAX);
        float dv_max = A_MAX * dt;                       // accel cap
        v_cmd += constrain(v_target - v_cmd, -dv_max, dv_max);
        if (fabs(v_cmd) > V_DEADBAND) {
          digitalWrite(DIR_PIN, v_cmd > 0 ? HIGH : LOW);
          step_period_us = (STEP_DIST / fabs(v_cmd)) * 1e6;
        } else {
          step_period_us = 1e9;
        }
        break;
      }
 
    case MODE_SWINGUP_BALANCE: {
      double u;
      
      // Update omega history (circular buffer)
      omega_history[history_idx] = omega;
      history_idx = (history_idx + 1) % 1000;
      
      float sum_omega = 0;
      for(int i=0; i<10; i++) sum_omega += omega_history[i];
      float avg_omega = sum_omega / 1000.0;

      // --- Periodic Telemetry for Tuning ---
      static unsigned long last_print = 0;
      if (millis() - last_print > 250) { // Prints every 250ms
        last_print = millis();
        
        Serial.print("Theta: "); Serial.print(theta);
        Serial.print(" | Omega: "); Serial.print(omega);
        Serial.print(" | AvgOmega: "); Serial.print(avg_omega);
        Serial.print(" | v_cmd: "); Serial.println(v_cmd);
      }
      
      bool angle_ok = fabs(wrap_pi(theta - PI)) < BALANCE_ANGLE_TOL;
      
      if (angle_ok) {
        // HAND-OFF: If we are close to upright, kill residual velocity to "catch" it
        // This forces the cart to match the pendulum's motion, settling the swing
        if (fabs(avg_omega) > 0.1) { 
           u = lqr.LQR_step(state, target_state) - (omega * DAMPING_GAIN);
        } else {
           u = lqr.LQR_step(state, target_state);
        }
      } else {
        // SWING-UP: Keep pumping
        u = swing_step(state);
      }
      
      float v_target = constrain(u / b_friction, -V_MAX, V_MAX);
      float dv_max = A_MAX * dt;
      v_cmd += constrain(v_target - v_cmd, -dv_max, dv_max);
      
      if (fabs(v_cmd) > V_DEADBAND) {
        digitalWrite(DIR_PIN, v_cmd > 0 ? HIGH : LOW);
        step_period_us = (STEP_DIST / fabs(v_cmd)) * 1e6;
      } else {
        step_period_us = 1e9;
      }
      break;
    }
  }

  if (!hardstop && step_period_us < 1e8) {
    unsigned long now_us = micros();
    if ((now_us - last_step_time) >= (unsigned long)step_period_us) {
      digitalWrite(STEP_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(STEP_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      last_step_time = now_us;
    }
  }
}
