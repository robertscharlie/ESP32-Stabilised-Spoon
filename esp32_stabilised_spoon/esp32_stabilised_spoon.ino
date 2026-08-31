/*
  Self-leveling spoon for tremor stabilization
  ESP32 + MPU6050 (I2C) + 2x MG90 servos (roll + pitch)

  Libraries needed (Arduino Library Manager):
    - "MPU6050" by Electronic Cats (or jrowberg's I2Cdevlib)
    - "ESP32Servo" by Kevin Harrington (standard Servo lib doesn't
      play well with ESP32's PWM peripheral)

  Behavior:
    - Hold the button: leveling is active, LED lights up.
    - Release the button: servos return to center, LED off.
    - On boot, the code auto-calibrates assuming the spoon is
      being held roughly level for the first ~1 second.
*/

#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

// ---- Pins ----
// All chosen from the LEFT-side pin column of a standard 30-pin
// DOIT ESP32 DevKit V1 board (VIN, GND, D32, D33, D25, D26, D13, D14).
// D34/D35/VP/VN on that same side are input-only and can't be used
// here; D12 is a boot-strapping pin and is skipped on purpose.
const int MPU_SDA = 25;
const int MPU_SCL = 26;
const int SERVO_PITCH_PIN = 33;   // forward/back correction
const int SERVO_ROLL_PIN  = 32;   // side-to-side correction
const int BUTTON_PIN = 13;
const int LED_PIN = 14;

MPU6050 mpu;
Servo servoPitch;
Servo servoRoll;

// ---- Filter state ----
float angleRoll = 0, anglePitch = 0;
unsigned long lastTime;

// ---- Servo tuning ----
const int SERVO_CENTER = 90;
const int SERVO_MAX_DEFLECTION = 90; // degrees; caps commanded angle to the servo's 0-180 range

// ---- Calibration offsets (set at boot) ----
float rollOffset = 0, pitchOffset = 0;

void calibrate() {
  // Average samples while assumed level, cancels out mounting offset
  // and any small full-scale error in the MPU6050.
  long sumAx = 0, sumAy = 0, sumAz = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    sumAx += ax; sumAy += ay; sumAz += az;
    delay(5);
  }
  float ax = sumAx / (float)N;
  float ay = sumAy / (float)N;
  float az = sumAz / (float)N;

  rollOffset  = atan2(ay, az) * 180.0 / PI;
  pitchOffset = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(MPU_SDA, MPU_SCL);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed — check wiring");
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  servoPitch.setPeriodHertz(50);
  servoPitch.attach(SERVO_PITCH_PIN, 500, 2400);
  servoRoll.setPeriodHertz(50);
  servoRoll.attach(SERVO_ROLL_PIN, 500, 2400);

  servoPitch.write(SERVO_CENTER);
  servoRoll.write(SERVO_CENTER);

  delay(300); // let servos settle before calibrating
  calibrate();

  lastTime = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Accelerometer-derived angle (absolute, but noisy)
  float accRoll  = atan2(ay, az) * 180.0 / PI;
  float accPitch = atan2(-ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;

  // Gyro rate in deg/s (MPU6050 default sensitivity: 131 LSB per deg/s)
  float gyroRollRate  = gx / 131.0;
  float gyroPitchRate = gy / 131.0;

  // Complementary filter: mostly trust integrated gyro (smooth, no
  // vibration noise), slowly correct drift toward the accelerometer.
  const float ALPHA = 0.98;
  angleRoll  = ALPHA * (angleRoll  + gyroRollRate  * dt) + (1 - ALPHA) * accRoll;
  anglePitch = ALPHA * (anglePitch + gyroPitchRate * dt) + (1 - ALPHA) * accPitch;

  float correctedRoll  = angleRoll  - rollOffset;
  float correctedPitch = anglePitch - pitchOffset;

  bool levelingOn = (digitalRead(BUTTON_PIN) == LOW); // pressed = LOW with pullup
  digitalWrite(LED_PIN, levelingOn ? HIGH : LOW);

  if (levelingOn) {
    int rollCmd  = SERVO_CENTER - constrain((int)correctedRoll,  -SERVO_MAX_DEFLECTION, SERVO_MAX_DEFLECTION);
    int pitchCmd = SERVO_CENTER + constrain((int)correctedPitch, -SERVO_MAX_DEFLECTION, SERVO_MAX_DEFLECTION);
    servoRoll.write(rollCmd);
    servoPitch.write(pitchCmd);
  } else {
    servoRoll.write(SERVO_CENTER);
    servoPitch.write(SERVO_CENTER);
  }

  // Uncomment for tuning on the serial plotter:
  // Serial.print(correctedRoll); Serial.print(",");
  // Serial.println(correctedPitch);
}
