#include <Wire.h>
#include <ESP32Servo.h>

#define MPU_ADDR 0x68


#define SERVO_PITCH_PIN 18
#define SERVO_ROLL_PIN  19


Servo servoPitch;
Servo servoRoll;


int16_t ax, ay, az;
int16_t gx, gy, gz;


float ax_g, ay_g, az_g;
float gx_dps, gy_dps;


float pitch_acc, roll_acc;
float pitch = 0, roll = 0;

// Timing
unsigned long prevTime;
float dt;

// Complementary filter
#define ALPHA 0.98

// Servo settings
#define SERVO_CENTER 90
#define SERVO_LIMIT  12    // max correction (+/- degrees)

// Control gains (TUNE CAREFULLY)
float Kp = 1.5;   // start LOW
float Kd = 0.05;


float prevPitchError = 0;
float prevRollError  = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  servoPitch.attach(SERVO_PITCH_PIN);
  servoRoll.attach(SERVO_ROLL_PIN);

  servoPitch.write(SERVO_CENTER);
  servoRoll.write(SERVO_CENTER);

  delay(2000);
  prevTime = millis();

  Serial.println("TVC Controller Ready");
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temp
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();


  ax_g = ax / 16384.0;
  ay_g = ay / 16384.0;
  az_g = az / 16384.0;

  gx_dps = gx / 131.0;
  gy_dps = gy / 131.0;


  roll_acc  = atan2(ay_g, az_g) * 180 / PI;
  pitch_acc = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI;


  roll  = ALPHA * (roll + gx_dps * dt) + (1 - ALPHA) * roll_acc;
  pitch = ALPHA * (pitch + gy_dps * dt) + (1 - ALPHA) * pitch_acc;


  float pitchError = -pitch;
  float rollError  = -roll;


  float pitchRate = (pitchError - prevPitchError) / dt;
  float rollRate  = (rollError - prevRollError) / dt;

  prevPitchError = pitchError;
  prevRollError  = rollError;

  float pitchCmd = Kp * pitchError + Kd * pitchRate;
  float rollCmd  = Kp * rollError  + Kd * rollRate;


  pitchCmd = constrain(pitchCmd, -SERVO_LIMIT, SERVO_LIMIT);
  rollCmd  = constrain(rollCmd,  -SERVO_LIMIT, SERVO_LIMIT);


  servoPitch.write(SERVO_CENTER + pitchCmd);
  servoRoll.write(SERVO_CENTER + rollCmd);

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" | Roll: "); Serial.print(roll);
  Serial.print(" | SP: "); Serial.print(SERVO_CENTER + pitchCmd);
  Serial.print(" | SR: "); Serial.println(SERVO_CENTER + rollCmd);

  delay(5); // ~200 Hz
}
