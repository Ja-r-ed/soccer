#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Wire.h>
#define SLAVE_ADDR 0x42

struct Pose {
  float x;
  float y;
  float theta; // or yaw, in degrees
};

// Buffer for assembling UART input
String espBuffer = "";

// Latest formatted line from OpenMV
String lastLine;

Pose currentPosition = {0.0, 0.0, 0.0};

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
Adafruit_BNO08x bno;
float yawOffset;

// Joystick center and range
const float joystickCenter = XboxControllerNotificationParser::maxJoy / 2.0;
const float joystickRange = joystickCenter;  // = 32767.5

int M1a = 4;
int M1b = 12;
int M2a = 14;
int M2b = 13;
int M3a = 26;
int M3b = 25;
int M4a = 17;
int M4b = 16;

int d1 = 22;
int d2 = 23;
int dena = 2;

int M1Target = 0;
int M2Target = 0;
int M3Target = 0;
int M4Target = 0;

float ballDistance;
float ballAngle;
float goalAngle;
float goalDistance;

// PID controller struct
struct PID {
  float kP, kI, kD;
  float integral;
  float lastError;
  float outputMin, outputMax;

  void init(float p, float i, float d, float outMin, float outMax) {
    kP = p; kI = i; kD = d;
    integral = 0;
    lastError = 0;
    outputMin = outMin;
    outputMax = outMax;
  }

  float compute(float target, float current, float dt, bool wrapAngle = false) {
    float error = target - current;

    // Wrap rotation error to [-180, 180]
    if (wrapAngle) {
      while (error > 180) error -= 360;
      while (error < -180) error += 360;
    }

    integral += error * dt;
    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = kP * error + kI * integral + kD * derivative;

    // Clamp output
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;

    return output;
  }
};

PID rotationPID;
PID translationPID;

// Motion profile limits
float maxTranslationSpeed = 1;  // normalized (0–1)
float maxRotationSpeed = 1;     // normalized (-1–1)
float accelRate = 0.01;           // per loop (~50Hz)
float currentTranslationCmd = 0;
float currentRotationCmd = 0;

void setup() {
  pinMode(M1a, OUTPUT);
  pinMode(M1b, OUTPUT);
  pinMode(M2a, OUTPUT);
  pinMode(M2b, OUTPUT);
  pinMode(M3a, OUTPUT);
  pinMode(M3b, OUTPUT);
  pinMode(M4a, OUTPUT);
  pinMode(M4b, OUTPUT);
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(dena, OUTPUT);
  delay(100);

  Serial.begin(115200);    // Serial monitor (USB)
  Serial1.begin(9600);     // Serial1 for communication with ESP32

  setupBNO08x(); // Initialize BNO08x sensor
  xboxController.begin();  

  Wire.begin(21, 22); // SDA, SCL
  Serial.println("I2C Master ready");

  // Init PID controllers
  rotationPID.init(0.02, 0.0, 0.001, -maxRotationSpeed, maxRotationSpeed);
  translationPID.init(0.3, 0.0, 0.01, -maxTranslationSpeed, maxTranslationSpeed);

  Serial.println("Setup complete");
}

float convert(uint16_t raw) {
  return ((float)raw - joystickCenter) / joystickRange;
}

float applyMotionProfile(float target, float current) {
  if (target > current) {
    current += accelRate;
    if (current > target) current = target;
  } else if (target < current) {
    current -= accelRate;
    if (current < target) current = target;
  }
  return current;
}

void loop() {
    const int BUF_SIZE = 32;
    char buf[BUF_SIZE + 1]; // +1 for null terminator

    // Request exactly 32 bytes
    int bytesReceived = Wire.requestFrom(SLAVE_ADDR, BUF_SIZE);
    if (bytesReceived != BUF_SIZE) {
        // Slave didn’t respond fully; skip this loop
        Serial.println("I2C: incomplete read");
        delay(5);
        return;
    }

    // Read all 32 bytes into buf
    for (int i = 0; i < BUF_SIZE; i++) {
        buf[i] = Wire.read();
    }
    buf[BUF_SIZE] = '\0'; // null-terminate

    // Remove trailing spaces
    int len = BUF_SIZE;
    while (len > 0 && buf[len - 1] == ' ') buf[--len] = '\0';

    // Parse 4 floats
    float f1, f2, f3, f4;
    if (sscanf(buf, "%f %f %f %f", &f1, &f2, &f3, &f4) == 4) {
        ballDistance = f1;
        ballAngle    = f2;
        goalAngle    = f3;
        goalDistance = f4;
    } else {
        // parsing failed
        ballDistance = ballAngle = goalAngle = goalDistance = 0;
    }
    Serial.print(ballDistance);
    Serial.print(" ");
    Serial.println(ballAngle);

    // Run robot logic
    goToBall();
}


void goToBall() {
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0 || dt > 1) dt = 0.02;
  lastTime = now;

  // --- Targets ---
  float targetDistance = 10.5;
  float targetAngle = 209.3;

  // --- Current readings ---
  float dist = ballDistance;
  float yaw  = getYaw();

  // --- PID Outputs ---
  float rotationCmd = rotationPID.compute(targetAngle, ballAngle, dt, true);
  float translationCmd = translationPID.compute(targetDistance, dist, dt);

  Serial.print("rotationCmd: ");
  Serial.print(rotationCmd);
  Serial.print("translationCmd: ");
  Serial.print(translationCmd);

  // Apply trapezoidal ramp
  // currentRotationCmd = applyMotionProfile(rotationCmd, currentRotationCmd);
  // currentTranslationCmd = applyMotionProfile(translationCmd, currentTranslationCmd);

  // // --- Drive ---
  // drive(0, fabs(currentTranslationCmd), currentRotationCmd);

  drive(0, fabs(translationCmd), rotationCmd);

  if(ballDistance == 0.0 && ballAngle == 0.0) {
    drive(0,0,0);
    // Serial.println("drive getting 0,0");
  }
}

void updateOdometry() {
  static float vx = 0, vy = 0; // velocity estimates (m/s)
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // seconds
  if (dt <= 0 || dt > 1) dt = 0.02; // fallback for first run or large gaps
  lastTime = now;

  sh2_SensorValue_t sensorValue;
  if (bno.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      float ax = sensorValue.un.linearAcceleration.x;
      float ay = sensorValue.un.linearAcceleration.y;

      float theta_rad = currentPosition.theta * PI / 180.0;

      float ax_field = ax * cos(theta_rad) - ay * sin(theta_rad);
      float ay_field = ax * sin(theta_rad) + ay * cos(theta_rad);

      vx += ax_field * dt;
      vy += ay_field * dt;

      if (abs(ax_field) < 0.05) vx *= 0.98;
      if (abs(ay_field) < 0.05) vy *= 0.98;

      currentPosition.x += vx * dt;
      currentPosition.y += vy * dt;
    }
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float q0 = sensorValue.un.rotationVector.real;
      float q1 = sensorValue.un.rotationVector.i;
      float q2 = sensorValue.un.rotationVector.j;
      float q3 = sensorValue.un.rotationVector.k;
      float yaw = atan2(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3));
      yaw = yaw * 180.0f / PI;
      yaw -= yawOffset;
      if (yaw > 180) yaw -= 360;
      if (yaw < -180) yaw += 360;
      currentPosition.theta = yaw;
    }
  }
}

float getAngleDegrees(float x, float y) {
  float angleRad = atan2(x, -y);
  float angleDeg = angleRad * 180.0 / PI;
  if (angleDeg < 0) {
    angleDeg += 360.0;
  }
  return angleDeg;
}

float getDistance(float x, float y) {
  return sqrt(x * x + y * y);
}

void dribble() {
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
  analogWrite(dena, 80);
}

void stopdribble() {
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  analogWrite(dena, 0); 
}

void drive(float direction_deg, float speed, float rotation) {
  if (speed <= 0.05) {
    speed = 0.05;
  }
  if (abs(rotation) <= 0.05) {
    rotation = 0;
  }
  if (speed > 1.0) {
    speed = 1.0;
  }

  const float minPWM = 40.0;
  const float maxPWM = 255.0;

  float scaledPWM = 0;
  if (speed > 0) {
    scaledPWM = minPWM + (maxPWM - minPWM) * speed;
  }

  float direction_rad = direction_deg * PI / 180.0;
  float vx = speed * cos(direction_rad);
  float vy = speed * sin(direction_rad);

  float wheel_speeds[4];
  wheel_speeds[0] = vx * sin(45 * PI / 180.0) + vy * cos(45 * PI / 180.0) + rotation;
  wheel_speeds[1] = vx * sin(-45 * PI / 180.0) + vy * cos(-45 * PI / 180.0) + rotation;
  wheel_speeds[2] = vx * sin(-135 * PI / 180.0) + vy * cos(-135 * PI / 180.0) + rotation;
  wheel_speeds[3] = vx * sin(135 * PI / 180.0) + vy * cos(135 * PI / 180.0) + rotation;

  float max_speed = 0;
  for (int i = 0; i < 4; i++) {
    if (abs(wheel_speeds[i]) > max_speed) {
      max_speed = abs(wheel_speeds[i]);
    }
  }
  if (max_speed > 1.0) {
    for (int i = 0; i < 4; i++) {
      wheel_speeds[i] /= max_speed;
    }
  }

  SetSpeed(3, wheel_speeds[0] * scaledPWM); //FL
  SetSpeed(1, wheel_speeds[1] * scaledPWM); //FR
  SetSpeed(2, wheel_speeds[3] * scaledPWM); //BL
  SetSpeed(4, wheel_speeds[2] * scaledPWM); //BR
}

void FieldRelativeDrive(float joystick_direction_deg, float speed, float rotation, float gyroangle_deg) {
  float field_direction = joystick_direction_deg + gyroangle_deg;
  if (field_direction >= 360.0) {field_direction -= 360.0;}
  if (field_direction < 0.0) {field_direction += 360.0;}
  drive(field_direction, speed, rotation);
}

void SetSpeed(int Motor, int PWM) {
  static int PWMValue[4] = {0, 0, 0, 0};
  int motorA[] = {M1a, M2a, M3a, M4a};
  int motorB[] = {M1b, M2b, M3b, M4b};

  if (Motor >= 1 && Motor <= 4) {
    int idx = Motor - 1;
    if (PWM > 0) {
      analogWrite(motorA[idx], PWM);
      digitalWrite(motorB[idx], LOW);
    } else if (PWM < 0) {
      digitalWrite(motorA[idx], LOW);
      analogWrite(motorB[idx], -PWM);
    } else {
      digitalWrite(motorA[idx], LOW);
      digitalWrite(motorB[idx], LOW);
    }
    PWMValue[idx] = PWM;
  }
}

bool setupBNO08x() {
  if (!bno.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    return false;
  }
  Serial.println("BNO08x Found!");

  if (!bno.enableReport(SH2_LINEAR_ACCELERATION, 20000)) {
    Serial.println("Could not enable linear acceleration");
    return false;
  }
  if (!bno.enableReport(SH2_ROTATION_VECTOR, 20000)) {
    Serial.println("Could not enable rotation vector");
    return false;
  }
  return true;
}

float getYaw() {
  sh2_SensorValue_t sensorValue;
  static float oldyaw;
  
  if (bno.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float q0 = sensorValue.un.rotationVector.real;
      float q1 = sensorValue.un.rotationVector.i;
      float q2 = sensorValue.un.rotationVector.j;
      float q3 = sensorValue.un.rotationVector.k;

      float yaw = atan2(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3));
      yaw = yaw * 180.0f / PI;

      yaw -= yawOffset;
      if (yaw > 180) yaw -= 360;
      if (yaw < -180) yaw += 360;

      oldyaw = yaw;
      return yaw;
    }
  }
  return oldyaw;
}

void resetYaw() {
  yawOffset = getYaw();
}
