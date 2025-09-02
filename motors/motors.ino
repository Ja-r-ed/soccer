#include <Arduino.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno;
float yawOffset;

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

// Front Left  (M3)
// Front Right (M1)
// Back Left   (M2)
// Back Right  (M4)

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

  Serial.println("Setup complete");
}

void loop()
{

}

float getAngleDegrees(float x, float y) {
  float angleRad = atan2(x, -y);             // Angle in radians, -PI to PI
  float angleDeg = angleRad * 180.0 / PI;   // Convert to degrees
  if (angleDeg < 0) {
    angleDeg += 360.0;                      // Normalize to 0–360
  }
  return angleDeg;
}

float getDistance(float x, float y) {
  return sqrt(x * x + y * y);               // Pythagorean theorem
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
    // Apply deadzone thresholds
    if (speed <= 0.05) {
      speed = 0;
    }
    if (abs(rotation) <= 0.05) {
      rotation = 0;
    }
    if (speed > 1.0) {
      speed = 1.0;
    }
    
    speed = speed*speed*speed;

    if (speed > 1.0) {
      speed = 1.0;
    }

    rotation = rotation * 0.25; // Scale rotation for better control

    // Convert direction to radians
    float direction_rad = direction_deg * 3.14159265 / 180.0;
    float vx = speed * cos(direction_rad);
    float vy = speed * sin(direction_rad);

    // X-drive kinematics for 4 wheels: FL, FR, BR, BL
    float wheel_speeds[4];
    wheel_speeds[0] = vx * sin(45 * 3.14159265 / 180.0) + vy * cos(45 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[1] = vx * sin(-45 * 3.14159265 / 180.0) + vy * cos(-45 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[2] = vx * sin(-135 * 3.14159265 / 180.0) + vy * cos(-135 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[3] = vx * sin(135 * 3.14159265 / 180.0) + vy * cos(135 * 3.14159265 / 180.0) + rotation;

    // Normalize speeds if needed
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

    // Apply motor speeds (mapped to range -255 to 255)
    SetSpeed(3, wheel_speeds[0] * 255); //FL
    SetSpeed(1, wheel_speeds[1] * 255); //FR
    SetSpeed(2, wheel_speeds[3] * 255); //BL
    SetSpeed(4, wheel_speeds[2] * 255); //BR
}

void FieldRelativeDrive(float joystick_direction_deg, float speed, float rotation, float gyroangle_deg) {
    // Adjust joystick direction based on current gyro angle
    float field_direction = joystick_direction_deg + gyroangle_deg;

    // Keep angle in [0, 360)
    if (field_direction >= 360.0) {field_direction -= 360.0;}
    if (field_direction < 0.0) {field_direction += 360.0;}

    // Call your normal drive function
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

  // Enable rotation vector (quaternion-based orientation)
  // SH2_ROTATION_VECTOR works for yaw/pitch/roll
  if (!bno.enableReport(SH2_ROTATION_VECTOR, 20000)) { // 20000us = 50Hz
    Serial.println("Could not enable rotation vector");
    return false;
  }
  return true;
}

// Function to get yaw in degrees (adjusted with yawOffset)
float getYaw() {
  sh2_SensorValue_t sensorValue;
  static float oldyaw;
  
  // Get rotation vector
  if (bno.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      // Extract quaternion
      float q0 = sensorValue.un.rotationVector.real;
      float q1 = sensorValue.un.rotationVector.i;
      float q2 = sensorValue.un.rotationVector.j;
      float q3 = sensorValue.un.rotationVector.k;

      // Convert quaternion to yaw (Z axis rotation)
      float yaw = atan2(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3));

      // Convert to degrees
      yaw = yaw * 180.0f / PI;

      // Apply offset and keep in range [-180, 180]
      yaw -= yawOffset;
      if (yaw > 180) yaw -= 360;
      if (yaw < -180) yaw += 360;

      oldyaw = yaw;
      return yaw;
    }
  }
  return oldyaw; // No update
}

void resetYaw() {
  // sh2_SensorValue_t sensorValue;
  
  // // Get rotation vector
  // if (bno.getSensorEvent(&sensorValue)) {
  //   if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
  //     float q0 = sensorValue.un.rotationVector.real;
  //     float q1 = sensorValue.un.rotationVector.i;
  //     float q2 = sensorValue.un.rotationVector.j;
  //     float q3 = sensorValue.un.rotationVector.k;

  //     float yaw = atan2(2.0f * (q0 * q3 + q1 * q2),
  //                       1.0f - 2.0f * (q2 * q2 + q3 * q3));

  //     yawOffset = yaw * 180.0f / PI;
  //   }
  // }
  yawOffset = getYaw();
}