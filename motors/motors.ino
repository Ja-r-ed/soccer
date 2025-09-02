
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno;
float yawOffset;

int M1a = 13;
int M1b = 12;
int M2a = 4;
int M2b = 3;
int M3a = 7;
int M3b = 11;
int M4a = 6;
int M4b = 5;

int d1 = 22;
int d2 = 23;
int dena = 2;

int LeftX = A0;
int LeftY = A1;
int RightX = A2;
const unsigned long sampleDuration = 500; // Sampling time in milliseconds

/*
Front Left  (M1)
Front Right (M2)
Back Left   (M3)
Back Right  (M4)

m4a = YELLOW, 11 left side of robot
m4B = BLACK, 12
m2a = ORANGE, 2
m2b = BROWN, 3 
m1a = YELLOW, 4 right side of robot
m1b = ORANGE, 5
m3a = GREY, 6
m3b = WHITE, 7 

d1 = RED, 22
d2 = BLACK, 23
dena = BROWN, 13
*/
int M1Target = 0;
int M2Target = 0;
int M3Target = 0;
int M4Target = 0;

// Buffer for incoming serial data
float list[9];

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
  pinMode(LeftX, INPUT);
  pinMode(LeftY, INPUT);
  pinMode(RightX, INPUT);
  delay(100);

  Serial.begin(115200);    // Serial monitor (USB)
  Serial1.begin(9600);     // Serial1 for communication with ESP32

  setupBNO08x(); // Initialize BNO08x sensor
}

void loop()
{
  if (Serial1.available()) {
    String message = Serial1.readStringUntil('\n');  // Read the full line

    int index = 0;
    int lastComma = -1;

    for (int i = 0; i < message.length(); i++) {
      if (message.charAt(i) == ',' || i == message.length() - 1) {
        String valueStr;
        if (message.charAt(i) == ',') {
          valueStr = message.substring(lastComma + 1, i);
        } else {
          valueStr = message.substring(lastComma + 1);
        }

        valueStr.trim();  // Remove whitespace

        if (index < 9) {
          list[index] = valueStr.toFloat();  // Convert and store
          index++;
        }

        lastComma = i;
      }
    }

    // Optional: Print parsed values
    /*
    Serial.print("Parsed values: ");
    for (int i = 0; i < 9; i++) {
      Serial.print(list[i], 3);
      Serial.print(" ");
    }
    Serial.println();
    */
  }
  // Serial.print(list[0]);
  // Serial.print(list[1]);
  Serial.print(getAngleDegrees(list[0], list[1]));
  Serial.print(" ");
  Serial.print(getDistance(list[0], list[1])*getDistance(list[0], list[1])*getDistance(list[0], list[1]));
  Serial.print(" ");
  Serial.print(list[2]);
  Serial.print(" ");
  Serial.print(list[3]);
  Serial.print(" ");
  Serial.print(list[4]);
  Serial.print(" ");
  Serial.print(list[5]);
  Serial.print(" ");
  Serial.print(list[6]);
  Serial.print(" ");
  Serial.println(list[7]);
  Serial.println(getYaw());
  // Serial.println(yawOffset);
  FieldRelativeDrive(getAngleDegrees(list[0], list[1]), getDistance(list[0], list[1]), list[2], getYaw());
  if(list[4] > 0.5) {stopdribble();}
  if(list[5] > 0.5) {kickWithDribbler();}
  if(list[6] > 0.5) {dribble();}
  if(list[7] > 0.5) {resetGyro();}
  if(list[7] > 0.5) {yawOffset = getYaw();}
  if(-list[3] > 0.05 && -list[3] < -0.05) {
    dribbleWithStick(-list[3]);
  }
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

float convert(int value){
  return (value-270.0)/270.0;
}

void MotorTest() {
  analogWrite(M1a, 120);
  digitalWrite(M1b, LOW);

  analogWrite(M2a, 120);
  digitalWrite(M2b, LOW);

  analogWrite(M3a, 120);
  digitalWrite(M3b, LOW);

  analogWrite(M4a, 120);
  digitalWrite(M4b, LOW);
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

void dribbleWithStick(float speed) {
  if(speed >0) {
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
    analogWrite(dena, speed*255); 
  }
  if(speed <0) {
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
    analogWrite(dena, -speed*255);     
  }
}

void kickWithDribbler() {
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
  analogWrite(dena, 120);
}

void resetGyro() {

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
    SetSpeed(1, wheel_speeds[0] * 255);
    SetSpeed(2, wheel_speeds[1] * 255);
    SetSpeed(3, wheel_speeds[3] * 255);
    SetSpeed(4, wheel_speeds[2] * 255);
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
  static int PWM1Value = 0;
  static int PWM2Value = 0;
  static int PWM3Value = 0;
  static int PWM4Value = 0;

  if (Motor == 1) {
    if (PWM > 0) {
      analogWrite(M1a, PWM);
      digitalWrite(M1b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M1a, LOW);
      analogWrite(M1b, -PWM);
    }
    if (PWM == 0) {
      digitalWrite(M1a, LOW);
      digitalWrite(M1b, LOW);
    }
    PWM1Value = PWM;
  }

  if (Motor == 2) {
    if (PWM > 0) {
      analogWrite(M2a, PWM);
      digitalWrite(M2b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M2a, LOW);
      analogWrite(M2b, -PWM);
    }
    if (PWM == 0) {
      digitalWrite(M2a, LOW);
      digitalWrite(M2b, LOW);
    }
    PWM2Value = PWM;
  }

  if (Motor == 3) {
    if (PWM > 0) {
      analogWrite(M3a, PWM);
      digitalWrite(M3b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M3a, LOW);
      analogWrite(M3b, -PWM);
    }
    if (PWM == 0) {
      digitalWrite(M3a, LOW);
      digitalWrite(M3b, LOW);
    }
    PWM3Value = PWM;
  }

  if (Motor == 4) {
    if (PWM > 0) {
      analogWrite(M4a, PWM);
      digitalWrite(M4b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M4a, LOW);
      analogWrite(M4b, -PWM);
    }
    if (PWM == 0) {
      digitalWrite(M4a, LOW);
      digitalWrite(M4b, LOW);
    }
    PWM4Value = PWM;
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