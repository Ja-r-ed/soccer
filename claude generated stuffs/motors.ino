#include <HardwareSerial.h>
#include <ESP32Servo.h>

// Serial communication with OpenMV
HardwareSerial openMVSerial(1);

// Motor control pins (assuming H-bridge motor driver)
#define MOTOR_LEFT_PWM 18
#define MOTOR_LEFT_DIR1 19
#define MOTOR_LEFT_DIR2 21

#define MOTOR_RIGHT_PWM 22
#define MOTOR_RIGHT_DIR1 23
#define MOTOR_RIGHT_DIR2 25

// Servo for kicker mechanism
Servo kickerServo;
#define SERVO_PIN 26

// Robot state received from OpenMV
struct RobotState {
    float x;
    float y;
    float theta;
    uint32_t timestamp;
};

RobotState currentState;

// Motor control variables
int leftMotorSpeed = 0;   // -255 to 255
int rightMotorSpeed = 0;  // -255 to 255

// PID controller for heading
struct PIDController {
    float kp = 2.0;
    float ki = 0.1;
    float kd = 0.5;
    float integral = 0.0;
    float lastError = 0.0;
    float output = 0.0;
};

PIDController headingPID;

void setup() {
    Serial.begin(115200);
    openMVSerial.begin(115200, SERIAL_8N1, 16, 17); // RX, TX pins
    
    // Setup motor pins
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_LEFT_DIR1, OUTPUT);
    pinMode(MOTOR_LEFT_DIR2, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
    
    // Initialize servo
    kickerServo.attach(SERVO_PIN);
    kickerServo.write(90); // Neutral position
    
    Serial.println("Auxiliary controller initialized");
    
    // Initialize robot state
    currentState.x = 0.0;
    currentState.y = 0.0;
    currentState.theta = 0.0;
    currentState.timestamp = millis();
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Left motor
    if (leftSpeed > 0) {
        digitalWrite(MOTOR_LEFT_DIR1, HIGH);
        digitalWrite(MOTOR_LEFT_DIR2, LOW);
        analogWrite(MOTOR_LEFT_PWM, leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(MOTOR_LEFT_DIR1, LOW);
        digitalWrite(MOTOR_LEFT_DIR2, HIGH);
        analogWrite(MOTOR_LEFT_PWM, -leftSpeed);
    } else {
        digitalWrite(MOTOR_LEFT_DIR1, LOW);
        digitalWrite(MOTOR_LEFT_DIR2, LOW);
        analogWrite(MOTOR_LEFT_PWM, 0);
    }
    
    // Right motor
    if (rightSpeed > 0) {
        digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
        digitalWrite(MOTOR_RIGHT_DIR2, LOW);
        analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(MOTOR_RIGHT_DIR1, LOW);
        digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
        analogWrite(MOTOR_RIGHT_PWM, -rightSpeed);
    } else {
        digitalWrite(MOTOR_RIGHT_DIR1, LOW);
        digitalWrite(MOTOR_RIGHT_DIR2, LOW);
        analogWrite(MOTOR_RIGHT_PWM, 0);
    }
    
    leftMotorSpeed = leftSpeed;
    rightMotorSpeed = rightSpeed;
}

float calculatePID(PIDController* pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // Handle angle wrapping for heading control
    if (error > PI) error -= 2*PI;
    if (error < -PI) error += 2*PI;
    
    pid->integral += error * dt;
    pid->integral = constrain(pid->integral, -10, 10); // Anti-windup
    
    float derivative = (error - pid->lastError) / dt;
    
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->lastError = error;
    
    return pid->output;
}

void moveToPosition(float targetX, float targetY, float targetTheta) {
    float dx = targetX - currentState.x;
    float dy = targetY - currentState.y;
    float distance = sqrt(dx*dx + dy*dy);
    
    if (distance < 5.0) { // Close enough, just adjust heading
        float headingError = targetTheta - currentState.theta;
        float turnSpeed = calculatePID(&headingPID, 0, headingError, 0.05);
        
        setMotorSpeed(-turnSpeed, turnSpeed);
        return;
    }
    
    // Calculate desired heading to target
    float desiredHeading = atan2(dy, dx);
    float headingError = desiredHeading - currentState.theta;
    
    // Normalize angle
    if (headingError > PI) headingError -= 2*PI;
    if (headingError < -PI) headingError += 2*PI;
    
    // Simple proportional control
    int baseSpeed = 150;  // Base forward speed
    int turnSpeed = headingError * 100;  // Proportional turning
    
    // Differential drive
    int leftSpeed = baseSpeed - turnSpeed;
    int rightSpeed = baseSpeed + turnSpeed;
    
    setMotorSpeed(leftSpeed, rightSpeed);
}

void receiveStateFromOpenMV() {
    if (openMVSerial.available() >= 12) { // 3 floats * 4 bytes
        uint8_t buffer[12];
        openMVSerial.readBytes(buffer, 12);
        
        // Unpack position data
        memcpy(&currentState.x, buffer, 4);
        memcpy(&currentState.y, buffer + 4, 4);
        memcpy(&currentState.theta, buffer + 8, 4);
        currentState.timestamp = millis();
        
        Serial.printf("Received position: (%.1f, %.1f), angle: %.1f°\n", 
                     currentState.x, currentState.y, 
                     currentState.theta * 180.0 / PI);
    }
}

void kick() {
    // Activate kicker mechanism
    kickerServo.write(0);   // Kick position
    delay(200);             // Hold briefly
    kickerServo.write(90);  // Return to neutral
    
    Serial.println("Ball kicked!");
}

void executeStrategy() {
    // Simple strategy: move toward ball, then toward goal
    // This is where you'd implement your game strategy
    
    static enum State {SEEK_BALL, APPROACH_BALL, KICK_BALL, MOVE_TO_GOAL} state = SEEK_BALL;
    static unsigned long stateStartTime = millis();