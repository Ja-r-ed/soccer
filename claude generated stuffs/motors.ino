#include <HardwareSerial.h>
#include <math.h>

// Serial communication with OpenMV
HardwareSerial openMVSerial(1);

// X-Drive Motor Control Pins (assuming Cytron MD10C or similar drivers)
// Front-Left Motor (45° orientation)
#define MOTOR_FL_PWM 18
#define MOTOR_FL_DIR 19

// Front-Right Motor (-45° orientation)  
#define MOTOR_FR_PWM 21
#define MOTOR_FR_DIR 22

// Back-Left Motor (-45° orientation)
#define MOTOR_BL_PWM 23
#define MOTOR_BL_DIR 25

// Back-Right Motor (45° orientation)
#define MOTOR_BR_PWM 26
#define MOTOR_BR_DIR 27

// Dribbler Motor Control
#define DRIBBLER_PWM 32
#define DRIBBLER_DIR1 33
#define DRIBBLER_DIR2 34

// Robot state received from OpenMV
struct RobotState {
    float x, y, theta;           // Position and orientation
    float vx, vy, omega;         // Velocities
    float ball_x, ball_y;        // Ball position relative to robot
    bool ball_visible;           // Ball detection flag
    uint32_t timestamp;
};

RobotState currentState;
RobotState targetState;

// Motor speeds (-255 to 255)
int motorSpeeds[4] = {0, 0, 0, 0}; // FL, FR, BL, BR
int dribblerSpeed = 0;

// PID Controllers
struct PIDController {
    float kp, ki, kd;
    float integral;
    float lastError;
    float maxIntegral;
    float maxOutput;
};

PIDController positionPID_X = {1.5, 0.1, 0.3, 0, 0, 50, 200};
PIDController positionPID_Y = {1.5, 0.1, 0.3, 0, 0, 50, 200};
PIDController headingPID = {2.0, 0.05, 0.5, 0, 0, 10, 150};

// Game strategy states
enum GameState {
    SEEK_BALL,
    APPROACH_BALL,
    DRIBBLE_BALL,
    SHOOT_BALL,
    DEFEND,
    MANUAL_CONTROL
};

GameState currentGameState = SEEK_BALL;
unsigned long stateStartTime = 0;

void setup() {
    Serial.begin(115200);
    openMVSerial.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
    
    // Setup motor pins
    setupMotorPins();
    
    // Initialize robot state
    memset(&currentState, 0, sizeof(currentState));
    memset(&targetState, 0, sizeof(targetState));
    
    Serial.println("X-Drive Soccer Robot Controller Initialized");
    Serial.println("Commands: STOP, MANUAL, AUTO, DRIBBLE_ON, DRIBBLE_OFF");
    
    currentGameState = SEEK_BALL;
    stateStartTime = millis();
}

void setupMotorPins() {
    // Configure motor PWM pins
    pinMode(MOTOR_FL_PWM, OUTPUT);
    pinMode(MOTOR_FL_DIR, OUTPUT);
    pinMode(MOTOR_FR_PWM, OUTPUT);
    pinMode(MOTOR_FR_DIR, OUTPUT);
    pinMode(MOTOR_BL_PWM, OUTPUT);
    pinMode(MOTOR_BL_DIR, OUTPUT);
    pinMode(MOTOR_BR_PWM, OUTPUT);
    pinMode(MOTOR_BR_DIR, OUTPUT);
    
    // Configure dribbler pins
    pinMode(DRIBBLER_PWM, OUTPUT);
    pinMode(DRIBBLER_DIR1, OUTPUT);
    pinMode(DRIBBLER_DIR2, OUTPUT);
    
    // Setup PWM frequency (1kHz)
    analogWriteFreq(1000);
    
    Serial.println("Motor pins configured");
}

void setXDriveMotors(float vx, float vy, float omega) {
    // X-Drive kinematics: convert desired velocities to motor speeds
    // Each motor is at 45° to the robot frame
    
    // Motor equations for X-drive (omnidirectional)
    // FL: +vx +vy +omega (45° wheel)
    // FR: +vx -vy +omega (-45° wheel)  
    // BL: +vx +vy -omega (-45° wheel)
    // BR: +vx -vy -omega (45° wheel)
    
    float sqrt2 = 1.414213562;
    
    // Calculate motor speeds
    int fl = (int)((vx + vy + omega) * sqrt2);
    int fr = (int)((vx - vy + omega) * sqrt2);
    int bl = (int)((vx + vy - omega) * sqrt2);
    int br = (int)((vx - vy - omega) * sqrt2);
    
    // Constrain to motor limits
    fl = constrain(fl, -255, 255);
    fr = constrain(fr, -255, 255);
    bl = constrain(bl, -255, 255);
    br = constrain(br, -255, 255);
    
    // Set motor speeds
    setMotor(0, fl); // Front-Left
    setMotor(1, fr); // Front-Right
    setMotor(2, bl); // Back-Left
    setMotor(3, br); // Back-Right
    
    // Store for debugging
    motorSpeeds[0] = fl;
    motorSpeeds[1] = fr;
    motorSpeeds[2] = bl;
    motorSpeeds[3] = br;
}

void setMotor(int motorIndex, int speed) {
    // Motor pin assignments
    int pwmPin, dirPin;
    
    switch(motorIndex) {
        case 0: // Front-Left
            pwmPin = MOTOR_FL_PWM;
            dirPin = MOTOR_FL_DIR;
            break;
        case 1: // Front-Right
            pwmPin = MOTOR_FR_PWM;
            dirPin = MOTOR_FR_DIR;
            break;
        case 2: // Back-Left
            pwmPin = MOTOR_BL_PWM;
            dirPin = MOTOR_BL_DIR;
            break;
        case 3: // Back-Right
            pwmPin = MOTOR_BR_PWM;
            dirPin = MOTOR_BR_DIR;
            break;
        default:
            return;
    }
    
    // Set direction and PWM
    if (speed > 0) {
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin, abs(speed));
    } else if (speed < 0) {
        digitalWrite(dirPin, LOW);
        analogWrite(pwmPin, abs(speed));
    } else {
        analogWrite(pwmPin, 0);
    }
}

void setDribbler(int speed) {
    // Control dribbler motor (H-bridge control)
    dribblerSpeed = constrain(speed, -255, 255);
    
    if (speed > 0) {
        digitalWrite(DRIBBLER_DIR1, HIGH);
        digitalWrite(DRIBBLER_DIR2, LOW);
        analogWrite(DRIBBLER_PWM, speed);
    } else if (speed < 0) {
        digitalWrite(DRIBBLER_DIR1, LOW);
        digitalWrite(DRIBBLER_DIR2, HIGH);
        analogWrite(DRIBBLER_PWM, -speed);
    } else {
        digitalWrite(DRIBBLER_DIR1, LOW);
        digitalWrite(DRIBBLER_DIR2, LOW);
        analogWrite(DRIBBLER_PWM, 0);
    }
}

float calculatePID(PIDController* pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // Handle angle wrapping for heading control
    if (pid == &headingPID) {
        while (error > PI) error -= 2*PI;
        while (error < -PI) error += 2*PI;
    }
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    pid->integral = constrain(pid->integral, -pid->maxIntegral, pid->maxIntegral);
    
    // Derivative term
    float derivative = (error - pid->lastError) / dt;
    pid->lastError = error;
    
    // Calculate output
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    return constrain(output, -pid->maxOutput, pid->maxOutput);
}

void receiveStateFromOpenMV() {
    if (openMVSerial.available() >= 33) { // 6 floats + 2 floats + 1 bool = 33 bytes
        uint8_t buffer[33];
        openMVSerial.readBytes(buffer, 33);
        
        // Unpack robot state (6 floats)
        memcpy(&currentState.x, buffer, 4);
        memcpy(&currentState.y, buffer + 4, 4);
        memcpy(&currentState.theta, buffer + 8, 4);
        memcpy(&currentState.vx, buffer + 12, 4);
        memcpy(&currentState.vy, buffer + 16, 4);
        memcpy(&currentState.omega, buffer + 20, 4);
        
        // Unpack ball information (2 floats + 1 bool)
        memcpy(&currentState.ball_x, buffer + 24, 4);
        memcpy(&currentState.ball_y, buffer + 28, 4);
        memcpy(&currentState.ball_visible, buffer + 32, 1);
        
        currentState.timestamp = millis();
        
        // Debug output (reduced frequency)
        static unsigned long lastDebugTime = 0;
        if (millis() - lastDebugTime > 1000) {
            Serial.printf("State: (%.1f,%.1f) θ:%.1f° Ball:(%.1f,%.1f) Vis:%d\n",
                         currentState.x, currentState.y, currentState.theta * 180.0 / PI,
                         currentState.ball_x, currentState.ball_y, currentState.ball_visible);
            lastDebugTime = millis();
        }
    }
}

void executeGameStrategy() {
    unsigned long currentTime = millis();
    float dt = 0.05; // 50ms control loop
    
    switch(currentGameState) {
        case SEEK_BALL:
            seekBallBehavior();
            break;
            
        case APPROACH_BALL:
            approachBallBehavior(dt);
            break;
            
        case DRIBBLE_BALL:
            dribbleBallBehavior(dt);
            break;
            
        case SHOOT_BALL:
            shootBallBehavior(dt);
            break;
            
        case DEFEND:
            defendBehavior(dt);
            break;
            
        case MANUAL_CONTROL:
            // Do nothing, wait for manual commands
            break;
    }
}

void seekBallBehavior() {
    // Rotate to search for ball
    if (!currentState.ball_visible) {
        // Slow rotation to search
        setXDriveMotors(0, 0, 0.5); // Rotate counterclockwise
        setDribbler(0);
    } else {
        // Ball found, switch to approach
        currentGameState = APPROACH_BALL;
        stateStartTime = millis();
        Serial.println("State: APPROACH_BALL");
    }
}

void approachBallBehavior(float dt) {
    if (!currentState.ball_visible) {
        // Lost ball, go back to seeking
        currentGameState = SEEK_BALL;
        stateStartTime = millis();
        Serial.println("State: SEEK_BALL");
        return;
    }
    
    // Move toward ball using proportional control
    float ball_distance = sqrt(currentState.ball_x * currentState.ball_x + 
                              currentState.ball_y * currentState.ball_y);
    
    if (ball_distance < 15.0) { // Close to ball
        currentGameState = DRIBBLE_BALL;
        stateStartTime = millis();
        Serial.println("State: DRIBBLE_BALL");
        return;
    }
    
    // Proportional approach
    float approach_speed = 0.8;
    float vx = currentState.ball_y * approach_speed / 50.0;  // Forward toward ball
    float vy = -currentState.ball_x * approach_speed / 50.0; // Strafe to center ball
    float omega = -currentState.ball_x * 0.02; // Rotate to face ball
    
    setXDriveMotors(vx, vy, omega);
    setDribbler(0);
}

void dribbleBallBehavior(float dt) {
    if (!currentState.ball_visible) {
        // Lost ball
        currentGameState = SEEK_BALL;
        stateStartTime = millis();
        setDribbler(0);
        Serial.println("State: SEEK_BALL");
        return;
    }
    
    // Activate dribbler
    setDribbler(200); // Moderate dribbling speed
    
    // Check if we should shoot
    float goal_distance = sqrt((currentState.x - 0) * (currentState.x - 0) + 
                              (currentState.y - 91.5) * (currentState.y - 91.5));
    
    if (goal_distance < 40.0) { // Close to opponent goal
        currentGameState = SHOOT_BALL;
        stateStartTime = millis();
        Serial.println("State: SHOOT_BALL");
        return;
    }
    
    // Move toward opponent goal while dribbling
    float target_x = 0;   // Center of field
    float target_y = 80;  // Close to opponent goal
    
    float dx = target_x - currentState.x;
    float dy = target_y - currentState.y;
    
    float vx = dx * 0.02; // Proportional control
    float vy = dy * 0.02;
    float omega = 0;      // Keep current orientation
    
    setXDriveMotors(vx, vy, omega);
}

void shootBallBehavior(float dt) {
    // Maximum dribbler speed to "shoot" the ball
    setDribbler(255);
    
    // Move forward to push ball toward goal
    setXDriveMotors(1.0, 0, 0);
    
    // Hold shooting for 1 second
    if (millis() - stateStartTime > 1000) {
        currentGameState = SEEK_BALL;
        stateStartTime = millis();
        Serial.println("State: SEEK_BALL");
    }
}

void defendBehavior(float dt) {
    // Simple defense: stay between ball and own goal
    float own_goal_x = 0;
    float own_goal_y = -91.5;
    
    if (currentState.ball_visible) {
        // Position between ball and goal
        float target_x = currentState.ball_x * 0.5;
        float target_y = currentState.ball_y * 0.5 - 30; // Stay back
        
        float dx = target_x - currentState.x;
        float dy = target_y - currentState.y;
        
        float vx = dx * 0.015;
        float vy = dy * 0.015;
        
        setXDriveMotors(vx, vy, 0);
    } else {
        // No ball visible, stay near goal
        float vx = (own_goal_x - currentState.x) * 0.01;
        float vy = ((own_goal_y + 30) - currentState.y) * 0.01;
        
        setXDriveMotors(vx, vy, 0);
    }
    
    setDribbler(0);
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command == "STOP") {
            setXDriveMotors(0, 0, 0);
            setDribbler(0);
            Serial.println("All motors stopped");
        }
        else if (command == "MANUAL") {
            currentGameState = MANUAL_CONTROL;
            Serial.println("Switched to manual control");
        }
        else if (command == "AUTO") {
            currentGameState = SEEK_BALL;
            stateStartTime = millis();
            Serial.println("Switched to autonomous mode");
        }
        else if (command == "DRIBBLE_ON") {
            setDribbler(150);
            Serial.println("Dribbler ON");
        }
        else if (command == "DRIBBLE_OFF") {
            setDribbler(0);
            Serial.println("Dribbler OFF");
        }
        else if (command == "DEFEND") {
            currentGameState = DEFEND;
            stateStartTime = millis();
            Serial.println("Switched to defense mode");
        }
        else if (command.startsWith("MOVE")) {
            // Format: MOVE vx vy omega
            int space1 = command.indexOf(' ');
            int space2 = command.indexOf(' ', space1 + 1);
            int space3 = command.indexOf(' ', space2 + 1);
            
            if (space1 > 0 && space2 > 0 && space3 > 0) {
                float vx = command.substring(space1 + 1, space2).toFloat();
                float vy = command.substring(space2 + 1, space3).toFloat();
                float omega = command.substring(space3 + 1).toFloat();
                
                setXDriveMotors(vx, vy, omega);
                Serial.printf("Manual movement: vx=%.2f, vy=%.2f, omega=%.2f\n", vx, vy, omega);
            }
        }
        else if (command == "STATUS") {
            printRobotStatus();
        }
    }
}

void printRobotStatus() {
    Serial.println("=== ROBOT STATUS ===");
    Serial.printf("Position: (%.2f, %.2f) cm\n", currentState.x, currentState.y);
    Serial.printf("Orientation: %.2f degrees\n", currentState.theta * 180.0 / PI);
    Serial.printf("Ball Position: (%.2f, %.2f) cm, Visible: %s\n", 
                 currentState.ball_x, currentState.ball_y, 
                 currentState.ball_visible ? "Yes" : "No");
    Serial.printf("Motor Speeds: FL=%d, FR=%d, BL=%d, BR=%d\n", 
                 motorSpeeds[0], motorSpeeds[1], motorSpeeds[2], motorSpeeds[3]);
    Serial.printf("Dribbler Speed: %d\n", dribblerSpeed);
    Serial.printf("Game State: %d\n", currentGameState);
    Serial.printf("Uptime: %lu ms\n", millis());
    Serial.println("===================");
}

void emergencyStop() {
    setXDriveMotors(0, 0, 0);
    setDribbler(0);
    currentGameState = MANUAL_CONTROL;
    Serial.println("EMERGENCY STOP - All motors halted");
}

void loop() {
    // Receive updated state from OpenMV
    receiveStateFromOpenMV();
    
    // Handle manual commands
    handleSerialCommands();
    
    // Execute game strategy (only if not in manual mode)
    if (currentGameState != MANUAL_CONTROL) {
        executeGameStrategy();
    }
    
    // Safety check - stop if no recent data from OpenMV
    if (millis() - currentState.timestamp > 500) {
        static bool warningShown = false;
        if (!warningShown) {
            Serial.println("WARNING: No data from OpenMV - stopping motors");
            warningShown = true;
        }
        setXDriveMotors(0, 0, 0);
        setDribbler(0);
    }
    
    delay(20); // 50Hz control loop
}