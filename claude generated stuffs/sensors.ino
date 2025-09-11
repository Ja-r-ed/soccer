#include <Wire.h>
#include <VL53L0X.h>
#include <HardwareSerial.h>

// VL53L0X ToF sensors (up to 4 for X-drive robot)
VL53L0X sensor1, sensor2, sensor3, sensor4;

// Sensor addresses after initialization
#define SENSOR1_ADDRESS 0x30  // Front-Right
#define SENSOR2_ADDRESS 0x31  // Front-Left  
#define SENSOR3_ADDRESS 0x32  // Back-Right
#define SENSOR4_ADDRESS 0x33  // Back-Left

// GPIO pins for XSHUT (shutdown) control
#define XSHUT_PIN1 4   // Front-Right
#define XSHUT_PIN2 5   // Front-Left
#define XSHUT_PIN3 18  // Back-Right
#define XSHUT_PIN4 19  // Back-Left

// Serial communication with OpenMV
HardwareSerial openMVSerial(1);

// Timing
unsigned long lastMeasurement = 0;
const unsigned long MEASUREMENT_INTERVAL = 50; // 20Hz

// Data structure for sensor readings
struct SensorData {
    float distances[4];  // Four ToF sensors
    uint32_t timestamp;
    bool sensorsActive[4];
};

SensorData sensorData;

// Sensor configuration
struct SensorConfig {
    bool enabled;
    uint16_t timingBudget;  // Measurement timing budget in microseconds
    uint16_t maxRange;      // Maximum valid range in mm
} sensorConfigs[4];

void setup() {
    Serial.begin(115200);
    openMVSerial.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17 to OpenMV
    
    Serial.println("RoboCup Soccer Robot - Sensors Controller");
    Serial.println("Initializing ToF sensors...");
    
    // Initialize I2C
    Wire.begin(21, 22); // SDA=21, SCL=22
    Wire.setClock(400000); // 400kHz I2C speed
    
    // Setup XSHUT pins
    setupXSHUTPins();
    
    // Initialize all sensor configurations
    initSensorConfigs();
    
    // Initialize ToF sensors
    if (initializeToFSensors()) {
        Serial.println("All ToF sensors initialized successfully");
    } else {
        Serial.println("Some ToF sensors failed to initialize");
    }
    
    // Initialize data structure
    memset(&sensorData, 0, sizeof(sensorData));
    
    Serial.println("Sensors controller ready");
    Serial.println("Commands: RESET, FAST, ACCURATE, ENABLE_n, DISABLE_n (n=1-4)");
}

void setupXSHUTPins() {
    pinMode(XSHUT_PIN1, OUTPUT);
    pinMode(XSHUT_PIN2, OUTPUT);
    pinMode(XSHUT_PIN3, OUTPUT);
    pinMode(XSHUT_PIN4, OUTPUT);
    
    // Shutdown all sensors initially
    digitalWrite(XSHUT_PIN1, LOW);
    digitalWrite(XSHUT_PIN2, LOW);
    digitalWrite(XSHUT_PIN3, LOW);
    digitalWrite(XSHUT_PIN4, LOW);
    delay(10);
}

void initSensorConfigs() {
    for (int i = 0; i < 4; i++) {
        sensorConfigs[i].enabled = true;
        sensorConfigs[i].timingBudget = 50000;  // 50ms (moderate speed/accuracy)
        sensorConfigs[i].maxRange = 1500;       // 150cm maximum range
    }
}

bool initializeToFSensors() {
    bool allSuccess = true;
    
    // Initialize Sensor 1 (Front-Right)
    digitalWrite(XSHUT_PIN1, HIGH);
    delay(10);
    if (sensor1.init()) {
        sensor1.setAddress(SENSOR1_ADDRESS);
        configureSensor(&sensor1, 0);
        sensorData.sensorsActive[0] = true;
        Serial.println("Sensor 1 (Front-Right): OK");
    } else {
        Serial.println("Sensor 1 (Front-Right): FAILED");
        sensorData.sensorsActive[0] = false;
        allSuccess = false;
    }
    
    // Initialize Sensor 2 (Front-Left)
    digitalWrite(XSHUT_PIN2, HIGH);
    delay(10);
    if (sensor2.init()) {
        sensor2.setAddress(SENSOR2_ADDRESS);
        configureSensor(&sensor2, 1);
        sensorData.sensorsActive[1] = true;
        Serial.println("Sensor 2 (Front-Left): OK");
    } else {
        Serial.println("Sensor 2 (Front-Left): FAILED");
        sensorData.sensorsActive[1] = false;
        allSuccess = false;
    }
    
    // Initialize Sensor 3 (Back-Right)
    digitalWrite(XSHUT_PIN3, HIGH);
    delay(10);
    if (sensor3.init()) {
        sensor3.setAddress(SENSOR3_ADDRESS);
        configureSensor(&sensor3, 2);
        sensorData.sensorsActive[2] = true;
        Serial.println("Sensor 3 (Back-Right): OK");
    } else {
        Serial.println("Sensor 3 (Back-Right): FAILED");
        sensorData.sensorsActive[2] = false;
        allSuccess = false;
    }
    
    // Initialize Sensor 4 (Back-Left)
    digitalWrite(XSHUT_PIN4, HIGH);
    delay(10);
    if (sensor4.init()) {
        sensor4.setAddress(SENSOR4_ADDRESS);
        configureSensor(&sensor4, 3);
        sensorData.sensorsActive[3] = true;
        Serial.println("Sensor 4 (Back-Left): OK");
    } else {
        Serial.println("Sensor 4 (Back-Left): FAILED");
        sensorData.sensorsActive[3] = false;
        allSuccess = false;
    }
    
    return allSuccess;
}

void configureSensor(VL53L0X* sensor, int index) {
    sensor->setTimeout(500);
    sensor->setMeasurementTimingBudget(sensorConfigs[index].timingBudget);
    
    // Start continuous ranging
    sensor->startContinuous();
}

void readAllSensors() {
    sensorData.timestamp = millis();
    
    // Read Sensor 1 (Front-Right)
    if (sensorData.sensorsActive[0] && sensorConfigs[0].enabled) {
        uint16_t range = sensor1.readRangeContinuousMillimeters();
        if (sensor1.timeoutOccurred() || range > sensorConfigs[0].maxRange) {
            sensorData.distances[0] = -1; // Invalid reading
        } else {
            sensorData.distances[0] = range / 10.0; // Convert mm to cm
        }
    } else {
        sensorData.distances[0] = -1;
    }
    
    // Read Sensor 2 (Front-Left)
    if (sensorData.sensorsActive[1] && sensorConfigs[1].enabled) {
        uint16_t range = sensor2.readRangeContinuousMillimeters();
        if (sensor2.timeoutOccurred() || range > sensorConfigs[1].maxRange) {
            sensorData.distances[1] = -1;
        } else {
            sensorData.distances[1] = range / 10.0;
        }
    } else {
        sensorData.distances[1] = -1;
    }
    
    // Read Sensor 3 (Back-Right)
    if (sensorData.sensorsActive[2] && sensorConfigs[2].enabled) {
        uint16_t range = sensor3.readRangeContinuousMillimeters();
        if (sensor3.timeoutOccurred() || range > sensorConfigs[2].maxRange) {
            sensorData.distances[2] = -1;
        } else {
            sensorData.distances[2] = range / 10.0;
        }
    } else {
        sensorData.distances[2] = -1;
    }
    
    // Read Sensor 4 (Back-Left)
    if (sensorData.sensorsActive[3] && sensorConfigs[3].enabled) {
        uint16_t range = sensor4.readRangeContinuousMillimeters();
        if (sensor4.timeoutOccurred() || range > sensorConfigs[3].maxRange) {
            sensorData.distances[3] = -1;
        } else {
            sensorData.distances[3] = range / 10.0;
        }
    } else {
        sensorData.distances[3] = -1;
    }
}

void sendDataToOpenMV() {
    // Pack 4 float distances (16 bytes total)
    uint8_t buffer[16];
    
    for (int i = 0; i < 4; i++) {
        memcpy(buffer + (i * 4), &sensorData.distances[i], 4);
    }
    
    openMVSerial.write(buffer, 16);
}

void handleCommands() {
    // Handle commands from Serial Monitor
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command == "RESET") {
            Serial.println("Resetting all sensors...");
            resetAllSensors();
        }
        else if (command == "FAST") {
            Serial.println("Switching to fast mode (20ms timing budget)");
            setTimingBudget(20000);
        }
        else if (command == "ACCURATE") {
            Serial.println("Switching to accurate mode (100ms timing budget)");
            setTimingBudget(100000);
        }
        else if (command == "MODERATE") {
            Serial.println("Switching to moderate mode (50ms timing budget)");
            setTimingBudget(50000);
        }
        else if (command.startsWith("ENABLE_")) {
            int sensorNum = command.substring(7).toInt();
            if (sensorNum >= 1 && sensorNum <= 4) {
                sensorConfigs[sensorNum - 1].enabled = true;
                Serial.printf("Sensor %d enabled\n", sensorNum);
            }
        }
        else if (command.startsWith("DISABLE_")) {
            int sensorNum = command.substring(8).toInt();
            if (sensorNum >= 1 && sensorNum <= 4) {
                sensorConfigs[sensorNum - 1].enabled = false;
                Serial.printf("Sensor %d disabled\n", sensorNum);
            }
        }
        else if (command == "STATUS") {
            printSensorStatus();
        }
        else if (command == "CALIBRATE") {
            calibrateToFSensors();
        }
    }
    
    // Handle commands from OpenMV (optional)
    if (openMVSerial.available()) {
        String command = openMVSerial.readStringUntil('\n');
        // Process OpenMV commands if needed
    }
}

void resetAllSensors() {
    // Shutdown all sensors
    digitalWrite(XSHUT_PIN1, LOW);
    digitalWrite(XSHUT_PIN2, LOW);
    digitalWrite(XSHUT_PIN3, LOW);
    digitalWrite(XSHUT_PIN4, LOW);
    delay(10);
    
    // Reinitialize
    initializeToFSensors();
}

void setTimingBudget(uint32_t budget) {
    for (int i = 0; i < 4; i++) {
        sensorConfigs[i].timingBudget = budget;
    }
    
    // Apply to active sensors
    if (sensorData.sensorsActive[0]) sensor1.setMeasurementTimingBudget(budget);
    if (sensorData.sensorsActive[1]) sensor2.setMeasurementTimingBudget(budget);
    if (sensorData.sensorsActive[2]) sensor3.setMeasurementTimingBudget(budget);
    if (sensorData.sensorsActive[3]) sensor4.setMeasurementTimingBudget(budget);
    
    Serial.printf("Timing budget set to %lu microseconds\n", budget);
}

void printSensorStatus() {
    Serial.println("=== SENSOR STATUS ===");
    Serial.printf("Measurement Interval: %lu ms\n", MEASUREMENT_INTERVAL);
    Serial.printf("Last Reading Time: %lu ms\n", sensorData.timestamp);
    
    const char* sensorNames[] = {"Front-Right", "Front-Left", "Back-Right", "Back-Left"};
    
    for (int i = 0; i < 4; i++) {
        Serial.printf("Sensor %d (%s): ", i + 1, sensorNames[i]);
        Serial.printf("Active=%s, Enabled=%s, Distance=%.1f cm\n",
                     sensorData.sensorsActive[i] ? "Yes" : "No",
                     sensorConfigs[i].enabled ? "Yes" : "No",
                     sensorData.distances[i]);
    }
    
    Serial.printf("Uptime: %lu ms\n", millis());
    Serial.println("=====================");
}

void calibrateToFSensors() {
    Serial.println("=== ToF SENSOR CALIBRATION ===");
    Serial.println("Place robot at known distances from walls/obstacles");
    Serial.println("Press any key to start calibration sequence...");
    
    while (!Serial.available()) {
        delay(100);
    }
    Serial.read(); // Clear input
    
    int testDistances[] = {10, 20, 30, 50, 75, 100};
    int numTests = sizeof(testDistances) / sizeof(testDistances[0]);
    
    for (int d = 0; d < numTests; d++) {
        Serial.printf("\nPlace robot %d cm from obstacle, then press any key...", testDistances[d]);
        
        while (!Serial.available()) {
            delay(100);
        }
        Serial.read();
        
        // Take multiple readings
        float sums[4] = {0, 0, 0, 0};
        int validCounts[4] = {0, 0, 0, 0};
        
        for (int i = 0; i < 20; i++) {
            readAllSensors();
            
            for (int s = 0; s < 4; s++) {
                if (sensorData.distances[s] > 0) {
                    sums[s] += sensorData.distances[s];
                    validCounts[s]++;
                }
            }
            delay(100);
        }
        
        Serial.printf("Expected: %d cm\n", testDistances[d]);
        const char* names[] = {"FR", "FL", "BR", "BL"};
        for (int s = 0; s < 4; s++) {
            if (validCounts[s] > 0) {
                float average = sums[s] / validCounts[s];
                float error = average - testDistances[d];
                Serial.printf("  %s: %.1f cm (error: %.1f cm)\n", names[s], average, error);
            } else {
                Serial.printf("  %s: No valid readings\n", names[s]);
            }
        }
    }
    
    Serial.println("Calibration complete!");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Read sensors at specified interval
    if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
        readAllSensors();
        sendDataToOpenMV();
        
        // Debug output (reduced frequency)
        static unsigned long lastDebugTime = 0;
        if (currentTime - lastDebugTime >= 1000) { // Every second
            Serial.printf("ToF: FR=%.1f FL=%.1f BR=%.1f BL=%.1f cm\n",
                         sensorData.distances[0], sensorData.distances[1],
                         sensorData.distances[2], sensorData.distances[3]);
            lastDebugTime = currentTime;
        }
        
        lastMeasurement = currentTime;
    }
    
    // Handle commands
    handleCommands();
    
    // Small delay to prevent overwhelming the system
    delay(5);
}

// Additional utility functions
void performSensorTest() {
    Serial.println("=== SENSOR CONNECTIVITY TEST ===");
    
    for (int i = 0; i < 4; i++) {
        if (sensorData.sensorsActive[i]) {
            Serial.printf("Testing sensor %d... ", i + 1);
            
            // Take a few readings
            bool success = false;
            for (int attempt = 0; attempt < 5; attempt++) {
                uint16_t range;
                
                switch(i) {
                    case 0: range = sensor1.readRangeSingleMillimeters(); break;
                    case 1: range = sensor2.readRangeSingleMillimeters(); break;
                    case 2: range = sensor3.readRangeSingleMillimeters(); break;
                    case 3: range = sensor4.readRangeSingleMillimeters(); break;
                }
                
                if (range < 2000 && range > 10) { // Valid range
                    Serial.printf("OK (%.1f cm)\n", range / 10.0);
                    success = true;
                    break;
                }
                delay(100);
            }
            
            if (!success) {
                Serial.println("FAILED - No valid readings");
            }
        } else {
            Serial.printf("Sensor %d: NOT ACTIVE\n", i + 1);
        }
    }
    
    Serial.println("Test complete.");
}

void optimizeForRoboCup() {
    Serial.println("Optimizing sensors for RoboCup Junior Soccer...");
    
    // Set moderate timing budget for balance of speed and accuracy
    setTimingBudget(50000); // 50ms
    
    // Set maximum range appropriate for soccer field (122cm x 183cm)
    for (int i = 0; i < 4; i++) {
        sensorConfigs[i].maxRange = 1300; // 130cm max (field diagonal is ~220cm)
    }
    
    // Enable all sensors
    for (int i = 0; i < 4; i++) {
        sensorConfigs[i].enabled = true;
    }
    
    Serial.println("Sensors optimized for RoboCup field dimensions and requirements");
}

void enterLowPowerMode() {
    Serial.println("Entering low power mode - stopping continuous measurements");
    
    // Stop continuous measurements on all active sensors
    if (sensorData.sensorsActive[0]) sensor1.stopContinuous();
    if (sensorData.sensorsActive[1]) sensor2.stopContinuous();
    if (sensorData.sensorsActive[2]) sensor3.stopContinuous();
    if (sensorData.sensorsActive[3]) sensor4.stopContinuous();
    
    // You can add ESP32 sleep mode here if needed
}

void exitLowPowerMode() {
    Serial.println("Exiting low power mode - resuming continuous measurements");
    
    // Restart continuous measurements
    if (sensorData.sensorsActive[0]) sensor1.startContinuous();
    if (sensorData.sensorsActive[1]) sensor2.startContinuous();
    if (sensorData.sensorsActive[2]) sensor3.startContinuous();
    if (sensorData.sensorsActive[3]) sensor4.startContinuous();
}

// Advanced filtering function
float filterDistance(float newReading, float lastReading, int sensorIndex) {
    // Simple moving average filter
    static float readings[4][5] = {{0}}; // Store last 5 readings per sensor
    static int readingIndex[4] = {0};
    
    if (newReading < 0) return newReading; // Pass through invalid readings
    
    // Store new reading
    readings[sensorIndex][readingIndex[sensorIndex]] = newReading;
    readingIndex[sensorIndex] = (readingIndex[sensorIndex] + 1) % 5;
    
    // Calculate average
    float sum = 0;
    int count = 0;
    
    for (int i = 0; i < 5; i++) {
        if (readings[sensorIndex][i] > 0) {
            sum += readings[sensorIndex][i];
            count++;
        }
    }
    
    return count > 0 ? sum / count : newReading;
}

// Outlier detection
bool isOutlier(float reading, float expected, float tolerance) {
    return abs(reading - expected) > tolerance;
}

// Enhanced sensor data validation
void validateSensorData() {
    static float lastValidReadings[4] = {-1, -1, -1, -1};
    const float MAX_CHANGE_PER_CYCLE = 10.0; // Max cm change per measurement cycle
    
    for (int i = 0; i < 4; i++) {
        if (sensorData.distances[i] > 0 && lastValidReadings[i] > 0) {
            float change = abs(sensorData.distances[i] - lastValidReadings[i]);
            
            if (change > MAX_CHANGE_PER_CYCLE) {
                // Sudden large change detected - might be noise
                Serial.printf("Warning: Large change in sensor %d: %.1f -> %.1f cm\n", 
                             i + 1, lastValidReadings[i], sensorData.distances[i]);
                
                // Keep the last valid reading
                sensorData.distances[i] = lastValidReadings[i];
            } else {
                lastValidReadings[i] = sensorData.distances[i];
            }
        } else if (sensorData.distances[i] > 0) {
            lastValidReadings[i] = sensorData.distances[i];
        }
    }
}