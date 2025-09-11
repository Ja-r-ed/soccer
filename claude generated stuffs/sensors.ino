#include <Wire.h>
#include <VL53L0X.h>
#include <HardwareSerial.h>

// VL53L0X ToF sensors
VL53L0X sensor1;
VL53L0X sensor2;

// Sensor addresses after configuration
#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31

// GPIO pins for XSHUT (shutdown) control
#define XSHUT_PIN1 4
#define XSHUT_PIN2 5

// Serial communication with OpenMV
HardwareSerial openMVSerial(1);

// Timing
unsigned long lastMeasurement = 0;
const unsigned long MEASUREMENT_INTERVAL = 50; // 20Hz

// Data structure for sensor readings
struct SensorData {
    float distance1;
    float distance2;
    uint32_t timestamp;
};

void setup() {
    Serial.begin(115200);
    openMVSerial.begin(115200, SERIAL_8N1, 16, 17); // RX, TX pins
    
    // Initialize I2C
    Wire.begin();
    
    // Setup XSHUT pins
    pinMode(XSHUT_PIN1, OUTPUT);
    pinMode(XSHUT_PIN2, OUTPUT);
    
    Serial.println("Initializing ToF sensors...");
    
    // Initialize sensors with different addresses
    initializeSensors();
    
    Serial.println("ToF sensors initialized successfully");
}

void initializeSensors() {
    // Shutdown both sensors
    digitalWrite(XSHUT_PIN1, LOW);
    digitalWrite(XSHUT_PIN2, LOW);
    delay(10);
    
    // Initialize first sensor
    digitalWrite(XSHUT_PIN1, HIGH);
    delay(10);
    
    if (!sensor1.init()) {
        Serial.println("Failed to initialize sensor 1");
        while(1);
    }
    
    // Change address of first sensor
    sensor1.setAddress(SENSOR1_ADDRESS);
    
    // Initialize second sensor
    digitalWrite(XSHUT_PIN2, HIGH);
    delay(10);
    
    if (!sensor2.init()) {
        Serial.println("Failed to initialize sensor 2");
        while(1);
    }
    
    // Change address of second sensor
    sensor2.setAddress(SENSOR2_ADDRESS);
    
    // Configure sensors for high speed, medium accuracy
    sensor1.setTimeout(500);
    sensor2.setTimeout(500);
    
    // Set measurement timing budget (higher = more accurate but slower)
    sensor1.setMeasurementTimingBudget(50000); // 50ms
    sensor2.setMeasurementTimingBudget(50000); // 50ms
    
    // Start continuous ranging
    sensor1.startContinuous();
    sensor2.startContinuous();
    
    Serial.println("Both sensors configured and running");
}

SensorData readSensors() {
    SensorData data;
    data.timestamp = millis();
    
    // Read from both sensors
    uint16_t range1 = sensor1.readRangeContinuousMillimeters();
    uint16_t range2 = sensor2.readRangeContinuousMillimeters();
    
    // Convert to cm and apply basic filtering
    if (sensor1.timeoutOccurred() || range1 > 2000) {
        data.distance1 = -1; // Invalid reading
    } else {
        data.distance1 = range1 / 10.0; // mm to cm
    }
    
    if (sensor2.timeoutOccurred() || range2 > 2000) {
        data.distance2 = -1; // Invalid reading
    } else {
        data.distance2 = range2 / 10.0; // mm to cm
    }
    
    return data;
}

void sendDataToOpenMV(const SensorData& data) {
    // Pack data as floats for easy unpacking on OpenMV
    uint8_t buffer[8]; // 2 floats * 4 bytes each
    
    memcpy(buffer, &data.distance1, 4);
    memcpy(buffer + 4, &data.distance2, 4);
    
    openMVSerial.write(buffer, 8);
}

void loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
        // Read sensor data
        SensorData data = readSensors();
        
        // Send to OpenMV
        sendDataToOpenMV(data);
        
        // Debug output
        Serial.printf("Sensor1: %.1f cm, Sensor2: %.1f cm\n", 
                     data.distance1, data.distance2);
        
        lastMeasurement = currentTime;
    }
    
    // Check for commands from OpenMV (optional)
    if (openMVSerial.available()) {
        String command = openMVSerial.readStringUntil('\n');
        handleCommand(command);
    }
}

void handleCommand(String command) {
    command.trim();
    
    if (command == "RESET") {
        // Reset sensors
        Serial.println("Resetting sensors...");
        initializeSensors();
    } else if (command == "FAST") {
        // Switch to fast mode
        sensor1.setMeasurementTimingBudget(20000); // 20ms
        sensor2.setMeasurementTimingBudget(20000);
        Serial.println("Switched to fast mode");
    } else if (command == "ACCURATE") {
        // Switch to accurate mode
        sensor1.setMeasurementTimingBudget(100000); // 100ms
        sensor2.setMeasurementTimingBudget(100000);
        Serial.println("Switched to accurate mode");
    }
}