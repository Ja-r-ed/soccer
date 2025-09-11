#include <HardwareSerial.h>

// Use UART1 on pins 18 (RX), 19 (TX)
HardwareSerial OpenMVSerial(1);

void setup() {
  Serial.begin(115200); // USB debug
  delay(500);

  // Init UART1 for OpenMV link
  OpenMVSerial.begin(115200, SERIAL_8N1, 18, 19);

  Serial.println("ESP32: Echo test started");
  OpenMVSerial.println("hello from ESP32");
}

void loop() {
  // Forward OpenMV → USB
  while (OpenMVSerial.available()) {
    char c = OpenMVSerial.read();
    Serial.write(c);
  }

  // Forward USB → OpenMV
  while (Serial.available()) {
    char c = Serial.read();
    OpenMVSerial.write(c);
  }
}