#include <Arduino.h>
#include <SPI.h>

#define MISO 19
#define MOSI 23
#define SCK 18

#define START_BYTE 0xAA

uint8_t buffer[5];

void setup() {
  Serial.begin(115200);

  // Configure pins
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);

  SPI.begin();
  Serial.println("ESP32 SPI slave ready");

}

void loop() {
  // Print(buffer[0]);
  // Print(buffer[2]);
  // Print(buffer[3]);
  // Print(buffer[4]);
  // Print(buffer[5]);
  // Println(buffer[6]);


  for (int i = 0; i < 5; i++) {
    buffer[i] = SPI.transfer(0x00); // Send dummy byte, receive data
  Serial.print(buffer[1] << 8);
  Serial.print(" ");
  Serial.println(buffer[3] << 8);
  }
  //   // Check start byte
  // if (buffer[0] == START_BYTE) {
  //   int cx = (buffer[1] << 8) | buffer[2];
  //   int cy = (buffer[3] << 8) | buffer[4];

  //   Serial.print("Received coordinates: X=");
  //   Serial.print(cx);
  //   Serial.print(" Y=");
  //   Serial.println(cy);
  // }

    // Wait for CS to go HIGH before reading next frame
  }

