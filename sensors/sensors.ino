#include <HardwareSerial.h>
#include <Wire.h>

#define SLAVE_ADDR 0x42

// UART1 on pins 18 (RX), 19 (TX)
HardwareSerial OpenMVSerial(1);

// Fixed-size buffers
char uartBuffer[64];      // temporary UART line buffer
size_t uartIndex = 0;

char lastLineBuf[32] = "0.00 0.00 0.00 0.00       "; // always 32 bytes

void setup() {
    Serial.begin(115200); // USB debug

    // I2C slave setup (ESP32)
    Wire.begin(SLAVE_ADDR);        // initialize I2C as slave
    Wire.onRequest(requestEvent);  // attach request handler
    Serial.println("I2C Slave ready");

    // OpenMV UART setup
    OpenMVSerial.begin(115200, SERIAL_8N1, 18, 19);
}

void loop() {
    // Read UART bytes from OpenMV
    while (OpenMVSerial.available()) {
        char c = OpenMVSerial.read();

        if (c == '\n') {
            // End of line — copy into lastLineBuf safely
            size_t copyLen = uartIndex;
            if (copyLen > 32) copyLen = 32;

            // Copy into fixed buffer and pad with spaces if needed
            memcpy(lastLineBuf, uartBuffer, copyLen);
            for (size_t i = copyLen; i < 32; i++) {
                lastLineBuf[i] = ' ';
            }

            // Reset uartBuffer index
            uartIndex = 0;
        } else {
            if (uartIndex < sizeof(uartBuffer) - 1) {
                uartBuffer[uartIndex++] = c;
            }
        }
    }
    Serial.write((uint8_t*)lastLineBuf, 32);
    Serial.println();
}

// Interrupt-safe I2C response
void requestEvent() {
    Wire.write((uint8_t*)lastLineBuf, 32); // always 32 bytes
}
