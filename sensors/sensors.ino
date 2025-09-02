#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

// Joystick center and range
const float joystickCenter = XboxControllerNotificationParser::maxJoy / 2.0;
const float joystickRange = joystickCenter;  // = 32767.5

// Function prototype
float convert(uint16_t raw);

void setup() {
  pinMode(LeftX_PIN, OUTPUT);
  pinMode(LeftY_PIN, OUTPUT);
  pinMode(RightX_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println("Starting Xbox controller connection...");

  xboxController.begin();  // Initialize BLE client
}

void loop() {
  xboxController.onLoop();  // Must be called frequently

  if (xboxController.isConnected()) {
    float value[9] = {
      convert(xboxController.xboxNotif.joyLHori),
      convert(xboxController.xboxNotif.joyLVert),
      convert(xboxController.xboxNotif.joyRHori),
      convert(xboxController.xboxNotif.joyRVert),
      (float)xboxController.xboxNotif.btnA,
      (float)xboxController.xboxNotif.btnB,
      (float)xboxController.xboxNotif.btnX,
      (float)xboxController.xboxNotif.btnY,
      (float)xboxController.battery
    };

    Serial.printf("%f, %f, %f, %f, %f, %f, %f, %f %f\n",
                  value[0], value[1], value[2], value[3],
                  value[4], value[5], value[6], value[7],
                  value[8]);

    // Example: use analogWrite here if you're controlling something
    // analogWrite(LeftX_PIN, map(value[0], -1, 1, 0, 255));
  } else {
    Serial.println("Waiting for Xbox controller...");
    delay(1000);
  }
}

float convertbooleanforjoystick(float value) {
  if(value = -1) {return 0;}
  if(value != -1) {return 1;}
  else {return 3;}
}

// Convert joystick raw value (0–65535) to float in range -1.0 to 1.0
float convert(uint16_t raw) {
  return ((float)raw - joystickCenter) / joystickRange;
}
