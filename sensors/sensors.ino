#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "OPPO Find X5";
const char* password = "8kswn2cn";

WiFiServer server(10000);  // server port to listen on

// Define Serial1 pins (adjust as needed)
#define SERIAL1_RX 16
#define SERIAL1_TX 17
#define LeftX 17
#define LeftY 16
#define RightX 4

void setup() {
  pinMode(LeftX, OUTPUT);
  pinMode(LeftY, OUTPUT);
  pinMode(RightX, OUTPUT);

  Serial.begin(9600);

  Serial.printf("Connecting to %s\n", ssid);
  Serial.printf("\nattempting to connect to WiFi network SSID '%s' password '%s' \n", ssid, password);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  server.begin();
  printWifiStatus();
  Serial.println(" listening on port 10000");
}

boolean alreadyConnected = false;

void loop() {
  static WiFiClient client;
  if (!client)
    client = server.available();
  if (client) {
    if (!alreadyConnected) {
      client.flush();
      Serial.println("We have a new client");
      alreadyConnected = true;
    }
    int length;
    float values[4];
    if ((length = client.available()) >= sizeof(values)) {
      size_t bytesRead = client.readBytes((char*)values, sizeof(values));
      if (bytesRead == sizeof(values)) {
        Serial.printf("%f, %f, %f, %f\n", values[0], values[1], values[2], values[3]);
        analogWrite(LeftX, 100+(values[0]*100));
        analogWrite(LeftY, 100-(values[1]*100));
        analogWrite(RightX, 100+(values[2]*100));
      } else {
        Serial.println("Error: Did not read 16 bytes for 4 floats.");
      }
      while (client.available()) client.read();
        } else if (length > 0) {
      while (client.available()) client.read();
    }
  }
}

void printWifiStatus() {
  Serial.print("\nSSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
