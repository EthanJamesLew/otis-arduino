#include <WiFi.h>
#include <Arduino.h>

/* AP parameters */
#define SSID "OTIS-bot"
#define PASSWORD "japery2019"
#define SERVER_PORT 4141

#define PACKET_SIZE 4

#define IS_SERVER

enum Protocol{
    TILT_SET, 
    YAW_SET
};

WiFiServer server(SERVER_PORT);
WiFiClient client;
size_t len;
uint16_t tiltNumber, yawNumber;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASSWORD);
  server.begin();  
}

void loop() {
  client = server.available();
  if (client){  
    if (client.available()) {
      uint8_t buffer[PACKET_SIZE];
      len = client.read(buffer, PACKET_SIZE);
      const uint16_t* buff16 = (const uint16_t*)buffer;
      tiltNumber = buff16[0];
      yawNumber = buff16[1];

      float tiltDecode = ((float)tiltNumber) /(65535.0f)*(2.0f) - 1.0f;
      float yawDecode = ((float)yawNumber) /(65535.0f)*(2.0f) - 1.0f;

      Serial.print("Received. Tilt:");
      Serial.print(tiltDecode);
      Serial.print(" Yaw:");
      Serial.println(yawDecode);
    }
  }
}
