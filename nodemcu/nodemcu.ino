#define BLYNK_TEMPLATE_ID "TMPL6F8VAgoED"
#define BLYNK_TEMPLATE_NAME "Checkpoint 2"
#define BLYNK_AUTH_TOKEN "-ULiRAN0B-XcEMAx1Y9fBoaiAc2CzV0w"
char ssid[] = "Hydrangeas "; // Your WiFi SSID
char pass[] = "12345678";     // Your WiFi password

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>

SoftwareSerial NodeMCU(D2, D3); // RX | TX

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass); // Connect to Blynk
  NodeMCU.begin(9600);
}

BLYNK_WRITE(V0) {
  int value = param.asInt();  // Get the value from Blynk (0 or 1)
  if (value == 1) {
    Serial.println("A is ON");
    NodeMCU.println(1); // Send 'A' to Arduino
  } else {
    Serial.println("A is OFF");
    NodeMCU.println(2); // Send 'a' to Arduino for OFF
  }
  delay(50); // Short delay for stability
}

BLYNK_WRITE(V1) {
  int value = param.asInt();
  if (value == 1) {
    Serial.println("B is ON");
    NodeMCU.println(3); // Send 'B' to Arduino
  } else {
    Serial.println("B is OFF");
    NodeMCU.println(4); // Send 'b' to Arduino for OFF
  }
  delay(50);
}

BLYNK_WRITE(V2) {
  int value = param.asInt();
  if (value == 1) {
    Serial.println("C is ON");
    NodeMCU.println(5); // Send 'C' to Arduino
  } else {
    Serial.println("C is OFF");
    NodeMCU.println(6); // Send 'c' to Arduino for OFF
  }
  delay(50);
}

BLYNK_WRITE(V3) {
  int value = param.asInt();
  if (value == 1) {
    Serial.println("D is ON");
    NodeMCU.println(7); // Send 'D' to Arduino
  } else {
    Serial.println("D is OFF");
    NodeMCU.println(8); // Send 'd' to Arduino for OFF
  }
  delay(50);
}

void loop() {
  Blynk.run(); // Run Blynk
}
