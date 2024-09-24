#define BLYNK_TEMPLATE_ID "TMPL6F8VAgoED"
#define BLYNK_TEMPLATE_NAME "Checkpoint 2"
#define BLYNK_AUTH_TOKEN "-ULiRAN0B-XcEMAx1Y9fBoaiAc2CzV0w"
char ssid[] = "Hydrangeas "; // Your WiFi SSID
char pass[] = "12345678";   // Your WiFi password

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>

SoftwareSerial NodeMCU(D1, D2); // RX | TX

bool v0State = false;
bool v1State = false;
bool v2State = false;
bool v3State = false;
bool v4State = false;

void setup() {
  pinMode(D1, INPUT);
  pinMode(D2, OUTPUT);
  Serial.begin(9600);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass); // Connect to Blynk
  NodeMCU.begin(9600);
}

BLYNK_WRITE(V0) {
  int value = param.asInt();  // Get the value from Blynk (0 or 1)
  if (value == 1 && !v1State && !v2State && !v3State && !v4State) {
    Serial.println("A is ON");
    v0State = true;
  } else if (value == 0 && v0State) {
    Serial.println("A is OFF");
    v0State = false;
  }
}

BLYNK_WRITE(V1) {
  int value = param.asInt();
  if (value == 1 && !v0State && !v2State && !v3State && !v4State) {
    Serial.println("B is ON");
    v1State = true;
  } else if (value == 0 && v1State) {
    Serial.println("B is OFF");
    v1State = false;
  }
}

BLYNK_WRITE(V2) {
  int value = param.asInt();
  if (value == 1 && !v0State && !v1State && !v3State && !v4State) {
    Serial.println("C is ON");
    v2State = true;
  } else if (value == 0 && v2State) {
    Serial.println("C is OFF");
    v2State = false;
  }
}

BLYNK_WRITE(V3) {
  int value = param.asInt();
  if (value == 1 && !v0State && !v1State && !v2State && !v4State) {
    Serial.println("D is ON");
    v3State = true;
  } else if (value == 0 && v3State) {
    Serial.println("D is OFF");
    v3State = false;
  }
}

BLYNK_WRITE(V4) {
  int value = param.asInt();
  if (value == 1 && !v0State && !v1State && !v2State && !v3State) {
    Serial.println("Sleep mode is ON");
    v4State = true;
  } else if (value == 0 && v4State) {
    Serial.println("Sleep mode is OFF");
    v4State = false;
  }
}

void loop() {
  Blynk.run(); // Run Blynk

  if (v0State) {
    NodeMCU.println(1); // Continuously send 'A' to Arduino while A is ON
  }
  if (v1State) {
    NodeMCU.println(2); // Continuously send 'B' to Arduino while B is ON
  }
  if (v2State) {
    NodeMCU.println(3); // Continuously send 'C' to Arduino while C is ON
  }
  if (v3State) {
    NodeMCU.println(4); // Continuously send 'D' to Arduino while D is ON
  }
  if (v4State) {
    NodeMCU.println(5); // Continuously send 'Sleep mode ON' to Arduino while sleep mode is ON
  }

  delay(1500); // Adjust delay for the rate of sending data
}
