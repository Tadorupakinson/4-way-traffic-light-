#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_PCF8574.h>



Adafruit_PCF8574 pcf20;
// Initialize the PCF8574 at I2C address (usually 0x20)
#define ledAGreen 0   // P0
#define ledAYellow 1  // P1
#define ledARed 2     // P2
#define trigPinA1 3   //P3
#define echoPinA1 4   //P4
#define trigPinA2 5   //P5
#define echoPinA2 6   //P6

Adafruit_PCF8574 pcf21;
#define ledBGreen 0   // P0
#define ledBYellow 1  // P1
#define ledBRed 2     // P2
#define trigPinB1 3   //P3
#define echoPinB1 4   //P4
#define trigPinB2 5   //P5
#define echoPinB2 6   //P6


Adafruit_PCF8574 pcf22;
#define ledCGreen 0   // P0
#define ledCYellow 1  // P1
#define ledCRed 2     // P2
#define trigPinC1 3   //P3
#define echoPinC1 4   //P4
#define trigPinC2 5   //P5
#define echoPinC2 6   //P6

Adafruit_PCF8574 pcf23;
#define ledDGreen 0   // P0
#define ledDYellow 1  // P1
#define ledDRed 2     // P2
#define trigPinD1 3   //P3
#define echoPinD1 4   //P4
#define trigPinD2 5   //P5
#define echoPinD2 6   //P6

int Ye2Re = 1000;

void setup() {
  Serial.begin(9600);
  wdt_enable(WDTO_8S);
  // Initialize PCF8574 at address 0x20
  if (!pcf20.begin(0x20)) {
    Serial.println("Couldn't find PCF8574 at 0x20");
    while (1)
      ;
  }
  if (!pcf21.begin(0x38)) {
    Serial.println("Couldn't find PCF8574 at 0x38");
    while (1)
      ;
  }
  if (!pcf22.begin(0x21)) {
    Serial.println("Couldn't find PCF8574 at 0x21");
    while (1)
      ;
  }
  if (!pcf23.begin(0x22)) {
    Serial.println("Couldn't find PCF8574 at 0x22");
    while (1)
      ;
  }


  // Set the LED pins as OUTPUT
  pcf20.pinMode(ledARed, OUTPUT);
  pcf20.pinMode(ledAYellow, OUTPUT);
  pcf20.pinMode(ledAGreen, OUTPUT);
  pcf20.pinMode(trigPinA1, OUTPUT);  // Sensor 1 Trig pin is OUTPUT
  pcf20.pinMode(echoPinA1, INPUT);   // Sensor 1 Echo pin is INPUT
  pcf20.pinMode(trigPinA2, OUTPUT);  // Sensor 2 Trig pin is OUTPUT
  pcf20.pinMode(echoPinA2, INPUT);   // Sensor 2 Echo pin is INPUT

  pcf21.pinMode(ledBRed, OUTPUT);
  pcf21.pinMode(ledBYellow, OUTPUT);
  pcf21.pinMode(ledBGreen, OUTPUT);
  pcf21.pinMode(trigPinB1, OUTPUT);  // Sensor 1 Trig pin is OUTPUT
  pcf21.pinMode(echoPinB1, INPUT);   // Sensor 1 Echo pin is INPUT
  pcf21.pinMode(trigPinB2, OUTPUT);  // Sensor 2 Trig pin is OUTPUT
  pcf21.pinMode(echoPinB2, INPUT);   // Sensor 2 Echo pin is INPUT

  pcf22.pinMode(ledCRed, OUTPUT);
  pcf22.pinMode(ledCYellow, OUTPUT);
  pcf22.pinMode(ledCGreen, OUTPUT);
  pcf22.pinMode(trigPinC1, OUTPUT);  // Sensor 1 Trig pin is OUTPUT
  pcf22.pinMode(echoPinC1, INPUT);   // Sensor 1 Echo pin is INPUT
  pcf22.pinMode(trigPinC2, OUTPUT);  // Sensor 2 Trig pin is OUTPUT
  pcf22.pinMode(echoPinC2, INPUT);   // Sensor 2 Echo pin is INPUT

  pcf23.pinMode(ledDRed, OUTPUT);
  pcf23.pinMode(ledDYellow, OUTPUT);
  pcf23.pinMode(ledDGreen, OUTPUT);
  pcf23.pinMode(trigPinD1, OUTPUT);  // Sensor 1 Trig pin is OUTPUT
  pcf23.pinMode(echoPinD1, INPUT);   // Sensor 1 Echo pin is INPUT
  pcf23.pinMode(trigPinD2, OUTPUT);  // Sensor 2 Trig pin is OUTPUT
  pcf23.pinMode(echoPinD2, INPUT);   // Sensor 2 Echo pin is INPUT
}

void loop() {
  wdt_reset();
  pcf20.digitalWrite(ledARed, HIGH);
  pcf21.digitalWrite(ledBRed, HIGH);
  pcf22.digitalWrite(ledCRed, HIGH);
  pcf23.digitalWrite(ledDRed, HIGH);
  // Measure distance with both ultrasonic sensors
  long distanceA1 = getUltrasonicDistance(pcf20, trigPinA1, echoPinA1);
  long distanceA2 = getUltrasonicDistance(pcf20, trigPinA2, echoPinA2);

  // Measure distance with both ultrasonic sensors for Road B (pcf21)
  long distanceB1 = getUltrasonicDistance(pcf21, trigPinB1, echoPinB1);
  long distanceB2 = getUltrasonicDistance(pcf21, trigPinB2, echoPinB2);

  long distanceC1 = getUltrasonicDistance(pcf22, trigPinC1, echoPinC1);
  long distanceC2 = getUltrasonicDistance(pcf22, trigPinC2, echoPinC2);

  long distanceD1 = getUltrasonicDistance(pcf23, trigPinD1, echoPinD1);
  long distanceD2 = getUltrasonicDistance(pcf23, trigPinD2, echoPinD2);

  
  Serial.println("Distance A1 A2 B1 B2 C1 C2 D1 D2 "); Serial.print("         ");
  Serial.print(distanceA1);   Serial.print(" ");
  Serial.print(distanceA2);   Serial.print(" ");
  Serial.print(distanceB1);   Serial.print(" ");
  Serial.print(distanceB2);   Serial.print(" ");
  Serial.print(distanceC1);   Serial.print(" ");
  Serial.print(distanceC2);   Serial.print(" ");
  Serial.print(distanceD1);   Serial.print(" ");
  Serial.print(distanceD2);   Serial.println();  // Move to the next line after printing



  // Check distance conditions
  if (distanceA1 <= 15) {
    roadOpen(pcf20, ledARed, ledAYellow, ledAGreen, distanceA1, distanceA2);
  }

  // Check distance conditions for road B and call the reusable roadOpen function
  if (distanceB1 <= 15) {
    roadOpen(pcf21, ledBRed, ledBYellow, ledBGreen, distanceB1, distanceB2);
  }
  if (distanceC1 <= 15) {
    roadOpen(pcf22, ledCRed, ledCYellow, ledCGreen, distanceC1, distanceC2);
  }
  if (distanceD1 <= 15) {
    roadOpen(pcf23, ledDRed, ledDYellow, ledDGreen, distanceD1, distanceD2);
  }
  delay(1000);  // Delay for 1 second before the next measurement
}

// Function to handle road A logic
void roadOpen(Adafruit_PCF8574 &pcf, uint8_t ledRed, uint8_t ledYellow, uint8_t ledGreen, long distance1, long distance2) {
  int delayTime = (distance1 <= 15 && distance2 <= 15) ? 7000 : 2000;

  // Turn off the red light and turn on the green light
  pcf.digitalWrite(ledRed, LOW);
  pcf.digitalWrite(ledGreen, HIGH);
  delay(3000);
  wdt_reset();
  delay(delayTime);

  // Turn off the green light and turn on the yellow light
  pcf.digitalWrite(ledGreen, LOW);
  pcf.digitalWrite(ledYellow, HIGH);
  delay(2000);

  // Turn off the yellow light and turn on the red light
  pcf.digitalWrite(ledYellow, LOW);
  pcf.digitalWrite(ledRed, HIGH);
  delay(1000);
  
}


// Function to measure distance using the ultrasonic sensor
long getUltrasonicDistance(Adafruit_PCF8574 &pcf, uint8_t trigPin, uint8_t echoPin) {
  long duration, distance;

  // Trigger the ultrasonic sensor
  pcf.digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  pcf.digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  pcf.digitalWrite(trigPin, LOW);

  // Read the echo pin
  duration = pulseInEchoPCF8574(pcf, echoPin, HIGH);

  // Calculate distance in cm (speed of sound is ~343 m/s or 29 microseconds per cm)
  distance = duration / 29 / 2;

  return distance;
}

// Modify pulseInEchoPCF8574 to accept the PCF8574 instance
unsigned long pulseInEchoPCF8574(Adafruit_PCF8574 &pcf, uint8_t pin, uint8_t state) {
  unsigned long startMicros = micros();

  // Wait for the pin to enter the desired state
  while (pcf.digitalRead(pin) != state) {
    if (micros() - startMicros > 1000000) {
      return 0;  // Timeout after 1 second if no pulse is detected
    }
  }

  // Measure the length of time the pin is in the desired state
  unsigned long start = micros();
  while (pcf.digitalRead(pin) == state) {
    if (micros() - start > 1000000) {
      return 0;  // Timeout after 1 second if no pulse ends
    }
  }

  return micros() - start;
}