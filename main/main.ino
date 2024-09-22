#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_PCF8574.h>
#include <SoftwareSerial.h>

SoftwareSerial Uno(3, 2); // RX | TX
Adafruit_PCF8574 pcf20;   // Initialize PCF8574 for first set of LEDs
Adafruit_PCF8574 pcf21;   // Initialize PCF8574 for second set of LEDs

// LED Pin Definitions for PCF8574 at addresses 0x20 and 0x3E
#define ledAGreen 0
#define ledAYellow 1
#define ledARed 2
#define ledBGreen 3
#define ledBYellow 4
#define ledBRed 5

#define ledCGreen 0
#define ledCYellow 1
#define ledCRed 2
#define ledDGreen 3
#define ledDYellow 4
#define ledDRed 5

// Define the ultrasonic sensor pins
#define trigPinA1 4
#define echoPinA1 5
#define trigPinB1 6
#define echoPinB1 7
#define trigPinC1 8  // Assuming same pins for C; adjust as necessary
#define echoPinC1 9
#define trigPinD1 10
#define echoPinD1 11

void setup() {
  Serial.begin(9600);

  // Initialize PCF8574 at the specified addresses
  if (!pcf20.begin(0x20)) {
    Serial.println("Couldn't find PCF8574 at 0x20");
    while (1);
  }
  if (!pcf21.begin(0x3E)) {
    Serial.println("Couldn't find PCF8574 at 0x3E");
    while (1);
  }

  // Set the LED pins as OUTPUT
  for (int i = 0; i <= ledBRed; i++) {
    pcf20.pinMode(i, OUTPUT);
    pcf21.pinMode(i, OUTPUT);
  }
}

void DoNormalTraffic();  // Declare the function prototype

void loop() {
  int receivedValue = ReceiveData();  // Get the received value

  if (receivedValue == 0) {
    DoNormalTraffic();  // Handle normal traffic if no special command is received
  } else if (receivedValue > 0) {
    controlLEDs(receivedValue);  // Control LEDs based on the received value
  }
}

int ReceiveData() {
  static String inputString = "";     // A string to hold incoming data
  while (Uno.available() > 0) {       // If there's incoming data
    char c = Uno.read();              // Read a character
    if (c == '\n') {                  // If the character is a newline
      int val = inputString.toInt();  // Convert the string to an integer
      Serial.println(val);            // Print the value to Serial Monitor
      inputString = "";               // Clear the string for the next input
      return val;                     // Return the converted value
    } else {
      inputString += c;  // Add the character to the input string
    }
  }
  return -1;  // Return -1 if no complete data is received
}

void DoNormalTraffic() {  // Correct function name
  long distanceA1 = getUltrasonicDistance(trigPinA1, echoPinA1);
  long distanceB1 = getUltrasonicDistance(trigPinB1, echoPinB1);
  long distanceC1 = getUltrasonicDistance(trigPinC1, echoPinC1);
  long distanceD1 = getUltrasonicDistance(trigPinD1, echoPinD1);

  Serial.print("Distance A1: ");
  Serial.print(distanceA1);
  Serial.print(" B1: ");
  Serial.print(distanceB1);
  Serial.print(" C1: ");
  Serial.print(distanceC1);
  Serial.print(" D1: ");
  Serial.println(distanceD1);

  int distancelimit = 1000;  // Distance limit in cm

  if (distanceA1 <= distancelimit) {
    roadOpen(pcf20, ledARed, ledAYellow, ledAGreen, distanceA1);
  }
  if (distanceB1 <= distancelimit) {
    roadOpen(pcf20, ledBRed, ledBYellow, ledBGreen, distanceB1);
  }
  if (distanceC1 <= distancelimit) {
    roadOpen(pcf21, ledCRed, ledCYellow, ledCGreen, distanceC1);
  }
  if (distanceD1 <= distancelimit) {
    roadOpen(pcf21, ledDRed, ledDYellow, ledDGreen, distanceD1);
  }

  delay(1000);  // Delay for 1 second before the next measurement
}

void roadOpen(Adafruit_PCF8574 &pcf, uint8_t ledRed, uint8_t ledYellow, uint8_t ledGreen, long distance) {
  pcf.digitalWrite(ledRed, LOW);
  pcf.digitalWrite(ledGreen, HIGH);
  delay(3000);  // Keep green on for 3 seconds
  pcf.digitalWrite(ledGreen, LOW);
  pcf.digitalWrite(ledYellow, HIGH);
  delay(2000);  // Keep yellow on for 2 seconds
  pcf.digitalWrite(ledYellow, LOW);
  pcf.digitalWrite(ledRed, HIGH);
  delay(1000);  // Keep red on for 1 second
}

void controlLEDs(int val) {
  // Turn on or off LEDs based on the received value
  switch (val) {
    case 1:  // Turn on ledAGreen
      pcf20.digitalWrite(ledARed, LOW);
      pcf20.digitalWrite(ledAYellow, LOW);
      pcf20.digitalWrite(ledAGreen, HIGH);
      break;
    case 2:  // Turn off ledAGreen
      pcf20.digitalWrite(ledAGreen, LOW);
      pcf20.digitalWrite(ledAYellow, HIGH);
      delay(2000);
      pcf20.digitalWrite(ledAYellow, LOW);
      pcf20.digitalWrite(ledARed, HIGH);
      break;
    case 3:  // Turn on ledBGreen
      pcf20.digitalWrite(ledBRed, LOW);
      pcf20.digitalWrite(ledBYellow, LOW);
      pcf20.digitalWrite(ledBGreen, HIGH);
      break;
    case 4:  // Turn off ledBGreen
      pcf20.digitalWrite(ledBGreen, LOW);
      pcf20.digitalWrite(ledBYellow, HIGH);
      delay(2000);
      pcf20.digitalWrite(ledBYellow, LOW);
      pcf20.digitalWrite(ledBRed, HIGH);
      break;
    case 5:  // Turn on ledCGreen
      pcf21.digitalWrite(ledCRed, LOW);
      pcf21.digitalWrite(ledCYellow, LOW);
      pcf21.digitalWrite(ledCGreen, HIGH);
      break;
    case 6:  // Turn off ledCGreen
      pcf21.digitalWrite(ledCGreen, LOW);
      pcf21.digitalWrite(ledCYellow, HIGH);
      delay(2000);
      pcf21.digitalWrite(ledCYellow, LOW);
      pcf21.digitalWrite(ledCRed, HIGH);   // Use pcf21
      break;
    case 7:  // Turn on ledDGreen
      pcf21.digitalWrite(ledDRed, LOW);
      pcf21.digitalWrite(ledDYellow, LOW);
      pcf21.digitalWrite(ledDGreen, HIGH);
      break;
    case 8:  // Turn off ledDGreen
      pcf21.digitalWrite(ledDGreen, LOW);
      pcf21.digitalWrite(ledDYellow, HIGH);
      delay(2000);
      pcf21.digitalWrite(ledDYellow, LOW);
      pcf21.digitalWrite(ledDRed, HIGH);   
      break;
    default:
      Serial.println("Unknown command");
      break;
  }
}

long getUltrasonicDistance(uint8_t trigPin, uint8_t echoPin) {
  long duration, distance;

  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin
  duration = pulseIn(echoPin, HIGH);

  // Calculate distance in cm
  distance = duration / 29 / 2;

  return distance;
}
