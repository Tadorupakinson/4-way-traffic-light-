#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_PCF8574.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include "RTClib.h"
SoftwareSerial Uno(3, 2);  // RX | TX
Adafruit_PCF8574 pcf20;    // Initialize PCF8574 for first set of LEDs
Adafruit_PCF8574 pcf21;    // Initialize PCF8574 for second set of LEDs
RTC_DS1307 rtc;
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
#define trigPinC1 8  
#define echoPinC1 9
#define trigPinD1 10
#define echoPinD1 11

void setup() {
  pinMode(3, INPUT);
  pinMode(2, OUTPUT);

  Uno.begin(9600);
  Serial.begin(9600);

  // Initialize PCF8574 at the specified addresses
  if (!pcf20.begin(0x20)) {
    Serial.println("Couldn't find PCF8574 at 0x20");
    while (1)
      ;
  }
  if (!pcf21.begin(0x21)) {
    Serial.println("Couldn't find PCF8574 at 0x21");
    while (1)
      ;
  }
 if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
  // Set the LED pins as OUTPUT
  for (int i = 0; i <= ledBRed; i++) {
    pcf20.pinMode(i, OUTPUT);
    pcf21.pinMode(i, OUTPUT);
  }
  // Set the sensor pins as OUTPUT
  pinMode(trigPinA1, OUTPUT);
  pinMode(echoPinA1, INPUT);

  pinMode(trigPinB1, OUTPUT);
  pinMode(echoPinB1, INPUT);

  pinMode(trigPinC1, OUTPUT);
  pinMode(echoPinC1, INPUT);

  pinMode(trigPinD1, OUTPUT);
  pinMode(echoPinD1, INPUT);
  Serial.println("--start-- ");
  for (int i = 0; i <= ledBRed; i++) {
    pcf20.digitalWrite(i, HIGH);
    pcf21.digitalWrite(i, HIGH);
  }
  wdt_enable(WDTO_8S);
}

void DoNormalTraffic();  // Declare the function prototype
void setzero();

void loop() {
  wdt_reset();
  checkAndSleep();
  int receivedValue = ReceiveData();  // Get the received value
  Serial.println(receivedValue);
  if (receivedValue == 0 || receivedValue == -1) {
    setzero();
    DoNormalTraffic();  // Handle normal traffic if no special command is received
  } else if (receivedValue > 0) {
    controlLEDs(receivedValue);  // Control LEDs based on the received value
  } 
  delay(1000);
}

void checkAndSleep() {
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  DateTime now = rtc.now();
  
  // Print current time
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);

  // Check if the time is between 22:00 and 04:00
  if (now.hour() >= 22 || now.hour() < 4) {
    Serial.println("Entering sleep mode...");
    enterSleepMode();
  }
}

int ReceiveData() {
  static String inputString = "";  // A string to hold incoming data
  while (Uno.available() > 0) {    // If there's incoming data
    char c = Uno.read();           // Read a character
    if (c == '\n') {
      int val = Uno.parseInt();
      return val;
    }
  }
  return -1;  // Return -1 if no complete data is received
}

void controlLEDs(int val) {
  // Turn on or off LEDs based on the received value
  switch (val) {
    case 1:  // Turn on ledAGreen
      pcf20.digitalWrite(ledARed, HIGH);
      pcf20.digitalWrite(ledAYellow, HIGH);
      pcf20.digitalWrite(ledAGreen, LOW);
      break;
    case 2:  // Turn off ledAGreen
      pcf20.digitalWrite(ledBRed, HIGH);
      pcf20.digitalWrite(ledBYellow, HIGH);
      pcf20.digitalWrite(ledBGreen, LOW);
      break;
    case 3:  // Turn on ledBGreen
      pcf21.digitalWrite(ledCRed, HIGH);
      pcf21.digitalWrite(ledCYellow, HIGH);
      pcf21.digitalWrite(ledCGreen, LOW);
      break;
    case 4:  // Turn off ledBGreen
      pcf21.digitalWrite(ledDRed, HIGH);
      pcf21.digitalWrite(ledDYellow, HIGH);
      pcf21.digitalWrite(ledDGreen, LOW);
      break;
    case 5:
      enterSleepMode();
      break;
    default:
      Serial.println("Unknown command");
      break;
  }
}

void DoNormalTraffic() {  // Correct function name
  
  long distanceA1 = getUltrasonicDistance(trigPinA1, echoPinA1);
  Serial.print("Distance A1: ");
  Serial.println(distanceA1);
  roadOpen(pcf20, ledARed, ledAYellow, ledAGreen, distanceA1);

  long distanceB1 = getUltrasonicDistance(trigPinB1, echoPinB1);
  Serial.print(" B1: ");
  Serial.println(distanceB1);
  roadOpen(pcf20, ledBRed, ledBYellow, ledBGreen, distanceB1);

  long distanceC1 = getUltrasonicDistance(trigPinC1, echoPinC1);
  Serial.print(" C1: ");
  Serial.println(distanceC1);
  roadOpen(pcf21, ledCRed, ledCYellow, ledCGreen, distanceC1);

  long distanceD1 = getUltrasonicDistance(trigPinD1, echoPinD1);
  Serial.print(" D1: ");
  Serial.println(distanceD1);
  roadOpen(pcf21, ledDRed, ledDYellow, ledDGreen, distanceD1);


  delay(1000);  // Delay for 1 second before the next measurement
}

void roadOpen(Adafruit_PCF8574 &pcf, uint8_t ledRed, uint8_t ledYellow, uint8_t ledGreen, long distance) {
  int receivedValue = ReceiveData();
  if (receivedValue > 1) {
    return;
  }

  int delayTime = (distance <= 5) ? 7000 : 2000;
  pcf.digitalWrite(ledRed, HIGH);
  pcf.digitalWrite(ledGreen, LOW);
  delay(delayTime);
  wdt_reset();
  delay(3000);  // Keep green on for 3 seconds
  pcf.digitalWrite(ledGreen, HIGH);
  pcf.digitalWrite(ledYellow, LOW);
  delay(2000);  // Keep yellow on for 2 seconds
  pcf.digitalWrite(ledYellow, HIGH);
  pcf.digitalWrite(ledRed, LOW);
  delay(2000);  // Keep red on for 1 second
  wdt_reset();
}

void setzero() {
  for (int i = 0; i <= ledBRed; i++) {
    pcf20.digitalWrite(i, HIGH);
    pcf21.digitalWrite(i, HIGH);
  }
  pcf20.digitalWrite(ledARed, LOW);
  pcf20.digitalWrite(ledBRed, LOW);
  pcf21.digitalWrite(ledCRed, LOW);
  pcf21.digitalWrite(ledDRed, LOW);
}

void enterSleepMode() {
  // Blink the yellow LED a few times before going to sleep
  for (int i = 0; i <= ledBRed; i++) {
    pcf20.digitalWrite(i, HIGH);
    pcf21.digitalWrite(i, HIGH);
  }
  pcf20.digitalWrite(ledAYellow, LOW);  // Turn the yellow LED on
  pcf20.digitalWrite(ledBYellow, LOW);  // Turn the yellow LED on
  pcf21.digitalWrite(ledCYellow, LOW);  // Turn the yellow LED on
  pcf21.digitalWrite(ledDYellow, LOW);  // Turn the yellow LED on
  // Enable the watchdog timer for sleep
  wdt_enable(WDTO_2S);  // Set the WDT timeout to 1 second

  // Sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();  // Enable sleep mode
  sleep_cpu();     // Put the Arduino to sleep

  // The processor will wake up here after the WDT timeout
  sleep_disable();  // Disable sleep mode
  wdt_reset();      // Reset the watchdog timer
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
