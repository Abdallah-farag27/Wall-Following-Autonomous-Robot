#include <SoftwareSerial.h>
SoftwareSerial btSerial(12, 13);

// PIN DEFINITIONS (Match your current setup)
#define ENA 9
#define ENB 10
#define IN1 8
#define IN2 11
#define IN3 4
#define IN4 7
#define ENC_LEFT 3
#define ENC_RIGHT 2

volatile long leftCount = 0;
volatile long rightCount = 0;

void countLeft() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = micros(); // Use micros for high speed
  
  // Only count if pulses are more than 500 microseconds apart
  if (interruptTime - lastInterruptTime > 500) {
    leftCount++;
  }
  lastInterruptTime = interruptTime;
}
void countRight() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = micros(); // Use micros for high speed
  
  // Only count if pulses are more than 500 microseconds apart
  if (interruptTime - lastInterruptTime > 500) {
    rightCount++;
  }
  lastInterruptTime = interruptTime;
}

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  // Use INPUT_PULLUP to prevent the pin from "floating" and picking up noise
  // pinMode(ENC_LEFT, INPUT_PULLUP);
  // pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), countRight, RISING);

  Serial.println("Place robot on floor. Starting in 3 seconds...");
  btSerial.println("Place robot on floor. Starting in 3 seconds...");
  delay(3000);

  // Start a Tank Turn (Spin in place)
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // Left Back
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Right Forward
  analogWrite(ENA, 140);   // Use a slow, consistent speed
  analogWrite(ENB, 140);
}

void loop() {
  // Every half second, tell us how many pulses we've seen
  Serial.print("Left Pulses: ");
  Serial.print(leftCount);
  Serial.print(" | Right Pulses: ");
  Serial.println(rightCount);
  btSerial.print("Left Pulses: ");
  btSerial.print(leftCount);
  btSerial.print(" | Right Pulses: ");
  btSerial.println(rightCount);
  
  delay(500); 
}