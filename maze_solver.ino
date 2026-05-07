#include <SoftwareSerial.h>
SoftwareSerial btSerial(12, 13);

// ============================================================
// ULTRASONIC PINS
// ============================================================

#define TRIG_F  A0
#define ECHO_F  A1

#define TRIG_L  A2
#define ECHO_L  A3

#define TRIG_R  A4
#define ECHO_R  A5

// ============================================================
// MOTOR PINS
// ============================================================

#define ENA 9
#define ENB 10

#define IN1 8
#define IN2 11
#define IN3 4
#define IN4 7

// ============================================================
// ENCODERS
// ============================================================

#define ENC_LEFT  3
#define ENC_RIGHT 2

volatile long leftCount = 0;
volatile long rightCount = 0;


const int TURN_PULSES_L  = 13;
const int TURN_PULSES_R  = 12;

// ============================================================
// THRESHOLDS
// ============================================================

const int FT = 18;

const int LT = 35;
const int RT = 30;

// ============================================================
// SPEEDS
// ============================================================

int speedA = 150;
int speedB = 129;

// ============================================================
// FSM STATES
// ============================================================

enum State {
  IDLE,
  MOVE_FORWARD,
  CHECK_TURN_DIR,
  TURN_LEFT,
  TURN_RIGHT,
  STOP,
  MEASURE
};

State currentState = IDLE;
State prevState = IDLE;

const int MAX_TURNS = 64;

char turnSequence[MAX_TURNS];
int turnCount = 0;

void countLeft() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = micros(); 
  
  if (interruptTime - lastInterruptTime > 500) {
    leftCount++;
  }
  lastInterruptTime = interruptTime;
}

void countRight() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = micros(); 
  
  if (interruptTime - lastInterruptTime > 500) {
    rightCount++;
  }
  lastInterruptTime = interruptTime;
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);

  if (duration == 0) {
    return 400; 
  }

  long distance = duration * 0.034 / 2;

  if (distance <= 0) return 2; 

  return distance;
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}


void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void turnLeft() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void turnRight() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void turnLeft90() {
  leftCount = 0;
  rightCount = 0;
  bool leftMoving = true;
  bool rightMoving = true;
  unsigned long moveStartTime = millis();

  turnLeft(); // Starts both motors

  // Loop until both motors have reached their targets or timeout occurs
  while ((leftMoving || rightMoving) && (millis() - moveStartTime < 2000)) {
    
    // Check Left Motor (In turnLeft, IN1/IN2 are usually active)
    if (leftMoving && leftCount >= TURN_PULSES_L) {
      analogWrite(ENB, 0); 
      leftMoving = false;
    }

    // Check Right Motor (In turnLeft, IN3/IN4 are usually active)
    if (rightMoving && rightCount >= TURN_PULSES_R) {
      analogWrite(ENA, 0);
      rightMoving = false;
    }
  }

  stopMotors(); // Final safety stop
}

void turnRight90() {
  leftCount = 0;
  rightCount = 0;
  bool leftMoving = true;
  bool rightMoving = true;
  unsigned long moveStartTime = millis();

  turnRight();

  while ((leftMoving || rightMoving) && (millis() - moveStartTime < 2000)) {
    
    if (leftMoving && leftCount >= TURN_PULSES_L) {
      analogWrite(ENB, 0);
      leftMoving = false;
    }

    if (rightMoving && rightCount >= TURN_PULSES_R) {
      analogWrite(ENA, 0);
      rightMoving = false;
    }
  }

  stopMotors();
}
void sendResults() {
  String msg = "Turns: ";
  msg += turnCount;
  msg += "\nSequence: ";

  for (int i = 0; i < turnCount; i++) {

    if (i > 0)
      msg += ", ";

    msg += turnSequence[i];
  }

  msg += "\n";

  Serial.print(msg);
  btSerial.print(msg);
}

void setup() {
  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_LEFT, INPUT);
  pinMode(ENC_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), countRight, RISING);

  Serial.begin(9600);
  btSerial.begin(9600);
  stopMotors();

  delay(20);

  currentState = IDLE;
}


long D = 0;
long L = 0;
long R = 0;
int count = 0;

void printState(){
  Serial.print("STATE=");
  Serial.print(currentState);

  btSerial.print("STATE=");
  btSerial.print(currentState);
  
  Serial.print(" D=");
  Serial.print(D);

  Serial.print(" L=");
  Serial.print(L);

  Serial.print(" R=");
  Serial.println(R);

  btSerial.print(" D=");
  btSerial.print(D);

  btSerial.print(" L=");
  btSerial.print(L);

  btSerial.print(" R=");
  btSerial.println(R);
}

void loop() {
  printState();

  switch(currentState) {
    case IDLE:
      stopMotors();
      delay(20);

      currentState = CHECK_TURN_DIR;
      break;

    case MOVE_FORWARD:
      btSerial.println("Forward");

      D = readUltrasonic(TRIG_F, ECHO_F);
      delay(10);

      forward();
      delay(20);

      // obstacle ahead
      if (D < FT) {
        stopMotors();
        delay(20);

        currentState = CHECK_TURN_DIR;
      }

      break;

    case CHECK_TURN_DIR:
      stopMotors();
      delay(20);

      D = readUltrasonic(TRIG_F, ECHO_F);
      delay(20);
      L = readUltrasonic(TRIG_L, ECHO_L);
      delay(20);
      R = readUltrasonic(TRIG_R, ECHO_R);
      delay(20);


      // front became open again
      if (D > FT) {
        currentState = MOVE_FORWARD;
      }

      // left open only
      else if ((L > LT) && (R <= RT)) {
        currentState = TURN_LEFT;
      }

      //right open only
      else if ((R > RT) && (L <= LT)) {
        currentState = TURN_RIGHT;
      }

      // both open
      else if ((L > LT) && (R > RT)) {

        int leftMargin  = L - LT;
        int rightMargin = R - RT;

        if (leftMargin > rightMargin)
          currentState = TURN_LEFT;
        else
          currentState = TURN_RIGHT;
      }

      // both blocked
      else {
        currentState = STOP;
      }
      break;

    case TURN_LEFT:
      btSerial.println("Turning Left");
      turnLeft90();

      if (turnCount < MAX_TURNS)
        turnSequence[turnCount] = 'L';

      turnCount++;

      currentState = MOVE_FORWARD;

      break;

    case TURN_RIGHT:
      btSerial.println("Turning Right");
      turnRight90();

      if (turnCount < MAX_TURNS)
        turnSequence[turnCount] = 'R';

      turnCount++;

      currentState = MOVE_FORWARD;

      break;
  
    case STOP:
      if(count == 0){
        sendResults();
        count++;
      }
      D = readUltrasonic(TRIG_F, ECHO_F);
      delay(20);
      L = readUltrasonic(TRIG_L, ECHO_L);
      delay(20);
      R = readUltrasonic(TRIG_R, ECHO_R);
      delay(20);
      break;
  }

}