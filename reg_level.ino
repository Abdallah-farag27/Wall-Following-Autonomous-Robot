#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#define BT_TX_BIT  PB5
#define BT_RX_BIT  PB4
#define BT_TX_PORT PORTB
#define BT_RX_PIN  PINB

// 9600 Baud = 104.16us per bit. 
#define BIT_DELAY 102 

// ============================================================
// REGISTER-LEVEL BLUETOOTH TX
// ============================================================

void bt_tx_byte(uint8_t b) {
  uint8_t sreg = SREG;
  cli();
  BT_TX_PORT &= ~(1 << BT_TX_BIT); // Start bit
  _delay_us(BIT_DELAY);
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x01) BT_TX_PORT |= (1 << BT_TX_BIT);
    else BT_TX_PORT &= ~(1 << BT_TX_BIT);
    _delay_us(BIT_DELAY);
    b >>= 1;
  }
  BT_TX_PORT |= (1 << BT_TX_BIT); // Stop bit
  _delay_us(BIT_DELAY);
  SREG = sreg;
}

void bt_print(const char *s) {
  while (*s) bt_tx_byte((uint8_t)*s++);
}

void bt_println(const char *s) {
  bt_print(s);
  bt_tx_byte('\r');
  bt_tx_byte('\n');
}

void bt_print_int(int val) {
  char buf[8];
  itoa(val, buf, 10);
  bt_print(buf);
}

// ============================================================
// REGISTER-LEVEL BLUETOOTH RX
// ============================================================

uint8_t bt_rx_byte_immediate() {
  uint8_t b = 0;
  uint8_t sreg = SREG;
  cli();
  _delay_us(BIT_DELAY / 2); // Jump to middle of start bit
  for (uint8_t i = 0; i < 8; i++) {
    _delay_us(BIT_DELAY);
    if (BT_RX_PIN & (1 << BT_RX_BIT)) b |= (1 << i);
  }
  _delay_us(BIT_DELAY); // Wait for stop bit
  SREG = sreg;
  return b;
}

// ============================================================
// GLOBAL ROBOT DATA
// ============================================================

volatile long leftCount = 0;
volatile long rightCount = 0;
const int TURN_PULSES_L = 10;
const int TURN_PULSES_R = 10; 

const int FT = 25, LT = 35, RT = 30;
int speedA = 120, speedB = 110;
bool followRightWall = true;

enum State { IDLE, MOVE_FORWARD, CHECK_TURN_DIR, TURN_LEFT, TURN_RIGHT, STOP };
State currentState = IDLE;

const int MAX_TURNS = 64;
char turnSequence[MAX_TURNS];
int turnCount = 0;
int count = 0;

// ============================================================
// RESULTS FUNCTION (Updated for Specific Format)
// ============================================================

void sendResults() {
  bt_print("Turns: ");
  bt_print_int(turnCount);
  bt_println(""); // Move to next line

  bt_print("Sequence: ");
  for (int i = 0; i < turnCount; i++) {
    if (i > 0) bt_print(", ");
    bt_tx_byte((uint8_t)turnSequence[i]);
  }
  bt_println(""); // End with newline
}

// ============================================================
// MOTOR & SENSOR HELPERS
// ============================================================

void pwm_timer1_init() {
  TCCR1A = (1 << WGM10);
  TCCR1B = (1 << CS11) | (1 << CS10);
}

void setPWM_A(uint8_t val) {
  if (val == 0) { TCCR1A &= ~(1 << COM1A1); PORTB &= ~(1 << PB1); }
  else { TCCR1A |= (1 << COM1A1); OCR1A = val; }
}

void setPWM_B(uint8_t val) {
  if (val == 0) { TCCR1A &= ~(1 << COM1B1); PORTB &= ~(1 << PB2); }
  else { TCCR1A |= (1 << COM1B1); OCR1B = val; }
}

ISR(INT1_vect) { leftCount++; }
ISR(INT0_vect) { rightCount++; }

long readUltrasonic(uint8_t trigBit, uint8_t echoBit) {
  PORTC &= ~(1 << trigBit); _delay_us(2);
  PORTC |= (1 << trigBit); _delay_us(10);
  PORTC &= ~(1 << trigBit);
  unsigned long timeout = 20000, start = micros();
  while (PINC & (1 << echoBit)) if (micros() - start >= timeout) return 400;
  while (!(PINC & (1 << echoBit))) if (micros() - start >= timeout) return 400;
  unsigned long pStart = micros();
  while (PINC & (1 << echoBit)) if (micros() - start >= timeout) return 400;
  return (micros() - pStart) * 0.034 / 2;
}

void stopMotors() { setPWM_A(0); setPWM_B(0); }

void turnLeft90() {
  leftCount = 0; rightCount = 0;
  PORTB |= (1 << PB0); PORTB &= ~(1 << PB3); 
  PORTD &= ~(1 << PD4); PORTD |= (1 << PD7);
  setPWM_A(speedA); setPWM_B(speedB);
  unsigned long start = millis();
  while ((leftCount < TURN_PULSES_L || rightCount < TURN_PULSES_R) && (millis() - start < 2000));
  stopMotors();
}

void turnRight90() {
  leftCount = 0; rightCount = 0;
  PORTB &= ~(1 << PB0); PORTB |= (1 << PB3);
  PORTD |= (1 << PD4); PORTD &= ~(1 << PD7);
  setPWM_A(speedA); setPWM_B(speedB);
  unsigned long start = millis();
  while ((leftCount < TURN_PULSES_L || rightCount < TURN_PULSES_R) && (millis() - start < 2000));
  stopMotors();
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  DDRC |= (1 << PC0) | (1 << PC2) | (1 << PC4);
  DDRC &= ~((1 << PC1) | (1 << PC3) | (1 << PC5));
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << BT_TX_BIT);
  DDRB &= ~(1 << BT_RX_BIT);
  DDRD |= (1 << PD4) | (1 << PD7);
  PORTB |= (1 << BT_TX_BIT); 

  pwm_timer1_init();
  EICRA = (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00);
  EIMSK = (1 << INT1) | (1 << INT0);
  sei();

  stopMotors();
  currentState = IDLE;
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  switch (currentState) {
    case IDLE:
      stopMotors();
      if (count == 0) {
        bt_println("Waiting... Send 's' to start.");
        count++;
      }
      if (!(BT_RX_PIN & (1 << BT_RX_BIT))) {
        if (bt_rx_byte_immediate() == 's') {
          bt_println("Starting!");
          count = 0;
          currentState = MOVE_FORWARD;
        }
      }
      break;

    case MOVE_FORWARD:
      {
        long D = readUltrasonic(PC0, PC1);
        PORTB |= (1 << PB0); PORTB &= ~(1 << PB3);
        PORTD |= (1 << PD4); PORTD &= ~(1 << PD7);

        if (followRightWall) {
          long R = readUltrasonic(PC4, PC5);
          if (R < 13) { setPWM_A(speedA); setPWM_B(speedB - 15); }
          else if (R > 16) { setPWM_A(speedA - 15); setPWM_B(speedB); }
          else { setPWM_A(speedA); setPWM_B(speedB); }
        } else {
          long L = readUltrasonic(PC2, PC3);
          if (L < 13) { setPWM_A(speedA - 20); setPWM_B(speedB); }
          else if (L > 17) { setPWM_A(speedA); setPWM_B(speedB - 15); }
          else { setPWM_A(speedA); setPWM_B(speedB); }
        }
        if (D < FT) { stopMotors(); currentState = CHECK_TURN_DIR; }
      }
      break;

    case CHECK_TURN_DIR:
      {
        long D = readUltrasonic(PC0, PC1);
        long L = readUltrasonic(PC2, PC3);
        long R = readUltrasonic(PC4, PC5);
        if (D > FT) currentState = MOVE_FORWARD;
        else if ((L > LT) && (R <= RT)) currentState = TURN_LEFT;
        else if ((R > RT) && (L <= LT)) currentState = TURN_RIGHT;
        else if ((L > LT) && (R > RT)) currentState = ((L-LT) > (R-RT)) ? TURN_LEFT : TURN_RIGHT;
        else currentState = STOP;
      }
      break;

    case TURN_LEFT:
      bt_println("Turning Left"); // Status message
      turnLeft90();
      followRightWall = true;
      if (turnCount < MAX_TURNS) turnSequence[turnCount++] = 'L';
      sendResults(); // Format turns and sequence
      currentState = MOVE_FORWARD;
      break;

    case TURN_RIGHT:
      bt_println("Turning Right"); // Status message
      turnRight90();
      followRightWall = false;
      if (turnCount < MAX_TURNS) turnSequence[turnCount++] = 'R';
      sendResults(); // Format turns and sequence
      currentState = MOVE_FORWARD;
      break;

    case STOP:
      if (count == 0) {
        bt_println("Track is Done");
        sendResults(); 
        count++;
      }
      if (!(BT_RX_PIN & (1 << BT_RX_BIT))) {
        if (bt_rx_byte_immediate() == 'r') {
          bt_println("Resetting...");
          turnCount = 0;
          count = 0;
          memset(turnSequence, 0, sizeof(turnSequence));
          currentState = IDLE;
        }
      }
      break;
  }
}