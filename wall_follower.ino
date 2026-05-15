#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>


volatile unsigned long t1_ovf = 0;

ISR(TIMER1_OVF_vect) {
  t1_ovf++;
}

unsigned long custom_micros() {
  unsigned long m;
  uint8_t t;
  
  uint8_t oldSREG = SREG;
  cli();
  m = t1_ovf;
  t = TCNT1L;
  
  if ((TIFR1 & (1 << TOV1)) && (t < 255)) {
    m++;
  }
  SREG = oldSREG;
  
  return (m * 1024UL) + (t * 4UL);
}

unsigned long custom_millis() {
  return custom_micros() / 1000UL;
}

// Override standard Arduino functions locally
#define micros custom_micros
#define millis custom_millis

float TARGET_ANGLE = 30.0f;
float yawAngle = 0.0f;
float gyroZ_offset = 0.0f;
unsigned long lastGyroTime = 0;
unsigned long turnStartTime = 0;
unsigned long pushStartTime = 0;

// --- LOW LEVEL I2C ---
void i2c_init() {
  TWSR = 0x00; // Prescaler 1
  TWBR = 72;   // 100 kHz @ 16 MHz
  TWCR = (1 << TWEN);
}

void i2c_start() {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

void i2c_stop() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void i2c_write(uint8_t data) {
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_readAck() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

uint8_t i2c_readNak() {
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

void mpu_init() {
  i2c_init();
  
  // Wake up MPU6050
  i2c_start(); i2c_write(0xD0); i2c_write(0x6B); i2c_write(0x00); i2c_stop();
  
  // Set Gyro to +/- 500 deg/s
  i2c_start(); i2c_write(0xD0); i2c_write(0x1B); i2c_write(0x08); i2c_stop();

  // Calibrate Z-axis
  long sum = 0;
  for(int i=0; i<300; i++) {
    i2c_start(); i2c_write(0xD0); i2c_write(0x47);
    i2c_start(); i2c_write(0xD1);
    int16_t gz = (i2c_readAck() << 8) | i2c_readNak();
    i2c_stop();
    sum += gz;
    unsigned long wait_ms = millis();
    while (millis() - wait_ms < 2); // Wait 2ms
  }
  gyroZ_offset = sum / 300.0f;
  lastGyroTime = millis();
}

void mpu_update() {
  i2c_start(); i2c_write(0xD0); i2c_write(0x47);
  i2c_start(); i2c_write(0xD1);
  int16_t gz = (i2c_readAck() << 8) | i2c_readNak();
  i2c_stop();
  
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0f;
  lastGyroTime = now;
  
  float gz_deg = (gz - gyroZ_offset) / 65.5f;
  if (abs(gz_deg) > 1.0f) { // Ignore small noise
    yawAngle += gz_deg * dt;
  }
}

volatile uint8_t leftCount = 0;
volatile uint8_t rightCount = 0;

int TURN_PULSES_L  = 10;
int TURN_PULSES_R  = 10;

// THRESHOLDS

int FT = 27;          // front-stop distance (cm)
int FRONT_SLOW = 45;  // start braking below this (cm)  

// PID CONSTANTS  

float Kp          = 1.7f;
float Ki          = 0.0f;
float Kd          = 1.9f;

float TARGET_DIST = 15.0f;   // desired side-wall distance (cm)
float WALL_DETECT = 40.0f;   // side wall "present" threshold (cm)

// SPEEDS  

int BASE_SPEED_A = 170;
int BASE_SPEED_B = 160;
int MAX_SPEED  = 255;  // Increased to give PID headroom to speed up outer wheels
int MIN_SPEED  =  60;
int TURN_SPEED = 150;  // Increased for more torque (max 255)

// PID STATE VARIABLES

float pid_prev_error = 0.0f;
float pid_integral   = 0.0f;
unsigned long last_pid_ms = 0;

void pid_reset() {
  pid_prev_error = 0.0f;
  pid_integral   = 0.0f;
  last_pid_ms    = millis();
}

// FSM STATES

enum State {
  IDLE,
  MOVE_FORWARD,
  READ_LEFT_SENSOR,
  READ_RIGHT_SENSOR,
  PREPARE_DECISION,
  CHECK_TURN_DIR_DECIDE,
  TURN_LEFT,
  TURN_RIGHT,
  STOP
};

State currentState = IDLE;
State prevState    = IDLE;

const int MAX_TURNS = 64;

char turnSequence[MAX_TURNS];
int  turnCount = 0;

// UART HELPERS (Hardware USART0, 9600 baud @ 16MHz)

#define UART_RX_BUFFER_SIZE 64
volatile char uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uart_rx_head = 0;
volatile uint8_t uart_rx_tail = 0;

ISR(USART_RX_vect) {
  char c = UDR0;
  uint8_t next_head = (uart_rx_head + 1) % UART_RX_BUFFER_SIZE;
  if (next_head != uart_rx_tail) {
    uart_rx_buffer[uart_rx_head] = c;
    uart_rx_head = next_head;
  }
}

void uart_init() {
  uint16_t ubrr = 103;                           // 16000000 / (16 * 9600) - 1
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)(ubrr);
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Enable TX, RX, and RX Interrupt
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);       // 8-bit, no parity, 1 stop
}

void uart_tx(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void uart_print(const char* s) {
  while (*s) uart_tx(*s++);
}

void uart_println(const char* s) {
  uart_print(s);
  uart_tx('\r');
  uart_tx('\n');
}

void uart_print_long(long val) {
  char buf[12];
  ltoa(val, buf, 10);
  uart_print(buf);
}

void uart_println_long(long val) {
  uart_print_long(val);
  uart_tx('\r');
  uart_tx('\n');
}

void uart_print_int(int val) {
  char buf[8];
  itoa(val, buf, 10);
  uart_print(buf);
}

void uart_println_int(int val) {
  uart_print_int(val);
  uart_tx('\r');
  uart_tx('\n');
}

void uart_print_float(float val) {
  char buf[10];
  dtostrf(val, 4, 2, buf);
  uart_print(buf);
}

void uart_println_float(float val) {
  uart_print_float(val);
  uart_tx('\r');
  uart_tx('\n');
}

bool uart_available() {
  return (uart_rx_head != uart_rx_tail);
}

char uart_rx() {
  if (uart_rx_head == uart_rx_tail) return 0;
  char c = uart_rx_buffer[uart_rx_tail];
  uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
  return c;
}

String uart_readStringUntil(char terminator) {
  String str = "";
  unsigned long start = millis();
  while (millis() - start < 50) { // 50ms timeout for non-blocking
    if (uart_available()) {
      char c = uart_rx();
      if (c == terminator) break;
      str += c;
      start = millis(); // Reset timeout on valid character
    }
  }
  return str;
}

// PWM HELPERS (Timer1 — OC1A on PB1/D9, OC1B on PB2/D10)

void pwm_timer1_init() {
  // 8-bit Fast PWM, prescaler 64 (provides monotonic 4us ticks for custom_micros)
  TCCR1A = (1 << WGM10);                         // WGM10=1
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // WGM12=1 → Fast PWM 8-bit, prescaler 64
  TIMSK1 |= (1 << TOIE1);                        // Enable overflow interrupt for timekeeping
  OCR1A = 0;
  OCR1B = 0;
}

// Set PWM duty on ENA (pin 9 / OC1A)  — drives LEFT motor
void setPWM_A(uint8_t val) {
  if (val == 0) {
    TCCR1A &= ~(1 << COM1A1);     // Disconnect OC1A
    PORTB &= ~(1 << PB1);         // Pin LOW
  } else {
    TCCR1A |= (1 << COM1A1);      // Connect OC1A, non-inverting
    OCR1A = val;
  }
}

// Set PWM duty on ENB (pin 10 / OC1B)  — drives RIGHT motor
void setPWM_B(uint8_t val) {
  if (val == 0) {
    TCCR1A &= ~(1 << COM1B1);     // Disconnect OC1B
    PORTB &= ~(1 << PB2);         // Pin LOW
  } else {
    TCCR1A |= (1 << COM1B1);      // Connect OC1B, non-inverting
    OCR1B = val;
  }
}

// ENCODER ISRs (INT0 = ENC_RIGHT/PD2, INT1 = ENC_LEFT/PD3)

ISR(INT1_vect) {                         // ENC_LEFT on PD3
  static unsigned long lastTime = 0;
  unsigned long now = micros();
  if (now - lastTime > 500) {
    leftCount++;
  }
  lastTime = now;
}

ISR(INT0_vect) {                         // ENC_RIGHT on PD2
  static unsigned long lastTime = 0;
  unsigned long now = micros();
  if (now - lastTime > 500) {
    rightCount++;
  }
  lastTime = now;
}

// ULTRASONIC (all on PORTC — trig bits: 0,2,4  echo bits: 1,3,5)

long readUltrasonic(volatile uint8_t *port, volatile uint8_t *pinReg, uint8_t trigBit, uint8_t echoBit) {
  // Trigger pulse
  *port &= ~(1 << trigBit);               // TRIG LOW
  unsigned long t_wait = micros();
  while (micros() - t_wait < 2);          // Wait 2us
  *port |= (1 << trigBit);                // TRIG HIGH
  t_wait = micros();
  while (micros() - t_wait < 10);         // Wait 10us
  *port &= ~(1 << trigBit);               // TRIG LOW

  // pulseIn equivalent — measure HIGH pulse on echo, 10 ms timeout (non-blocking)
  unsigned long timeout = 10000;
  unsigned long start = micros();

  // Wait for any previous HIGH to end
  while (*pinReg & (1 << echoBit)) {
    if (micros() - start >= timeout) return 400;
  }

  // Wait for rising edge
  while (!(*pinReg & (1 << echoBit))) {
    if (micros() - start >= timeout) return 400;
  }

  unsigned long pulseStart = micros();

  // Wait for falling edge
  while (*pinReg & (1 << echoBit)) {
    if (micros() - pulseStart >= timeout) return 400;
  }

  long duration = micros() - pulseStart;

  if (duration == 0) {
    return 0;
  }

  long distance = duration * 0.034 / 2;

  if (distance <= 0) return 2;

  return distance;
}

// MOTOR CONTROL

void stopMotors() {
  setPWM_A(0);
  setPWM_B(0);
}

void forward() {
  // IN1(PB0)=HIGH, IN2(PB3)=LOW → left motor forward
  PORTB |=  (1 << PB0);
  PORTB &= ~(1 << PB3);

  // IN3(PD4)=HIGH, IN4(PD7)=LOW → right motor forward
  PORTD |=  (1 << PD4);
  PORTD &= ~(1 << PD7);
}

void turnLeft() {
  // IN1(PB0)=HIGH, IN2(PB3)=LOW
  PORTB |=  (1 << PB0);
  PORTB &= ~(1 << PB3);

  // IN3(PD4)=LOW, IN4(PD7)=HIGH → right motor reverse
  PORTD &= ~(1 << PD4);
  PORTD |=  (1 << PD7);

  setPWM_A(TURN_SPEED);
  setPWM_B(TURN_SPEED);
}

void turnRight() {
  // IN1(PB0)=LOW, IN2(PB3)=HIGH → left motor reverse
  PORTB &= ~(1 << PB0);
  PORTB |=  (1 << PB3);

  // IN3(PD4)=HIGH, IN4(PD7)=LOW
  PORTD |=  (1 << PD4);
  PORTD &= ~(1 << PD7);

  setPWM_A(TURN_SPEED);
  setPWM_B(TURN_SPEED);
}

void turnLeft90() {
  leftCount = 0;
  rightCount = 0;
  yawAngle = 0.0f;
  lastGyroTime = millis();
  unsigned long moveStartTime = millis();

  turnLeft(); // Starts both motors

  // Loop until target gyro angle is reached
  while (abs(yawAngle) < TARGET_ANGLE && (millis() - moveStartTime < 3000)) {
    mpu_update();

    // P-Controller to keep wheels synchronized during the turn
    int error = leftCount - rightCount; // positive if left is ahead
    int spd_L = constrain(TURN_SPEED - (error * 5), MIN_SPEED, 255);
    int spd_R = constrain(TURN_SPEED + (error * 5), MIN_SPEED, 255);
    setPWM_B((uint8_t)spd_L); // Left motor PWM
    setPWM_A((uint8_t)spd_R); // Right motor PWM
  }

  // Active brake both motors
  PORTD |= (1 << PD4) | (1 << PD7); // Active brake left motor
  PORTB |= (1 << PB0) | (1 << PB3); // Active brake right motor
  
  unsigned long brakeStart = millis();
  while (millis() - brakeStart < 30);
  stopMotors(); // Final safety stop
  
  // ----- SENSOR FEEDBACK (Extra Turn) -----
  // If the front is still blocked (meaning the turn was weak), nudge until clear
  unsigned long delayStart = millis();
  while (millis() - delayStart < 50);
  int frontDist = readUltrasonic(&PORTC, &PINC, PC0, PC1);
  unsigned long nudgeStart = millis();
  while (frontDist < FT && (millis() - nudgeStart < 1500)) {
    turnLeft();
    delayStart = millis();
    while (millis() - delayStart < 40);
    stopMotors();
    delayStart = millis();
    while (millis() - delayStart < 60);
    frontDist = readUltrasonic(&PORTC, &PINC, PC0, PC1);
  }
}

void turnRight90() {
  leftCount = 0;
  rightCount = 0;
  yawAngle = 0.0f;
  lastGyroTime = millis();
  unsigned long moveStartTime = millis();

  turnRight(); // Starts both motors

  // Loop until target gyro angle is reached
  while (abs(yawAngle) < TARGET_ANGLE && (millis() - moveStartTime < 3000)) {
    mpu_update();

    // P-Controller to keep wheels synchronized during the turn
    int error = leftCount - rightCount; // positive if left is ahead
    int spd_L = constrain(TURN_SPEED - (error * 5), MIN_SPEED, 255);
    int spd_R = constrain(TURN_SPEED + (error * 5), MIN_SPEED, 255);
    setPWM_B((uint8_t)spd_L); // Left motor PWM
    setPWM_A((uint8_t)spd_R); // Right motor PWM
  }

  // Active brake both motors
  PORTD |= (1 << PD4) | (1 << PD7); // Active brake left motor
  PORTB |= (1 << PB0) | (1 << PB3); // Active brake right motor
  
  unsigned long brakeStart = millis();
  while (millis() - brakeStart < 30);
  stopMotors(); // Final safety stop
  
  // ----- SENSOR FEEDBACK (Extra Turn) -----
  // If the front is still blocked (meaning the turn was weak), nudge until clear
  unsigned long delayStart = millis();
  while (millis() - delayStart < 50);
  int frontDist = readUltrasonic(&PORTC, &PINC, PC0, PC1);
  unsigned long nudgeStart = millis();
  while (frontDist < FT && (millis() - nudgeStart < 1500)) {
    turnRight();
    delayStart = millis();
    while (millis() - delayStart < 40);
    stopMotors();
    delayStart = millis();
    while (millis() - delayStart < 60);
    frontDist = readUltrasonic(&PORTC, &PINC, PC0, PC1);
  }
}

// RESULTS

void sendResults() {
  // Build and send via hardware UART (goes to Bluetooth on pins 0/1)
  uart_print("\n--------------------\n");
  uart_print("Turns: ");
  uart_print_int(turnCount);
  uart_print("\nSequence: ");

  for (int i = 0; i < turnCount; i++) {
    if (i > 0) {
      uart_print(", ");
    }
    uart_tx(turnSequence[i]);
  }

  uart_print("\n--------------------\n");
}

// SETUP

void setup() {
  // --- PORTC: ultrasonic pins ---
  DDRC |=  (1 << PC0) | (1 << PC2);
  DDRC &= ~((1 << PC1) | (1 << PC3));

  // --- PORTB: motor control + PWM ---
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);

  // --- PORTD: motor control + encoder inputs + right ultrasonic (D6, D5) ---
  DDRD |= (1 << PD4) | (1 << PD7) | (1 << PD6); // PD6 is Trig
  DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD5)); // PD5 is Echo

  // --- Timer1 PWM init ---
  pwm_timer1_init();

  // --- External Interrupts ---
  EICRA = (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00);
  EIMSK = (1 << INT1) | (1 << INT0);

  sei();  // Enable global interrupts

  // --- UART init (9600 baud) ---
  // This will handle both PC Serial Monitor and Bluetooth on pins 0/1
  uart_init();

  mpu_init(); // Initialize and calibrate gyro

  stopMotors();

  unsigned long start_d = millis();
  while (millis() - start_d < 20);

  currentState = IDLE;
}

// GLOBAL SENSOR DATA

long D = 0;
long L = 0;
long R = 0;
int count = 0;

void printState(){
  uart_print("STATE=");
  uart_print_int(currentState);

  uart_print(" D=");
  uart_print_long(D);

  uart_print(" L=");
  uart_print_long(L);

  uart_print(" R=");
  uart_println_long(R);
}

// BLUETOOTH COMMAND PROCESSING

void processBluetoothCommand(String btData) {
  char cmd = btData.charAt(0);
  if (cmd == 'P' || cmd == 'p') {
    Kp = btData.substring(1).toFloat();
    uart_print("Kp set to: "); uart_println_float(Kp);
  }
  else if (cmd == 'I' || cmd == 'i') {
    Ki = btData.substring(1).toFloat();
    uart_print("Ki set to: "); uart_println_float(Ki);
  }
  else if (cmd == 'D' || cmd == 'd') {
    Kd = btData.substring(1).toFloat();
    uart_print("Kd set to: "); uart_println_float(Kd);
  }
  else if (cmd == 'T' || cmd == 't') {
    TARGET_DIST = btData.substring(1).toFloat();
    uart_print("TARGET_DIST set to: "); uart_println_float(TARGET_DIST);
  }
  else if (cmd == 'A' || cmd == 'a') {
    BASE_SPEED_A = btData.substring(1).toInt();
    uart_print("BASE_SPEED_A set to: "); uart_println_int(BASE_SPEED_A);
  }
  else if (cmd == 'B' || cmd == 'b') {
    BASE_SPEED_B = btData.substring(1).toInt();
    uart_print("BASE_SPEED_B set to: "); uart_println_int(BASE_SPEED_B);
  }
  else if (cmd == 'F' || cmd == 'f') {
    FT = btData.substring(1).toInt();
    uart_print("FT set to: "); uart_println_int(FT);
  }
  else if (cmd == 'Q' || cmd == 'q') {
    TURN_SPEED = btData.substring(1).toInt();
    uart_print("Turn Speed set to: "); uart_println_int(TURN_SPEED);
  }
  else if (cmd == 'X' || cmd == 'x') {
    TURN_PULSES_L = btData.substring(1).toInt();
    uart_print("TURN_PULSES_L set to: "); uart_println_int(TURN_PULSES_L);
  }
  else if (cmd == 'Y' || cmd == 'y') {
    TURN_PULSES_R = btData.substring(1).toInt();
    uart_print("TURN_PULSES_R set to: "); uart_println_int(TURN_PULSES_R);
  }
  else if (cmd == 'G' || cmd == 'g') {
    TARGET_ANGLE = btData.substring(1).toFloat();
    uart_print("TARGET_ANGLE set to: "); uart_println_float(TARGET_ANGLE);
  }
  else if (cmd == 'C' || cmd == 'c') {
    uart_print("Kp="); uart_print_float(Kp);
    uart_print(" Ki="); uart_print_float(Ki);
    uart_print(" Kd="); uart_print_float(Kd);
    uart_print(" TARGET_DIST="); uart_print_float(TARGET_DIST);
    uart_print(" FT="); uart_print_int(FT);
    uart_print(" BASE_SPEED_A="); uart_print_int(BASE_SPEED_A);
    uart_print(" BASE_SPEED_B="); uart_print_int(BASE_SPEED_B);
    uart_print(" TURN_SPEED="); uart_print_int(TURN_SPEED);
    uart_print(" TURN_PULSES_L="); uart_print_int(TURN_PULSES_L);
    uart_print(" TURN_PULSES_R="); uart_print_int(TURN_PULSES_R);
    uart_print(" TARGET_ANGLE="); uart_println_float(TARGET_ANGLE);
  }
  else if (cmd == 's' || cmd == 'S') {
    if (currentState == IDLE) {
      uart_println("Starting Robot...");
      pid_reset();
      count = 0;
      currentState = MOVE_FORWARD;
    }
  }
  else if (cmd == 'r' || cmd == 'R') {
    uart_println("Resetting to Idle...");
    stopMotors();
    turnCount = 0;
    memset(turnSequence, 0, sizeof(turnSequence));
    count = 0;
    pid_reset();
    currentState = IDLE;
  }
}

// MAIN LOOP

void loop() {
  // --- Bluetooth commands via Hardware UART (Pins 0/1) ---
  if (uart_available()) {
    String btData = uart_readStringUntil('\n');
    btData.trim();
    if (btData.length() > 0) {
      processBluetoothCommand(btData);
    }
  }
  
  D = readUltrasonic(&PORTC, &PINC, PC0, PC1);     // Front: trig=PC0, echo=PC1
  
  switch(currentState) {
    case IDLE:
    stopMotors();
    if(count == 0){
        uart_println("Waiting...");
        count++;
      }
      break;

    case MOVE_FORWARD: {
      // Set direction registers: both motors forward (same as original)
      PORTB |=  (1 << PB0);
      PORTB &= ~(1 << PB3);
      PORTD |=  (1 << PD4);
      PORTD &= ~(1 << PD7);

      // Read side sensors
      L = readUltrasonic(&PORTC, &PINC, PC2, PC3);
      R = readUltrasonic(&PORTD, &PIND, PD6, PD5); // Right on D6(Trig) and D5(Echo)

      bool l_wall = (L < (long)WALL_DETECT);
      bool r_wall = (R < (long)WALL_DETECT);

      // Determine current tracking mode
      int current_mode = 0;
      if (l_wall && r_wall) current_mode = 3;
      else if (l_wall) current_mode = 1;
      else if (r_wall) current_mode = 2;

      static int prev_mode = 0;

      // ── Compute PID error ──────────────────────────────────────
      float error = 0.0f;
      if (current_mode == 3) {
        error = (float)(L - R);              // centre between both walls
      } else if (current_mode == 1) {
        error = (float)L - TARGET_DIST;      // keep left wall at TARGET_DIST
      } else if (current_mode == 2) {
        error = TARGET_DIST - (float)R;      // keep right wall at TARGET_DIST
      }
      // else: open space → error = 0 → drive straight

      // Prevent massive derivative spikes when sensors lose/find walls
      if (current_mode != prev_mode) {
        pid_prev_error = error;
      }
      prev_mode = current_mode;

      // ── dt (seconds) ──────────────────────────────────────────
      unsigned long t_now = millis();
      float dt = (float)(t_now - last_pid_ms) / 1000.0f;
      if (dt < 0.02f) break; // Replaces delay(20) - PID runs at max 50Hz
      last_pid_ms = t_now;

      // ── PID terms ─────────────────────────────────────────────
      pid_integral += error * dt;
      float derivative = (error - pid_prev_error) / dt;
      pid_prev_error = error;

      float correction = (Kp * error) + (Ki * pid_integral) + (Kd * derivative);

      // ── Base speed: taper as front wall approaches ─────────────
      int base_spd_a = BASE_SPEED_A;
      int base_spd_b = BASE_SPEED_B;
      if (D < FRONT_SLOW) {
        float t = (float)(D - FT) / (float)(FRONT_SLOW - FT);
        if (t < 0.0f) t = 0.0f;
        base_spd_a = MIN_SPEED + (int)(t * (float)(BASE_SPEED_A - MIN_SPEED));
        base_spd_b = MIN_SPEED + (int)(t * (float)(BASE_SPEED_B - MIN_SPEED));
      }

      // ── Apply correction ───────────────────────────────────────
      // Note: Hardware wiring is ENA (Motor A) = Right Motor, ENB (Motor B) = Left Motor
      // If correction > 0 (too close to right wall), steer left -> speed up A (Right), slow down B (Left)
      // If correction < 0 (too close to left wall), steer right -> slow down A (Right), speed up B (Left)
      int spd_A = constrain(base_spd_a + (int)correction, MIN_SPEED, MAX_SPEED);
      int spd_B = constrain(base_spd_b - (int)correction, MIN_SPEED, MAX_SPEED);

      setPWM_A((uint8_t)spd_A);   // ENA — right motor
      setPWM_B((uint8_t)spd_B);   // ENB — left motor

      // ── Front-wall check → hand off to turn logic ──────────────
      if (D < FT) {
        stopMotors();
        pid_reset();
        turnStartTime = millis();
        currentState = READ_LEFT_SENSOR;
      }
      break;
    }

    case READ_LEFT_SENSOR:
      if (millis() - turnStartTime >= 20) {
        L = readUltrasonic(&PORTC, &PINC, PC2, PC3);
        turnStartTime = millis();
        currentState =  READ_RIGHT_SENSOR;
      }
      break;

    case  READ_RIGHT_SENSOR:
      if (millis() - turnStartTime >= 20) {
        R = readUltrasonic(&PORTD, &PIND, PD6, PD5);
        turnStartTime = millis();
        currentState = PREPARE_DECISION;
      }
      break;

    case PREPARE_DECISION:
      if (millis() - turnStartTime >= 20) {
        currentState = CHECK_TURN_DIR_DECIDE;
      }
      break;

    case CHECK_TURN_DIR_DECIDE:
      if (D > FT) {
        currentState = MOVE_FORWARD;
      }
      else if ((L > 35) && (R <= 30)) {
        currentState = TURN_LEFT;
      }
      else if ((R > 30) && (L <= 35)) {
        currentState = TURN_RIGHT;
      }
      else if ((L > 35) && (R > 30)) {
        int leftMargin  = L - 35;
        int rightMargin = R - 30;

        if (leftMargin > rightMargin)
          currentState = TURN_LEFT;
        else
          currentState = TURN_RIGHT;
      }
      else {
        currentState = STOP;
      }
      break;

    case TURN_LEFT:
      uart_println("Turning Left");
      turnLeft90();
      
      // Push into the new corridor to align sensors
      forward();
      setPWM_A(BASE_SPEED_A);
      setPWM_B(BASE_SPEED_B);
      pushStartTime = millis();
      while(millis() - pushStartTime < 300);
      stopMotors();

      pid_reset();
      if (turnCount < MAX_TURNS)
        turnSequence[turnCount] = 'L';
      turnCount++;
      sendResults();
      currentState = MOVE_FORWARD;
      break;

    case TURN_RIGHT:
      uart_println("Turning Right");
      turnRight90();
      
      // Push into the new corridor to align sensors
      forward();
      setPWM_A(BASE_SPEED_A);
      setPWM_B(BASE_SPEED_B);
      pushStartTime = millis();
      while(millis() - pushStartTime < 300);
      stopMotors();

      pid_reset();
      if (turnCount < MAX_TURNS)
        turnSequence[turnCount] = 'R';
      turnCount++;
      sendResults();
      currentState = MOVE_FORWARD;
      break;

    case STOP:
      if(count == 0){
        uart_println("Track is Done");
        sendResults();
        count++;
      }
      break;
  }
}