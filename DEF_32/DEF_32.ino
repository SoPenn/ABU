const int MAX_DUTY = 255;

// เปิดหรือปิดการดีบัคที่นี่
#define DEBUG_ENABLE 1
#define DEBUG_PRINT(...) \
  do { \
    if (DEBUG_ENABLE) Serial.print(__VA_ARGS__); \
  } while (0)
#define DEBUG_PRINTLN(...) \
  do { \
    if (DEBUG_ENABLE) Serial.println(__VA_ARGS__); \
  } while (0)

HardwareSerial UART_IN(1);  // UART1 RX=16, TX=17

// Motor pins
const int motorPWMPins[] = { 4, 14, 18, 17 };
const int motorDIRPins[] = { 15, 12, 19, 5 };
// const int DEF_PWM = 32;
const int DEF_CW = 32;
const int DEF_CCW = 33;

int16_t lx, ly, rx;
uint16_t dpad, buttons;

void driveMotor(int pwmPin, int dirPin, float power, int channel) {
  if (fabs(power) < 0.01f) {
    digitalWrite(dirPin, LOW);
    ledcWrite(channel, 0);
    return;
  }

  bool forward = power >= 0;
  digitalWrite(dirPin, forward ? HIGH : LOW);
  int pwmVal = int(fabs(power) * MAX_DUTY);
  ledcWrite(channel, pwmVal);
}

void brakeAllMotors() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorDIRPins[i], LOW);
    ledcWrite(i, 0);
  }

  ledcWrite(4, 1);
  digitalWrite(DEF_CW, 1);
  digitalWrite(DEF_CCW, 1);
}

void MOVE_MENT() {
  float tx = constrain(lx / 512.0f, -1.0f, 1.0f);
  float ty = constrain(-ly / 512.0f, -1.0f, 1.0f);
  float rot = constrain(rx / 512.0f, -1.0f, 1.0f);

  if (fabs(tx) < 0.2f) tx = 0;
  if (fabs(ty) < 0.2f) ty = 0;
  if (fabs(rot) < 0.2f) rot = 0;

  if (abs(ty) > 0.4 && abs(tx) < 0.4) tx = 0;
  if (abs(ty) < 0.4 && abs(tx) > 0.4) ty = 0;

  float magnitude = sqrt(tx * tx + ty * ty);
  if (magnitude > 1.0f) {
    tx /= magnitude;
    ty /= magnitude;
  }

  float m1 = ty - tx - rot;
  float m2 = -(ty + tx + rot);
  float m3 = -(ty - tx + rot);
  float m4 = ty + tx - rot;

  float maxVal = max(max(fabs(m1), fabs(m2)), max(fabs(m3), fabs(m4)));
  if (maxVal > 1.0f) {
    m1 /= maxVal;
    m2 /= maxVal;
    m3 /= maxVal;
    m4 /= maxVal;
  }

  driveMotor(motorPWMPins[0], motorDIRPins[0], m1, 0);
  driveMotor(motorPWMPins[1], motorDIRPins[1], m2, 1);
  driveMotor(motorPWMPins[2], motorDIRPins[2], m3, 2);
  driveMotor(motorPWMPins[3], motorDIRPins[3], m4, 3);
}

void DEFENDER() {
  digitalWrite(DEF_CW, 1);
  digitalWrite(DEF_CCW, 1);

  if (buttons & 0x10) {  // L1 กด → หมุนตามเข็มนาฬิกา
    digitalWrite(DEF_CW, 0);
  }
  if (buttons & 0x40) {  // L2 กด → หมุนทวนเข็มนาฬิกา
    digitalWrite(DEF_CCW, 0);
  }
}

int16_t readInt16() {
  uint16_t raw = UART_IN.read() << 8 | UART_IN.read();
  return (int16_t)raw;
}

unsigned long lastUARTTime = 0;
const unsigned long UART_TIMEOUT = 100;

void processUART() {
  while (UART_IN.available()) {
    if (UART_IN.peek() == 0xAA) break;
    UART_IN.read();
  }

  if (UART_IN.available() >= 17 && UART_IN.peek() == 0xAA) {
    UART_IN.read();  // consume 0xAA header

    lx = readInt16();
    ly = readInt16();
    rx = readInt16();
    uint16_t throttle = UART_IN.read() << 8 | UART_IN.read();
    uint16_t brake = UART_IN.read() << 8 | UART_IN.read();
    dpad = UART_IN.read() << 8 | UART_IN.read();

    buttons = (uint32_t)UART_IN.read() << 24;
    buttons |= (uint32_t)UART_IN.read() << 16;
    buttons |= (uint32_t)UART_IN.read() << 8;
    buttons |= (uint32_t)UART_IN.read();

    lastUARTTime = millis();

    DEBUG_PRINT("LX: ");
    DEBUG_PRINT(lx);
    DEBUG_PRINT("\tLY: ");
    DEBUG_PRINT(ly);
    DEBUG_PRINT("\tRX: ");
    DEBUG_PRINT(rx);
    DEBUG_PRINT("\tThrottle: ");
    DEBUG_PRINT(throttle);
    DEBUG_PRINT("\tBrake: ");
    DEBUG_PRINT(brake);
    DEBUG_PRINT("\tDpad: ");
    DEBUG_PRINT(dpad, HEX);
    DEBUG_PRINT("\tButtons: ");
    DEBUG_PRINTLN(buttons, HEX);

    MOVE_MENT();
    DEFENDER();
  }
}

void setup() {
  Serial.begin(115200);

  UART_IN.begin(115200, SERIAL_8E1, 16, -1);

  for (int i = 0; i < 4; i++) {
    pinMode(motorDIRPins[i], OUTPUT);
    ledcSetup(i, 5000, 8);
    ledcAttachPin(motorPWMPins[i], i);
    ledcWrite(i, 0);
  }

  pinMode(DEF_CW, OUTPUT);
  pinMode(DEF_CCW, OUTPUT);
  ledcSetup(4, 5000, 8);
  ledcWrite(4, 0);
}

void loop() {
  processUART();

  if (millis() - lastUARTTime > UART_TIMEOUT) {
    brakeAllMotors();
  }
}
