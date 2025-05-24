const int MAX_DUTY = 255;

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
    ledcWrite(channel, 0);  // หยุดส่ง PWM
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

  // Defender motor stop
  ledcWrite(4, 0);
  digitalWrite(DEF_CW, LOW);
  digitalWrite(DEF_CCW, LOW);
}

void MOVE_MENT() {
  float tx = constrain(lx / 512.0f, -1.0f, 1.0f);
  float ty = constrain(ly / 512.0f, -1.0f, 1.0f);
  float rot = constrain(rx / 512.0f, -1.0f, 1.0f);

  // ✨ Deadzone
  if (fabs(tx) < 0.2f) tx = 0;
  if (fabs(ty) < 0.2f) ty = 0;
  if (fabs(rot) < 0.2f) rot = 0;

  // เงื่อนไขช่วยให้เคลื่อนที่เป็นเส้นตรง
  if (abs(ty) > 0.4 && abs(tx) < 0.4) tx = 0;
  if (abs(ty) < 0.4 && abs(tx) > 0.4) ty = 0;

  // Normalize vector
  float magnitude = sqrt(tx * tx + ty * ty);
  if (magnitude > 1.0f) {
    tx /= magnitude;
    ty /= magnitude;
  }

  float m1 = ty - tx - rot;   // Front Right
  float m2 = -(ty + tx + rot);   // Front Left
  float m3 = -(ty - tx + rot);  // Back Left
  float m4 = ty + tx - rot;  // Back Right

    // Normalize ค่ากำลังมอเตอร์
  float maxVal = max(max(fabs(m1), fabs(m2)), max(fabs(m3), fabs(m4)));
  if (maxVal > 1.0f) {
    m1 /= maxVal;
    m2 /= maxVal;
    m3 /= maxVal;
    m4 /= maxVal;
  }

  // ส่งค่าความเร็วที่ปรับแล้วไปควบคุมมอเตอร์
  driveMotor(motorPWMPins[0], motorDIRPins[0], m1, 0);
  driveMotor(motorPWMPins[1], motorDIRPins[1], m2, 1);
  driveMotor(motorPWMPins[2], motorDIRPins[2], m3, 2);
  driveMotor(motorPWMPins[3], motorDIRPins[3], m4, 3);
}

void DEFENDER() {
  digitalWrite(DEF_CW, 1);
  digitalWrite(DEF_CCW, 1);
  switch (dpad) {
    // case 0x00:
    //   // ledcWrite(4, 0);
    //   digitalWrite(DEF_CW, 1);
    //   digitalWrite(DEF_CCW, 1);
    //   break;
    case 0x01:  // Up = CW
      // ledcWrite(4, MAX_DUTY);
      digitalWrite(DEF_CW, 0);
      break;
    case 0x02:  // Down = CCW
      // ledcWrite(4, MAX_DUTY);
      digitalWrite(DEF_CCW, 0);
      break;
  }
}

int16_t readInt16() {
  uint16_t raw = UART_IN.read() << 8 | UART_IN.read();
  return (int16_t)raw;
}

void processUART() {
  // มองหา header 0xAA อย่างระมัดระวัง
  while (UART_IN.available()) {
    if (UART_IN.peek() == 0xAA) break;
    UART_IN.read();  // ทิ้ง byte จนกว่าจะเจอ 0xAA
  }

  // ตรวจสอบว่ามีข้อมูลครบ 17 ไบต์ก่อนอ่าน
  if (UART_IN.available() >= 17 && UART_IN.peek() == 0xAA) {
    UART_IN.read(); // consume 0xAA header

    lx = readInt16();
    ly = readInt16();
    rx = readInt16();
    // int16_t ry = readInt16(); // ยังไม่ใช้งาน

    uint16_t throttle = UART_IN.read() << 8 | UART_IN.read();
    uint16_t brake = UART_IN.read() << 8 | UART_IN.read();
    dpad = UART_IN.read() << 8 | UART_IN.read();

    // ปุ่ม 4 ไบต์
    buttons = (uint32_t)UART_IN.read() << 24;
    buttons |= (uint32_t)UART_IN.read() << 16;
    buttons |= (uint32_t)UART_IN.read() << 8;
    buttons |= (uint32_t)UART_IN.read();

    // 🔍 Debug
    Serial.print("LX: "); Serial.print(lx);
    Serial.print("\tLY: "); Serial.print(ly);
    Serial.print("\tRX: "); Serial.print(rx);
    Serial.print("\tThrottle: "); Serial.print(throttle);
    Serial.print("\tBrake: "); Serial.print(brake);
    Serial.print("\tDpad: "); Serial.print(dpad, HEX);
    Serial.print("\tButtons: "); Serial.println(buttons, HEX);

    MOVE_MENT();
    DEFENDER();
  }
}

void setup() {
  Serial.begin(115200);

  UART_IN.begin(115200, SERIAL_8E1, 16, -1);  // RX=16, TX ไม่ใช้

  // Motor setup
  for (int i = 0; i < 4; i++) {
    pinMode(motorDIRPins[i], OUTPUT);
    ledcSetup(i, 5000, 8);
    ledcAttachPin(motorPWMPins[i], i);
    ledcWrite(i, 0);  // Clear PWM เริ่มต้น
  }

  // Defender setup
  pinMode(DEF_CW, OUTPUT);
  pinMode(DEF_CCW, OUTPUT);
  ledcSetup(4, 5000, 8);  // Channel 4
  // ledcAttachPin(DEF_PWM, 4);  // Attach DEF_PWM to channel 4
  ledcWrite(4, 0);  // Clear PWM เริ่มต้น
}


void loop() {
  processUART();
}
