#include <Arduino.h>

// #define ENABLE_DEBUG // ถ้าอยากเปิด debug ให้ uncomment บรรทัดนี้

#ifdef ENABLE_DEBUG
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)    do {} while(0)
  #define DEBUG_PRINTLN(...)  do {} while(0)
#endif

const int MAX_DUTY = 4095;

HardwareSerial UART_IN(1);  // UART1 RX=16, TX=17

// Motor pins
const int motorPWMPins[] = { 4, 14, 18, 17 };  // pwm1 , pwm2 , pwm3 , pwm4
const int motorDIRPins[] = { 15, 12, 19, 5 };  // dir1 , dir2 , dir3 , dir4

// relay Active HIGH
const int linear_UP = 32;    // ปรับองศา relay ch 1
const int linear_DOWN = 33;  // ปรับองศา relay ch 2

const int Cylinder_PUSH = 25;         // ดันบอล   relay ch3
const int Cylinder_Bounce_ball = 26;  // เดาะบอล  relay ch4
const int Cylinder_Receive = 27;      // รับบอล    relay ch5

// ชุดยิง cytron 20A ชุดยิง
const int cytronPWM1 = 21;  // ledcWrite   channel 4
const int cytronDIR1 = 22;
const int cytronPWM2 = 23;  // ledcWrite   channel 5
const int cytronDIR2 = 2;

int MAXPWM = 4095;
int MINPWM = 0;
int pwmVal1 = 0;
int level = 0;

bool lastBrakeState = false;

int16_t lx, ly, rx;
uint16_t dpad, buttons;

unsigned long lastUARTTime = 0;
const unsigned long UART_TIMEOUT = 100;  // ms

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
}

// ตัวคูณกำลังของมอเตอร์แต่ละตัว
const float motorGain[4] = { // จูนมอเตอร์ตรงนี้
  1.0f,  // Motor 1 (Front Right)
  1.0f,  // Motor 2 (Front Left)
  1.0f,  // Motor 3 (Back Left)
  1.0f   // Motor 4 (Back Right)
};

void MOVE_MENT() {
  float tx = constrain(lx / 512.0f, -1.0f, 1.0f);
  float ty = constrain(ly / 512.0f, -1.0f, 1.0f);
  float rot = constrain(rx / 256.0f, -1.0f, 1.0f);

  // Deadzone
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

  float m1 = ty - tx - rot;     // Front Right
  float m2 = -(ty + tx + rot);  // Front Left
  float m3 = -(ty - tx + rot);  // Back Left
  float m4 = ty + tx - rot;     // Back Right

  // Normalize ค่ากำลังมอเตอร์
  float maxVal = max(max(fabs(m1), fabs(m2)), max(fabs(m3), fabs(m4)));
  if (maxVal > 1.0f) {
    m1 /= maxVal;
    m2 /= maxVal;
    m3 /= maxVal;
    m4 /= maxVal;
  }

  // เพิ่มการคูณด้วย gain สำหรับมอเตอร์แต่ละตัว
  m1 *= motorGain[0];
  m2 *= motorGain[1];
  m3 *= motorGain[2];
  m4 *= motorGain[3];

  // Normalize อีกรอบหลังคูณ gain
  float maxAfterGain = max(max(fabs(m1), fabs(m2)), max(fabs(m3), fabs(m4)));
  if (maxAfterGain > 1.0f) {
    m1 /= maxAfterGain;
    m2 /= maxAfterGain;
    m3 /= maxAfterGain;
    m4 /= maxAfterGain;
  }

  // ส่งค่าความเร็วที่ปรับแล้วไปควบคุมมอเตอร์
  driveMotor(motorPWMPins[0], motorDIRPins[0], m1, 0);
  driveMotor(motorPWMPins[1], motorDIRPins[1], m2, 1);
  driveMotor(motorPWMPins[2], motorDIRPins[2], m3, 2);
  driveMotor(motorPWMPins[3], motorDIRPins[3], m4, 3);
}

void Shooting(uint16_t brake, uint16_t throttle) {
  digitalWrite(linear_UP, LOW);
  digitalWrite(linear_DOWN, LOW);
  digitalWrite(Cylinder_PUSH, LOW);
  digitalWrite(Cylinder_Bounce_ball, LOW);
  digitalWrite(cytronDIR1, LOW);
  digitalWrite(cytronDIR2, LOW);
  static bool toggleCylinderReceive = false;
  static bool lastButtonXState = false;

  bool currentButtonXState = buttons & 0x01;
  switch (dpad) {
    case 0x01:                        // Up = CW
      digitalWrite(linear_UP, HIGH);  // ปรับองศา ขึ้น
      break;
    case 0x02:  // ปรับองศา ลง
      digitalWrite(linear_DOWN, HIGH);
      break;
  }

  bool currentBrakeState = (brake == 1020);  // หรือแล้วแต่จอยส่งค่าอะไรตอนกด L2
  if (currentBrakeState && !lastBrakeState) {
    level++;
    if (level > 4) level = 0;
  }
  lastBrakeState = currentBrakeState;

  switch (level) {
    case 0: pwmVal1 = 0; break;     // 0%
    case 1: pwmVal1 = 1023; break;  // 25% 64
    case 2: pwmVal1 = 2046; break;  // 50% 128
    case 3: pwmVal1 = 3092; break;  // 75% 191
    case 4: pwmVal1 = 4095; break;  // 100% 255
  }

  switch (throttle) {
    case 1020:
      digitalWrite(Cylinder_PUSH, HIGH);
      break;
  }

  if (buttons & 0x04) {
    digitalWrite(Cylinder_Bounce_ball, HIGH);
  }
  if (buttons & 0x02) {
    level = 0;
  }

  if (currentButtonXState && !lastButtonXState) {
    toggleCylinderReceive = !toggleCylinderReceive;
    digitalWrite(Cylinder_Receive, toggleCylinderReceive ? HIGH : LOW);
  }

  lastButtonXState = currentButtonXState;
  ledcWrite(4, pwmVal1);
  ledcWrite(5, pwmVal1);
}

int16_t readInt16() {
  uint16_t raw = UART_IN.read() << 8 | UART_IN.read();
  return (int16_t)raw;
}

void processUART() {
  if (UART_IN.available() >= 17) {
    uint8_t start = UART_IN.read();
    if (start == 0xAA) {
      lastUARTTime = millis();  // อัปเดตเวลาที่ได้รับ UART ล่าสุด

      lx = UART_IN.read() << 8 | UART_IN.read();
      ly = UART_IN.read() << 8 | UART_IN.read();
      rx = UART_IN.read() << 8 | UART_IN.read();
      uint16_t throttle = UART_IN.read() << 8 | UART_IN.read();
      uint16_t brake = UART_IN.read() << 8 | UART_IN.read();
      dpad = UART_IN.read() << 8 | UART_IN.read();
      buttons = (uint32_t)UART_IN.read() << 24;
      buttons |= (uint32_t)UART_IN.read() << 16;
      buttons |= (uint32_t)UART_IN.read() << 8;
      buttons |= (uint32_t)UART_IN.read();

      DEBUG_PRINT("Level: ");
      DEBUG_PRINT(level);
      DEBUG_PRINT("  PWM: ");
      DEBUG_PRINTLN(pwmVal1);

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
      Shooting(brake, throttle);
    }
  }
}

void setup() {
  Serial.begin(115200);

  UART_IN.begin(115200, SERIAL_8E1, 16, -1);  // RX=16, TX ไม่ใช้

  // Motor setup
  for (int i = 0; i < 4; i++) {
    pinMode(motorDIRPins[i], OUTPUT);
    ledcSetup(i, 5000, 12);
    ledcAttachPin(motorPWMPins[i], i);
    ledcWrite(i, 0);  // Clear PWM เริ่มต้น
  }

  // Shooting setup
  ledcAttachPin(cytronPWM1, 4);
  ledcSetup(4, 5000, 12);  
  ledcAttachPin(cytronPWM2, 5);
  ledcSetup(5, 5000, 12);  

  // Defender setup
  pinMode(linear_UP, OUTPUT);
  pinMode(linear_DOWN, OUTPUT);
  pinMode(Cylinder_PUSH, OUTPUT);
  pinMode(Cylinder_Bounce_ball, OUTPUT);
  pinMode(Cylinder_Receive, OUTPUT);
  pinMode(cytronDIR1, OUTPUT);
  pinMode(cytronDIR2, OUTPUT);
}

void loop() {
  processUART();

  // ถ้าไม่ได้รับ UART ภายใน 100ms ให้หยุดมอเตอร์
  if (millis() - lastUARTTime > UART_TIMEOUT) {
    brakeAllMotors();
    ledcWrite(4, 0);  // หยุดชุดยิงด้วย
    ledcWrite(5, 0);
  }
}
