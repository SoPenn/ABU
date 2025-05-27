#define BP32_LOG_LEVEL 0
#include <Arduino.h>
#include <Bluepad32.h>
#include "esp_bt.h"

#define ENABLE_DEBUG 0 // ถ้าอยากเปิด debug: uncomment บรรทัดนี้ หรือใช้ 0

#ifdef ENABLE_DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) \
  do { \
  } while (0)
#define DEBUG_PRINTLN(...) \
  do { \
  } while (0)
#define DEBUG_PRINTF(...) \
  do { \
  } while (0)
#endif

ControllerPtr activeCtl = nullptr;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// PS5 VID & PID
constexpr uint16_t VID_PS5 = 0x054c;
constexpr uint16_t PID_PS5 = 0x0ce6;

// UART
HardwareSerial UART_OUT(1);  // TX=17, RX=16

void processGamepad(ControllerPtr ctl) {
  static bool brakePressedLast = false;
  static int led = 0;

  ctl->setPlayerLEDs(led & 0x0f);
  bool brakeNow = (ctl->brake() == 1020);
  uint32_t buttons = ctl->buttons();

  if (brakeNow && !brakePressedLast) {
    led = (led + 1) % 5;
  }
  brakePressedLast = brakeNow;

  // ปุ่ม O รีเซตสี
  if (buttons & 0x0002) {
    led = 0;
  }

  switch (led) {
    case 1: ctl->setColorLED(0, 255, 0); break;       // เขียว
    case 2: ctl->setColorLED(200, 255, 0); break;     // เหลือง
    case 3: ctl->setColorLED(255, 165, 0); break;     // ส้ม
    case 4: ctl->setColorLED(255, 0, 0); break;       // แดง
    default: ctl->setColorLED(255, 255, 255); break;  // ขาว
  }
}

void onConnectedController(ControllerPtr ctl) {
  auto props = ctl->getProperties();
  ctl->setRumble(0x50, 0x50);
  delay(500);

  DEBUG_PRINTLN("New controller connected");

  // ตรวจสอบว่าเป็น PS5
  if (props.vendor_id == VID_PS5 && props.product_id == PID_PS5) {
    if (!activeCtl || !activeCtl->isConnected()) {
      activeCtl = ctl;
      DEBUG_PRINTLN("PS5 controller accepted and set as activeCtl");
      // หยุด callback ตัวอื่น
      BP32.setup(nullptr, nullptr);

      BP32.disconnectNotUsedGamepads();

      ctl->setPlayerLEDs(0x04);
      ctl->setPlayerLEDs(0x00);
    } else {
      DEBUG_PRINTLN("Another controller is already active. Disconnecting this one.");
      ctl->disconnect();
    }
  } else {
    DEBUG_PRINTLN("Non-PS5 controller. Disconnecting.");
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  DEBUG_PRINTLN("Controller disconnected");
  if (ctl == activeCtl) {
    activeCtl = nullptr;
    DEBUG_PRINTLN("activeCtl cleared");
    // เริ่มค้นหาใหม่
    BP32.setup(&onConnectedController, &onDisconnectedController);
  }
}

int16_t applyDeadzone(int16_t val, int threshold = 60) {
  return (abs(val) < threshold) ? 0 : val;
}

void sendGamepadData(ControllerPtr ctl) {
  int16_t lx = applyDeadzone((int16_t)ctl->axisX());
  int16_t ly = applyDeadzone((int16_t)-ctl->axisY());
  int16_t rx = applyDeadzone((int16_t)ctl->axisRX());
  int16_t ry = applyDeadzone((int16_t)-ctl->axisRY());
  uint16_t throttle = ctl->throttle();
  uint16_t brake = ctl->brake();
  uint16_t dpad = ctl->dpad();
  uint32_t buttons = ctl->buttons();

  DEBUG_PRINTF("LX:%d\tLY:%d\tRX:%d\tRY:%d\tThrottle:%d\tBrake:%d\tDpad:0x%04X\tButtons:0x%04lX\n",
               lx, ly, rx, ry, throttle, brake, dpad, buttons);

  uint8_t data[17] = {
    0xAA,
    highByte(lx), lowByte(lx),
    highByte(ly), lowByte(ly),
    highByte(rx), lowByte(rx),
    highByte(throttle), lowByte(throttle),
    highByte(brake), lowByte(brake),
    highByte(dpad), lowByte(dpad),
    uint8_t(buttons >> 24), uint8_t(buttons >> 16),
    uint8_t(buttons >> 8), uint8_t(buttons)
  };

  UART_OUT.write(data, sizeof(data));
}

void setup() {
  Serial.begin(115200);
  UART_OUT.begin(115200, SERIAL_8E1, -1, 17);  // TX=17
  delay(1000);

  // เพิ่มกำลังส่ง Bluetooth
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  DEBUG_PRINTLN("Bluetooth controller ready.");
}

void loop() {
  BP32.update();

  // กรณีหลุดโดยไม่มี event
  if (activeCtl && !activeCtl->isConnected()) {
    DEBUG_PRINTLN("Controller lost without disconnect event. Forcing clear.");
    activeCtl = nullptr;
  }

  if (activeCtl && activeCtl->isConnected()) {
    sendGamepadData(activeCtl);
    processGamepad(activeCtl);
    delay(20);
  }
}
