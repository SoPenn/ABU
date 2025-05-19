#define BP32_LOG_LEVEL 0
#include <Bluepad32.h>
#include "esp_bt.h"

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

    led++;
    if (led > 4) {
      led = 0;
    }
  }

  // ตรวจจับปุ่ม O ตลอดเวลา
  if (buttons & 0x0002) {
    led = 0;
  }
  switch (led) {
    case 1: ctl->setColorLED(0, 255, 0); break;    // เขียว
    case 2: ctl->setColorLED(200, 255, 0); break;  // เหลือง
    case 3: ctl->setColorLED(255, 165, 0); break;  // ส้ม
    case 4: ctl->setColorLED(255, 0, 0); break;    // แดง
    default: ctl->setColorLED(255, 255, 255); break;   // น้ำเงิน (หรือดับ)
  }
  brakePressedLast = brakeNow;
}

void onConnectedController(ControllerPtr ctl) {
  auto props = ctl->getProperties();
  ctl->setRumble(0x40, 0x40);
  delay(500);

  Serial.println("New controller connected");

  // ตรวจสอบว่าเป็น PS5
  if (props.vendor_id == VID_PS5 && props.product_id == PID_PS5) {
    if (!activeCtl || !activeCtl->isConnected()) {
      activeCtl = ctl;
      Serial.println("PS5 controller accepted and set as activeCtl");

      // ปิดรับ controller ใหม่ ด้วยการลบ callback
      BP32.setup(nullptr, nullptr);  // ยกเลิก callback

      ctl->setPlayerLEDs(0x04);
      ctl->setPlayerLEDs(0x00);
    } else {
      Serial.println("Another controller is already active. Disconnecting this one.");
      ctl->disconnect();
    }
  } else {
    Serial.println("Non-PS5 controller. Disconnecting.");
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected");
  if (ctl == activeCtl) {
    activeCtl = nullptr;
    Serial.println("activeCtl cleared");

    // ✅ เรียก setup ใหม่ ถ้าต้องการให้ค้นหาจอยอีกครั้ง
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

  // ตรวจสอบว่าทุกค่าเป็นศูนย์หรือไม่
  if (lx == 0 && ly == 0 && rx == 0 && ry == 0 &&
      throttle == 0 && brake == 0 &&
      dpad == 0 && buttons == 0) {
    return;  // ไม่ส่ง UART หากไม่มีการเคลื่อนไหว/กดปุ่ม
  }

  // ถ้ามีค่าใดค่าใดไม่เป็น 0 ให้ส่งผ่าน UART
  Serial.printf("LX:%d\tLY:%d\tRX:%d\tRY:%d\tThrottle:%d\tBrake:%d\tDpad:0x%04X\tButtons:0x%04lX\n",
                lx, ly, rx, ry, throttle, brake, dpad, buttons);

  uint8_t data[17];
  data[0] = 0xAA;
  data[1] = highByte(lx);
  data[2] = lowByte(lx);
  data[3] = highByte(ly);
  data[4] = lowByte(ly);
  data[5] = highByte(rx);
  data[6] = lowByte(rx);
  data[7] = highByte(throttle);
  data[8] = lowByte(throttle);
  data[9] = highByte(brake);
  data[10] = lowByte(brake);
  data[11] = highByte(dpad);
  data[12] = lowByte(dpad);
  data[13] = (buttons >> 24) & 0xFF;
  data[14] = (buttons >> 16) & 0xFF;
  data[15] = (buttons >> 8) & 0xFF;
  data[16] = (buttons >> 0) & 0xFF;

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

  //BP32.enableVirtualDevice(false);
  // BP32.forgetBluetoothKeys();  // ล้างจอยที่จับคู่อยู่ก่อนหน้า
  BP32.setup(&onConnectedController, &onDisconnectedController);

  Serial.println("Bluetooth controller ready.");
}

void loop() {
  BP32.update();

  // ตรวจสอบกรณี controller หลุด แต่ callback ไม่ทำงาน
  if (activeCtl && !activeCtl->isConnected()) {
    Serial.println("Controller lost without disconnect event. Forcing clear.");
    activeCtl = nullptr;
  }

  if (activeCtl && activeCtl->isConnected()) {
    sendGamepadData(activeCtl);
    processGamepad(activeCtl);
    delay(20);
  }
}
