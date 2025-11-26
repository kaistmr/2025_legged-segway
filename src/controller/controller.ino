// ESP32 Bluetooth Controller for Legged Segway
// HC-06 블루투스 모듈을 통해 로봇 조종
// ESP32 WROOM 32 (USB C, CH340)

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled! Please run `make menuconfig` to enable it"
#endif

// -------------------- Bluetooth Serial --------------------
BluetoothSerial SerialBT;

const char* BT_DEVICE_NAME = "ESP32_Controller";
const char* BT_SERVER_NAME = "HC06";  // HC-06 기본 이름 (필요시 변경)

// -------------------- 핀 정의 --------------------
// 조이스틱
const int JOYSTICK_VRX_PIN = 34;  // GPIO34 (ADC1_CH6)
const int JOYSTICK_VRY_PIN = 35;  // GPIO35 (ADC1_CH7)
const int JOYSTICK_SW_PIN  = 32;  // GPIO32

// 버튼
const int BUTTON_D_PIN = 25;   // GPIO25 - 서보 내리기 (D)
const int BUTTON_U_PIN = 26;   // GPIO26 - 서보 올리기 (U)
const int BUTTON_0_PIN = 27;   // GPIO27 - IMU 리셋 (0)

// LED
const int LED_PIN = 2;  // GPIO2 (내장 LED 또는 외부 LED)

// -------------------- 조이스틱 설정 --------------------
const int JOYSTICK_CENTER = 2048;  // 중앙값 (12비트 ADC: 0-4095)
const int JOYSTICK_DEADZONE = 200;  // 데드존 (중앙 근처 무시)
const int JOYSTICK_THRESHOLD = 1000;  // 회전 감지 임계값

// 조이스틱 상태
int lastVrx = JOYSTICK_CENTER;
int lastVry = JOYSTICK_CENTER;
bool lastSwState = HIGH;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_INTERVAL_MS = 100;  // 명령 전송 간격 (100ms)

// 버튼 상태 (디바운싱)
bool lastButtonD = HIGH;
bool lastButtonU = HIGH;
bool lastButton0 = HIGH;
unsigned long lastButtonTime = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

// 블루투스 연결 상태
bool btConnected = false;
unsigned long lastLedToggle = 0;
const unsigned long LED_BLINK_INTERVAL_MS = 500;  // 연결 안됨 시 깜빡임 간격
bool ledState = false;

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  // 핀 설정
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_D_PIN, INPUT_PULLUP);
  pinMode(BUTTON_U_PIN, INPUT_PULLUP);
  pinMode(BUTTON_0_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // LED 초기 상태 (깜빡임 시작)
  digitalWrite(LED_PIN, LOW);

  // 블루투스 시리얼 초기화
  SerialBT.begin(BT_DEVICE_NAME);
  Serial.println("Bluetooth Started! Ready to pair.");
  Serial.println("Looking for HC-06...");

  // HC-06 연결 시도 (선택사항 - 자동 연결 시도)
  // SerialBT.connect(BT_SERVER_NAME);  // 필요시 주석 해제
}

// -------------------- Loop --------------------
void loop() {
  // 블루투스 연결 상태 확인
  checkBluetoothConnection();

  // LED 상태 업데이트
  updateLED();

  // 버튼 처리
  handleButtons();

  // 조이스틱 처리
  handleJoystick();

  delay(10);  // 작은 딜레이
}

// -------------------- 블루투스 연결 확인 --------------------
void checkBluetoothConnection() {
  bool wasConnected = btConnected;
  btConnected = SerialBT.hasClient();

  if (!wasConnected && btConnected) {
    Serial.println("Bluetooth Connected!");
  } else if (wasConnected && !btConnected) {
    Serial.println("Bluetooth Disconnected!");
  }
}

// -------------------- LED 상태 업데이트 --------------------
void updateLED() {
  if (btConnected) {
    // 연결됨: LED 켜기
    digitalWrite(LED_PIN, HIGH);
  } else {
    // 연결 안됨: 깜빡이기
    unsigned long now = millis();
    if (now - lastLedToggle >= LED_BLINK_INTERVAL_MS) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastLedToggle = now;
    }
  }
}

// -------------------- 버튼 처리 --------------------
void handleButtons() {
  unsigned long now = millis();

  // 디바운싱
  if (now - lastButtonTime < BUTTON_DEBOUNCE_MS) {
    return;
  }

  // 버튼 D (서보 내리기)
  bool buttonD = digitalRead(BUTTON_D_PIN);
  if (buttonD == LOW && lastButtonD == HIGH) {
    // 버튼 눌림 (LOW = 눌림, 풀업 저항)
    if (btConnected) {
      SerialBT.write('D');
      Serial.println("[Button] D - Servo Fold");
    }
    lastButtonTime = now;
  }
  lastButtonD = buttonD;

  // 버튼 U (서보 올리기)
  bool buttonU = digitalRead(BUTTON_U_PIN);
  if (buttonU == LOW && lastButtonU == HIGH) {
    if (btConnected) {
      SerialBT.write('U');
      Serial.println("[Button] U - Servo Stand");
    }
    lastButtonTime = now;
  }
  lastButtonU = buttonU;

  // 버튼 0 (IMU 리셋)
  bool button0 = digitalRead(BUTTON_0_PIN);
  if (button0 == LOW && lastButton0 == HIGH) {
    if (btConnected) {
      SerialBT.write('0');
      Serial.println("[Button] 0 - IMU Reset");
    }
    lastButtonTime = now;
  }
  lastButton0 = button0;
}

// -------------------- 조이스틱 처리 --------------------
void handleJoystick() {
  if (!btConnected) {
    return;  // 블루투스 연결 안됨
  }

  unsigned long now = millis();
  if (now - lastCommandTime < COMMAND_INTERVAL_MS) {
    return;  // 명령 전송 간격 유지
  }

  // 조이스틱 값 읽기
  int vrx = analogRead(JOYSTICK_VRX_PIN);
  int vry = analogRead(JOYSTICK_VRY_PIN);
  bool swState = digitalRead(JOYSTICK_SW_PIN);

  // SW 버튼 처리 (누르면 정지)
  if (swState == LOW && lastSwState == HIGH) {
    // SW 눌림
    SerialBT.write('S');
    Serial.println("[Joystick] SW - Stop");
    lastCommandTime = now;
    lastSwState = swState;
    return;
  }
  lastSwState = swState;

  // SW가 눌려있으면 정지 명령 유지
  if (swState == LOW) {
    SerialBT.write('S');
    return;
  }

  // VRx, VRy 값 정규화 (중앙 기준)
  int vrxOffset = vrx - JOYSTICK_CENTER;
  int vryOffset = vry - JOYSTICK_CENTER;

  // 방향 결정 (앞뒤 우선, 좌우는 앞뒤가 중앙일 때만)
  char command = 'S';  // 기본값: 정지

  // 전진/후진 체크 (VRy) - 우선순위 1
  if (abs(vryOffset) > JOYSTICK_THRESHOLD) {
    if (vryOffset < 0) {
      command = 'F';  // 전진
    } else {
      command = 'B';  // 후진
    }
  }
  // 좌/우 회전 체크 (VRx) - 앞뒤가 거의 중앙일 때만 처리
  else if (abs(vryOffset) < JOYSTICK_DEADZONE && abs(vrxOffset) > JOYSTICK_THRESHOLD) {
    if (vrxOffset < 0) {
      command = 'L';  // 좌회전
    } else {
      command = 'R';  // 우회전
    }
  }

  // 명령 전송
  SerialBT.write(command);
  if (command != 'S') {
    Serial.print("[Joystick] Command: ");
    Serial.println(command);
  }
  lastCommandTime = now;

  lastVrx = vrx;
  lastVry = vry;
}

