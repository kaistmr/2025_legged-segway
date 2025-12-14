// Arduino Nano Controller for Legged Segway
// HC-06 블루투스 모듈을 통해 로봇 조종
// Arduino Nano + HC-06
//
// ===== HC-06 페어링 설정 =====
// MAC 주소:
//   - 조종기 HC-06: 98:DA:60:08:7F:9B (마스터 모드)
//   - 로봇 HC-06:   98:DA:60:08:61:BB (슬레이브 모드)
//
// 이 코드는 자동으로:
//   1. 컨트롤러 HC-06을 마스터 모드로 설정
//   2. 로봇 HC-06의 MAC 주소로 바인딩
//   3. 연결 초기화 및 자동 연결
//
// 로봇 측 설정 필요:
//   - 로봇 HC-06을 슬레이브 모드로 설정: AT+ROLE=0
//
// 연결 확인:
//   - LED가 켜지면 연결 성공
//   - LED가 깜빡이면 연결 대기 중

#include <SoftwareSerial.h>

// -------------------- HC-06 Bluetooth Serial --------------------
// HC-06 연결 핀 (Arduino Nano)
const int HC06_TX_PIN = 2;  // Arduino Nano RX → HC-06 TX
const int HC06_RX_PIN = 3;  // Arduino Nano TX → HC-06 RX

SoftwareSerial SerialBT(HC06_RX_PIN, HC06_TX_PIN);  // RX, TX

// HC-06 설정
const char* ROBOT_HC06_NAME = "jalseobot";  // 로봇의 HC-06 이름
// MAC 주소: 조종기=98:DA:60:08:7F:9B, 로봇=98:DA:60:08:61:BB
const char* ROBOT_HC06_MAC = "98,DA,60,08,61,BB";  // 로봇의 HC-06 MAC 주소 (HC-06 AT 명령 형식)

// -------------------- 핀 정의 --------------------
// 조이스틱
const int JOYSTICK_VRX_PIN = A0;  // VRx - 아날로그 핀
const int JOYSTICK_VRY_PIN = A1;  // VRy - 아날로그 핀
const int JOYSTICK_SW_PIN  = 4;   // SW - 디지털 핀

// 버튼
const int BUTTON_D_PIN = 5;   // GPIO5 - 서보 내리기 (D)
const int BUTTON_U_PIN = 6;   // GPIO6 - 서보 올리기 (U)
const int BUTTON_0_PIN = 7;   // GPIO7 - IMU 리셋 (0)

// LED
const int LED_PIN = 8;  // GPIO8 (블루투스 연결 상태 표시)

// -------------------- 조이스틱 설정 --------------------
const int JOYSTICK_CENTER = 512;  // 중앙값 (10비트 ADC: 0-1023)
const int JOYSTICK_DEADZONE = 50;  // 데드존 (중앙 근처 무시)
const int JOYSTICK_THRESHOLD = 200;  // 회전 감지 임계값

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

// HC-06 연결 확인용
unsigned long lastConnectionCheck = 0;
const unsigned long CONNECTION_CHECK_INTERVAL_MS = 3000;  // 3초마다 연결 확인
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL_MS = 10000;  // 10초마다 재연결 시도

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  // HC-06 블루투스 시리얼 초기화
  SerialBT.begin(9600);  // HC-06 기본 보드레이트
  delay(100);

  // 핀 설정
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_D_PIN, INPUT_PULLUP);
  pinMode(BUTTON_U_PIN, INPUT_PULLUP);
  pinMode(BUTTON_0_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // LED 초기 상태 (깜빡임 시작)
  digitalWrite(LED_PIN, LOW);

  Serial.println("Arduino Nano Controller Started");
  Serial.println("HC-06 Bluetooth initialized at 9600 baud");

  // HC-06 마스터 모드 설정 (컨트롤러는 마스터로 동작)
  setupHC06AsMaster();

  Serial.println("Waiting for connection to robot HC-06...");
  delay(500);
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

// -------------------- HC-06 AT 명령 전송 및 응답 확인 --------------------
bool sendATCommand(const char* cmd, const char* expectedResponse, int timeout = 1000) {
  // 버퍼 비우기
  while (SerialBT.available()) {
    SerialBT.read();
  }

  SerialBT.print(cmd);
  SerialBT.print("\r\n");

  delay(100);

  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < timeout) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      response += c;
      if (response.indexOf(expectedResponse) >= 0) {
        Serial.print("  -> OK: ");
        Serial.println(cmd);
        return true;
      }
    }
  }

  Serial.print("  -> Timeout/No response: ");
  Serial.println(cmd);
  if (response.length() > 0) {
    Serial.print("    Response: ");
    Serial.println(response);
  }
  return false;
}

// -------------------- HC-06 마스터 모드 설정 --------------------
void setupHC06AsMaster() {
  Serial.println("Configuring HC-06 as Master...");
  delay(2000);  // HC-06 초기화 대기

  // AT 명령 테스트
  Serial.println("Testing AT command...");
  if (!sendATCommand("AT", "OK", 1000)) {
    Serial.println("WARNING: HC-06 not responding to AT commands!");
    Serial.println("Make sure HC-06 is powered and connected correctly.");
  }

  // HC-06을 마스터 모드로 설정
  Serial.println("Setting to Master mode...");
  sendATCommand("AT+ROLE=1", "OK", 1000);
  delay(500);

  // 연결 모드 설정 (지정된 MAC 주소에만 연결)
  Serial.println("Setting connection mode...");
  sendATCommand("AT+CMODE=0", "OK", 1000);
  delay(500);

  // 로봇 HC-06 MAC 주소로 바인딩
  Serial.print("Binding to robot MAC: ");
  Serial.println(ROBOT_HC06_MAC);
  String bindCmd = "AT+BIND=";
  bindCmd += ROBOT_HC06_MAC;
  sendATCommand(bindCmd.c_str(), "OK", 1000);
  delay(500);

  // 연결 초기화 (연결 시작)
  Serial.println("Initializing connection...");
  sendATCommand("AT+INIT", "OK", 2000);
  delay(1000);

  Serial.println("HC-06 configured as Master");
  Serial.println("Connecting to robot HC-06...");
  Serial.print("Robot MAC: ");
  Serial.println(ROBOT_HC06_MAC);
  Serial.println("Note: Connection may take a few seconds...");
}

// -------------------- 블루투스 연결 확인 --------------------
void checkBluetoothConnection() {
  unsigned long now = millis();

  // 주기적으로 연결 확인
  if (now - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL_MS) {
    lastConnectionCheck = now;

    // 버퍼 비우기
    while (SerialBT.available()) {
      SerialBT.read();
    }

    // HC-06 연결 상태 확인
    SerialBT.print("AT+STATE?\r\n");
    delay(300);

    // 응답 확인
    bool isConnected = false;
    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < 500) {
      if (SerialBT.available()) {
        char c = SerialBT.read();
        response += c;
        // "CONNECTED" 또는 "PAIR" 응답이 오면 연결됨
        if (response.indexOf("CONNECTED") >= 0 || response.indexOf("PAIR") >= 0) {
          isConnected = true;
          break;
        }
        // "DISCONNECT" 응답이 오면 연결 안됨
        if (response.indexOf("DISCONNECT") >= 0) {
          isConnected = false;
          break;
        }
      }
    }

    // 연결 상태 업데이트
    static bool lastBtState = false;
    if (isConnected != lastBtState || response.length() > 0) {
      btConnected = isConnected;
      if (btConnected && !lastBtState) {
        Serial.println("HC-06 Connected to Robot!");
      } else if (!btConnected && lastBtState) {
        Serial.println("HC-06 Disconnected!");
      }
      lastBtState = btConnected;
    }
  }

  // 연결이 안 되어 있으면 주기적으로 재연결 시도
  if (!btConnected && now - lastReconnectAttempt >= RECONNECT_INTERVAL_MS) {
    lastReconnectAttempt = now;
    Serial.println("Attempting to reconnect...");
    SerialBT.print("AT+INIT\r\n");
    delay(1000);
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
