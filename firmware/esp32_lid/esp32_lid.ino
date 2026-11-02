// ESP32-S3 뚜껑 스텝모터 참고 펌웨어 (DRV8825) — docs/esp32_lid_protocol.md
//
// 뚜껑 부분만 있다. 기존 ESP32 펌웨어(LOCK/UNLOCK/LIFT_*/LED)에 합칠 때는 "뚜껑" 구역을 복사하고
//   setup() 에서 lidSetup(), loop() 에서 lidLoop(), 명령 파서에서 if (lidHandleLine(text)) return;
// 만 부르면 된다. 전역 이름은 모두 lid 로 시작한다(기존 코드와 겹치지 않게).
// 핀·스텝 수·방향·속도는 배선과 기구를 확인한 뒤 정한다. 정하기 전에는 컴파일되지 않게 막아 두었다.
//
// 위치(스텝): 0 = 닫힘, LID_OPEN_STEPS = 열림. 켜질 때 열림 스위치만 눌려 있으면 열림,
// 그 밖에는 닫혀 있다고 본다(0). 스위치가 없으면 위치는 센 값(추정)일 뿐이다
// → ESP32 를 켜거나 리셋하기 전에 뚜껑을 손으로 닫아 둔다.
// 시리얼: 115200, 한 줄 = 명령 하나 ('\n'). Pi 쪽 esp32_serial_bridge 와 같은 설정.
// ESP32 보드에서 컴파일·실행해 본 적은 없다(PC 에서 가짜 Arduino 함수로 논리만 시험함).

#include <Arduino.h>

// ============================== TODO: 배선 확인 후 설정 ==============================
#error "핀 번호와 스텝 수를 배선·기구에 맞게 정한 뒤 이 줄을 지우세요 (esp32_lid_protocol.md 3절)"
const int LID_PIN_STEP = -1;          // DRV8825 STEP
const int LID_PIN_DIR = -1;           // DRV8825 DIR
const int LID_PIN_EN = -1;            // DRV8825 EN (LOW = 켜짐). EN 을 GND 에 묶었으면 -1
const int LID_PIN_LIMIT_OPEN = -1;    // 열림 리미트 스위치 (눌림 = LOW, 내부 풀업). 없으면 -1
const int LID_PIN_LIMIT_CLOSED = -1;  // 닫힘 리미트 스위치. 없으면 -1
const int LID_PIN_FAULT = -1;         // DRV8825 FAULT (LOW = 고장). 안 쓰면 -1
const long LID_OPEN_STEPS = 0;        // 닫힘 → 열림까지 스텝 수 (마이크로스텝 포함)
const bool LID_OPEN_DIR_LEVEL = HIGH; // 열림 방향일 때 DIR 레벨
const float LID_MAX_SPS = 800.0;      // 최고 속도 (스텝/초)
const float LID_START_SPS = 200.0;    // 출발·끝 속도
const float LID_ACCEL = 800.0;        // 가감속 (스텝/초²)
const float LID_SEARCH_RATIO = 0.3;   // 스위치가 있을 때 끝 위치를 지나 LID_OPEN_STEPS × 이 비율까지 천천히 더 찾음
const bool LID_HOLD_AFTER_MOVE = false;  // 끝난 뒤에도 EN 유지(유지 토크, 발열 주의)
// =====================================================================================

// ================================ 뚜껑 (여기부터 복사) ================================
long lidPosition = 0;   // 지금 위치(스텝). 스위치가 눌리면 0 또는 LID_OPEN_STEPS 로 맞춘다
long lidTarget = 0;     // 이번 동작의 목표 위치
bool lidMoving = false;
bool lidMovingOpen = false;
float lidSpeed = LID_START_SPS;
unsigned long lidLastStepMicros = 0;
unsigned long lidLastStateMillis = 0;

bool lidHasLimits() { return LID_PIN_LIMIT_OPEN >= 0 || LID_PIN_LIMIT_CLOSED >= 0; }
bool lidPressed(int pin) { return pin >= 0 && digitalRead(pin) == LOW; }

void lidEnableDriver(bool on) {
  if (LID_PIN_EN >= 0) digitalWrite(LID_PIN_EN, on ? LOW : HIGH);
}

// LID_STATE 값. 센서가 없으면 UNKNOWN (센 위치를 "확인됨"으로 보내지 않는다)
const char *lidStateName() {
  if (lidMoving) return "MOVING";
  bool o = lidPressed(LID_PIN_LIMIT_OPEN), c = lidPressed(LID_PIN_LIMIT_CLOSED);
  if (o && c) return "UNKNOWN";  // 둘 다 눌림 = 배선/스위치 이상
  if (o) return "OPEN";
  if (c) return "CLOSED";
  return "UNKNOWN";
}

void lidReportState() {
  Serial.print("LID_STATE:");
  Serial.println(lidStateName());
}

void lidFinish(const char *message) {
  lidMoving = false;
  if (!LID_HOLD_AFTER_MOVE) lidEnableDriver(false);
  Serial.println(message);
}

void lidStartMove(bool open) {
  if (lidMoving) {
    Serial.print("BUSY:");
    Serial.println(lidMovingOpen ? "LID_OPEN" : "LID_CLOSE");
    return;
  }
  if (lidPressed(LID_PIN_LIMIT_OPEN) && lidPressed(LID_PIN_LIMIT_CLOSED)) {
    Serial.println("LID_ERROR:BOTH_LIMITS");
    return;
  }
  int endSwitch = open ? LID_PIN_LIMIT_OPEN : LID_PIN_LIMIT_CLOSED;
  if (lidPressed(endSwitch)) {  // 이미 그 끝 (스위치로 확인)
    lidPosition = open ? LID_OPEN_STEPS : 0;
    Serial.println(open ? "LID_OPENED" : "LID_CLOSED");
    return;
  }
  lidTarget = open ? LID_OPEN_STEPS : 0;
  if (endSwitch < 0 && lidPosition == lidTarget) {  // 스위치 없음, 센 위치가 이미 목표: 움직이지 않음
    Serial.println(open ? "LID_OPEN_STEPS_DONE" : "LID_CLOSE_STEPS_DONE");
    return;
  }
  lidMovingOpen = open;
  lidMoving = true;
  lidSpeed = LID_START_SPS;
  digitalWrite(LID_PIN_DIR, open ? LID_OPEN_DIR_LEVEL : !LID_OPEN_DIR_LEVEL);
  lidEnableDriver(true);
  delayMicroseconds(5);  // DIR·EN 설정 시간 (DRV8825 최소 650 ns)
  lidLastStepMicros = micros();
}

void lidStep() {
  if (!lidMoving) return;
  if (LID_PIN_FAULT >= 0 && digitalRead(LID_PIN_FAULT) == LOW) {
    lidFinish("LID_ERROR:DRIVER_FAULT");
    return;
  }
  int endSwitch = lidMovingOpen ? LID_PIN_LIMIT_OPEN : LID_PIN_LIMIT_CLOSED;
  if (lidPressed(endSwitch)) {  // 끝 확인
    lidPosition = lidMovingOpen ? LID_OPEN_STEPS : 0;
    lidFinish(lidMovingOpen ? "LID_OPENED" : "LID_CLOSED");
    return;
  }
  long remaining = lidMovingOpen ? lidTarget - lidPosition : lidPosition - lidTarget;
  if (remaining <= 0) {
    if (endSwitch < 0) {  // 스위치 없음: 스텝 수로만 끝 (확인 아님)
      lidFinish(lidMovingOpen ? "LID_OPEN_STEPS_DONE" : "LID_CLOSE_STEPS_DONE");
      return;
    }
    if (-remaining >= (long)(LID_OPEN_STEPS * LID_SEARCH_RATIO)) {  // 더 가도 스위치가 안 눌림
      lidFinish("LID_ERROR:TIMEOUT");
      return;
    }
  }
  unsigned long now = micros();
  unsigned long interval = (unsigned long)(1000000.0f / lidSpeed);
  if (now - lidLastStepMicros < interval) return;
  lidLastStepMicros = now;
  digitalWrite(LID_PIN_STEP, HIGH);
  delayMicroseconds(3);  // DRV8825 STEP 펄스 최소 1.9 us
  digitalWrite(LID_PIN_STEP, LOW);
  lidPosition += lidMovingOpen ? 1 : -1;

  // 가감속: 목표 위치에서 출발 속도가 되도록 줄이고, 그 뒤(스위치 찾기)는 출발 속도로
  float dt = interval / 1000000.0f;
  float brakeSteps = (lidSpeed * lidSpeed - LID_START_SPS * LID_START_SPS) / (2.0f * LID_ACCEL);
  if (remaining - 1 <= brakeSteps) {
    lidSpeed = max(LID_START_SPS, lidSpeed - LID_ACCEL * dt);
  } else {
    lidSpeed = min(LID_MAX_SPS, lidSpeed + LID_ACCEL * dt);
  }
}

// LID_ 로 시작하는 줄을 처리했으면 true
bool lidHandleLine(const String &text) {
  if (text == "LID_OPEN") {
    lidStartMove(true);
  } else if (text == "LID_CLOSE") {
    lidStartMove(false);
  } else if (text == "LID_STOP") {
    if (lidMoving) lidFinish("LID_STOPPED");  // 위치는 센 값 그대로 (다음 명령이 거기서 이어감)
    else Serial.println("LID_STOPPED");
  } else if (text == "LID_STATUS") {
    lidReportState();
  } else {
    return false;
  }
  return true;
}

void lidSetup() {
  pinMode(LID_PIN_STEP, OUTPUT);
  pinMode(LID_PIN_DIR, OUTPUT);
  if (LID_PIN_EN >= 0) pinMode(LID_PIN_EN, OUTPUT);
  lidEnableDriver(false);  // 시작할 때는 꺼둠
  if (LID_PIN_LIMIT_OPEN >= 0) pinMode(LID_PIN_LIMIT_OPEN, INPUT_PULLUP);
  if (LID_PIN_LIMIT_CLOSED >= 0) pinMode(LID_PIN_LIMIT_CLOSED, INPUT_PULLUP);
  if (LID_PIN_FAULT >= 0) pinMode(LID_PIN_FAULT, INPUT_PULLUP);
  delay(5);
  bool openOnly = lidPressed(LID_PIN_LIMIT_OPEN) && !lidPressed(LID_PIN_LIMIT_CLOSED);
  lidPosition = openOnly ? LID_OPEN_STEPS : 0;
}

void lidLoop() {
  lidStep();
  if (lidHasLimits() && millis() - lidLastStateMillis >= 500) {  // 스위치가 있을 때만 2 Hz
    lidLastStateMillis = millis();
    lidReportState();
  }
}
// ================================ 뚜껑 (여기까지) ====================================

String serialLine;

void setup() {
  Serial.begin(115200);
  lidSetup();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialLine.trim();
      lidHandleLine(serialLine);  // 기존 펌웨어라면: if (!lidHandleLine(serialLine)) { 기존 처리 }
      serialLine = "";
    } else if (c != '\r' && serialLine.length() < 120) {
      serialLine += c;
    }
  }
  lidLoop();
}
