#include <string.h>

// Complete signed frames need room for all values and their terminator.
// Reject overflow instead of executing a silently truncated command.
const unsigned int MAX_INPUT = 48;

// 핀 번호 변수
const int STEERING_1 = 2;
const int STEERING_2 = 3;
const int FORWARD_RIGHT_1 = 4;
const int FORWARD_RIGHT_2 = 5;
const int FORWARD_LEFT_1 = 7;
const int FORWARD_LEFT_2 =6;
const int POT = A2;

// 조향 속도 상수
const int STEERING_SPEED = 200;

// 가변저항 값 범위
const int resistance_most_left = 430;
const int resistance_most_right = 275;

// 조향 최대 단계 수 (한 쪽 기준)
const int MAX_STEERING_STEP = 7;

// 제어 상태 변수
int angle = 0, resistance = 0, mapped_resistance = 0;
int left_speed = 0, right_speed = 0;

// 명령 주기 제한 변수
// Control scheduling and communication freshness are independent clocks.
// Updating the control loop must NEVER feed the receive watchdog.
unsigned long lastControlTime = 0;
unsigned long lastValidCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 500; // ~15 nominal 30 Hz command periods
bool command_active = false;
const unsigned int COMMAND_INTERVAL = 50; // 명령 처리 간 최소 대기 시간(ms)

// 함수 선언
void steerRight();
void steerLeft();
void maintainSteering();
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);
void processIncomingByte(const byte inByte);
void processData(const char *data);
void emergencyStop();
bool parseValue(const char *&cursor, char separator, int low, int high, int &value);

void setup() {
    Serial.begin(115200);

    // 핀 모드 설정
    pinMode(POT, INPUT);
    pinMode(STEERING_1, OUTPUT);
    pinMode(STEERING_2, OUTPUT);
    pinMode(FORWARD_RIGHT_1, OUTPUT);
    pinMode(FORWARD_RIGHT_2, OUTPUT);
    pinMode(FORWARD_LEFT_1, OUTPUT);
    pinMode(FORWARD_LEFT_2, OUTPUT);
    emergencyStop(); // Boot is PWM-off; do not move steering toward center.
}

void loop() {
    // 현재 시간 가져오기
    unsigned long currentTime = millis();

    // 직렬 데이터 처리
    // Bound serial work so even a continuous bad stream cannot starve stop.
    for (unsigned int n = 0; n < MAX_INPUT && Serial.available() > 0; ++n) {
        processIncomingByte(Serial.read());
    }

    // Only a fully validated motion frame renews lastValidCommandTime.
    // Unsigned subtraction also works across millis() wraparound.
    currentTime = millis();
    if (command_active && currentTime - lastValidCommandTime >= COMMAND_TIMEOUT_MS) {
        emergencyStop();
    }

    // No steering correction while stopped: X means all six PWM outputs zero.
    if (command_active && currentTime - lastControlTime >= COMMAND_INTERVAL) {
        // 포텐셔미터 값을 읽어 조향 계산
        resistance = analogRead(POT);
        mapped_resistance = map(resistance, resistance_most_left, resistance_most_right, -MAX_STEERING_STEP, MAX_STEERING_STEP);

        // 조향 상태에 따라 동작 제어
        if (mapped_resistance == angle) {
            maintainSteering();
        } else if (mapped_resistance > angle) {
            steerLeft();
        } else {
            steerRight();
        }

        // 모터 속도 설정
        setLeftMotorSpeed(left_speed);
        setRightMotorSpeed(right_speed);

        // 마지막 명령 시간 갱신
        lastControlTime = currentTime;
    }
}

// 조향 제어 함수
void steerRight() {
    analogWrite(STEERING_1, STEERING_SPEED);
    analogWrite(STEERING_2, LOW);
}

void steerLeft() {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, STEERING_SPEED);
}

void maintainSteering() {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, LOW);
}

// 모터 속도 설정 함수
void setLeftMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(FORWARD_LEFT_1, speed);
        analogWrite(FORWARD_LEFT_2, LOW);
    } else {
        analogWrite(FORWARD_LEFT_1, LOW);
        analogWrite(FORWARD_LEFT_2, (-1) * speed);
    }
}

void setRightMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(FORWARD_RIGHT_1, speed);
        analogWrite(FORWARD_RIGHT_2, LOW);
    } else {
        analogWrite(FORWARD_RIGHT_1, LOW);
        analogWrite(FORWARD_RIGHT_2, (-1) * speed);
    }
}

// X cancels the active target; it never commands steering to center. Stored
// angle can remain unchanged because the control loop is disabled until a new
// valid motion frame arrives. The ROS bridge requires a fresh W before sending it.
void emergencyStop() {
    command_active = false;
    left_speed = right_speed = 0;
    setLeftMotorSpeed(0);
    setRightMotorSpeed(0);
    maintainSteering();
}

// Discard the whole malformed frame (overflow or embedded NUL), not a prefix.
void processIncomingByte(const byte inByte) {
    static char input_line[MAX_INPUT];
    static unsigned int input_pos = 0;
    static bool overflow = false;
    if (inByte == '\n') {
        input_line[input_pos] = '\0';
        if (!overflow) processData(input_line);
        input_pos = 0;
        overflow = false;
    } else if (inByte == 0) {
        overflow = true;
    } else if (inByte != '\r') {
        if (input_pos < MAX_INPUT - 1) input_line[input_pos++] = (char)inByte;
        else overflow = true;
    }
}

void processData(const char *data) {
    if (strcmp(data, "X") == 0) {
        emergencyStop(); // Immediate, independent of the 50 ms control schedule.
        Serial.println("STOPPED");
        return;
    }
    // This legacy sketch has no calibration protocol. Queries/invalid messages
    // intentionally do not refresh the motion watchdog or move any motor.
    if (data[0] != 's') return;
    int steering, left, right;
    const char *cursor = data + 1;
    if (!parseValue(cursor, 'l', -MAX_STEERING_STEP, MAX_STEERING_STEP, steering) ||
        !parseValue(cursor, 'r', -255, 255, left) ||
        !parseValue(cursor, '\0', -255, 255, right)) return;
    angle = steering;
    left_speed = left;
    right_speed = right;
    // Repeated identical commands are valid heartbeats too. Refresh only here,
    // after validating EVERY field, delimiter and the end of the frame.
    lastValidCommandTime = millis();
    command_active = true;
}

bool parseValue(const char *&cursor, char separator, int low, int high, int &value) {
    bool negative = *cursor == '-';
    if (negative || *cursor == '+') ++cursor;
    if (*cursor < '0' || *cursor > '9') return false;
    long magnitude = 0;
    while (*cursor >= '0' && *cursor <= '9') {
        magnitude = magnitude * 10 + (*cursor++ - '0');
        if (magnitude > 1023) return false; // Before AVR int overflow is possible.
    }
    if (*cursor != separator) return false;
    if (separator != '\0') ++cursor;
    long result = negative ? -magnitude : magnitude;
    if (result < low || result > high) return false;
    value = (int)result;
    return true;
}
