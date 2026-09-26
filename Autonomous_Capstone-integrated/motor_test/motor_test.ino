/*
 * ============================================================
 * Capstone Arduino Hardware Test
 *
 * 테스트 항목
 * 1. 조향 가변저항(POT) 실시간 값 확인
 * 2. 조향 모터 좌/우 테스트
 * 3. 좌측 구동 모터 양방향 테스트
 * 4. 우측 구동 모터 양방향 테스트
 *
 * Serial Monitor : 115200 baud
 * ============================================================
 */

// ------------------------------------------------------------
// 기존 driving.ino의 CAPSTONE 핀 설정 그대로 사용
// ------------------------------------------------------------

// 조향 모터 드라이버
const int STEERING_1 = 2;
const int STEERING_2 = 3;

// 오른쪽 구동 모터 드라이버
const int FORWARD_RIGHT_1 = 4;
const int FORWARD_RIGHT_2 = 5;

// 왼쪽 구동 모터 드라이버
const int FORWARD_LEFT_1 = 7;
const int FORWARD_LEFT_2 = 6;

// 조향 가변저항
const int POT = A2;


// ------------------------------------------------------------
// 테스트 설정
// ------------------------------------------------------------

// PWM 테스트 출력값
// 0 ~ 255
const int TEST_PWM = 100;

// 조향모터는 끝까지 밀어버리지 않도록 짧게만 동작
const unsigned long STEERING_TEST_TIME = 200;   // ms

// 구동모터 테스트 시간
const unsigned long DRIVE_TEST_TIME = 500;      // ms

// 가변저항 출력 주기
const unsigned long POT_PRINT_INTERVAL = 100;   // ms


// ------------------------------------------------------------
// 가변저항 측정용 변수
// ------------------------------------------------------------

int potValue = 0;

// 테스트 시작 이후 관측된 최대/최소값
int potMin = 1023;
int potMax = 0;

unsigned long lastPotPrintTime = 0;


// ------------------------------------------------------------
// 모든 모터 정지
// ------------------------------------------------------------

void stopAllMotors()
{
    digitalWrite(STEERING_1, LOW);
    digitalWrite(STEERING_2, LOW);

    digitalWrite(FORWARD_RIGHT_1, LOW);
    digitalWrite(FORWARD_RIGHT_2, LOW);

    digitalWrite(FORWARD_LEFT_1, LOW);
    digitalWrite(FORWARD_LEFT_2, LOW);
}


// ------------------------------------------------------------
// 조향 모터 방향 1 테스트
//
// STEERING_1 = pin 3
// → UNO/Nano 기준 PWM 가능
// ------------------------------------------------------------

void testSteering1()
{
    Serial.println();
    Serial.println("=== STEERING direction 1 ===");

    stopAllMotors();

    analogWrite(STEERING_1, TEST_PWM);
    digitalWrite(STEERING_2, LOW);

    unsigned long startTime = millis();

    // 조향 중 가변저항 값도 같이 출력
    while (millis() - startTime < STEERING_TEST_TIME)
    {
        int value = analogRead(POT);

        Serial.print("POT = ");
        Serial.println(value);

        delay(20);
    }

    stopAllMotors();

    Serial.println("STEERING STOP");
}


// ------------------------------------------------------------
// 조향 모터 방향 2 테스트
//
// STEERING_2 = pin 2
//
// ★ UNO/Nano의 2번 핀은 PWM 핀이 아님.
// 따라서 analogWrite 대신 digitalWrite를 사용.
// 즉 이 방향은 사실상 최대 출력으로 짧게 테스트됨.
// ------------------------------------------------------------

void testSteering2()
{
    Serial.println();
    Serial.println("=== STEERING direction 2 ===");

    stopAllMotors();

    digitalWrite(STEERING_1, LOW);
    digitalWrite(STEERING_2, HIGH);

    unsigned long startTime = millis();

    while (millis() - startTime < STEERING_TEST_TIME)
    {
        int value = analogRead(POT);

        Serial.print("POT = ");
        Serial.println(value);

        delay(20);
    }

    stopAllMotors();

    Serial.println("STEERING STOP");
}


// ------------------------------------------------------------
// 좌측 구동모터 방향 1
//
// pin 6 = PWM 가능
// ------------------------------------------------------------

void testLeftMotor1()
{
    Serial.println();
    Serial.println("=== LEFT MOTOR direction 1 ===");

    stopAllMotors();

    analogWrite(FORWARD_LEFT_1, TEST_PWM);
    digitalWrite(FORWARD_LEFT_2, LOW);

    delay(DRIVE_TEST_TIME);

    stopAllMotors();

    Serial.println("LEFT MOTOR STOP");
}


// ------------------------------------------------------------
// 좌측 구동모터 방향 2
//
// pin 7 = PWM 불가능 (UNO/Nano 기준)
// → 최대출력으로 짧게 테스트
// ------------------------------------------------------------

void testLeftMotor2()
{
    Serial.println();
    Serial.println("=== LEFT MOTOR direction 2 ===");

    stopAllMotors();

    digitalWrite(FORWARD_LEFT_1, LOW);
    digitalWrite(FORWARD_LEFT_2, HIGH);

    delay(DRIVE_TEST_TIME);

    stopAllMotors();

    Serial.println("LEFT MOTOR STOP");
}


// ------------------------------------------------------------
// 우측 구동모터 방향 1
//
// pin 4 = PWM 불가능 (UNO/Nano 기준)
// ------------------------------------------------------------

void testRightMotor1()
{
    Serial.println();
    Serial.println("=== RIGHT MOTOR direction 1 ===");

    stopAllMotors();

    digitalWrite(FORWARD_RIGHT_1, HIGH);
    digitalWrite(FORWARD_RIGHT_2, LOW);

    delay(DRIVE_TEST_TIME);

    stopAllMotors();

    Serial.println("RIGHT MOTOR STOP");
}


// ------------------------------------------------------------
// 우측 구동모터 방향 2
//
// pin 5 = PWM 가능
// ------------------------------------------------------------

void testRightMotor2()
{
    Serial.println();
    Serial.println("=== RIGHT MOTOR direction 2 ===");

    stopAllMotors();

    digitalWrite(FORWARD_RIGHT_1, LOW);
    analogWrite(FORWARD_RIGHT_2, TEST_PWM);

    delay(DRIVE_TEST_TIME);

    stopAllMotors();

    Serial.println("RIGHT MOTOR STOP");
}


// ------------------------------------------------------------
// 가변저항 min/max 초기화
// ------------------------------------------------------------

void resetPotRange()
{
    potMin = 1023;
    potMax = 0;

    Serial.println();
    Serial.println("POT min/max RESET");
}


// ------------------------------------------------------------
// 사용법 출력
// ------------------------------------------------------------

void printHelp()
{
    Serial.println();
    Serial.println("======================================");
    Serial.println(" CAPSTONE HARDWARE TEST");
    Serial.println("======================================");

    Serial.println("Steering");
    Serial.println("  a : steering direction 1");
    Serial.println("  d : steering direction 2");

    Serial.println();

    Serial.println("Left drive motor");
    Serial.println("  q : left direction 1");
    Serial.println("  w : left direction 2");

    Serial.println();

    Serial.println("Right drive motor");
    Serial.println("  e : right direction 1");
    Serial.println("  r : right direction 2");

    Serial.println();

    Serial.println("Utility");
    Serial.println("  x : ALL MOTOR STOP");
    Serial.println("  c : reset POT min/max");
    Serial.println("  h : print help");

    Serial.println("======================================");
    Serial.println();
}


// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    pinMode(POT, INPUT);

    pinMode(STEERING_1, OUTPUT);
    pinMode(STEERING_2, OUTPUT);

    pinMode(FORWARD_RIGHT_1, OUTPUT);
    pinMode(FORWARD_RIGHT_2, OUTPUT);

    pinMode(FORWARD_LEFT_1, OUTPUT);
    pinMode(FORWARD_LEFT_2, OUTPUT);

    // 시작할 때 모터가 갑자기 움직이지 않도록 전부 LOW
    stopAllMotors();

    delay(1000);

    printHelp();
}


// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop()
{
    // ========================================================
    // 1. 가변저항 실시간 측정
    // ========================================================

    potValue = analogRead(POT);

    // 지금까지 측정된 최소/최대값 저장
    if (potValue < potMin)
        potMin = potValue;

    if (potValue > potMax)
        potMax = potValue;


    // 100 ms마다 출력
    if (millis() - lastPotPrintTime >= POT_PRINT_INTERVAL)
    {
        Serial.print("POT = ");
        Serial.print(potValue);

        Serial.print("   MIN = ");
        Serial.print(potMin);

        Serial.print("   MAX = ");
        Serial.println(potMax);

        lastPotPrintTime = millis();
    }


    // ========================================================
    // 2. 시리얼 명령 처리
    // ========================================================

    if (Serial.available() > 0)
    {
        char command = Serial.read();

        switch (command)
        {
            // 조향
            case 'a':
                testSteering1();
                break;

            case 'd':
                testSteering2();
                break;


            // 좌측 모터
            case 'q':
                testLeftMotor1();
                break;

            case 'w':
                testLeftMotor2();
                break;


            // 우측 모터
            case 'e':
                testRightMotor1();
                break;

            case 'r':
                testRightMotor2();
                break;


            // 전체 정지
            case 'x':
                stopAllMotors();
                Serial.println("!!! ALL MOTOR STOP !!!");
                break;


            // POT min/max 리셋
            case 'c':
                resetPotRange();
                break;


            // 도움말
            case 'h':
                printHelp();
                break;
        }
    }
}
