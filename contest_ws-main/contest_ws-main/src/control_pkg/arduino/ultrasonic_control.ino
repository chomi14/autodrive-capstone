// 이 코드는 예시이며, 사용하는 초음파 센서 라이브러리나 핀 구성에 맞게 수정해야 합니다.
const int NUM_SENSORS = 10;
// 각 센서의 trig, echo 핀을 배열로 정의
int trigPins[NUM_SENSORS] = {2, 4, 6, 8, 10, 12, 14, 16, 18, 20};
int echoPins[NUM_SENSORS] = {3, 5, 7, 9, 11, 13, 15, 17, 19, 21};

void setup() {
  Serial.begin(9600); // 파이썬 코드의 baud_rate와 일치해야 함
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() {
  String dataString = "";
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    long duration = pulseIn(echoPins[i], HIGH);
    // 센티미터(cm) 단위로 거리 계산
    float distance = duration * 0.034 / 2.0;

    // "인덱스,거리;" 형태로 문자열 구성
    dataString += String(i) + "," + String(distance);
    if (i < NUM_SENSORS - 1) {
      dataString += ";";
    }
  }
  // 완성된 문자열을 시리얼로 전송
  Serial.println(dataString);

  delay(100); // 0.1초 대기
}
