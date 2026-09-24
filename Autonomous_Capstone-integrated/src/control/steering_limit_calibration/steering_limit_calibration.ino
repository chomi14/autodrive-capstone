/*
 * Steering physical-limit measurement for the current SKKU vehicle wiring.
 *
 * Wiring:
 *   Steering IN1=D3, IN2=D2
 *   Potentiometer OUT=A2
 *   Left rear IN1=D7, IN2=D6   (held OFF in this sketch)
 *   Right rear IN1=D4, IN2=D5  (held OFF in this sketch)
 *
 * Usage:
 *   1) Lift/secure the vehicle so it cannot roll.
 *   2) Upload this sketch and open Serial Monitor at 115200 baud.
 *   3) Type c + Enter once.
 *   4) The steering moves slowly LEFT -> RIGHT -> CENTER.
 *   5) Endpoint RESULT_CENTER is only an estimated midpoint, not straight wheels.
 *      With steering stopped, align wheels manually and send p to read CENTER ADC.
 *   6) Copy RESULT_LEFT and RESULT_RIGHT into DEFAULT_LEFT/DEFAULT_RIGHT
 *      in driving_user_pins.ino.
 *
 * The end-stop detector watches the potentiometer.  If its value stops changing
 * for END_STOP_STILL_MS, the code treats that position as the mechanical limit
 * and stops driving the steering motor.
 */

const int STEERING_1 = 3;
const int STEERING_2 = 2;
const int FORWARD_RIGHT_1 = 4;
const int FORWARD_RIGHT_2 = 5;
const int FORWARD_LEFT_1 = 7;
const int FORWARD_LEFT_2 = 6;
const int POT = A2;

const int CAL_PWM = 90;
const int POT_SAMPLE_MS = 30;
const int POT_CHANGE_THRESHOLD = 1;
const int END_STOP_STILL_MS = 350;
const int MIN_MOVE_MS = 300;
const int SIDE_TIMEOUT_MS = 6500;
const int STABLE_MS = 500;
const int CENTER_TIMEOUT_MS = 5000;
const int CENTER_DEADBAND_ADC = 3;
const int MIN_VALID_SPAN = 80;

bool running = false;
int state = 0;
unsigned long stateStart = 0;
unsigned long lastSample = 0;
unsigned long lastChange = 0;
int lastPot = 0;
int stableMin = 1023;
int stableMax = 0;
int leftValue = 0;
int rightValue = 0;
int centerValue = 0;

void steerLeft() {
  analogWrite(STEERING_1, 0);
  analogWrite(STEERING_2, CAL_PWM);
}

void steerRight() {
  analogWrite(STEERING_1, CAL_PWM);
  analogWrite(STEERING_2, 0);
}

void stopSteering() {
  analogWrite(STEERING_1, 0);
  analogWrite(STEERING_2, 0);
}

void stopDrive() {
  analogWrite(FORWARD_LEFT_1, 0);
  analogWrite(FORWARD_LEFT_2, 0);
  analogWrite(FORWARD_RIGHT_1, 0);
  analogWrite(FORWARD_RIGHT_2, 0);
}

int mappedStep(int p) {
  if (leftValue == rightValue) return 0;
  return constrain((int)map(p, leftValue, rightValue, -7, 7), -7, 7);
}

void fail(const char *why) {
  stopSteering();
  stopDrive();
  running = false;
  state = 0;
  Serial.print("CALIBRATION_FAILED: ");
  Serial.println(why);
}

void setup() {
  Serial.begin(115200);
  pinMode(STEERING_1, OUTPUT);
  pinMode(STEERING_2, OUTPUT);
  pinMode(FORWARD_RIGHT_1, OUTPUT);
  pinMode(FORWARD_RIGHT_2, OUTPUT);
  pinMode(FORWARD_LEFT_1, OUTPUT);
  pinMode(FORWARD_LEFT_2, OUTPUT);
  pinMode(POT, INPUT);
  stopSteering();
  stopDrive();
  delay(1000);
  Serial.println("=== STEERING LIMIT CALIBRATION ===");
  Serial.println("Type 'c' then Enter to start. Drive wheels stay OFF.");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') {
      fail("operator stop");
      return;
    }
    if ((c == 'p' || c == 'P') && !running) {
      stopDrive();
      stopSteering();
      Serial.print("STRAIGHT_CENTER_ADC=");
      Serial.println(analogRead(POT));
    }
    if ((c == 'c' || c == 'C') && !running) {
      stopDrive();
      running = true;
      state = 0;
      Serial.println("START");
    }
  }

  if (!running) {
    return;
  }

  const unsigned long now = millis();

  switch (state) {
    case 0:
      Serial.println("Searching LEFT end...");
      steerLeft();
      lastPot = analogRead(POT);
      lastSample = now;
      lastChange = now;
      stateStart = now;
      state = 1;
      break;

    case 1: {
      if (now - stateStart > SIDE_TIMEOUT_MS) {
        fail("left timeout");
        break;
      }
      if (now - lastSample >= POT_SAMPLE_MS) {
        lastSample = now;
        int p = analogRead(POT);
        if (abs(p - lastPot) > POT_CHANGE_THRESHOLD) {
          lastPot = p;
          lastChange = now;
        }
      }
      if ((now - stateStart >= MIN_MOVE_MS) && (now - lastChange >= END_STOP_STILL_MS)) {
        stopSteering();
        stableMin = 1023;
        stableMax = 0;
        stateStart = now;
        state = 2;
      }
      break;
    }

    case 2: {
      int p = analogRead(POT);
      stableMin = min(stableMin, p);
      stableMax = max(stableMax, p);
      if (now - stateStart >= STABLE_MS) {
        leftValue = (stableMin + stableMax) / 2;
        Serial.print("LEFT=");
        Serial.print(leftValue);
        Serial.print(" jitter=");
        Serial.println(stableMax - stableMin);
        state = 3;
      }
      break;
    }

    case 3:
      Serial.println("Searching RIGHT end...");
      steerRight();
      lastPot = analogRead(POT);
      lastSample = now;
      lastChange = now;
      stateStart = now;
      state = 4;
      break;

    case 4: {
      if (now - stateStart > SIDE_TIMEOUT_MS) {
        fail("right timeout");
        break;
      }
      if (now - lastSample >= POT_SAMPLE_MS) {
        lastSample = now;
        int p = analogRead(POT);
        if (abs(p - lastPot) > POT_CHANGE_THRESHOLD) {
          lastPot = p;
          lastChange = now;
        }
      }
      if ((now - stateStart >= MIN_MOVE_MS) && (now - lastChange >= END_STOP_STILL_MS)) {
        stopSteering();
        stableMin = 1023;
        stableMax = 0;
        stateStart = now;
        state = 5;
      }
      break;
    }

    case 5: {
      int p = analogRead(POT);
      stableMin = min(stableMin, p);
      stableMax = max(stableMax, p);
      if (now - stateStart >= STABLE_MS) {
        rightValue = (stableMin + stableMax) / 2;
        centerValue = (leftValue + rightValue) / 2;
        Serial.print("RIGHT=");
        Serial.print(rightValue);
        Serial.print(" jitter=");
        Serial.println(stableMax - stableMin);

        if (abs(leftValue - rightValue) < MIN_VALID_SPAN) {
          fail("span too small - check POT wiring");
          break;
        }

        Serial.println("Returning to CENTER...");
        stateStart = now;
        state = 6;
      }
      break;
    }

    case 6: {
      if (now - stateStart > CENTER_TIMEOUT_MS) {
        fail("center timeout");
        break;
      }
      int p = analogRead(POT);
      if (abs(p - centerValue) <= CENTER_DEADBAND_ADC) {
        stopSteering();
        running = false;
        state = 0;
        Serial.println("=== RESULT ===");
        Serial.print("RESULT_LEFT="); Serial.println(leftValue);
        Serial.print("RESULT_RIGHT="); Serial.println(rightValue);
        Serial.print("RESULT_CENTER="); Serial.println(centerValue);
        Serial.print("RESULT_SPAN="); Serial.println(abs(leftValue - rightValue));
        Serial.println("Copy LEFT/RIGHT into DEFAULT_LEFT/DEFAULT_RIGHT in driving_user_pins.ino");
        break;
      }

      int step = mappedStep(p);
      if (step > 0) steerLeft();
      else steerRight();
      break;
    }
  }
}
