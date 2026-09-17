/*
 * SKKU autonomous vehicle - ROS2 serial control firmware
 *
 * User-confirmed wiring:
 *   Steering:   IN1=D3, IN2=D2
 *   Right rear: IN1=D4, IN2=D5
 *   Left rear:  IN1=D7, IN2=D6
 *   Potentiometer OUT=A2
 *
 * Normal serial command (115200 baud):
 *   s{steering}l{left_speed}r{right_speed}\n
 *   steering: -7 .. +7  (negative=left, positive=right)
 *   speed:    -255 .. +255
 *
 * Runtime calibration protocol:
 *   ?                 -> CONFIG,left=...,right=...,center=...,max_step=7
 *   C                 -> measure physical left/right steering limits, then center
 *                        and report CAL_RESULT,left=...,right=...,center=...,span=...
 *   K{left},{right}    -> apply measured calibration for THIS power/run only
 *                        and report CAL_APPLIED,...
 *   X                 -> emergency stop + steering target center
 *
 * IMPORTANT
 *   DEFAULT_LEFT / DEFAULT_RIGHT are the baseline values measured with
 *   steering_limit_calibration.ino.  Runtime auto-calibration is intentionally
 *   accepted by the PC only when the new values are close to this baseline.
 */

#include <stdlib.h>
#include <string.h>

const unsigned int MAX_INPUT = 48;

// -----------------------------------------------------------------------------
// User-confirmed wiring
// -----------------------------------------------------------------------------
const int STEERING_1 = 3;
const int STEERING_2 = 2;
const int FORWARD_RIGHT_1 = 4;
const int FORWARD_RIGHT_2 = 5;
const int FORWARD_LEFT_1 = 7;
const int FORWARD_LEFT_2 = 6;
const int POT = A2;

// -----------------------------------------------------------------------------
// Vehicle configuration
// -----------------------------------------------------------------------------
const int MAX_STEERING_STEP = 7;
const int STEERING_SPEED = 128;

// Replace these ONCE after running steering_limit_calibration.ino on this car.
const int DEFAULT_LEFT = 600;
const int DEFAULT_RIGHT = 445;

// Runtime values.  The PC may update these with K<left>,<right> after startup
// calibration.  They are intentionally not written to EEPROM.
int runtime_left = DEFAULT_LEFT;
int runtime_right = DEFAULT_RIGHT;
int runtime_center = (DEFAULT_LEFT + DEFAULT_RIGHT) / 2;

int target_angle = 0;
int left_speed = 0;
int right_speed = 0;

unsigned long lastControlTime = 0;
unsigned long lastStatusTime = 0;
const unsigned int CONTROL_INTERVAL_MS = 30;
const unsigned int STATUS_INTERVAL_MS = 500;

// -----------------------------------------------------------------------------
// Automatic physical-limit measurement
// -----------------------------------------------------------------------------
const int CAL_STEERING_SPEED = 90;      // deliberately lower than normal control
const int POT_SAMPLE_MS = 30;
const int POT_CHANGE_THRESHOLD = 1;
const int END_STOP_STILL_MS = 350;
const int MIN_MOVE_MS = 300;
const int END_STABLE_MS = 450;
const int CENTER_TIMEOUT_MS = 5000;
const int SIDE_TIMEOUT_MS = 6500;
const int CENTER_DEADBAND_ADC = 3;
const int MIN_VALID_SPAN = 80;

bool calibration_running = false;
bool calibration_waiting_apply = false;
int calibration_state = 0;
unsigned long state_start_ms = 0;
unsigned long last_pot_change_ms = 0;
unsigned long last_pot_sample_ms = 0;
int last_pot_value = 0;
int stable_min = 1023;
int stable_max = 0;
int measured_left = 0;
int measured_right = 0;
int measured_center = 0;

// -----------------------------------------------------------------------------
// Prototypes
// -----------------------------------------------------------------------------
void steerRight(int pwm = STEERING_SPEED);
void steerLeft(int pwm = STEERING_SPEED);
void maintainSteering();
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);
void processIncomingByte(const byte inByte);
void processData(const char *data);
void stopDriveMotors();
void emergencyStop();
int potToStep(int pot, int leftValue, int rightValue);
void startCalibration();
void runCalibration();
void calibrationFail(const char *reason);
void printConfig();

void setup() {
  Serial.begin(115200);

  pinMode(POT, INPUT);
  pinMode(STEERING_1, OUTPUT);
  pinMode(STEERING_2, OUTPUT);
  pinMode(FORWARD_RIGHT_1, OUTPUT);
  pinMode(FORWARD_RIGHT_2, OUTPUT);
  pinMode(FORWARD_LEFT_1, OUTPUT);
  pinMode(FORWARD_LEFT_2, OUTPUT);

  emergencyStop();
  delay(1200);  // allow the USB serial connection to settle after reset
  Serial.println("ARDUINO_BOOTED");
  printConfig();
}

void loop() {
  while (Serial.available() > 0) {
    processIncomingByte(Serial.read());
  }

  const unsigned long now = millis();

  if (calibration_running) {
    runCalibration();
    return;
  }

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    const int resistance = analogRead(POT);
    const int current_angle = potToStep(resistance, runtime_left, runtime_right);

    if (current_angle == target_angle) {
      maintainSteering();
    } else if (current_angle > target_angle) {
      // Current steering is more to the right than target -> physically steer left.
      steerLeft();
    } else {
      steerRight();
    }

    setLeftMotorSpeed(left_speed);
    setRightMotorSpeed(right_speed);
    lastControlTime = now;
  }

  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    Serial.print("STATUS,pot=");
    Serial.print(analogRead(POT));
    Serial.print(",target=");
    Serial.print(target_angle);
    Serial.print(",left_speed=");
    Serial.print(left_speed);
    Serial.print(",right_speed=");
    Serial.println(right_speed);
    lastStatusTime = now;
  }
}

// -----------------------------------------------------------------------------
// Steering / drive helpers
// -----------------------------------------------------------------------------
void steerRight(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(STEERING_1, pwm);
  analogWrite(STEERING_2, 0);
}

void steerLeft(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(STEERING_1, 0);
  analogWrite(STEERING_2, pwm);
}

void maintainSteering() {
  analogWrite(STEERING_1, 0);
  analogWrite(STEERING_2, 0);
}

void setLeftMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(FORWARD_LEFT_1, speed);
    analogWrite(FORWARD_LEFT_2, 0);
  } else if (speed < 0) {
    analogWrite(FORWARD_LEFT_1, 0);
    analogWrite(FORWARD_LEFT_2, -speed);
  } else {
    analogWrite(FORWARD_LEFT_1, 0);
    analogWrite(FORWARD_LEFT_2, 0);
  }
}

void setRightMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(FORWARD_RIGHT_1, speed);
    analogWrite(FORWARD_RIGHT_2, 0);
  } else if (speed < 0) {
    analogWrite(FORWARD_RIGHT_1, 0);
    analogWrite(FORWARD_RIGHT_2, -speed);
  } else {
    analogWrite(FORWARD_RIGHT_1, 0);
    analogWrite(FORWARD_RIGHT_2, 0);
  }
}

void stopDriveMotors() {
  left_speed = 0;
  right_speed = 0;
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
}

void emergencyStop() {
  target_angle = 0;
  stopDriveMotors();
  maintainSteering();
}

int potToStep(int pot, int leftValue, int rightValue) {
  if (leftValue == rightValue) return 0;
  long mapped = map(pot, leftValue, rightValue, -MAX_STEERING_STEP, MAX_STEERING_STEP);
  return constrain((int)mapped, -MAX_STEERING_STEP, MAX_STEERING_STEP);
}

void printConfig() {
  Serial.print("CONFIG,left=");
  Serial.print(runtime_left);
  Serial.print(",right=");
  Serial.print(runtime_right);
  Serial.print(",center=");
  Serial.print(runtime_center);
  Serial.print(",max_step=");
  Serial.println(MAX_STEERING_STEP);
}

// -----------------------------------------------------------------------------
// Auto calibration state machine
// -----------------------------------------------------------------------------
void startCalibration() {
  if (calibration_running) return;

  emergencyStop();
  calibration_waiting_apply = false;
  calibration_running = true;
  calibration_state = 0;
  state_start_ms = millis();
  Serial.println("CAL_START");
}

void calibrationFail(const char *reason) {
  maintainSteering();
  stopDriveMotors();
  calibration_running = false;
  calibration_waiting_apply = false;
  calibration_state = 0;
  Serial.print("CAL_ERROR,reason=");
  Serial.println(reason);
}

void runCalibration() {
  const unsigned long now = millis();

  switch (calibration_state) {
    case 0:  // start moving to physical LEFT end
      stopDriveMotors();
      Serial.println("CAL_PHASE,left_search");
      steerLeft(CAL_STEERING_SPEED);
      last_pot_value = analogRead(POT);
      last_pot_change_ms = now;
      last_pot_sample_ms = now;
      state_start_ms = now;
      calibration_state = 1;
      break;

    case 1: {  // detect left mechanical end by potentiometer no longer changing
      if (now - state_start_ms > SIDE_TIMEOUT_MS) {
        calibrationFail("left_timeout");
        break;
      }
      if (now - last_pot_sample_ms >= POT_SAMPLE_MS) {
        last_pot_sample_ms = now;
        const int p = analogRead(POT);
        if (abs(p - last_pot_value) > POT_CHANGE_THRESHOLD) {
          last_pot_value = p;
          last_pot_change_ms = now;
        }
      }
      if ((now - state_start_ms >= MIN_MOVE_MS) &&
          (now - last_pot_change_ms >= END_STOP_STILL_MS)) {
        maintainSteering();
        stable_min = 1023;
        stable_max = 0;
        state_start_ms = now;
        calibration_state = 2;
        Serial.println("CAL_PHASE,left_stable");
      }
      break;
    }

    case 2: {  // stable averaging at left end
      const int p = analogRead(POT);
      stable_min = min(stable_min, p);
      stable_max = max(stable_max, p);
      if (now - state_start_ms >= END_STABLE_MS) {
        measured_left = (stable_min + stable_max) / 2;
        Serial.print("CAL_LEFT,value=");
        Serial.print(measured_left);
        Serial.print(",jitter=");
        Serial.println(stable_max - stable_min);
        calibration_state = 3;
      }
      break;
    }

    case 3:  // start moving to physical RIGHT end
      Serial.println("CAL_PHASE,right_search");
      steerRight(CAL_STEERING_SPEED);
      last_pot_value = analogRead(POT);
      last_pot_change_ms = now;
      last_pot_sample_ms = now;
      state_start_ms = now;
      calibration_state = 4;
      break;

    case 4: {
      if (now - state_start_ms > SIDE_TIMEOUT_MS) {
        calibrationFail("right_timeout");
        break;
      }
      if (now - last_pot_sample_ms >= POT_SAMPLE_MS) {
        last_pot_sample_ms = now;
        const int p = analogRead(POT);
        if (abs(p - last_pot_value) > POT_CHANGE_THRESHOLD) {
          last_pot_value = p;
          last_pot_change_ms = now;
        }
      }
      if ((now - state_start_ms >= MIN_MOVE_MS) &&
          (now - last_pot_change_ms >= END_STOP_STILL_MS)) {
        maintainSteering();
        stable_min = 1023;
        stable_max = 0;
        state_start_ms = now;
        calibration_state = 5;
        Serial.println("CAL_PHASE,right_stable");
      }
      break;
    }

    case 5: {  // stable averaging at right end
      const int p = analogRead(POT);
      stable_min = min(stable_min, p);
      stable_max = max(stable_max, p);
      if (now - state_start_ms >= END_STABLE_MS) {
        measured_right = (stable_min + stable_max) / 2;
        measured_center = (measured_left + measured_right) / 2;
        const int span = abs(measured_left - measured_right);

        Serial.print("CAL_RIGHT,value=");
        Serial.print(measured_right);
        Serial.print(",jitter=");
        Serial.println(stable_max - stable_min);

        if (span < MIN_VALID_SPAN) {
          calibrationFail("span_too_small");
          break;
        }

        state_start_ms = now;
        calibration_state = 6;
        Serial.println("CAL_PHASE,centering");
      }
      break;
    }

    case 6: {  // use TEMPORARY measured endpoints only to return to midpoint
      if (now - state_start_ms > CENTER_TIMEOUT_MS) {
        calibrationFail("center_timeout");
        break;
      }

      const int p = analogRead(POT);
      const int err = p - measured_center;
      if (abs(err) <= CENTER_DEADBAND_ADC) {
        maintainSteering();
        calibration_running = false;
        calibration_waiting_apply = true;
        calibration_state = 0;
        Serial.print("CAL_RESULT,left=");
        Serial.print(measured_left);
        Serial.print(",right=");
        Serial.print(measured_right);
        Serial.print(",center=");
        Serial.print(measured_center);
        Serial.print(",span=");
        Serial.println(abs(measured_left - measured_right));
        break;
      }

      const int current_step = potToStep(p, measured_left, measured_right);
      if (current_step > 0) {
        steerLeft(CAL_STEERING_SPEED);
      } else {
        steerRight(CAL_STEERING_SPEED);
      }
      break;
    }
  }
}

// -----------------------------------------------------------------------------
// Serial protocol
// -----------------------------------------------------------------------------
void processIncomingByte(const byte inByte) {
  static char input_line[MAX_INPUT];
  static unsigned int input_pos = 0;

  if (inByte == '\n') {
    input_line[input_pos] = '\0';
    processData(input_line);
    input_pos = 0;
  } else if (inByte != '\r' && input_pos < MAX_INPUT - 1) {
    input_line[input_pos++] = (char)inByte;
  }
}

void processData(const char *data) {
  if (data[0] == '\0') return;

  if (strcmp(data, "?") == 0) {
    Serial.println("ARDUINO_READY");
    printConfig();
    return;
  }

  if (strcmp(data, "X") == 0) {
    calibration_running = false;
    calibration_waiting_apply = false;
    emergencyStop();
    Serial.println("STOPPED");
    return;
  }

  if (strcmp(data, "C") == 0) {
    startCalibration();
    return;
  }

  // Apply runtime calibration only after a successful measurement.
  // Format: K600,445
  if (data[0] == 'K') {
    if (!calibration_waiting_apply) {
      Serial.println("CAL_ERROR,reason=no_pending_measurement");
      return;
    }

    const char *comma = strchr(data + 1, ',');
    if (!comma) {
      Serial.println("CAL_ERROR,reason=bad_apply_format");
      return;
    }

    const int new_left = atoi(data + 1);
    const int new_right = atoi(comma + 1);
    if (new_left < 0 || new_left > 1023 || new_right < 0 || new_right > 1023 ||
        abs(new_left - new_right) < MIN_VALID_SPAN) {
      Serial.println("CAL_ERROR,reason=bad_apply_values");
      return;
    }

    runtime_left = new_left;
    runtime_right = new_right;
    runtime_center = (runtime_left + runtime_right) / 2;
    calibration_waiting_apply = false;
    target_angle = 0;

    Serial.print("CAL_APPLIED,left=");
    Serial.print(runtime_left);
    Serial.print(",right=");
    Serial.print(runtime_right);
    Serial.print(",center=");
    Serial.println(runtime_center);
    return;
  }

  // Ignore driving commands while steering calibration is physically moving.
  if (calibration_running || calibration_waiting_apply) {
    return;
  }

  const char *s = strchr(data, 's');
  const char *l = strchr(data, 'l');
  const char *r = strchr(data, 'r');
  if (!s || !l || !r) return;

  target_angle = constrain(atoi(s + 1), -MAX_STEERING_STEP, MAX_STEERING_STEP);
  left_speed = constrain(atoi(l + 1), -255, 255);
  right_speed = constrain(atoi(r + 1), -255, 255);
}
