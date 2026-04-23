# Cart FSD — Actuator & Pot Test Scripts

## Pin Mapping (Corrected)

### Gas Actuator (H-Bridge: BTS7960)
| Pin    | Arduino |
|--------|---------|
| R_EN   | 12      |
| R_PWM  | 10      |
| L_EN   | 13      |
| L_PWM  | 11      |

### Brake Actuator (H-Bridge: BTS7960)
| Pin    | Arduino |
|--------|---------|
| R_EN   | 6       |
| R_PWM  | 4       |
| L_EN   | 7       |
| L_PWM  | 5       |

### Linear Pots
| Pot       | Arduino Pin |
|-----------|-------------|
| Gas Pot   | A4          |
| Brake Pot | A0          |

### Direction Reference
- **L_PWM** = Forward (pedal engaged: gas pressed / brake pressed)
- **R_PWM** = Backward (pedal released)

> **Note:** The original Notion spec had gas and brake actuator pins swapped. This document reflects the corrected mapping.

---

## Pot Limits & Safety Stop

The linear pots output 0–5V across their **full physical travel** (0.0 to 1.0 normalized). However, each pedal's usable range is only a **subset** of the pot's full motion — the actuator physically can't press the pedal past a certain point, and there's no reason to retract it past its resting position.

To protect the mechanism, each script enforces min/max limits in normalized pot space. **When the pot reading reaches a limit in the direction the motor is moving, the motor stops automatically.**

### Current Calibrated Limits

| Pedal | Min  | Max  | Notes                                                                 |
|-------|------|------|-----------------------------------------------------------------------|
| Gas   | 0.00 | 0.68 | 0.68 = full throttle                                                  |
| Brake | 0.00 | 0.45 | 0.45 = full brake (actuator can't press further, but brakes engaged)  |

### How the Limits Work

- Sending `f` (forward) drives the pedal down. If normalized pot ≥ `MAX`, motor stops.
- Sending `b` (backward) retracts the pedal. If normalized pot ≤ `MIN`, motor stops.
- Sending `s` always stops the motor immediately.
- Direction state is tracked so the limit check only blocks motion that would push *past* the limit — you can always reverse away from a limit.

Tweak `POT_MIN` and `POT_MAX` at the top of each script if calibration changes.

---

## Gas Pedal Test Script

```cpp
const int GAS_R_EN  = 12;
const int GAS_R_PWM = 10;
const int GAS_L_EN  = 13;
const int GAS_L_PWM = 11;

const int GAS_POT = A4;

const int PWM_SPEED = 80;

// Normalized pot limits (0.0 = fully retracted, 1.0 = full pot travel)
const float POT_MIN = 0.00;
const float POT_MAX = 0.68;  // full throttle

// Direction state: 0 = stopped, 1 = forward, -1 = backward
int direction = 0;

void stopMotor() {
  analogWrite(GAS_R_PWM, 0);
  analogWrite(GAS_L_PWM, 0);
  direction = 0;
}

void driveForward() {
  analogWrite(GAS_R_PWM, 0);
  analogWrite(GAS_L_PWM, PWM_SPEED);
  direction = 1;
}

void driveBackward() {
  analogWrite(GAS_L_PWM, 0);
  analogWrite(GAS_R_PWM, PWM_SPEED);
  direction = -1;
}

void setup() {
  Serial.begin(115200);

  pinMode(GAS_R_EN, OUTPUT);
  pinMode(GAS_R_PWM, OUTPUT);
  pinMode(GAS_L_EN, OUTPUT);
  pinMode(GAS_L_PWM, OUTPUT);

  digitalWrite(GAS_R_EN, HIGH);
  digitalWrite(GAS_L_EN, HIGH);

  Serial.println("f = forward, b = backward, s = stop");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'f') {
      driveForward();
      Serial.println("Forward");
    } else if (c == 'b') {
      driveBackward();
      Serial.println("Backward");
    } else if (c == 's') {
      stopMotor();
      Serial.println("Stopped");
    }
  }

  float voltage = analogRead(GAS_POT) * (5.0 / 1023.0);
  float normalized = voltage / 5.0;

  // Safety limit: stop motor if past a limit in the direction of motion
  if (direction == 1 && normalized >= POT_MAX) {
    stopMotor();
    Serial.println("MAX limit reached - stopped");
  } else if (direction == -1 && normalized <= POT_MIN) {
    stopMotor();
    Serial.println("MIN limit reached - stopped");
  }

  Serial.print(voltage, 3);
  Serial.print("V | ");
  Serial.println(normalized, 3);
  delay(50);
}
```

---

## Brake Pedal Test Script

```cpp
const int BRAKE_R_EN  = 6;
const int BRAKE_R_PWM = 4;
const int BRAKE_L_EN  = 7;
const int BRAKE_L_PWM = 5;

const int BRAKE_POT = A0;

const int PWM_SPEED = 80;

// Normalized pot limits (0.0 = fully retracted, 1.0 = full pot travel)
const float POT_MIN = 0.00;
const float POT_MAX = 0.45;  // full brake (mechanical limit)

// Direction state: 0 = stopped, 1 = forward, -1 = backward
int direction = 0;

void stopMotor() {
  analogWrite(BRAKE_R_PWM, 0);
  analogWrite(BRAKE_L_PWM, 0);
  direction = 0;
}

void driveForward() {
  analogWrite(BRAKE_R_PWM, 0);
  analogWrite(BRAKE_L_PWM, PWM_SPEED);
  direction = 1;
}

void driveBackward() {
  analogWrite(BRAKE_L_PWM, 0);
  analogWrite(BRAKE_R_PWM, PWM_SPEED);
  direction = -1;
}

void setup() {
  Serial.begin(115200);

  pinMode(BRAKE_R_EN, OUTPUT);
  pinMode(BRAKE_R_PWM, OUTPUT);
  pinMode(BRAKE_L_EN, OUTPUT);
  pinMode(BRAKE_L_PWM, OUTPUT);

  digitalWrite(BRAKE_R_EN, HIGH);
  digitalWrite(BRAKE_L_EN, HIGH);

  Serial.println("f = forward, b = backward, s = stop");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'f') {
      driveForward();
      Serial.println("Forward");
    } else if (c == 'b') {
      driveBackward();
      Serial.println("Backward");
    } else if (c == 's') {
      stopMotor();
      Serial.println("Stopped");
    }
  }

  float voltage = analogRead(BRAKE_POT) * (5.0 / 1023.0);
  float normalized = voltage / 5.0;

  // Safety limit: stop motor if past a limit in the direction of motion
  if (direction == 1 && normalized >= POT_MAX) {
    stopMotor();
    Serial.println("MAX limit reached - stopped");
  } else if (direction == -1 && normalized <= POT_MIN) {
    stopMotor();
    Serial.println("MIN limit reached - stopped");
  }

  Serial.print(voltage, 3);
  Serial.print("V | ");
  Serial.println(normalized, 3);
  delay(50);
}
```