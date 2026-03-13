/*
 * ============================================
 *  Smart Farm — Stepper + Pump + Soil Moisture
 *  ESP32 + Stepper Driver (PUL/DIR)
 * ============================================
 * 
 *  Wiring:
 *    Stepper Driver:
 *      PUL+ → GND      DIR+ → GND
 *      PUL- → Pin 27   DIR- → Pin 14
 *      (Active LOW — เนื่องจาก PUL+/DIR+ ต่อ GND)
 * 
 *    Limit Switches (NO, ต่อกับ GND):
 *      Start → Pin 16 (INPUT_PULLUP)
 *      End   → Pin 17 (INPUT_PULLUP)
 * 
 *    Relay (Active LOW — ปั้มน้ำ):
 *      Pump 1 → Pin 12
 *      Pump 2 → Pin 13
 * 
 *    Soil Moisture (Analog 0-4095):
 *      Sensor 1 → Pin 32
 *      Sensor 2 → Pin 33
 * 
 *  Serial Commands (115200 baud):
 *    calibrate          → Calibrate: homing → วัดระยะทาง
 *    home               → กลับไปตำแหน่ง Start
 *    move <steps>       → เลื่อน N steps (บวก=ไป End, ลบ=ไป Start)
 *    goto <position>    → ไปตำแหน่ง step ที่ระบุ
 *    speed <sps>        → ตั้งความเร็ว (steps/sec)
 *    status             → แสดงสถานะปัจจุบัน
 *    stop               → หยุดทันที
 *    pump <1/2> <on/off>→ เปิด/ปิดปั้มน้ำ
 *    soil               → อ่านค่า soil moisture
 *    sensors            → อ่าน sensor ทั้งหมด (soil + pump state)
 * ============================================
 */

// ===== Pin Definitions =====
const int PIN_PUL = 27;       // Pulse (PUL-)
const int PIN_DIR = 14;       // Direction (DIR-)
const int PIN_SW_START = 17;  // Limit Switch - Start (Home)
const int PIN_SW_END = 16;    // Limit Switch - End

// Relay — ปั้มน้ำ (Active LOW: LOW = ON, HIGH = OFF)
const int PIN_RELAY[2] = { 13, 12 };  // Pump1=Pot1, Pump2=Pot2

// Soil Moisture — Analog (0-4095)
const int PIN_SOIL[2] = { 33, 32 };   // Soil1=Pot1, Soil2=Pot2

// ===== Motor Settings =====
// เนื่องจาก PUL+/DIR+ ต่อ GND → สัญญาณ Active LOW
// LOW = Active, HIGH = Inactive
const int ACTIVE = LOW;
const int INACTIVE = HIGH;

// Direction: ปรับทิศทางตรงนี้ถ้าหมุนกลับด้าน
const int DIR_TOWARD_END = ACTIVE;      // ทิศทางไปหา End switch
const int DIR_TOWARD_START = INACTIVE;  // ทิศทางไปหา Start switch

// ===== Variables =====
long currentPosition = 0;    // ตำแหน่งปัจจุบัน (steps)
long totalSteps = 0;         // จำนวน steps ทั้งหมด (Start → End)
bool isCalibrated = false;   // ผ่านการ calibrate แล้วหรือยัง
bool isMoving = false;       // กำลังเคลื่อนที่อยู่หรือไม่
bool stopRequested = false;  // มีคำสั่งหยุดหรือไม่

int stepDelay = 100;         // Microseconds ระหว่าง pulse (ยิ่งน้อยยิ่งเร็ว)
                             // 500µs = 1000 steps/sec
int calibrationSpeed = 100;  // ความเร็วตอน calibrate (µs, ช้ากว่าปกติ)

bool pumpState[2] = { false, false };  // สถานะปั้มน้ำ (false=OFF)

// ===== Setup =====
void setup() {
  Serial.begin(115200);

  // ตั้งค่า pins — Stepper
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SW_START, INPUT_PULLUP);
  pinMode(PIN_SW_END, INPUT_PULLUP);

  // ปิด outputs เริ่มต้น
  digitalWrite(PIN_PUL, INACTIVE);
  digitalWrite(PIN_DIR, INACTIVE);

  // ตั้งค่า pins — Relay (ปั้มน้ำ)
  for (int i = 0; i < 2; i++) {
    pinMode(PIN_RELAY[i], OUTPUT);
    digitalWrite(PIN_RELAY[i], HIGH);  // HIGH = OFF (Active LOW)
  }

  delay(500);
  Serial.println();
  Serial.println("============================================");
  Serial.println("  Stepper Motor Controller - Ready");
  Serial.println("============================================");
  Serial.println("Commands:");
  Serial.println("  calibrate           - Home + measure travel");
  Serial.println("  home                - Go to Start position");
  Serial.println("  move <steps>        - Relative move (+/-)");
  Serial.println("  goto <position>     - Absolute move");
  Serial.println("  speed <sps>         - Set speed (steps/sec)");
  Serial.println("  status              - Show current state");
  Serial.println("  stop                - Emergency stop");
  Serial.println("  pump <1/2> <on/off> - Water pump control");
  Serial.println("  soil                - Read soil moisture");
  Serial.println("  sensors             - Read all sensors");
  Serial.println("============================================");
  printStatus();
}

// ===== Main Loop =====
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      processCommand(input);
    }
  }
}

// ===== Command Processing =====
void processCommand(String cmd) {
  cmd.toLowerCase();

  if (cmd == "calibrate") {
    calibrate();
  } else if (cmd == "home") {
    goHome();
  } else if (cmd.startsWith("move ")) {
    long steps = cmd.substring(5).toInt();
    if (steps == 0 && cmd.substring(5) != "0") {
      Serial.println("ERR: Invalid step count");
      return;
    }
    moveRelative(steps);
  } else if (cmd.startsWith("goto ")) {
    long pos = cmd.substring(5).toInt();
    moveAbsolute(pos);
  } else if (cmd.startsWith("speed ")) {
    int sps = cmd.substring(6).toInt();
    if (sps <= 0) {
      Serial.println("ERR: Speed must be > 0");
      return;
    }
    // แปลง steps/sec → microseconds delay
    stepDelay = 1000000 / sps / 2;         // หาร 2 เพราะ 1 step = HIGH + LOW
    if (stepDelay < 100) stepDelay = 100;  // จำกัดความเร็วสูงสุด
    Serial.print("OK: Speed set to ");
    Serial.print(sps);
    Serial.print(" steps/sec (delay=");
    Serial.print(stepDelay);
    Serial.println("us)");
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "stop") {
    stopRequested = true;
    Serial.println("OK: Stop requested");
  } else if (cmd.startsWith("pump ")) {
    handlePump(cmd);
  } else if (cmd == "soil") {
    readSoilMoisture();
  } else if (cmd == "sensors") {
    readAllSensors();
  } else if (cmd == "switches") {
    readSwitches();
  } else {
    Serial.print("ERR: Unknown command: ");
    Serial.println(cmd);
  }
}

// ===== Limit Switch Reading (with Debounce) =====
bool isStartPressed() {
  // อ่าน 3 ครั้ง — ต้อง LOW ทั้ง 3 ครั้งจึงถือว่ากด
  int count = 0;
  for (int i = 0; i < 3; i++) {
    if (digitalRead(PIN_SW_START) == LOW) count++;
    delayMicroseconds(200);
  }
  return count >= 3;
}

bool isEndPressed() {
  int count = 0;
  for (int i = 0; i < 3; i++) {
    if (digitalRead(PIN_SW_END) == LOW) count++;
    delayMicroseconds(200);
  }
  return count >= 3;
}

// ===== Switch Debug =====
void readSwitches() {
  Serial.println("--- Switch Status ---");
  Serial.print("  Start (Pin ");
  Serial.print(PIN_SW_START);
  Serial.print("): raw=");
  Serial.print(digitalRead(PIN_SW_START));
  Serial.print(" → ");
  Serial.println(isStartPressed() ? "PRESSED" : "open");
  Serial.print("  End   (Pin ");
  Serial.print(PIN_SW_END);
  Serial.print("): raw=");
  Serial.print(digitalRead(PIN_SW_END));
  Serial.print(" → ");
  Serial.println(isEndPressed() ? "PRESSED" : "open");
  Serial.println("OK: Switch readings done");
}

// ===== Single Step =====
void stepOnce(int direction, int delayUs) {
  digitalWrite(PIN_DIR, direction);
  delayMicroseconds(10);  // ให้ DIR เซ็ตก่อน

  digitalWrite(PIN_PUL, ACTIVE);
  delayMicroseconds(delayUs);
  digitalWrite(PIN_PUL, INACTIVE);
  delayMicroseconds(delayUs);
}

// ===== Calibration =====
void calibrate() {
  Serial.println("--- Calibration Start ---");
  isCalibrated = false;
  stopRequested = false;

  // === Phase 1: หา Start (Home) ===
  Serial.println("[1/3] Homing → moving toward Start switch...");

  if (isStartPressed()) {
    // ถ้าอยู่ที่ Start แล้ว ถอยออกมาก่อน
    Serial.println("  Already at Start, backing off...");
    while (isStartPressed() && !stopRequested) {
      stepOnce(DIR_TOWARD_END, calibrationSpeed);
    }
    if (stopRequested) {
      Serial.println("STOPPED");
      return;
    }
    delay(200);
  }

  // เลื่อนไปหา Start switch
  long safetyLimit = 500000;  // ป้องกันวนไม่จบ
  long count = 0;
  while (!isStartPressed() && !stopRequested && count < safetyLimit) {
    stepOnce(DIR_TOWARD_START, calibrationSpeed);
    count++;
  }

  if (stopRequested) {
    Serial.println("STOPPED");
    return;
  }
  if (count >= safetyLimit) {
    Serial.println("ERR: Start switch not found! Check wiring.");
    return;
  }

  Serial.print("  Start switch hit after ");
  Serial.print(count);
  Serial.println(" steps");

  // ถอยออกจาก switch เล็กน้อย
  delay(200);
  while (isStartPressed() && !stopRequested) {
    stepOnce(DIR_TOWARD_END, calibrationSpeed);
  }
  // เลื่อนเพิ่มอีกนิดให้ห่าง switch
  for (int i = 0; i < 50 && !stopRequested; i++) {
    stepOnce(DIR_TOWARD_END, calibrationSpeed);
  }

  if (stopRequested) {
    Serial.println("STOPPED");
    return;
  }

  currentPosition = 0;
  Serial.println("  Home position set (position = 0)");

  // === Phase 2: วัดระยะทางไป End ===
  Serial.println("[2/3] Measuring travel → moving toward End switch...");
  delay(300);

  totalSteps = 0;
  while (!isEndPressed() && !stopRequested && totalSteps < safetyLimit) {
    stepOnce(DIR_TOWARD_END, calibrationSpeed);
    totalSteps++;
  }

  if (stopRequested) {
    Serial.println("STOPPED");
    return;
  }
  if (totalSteps >= safetyLimit) {
    Serial.println("ERR: End switch not found! Check wiring.");
    return;
  }

  Serial.print("  End switch hit at step ");
  Serial.println(totalSteps);

  // === Phase 3: กลับ Home ===
  Serial.println("[3/3] Returning to Home...");
  delay(200);

  // ถอยออกจาก End switch
  while (isEndPressed() && !stopRequested) {
    stepOnce(DIR_TOWARD_START, calibrationSpeed);
  }
  delay(100);

  // เลื่อนกลับ Home
  long stepsBack = 0;
  while (!isStartPressed() && !stopRequested) {
    stepOnce(DIR_TOWARD_START, calibrationSpeed);
    stepsBack++;
  }

  if (stopRequested) {
    Serial.println("STOPPED");
    return;
  }

  // ถอยออกจาก Start switch
  delay(200);
  while (isStartPressed() && !stopRequested) {
    stepOnce(DIR_TOWARD_END, calibrationSpeed);
  }
  for (int i = 0; i < 50 && !stopRequested; i++) {
    stepOnce(DIR_TOWARD_END, calibrationSpeed);
  }

  currentPosition = 0;
  isCalibrated = true;

  Serial.println("--- Calibration Complete ---");
  Serial.print("  Total travel: ");
  Serial.print(totalSteps);
  Serial.println(" steps");
  printStatus();
}

// ===== Go Home (ไปหา Start switch จริง) =====
void goHome() {
  Serial.println("Going Home...");
  stopRequested = false;
  isMoving = true;

  // ถ้าอยู่ที่ Start แล้ว ถอยออกก่อน
  if (isStartPressed()) {
    while (isStartPressed() && !stopRequested) {
      stepOnce(DIR_TOWARD_END, stepDelay);
    }
    delay(100);
  }

  // เลื่อนไปหา Start switch
  long safetyLimit = 500000;
  long count = 0;
  while (!isStartPressed() && !stopRequested && count < safetyLimit) {
    stepOnce(DIR_TOWARD_START, stepDelay);
    count++;
    if (count % 1000 == 0) {
      Serial.print("  homing: ");
      Serial.print(count);
      Serial.println(" steps");
    }
  }

  if (stopRequested) {
    Serial.println("STOPPED");
    isMoving = false;
    return;
  }

  if (count >= safetyLimit) {
    Serial.println("ERR: Start switch not found!");
    isMoving = false;
    return;
  }

  // ถอยออกจาก switch เล็กน้อย
  delay(200);
  while (isStartPressed() && !stopRequested) {
    stepOnce(DIR_TOWARD_END, stepDelay);
  }
  for (int i = 0; i < 50 && !stopRequested; i++) {
    stepOnce(DIR_TOWARD_END, stepDelay);
  }

  currentPosition = 0;
  isMoving = false;
  Serial.println("OK: Arrived at position 0");
}

// ===== Relative Move =====
void moveRelative(long steps) {
  if (steps == 0) {
    Serial.println("OK: No movement needed");
    return;
  }

  long targetPos = currentPosition + steps;

  Serial.print("Moving ");
  Serial.print(steps);
  Serial.print(" steps (");
  Serial.print(currentPosition);
  Serial.print(" → ");
  Serial.print(targetPos);
  Serial.println(")");

  moveAbsolute(targetPos);
}

// ===== Absolute Move =====
void moveAbsolute(long targetPos) {
  // ตรวจสอบขอบเขต (ถ้า calibrate แล้ว)
  if (isCalibrated) {
    if (targetPos < 0) {
      Serial.print("WARN: Clamping to 0 (was ");
      Serial.print(targetPos);
      Serial.println(")");
      targetPos = 0;
    }
    if (targetPos > totalSteps) {
      Serial.print("WARN: Clamping to ");
      Serial.print(totalSteps);
      Serial.print(" (was ");
      Serial.print(targetPos);
      Serial.println(")");
      targetPos = totalSteps;
    }
  }

  long stepsToMove = targetPos - currentPosition;
  if (stepsToMove == 0) {
    Serial.println("OK: Already at target position");
    return;
  }

  int direction;
  long absSteps;
  if (stepsToMove > 0) {
    direction = DIR_TOWARD_END;
    absSteps = stepsToMove;
  } else {
    direction = DIR_TOWARD_START;
    absSteps = -stepsToMove;
  }

  isMoving = true;
  stopRequested = false;

  Serial.print("Moving to position ");
  Serial.print(targetPos);
  Serial.print(" (");
  Serial.print(absSteps);
  Serial.print(" steps ");
  Serial.print(stepsToMove > 0 ? "→ End" : "← Start");
  Serial.println(")...");

  for (long i = 0; i < absSteps; i++) {
    if (stopRequested) {
      Serial.print("STOPPED at position ");
      Serial.println(currentPosition);
      isMoving = false;
      return;
    }

    // ตรวจ limit switch ป้องกันชน
    if (direction == DIR_TOWARD_START && isStartPressed()) {
      Serial.println("LIMIT: Start switch hit!");
      currentPosition = 0;
      isMoving = false;
      return;
    }
    if (direction == DIR_TOWARD_END && isEndPressed()) {
      Serial.println("LIMIT: End switch hit!");
      if (isCalibrated) currentPosition = totalSteps;
      isMoving = false;
      return;
    }

    stepOnce(direction, stepDelay);

    // อัพเดทตำแหน่ง
    if (direction == DIR_TOWARD_END) {
      currentPosition++;
    } else {
      currentPosition--;
    }

    // แสดง progress ทุก 1000 steps
    if ((i + 1) % 1000 == 0) {
      Serial.print("  progress: ");
      Serial.print(i + 1);
      Serial.print("/");
      Serial.print(absSteps);
      Serial.print(" (pos=");
      Serial.print(currentPosition);
      Serial.println(")");
    }
  }

  isMoving = false;
  Serial.print("OK: Arrived at position ");
  Serial.println(currentPosition);
}

// ===== Print Status =====
void printStatus() {
  Serial.println("--- Status ---");
  Serial.print("  Calibrated:  ");
  Serial.println(isCalibrated ? "YES" : "NO");
  Serial.print("  Position:    ");
  Serial.println(currentPosition);
  Serial.print("  Total range: ");
  Serial.println(isCalibrated ? String(totalSteps) + " steps" : "N/A");
  Serial.print("  Speed:       ");
  Serial.print(1000000 / (stepDelay * 2));
  Serial.println(" steps/sec");
  Serial.print("  Moving:      ");
  Serial.println(isMoving ? "YES" : "NO");
  Serial.print("  Start SW:    ");
  Serial.println(isStartPressed() ? "PRESSED" : "open");
  Serial.print("  End SW:      ");
  Serial.println(isEndPressed() ? "PRESSED" : "open");
  Serial.print("  Pump 1:      ");
  Serial.println(pumpState[0] ? "ON" : "OFF");
  Serial.print("  Pump 2:      ");
  Serial.println(pumpState[1] ? "ON" : "OFF");
  Serial.print("  Soil 1:      ");
  Serial.println(analogRead(PIN_SOIL[0]));
  Serial.print("  Soil 2:      ");
  Serial.println(analogRead(PIN_SOIL[1]));
  Serial.println("--------------");
}

// ===== Pump Control =====
void handlePump(String cmd) {
  // cmd = "pump 1 on" หรือ "pump 2 off"
  int spaceIdx = cmd.indexOf(' ', 5);  // หา space หลังตัวเลข
  if (spaceIdx < 0) {
    Serial.println("ERR: Usage: pump <1/2> <on/off>");
    return;
  }
  int pumpNum = cmd.substring(5, spaceIdx).toInt();
  String action = cmd.substring(spaceIdx + 1);
  action.trim();

  if (pumpNum < 1 || pumpNum > 2) {
    Serial.println("ERR: Pump number must be 1 or 2");
    return;
  }

  int idx = pumpNum - 1;
  if (action == "on") {
    pumpState[idx] = true;
    digitalWrite(PIN_RELAY[idx], LOW);  // Active LOW → ON
    Serial.print("OK: Pump ");
    Serial.print(pumpNum);
    Serial.println(" ON");
  } else if (action == "off") {
    pumpState[idx] = false;
    digitalWrite(PIN_RELAY[idx], HIGH);  // HIGH → OFF
    Serial.print("OK: Pump ");
    Serial.print(pumpNum);
    Serial.println(" OFF");
  } else {
    Serial.println("ERR: Usage: pump <1/2> <on/off>");
  }
}

// ===== Soil Moisture =====
void readSoilMoisture() {
  Serial.println("--- Soil Moisture ---");
  for (int i = 0; i < 2; i++) {
    int raw = analogRead(PIN_SOIL[i]);
    Serial.print("  Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(raw);
    Serial.print(" / 4095");
    Serial.println();
  }
  Serial.println("OK: Soil readings done");
}

// ===== Read All Sensors (machine-readable) =====
void readAllSensors() {
  int s1 = analogRead(PIN_SOIL[0]);
  int s2 = analogRead(PIN_SOIL[1]);
  Serial.print("SENSORS:soil1=");
  Serial.print(s1);
  Serial.print(",soil2=");
  Serial.print(s2);
  Serial.print(",pump1=");
  Serial.print(pumpState[0] ? 1 : 0);
  Serial.print(",pump2=");
  Serial.println(pumpState[1] ? 1 : 0);
}
