#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFun_APDS9960.h>

// =====================================================
//  โหมดใช้งาน
//  0 = วิ่งจริง
//  1 = calibrate เซนเซอร์สี (ปริ้นค่า prox และ amb + บอกว่าเป็นเทปดำหรือไม่)
//  2 = calibrate ToF (ปริ้นค่า L / F / R)
//  3 = ทดสอบเลี้ยว (ขวา 4 ครั้ง แล้วซ้าย 4 ครั้ง ต้องกลับมาหันทิศเดิม)
// =====================================================
#define MODE 0

// เลือกวิธีตรวจเทปดำ
//  0 = ใช้ proximity (IR)  : พื้นขาว = ค่าสูง, เทปดำ = ค่าต่ำ
//  1 = ใช้ ambient light   : พื้นขาว = ค่าสูง, เทปดำ = ค่าต่ำ
#define TAPE_USE_AMBIENT 0

// 1 = ปริ้นค่าเซนเซอร์สีเมื่อเริ่มต่ำ (ใช้ตอนแก้ปัญหาเท่านั้น)
#define TAPE_DEBUG 0

// ---------- VL53L0X ----------
// L = เฉียงซ้าย, F = หน้า, R = เฉียงขวา
VL53L0X sensorL, sensorF, sensorR;
const int XSHUT_L = 10, XSHUT_F = 9, XSHUT_R = 8;

// ---------- GY-9960 (APDS9960) ติดหลังสุดใต้ตัว ----------
SparkFun_APDS9960 apds;
const uint8_t  PROX_BLACK = 200;   // ต่ำกว่านี้ = เทปดำ (calibrate ด้วย MODE 1)
const uint16_t AMB_BLACK  = 5;     // ต่ำกว่านี้ = เทปดำ (calibrate ด้วย MODE 1)
const uint8_t  TAPE_CONFIRM = 1;   // ต้องเห็นเทปติดกันกี่รอบ

// ---------- กฎมือซ้าย (เกาะกำแพงซ้ายตลอด) ----------
const bool rightHand = false;

// ---------- ค่าที่ต้อง calibrate (หน่วย mm) ----------
const int DIAG_WALL   = 60;              // ค่า ToF เฉียงเมื่ออยู่กลางช่องและมีกำแพง
const int DIAG_OPEN   = 110;             // มากกว่านี้ = ช่องข้างเปิด
const int SIDE_NEAR   = DIAG_WALL - 20;  // ใกล้กว่านี้ = ชิดกำแพงข้างมาก หักหนีทันที
const int FRONT_STOP  = 60;              // น้อยกว่านี้ = หน้าตัน
const int FRONT_MIN   = 35;              // ใกล้กว่านี้ระหว่างเดิน = หยุดกันชน
const int SLOW_DIST   = 150;             // เริ่มชะลอเมื่อกำแพงหน้าใกล้กว่านี้
const int OPEN_CONFIRM = 1;              // ต้องเห็นช่องเปิดติดกันกี่รอบ (ถ้าเลี้ยวมั่วที่ปลายกำแพงให้เพิ่มเป็น 2)
const float KP        = 1.0;             // ความแรงของการประคองกลางช่อง
const int MAX_CORR    = 50;

// ---------- ความเร็ว (PWM 0-255) ----------
const int BASE_SPEED = 100;    // วิ่งตรง
const int MIN_SPEED  = 90;     // ความเร็วตอนใกล้กำแพงหน้า
const int MIN_MOTOR  = 70;     // ล้อข้างที่ช้ากว่าห้ามต่ำกว่านี้ (กันมอเตอร์หยุด)
const int TURN_SPEED = 120;    // ความเร็วตอนหมุน
const int TRIM_L = 0;          // ชดเชยมอเตอร์สองข้างไม่เท่ากัน (บวก = เร็วขึ้น)
const int TRIM_R = 0;

// ---------- เวลา (ms) ต้องปรับตามหุ่นจริง ----------
const int ALIGN_R_MS    = 420; // เดินต่อก่อนเลี้ยวขวาที่ช่องเปิด
const int ALIGN_L_MS    = 420; // เดินต่อก่อนเลี้ยวซ้ายที่ช่องเปิด
const int ENTER_MS      = 250; // เดินเข้าช่องใหม่หลังเลี้ยว
const int KICK_MS       = 25;  // กระตุกออกตัวด้วย PWM เต็มตอนเริ่มหมุน
const int TURN90_R_MS   = 210; // เลี้ยวขวา 90 องศา (ปรับด้วย MODE 3)
const int TURN90_L_MS   = 210; // เลี้ยวซ้าย 90 องศา (ปรับแยกจากขวา)
const int PARK_MS       = 40;  // เดินต่อหลังเจอเทป ให้ตัวหุ่นเข้าโซนทั้งตัว (0 = หยุดทันที)
const int PARK_SPEED    = 50;  // ความเร็วตอนคืบเข้าโซน

// ---------- Motor (L298N) ----------
const int enL = 6, L1 = 4, L2 = 7;
const int R1 = 3, R2 = 2, enR = 5;

uint8_t openL = 0, openR = 0;

// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);          // I2C 400 kHz อ่านเซนเซอร์เร็วขึ้น

  pinMode(enL, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(enR, OUTPUT);
  brake();

  pinMode(XSHUT_L, OUTPUT);
  pinMode(XSHUT_F, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_L, LOW);
  digitalWrite(XSHUT_F, LOW);
  digitalWrite(XSHUT_R, LOW);
  delay(10);

  initToF(sensorL, XSHUT_L, 0x30);
  initToF(sensorF, XSHUT_F, 0x31);
  initToF(sensorR, XSHUT_R, 0x32);

  if (!apds.init()) Serial.println(F("APDS-9960 init failed!"));
  if (!apds.enableProximitySensor(false)) Serial.println(F("Proximity enable failed!"));
#if MODE == 1 || TAPE_USE_AMBIENT
  // เปิด ALS เฉพาะตอนต้องใช้ เพราะมันทำให้ prox อัปเดตช้าลงมาก
  if (!apds.enableLightSensor(false)) Serial.println(F("Light enable failed!"));
#endif
  // ต้องตั้งหลัง enable เพราะ enable จะรีเซ็ตค่าเหล่านี้เป็นค่าเริ่มต้น
  apds.setProximityGain(PGAIN_1X);
  apds.setLEDDrive(LED_DRIVE_12_5MA);

  // ปริ้นค่าพื้นตอนเริ่ม ไว้เช็คว่าสูงกว่า PROX_BLACK ชัดเจน
  delay(50);
  uint8_t p0 = 0;
  apds.readProximity(p0);
  Serial.print(F("floor prox = ")); Serial.println(p0);

  Serial.println(F("Left-hand rule"));
  delay(1000);
}

void initToF(VL53L0X &s, int xshut, uint8_t addr) {
  digitalWrite(xshut, HIGH);
  delay(10);
  s.setTimeout(500);
  if (!s.init()) Serial.println(F("ToF init failed"));
  s.setAddress(addr);
  s.setMeasurementTimingBudget(20000);
  s.startContinuous();
}

// อ่าน ToF โดยระหว่างรอข้อมูลพร้อม จะวนเช็คเทปดำตลอด (ไม่ค้างรอเฉย ๆ)
uint16_t readToF(VL53L0X &s) {
  unsigned long t0 = millis();
  while ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    checkTape();
    if (millis() - t0 > 500) return 8190;       // timeout -> ถือว่าไกลมาก
  }
  uint16_t v = s.readRangeContinuousMillimeters();
  if (s.timeoutOccurred() || v > 8000) v = 8190;
  return v;
}

// =====================================================
void loop() {
#if MODE == 1
  uint8_t p = 0;
  uint16_t a = 0;
  apds.readProximity(p);
  apds.readAmbientLight(a);

  bool pDark = (p < PROX_BLACK);
  bool aDark = (a < AMB_BLACK);
#if TAPE_USE_AMBIENT
  bool isBlack = aDark;       // วิธีที่หุ่นใช้ตัดสินจริง = ambient
#else
  bool isBlack = pDark;       // วิธีที่หุ่นใช้ตัดสินจริง = proximity
#endif

  Serial.print(F("prox: ")); Serial.print(p);
  Serial.print(pDark ? F(" [BLACK]") : F(" [white]"));
  Serial.print(F("   amb: ")); Serial.print(a);
  Serial.print(aDark ? F(" [BLACK]") : F(" [white]"));
  Serial.println(isBlack ? F("   >>> BLACK TAPE <<<") : F("   floor"));
  delay(100);
  return;
#elif MODE == 2
  Serial.print(F("L: ")); Serial.print(readToF(sensorL));
  Serial.print(F("  F: ")); Serial.print(readToF(sensorF));
  Serial.print(F("  R: ")); Serial.println(readToF(sensorR));
  delay(100);
  return;
#elif MODE == 3
  delay(3000);
  for (int i = 0; i < 4; i++) { turn(true, TURN90_R_MS); delay(500); }
  delay(2000);
  for (int i = 0; i < 4; i++) { turn(false, TURN90_L_MS); delay(500); }
  while (true);
#endif

  checkTape();
  uint16_t SL = readToF(sensorL);
  uint16_t SF = readToF(sensorF);
  uint16_t SR = readToF(sensorR);

  // นับรอบที่เห็นช่องเปิดติดกัน กันค่าหลอก
  openL = (SL > DIAG_OPEN) ? min(openL + 1, 10) : 0;
  openR = (SR > DIAG_OPEN) ? min(openR + 1, 10) : 0;

  bool leftOpen   = openL >= OPEN_CONFIRM;
  bool rightOpen  = openR >= OPEN_CONFIRM;
  bool frontBlock = SF < FRONT_STOP;

  // กฎมือซ้าย: ซ้ายโล่ง -> เลี้ยวซ้าย, หน้าโล่ง -> ตรง, ขวาโล่ง -> เลี้ยวขวา,
  // ตันทุกด้าน -> เลี้ยวขวา 90 องศา
  bool firstOpen  = rightHand ? rightOpen : leftOpen;   // ด้านที่ชิดกำแพง
  bool secondOpen = rightHand ? leftOpen  : rightOpen;

  if (firstOpen) {
    driveMs(rightHand ? ALIGN_R_MS : ALIGN_L_MS);
    turn(rightHand, t90(rightHand));
    driveMs(ENTER_MS);
    resetOpen();
  }
  else if (!frontBlock) {
    driveCentered(SL, SF, SR);
  }
  else if (secondOpen) {
    brake(); waitMs(100);
    turn(!rightHand, t90(!rightHand));
    driveMs(ENTER_MS);
    resetOpen();
  }
  else {                                  // กำแพงทุกด้าน -> เลี้ยวขวา 90 องศา
    brake(); waitMs(100);
    turn(true, t90(true));
    resetOpen();
  }
}

void resetOpen() { openL = 0; openR = 0; }

// รอแบบเช็คเทปดำตลอด (ใช้แทน delay ทุกช่วงที่หุ่นเคลื่อนที่หรือเพิ่งหยุด)
void waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    checkTape();
  }
}

// =====================================================
//  เดินตรงพร้อมประคองกลางช่อง + ชะลอเมื่อใกล้กำแพงหน้า
// =====================================================
void driveCentered(uint16_t SL, uint16_t SF, uint16_t SR) {
  bool lw = SL < DIAG_OPEN;
  bool rw = SR < DIAG_OPEN;
  float corr = 0;

  if (SR < SIDE_NEAR && SL >= SIDE_NEAR)      corr =  MAX_CORR;   // ชิดขวามาก -> หักซ้ายแรง
  else if (SL < SIDE_NEAR && SR >= SIDE_NEAR) corr = -MAX_CORR;   // ชิดซ้ายมาก -> หักขวาแรง
  else if (lw && rw) corr = (float)((int)SL - (int)SR) * KP / 2.0;
  else if (rw)       corr = (float)(DIAG_WALL - (int)SR) * KP;
  else if (lw)       corr = (float)((int)SL - DIAG_WALL) * KP;

  corr = constrain(corr, -MAX_CORR, MAX_CORR);

  // ชะลอเมื่อเข้าใกล้กำแพงหน้า
  int spd = BASE_SPEED;
  if (SF < SLOW_DIST) {
    spd = map(constrain(SF, FRONT_STOP, SLOW_DIST), FRONT_STOP, SLOW_DIST, MIN_SPEED, BASE_SPEED);
  }

  // corr > 0 -> เลี้ยวซ้าย (ล้อขวาเร็วขึ้น)
  int sl = constrain(spd - (int)corr, MIN_MOTOR, 255);
  int sr = constrain(spd + (int)corr, MIN_MOTOR, 255);
  setMotors(sl, sr);
}

// เดินหน้าตามเวลา โดยประคองกลางช่องและเช็คเทปดำ/กำแพงหน้าตลอด
void driveMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    uint16_t SL = readToF(sensorL);
    uint16_t SF = readToF(sensorF);
    uint16_t SR = readToF(sensorR);
    if (SF < FRONT_MIN) break;
    driveCentered(SL, SF, SR);
  }
  brake();
  waitMs(60);
}

// =====================================================
//  การเลี้ยว
// =====================================================
unsigned long t90(bool toRight) {
  return toRight ? TURN90_R_MS : TURN90_L_MS;
}

// หมุนอยู่กับที่ตามเวลา (กระตุกออกตัวก่อน แล้วลดเป็นความเร็วหมุน) เช็คเทปตลอด
void turn(bool toRight, unsigned long ms) {
  if (toRight) {
    digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
    digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);
  } else {
    digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);
    digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
  }
  analogWrite(enL, 255);
  analogWrite(enR, 255);
  waitMs(KICK_MS);
  analogWrite(enL, constrain(TURN_SPEED + TRIM_L, 0, 255));
  analogWrite(enR, constrain(TURN_SPEED + TRIM_R, 0, 255));
  waitMs(ms > KICK_MS ? ms - KICK_MS : 0);
  brake();
  waitMs(200);                   // รอให้หุ่นนิ่งก่อนเดินต่อ
}

// =====================================================
//  เทปดำ = ถึง Parking zone (ไม่มีกำแพงหน้า หยุดด้วยเทปและเวลาอย่างเดียว)
// =====================================================
void checkTape() {
  static uint8_t cnt = 0;
  bool dark = false;

#if TAPE_USE_AMBIENT
  uint16_t a = 0;
  if (!apds.readAmbientLight(a)) return;
  dark = (a < AMB_BLACK);
  #if TAPE_DEBUG
    if (a < AMB_BLACK * 3) Serial.println(a);
  #endif
#else
  uint8_t p = 0;
  if (!apds.readProximity(p)) return;
  dark = (p < PROX_BLACK);
  #if TAPE_DEBUG
    if (p < 40) Serial.println(p);
  #endif
#endif

  cnt = dark ? cnt + 1 : 0;
  if (cnt >= TAPE_CONFIRM) parkAndStop();
}

void parkAndStop() {
  if (PARK_MS > 0) {
    setMotors(PARK_SPEED, PARK_SPEED);
    delay(PARK_MS);
  }
  brake();
  Serial.println(F("PARKED"));
  while (true);                           // หยุดถาวร
}

// =====================================================
//  Motor
// =====================================================
void setMotors(int sl, int sr) {
  sl = constrain(sl + TRIM_L, 0, 255);
  sr = constrain(sr + TRIM_R, 0, 255);
  digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
  digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
  analogWrite(enL, sl);
  analogWrite(enR, sr);
}

// เบรกแบบ short (หยุดเร็วกว่าปล่อยไหล)
void brake() {
  digitalWrite(L1, HIGH); digitalWrite(L2, HIGH);
  digitalWrite(R1, HIGH); digitalWrite(R2, HIGH);
  analogWrite(enL, 255);
  analogWrite(enR, 255);
}