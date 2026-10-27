#define DEBUG 0

#if DEBUG
  #define DBG(x)    Serial.print(x)
  #define DBGLN(x)  Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

#include <Wire.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h> 
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <MPU6050.h>
#include <math.h>

// ─────────────────────────────────────────────
//  PINOUT
// ─────────────────────────────────────────────
constexpr uint8_t PIN_TRIG      = 11; 
constexpr uint8_t PIN_ECHO      = 12; 
constexpr uint8_t PIN_SERVO_LR  = 9;
constexpr uint8_t PIN_SERVO_UD  = 10;

constexpr uint8_t PIN_VOLTAGE   = A0; 
constexpr uint8_t PIN_LDR_L     = A3;
constexpr uint8_t PIN_LDR_R     = A2;
constexpr uint8_t PIN_LDR_U     = A1;

constexpr uint8_t GPS_RX        = 4;   // Arduino RX <- GPS TX
constexpr uint8_t GPS_TX        = 5;   // Arduino TX -> GPS RX

// ─────────────────────────────────────────────
//  MOTOR
// ─────────────────────────────────────────────
constexpr long     MOTOR_BAUD     = 9600;
constexpr uint8_t  MOTOR_L        = 1;
constexpr uint8_t  MOTOR_R        = 2;
constexpr uint8_t  CMD_FWD        = 0xF0;
constexpr uint8_t  CMD_BWD        = 0xBA;
constexpr uint8_t  BASE_SPEED     = 80;
constexpr int8_t   LEFT_TRIM      = 7;


// ─────────────────────────────────────────────
//  SERVO LIMITS
// ─────────────────────────────────────────────
constexpr int LR_MIN = 30,  LR_MAX = 150, LR_MID = (LR_MIN + LR_MAX) / 2;
constexpr int UD_MIN = 20,  UD_MAX = 80,  UD_MID = (UD_MIN + UD_MAX) / 2;
constexpr int SCAN_COARSE_H = 10;
constexpr int SCAN_COARSE_V = 10;

// ─────────────────────────────────────────────
//  NAVIGATIE / MISCARE
// ─────────────────────────────────────────────
constexpr float SPEED_CM_S      = 8.0f;   
constexpr float TURN_DEG_S      = 85.0f;  
constexpr float OBSTACLE_CM     = 25.0f;
constexpr unsigned long KICK_MS = 60UL;   

// ─────────────────────────────────────────────
//  GRID
// ─────────────────────────────────────────────
constexpr float GRID_CM   = 100.0f;
constexpr float STEP_CM   = 20.0f;
constexpr int   NX        = (int)(GRID_CM / STEP_CM) + 1;  
constexpr int   NY        = (int)(GRID_CM / STEP_CM) + 1;  
constexpr int   MAX_PTS   = NX * NY;                        

// ─────────────────────────────────────────────
//  STRUCTURI DATE
// ─────────────────────────────────────────────
struct GridPoint {
  float x_cm, y_cm;
  float score, lux, voltage;
  int   bestLR, bestUD;
  bool  valid;
};

// ─────────────────────────────────────────────
//  OBIECTE GLOBALE
// ─────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);
BH1750            lightMeter;
Servo             servoLR, servoUD;
MPU6050           mpu;
SoftwareSerial    gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus       gps;

GridPoint gridPts[MAX_PTS];
int       totalPts = 0;

float pose_x       = 0.0f;
float pose_y       = 0.0f;
float pose_heading = 0.0f;   

float gyroZ_offset = 0.0f;
unsigned long lastGyroUs = 0;

// ─────────────────────────────────────────────
//  FUNCTII
// ─────────────────────────────────────────────
void gyroInit() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    DBGLN(F("MPU6050 nu raspunde!"));
  }
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(5);
  }
  gyroZ_offset = sum / 200.0f;
  lastGyroUs = micros();
  DBGLN(F("Gyro calibrat."));
}

void gyroUpdate() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long now = micros();
  float dt = (now - lastGyroUs) / 1e6f;
  lastGyroUs = now;
  float degPerSec = (gz - gyroZ_offset) / 131.0f;
  pose_heading += degPerSec * dt;
  while (pose_heading >= 360.0f) pose_heading -= 360.0f;
  while (pose_heading <    0.0f) pose_heading += 360.0f;
}

void gpsFeed(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
  }
}

void motorFrame(uint8_t id, uint8_t cmd, uint8_t spd) {
  Serial.write(0xFE);
  Serial.write(id);
  Serial.write(cmd);
  Serial.write(spd);
  delayMicroseconds(400);
}

void motorFwd(uint8_t id, uint8_t spd) { motorFrame(id, CMD_FWD, spd); }
void motorBwd(uint8_t id, uint8_t spd) { motorFrame(id, CMD_BWD, spd); }
void motorStop(uint8_t id)             { motorFrame(id, CMD_FWD, 0);   }

void stopAll() {
  motorStop(MOTOR_L);
  motorStop(MOTOR_R);
  delay(80);
}

void kickStart(bool leftFwd, bool rightFwd) {
  if (leftFwd)  motorFwd(MOTOR_L, 220); else motorBwd(MOTOR_L, 220);
  if (rightFwd) motorFwd(MOTOR_R, 220); else motorBwd(MOTOR_R, 220);
  delay(KICK_MS);
}

void ultrasonicInit() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}

float readUsCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long dur = pulseIn(PIN_ECHO, HIGH, 25000UL);
  return (dur == 0) ? 999.0f : dur * 0.017f;
}

float readLux(uint8_t n) {
  float s = 0;
  for (uint8_t i = 0; i < n; i++) {
    float v = lightMeter.readLightLevel();
    s += (v < 0) ? 0 : v;
    delay(35);
  }
  return s / n;
}

float readVoltage(uint8_t n) {
  float s = 0;
  for (uint8_t i = 0; i < n; i++) {
    s += analogRead(PIN_VOLTAGE) * (5.0f / 1023.0f);
    delay(5);
  }
  return s / n;
}

inline float calcScore(float lux, float volt) {
  return lux * volt;
}

void servoMove(int lr, int ud) {
  servoLR.write(constrain(lr, LR_MIN, LR_MAX));
  servoUD.write(constrain(ud, UD_MIN, UD_MAX));
  delay(100);
}

void lcdShow(const __FlashStringHelper *l1, const __FlashStringHelper *l2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

void lcdShow(const char *l1, const char *l2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

float driveStraightCm(float distCm) {
  if (fabs(distCm) < 1.0f) return 0.0f;

  bool fwd = distCm > 0;
  float dist = fabs(distCm);
  unsigned long travelMs = (unsigned long)((dist / SPEED_CM_S) * 1000.0f);
  if (travelMs < 100) travelMs = 100;

  uint8_t lSpd = constrain(BASE_SPEED + LEFT_TRIM, 0, 255);
  uint8_t rSpd = constrain(BASE_SPEED,             0, 255);

  kickStart(fwd, fwd);
  if (fwd) { motorFwd(MOTOR_L, lSpd); motorFwd(MOTOR_R, rSpd); }
  else     { motorBwd(MOTOR_L, lSpd); motorBwd(MOTOR_R, rSpd); }

  float targetHeading = pose_heading;
  unsigned long startMs = millis();
  float elapsed = 0.0f;
  bool  blocked = false;

  while (true) {
    gyroUpdate();
    unsigned long now = millis();
    elapsed = (float)(now - startMs);
    if (elapsed >= travelMs) break;

    float err = pose_heading - targetHeading;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    int8_t corr = constrain((int)(err * 2.0f), -20, 20);
    uint8_t lC = constrain(lSpd + corr, 0, 255);
    uint8_t rC = constrain(rSpd - corr, 0, 255);
    if (fwd) { motorFwd(MOTOR_L, lC); motorFwd(MOTOR_R, rC); }
    else     { motorBwd(MOTOR_L, lC); motorBwd(MOTOR_R, rC); }

    // Verificam obstacole DOAR daca mergem in fata (fwd == true)
  if (fwd && readUsCm() < OBSTACLE_CM) {
    blocked = true;
    break;
  }
    delay(20);
  }

  stopAll();

  float actualDist = (elapsed / 1000.0f) * SPEED_CM_S;
  if (!fwd) actualDist = -actualDist;

  float rad = pose_heading * (float)M_PI / 180.0f;
  pose_x += actualDist * sinf(rad);
  pose_y += actualDist * cosf(rad);

  if (blocked) {
    driveStraightCm(-8.0f);
    return actualDist;  
  }
  return actualDist;
}

void rotateDeg(float delta) {
  if (fabs(delta) < 1.5f) return;

  bool right = delta > 0;
  float targetH = pose_heading + delta;
  while (targetH >= 360.0f) targetH -= 360.0f;
  while (targetH <    0.0f) targetH += 360.0f;

  kickStart(right, !right);
  if (right) {
    motorFwd(MOTOR_L, BASE_SPEED);
    motorBwd(MOTOR_R, BASE_SPEED);
  } else {
    motorBwd(MOTOR_L, BASE_SPEED);
    motorFwd(MOTOR_R, BASE_SPEED);
  }

  unsigned long safetyMs = (unsigned long)((fabs(delta) / TURN_DEG_S) * 1000.0f) + 500UL;
  unsigned long startMs  = millis();
  while (millis() - startMs < safetyMs) {
    gyroUpdate();
    float err = targetH - pose_heading;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    if (fabs(err) < 2.0f) break;
    delay(5);
  }

  stopAll();
  pose_heading = targetH;
}

bool navigateTo(float goalX, float goalY) {
  float dx   = goalX - pose_x;
  float dy   = goalY - pose_y;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < 3.0f) return true;

  float bearing = atan2f(dx, dy) * 180.0f / (float)M_PI;
  float delta   = bearing - pose_heading;
  while (delta >  180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;

  rotateDeg(delta);

  float traveled = driveStraightCm(dist);
  return fabs(traveled) >= dist * 0.8f;
}

float panelScan(float &luxOut, float &voltOut, int &lrOut, int &udOut) {
  int   bLR = LR_MID, bUD = UD_MID;
  float bLux = 0, bVolt = 0, bScore = -1.0f;

  servoMove(LR_MID, UD_MID);
  delay(120);

  // Pasul 1
  for (int lr = LR_MIN; lr <= LR_MAX; lr += SCAN_COARSE_H) {
    servoLR.write(lr);
    delay(50);
    for (int ud = UD_MIN; ud <= UD_MAX; ud += SCAN_COARSE_V) {
      servoUD.write(ud);
      delay(35);
      float lux  = readLux(3);
      float volt = readVoltage(3);
      float sc   = calcScore(lux, volt);
      if (sc > bScore) {
        bScore = sc; bLR = lr; bUD = ud; bLux = lux; bVolt = volt;
      }
    }
  }

  // Pasul 2
  int fH = SCAN_COARSE_H / 2;  if (fH < 1) fH = 1;
  int fV = SCAN_COARSE_V / 2;  if (fV < 1) fV = 1;
  int h0 = constrain(bLR - 2 * SCAN_COARSE_H, LR_MIN, LR_MAX);
  int h1 = constrain(bLR + 2 * SCAN_COARSE_H, LR_MIN, LR_MAX);
  int v0 = constrain(bUD - 2 * SCAN_COARSE_V, UD_MIN, UD_MAX);
  int v1 = constrain(bUD + 2 * SCAN_COARSE_V, UD_MIN, UD_MAX);

  for (int lr = h0; lr <= h1; lr += fH) {
    servoLR.write(lr);
    delay(40);
    for (int ud = v0; ud <= v1; ud += fV) {
      servoUD.write(ud);
      delay(30);
      float lux  = readLux(4);
      float volt = readVoltage(4);
      float sc   = calcScore(lux, volt);
      if (sc > bScore) {
        bScore = sc; bLR = lr; bUD = ud; bLux = lux; bVolt = volt;
      }
    }
  }

  // Pasul 3
  bool improved = true;
  int  tries    = 0;
  while (improved && tries < 10) {
    improved = false;
    tries++;
    for (int dh = -1; dh <= 1; dh++) {
      for (int dv = -1; dv <= 1; dv++) {
        if (dh == 0 && dv == 0) continue;
        int lr = constrain(bLR + dh, LR_MIN, LR_MAX);
        int ud = constrain(bUD + dv, UD_MIN, UD_MAX);
        servoLR.write(lr); delay(25);
        servoUD.write(ud); delay(20);
        float lux  = readLux(5);
        float volt = readVoltage(5);
        float sc   = calcScore(lux, volt);
        if (sc > bScore + 1e-5f) {
          bScore = sc; bLR = lr; bUD = ud; bLux = lux; bVolt = volt;
          improved = true;
        }
      }
    }
  }

  servoMove(bLR, bUD);
  luxOut  = bLux;
  voltOut = bVolt;
  lrOut   = bLR;
  udOut   = bUD;

  DBG(F("PanelScan score=")); DBGLN(bScore);
  return bScore;
}

void buildGrid() {
  DBGLN(F("=== Start grid scan ==="));
  lcdShow(F("Grid 100x100 cm"), F("Se initiaza..."));

  totalPts = 0;

  for (int iy = 0; iy < NY; iy++) {
    bool l2r = (iy % 2 == 0);

    for (int ix = 0; ix < NX; ix++) {
      int aix   = l2r ? ix : (NX - 1 - ix);
      float gx  = aix * STEP_CM;
      float gy  = iy  * STEP_CM;

      char ln1[17];
      snprintf(ln1, 17, "Pt %2d/%2d", totalPts + 1, MAX_PTS);
      lcd.clear();
      lcd.setCursor(0,0); lcd.print(ln1);
      lcd.setCursor(0,1); 
      lcd.print(F("X:")); lcd.print((int)gx);
      lcd.print(F(" Y:")); lcd.print((int)gy);

      bool ok = navigateTo(gx, gy);
      if (!ok) {
        DBG(F("Blocat la X=")); DBG(gx); DBG(F(" Y=")); DBGLN(gy);
        gridPts[totalPts].x_cm  = gx;
        gridPts[totalPts].y_cm  = gy;
        gridPts[totalPts].score = -1.0f;
        gridPts[totalPts].valid = false;
        totalPts++;
        continue;
      }

      float lux, volt;
      int   lr, ud;
      delay(500);
      float sc = panelScan(lux, volt, lr, ud);

      gridPts[totalPts] = { gx, gy, sc, lux, volt, lr, ud, true };
      totalPts++;

      if (gps.location.isValid()) {
        #if DEBUG
          Serial.print(F("GPS lat=")); Serial.print(gps.location.lat(), 6);
          Serial.print(F(" lng="));   Serial.println(gps.location.lng(), 6);
        #endif
      }
      gpsFeed(10);
    }
  }
}

void findTopTwo(int &best, int &second) {
  best = -1; second = -1;
  for (int i = 0; i < totalPts; i++) {
    if (!gridPts[i].valid) continue;
    if (best < 0 || gridPts[i].score > gridPts[best].score) {
      second = best;
      best   = i;
    } else if (second < 0 || gridPts[i].score > gridPts[second].score) {
      second = i;
    }
  }
}

void setup() {
  Serial.begin(MOTOR_BAUD);
  stopAll(); 
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcdShow(F("SolarRobot v2"), F("Initializare..."));

  lightMeter.begin();
  gyroInit();
  gpsSerial.begin(9600);
  ultrasonicInit();

  servoLR.attach(PIN_SERVO_LR);
  servoUD.attach(PIN_SERVO_UD);
  servoMove(LR_MID, UD_MID);

  pose_x = 0.0f;
  pose_y = 0.0f;
  pose_heading = 0.0f;

  lcdShow(F("Gata! Start in"), F("3 secunde..."));
  delay(3000);
}

void loop() {
  buildGrid();

  int best, second;
  findTopTwo(best, second);

  if (best >= 0) {
    GridPoint &pt = gridPts[best];
    
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(F("Best sc=")); lcd.print(pt.score, 1);
    lcd.setCursor(0, 1); lcd.print(F("X:")); lcd.print((int)pt.x_cm); lcd.print(F(" Y:")); lcd.print((int)pt.y_cm);

    DBG(F("BEST idx=")); DBG(best);
    DBG(F(" score="));   DBG(pt.score);
    DBG(F(" X="));       DBG(pt.x_cm);
    DBG(F(" Y="));       DBGLN(pt.y_cm);

    navigateTo(pt.x_cm, pt.y_cm);
    servoMove(pt.bestLR, pt.bestUD);
    delay(5000);
  }

  if (second >= 0) {
    GridPoint &pt = gridPts[second];
    
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(F("2nd sc=")); lcd.print(pt.score, 1);
    lcd.setCursor(0, 1); lcd.print(F("X:")); lcd.print((int)pt.x_cm); lcd.print(F(" Y:")); lcd.print((int)pt.y_cm);

    DBG(F("SECOND idx=")); DBG(second);
    DBG(F(" score="));     DBGLN(pt.score);

    navigateTo(pt.x_cm, pt.y_cm);
    servoMove(pt.bestLR, pt.bestUD);
    delay(5000);
  }

  lcdShow(F("Demo finalizat!"), F("Reset pt restart"));
  DBGLN(F("Demo complet."));

  while (true) {
    gpsFeed(1000);  
  }
}
