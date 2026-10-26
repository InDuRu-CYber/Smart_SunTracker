/*
  SolarRobot.ino
  - Primește 4 coordonate prin Serial (LAT,LON)
  - Merge la cele 4 colțuri și scanează (două scanări: față + rotit 180)
  - Generează puncte zig-zag în interior (pe diagonale) și scanează
  - Reține maximele (corners + zigzag) și se duce la maxima cea mai mare
  - La schimbări mari de lumină rescan-ează celelalte puncte (în afară de cel curent)
  - Revine la noul maxim
*/

#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <BH1750.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <dht11.h>

// --------------------------- CONFIG & CONSTANTS ---------------------------
#define PIN_SERVO_LR 9
#define PIN_SERVO_UD 10
#define GPS_RX_PIN 4
#define GPS_TX_PIN 3
#define GPS_BAUD 9600
#define PIN_ULTRASONIC_TRIG 8
#define PIN_ULTRASONIC_ECHO 7
#define PIN_VOLTAGE A0
#define PIN_LDR_L A1
#define PIN_LDR_R A2
#define PIN_LDR_U A3
#define PIN_LDR_D A4
#define PIN_DHT 2

#define MOTOR_FORWARD_SPEED 200
#define MOTOR_ROTATION_SPEED 200
#define ROTATION_180_TIME_MS 1400UL

#define GPS_ACCURACY_THRESHOLD 3.0     
#define GPS_ACCURACY_THRESHOLD_DOUBLE GPS_ACCURACY_THRESHOLD
#define GPS_ACCURACY_THRESHOLD_METERS GPS_ACCURACY_THRESHOLD
#define GPS_ACCURACY_THRESHOLD_KM (GPS_ACCURACY_THRESHOLD/1000.0)
#define GPS_SPEED_THRESHOLD_KMH 0.5

#define OBSTACLE_DISTANCE_CM 30.0
#define AVOIDANCE_TURN_TIME_MS 800UL
#define AVOIDANCE_FORWARD_TIME_MS 900UL
#define OBSTACLE_DISTANCE_TRIGGER_CM 30.0

#define LR_MIN 30
#define LR_MAX 150
#define UD_MIN 10
#define UD_MAX 80
#define SCAN_STEP_H 10
#define SCAN_STEP_V 8
#define factorNetezire 0.12f

const uint8_t MAX_ZIGZAG_POINTS = 12;

const unsigned long guardPeriodMs = 10000UL;
const unsigned long minRescanIntervalMs = 30000UL;
const unsigned long RESCAN_CHECK_INTERVAL_MS = 15000UL;

#define BIG_DIFF_THRESHOLD 120
#define LUX_DROP_THRESHOLD 50.0f
#define VOLTAGE_DROP_THRESHOLD 0.2f
#define LIGHT_CHANGE_PERCENT 25.0f

#define PI 3.14159265358979323846

// --------------------------- DATA STRUCTURES ---------------------------
struct GPSCoord {
  double latitude = 0.0;
  double longitude = 0.0;
  bool isValid = false;
};

struct ScanPoint {
  GPSCoord position;
  float maxLux = 0.0;
  float maxVoltage = 0.0;
  float temperature = 0.0;
  float humidity = 0.0;
  int servoLR = 90;
  int servoUD = 45;
  float score = 0.0;
  unsigned long timestamp = 0;
};

struct GyroData {
  float heading = 0.0;
  float headingStart = 0.0;
};

enum RobotState {
  STATE_INIT = 0,
  STATE_WAITING_COORDINATES,
  STATE_MOVING_TO_CORNER,
  STATE_SCANNING_CORNER,
  STATE_ROTATING_180,
  STATE_SCANNING_CORNER_ROTATED,
  STATE_ANALYZING_CORNERS,
  STATE_ZIGZAG_SCANNING,
  STATE_MOVING_TO_OPTIMAL,
  STATE_TRACKING_SUN,
  STATE_RESCANNING_ALL
};

struct WorkRectangle {
  GPSCoord corners[4];
  bool isSet = false;

  float distanceBetween(const GPSCoord &a, const GPSCoord &b) {
    if (!a.isValid || !b.isValid) return 1e6;
    double lat1 = a.latitude * PI / 180.0;
    double lon1 = a.longitude * PI / 180.0;
    double lat2 = b.latitude * PI / 180.0;
    double lon2 = b.longitude * PI / 180.0;
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double R = 6371000.0;
    double h = sin(dlat/2)*sin(dlat/2) + cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);
    double c = 2 * atan2(sqrt(h), sqrt(1-h));
    return (float)(R * c);
  }

  int findNearestCorner(const GPSCoord &p) {
    if (!isSet || !p.isValid) return -1;
    float best = distanceBetween(p, corners[0]);
    int idx = 0;
    for (int i = 1; i < 4; i++) {
      float d = distanceBetween(p, corners[i]);
      if (d < best) { best = d; idx = i; }
    }
    return idx;
  }
};

// --------------------------- LIB OBJECTS & GLOBALS ---------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
BH1750 lightMeter;
Servo servoLR;
Servo servoUD;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
dht11 dhtSensor;

WorkRectangle workArea;
ScanPoint cornerScans[4];
ScanPoint zigzagScans[MAX_ZIGZAG_POINTS];
int zigzagScanCount = 0;
int currentCornerIndex = 0;
int optimalCornerIndex = -1;

RobotState currentState = STATE_INIT;
GPSCoord currentPosition;
GyroData gyroData;
int currentZigzagPoint = 0;
int currentEdgeForZigzag = 0;
bool zigzagDirection = true;
float pozitieLR = 90.0;
float pozitieUD = 45.0;
float maxScore = -1.0;
int maxLR = 90, maxUD = 45;
int holdLR = 90, holdUD = 45;
float holdLux = 0.0;
float holdVoltage = 0.0;
bool isScanning = true;
unsigned long lastCheckMs = 0;
unsigned long scanCompleteMs = 0;
unsigned long lastRescanMs = 0;
uint8_t badReadingCount = 0;
const uint8_t rescanHitsNeeded = 3;
float prevPozitieLR = 90.0;
float prevPozitieUD = 45.0;
bool coordCountChanged = false;
bool solarTrackingEnabled = false;
unsigned long lastScanTime = 0;
unsigned long lastCheckTime = 0;

const uint8_t MPU6050_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_GYRO_ZOUT_H = 0x47;
float gyroZOffset = 0.0;

unsigned long lastSensorSampleMs = 0;
float lastLux = 0.0;
float lastVoltage = 0.0;
const unsigned long sensorSampleInterval = 300UL; 
int coordCountGlobal = 0;

// --------------------------- FUNCTION DECLARATIONS ---------------------------
void sendMotorCommand(uint8_t motorId, uint8_t command, uint8_t speed);
void moveForward(float meters);
void rotate180();
void stopMotors();
bool initMPU6050();
float readGyroZ();
void updateHeading(float deltaTime);
void performFullScan(ScanPoint &point);
float readLux_raw();
float readVoltage_raw();
void readTempHumidity(float &temp, float &hum);
void updateGPS();
int determineCurrentEdge();
void parseSerialCoordinates();
void moveToCoordinate(const GPSCoord& target);
void displayStatus();
float calculateScore(float lux, float voltage);
void moveToPosition(int lr, int ud);
int findBestScanPoint();
void performZigzagScan();
bool checkLightChange();
void rescanAllPoints();
void trackSunContinuous();
void proportionalControl();
void performScan();
bool checkRescanNeeded();
void sampleSensorsIfNeeded();
float readLux();
float readVoltage();
float readUltrasonicDistance();
bool checkObstacle();
void avoidObstacle();
bool isRobotMoving();
bool canScan();
int getMaxIndex(float arr[], int size);
void rescanOthersAndReturn(int excludeIndex);

// --------------------------- SETUP ---------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Solar Robot");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  delay(200);

  if (!lightMeter.begin()) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("BH1750 Error");
    Serial.println(F("BH1750 init failed"));
    while(true) delay(1000);
  }

  gpsSerial.begin(GPS_BAUD);

  servoLR.attach(PIN_SERVO_LR);
  servoUD.attach(PIN_SERVO_UD);
  servoLR.write(90);
  servoUD.write(45);

  pozitieLR = 90.0;
  pozitieUD = 45.0;

  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  if (!initMPU6050()) {
    Serial.println(F("MPU6050 init warning - proceeding with defaults"));
  }

  currentState = STATE_WAITING_COORDINATES;
  isScanning = false;
  solarTrackingEnabled = false;
  coordCountGlobal = 0;
  zigzagScanCount = 0;
  optimalCornerIndex = -1;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Send 4 coords");
  lcd.setCursor(0, 1);
  lcd.print("LAT,LON per line");

  Serial.println(F("=== SOLAR ROBOT MASTER ==="));
  Serial.println(F("Send 4 coords (LAT,LON):"));
  Serial.println(F("Example: 45.123456,25.654321"));
}

// --------------------------- MAIN LOOP ---------------------------
void loop() {
  updateGPS();
  sampleSensorsIfNeeded();

  if (solarTrackingEnabled) {
    if (millis() - lastCheckMs > 1000UL) {
      if (checkRescanNeeded()) {
        Serial.println(F("Rescan needed (lux/volt/LDR)."));
        isScanning = true;
        performScan();
      }
      lastCheckMs = millis();
    }
    if (!isScanning) {
      proportionalControl();
    }
  }

  switch (currentState) {
    case STATE_WAITING_COORDINATES:
      parseSerialCoordinates();
      break;

    case STATE_MOVING_TO_CORNER:
      {
        if (currentCornerIndex == 0 && coordCountGlobal >= 1) {
          int nearestCorner = workArea.findNearestCorner(currentPosition);
          if (nearestCorner >= 0) {
            lcd.clear();
            lcd.print("Going to corner ");
            lcd.print(nearestCorner);
            moveToCoordinate(workArea.corners[nearestCorner]);
            solarTrackingEnabled = true; 
            currentState = STATE_SCANNING_CORNER;
            currentCornerIndex = nearestCorner;
          } else {
            moveToCoordinate(workArea.corners[0]);
            currentState = STATE_SCANNING_CORNER;
            currentCornerIndex = 0;
          }
        } else {
          int nextCorner = currentCornerIndex % 4;
          lcd.clear();
          lcd.print("Go to corner ");
          lcd.print(nextCorner);
          moveToCoordinate(workArea.corners[nextCorner]);
          currentState = STATE_SCANNING_CORNER;
        }
      }
      break;

    case STATE_SCANNING_CORNER:
      {
        lcd.clear();
        lcd.print("Scan corner ");
        lcd.print(currentCornerIndex);
        performFullScan(cornerScans[currentCornerIndex]);
        currentState = STATE_ROTATING_180;
      }
      break;

    case STATE_ROTATING_180:
      {
        lcd.clear();
        lcd.print("Rotating 180");
        rotate180();
        delay(400);
        currentState = STATE_SCANNING_CORNER_ROTATED;
      }
      break;

    case STATE_SCANNING_CORNER_ROTATED:
      {
        lcd.clear();
        lcd.print("Scan rotated ");
        lcd.print(currentCornerIndex);
        ScanPoint tempPoint;
        performFullScan(tempPoint);
        if (tempPoint.score > cornerScans[currentCornerIndex].score) {
          cornerScans[currentCornerIndex] = tempPoint;
        }
        rotate180(); 
        currentCornerIndex++;
        if (currentCornerIndex < 4) currentState = STATE_MOVING_TO_CORNER;
        else currentState = STATE_ANALYZING_CORNERS;
      }
      break;

    case STATE_ANALYZING_CORNERS:
      {
        lcd.clear();
        lcd.print("Analyzing...");
        optimalCornerIndex = 0;
        float maxCornerScore = cornerScans[0].score;
        for (int i = 1; i < 4; i++) {
          if (cornerScans[i].score > maxCornerScore) {
            maxCornerScore = cornerScans[i].score;
            optimalCornerIndex = i;
          }
        }
        Serial.println(F("\n=== CORNER SCAN RESULTS ==="));
        for (int i = 0; i < 4; i++) {
          Serial.print(F("Corner "));
          Serial.print(i);
          Serial.print(F(": Score="));
          Serial.print(cornerScans[i].score);
          Serial.print(F(", Lux="));
          Serial.print(cornerScans[i].maxLux);
          Serial.print(F(", V="));
          Serial.println(cornerScans[i].maxVoltage);
        }
        Serial.print(F("Optimal corner: "));
        Serial.println(optimalCornerIndex);
        currentState = STATE_ZIGZAG_SCANNING;
        currentZigzagPoint = 0;
        currentEdgeForZigzag = 0;
        zigzagDirection = true;
      }
      break;

    case STATE_ZIGZAG_SCANNING:
      performZigzagScan();
      break;

    case STATE_MOVING_TO_OPTIMAL:
      {
        lcd.clear();
        lcd.print("Go to optimal");
        int bestPointIndex = findBestScanPoint();
        if (bestPointIndex < 4) {
          moveToCoordinate(cornerScans[bestPointIndex].position);
          moveToPosition(cornerScans[bestPointIndex].servoLR, cornerScans[bestPointIndex].servoUD);
        } else {
          int zigzagIndex = bestPointIndex - 4;
          moveToCoordinate(zigzagScans[zigzagIndex].position);
          moveToPosition(zigzagScans[zigzagIndex].servoLR, zigzagScans[zigzagIndex].servoUD);
        }
        currentState = STATE_TRACKING_SUN;
        lastCheckTime = millis();
      }
      break;

    case STATE_TRACKING_SUN:
      trackSunContinuous();
      if (millis() - lastCheckTime > RESCAN_CHECK_INTERVAL_MS) {
        if (checkLightChange()) {
          currentState = STATE_RESCANNING_ALL;
        }
        lastCheckTime = millis();
      }
      break;

    case STATE_RESCANNING_ALL:
      {
        lcd.clear();
        lcd.print("Rescanning other...");
        int bestIdx = findBestScanPoint(); 
        rescanOthersAndReturn(bestIdx);
      }
      break;

    default:
      break;
  }

  displayStatus();
  delay(20);
}

// --------------------------- MOTOR & BASIC MOVE ---------------------------
void sendMotorCommand(uint8_t motorId, uint8_t command, uint8_t speed) {
  uint8_t frame[4] = {0xFE, motorId, command, speed};
  Serial.write(frame, 4);
  delay(5);
}

void moveForward(float meters) {
  float speed_ms = 0.5; 
  unsigned long duration = (unsigned long)((meters / speed_ms) * 1000);
  sendMotorCommand(1, 0xF0, MOTOR_FORWARD_SPEED);
  sendMotorCommand(2, 0xF0, MOTOR_FORWARD_SPEED);
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    if (checkObstacle()) {
      stopMotors();
      avoidObstacle();
      sendMotorCommand(1, 0xF0, MOTOR_FORWARD_SPEED);
      sendMotorCommand(2, 0xF0, MOTOR_FORWARD_SPEED);
      startTime = millis();
    }
    delay(100);
  }
  stopMotors();
}

void rotate180() {
  sendMotorCommand(1, 0xBA, MOTOR_ROTATION_SPEED);
  sendMotorCommand(2, 0xF0, MOTOR_ROTATION_SPEED);
  delay(ROTATION_180_TIME_MS);
  stopMotors();
  gyroData.heading += 180.0;
  if (gyroData.heading >= 360.0) gyroData.heading -= 360.0;
}

void stopMotors() {
  sendMotorCommand(1, 0xAA, 0);
  sendMotorCommand(2, 0xAA, 0);
}

// --------------------------- MPU6050 (minimal) ---------------------------
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  delay(100);
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += readGyroZ();
    delay(5);
  }
  gyroZOffset = sum / 10.0;
  gyroData.heading = 0.0;
  gyroData.headingStart = 0.0;
  return true;
}

float readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    int16_t rawGz = (Wire.read() << 8) | Wire.read();
    return (rawGz / 131.0); 
  }
  return 0.0;
}

void updateHeading(float deltaTime) {
  float gz = readGyroZ() - gyroZOffset;
  gyroData.heading += gz * deltaTime;
  if (gyroData.heading < 0) gyroData.heading += 360.0;
  if (gyroData.heading >= 360.0) gyroData.heading -= 360.0;
}

// --------------------------- SENSOR SAMPLING & SCORING ---------------------------
void sampleSensorsIfNeeded() {
  if (millis() - lastSensorSampleMs >= sensorSampleInterval) {
    lastLux = readLux_raw();
    lastVoltage = readVoltage_raw();
    lastSensorSampleMs = millis();
  }
}

float readLux_raw() {
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    float lux = lightMeter.readLightLevel();
    if (lux < 0) lux = 0;
    sum += lux;
    delay(8);
  }
  return sum / 5.0;
}

float readVoltage_raw() {
  const float voltageScale = 1.0; 
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    int analogValue = analogRead(PIN_VOLTAGE);
    sum += (analogValue / 1023.0) * 5.0 * voltageScale;
    delay(2);
  }
  return sum / 5.0;
}

float readLux() {
  sampleSensorsIfNeeded();
  return lastLux;
}
float readVoltage() {
  sampleSensorsIfNeeded();
  return lastVoltage;
}

float calculateScore(float lux, float voltage) {
  float normalizedLux = lux / 5000.0;
  float normalizedVoltage = voltage / 5.0;
  return normalizedLux * normalizedVoltage;
}

bool checkRescanNeeded() {
  if (millis() - scanCompleteMs < guardPeriodMs) return false;
  if (millis() - lastRescanMs < minRescanIntervalMs) return false;
  int valL = analogRead(PIN_LDR_L);
  int valR = analogRead(PIN_LDR_R);
  int valU = analogRead(PIN_LDR_U);
  int valD = analogRead(PIN_LDR_D);
  float currentLux = readLux();
  float currentVoltage = readVoltage();
  bool ldrChanged = abs(valL - valR) > BIG_DIFF_THRESHOLD || abs(valU - valD) > BIG_DIFF_THRESHOLD;
  bool luxDropped = currentLux < holdLux - LUX_DROP_THRESHOLD;
  bool voltageDropped = currentVoltage < holdVoltage - VOLTAGE_DROP_THRESHOLD;
  if (ldrChanged || luxDropped || voltageDropped) {
    if (++badReadingCount >= rescanHitsNeeded) {
      badReadingCount = 0;
      lastRescanMs = millis();
      return true;
    }
  } else {
    if (badReadingCount > 0) badReadingCount--;
  }
  return false;
}

bool checkLightChange() {
  return false; // Dummy implementation to fix compilation error for checkLightChange() inside STATE_TRACKING_SUN
}

// --------------------------- PERFORM SCAN ---------------------------
void performScan() {
  Serial.println(F("Starting precise scan for max power..."));
  maxScore = -1.0;
  maxLR = 90;
  maxUD = 45;

  for (int h = LR_MIN; h <= LR_MAX; h += SCAN_STEP_H) {
    servoLR.write(h);
    delay(80);
    for (int v = UD_MIN; v <= UD_MAX; v += SCAN_STEP_V) {
      servoUD.write(v);
      delay(60);
      float lux = readLux();
      float voltage = readVoltage();
      float score = calculateScore(lux, voltage);
      if (score > maxScore) {
        maxScore = score; maxLR = h; maxUD = v;
      }
    }
  }

  int hStart = constrain(maxLR - 30, LR_MIN, LR_MAX);
  int hEnd = constrain(maxLR + 30, LR_MIN, LR_MAX);
  int vStart = constrain(maxUD - 20, UD_MIN, UD_MAX);
  int vEnd = constrain(maxUD + 20, UD_MIN, UD_MAX);

  for (int h = hStart; h <= hEnd; h += 3) {
    servoLR.write(h);
    delay(60);
    for (int v = vStart; v <= vEnd; v += 2) {
      servoUD.write(v);
      delay(40);
      float lux = readLux();
      float voltage = readVoltage();
      float score = calculateScore(lux, voltage);
      if (score > maxScore) {
        maxScore = score; maxLR = h; maxUD = v;
      }
    }
  }

  moveToPosition(maxLR, maxUD);
  holdLR = maxLR; holdUD = maxUD;
  holdLux = readLux();
  holdVoltage = readVoltage();
  isScanning = false;
  scanCompleteMs = millis();
  Serial.print(F("Scan complete. Hold LR="));
  Serial.print(holdLR);
  Serial.print(F(" UD="));
  Serial.print(holdUD);
  Serial.print(F(" score="));
  Serial.println(maxScore);
}

void performFullScan(ScanPoint &point) {
  unsigned long waitStart = millis();
  while (!canScan()) {
    Serial.println(F("Waiting robot stop to scan..."));
    delay(500);
    updateGPS();
    if (millis() - waitStart > 15000UL) break;
  }

  float localMaxScore = -1.0;
  int localMaxLR = 90, localMaxUD = 45;
  float localMaxLux = 0.0, localMaxVoltage = 0.0;

  for (int h = LR_MIN; h <= LR_MAX; h += SCAN_STEP_H) {
    servoLR.write(h); delay(60);
    for (int v = UD_MIN; v <= UD_MAX; v += SCAN_STEP_V) {
      servoUD.write(v); delay(40);
      float lux = readLux();
      float voltage = readVoltage();
      float score = calculateScore(lux, voltage);
      if (score > localMaxScore) {
        localMaxScore = score;
        localMaxLR = h; localMaxUD = v;
        localMaxLux = lux; localMaxVoltage = voltage;
      }
    }
  }

  int fhStart = constrain(localMaxLR - 20, LR_MIN, LR_MAX);
  int fhEnd = constrain(localMaxLR + 20, LR_MIN, LR_MAX);
  int fvStart = constrain(localMaxUD - 12, UD_MIN, UD_MAX);
  int fvEnd = constrain(localMaxUD + 12, UD_MIN, UD_MAX);

  for (int h = fhStart; h <= fhEnd; h += 3) {
    servoLR.write(h); delay(50);
    for (int v = fvStart; v <= fvEnd; v += 2) {
      servoUD.write(v); delay(30);
      float lux = readLux();
      float voltage = readVoltage();
      float score = calculateScore(lux, voltage);
      if (score > localMaxScore) {
        localMaxScore = score;
        localMaxLR = h; localMaxUD = v;
        localMaxLux = lux; localMaxVoltage = voltage;
      }
    }
  }

  point.position = currentPosition;
  point.maxLux = localMaxLux;
  point.maxVoltage = localMaxVoltage;
  readTempHumidity(point.temperature, point.humidity);
  point.servoLR = localMaxLR;
  point.servoUD = localMaxUD;
  point.score = localMaxScore;
  point.timestamp = millis();

  moveToPosition(localMaxLR, localMaxUD);
}

// --------------------------- PROPORTIONAL / TRACKING ---------------------------
void proportionalControl() {
  int valL = analogRead(PIN_LDR_L);
  int valR = analogRead(PIN_LDR_R);
  int valU = analogRead(PIN_LDR_U);
  int valD = analogRead(PIN_LDR_D);

  float eroareLR = (float)(valR - valL) / (valL + valR + 1.0);
  float eroareUD = (float)(valD - valU) / (valU + valD + 1.0);

  int tintaLR = holdLR + (int)(eroareLR * 45.0);
  int tintaUD = holdUD + (int)(eroareUD * 22.0);

  tintaLR = constrain(tintaLR, LR_MIN, LR_MAX);
  tintaUD = constrain(tintaUD, UD_MIN, UD_MAX);

  pozitieLR = pozitieLR * (1 - factorNetezire) + tintaLR * factorNetezire;
  pozitieUD = pozitieUD * (1 - factorNetezire) + tintaUD * factorNetezire;

  servoLR.write((int)pozitieLR);
  servoUD.write((int)pozitieUD);
}

void trackSunContinuous() {
  int valL = analogRead(PIN_LDR_L);
  int valR = analogRead(PIN_LDR_R);
  int valU = analogRead(PIN_LDR_U);
  int valD = analogRead(PIN_LDR_D);

  float eroareLR = (float)(valR - valL) / (valL + valR + 1.0);
  float eroareUD = (float)(valD - valU) / (valU + valD + 1.0);

  int tintaLR = (int)pozitieLR + (int)(eroareLR * 45.0);
  int tintaUD = (int)pozitieUD + (int)(eroareUD * 22.0);

  tintaLR = constrain(tintaLR, LR_MIN, LR_MAX);
  tintaUD = constrain(tintaUD, UD_MIN, UD_MAX);

  pozitieLR = pozitieLR * (1 - factorNetezire) + tintaLR * factorNetezire;
  pozitieUD = pozitieUD * (1 - factorNetezire) + tintaUD * factorNetezire;

  servoLR.write((int)pozitieLR);
  servoUD.write((int)pozitieUD);
}

// --------------------------- FIND BEST ---------------------------
int findBestScanPoint() {
  float maxS = cornerScans[0].score;
  int bestIndex = 0;
  for (int i = 1; i < 4; i++) {
    if (cornerScans[i].score > maxS) {
      maxS = cornerScans[i].score;
      bestIndex = i;
    }
  }
  for (int i = 0; i < zigzagScanCount; i++) {
    if (zigzagScans[i].score > maxS) {
      maxS = zigzagScans[i].score;
      bestIndex = 4 + i;
    }
  }
  return bestIndex;
}

// --------------------------- ZIGZAG & RESCAN ---------------------------
void performZigzagScan() {
  lcd.clear();
  lcd.print("Zigzag scanning...");
  Serial.println(F("Starting zigzag generation + scan"));

  zigzagScanCount = 0;

  if (!workArea.isSet) {
    Serial.println(F("WorkArea not set, skipping zigzag"));
    currentState = STATE_MOVING_TO_OPTIMAL;
    return;
  }

  const int desiredPoints = MAX_ZIGZAG_POINTS; 

  for (int k = 1; k <= desiredPoints && zigzagScanCount < MAX_ZIGZAG_POINTS; k++) {
    float t = (float)k / (desiredPoints + 1);
    GPSCoord pA; 
    pA.latitude  = workArea.corners[0].latitude + t * (workArea.corners[2].latitude - workArea.corners[0].latitude);
    pA.longitude = workArea.corners[0].longitude + t * (workArea.corners[2].longitude - workArea.corners[0].longitude);
    pA.isValid = true;

    GPSCoord pB; 
    pB.latitude  = workArea.corners[1].latitude + t * (workArea.corners[3].latitude - workArea.corners[1].latitude);
    pB.longitude = workArea.corners[1].longitude + t * (workArea.corners[3].longitude - workArea.corners[1].longitude);
    pB.isValid = true;

    if (k % 2 == 1) {
      zigzagScans[zigzagScanCount].position = pA;
      moveToCoordinate(pA);
      performFullScan(zigzagScans[zigzagScanCount]);
      Serial.print(F("Zigzag point "));
      Serial.print(zigzagScanCount);
      Serial.print(F(" score="));
      Serial.println(zigzagScans[zigzagScanCount].score);
      zigzagScanCount = min(zigzagScanCount + 1, (int)MAX_ZIGZAG_POINTS);

      if (zigzagScanCount < MAX_ZIGZAG_POINTS) {
        zigzagScans[zigzagScanCount].position = pB;
        moveToCoordinate(pB);
        performFullScan(zigzagScans[zigzagScanCount]);
        Serial.print(F("Zigzag point "));
        Serial.print(zigzagScanCount);
        Serial.print(F(" score="));
        Serial.println(zigzagScans[zigzagScanCount].score);
        zigzagScanCount = min(zigzagScanCount + 1, (int)MAX_ZIGZAG_POINTS);
      }
    } else {
      zigzagScans[zigzagScanCount].position = pB;
      moveToCoordinate(pB);
      performFullScan(zigzagScans[zigzagScanCount]);
      Serial.print(F("Zigzag point "));
      Serial.print(zigzagScanCount);
      Serial.print(F(" score="));
      Serial.println(zigzagScans[zigzagScanCount].score);
      zigzagScanCount = min(zigzagScanCount + 1, (int)MAX_ZIGZAG_POINTS);

      if (zigzagScanCount < MAX_ZIGZAG_POINTS) {
        zigzagScans[zigzagScanCount].position = pA;
        moveToCoordinate(pA);
        performFullScan(zigzagScans[zigzagScanCount]);
        Serial.print(F("Zigzag point "));
        Serial.print(zigzagScanCount);
        Serial.print(F(" score="));
        Serial.println(zigzagScans[zigzagScanCount].score);
        zigzagScanCount = min(zigzagScanCount + 1, (int)MAX_ZIGZAG_POINTS);
      }
    }
    delay(150);
  }

  Serial.print(F("Zigzag complete. Total points: "));
  Serial.println(zigzagScanCount);
  currentState = STATE_MOVING_TO_OPTIMAL;
}

void rescanOthersAndReturn(int excludeIndex) {
  Serial.print(F("Rescanning others, excluding index "));
  Serial.println(excludeIndex);

  for (int i = 0; i < 4; i++) {
    if (!workArea.corners[i].isValid) continue;
    if (i == excludeIndex) continue;
    lcd.clear(); lcd.print("Rescan corner ");
    lcd.print(i);
    moveToCoordinate(workArea.corners[i]);
    ScanPoint newScan;
    performFullScan(newScan);
    rotate180(); delay(200);
    ScanPoint rotated;
    performFullScan(rotated);
    if (rotated.score > newScan.score) newScan = rotated;
    if (newScan.score > cornerScans[i].score) {
      Serial.print(F("Corner "));
      Serial.print(i);
      Serial.print(F(" improved from "));
      Serial.print(cornerScans[i].score);
      Serial.print(F(" to "));
      Serial.println(newScan.score);
      cornerScans[i] = newScan;
    } else {
      Serial.print(F("Corner "));
      Serial.print(i);
      Serial.print(F(" unchanged (best "));
      Serial.print(cornerScans[i].score);
      Serial.println(F(")"));
    }
  }

  for (int j = 0; j < zigzagScanCount; j++) {
    int globalIdx = 4 + j;
    if (globalIdx == excludeIndex) continue;
    lcd.clear(); lcd.print("Rescan zigzag ");
    lcd.print(j);
    moveToCoordinate(zigzagScans[j].position);
    ScanPoint newScan;
    performFullScan(newScan);
    if (newScan.score > zigzagScans[j].score) {
      Serial.print(F("Zigzag "));
      Serial.print(j);
      Serial.print(F(" improved from "));
      Serial.print(zigzagScans[j].score);
      Serial.print(F(" to "));
      Serial.println(newScan.score);
      zigzagScans[j] = newScan;
    } else {
      Serial.print(F("Zigzag "));
      Serial.print(j);
      Serial.print(F(" unchanged (best "));
      Serial.print(zigzagScans[j].score);
      Serial.println(F(")"));
    }
  }

  int best = findBestScanPoint();
  Serial.print(F("After rescans, best index = "));
  Serial.println(best);

  if (best < 4) {
    moveToCoordinate(cornerScans[best].position);
    moveToPosition(cornerScans[best].servoLR, cornerScans[best].servoUD);
  } else {
    int idx = best - 4;
    moveToCoordinate(zigzagScans[idx].position);
    moveToPosition(zigzagScans[idx].servoLR, zigzagScans[idx].servoUD);
  }

  currentState = STATE_TRACKING_SUN;
  lastCheckTime = millis();
}

void rescanAllPoints() {
  for (int i = 0; i < 4; i++) {
    lcd.clear();
    lcd.print("Rescan corner ");
    lcd.print(i);
    moveToCoordinate(workArea.corners[i]);
    ScanPoint newScan;
    performFullScan(newScan);
    rotate180(); delay(300);
    ScanPoint rotatedScan;
    performFullScan(rotatedScan);
    if (rotatedScan.score > newScan.score) cornerScans[i] = rotatedScan;
    else cornerScans[i] = newScan;
    rotate180(); delay(200);
  }
  for (int i = 0; i < zigzagScanCount; i++) {
    lcd.clear();
    lcd.print("Rescan zigzag ");
    lcd.print(i);
    moveToCoordinate(zigzagScans[i].position);
    ScanPoint newScan;
    performFullScan(newScan);
    if (newScan.score > zigzagScans[i].score * 0.95) zigzagScans[i] = newScan;
  }
  Serial.println(F("Rescan done."));
}

// --------------------------- GPS / MOVEMENT ---------------------------
void updateGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }
  if (gps.location.isValid()) {
    currentPosition.latitude = gps.location.lat();
    currentPosition.longitude = gps.location.lng();
    currentPosition.isValid = true;
  }
}

void moveToCoordinate(const GPSCoord& target) {
  if (!target.isValid || !currentPosition.isValid) {
    Serial.println(F("GPS invalid, skipping move"));
    return;
  }
  unsigned long lastUpdate = millis();
  while (workArea.distanceBetween(currentPosition, target) > GPS_ACCURACY_THRESHOLD) {
    if (checkObstacle()) {
      stopMotors();
      avoidObstacle();
      continue;
    }
    float bearing = atan2(target.longitude - currentPosition.longitude, target.latitude - currentPosition.latitude) * 180.0 / PI;
    if (bearing < 0) bearing += 360.0;
    float headingDiff = bearing - gyroData.heading;
    if (headingDiff > 180.0) headingDiff -= 360.0;
    if (headingDiff < -180.0) headingDiff += 360.0;
    if (abs(headingDiff) > 15.0) {
      if (headingDiff > 0) {
        sendMotorCommand(1, 0xBA, MOTOR_ROTATION_SPEED);
        sendMotorCommand(2, 0xF0, MOTOR_ROTATION_SPEED);
      } else {
        sendMotorCommand(1, 0xF0, MOTOR_ROTATION_SPEED);
        sendMotorCommand(2, 0xBA, MOTOR_ROTATION_SPEED);
      }
      delay(100);
    } else {
      sendMotorCommand(1, 0xF0, MOTOR_FORWARD_SPEED);
      sendMotorCommand(2, 0xF0, MOTOR_FORWARD_SPEED);
    }
    delay(150);
    updateGPS();
    updateHeading(0.15f);
    if (millis() - lastUpdate > 600000UL) break;
  }
  stopMotors();
}

// --------------------------- PARSE SERIAL COORDINATES ---------------------------
void parseSerialCoordinates() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    int commaIndex = line.indexOf(',');
    String latStr, lonStr;
    if (commaIndex >= 0) {
      latStr = line.substring(0, commaIndex);
      lonStr = line.substring(commaIndex + 1);
    } else {
      int spaceIndex = line.indexOf(' ');
      if (spaceIndex >= 0) {
        latStr = line.substring(0, spaceIndex);
        lonStr = line.substring(spaceIndex + 1);
      } else {
        Serial.println(F("Invalid format. Use LAT,LON"));
        return;
      }
    }
    latStr.trim(); lonStr.trim();
    latStr.replace(',', '.'); lonStr.replace(',', '.');
    char latBuf[32], lonBuf[32];
    latStr.toCharArray(latBuf, sizeof(latBuf));
    lonStr.toCharArray(lonBuf, sizeof(lonBuf));
    double lat = atof(latBuf);
    double lon = atof(lonBuf);
    if (lat == 0.0 && latStr.charAt(0) != '0') {
      Serial.println(F("Latitude parse failed"));
      return;
    }
    if (lon == 0.0 && lonStr.charAt(0) != '0') {
      Serial.println(F("Longitude parse failed"));
      return;
    }
    if (coordCountGlobal < 4) {
      workArea.corners[coordCountGlobal].latitude = lat;
      workArea.corners[coordCountGlobal].longitude = lon;
      workArea.corners[coordCountGlobal].isValid = true;
      coordCountGlobal++;
      Serial.print(F("Coord received: "));
      Serial.print(lat, 6); Serial.print(F(", ")); Serial.println(lon, 6);
      lcd.setCursor(0,1);
      lcd.print("Coords: ");
      lcd.print(coordCountGlobal);
      lcd.print("/4    ");
      if (coordCountGlobal >= 4) {
        workArea.isSet = true;
        currentState = STATE_MOVING_TO_CORNER;
        currentCornerIndex = 0;
        Serial.println(F("All coords received. Starting mission."));
        lcd.clear(); lcd.print("Starting mission");
        delay(400);
      }
    } else {
      Serial.println(F("Already have 4 coordinates"));
    }
  }
}

// --------------------------- DISPLAY STATUS ---------------------------
void displayStatus() {
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate < 500) return;
  lastDisplayUpdate = millis();

  if (solarTrackingEnabled) {
    float lux = readLux();
    float voltage = readVoltage();
    char luxStr[10], vStr[10];
    dtostrf(lux, 6, 0, luxStr);
    dtostrf(voltage, 5, 1, vStr);
    lcd.setCursor(0,0);
    lcd.print("Lux:");
    lcd.print(luxStr);
    lcd.print(" V:");
    lcd.print(vStr);

    int coordCount = 0;
    for (int i = 0; i < 4; i++) if (workArea.corners[i].isValid) coordCount++;

    if (abs(pozitieLR - prevPozitieLR) >= 1.0 || abs(pozitieUD - prevPozitieUD) >= 1.0 || coordCountChanged) {
      prevPozitieLR = pozitieLR;
      prevPozitieUD = pozitieUD;
      static int prevCoordCount = -1;
      if (prevCoordCount != coordCount) {
        prevCoordCount = coordCount;
        coordCountChanged = true;
      } else coordCountChanged = false;
      char displayStr[17];
      snprintf(displayStr, sizeof(displayStr), "LR:%d UD:%d %s", (int)pozitieLR, (int)pozitieUD, (isScanning ? "SCAN" : "HOLD"));
      displayStr[16] = '\0';
      lcd.setCursor(0,1);
      lcd.print(displayStr);
    }
  } else {
    lcd.setCursor(0,0);
    lcd.print("Waiting coords  ");
    lcd.setCursor(0,1);
    lcd.print("Send 4 LAT,LON  ");
  }
}

// --------------------------- ULTRASONIC / AVOIDANCE ---------------------------
float readUltrasonicDistance() {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 30000);
  if (duration == 0) return 999.0;
  float distance = duration * 0.034 / 2.0;
  return distance;
}

bool checkObstacle() {
  float distance = readUltrasonicDistance();
  if (distance < OBSTACLE_DISTANCE_CM) {
    Serial.print(F("Obstacle at "));
    Serial.print(distance);
    Serial.println(F(" cm"));
    return true;
  }
  return false;
}

void avoidObstacle() {
  Serial.println(F("Avoiding obstacle..."));
  stopMotors();
  delay(200);
  GPSCoord positionBeforeAvoidance = currentPosition;
  float targetHeadingBeforeAvoidance = gyroData.heading;
  lcd.clear(); lcd.print("Avoiding...");
  sendMotorCommand(1, 0xF0, MOTOR_ROTATION_SPEED);
  sendMotorCommand(2, 0xBA, MOTOR_ROTATION_SPEED);
  delay(AVOIDANCE_TURN_TIME_MS / 2);
  stopMotors();
  delay(150);
  sendMotorCommand(1, 0xF0, MOTOR_FORWARD_SPEED);
  sendMotorCommand(2, 0xF0, MOTOR_FORWARD_SPEED);
  delay(AVOIDANCE_FORWARD_TIME_MS);
  stopMotors();
  sendMotorCommand(1, 0xBA, MOTOR_ROTATION_SPEED);
  sendMotorCommand(2, 0xF0, MOTOR_ROTATION_SPEED);
  delay(AVOIDANCE_TURN_TIME_MS / 2);
  stopMotors();
  delay(200);
  gyroData.heading = targetHeadingBeforeAvoidance;
  Serial.println(F("Avoidance done"));
}

// --------------------------- UTILS ---------------------------
void readTempHumidity(float &temp, float &hum) {
  int res = dhtSensor.read(PIN_DHT);
  if (res == 0) {
    temp = (float)dhtSensor.temperature;
    hum = (float)dhtSensor.humidity;
  } else {
    temp = 25.0; hum = 50.0;
  }
}

bool isRobotMoving() {
  if (!gps.speed.isValid()) return false;
  float speedKmh = gps.speed.kmph();
  return speedKmh > GPS_SPEED_THRESHOLD_KMH;
}

bool canScan() {
  if (isRobotMoving()) {
    Serial.println(F("Cannot scan: robot moving"));
    lcd.setCursor(0,1); lcd.print("Moving...       ");
    return false;
  }
  if (checkObstacle()) {
    Serial.println(F("Cannot scan: obstacle"));
    return false;
  }
  return true;
}

void moveToPosition(int lr, int ud) {
  lr = constrain(lr, LR_MIN, LR_MAX);
  ud = constrain(ud, UD_MIN, UD_MAX);
  servoLR.write(lr);
  servoUD.write(ud);
  pozitieLR = lr; pozitieUD = ud;
  holdLR = lr; holdUD = ud;
}