#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Sensor Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>

// ================= Pin config =================
#define BUTTON_PIN   4
#define BUTTON2_PIN  19    // reported over serial only, does NOT touch the LEDs
#define LED1_PIN     17
#define LED2_PIN     0     // strapping pin, fine once booted
#define BUZZER_PIN   18    // passive buzzer, driven via standard tone()

// ================= I2C addresses =================
#define TCA9548A_ADDR   0x71   // A0 tied high -> 0x71
#define PCA9685_ADDR    0x40   // main bus, not muxed
#define ADS1115_ADDR    0x48

const uint8_t VL_CHANNELS[4] = {7, 6, 5, 4};
const uint8_t TCS_CHANNEL    = 3;

// ================= Sensor / Driver Instances =================
Adafruit_MPU6050       mpu;
Adafruit_ADS1115       ads;
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);
Adafruit_VL53L0X       vl53[4];
Adafruit_TCS34725      tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X);

bool vl53_ready[4] = {false, false, false, false};

// ================= Servo config =================
#define SERVO_MIN_US  900.0
#define SERVO_MAX_US  2100.0
#define SERVO_COUNT   16

struct ServoState {
  float current;   // deg
  float target;    // deg
  float speed;     // deg/sec, used only while sweeping
  bool  sweeping;
};
ServoState servos[SERVO_COUNT];

// ================= LED state =================
bool ledState1 = false;
bool ledState2 = false;

// ================= Schedule periods (ms) =================
const uint32_t LIDAR_PERIOD  = 28;   // ~35.5 Hz
const uint32_t COLOR_PERIOD  = 50;   // ~20 Hz
const uint32_t ADC_PERIOD    = 17;   // ~58.8 Hz
const uint32_t BUTTON_PERIOD = 20;   // 50 Hz
const uint32_t IMU_PERIOD    = 10;   // 100 Hz
const uint32_t SERVO_PERIOD  = 20;   // 50 Hz

uint32_t tLidar = 0, tColor = 0, tAdc = 0, tButton = 0, tImu = 0, tServo = 0;

// ================= Multiplexer Helper =================
void muxSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ================= MPU6050 Data =================
float roll = 0, pitch = 0, yaw = 0;
uint32_t lastMPUus = 0;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;

float lastGX = 0, lastGY = 0, lastGZ = 0;
float lastAX = 0, lastAY = 0, lastAZ = 0;

void mpu6050Calibrate() {
  const int N = 200;
  double sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
  sensors_event_t a, g, temp;

  for (int i = 0; i < N; i++) {
    mpu.getEvent(&a, &g, &temp);
    sax += a.acceleration.x / SENSORS_GRAVITY_STANDARD;
    say += a.acceleration.y / SENSORS_GRAVITY_STANDARD;
    saz += a.acceleration.z / SENSORS_GRAVITY_STANDARD;
    sgx += g.gyro.x * 180.0 / PI;
    sgy += g.gyro.y * 180.0 / PI;
    sgz += g.gyro.z * 180.0 / PI;
  }
  accelBiasX = sax / N; accelBiasY = say / N; accelBiasZ = saz / N;
  gyroBiasX  = sgx / N; gyroBiasY  = sgy / N; gyroBiasZ  = sgz / N;
}

void mpu6050Update() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float axg  = (a.acceleration.x / SENSORS_GRAVITY_STANDARD) - accelBiasX;
  float ayg  = (a.acceleration.y / SENSORS_GRAVITY_STANDARD) - accelBiasY;
  float azg  = (a.acceleration.z / SENSORS_GRAVITY_STANDARD) - accelBiasZ;

  float gxds = (g.gyro.x * 180.0 / PI) - gyroBiasX;
  float gyds = (g.gyro.y * 180.0 / PI) - gyroBiasY;
  float gzds = (g.gyro.z * 180.0 / PI) - gyroBiasZ;

  lastAX = axg; lastAY = ayg; lastAZ = azg;
  lastGX = gxds; lastGY = gyds; lastGZ = gzds;

  float accelRoll  = atan2(ayg, azg + 1.0) * 180.0 / PI;
  float accelPitch = atan2(-axg, sqrt(ayg * ayg + (azg + 1.0) * (azg + 1.0))) * 180.0 / PI;

  uint32_t now = micros();
  float dt = (lastMPUus == 0) ? 0.01 : (now - lastMPUus) / 1000000.0;
  lastMPUus = now;

  roll  = 0.98 * (roll  + gxds * dt) + 0.02 * accelRoll;
  pitch = 0.98 * (pitch + gyds * dt) + 0.02 * accelPitch;
  yaw  += gzds * dt;
}

// ================= Buttons =================
bool lastButtonState = HIGH;
bool lastButton2State = HIGH;

void buttonPoll() {
  static uint32_t lastDebounce = 0;
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState && millis() - lastDebounce > 50) {
    lastDebounce = millis();
    lastButtonState = reading;
  }

  static uint32_t lastDebounce2 = 0;
  bool reading2 = digitalRead(BUTTON2_PIN);
  if (reading2 != lastButton2State && millis() - lastDebounce2 > 50) {
    lastDebounce2 = millis();
    lastButton2State = reading2;
  }
}

// ================= ADS1115 (PGA: +-4.096V) =================
float adsScaled(uint8_t channel) {
  int16_t raw = ads.readADC_SingleEnded(channel);
  // Full scale range is +-4.096V (32767 count = 4.096V)
  // Scaling relative to 3.3V max input voltage:
  float voltage = ads.computeVolts(raw);
  float pct = (voltage / 3.3f) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}

// ================= Servo Control =================
void servoWriteDeg(uint8_t channel, float deg) {
  deg = constrain(deg, 0.0, 180.0);
  if(channel==3){
    pca.setPWM(channel, 0, map(deg,0,180,150,570));
  } else {
    pca.setPWM(channel, 0, map(deg,0,180,140,510));
  }
}

void servosUpdate(float dt) {
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    if (!servos[ch].sweeping) continue;
    float maxStep = servos[ch].speed * dt;
    float diff = servos[ch].target - servos[ch].current;
    if (fabs(diff) <= maxStep) {
      servos[ch].current = servos[ch].target;
      servos[ch].sweeping = false;
    } else {
      servos[ch].current += (diff > 0 ? maxStep : -maxStep);
    }
    servoWriteDeg(ch, servos[ch].current);
  }
}

// ================= Serial Command Parsing =================
char serialBuf[64];
uint8_t serialIdx = 0;

void processCommand(char *line) {
  char *tok = strtok(line, ",");
  if (!tok) return;

  if (strcmp(tok, "$SERVO") == 0) {
    char *chStr     = strtok(NULL, ",");
    char *targetStr = strtok(NULL, ",");
    char *sweepStr  = strtok(NULL, ",");
    char *speedStr  = strtok(NULL, ",");
    if (!chStr || !targetStr || !sweepStr || !speedStr) return;

    int ch = atoi(chStr);
    if (ch < 0 || ch >= SERVO_COUNT) return;
    float target = constrain((float)atof(targetStr), 0.0, 180.0);
    bool sweep = (strcmp(sweepStr, "true") == 0);
    float speed = atof(speedStr);

    if (sweep) {
      servos[ch].target = target;
      servos[ch].speed = (speed > 0) ? speed : 60.0;
      servos[ch].sweeping = true;
    } else {
      servos[ch].current = target;
      servos[ch].target = target;
      servos[ch].sweeping = false;
      servoWriteDeg(ch, target);
    }
  } else if (strcmp(tok, "$LED") == 0) {
    char *l1 = strtok(NULL, ",");
    char *l2 = strtok(NULL, ",");
    if (!l1 || !l2) return;
    ledState1 = atoi(l1) ? true : false;
    ledState2 = atoi(l2) ? true : false;
    digitalWrite(LED1_PIN, ledState1);
    digitalWrite(LED2_PIN, ledState2);
  } else if (strcmp(tok, "$BEEP") == 0) {
    char *freqStr = strtok(NULL, ",");
    char *durStr  = strtok(NULL, ",");
    if (!freqStr || !durStr) return;
    
    float freq = atof(freqStr);
    uint32_t dur = (uint32_t)atol(durStr);
    if (freq <= 0 || dur == 0) return;
    
    tone(BUZZER_PIN, (unsigned int)freq, dur);
  }
}

void serialPoll() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        processCommand(serialBuf);
        serialIdx = 0;
      }
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = c;
    }
  }
}

// ================= Setup & Loop =================
void setup() {
  Serial.begin(921600);
  Wire.begin();

  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  noTone(BUZZER_PIN);

  // MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu6050Calibrate();
  }

  // ADS1115 -> Set gain to GAIN_ONE (+-4.096V range)
  if (ads.begin(ADS1115_ADDR)) {
    ads.setGain(GAIN_ONE);
  }

  // VL53L0X
  for (uint8_t i = 0; i < 4; i++) {
    muxSelect(VL_CHANNELS[i]);
    if (vl53[i].begin()) {
      vl53[i].startRangeContinuous();
      vl53_ready[i] = true;
    }
  }

  // TCS34725
  muxSelect(TCS_CHANNEL);
  tcs.begin();

  // PCA9685 Servos
  pca.begin();
  pca.setPWMFreq(50.0);
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    servos[ch].current = 90;
    servos[ch].target = 90;
    servos[ch].speed = 60;
    servos[ch].sweeping = false;
    servoWriteDeg(ch, 90); // Set to active 90 deg position on boot
  }
}

void loop() {
  serialPoll();

  uint32_t now = millis();

  // IMU Loop
  if (now - tImu >= IMU_PERIOD) {
    tImu = now;
    mpu6050Update();
    Serial.print("$IMU,");
    Serial.print(lastGX, 2); Serial.print(",");
    Serial.print(lastGY, 2); Serial.print(",");
    Serial.print(lastGZ, 2); Serial.print(",");
    Serial.print(lastAX, 3); Serial.print(",");
    Serial.print(lastAY, 3); Serial.print(",");
    Serial.print(lastAZ, 3); Serial.print(",");
    Serial.print(roll, 1);   Serial.print(",");
    Serial.print(pitch, 1);  Serial.print(",");
    Serial.println(yaw, 1);
  }

  // Servo Loop
  if (now - tServo >= SERVO_PERIOD) {
    float dt = (now - tServo) / 1000.0;
    tServo = now;
    servosUpdate(dt);
  }

  // Button Loop
  if (now - tButton >= BUTTON_PERIOD) {
    tButton = now;
    buttonPoll();
    Serial.print("$BUTTON,");
    Serial.print(digitalRead(BUTTON_PIN));
    Serial.print(",");
    Serial.println(digitalRead(BUTTON2_PIN));
  }

  // ADC Loop
  if (now - tAdc >= ADC_PERIOD) {
    tAdc = now;
    Serial.print("$ADC,");
    Serial.print(adsScaled(0), 1); Serial.print(",");
    Serial.print(adsScaled(1), 1); Serial.print(",");
    Serial.print(adsScaled(2), 1); Serial.print(",");
    Serial.println(adsScaled(3), 1);
  }

  // Distance (VL53L0X) Loop
  if (now - tLidar >= LIDAR_PERIOD) {
    tLidar = now;
    uint16_t d[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < 4; i++) {
      if (vl53_ready[i]) {
        muxSelect(VL_CHANNELS[i]);
        if (vl53[i].isRangeComplete()) {
          d[i] = vl53[i].readRangeResult();
        }
      }
    }
    Serial.print("$DIST,");
    Serial.print(d[0]); Serial.print(",");
    Serial.print(d[1]); Serial.print(",");
    Serial.print(d[2]); Serial.print(",");
    Serial.println(d[3]);
  }

  // Color (TCS34725) Loop
  if (now - tColor >= COLOR_PERIOD) {
    tColor = now;
    muxSelect(TCS_CHANNEL);
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    uint16_t lux = tcs.calculateLux(r, g, b);
    float rPct = (c > 0) ? constrain(r * 100.0f / c, 0, 100) : 0;
    float gPct = (c > 0) ? constrain(g * 100.0f / c, 0, 100) : 0;
    float bPct = (c > 0) ? constrain(b * 100.0f / c, 0, 100) : 0;
    Serial.print("$COLOR,");
    Serial.print(lux);      Serial.print(",");
    Serial.print(rPct, 1); Serial.print(",");
    Serial.print(gPct, 1); Serial.print(",");
    Serial.println(bPct, 1);
  }
}